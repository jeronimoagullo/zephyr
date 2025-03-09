/*
 * Copyright (c) Jeronimo Agullo Ocampos <jeronimoagullo97@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_lcd_cam_display

#include <periph_ctrl.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/display.h>

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/esp32_clock_control.h>

#include <hal/lcd_hal.h>
#include <hal/lcd_ll.h>

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>

#include <zephyr/multi_heap/shared_multi_heap.h>
#include <soc/soc_memory_layout.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_esp32lcd, CONFIG_DISPLAY_LOG_LEVEL);

#define DISPLAY_ESP32_DMA_BUFFER_MAX_SIZE 4095

struct esp32lcd_data {
	enum display_pixel_format current_pixel_format;
	uint8_t current_pixel_size;
	uint8_t *frame_buffer;
	uint32_t frame_buffer_len;
	const uint8_t *pend_buf;
	const uint8_t *front_buf;
	struct k_sem sem;

	lcd_hal_context_t hal;

	uint8_t hsync_len;
	uint8_t hfront_porch;
	uint8_t hback_porch;
	uint8_t vsync_len;
	uint8_t vfront_porch;
	uint8_t vback_porch;
	uint8_t hsync_active;
	uint8_t vsync_active;

	/* DMA configuration structures */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blocks[CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM];

	/* Semaphore to block until DMA transfer completes */
	struct k_sem dma_sem;
};

struct esp32lcd_config {
	uint32_t width;
	uint32_t height;
	uint32_t clock_frequency;
	struct gpio_dt_spec bl_ctrl_gpio;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	//struct stm32_pclken pclken;
	//const struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pctrl;
	//void (*irq_config_func)(const struct device *dev);
	//const struct device *display_controller;

	uint8_t tx_dma_channel;
	const struct device *dma_dev;

};

static int esp32lcd_write(const struct device *dev, const uint16_t x,
				const uint16_t y,
				const struct display_buffer_descriptor *desc,
				const void *buf)
{
	LOG_INF("esp32lcd_write");
	const struct esp32lcd_config *config = dev->config;
	struct esp32lcd_data *data = dev->data;
	struct dma_status dma_status = {0};
	int ret = 0;

	/* Validate inputs */
	if (!buf || !desc || x + desc->width > config->width || y + desc->height > config->height) {
		LOG_ERR("Invalid input");
		return -EINVAL;
	}

	/* Wait for previous DMA transfer to complete */
	ret = k_sem_take(&data->dma_sem, K_MSEC(1000));

	ret = dma_get_status(config->dma_dev, config->tx_dma_channel, &dma_status);

	if(ret != 0){
		LOG_ERR("DMA failed to get status: %d", ret);
		return ret;
	}

	if (dma_status.busy) {
		LOG_ERR("Tx DMA Channel %d is busy", config->tx_dma_channel);
		//return -EBUSY;
	}

	/* Copy data to framebuffer */
	uint32_t offset = (y * config->width + x) * data->current_pixel_size;
	for (uint16_t row = 0; row < desc->height; row++) {
		const uint8_t *src = (const uint8_t *)buf + (row * desc->pitch * data->current_pixel_size);
		uint8_t *dst = data->frame_buffer + offset + (row * config->width * data->current_pixel_size);
		memcpy(dst, src, desc->width * data->current_pixel_size);
	}

	/* Trigger DMA transfer */
	ret = dma_start(config->dma_dev, config->tx_dma_channel);

	if(ret != 0){
		LOG_ERR("DMA failed to start: %d", ret);
		/* Release semaphore to avoid deadlock */
		k_sem_give(&data->dma_sem);
		return ret;
	}

	lcd_cam_dev_t *lcd_hw = LCD_LL_GET_HW(0);
	lcd_ll_start(lcd_hw);

	return 0;
}

static void esp32lcd_get_capabilities(const struct device *dev,
				struct display_capabilities *capabilities)
{
	LOG_INF("get capabitilies");
	struct esp32lcd_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->x_resolution = 10;
	capabilities->y_resolution = 10;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	capabilities->screen_info = 0;
	capabilities->current_pixel_format = data->current_pixel_format;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;

	LOG_INF("current pixel format: %d", data->current_pixel_format);
}

static int esp32lcd_set_pixel_format(const struct device *dev, enum display_pixel_format format) {
    struct esp32lcd_data *data = dev->data;

    if (format != PIXEL_FORMAT_RGB_565) {
        return -ENOTSUP;
    }

    data->current_pixel_format = format;
    data->current_pixel_size = 2;  // RGB565: 2 bytes per pixel
    return 0;
}

static int esp32lcd_display_blanking_off(const struct device *dev)
{
	const struct esp32lcd_config *config = dev->config;

	return gpio_pin_set_dt(&config->bl_ctrl_gpio, 1);
}

static int esp32lcd_display_blanking_on(const struct device *dev)
{
	const struct esp32lcd_config *config = dev->config;

	return gpio_pin_set_dt(&config->bl_ctrl_gpio, 0);
}

static int lcd_esp32_init_peripheral(const struct device *dev)
{
	// Best resource: https://github.com/vroland/epdiy/blob/main/src/output_lcd/lcd_driver.c#L121
	// Best from espressif: https://github.com/espressif/esp-idf/blob/master/components/esp_lcd/rgb/esp_lcd_panel_rgb.c
	// read about registers: https://www.esp32.com/viewtopic.php?t=24998
	// great topic: https://www.esp32.com/viewtopic.php?f=5&t=24459&p=100920&hilit=LCD#p100920

	const struct esp32lcd_config *cfg = dev->config;
	struct esp32lcd_data *data = dev->data;
	lcd_cam_dev_t *lcd_hw = LCD_LL_GET_HW(0);
	int ret = 0;

	/* Enable peripheral clock */
	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock device not ready");
		return -ENODEV;
	}

	/* Enable peripheral LCD_CAM */
	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
    		LOG_ERR("Failed to enable clock: %d\n", ret);
		return ret;
	}

	/* Initialize LCD_CAM peripheral */
	lcd_hal_init(&data->hal, 0);

	lcd_ll_reset(lcd_hw);
	lcd_ll_fifo_reset(lcd_hw);

	// Enable LCD peripheral clock
	lcd_ll_enable_clock(lcd_hw, true);

	// pixel clock phase and polarity
	lcd_ll_set_clock_idle_level(lcd_hw, false); // PCLK low idle
	lcd_ll_set_pixel_clock_edge(lcd_hw, false); // PCLK low in 1st half cycle
	lcd_ll_set_pixel_clock_prescale(lcd_hw, 1); // PCLK = CLK / (CLKCNT_N+1)

	/* Enable LCD master clock output */
	if (!cfg->clock_frequency) {
		LOG_ERR("No clock_frequency specified\n");
		return -EINVAL;
	}

	if (ESP32_CLK_CPU_PLL_240M % cfg->clock_frequency) {
		LOG_ERR("Invalid clock_frequency value. It must be a divisor of 160M\n");
		return -EINVAL;
	}

	lcd_ll_select_clk_src(lcd_hw, LCD_CLK_SRC_PLL240M);
	int div_num = ESP32_CLK_CPU_PLL_240M / cfg->clock_frequency;
	LOG_INF("div num: %d", div_num);
	lcd_ll_set_group_clock_coeff(lcd_hw, div_num, 0, 0);

	/* Enable RGB mode and set data width to 16 bits (RGB565) */
    	lcd_ll_enable_rgb_mode(lcd_hw, true);		// Enable RGB interface
	lcd_ll_set_data_width(lcd_hw, 16);       	// 16-bit data mode
	lcd_ll_swap_byte_order(lcd_hw, 8, false); // Do not swap bytes
	lcd_ll_reverse_bit_order(lcd_hw, false);      // Do not reverse bit order

	// Configure timing signals
	lcd_ll_set_idle_level(lcd_hw, true, false, false);

	lcd_ll_enable_rgb_yuv_convert(lcd_hw, false); // Disable RGB/YUV converter
	lcd_ll_enable_auto_next_frame(lcd_hw, false);  // Do NOT auto-frame
	lcd_ll_set_data_delay_ticks(lcd_hw, false);      // No data delays
	lcd_ll_enable_output_always_on(lcd_hw, true);  // Enable 'always out' mode
	lcd_ll_set_phase_cycles(lcd_hw, 1, 1 , 20); // Dummy phase(s) @ LCD start 1 dummy phase  No command at LCD start*/

	// Enable Interrupts
	lcd_ll_enable_interrupt(lcd_hw, LCD_LL_EVENT_VSYNC_END, true);
	lcd_ll_enable_interrupt(lcd_hw, LCD_LL_EVENT_TRANS_DONE, true);
    	lcd_ll_clear_interrupt_status(lcd_hw, UINT32_MAX); // clear pending interrupt

	// LCD start
	//lcd_ll_start(lcd_hw);
	//lcd_ll_set_data_width(lcd_hw, 16);

	//TODO
	//lcd_ll_set_phase_cycles(lcd_hw, 0, (lcd.dummy_bytes > 0), 1);  // enable data phase only
	//lcd_ll_enable_output_hsync_in_porch_region(lcd_hw, false);     // enable data phase only

	// number of data cycles is controlled by DMA buffer size
	//lcd_ll_enable_output_always_on(lcd_hw, false);
	

	// configure blank region timing
	// RGB panel always has a front and back blank (porch region)
	//lcd_ll_set_blank_cycles(lcd_hw, data->hback_porch, data->hfront_porch);
	// output hsync even in porch region?
	//lcd_ll_enable_output_hsync_in_porch_region(lcd_hw, false);

	// send next frame automatically in stream mode
	//lcd_ll_enable_auto_next_frame(lcd_hw, false);

	// enable intr
	//esp_intr_enable(lcd.vsync_intr);
	//esp_intr_enable(lcd.done_intr);


	//lcd_ll_stop(lcd_hw);

	LOG_INF("LCD peripheral configured successfully");

	return 0;
}



void esp32lcd_esp32_dma_tx_done(const struct device *dev, void *user_data, uint32_t channel,
			     int status)
{

}



static int esp32lcd_display_dma_config(const struct device *dev)
{
	const struct esp32lcd_config *config = dev->config;
	struct esp32lcd_data *data = dev->data;
	struct dma_block_config *dma_block_iter = data->dma_blocks;
	uint32_t buffer_size = 0;

	if (!device_is_ready(config->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	//ret = dma_get_attribute(config->dma_dev, )


	buffer_size = data->frame_buffer_len;
	memset(data->dma_blocks, 0, sizeof(data->dma_blocks));
	for (int i = 0; i < CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM; ++i) {
		dma_block_iter->source_address =
			(uint32_t)data->frame_buffer + (i * DISPLAY_ESP32_DMA_BUFFER_MAX_SIZE);
		if (buffer_size < DISPLAY_ESP32_DMA_BUFFER_MAX_SIZE) {
			dma_block_iter->block_size = buffer_size;
			dma_block_iter->next_block = NULL;
			data->dma_cfg.block_count = i + 1;
			LOG_INF("using %d DMA blocks", data->dma_cfg.block_count);
			break;
		}
		dma_block_iter->block_size = DISPLAY_ESP32_DMA_BUFFER_MAX_SIZE;
		dma_block_iter->next_block = dma_block_iter + 1;
		dma_block_iter++;
		buffer_size -= DISPLAY_ESP32_DMA_BUFFER_MAX_SIZE;
	}

	if (dma_block_iter->next_block) {
		LOG_ERR("Not enough descriptors available. Increase "
			"CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM");
		return -ENOBUFS;
	}

	data->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	data->dma_cfg.user_data = data;
	data->dma_cfg.dma_callback = esp32lcd_esp32_dma_tx_done;
	data->dma_cfg.head_block = &data->dma_blocks[0];
	data->dma_cfg.error_callback_dis = 1;
	data->dma_cfg.dma_slot = ESP_GDMA_TRIG_PERIPH_LCD0;

	//data->dma_cfg.block_count = 1;
	//dma_cfg.complete_callback_en = 1;

	int error = dma_config(config->dma_dev, config->tx_dma_channel, &data->dma_cfg);
	if (error) {
		LOG_ERR("Unable to configure DMA (%d)", error);
		return error;
	}

	LOG_INF("DMA configured successfully");

	return 0;
}

static int esp32lcd_init(const struct device *dev)
{
	int err;
	const struct esp32lcd_config *config = dev->config;
	struct esp32lcd_data *data = dev->data;

	/* Configure and set display backlight control GPIO */
	if (config->bl_ctrl_gpio.port) {
		err = gpio_pin_configure_dt(&config->bl_ctrl_gpio, GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			LOG_ERR("Configuration of display backlight control GPIO failed");
			return err;
		}
	}

	/* Apply pinctrl configuration */
	err = pinctrl_apply_state(config->pctrl, PINCTRL_STATE_DEFAULT);
	if ( err < 0) {
		LOG_ERR("LCD pinctrl setup failed: %d", err);
		return err;
	}

	LOG_INF("display size: %dx%d", config->width, config->height);

	err = lcd_esp32_init_peripheral(dev);

	/* Initialize framebuffer in external RAM */

	data->frame_buffer_len = config->width * config->height * 2;
	LOG_INF("frame buffer length: %d", data->frame_buffer_len);

	data->frame_buffer = shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, data->frame_buffer_len);
	//data->frame_buffer = k_malloc(config->width * config->height * 2);  // For RGB565
	//shared_multi_heap_free(m_ext);

	if (!data->frame_buffer) {
		LOG_ERR("Error allocating framebuffer. Pointer: %p", (void*)data->frame_buffer);
		return -ENOMEM;
	}

	if (!esp_ptr_external_ram(data->frame_buffer)) {
		LOG_ERR("allocated framebuffer is not in External RAM: %p", (void*)data->frame_buffer);
	} else {
		LOG_INF("allocated framebuffer is in External RAM at %p", (void*)data->frame_buffer);
	}

	/* Initialize the DMA semaphore */
	k_sem_init(&data->dma_sem, 1, 1);

	esp32lcd_display_dma_config(dev);

	LOG_INF("esp32lcd_init successed");

	return 0;
}


static const struct display_driver_api esp32lcd_display_api = {
	.write = esp32lcd_write,
	//.read = esp32lcd_read,
	//.get_framebuffer = esp32lcd__get_framebuffer,
	.get_capabilities = esp32lcd_get_capabilities,
	.set_pixel_format = esp32lcd_set_pixel_format,
	//.set_orientation = esp32lcd_set_orientation,
	.blanking_off = esp32lcd_display_blanking_off,
	.blanking_on = esp32lcd_display_blanking_on,
};

PINCTRL_DT_INST_DEFINE(0);

static const struct esp32lcd_config esp32lcd_config = {
	.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.width = DT_INST_PROP(0, width),
	.height = DT_INST_PROP(0, height),
	.bl_ctrl_gpio = COND_CODE_1(DT_INST_NODE_HAS_PROP(0, bl_ctrl_gpios),
			(GPIO_DT_SPEC_INST_GET(0, bl_ctrl_gpios)), ({ 0 })),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, offset),
	.clock_frequency = (DT_PROP(DT_INST_CHILD(0,
			display_timings), clock_frequency)),
	.dma_dev = ESP32_DT_INST_DMA_CTLR(0, tx),
	.tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(0, tx, channel),
};

static struct esp32lcd_data esp32lcd_data = {
	.current_pixel_format = DT_INST_PROP(0, pixel_format),
	.hsync_len = DT_PROP(DT_INST_CHILD(0, display_timings), hsync_len),
	.hfront_porch = DT_PROP(DT_INST_CHILD(0, display_timings), hfront_porch),
	.hback_porch = DT_PROP(DT_INST_CHILD(0, display_timings), hback_porch),
	.vsync_len = DT_PROP(DT_INST_CHILD(0, display_timings), vsync_len),
	.vfront_porch = DT_PROP(DT_INST_CHILD(0, display_timings), vfront_porch),
	.vback_porch = DT_PROP(DT_INST_CHILD(0,	display_timings), vback_porch),
	.hsync_active = (DT_PROP(DT_INST_CHILD(0, display_timings), hsync_active)),
	.vsync_active = (DT_PROP(DT_INST_CHILD(0, display_timings), vsync_active)),
};

DEVICE_DT_INST_DEFINE(0, &esp32lcd_init, NULL, &esp32lcd_data, &esp32lcd_config,
		POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &esp32lcd_display_api);

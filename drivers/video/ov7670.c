/*
 * Copyright (c) 2020, FrankLi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ovti_ov7670
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
<<<<<<< HEAD
#include <zephyr/drivers/pwm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ov7670, LOG_LEVEL_DBG);

// Thread stack size and priority
#define THREAD_CLK_STACK_SIZE 500
#define THREAD_CLK_PRIORITY -1

K_THREAD_STACK_DEFINE(thread_clk_stack_area, THREAD_CLK_STACK_SIZE);
struct k_thread thread_clk_data;

K_TIMER_DEFINE(clk_timer, NULL, NULL);

void generate_clk_signal(const struct gpio_dt_spec *pinclk){
	LOG_INF("generate clk signal thread created");

	gpio_pin_configure_dt(pinclk, GPIO_OUTPUT_ACTIVE);
	k_timer_init(&clk_timer, NULL, NULL);
	k_timer_start(&clk_timer, K_NO_WAIT, K_MSEC(500));

	while(1){
		//LOG_INF("time: %lld", k_uptime_ticks());
		gpio_pin_toggle_dt(pinclk);
		k_timer_status_sync(&clk_timer);
	}
}

=======

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ov7670);
>>>>>>> 1344c67b8c6c7f734385a580335e4b495fe48fab

#define OV7670_REVISION  0x7673U //modified

#define OV7670_GAIN       0x00U
#define OV7670_BLUE       0x01U
#define OV7670_RED        0x02U
#define OV7670_GREEN      0x03U
#define OV7670_BAVG       0x05U
#define OV7670_GAVG       0x06U
#define OV7670_RAVG       0x07U
#define OV7670_AECH       0x08U
#define OV7670_COM2       0x09U
#define OV7670_PID        0x0AU //yes
#define OV7670_VER        0x0BU //yes
#define OV7670_COM3       0x0CU
#define OV7670_COM4       0x0DU
#define OV7670_COM5       0x0EU
#define OV7670_COM6       0x0FU
#define OV7670_AEC        0x10U
#define OV7670_CLKRC      0x11U
#define OV7670_COM7       0x12U //yes
#define OV7670_COM8       0x13U
#define OV7670_COM9       0x14U
#define OV7670_COM10      0x15U
#define OV7670_REG16      0x16U
#define OV7670_HSTART     0x17U
#define OV7670_HSIZE      0x18U
#define OV7670_VSTART     0x19U
#define OV7670_VSIZE      0x1AU
#define OV7670_PSHFT      0x1BU
#define OV7670_MIDH       0x1CU
#define OV7670_MIDL       0x1DU
#define OV7670_LAEC       0x1FU
#define OV7670_COM11      0x20U
#define OV7670_BDBASE     0x22U
#define OV7670_BDMSTEP    0x23U
#define OV7670_AEW        0x24U
#define OV7670_AEB        0x25U
#define OV7670_VPT        0x26U
#define OV7670_REG28      0x28U
#define OV7670_HOUTSIZE   0x29U
#define OV7670_EXHCH      0x2AU
#define OV7670_EXHCL      0x2BU
#define OV7670_VOUTSIZE   0x2CU
#define OV7670_ADVFL      0x2DU
#define OV7670_ADVFH      0x2EU
#define OV7670_YAVE       0x2FU
#define OV7670_LUMHTH     0x30U
#define OV7670_LUMLTH     0x31U
#define OV7670_HREF       0x32U
#define OV7670_DM_LNL     0x33U
#define OV7670_DM_LNH     0x34U
#define OV7670_ADOFF_B    0x35U
#define OV7670_ADOFF_R    0x36U
#define OV7670_ADOFF_GB   0x37U
#define OV7670_ADOFF_GR   0x38U
#define OV7670_OFF_B      0x39U
#define OV7670_OFF_R      0x3AU
#define OV7670_OFF_GB     0x3BU
#define OV7670_OFF_GR     0x3CU
#define OV7670_COM12      0x3DU
#define OV7670_COM13      0x3EU
#define OV7670_COM14      0x3FU
#define OV7670_COM16      0x41U
#define OV7670_TGT_B      0x42U
#define OV7670_TGT_R      0x43U
#define OV7670_TGT_GB     0x44U
#define OV7670_TGT_GR     0x45U
#define OV7670_LC_CTR     0x46U
#define OV7670_LC_XC      0x47U
#define OV7670_LC_YC      0x48U
#define OV7670_LC_COEF    0x49U
#define OV7670_LC_RADI    0x4AU
#define OV7670_LC_COEFB   0x4BU
#define OV7670_LC_COEFR   0x4CU
#define OV7670_FIXGAIN    0x4DU
#define OV7670_AREF1      0x4FU
#define OV7670_AREF6      0x54U
#define OV7670_UFIX       0x60U
#define OV7670_VFIX       0x61U
#define OV7670_AWBB_BLK   0x62U
#define OV7670_AWB_CTRL0  0x63U
#define OV7670_DSP_CTRL1  0x64U
#define OV7670_DSP_CTRL2  0x65U
#define OV7670_DSP_CTRL3  0x66U
#define OV7670_DSP_CTRL4  0x67U
#define OV7670_AWB_BIAS   0x68U
#define OV7670_AWB_CTRL1  0x69U
#define OV7670_AWB_CTRL2  0x6AU
#define OV7670_AWB_CTRL3  0x6BU
#define OV7670_AWB_CTRL4  0x6CU
#define OV7670_AWB_CTRL5  0x6DU
#define OV7670_AWB_CTRL6  0x6EU
#define OV7670_AWB_CTRL7  0x6FU
#define OV7670_AWB_CTRL8  0x70U
#define OV7670_AWB_CTRL9  0x71U
#define OV7670_AWB_CTRL10 0x72U
#define OV7670_AWB_CTRL11 0x73U
#define OV7670_AWB_CTRL12 0x74U
#define OV7670_AWB_CTRL13 0x75U
#define OV7670_AWB_CTRL14 0x76U
#define OV7670_AWB_CTRL15 0x77U
#define OV7670_AWB_CTRL16 0x78U
#define OV7670_AWB_CTRL17 0x79U
#define OV7670_AWB_CTRL18 0x7AU
#define OV7670_AWB_CTRL19 0x7BU
#define OV7670_AWB_CTRL20 0x7CU
#define OV7670_AWB_CTRL21 0x7DU
#define OV7670_GAM1       0x7EU
#define OV7670_GAM2       0x7FU
#define OV7670_GAM3       0x80U
#define OV7670_GAM4       0x81U
#define OV7670_GAM5       0x82U
#define OV7670_GAM6       0x83U
#define OV7670_GAM7       0x84U
#define OV7670_GAM8       0x85U
#define OV7670_GAM9       0x86U
#define OV7670_GAM10      0x87U
#define OV7670_GAM11      0x88U
#define OV7670_GAM12      0x89U
#define OV7670_GAM13      0x8AU
#define OV7670_GAM14      0x8BU
#define OV7670_GAM15      0x8CU
#define OV7670_SLOP       0x8DU
#define OV7670_DNSTH      0x8EU
#define OV7670_EDGE0      0x8FU
#define OV7670_EDGE1      0x90U
#define OV7670_DNSOFF     0x91U
#define OV7670_EDGE2      0x92U
#define OV7670_EDGE3      0x93U
#define OV7670_MTX1       0x94U
#define OV7670_MTX2       0x95U
#define OV7670_MTX3       0x96U
#define OV7670_MTX4       0x97U
#define OV7670_MTX5       0x98U
#define OV7670_MTX6       0x99U
#define OV7670_MTX_CTRL   0x9AU
#define OV7670_BRIGHT     0x9BU
#define OV7670_CNST       0x9CU
#define OV7670_UVADJ0     0x9EU
#define OV7670_UVADJ1     0x9FU
#define OV7670_SCAL0      0xA0U
#define OV7670_SCAL1      0xA1U
#define OV7670_SCAL2      0xA2U
#define OV7670_SDE        0xA6U
#define OV7670_USAT       0xA7U
#define OV7670_VSAT       0xA8U
#define OV7670_HUECOS     0xA9U
#define OV7670_HUESIN     0xAAU
#define OV7670_SIGN       0xABU
#define OV7670_DSPAUTO    0xACU

#define OV7670_COM10_VSYNC_NEG_MASK    BIT(1)
#define OV7670_COM10_HREF_REVERSE_MASK BIT(3)
#define OV7670_COM10_PCLK_REVERSE_MASK BIT(4)
#define OV7670_COM10_PCLK_OUT_MASK     BIT(5)
#define OV7670_COM10_DATA_NEG_MASK     BIT(7)

struct ov7670_config {
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec pins; 
	const struct pwm_dt_spec pinclk;
	char *pwm_node;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
};

struct ov7670_data {
	struct video_format fmt;
};

struct ov7670_clock {
	uint32_t input_clk;
	uint32_t framerate;
	uint8_t clkrc;  /*!< Register CLKRC. */
	uint8_t com4;   /*!< Register COM4. */
	uint8_t dm_lnl; /*!< Register DM_LNL. */
};

struct ov7670_pixel_format {
	uint32_t pixel_format;
	uint8_t com7;
};

struct ov7670_reg {
	uint8_t addr;
	uint8_t value;
};

static const struct ov7670_clock ov7670_clock_configs[] = {
	{ .input_clk = 24000000, .framerate = 30,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x00 },
	{ .input_clk = 24000000, .framerate = 15,
		.clkrc = 0x03, .com4 = 0x41, .dm_lnl = 0x00 },
	{ .input_clk = 24000000, .framerate = 25,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x66 },
	{ .input_clk = 24000000, .framerate = 14,
		.clkrc = 0x03, .com4 = 0x41, .dm_lnl = 0x1a },
	{ .input_clk = 26000000, .framerate = 30,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x2b },
	{ .input_clk = 26000000, .framerate = 15,
		.clkrc = 0x03, .com4 = 0x41, .dm_lnl = 0x2b },
	{ .input_clk = 26000000, .framerate = 25,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x99 },
	{ .input_clk = 26000000, .framerate = 14,
		.clkrc = 0x03, .com4 = 0x41, .dm_lnl = 0x46 },
	{ .input_clk = 13000000, .framerate = 30,
		.clkrc = 0x00, .com4 = 0x41, .dm_lnl = 0x2b },
	{ .input_clk = 13000000, .framerate = 15,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x2b },
	{ .input_clk = 13000000, .framerate = 25,
		.clkrc = 0x00, .com4 = 0x41, .dm_lnl = 0x99 },
	{ .input_clk = 13000000, .framerate = 14,
		.clkrc = 0x01, .com4 = 0x41, .dm_lnl = 0x46 },
};


static const struct ov7670_pixel_format ov7670_pf_configs[] = {
	{ .pixel_format = VIDEO_PIX_FMT_RGB565, .com7 = (1 << 2) | (2) }
};

static const struct ov7670_reg ov7670_init_reg_tb[] = {
	/*Output config*/
	{ OV7670_CLKRC,          0x00 },
	{ OV7670_COM7,           0x06 },
	{ OV7670_HSTART,         0x3f },
	{ OV7670_HSIZE,          0x50 },
	{ OV7670_VSTART,         0x03 },
	{ OV7670_VSIZE,          0x78 },
	{ OV7670_HREF,           0x00 },
	{ OV7670_HOUTSIZE,       0x50 },
	{ OV7670_VOUTSIZE,       0x78 },

	/*DSP control*/
	{ OV7670_TGT_B,          0x7f },
	{ OV7670_FIXGAIN,        0x09 },
	{ OV7670_AWB_CTRL0,      0xe0 },
	{ OV7670_DSP_CTRL1,      0xff },
	{ OV7670_DSP_CTRL2,      0x00 },
	{ OV7670_DSP_CTRL3,      0x00 },
	{ OV7670_DSP_CTRL4,      0x00 },

	/*AGC AEC AWB*/
	{ OV7670_COM8,           0xf0 },
	{ OV7670_COM4,           0x81 },
	{ OV7670_COM6,           0xc5 },
	{ OV7670_COM9,           0x11 },
	{ OV7670_BDBASE,         0x7F },
	{ OV7670_BDMSTEP,        0x03 },
	{ OV7670_AEW,            0x40 },
	{ OV7670_AEB,            0x30 },
	{ OV7670_VPT,            0xa1 },
	{ OV7670_EXHCL,          0x9e },
	{ OV7670_AWB_CTRL3,      0xaa },
	{ OV7670_COM8,           0xff },

	/*matrix sharpness brightness contrast*/
	{ OV7670_EDGE1,          0x08 },
	{ OV7670_DNSOFF,         0x01 },
	{ OV7670_EDGE2,          0x03 },
	{ OV7670_EDGE3,          0x00 },
	{ OV7670_MTX1,           0xb0 },
	{ OV7670_MTX2,           0x9d },
	{ OV7670_MTX3,           0x13 },
	{ OV7670_MTX4,           0x16 },
	{ OV7670_MTX5,           0x7b },
	{ OV7670_MTX6,           0x91 },
	{ OV7670_MTX_CTRL,       0x1e },
	{ OV7670_BRIGHT,         0x08 },
	{ OV7670_CNST,           0x20 },
	{ OV7670_UVADJ0,         0x81 },
	{ OV7670_SDE,            0X06 },
	{ OV7670_USAT,           0x65 },
	{ OV7670_VSAT,           0x65 },
	{ OV7670_HUECOS,         0X80 },
	{ OV7670_HUESIN,         0X80 },

	/*GAMMA config*/
	{ OV7670_GAM1,           0x0c },
	{ OV7670_GAM2,           0x16 },
	{ OV7670_GAM3,           0x2a },
	{ OV7670_GAM4,           0x4e },
	{ OV7670_GAM5,           0x61 },
	{ OV7670_GAM6,           0x6f },
	{ OV7670_GAM7,           0x7b },
	{ OV7670_GAM8,           0x86 },
	{ OV7670_GAM9,           0x8e },
	{ OV7670_GAM10,          0x97 },
	{ OV7670_GAM11,          0xa4 },
	{ OV7670_GAM12,          0xaf },
	{ OV7670_GAM13,          0xc5 },
	{ OV7670_GAM14,          0xd7 },
	{ OV7670_GAM15,          0xe8 },
	{ OV7670_SLOP,           0x20 },

	{ OV7670_COM3,           0x40 },
	{ OV7670_COM5,           0xf5 },
	{ OV7670_COM10,          0x02 },
	{ OV7670_COM2,           0x01 }
};

static int ov7670_write_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr,
			    uint8_t value)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = (uint8_t *)&reg_addr;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = (uint8_t *)&value;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(spec, msgs, 2);
}

static int ov7670_read_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr,
			   uint8_t *value)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = (uint8_t *)&reg_addr;
	msgs[0].len = 1;
	/*
	 * When using I2C to read the registers of the SCCB device,
	 * a stop bit is required after writing the register address
	 */
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	msgs[1].buf = (uint8_t *)value;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP | I2C_MSG_RESTART;

	return i2c_transfer_dt(spec, msgs, 2);
}

int ov7670_modify_reg(const struct i2c_dt_spec *spec,
		      uint8_t reg_addr,
		      uint8_t clear_mask,
		      uint8_t value)
{
	int ret;
	uint8_t set_value;

	ret = ov7670_read_reg(spec, reg_addr, &set_value);

	if (ret == 0) {
		set_value = (set_value & (~clear_mask)) |
				(set_value & clear_mask);
		ret = ov7670_write_reg(spec, reg_addr, set_value);
	}


	return ret;
}

static int ov7670_write_all(const struct device *dev,
			    const struct ov7670_reg *regs,
			    uint16_t reg_num)
{
	uint16_t i = 0;
	const struct ov7670_config *cfg = dev->config;

	for (i = 0; i < reg_num; i++) {
		int err;

		err = ov7670_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);
		if (err) {
			return err;
		}
	}

	return 0;
}

static int ov7670_set_clock(const struct device *dev,
				unsigned int framerate,
				unsigned int input_clk)
{
	const struct ov7670_config *cfg = dev->config;

	for (unsigned int i = 0; i < ARRAY_SIZE(ov7670_clock_configs); i++) {
		if ((ov7670_clock_configs[i].framerate == framerate) &&
			(ov7670_clock_configs[i].input_clk == input_clk)) {
			ov7670_write_reg(&cfg->i2c, OV7670_CLKRC,
						ov7670_clock_configs[i].clkrc);
			ov7670_modify_reg(&cfg->i2c, OV7670_COM4, 0xc0,
						ov7670_clock_configs[i].com4);
			ov7670_write_reg(&cfg->i2c, OV7670_EXHCL, 0x00);
			ov7670_write_reg(&cfg->i2c, OV7670_DM_LNL,
						ov7670_clock_configs[i].dm_lnl);
			ov7670_write_reg(&cfg->i2c, OV7670_DM_LNH, 0x00);
			ov7670_write_reg(&cfg->i2c, OV7670_ADVFL, 0x00);
			ov7670_write_reg(&cfg->i2c, OV7670_ADVFH, 0x00);
			return ov7670_write_reg(&cfg->i2c, OV7670_COM5, 0x65);
		}
	}

	return -1;
}

static int ov7670_set_fmt(const struct device *dev,
			  enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct ov7670_data *drv_data = dev->data;
	const struct ov7670_config *cfg = dev->config;
	uint8_t com10 = 0;
	uint16_t width, height;
	uint16_t hstart, vstart, hsize;
	int ret;

	/* we only support one format for now (VGA RGB565) */
	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565 || fmt->height != 480 ||
	    fmt->width != 640) {
		return -ENOTSUP;
	}

	width = fmt->width;
	height = fmt->height;

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		/* nothing to do */
		return 0;
	}

	drv_data->fmt = *fmt;

	/* Configure Sensor */
	ret = ov7670_write_all(dev, ov7670_init_reg_tb,
				ARRAY_SIZE(ov7670_init_reg_tb));
	if (ret) {
		LOG_ERR("Unable to write ov7670 config");
		return ret;
	}

	/* Set clock : framerate 30fps, input clock 24M*/
	ov7670_set_clock(dev, 30, 24000000);

	/* Set output format */
	for (uint8_t i = 0; i < ARRAY_SIZE(ov7670_pf_configs); i++) {
		if (ov7670_pf_configs[i].pixel_format == fmt->pixelformat) {
			ret =  ov7670_modify_reg(&cfg->i2c,
						OV7670_COM7,
						0x1FU,
						ov7670_pf_configs[i].com7);
			if (ret) {
				LOG_ERR("Unable to write ov7670 pixel format");
				return ret;
			}
		}
	}

	ov7670_modify_reg(&cfg->i2c, OV7670_COM7, (1 << 5), (0 << 5));

	com10 |= OV7670_COM10_VSYNC_NEG_MASK;
	ov7670_write_reg(&cfg->i2c, OV7670_COM10, com10);

	/* Don't swap output MSB/LSB. */
	ov7670_write_reg(&cfg->i2c, OV7670_COM3, 0x00);

	/*
	 * Output drive capability
	 * 0: 1X
	 * 1: 2X
	 * 2: 3X
	 * 3: 4X
	 */
	ov7670_modify_reg(&cfg->i2c, OV7670_COM2, 0x03, 0x03);

	/* Resolution and timing. */
	hstart = 0x22U << 2U;
	vstart = 0x07U << 1U;
	hsize = width + 16U;

	/* Set the window size. */
	ov7670_write_reg(&cfg->i2c, OV7670_HSTART, hstart >> 2U);
	ov7670_write_reg(&cfg->i2c, OV7670_HSIZE, hsize >> 2U);
	ov7670_write_reg(&cfg->i2c, OV7670_VSTART, vstart >> 1U);
	ov7670_write_reg(&cfg->i2c, OV7670_VSIZE, height >> 1U);
	ov7670_write_reg(&cfg->i2c, OV7670_HOUTSIZE, width >> 2U);
	ov7670_write_reg(&cfg->i2c, OV7670_VOUTSIZE, height >> 1U);
	ov7670_write_reg(&cfg->i2c, OV7670_HREF,
			 ((vstart & 1U) << 6U) |
			 ((hstart & 3U) << 4U) |
			 ((height & 1U) << 2U) |
			 ((hsize & 3U) << 0U));
	return ov7670_write_reg(&cfg->i2c, OV7670_EXHCH,
					((height & 1U) << 2U) |
					((width & 3U) << 0U));
}

static int ov7670_get_fmt(const struct device *dev,
			  enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct ov7670_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int ov7670_stream_start(const struct device *dev)
{
	return 0;
}

static int ov7670_stream_stop(const struct device *dev)
{
	return 0;
}

static const struct video_format_cap fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width_min = 640,
		.width_max = 640,
		.height_min = 480,
		.height_max = 480,
		.width_step = 0,
		.height_step = 0,
	},
	{ 0 }
};

static int ov7670_get_caps(const struct device *dev,
			   enum video_endpoint_id ep,
			   struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static const struct video_driver_api ov7670_driver_api = {
	.set_format = ov7670_set_fmt,
	.get_format = ov7670_get_fmt,
	.get_caps = ov7670_get_caps,
	.stream_start = ov7670_stream_start,
	.stream_stop = ov7670_stream_stop,
};

static int ov7670_init(const struct device *dev)
{
	const struct ov7670_config *cfg = dev->config;
	struct video_format fmt;
	uint8_t pid, ver;
	int ret;

	LOG_DBG("Wait some seconds");
	k_msleep(2 * MSEC_PER_SEC);
	LOG_DBG("Starting the ov7670 camera");

	LOG_INF("**** Starting a new scan ****\n");
	uint8_t data = 0x00;
	for(int add = 0; add < 127; add++ ){
		/* 3. verify i2c_write() */
		if (i2c_write(cfg->i2c.bus, &data, 1, add) == 0) {
			LOG_INF("Found sensor address 0x%02x\n", add);
		}
		k_msleep(1);
	}
	LOG_INF("**** Scanner finished ****\n");


#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	LOG_INF("It has a reset pin");
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	gpio_pin_set_dt(&cfg->reset_gpio, 0);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&cfg->reset_gpio, 1);
	k_sleep(K_MSEC(1));
#endif

	/* Identify the device. */
	ret = ov7670_read_reg(&cfg->i2c, OV7670_PID, &pid);
	if (ret) {
		LOG_ERR("Unable to read PID");
		return -ENODEV;
	}

	ret = ov7670_read_reg(&cfg->i2c, OV7670_VER, &ver);
	if (ret) {
		LOG_ERR("Unable to read VER");
		return -ENODEV;
	}

	LOG_DBG("PID -> 0x%x, VER -> 0x%x", pid, ver);

	if (OV7670_REVISION != (((uint32_t)pid << 8U) | (uint32_t)ver)) {
		LOG_ERR("OV7670 Get Vision fail\n");
		return -ENODEV;
	}

	LOG_DBG("The resvision ID is correct");

	/* Device identify OK, perform software reset. */
	ov7670_write_reg(&cfg->i2c, OV7670_COM7, 0x80);
	LOG_DBG("Reseting camera");

	k_sleep(K_MSEC(2));

	/* set default/init format VGA RGB565 */
	/*fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = 640;
	fmt.height = 480;
	fmt.pitch = 640 * 2;
	ret = ov7670_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}*/

	LOG_DBG("camera ready");

	return 0;
}

static struct ov7670_data ov7670_data_0;

static int ov7670_init_0(const struct device *dev)
{
	const struct ov7670_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	if (!device_is_ready(cfg->reset_gpio.port)) {
		LOG_ERR("%s: device %s is not ready", dev->name,
				cfg->reset_gpio.port->name);
		return -ENODEV;
	}
#endif

	k_tid_t thread_clk_id = k_thread_create(&thread_clk_data, thread_clk_stack_area,
					K_THREAD_STACK_SIZEOF(thread_clk_stack_area),
					generate_clk_signal,
					&cfg->pins, NULL, NULL,
					THREAD_CLK_PRIORITY, 0, K_NO_WAIT);
	k_thread_start(thread_clk_id);

	if (!device_is_ready(cfg->pinclk.dev)) {
		LOG_ERR("pwm device is not ready");
		return -ENODEV;
	}

	LOG_INF("PWM channel %d, period: %d", cfg->pinclk.channel, cfg->pinclk.period);


	int ret = pwm_set_dt(&cfg->pinclk, cfg->pinclk.period, cfg->pinclk.period / 2U);
	if (ret != 0){
		LOG_ERR("set period error: %d", ret);
	}

	return ov7670_init(dev);
}

static const struct ov7670_config ov7670_cfg_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
	.pins = GPIO_DT_SPEC_INST_GET(0, dvp_gpios),
	.pinclk = PWM_DT_SPEC_GET(DT_INST_PHANDLE(0, pwm)),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif
};


DEVICE_DT_INST_DEFINE(0, &ov7670_init_0, NULL,
		    &ov7670_data_0, &ov7670_cfg_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &ov7670_driver_api);

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
#include <zephyr/drivers/pwm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ov7670, LOG_LEVEL_INF);

#define OV7670_REVISION  0x7673U //modified

#define OV7670_GAIN		0x00U
#define OV7670_BLUE		0x01U
#define OV7670_RED		0x02U
#define OV7670_VREF		0x03U
#define OV7670_COM1		0x04U
#define OV7670_BAVE		0x05U
#define OV7670_GBAVE		0x06U
#define OV7670_AECHH		0x07U
#define OV7670_RAVE		0x08U
#define OV7670_COM2		0x09U
#define OV7670_PID		0x0AU
#define OV7670_VER		0x0BU
#define OV7670_COM3		0x0CU
#define OV7670_COM4		0x0DU
#define OV7670_COM5		0x0EU
#define OV7670_COM6		0x0FU
#define OV7670_AECH		0x10U
#define OV7670_CLKRC		0x11U
#define OV7670_COM7		0x12U
#define OV7670_COM8		0x13U
#define OV7670_COM9		0x14U
#define OV7670_COM10		0x15U
#define OV7670_HSTART		0x17U
#define OV7670_HSTOP		0x18U
#define OV7670_VSTART		0x19U
#define OV7670_VSTOP		0x1AU
#define OV7670_PSHFT		0x1BU
#define OV7670_MIDH		0x1CU
#define OV7670_MIDL		0x1DU
#define OV7670_MVFP		0x1EU
#define OV7670_LAEC		0x1FU
#define OV7670_ADCCTR0		0x20U
#define OV7670_ADCCTR1		0x21U
#define OV7670_ADCCTR2		0x22U
#define OV7670_ADCCTR3		0x23U
#define OV7670_AEW		0x24U
#define OV7670_AEB		0x25U
#define OV7670_VPT		0x26U
#define OV7670_BBIAS		0x27U
#define OV7670_GBBIAS		0x28U
#define OV7670_EXHCH		0x2AU
#define OV7670_EXHCL		0x2BU
#define OV7670_RBIAS		0x2CU
#define OV7670_ADVFL		0x2DU
#define OV7670_ADVFH		0x2EU
#define OV7670_YAVE		0x2FU
#define OV7670_HSYST		0x30U
#define OV7670_HSYEN		0x31U
#define OV7670_HREF		0x32U
#define OV7670_CHLF		0x33U
#define OV7670_ARBLM		0x34U
#define OV7670_ADC		0x37U
#define OV7670_ACOM		0x38U
#define OV7670_OFON		0x39U
#define OV7670_TSLB		0x3AU
#define OV7670_COM11		0x3BU
#define OV7670_COM12		0x3CU
#define OV7670_COM13		0x3DU
#define OV7670_COM14		0x3EU
#define OV7670_EDGE		0x3FU
#define OV7670_COM15		0x40U
#define OV7670_COM16		0x41U
#define OV7670_COM17		0x42U
#define OV7670_AWBC1		0x43U
#define OV7670_AWBC2		0x44U
#define OV7670_AWBC3		0x45U
#define OV7670_AWBC4		0x46U
#define OV7670_AWBC5		0x47U
#define OV7670_AWBC6		0x48U
#define OV7670_REG4B		0x4BU
#define OV7670_DNSTH		0x4CU
#define OV7670_MTX1		0x4FU
#define OV7670_MTX2		0x50U
#define OV7670_MTX3		0x51U
#define OV7670_MTX4		0x52U
#define OV7670_MTX5		0x53U
#define OV7670_MTX6		0x54U
#define OV7670_BRIGHT		0x55U
#define OV7670_CONTRAS		0x56U
#define OV7670_CONTRAS_CENTER	0x57U
#define OV7670_MTXS		0x58U
#define OV7670_LCC1		0x62U
#define OV7670_LCC2		0x63U
#define OV7670_LCC3		0x64U
#define OV7670_LCC4		0x65U
#define OV7670_LCC5		0x66U
#define OV7670_MANU		0x67U
#define OV7670_MANV		0x68U
#define OV7670_GFIX		0x69U
#define OV7670_GGAIN		0x6AU
#define OV7670_DBLV		0x6BU
#define OV7670_AWBCTR3		0x6CU
#define OV7670_AWBCTR2		0x6DU
#define OV7670_AWBCTR1		0x6EU
#define OV7670_AWBCTR0		0x6FU
#define OV7670_SCALING_XSC	0x70U
#define OV7670_SCALING_YSC	0x71U
#define OV7670_SCALING_DCWCTR	0x72U
#define OV7670_SCALING_PCLK_DIV	0x73U
#define OV7670_REG74		0x74U
#define OV7670_REG75		0x75U
#define OV7670_REG76		0x76U
#define OV7670_REG77		0x77U
#define OV7670_SLOP		0x7AU
#define OV7670_GAM1		0x7BU
#define OV7670_GAM2		0x7CU
#define OV7670_GAM3		0x7DU
#define OV7670_GAM4		0x7EU
#define OV7670_GAM5		0x7FU
#define OV7670_GAM6		0x80U
#define OV7670_GAM7		0x81U
#define OV7670_GAM8		0x82U
#define OV7670_GAM9		0x83U
#define OV7670_GAM10		0x84U
#define OV7670_GAM11		0x85U
#define OV7670_GAM12		0x86U
#define OV7670_GAM13		0x87U
#define OV7670_GAM14		0x88U
#define OV7670_GAM15		0x89U
#define OV7670_RGB444		0x8CU
#define OV7670_DM_LNL		0x92U
#define OV7670_DM_LNH		0x93U
#define OV7670_LCC6		0x94U
#define OV7670_LCC7		0x95U
#define OV7670_BD50ST		0x9DU
#define OV7670_BD60ST		0x9EU
#define OV7670_HAECC1		0x9FU
#define OV7670_HAECC2		0xA0U
#define OV7670_SCAL_PCLK_DELAY	0xA2U
#define OV7670_NT_CTRL		0xA4U
#define OV7670_BD50MAX		0xA5U
#define OV7670_HAECC3		0xA6U
#define OV7670_HAECC4		0xA7U
#define OV7670_HAECC5		0xA8U
#define OV7670_HAECC6		0xA9U
#define OV7670_HAECC7		0xAAU
#define OV7670_BD60MAX		0xABU
#define OV7670_STR_OPT		0xACU
#define OV7670_STR_R		0xADU
#define OV7670_STR_G		0xAEU
#define OV7670_STR_B		0xAFU
#define OV7670_ABLC1		0xB1U
#define OV7670_THL_ST		0xB3U
#define OV7670_THL_DLT		0xB5U
#define OV7670_AD_CHB		0xBEU
#define OV7670_AD_CHR		0xBFU
#define OV7670_AD_CHBB		0xC0U
#define OV7670_AD_CHGR		0xC1U
#define OV7670_SATCTR		0xC9U


#define  OV7670_COM8_FASTAEC	BIT(7)	/* Enable fast AGC/AEC */
#define  OV7670_COM8_AECSTEP	BIT(6)	/* Unlimited AEC step size */
#define  OV7670_COM8_BFILT	BIT(5)	/* Band filter enable */
#define  OV7670_COM8_AGC	BIT(2)	/* Auto gain enable */
#define  OV7670_COM8_AWB	BIT(1)	/* White balance enable */
#define  OV7670_COM8_AEC	BIT(0)	/* Auto exposure enable */


#define OV7670_COM10_VSYNC_NEG_MASK    BIT(1)
#define OV7670_COM10_HREF_REVERSE_MASK BIT(3)
#define OV7670_COM10_PCLK_REVERSE_MASK BIT(4)
#define OV7670_COM10_PCLK_OUT_MASK     BIT(5)
#define OV7670_COM10_DATA_NEG_MASK     BIT(7)

struct ov7670_config {
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec dvp_hsync; 
	const struct gpio_dt_spec dvp_vsync; 
	const struct gpio_dt_spec dvp_pclk; 
	const struct pwm_dt_spec pinclk;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
};

struct ov7670_data {
	struct video_format fmt;
	struct gpio_callback vsync_cb;
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
	/** COM7
	 * bit[5] -> CIF =  352 × 288 pixels
	 * bit[4] -> QVGA = 320 × 240 pixels
	 * bit[3] -> QCIF = 176 × 144 pixels
	 * bit[2] -> RGB selection (1) or YUV (0)
	 * bit[1] -> color bar disable (0) or enable (1)
	 * bit[0] -> raw RBG (1) or YUV (0)
	 */

	/*Output format 30fps QCIF (176 × 144 pixels) YUV mode */
	{ OV7670_CLKRC,			0x01 },
	{ OV7670_COM7,			0x00 },
	{ OV7670_COM3,			0x0C },
	{ OV7670_COM14,			0x11 },
	{ OV7670_SCALING_XSC,		0x3A },
	{ OV7670_SCALING_YSC,		0x35 },
	{ OV7670_SCALING_DCWCTR,	0x11 },
	{ OV7670_SCALING_PCLK_DIV,	0xF1 },
	{ OV7670_SCAL_PCLK_DELAY,	0x52 },
	{ OV7670_COM10,          0x00 }, // normal timing singals (positives)

	/*{ OV7670_HSTART,         0x13 },
	{ OV7670_HSTOP,          0x50 },
	{ OV7670_VSTART,         0x03 },
	{ OV7670_VSTOP,          0x78 },
	{ OV7670_HREF,           0x00 },*/

	/*AGC AEC AWB*/
	{ OV7670_COM8,			OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BFILT },
	{ OV7670_GAIN,			0x00 },	
	{ OV7670_AECH,			0x00 },
	{ OV7670_COM4,			0x40 }, /* magic reserved bit */
	{ OV7670_COM6,			0x02 },	/* reset timing when format changes */
	{ OV7670_COM9,			0x11 }, /* 4x gain + Freeze AGC/AEC*/ /* original magic rsvd bit (0x18) */
	{ OV7670_BD50MAX,		0x05 },	
	{ OV7670_BD60MAX,		0x07 },
	{ OV7670_AEW,			0x95 },	
	{ OV7670_AEB,			0x33 },
	{ OV7670_VPT,			0xe3 },	
	{ OV7670_HAECC1,		0x78 },
	{ OV7670_HAECC2,		0x68 },	
	{ 0xa1, 0x03 }, /* reserved magic */
	{ OV7670_HAECC3,		0xd8 },	
	{ OV7670_HAECC4,		0xd8 },
	{ OV7670_HAECC5,		0xf0 },	
	{ OV7670_HAECC6,		0x90 },
	{ OV7670_HAECC7,		0x94 },
	{ OV7670_COM8,		OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | 
				OV7670_COM8_BFILT | OV7670_COM8_AGC | OV7670_COM8_AEC },

	/* matrix sharpness brightness contrast*/
	// TODO: how are they calculated?
	{ OV7670_MTX1,		0x80 },
	{ OV7670_MTX2,		0x80 },
	{ OV7670_MTX3,		0x00 },
	{ OV7670_MTX4,		0x22 },
	{ OV7670_MTX5,		0x5E },
	{ OV7670_MTX6,		0x80 },
	{ OV7670_MTXS,		0x9E },
	{ OV7670_BRIGHT,	0x08 },

	/*GAMMA config*/
	{ OV7670_GAM1,			0x10 },
	{ OV7670_GAM2,			0x1e },
	{ OV7670_GAM3,			0x35 },
	{ OV7670_GAM4,			0x5a },
	{ OV7670_GAM5,			0x69 },
	{ OV7670_GAM6,			0x76 },
	{ OV7670_GAM7,			0x80 },
	{ OV7670_GAM8,			0x88 },
	{ OV7670_GAM9,			0x8f },
	{ OV7670_GAM10,			0x96 },
	{ OV7670_GAM11,			0xa3 },
	{ OV7670_GAM12,			0xaf },
	{ OV7670_GAM13,			0xc4 },
	{ OV7670_GAM14,			0xd7 },
	{ OV7670_GAM15,			0xe8 },
	{ OV7670_SLOP,			0x20 },

	/* Almost all of these are magic "reserved" values.  */
	{ OV7670_COM5,           	0xf5 },
	{ OV7670_COM12,			0x78 },
	{ OV7670_GFIX,			0x00 },
	{ OV7670_COM6,			0x4b },
	{ 0x16, 0x02 },	/* reserved magic */	
	{ OV7670_MVFP,			0x07 },


	/* Almost all of these are magic "reserved" values.  */
	{ 0x16, 0x02 },	
	{ 0x21, 0x02 },		{ 0x22, 0x91 },
	{ 0x29, 0x07 },		{ 0x33, 0x0b },
	{ 0x35, 0x0b },		{ 0x37, 0x1d },
	{ 0x38, 0x71 },		{ 0x39, 0x2a },
	{ 0x4d, 0x40 },
	{ 0x4e, 0x20 },	
	{ 0x6b, 0x4a },		{ 0x74, 0x10 },
	{ 0x8d, 0x4f },		{ 0x8e, 0 },
	{ 0x8f, 0 },		{ 0x90, 0 },
	{ 0x91, 0 },		{ 0x96, 0 },
	{ 0x9a, 0 },		{ 0xb0, 0x84 },
	{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
	{ 0xb3, 0x82 },		{ 0xb8, 0x0a },

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
	{ 0x79, 0x0f },		{ 0xc8, 0x00 },
	{ 0x79, 0x10 },		{ 0xc8, 0x7e },
	{ 0x79, 0x0a },		{ 0xc8, 0x80 },
	{ 0x79, 0x0b },		{ 0xc8, 0x01 },
	{ 0x79, 0x0c },		{ 0xc8, 0x0f },
	{ 0x79, 0x0d },		{ 0xc8, 0x20 },
	{ 0x79, 0x09 },		{ 0xc8, 0x80 },
	{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
	{ 0x79, 0x03 },		{ 0xc8, 0x40 },
	{ 0x79, 0x05 },		{ 0xc8, 0x30 },
	{ 0x79, 0x26 },
};

static int ov7670_write_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr,
			    uint8_t value)
{
	LOG_DBG("writing to 0x%x, reg: 0x%x, val: 0x%x", spec->addr, reg_addr, value);
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
	int rc = 0;

	rc = i2c_write_dt(spec, &reg_addr, 1);
	if (rc != 0) {
		LOG_ERR("error writing: %d", rc);
		return rc;
	}

	rc = i2c_read_dt(spec, value, 1);
	if (rc != 0) {
		LOG_ERR("error read: %d", rc);
		return rc;
	}

	LOG_DBG("Read from 0x%x, reg = 0x%x, val: 0x%x", spec->addr, reg_addr, *value);

	return 0;
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
	//ov7670_set_clock(dev, 30, 24000000);

	/* Set output format */
	/*for (uint8_t i = 0; i < ARRAY_SIZE(ov7670_pf_configs); i++) {
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
	ov7670_write_reg(&cfg->i2c, OV7670_COM10, com10);*/

	/* Don't swap output MSB/LSB. */
	//ov7670_write_reg(&cfg->i2c, OV7670_COM3, 0x00);

	/*
	 * Output drive capability
	 * 0: 1X
	 * 1: 2X
	 * 2: 3X
	 * 3: 4X
	 */
	//ov7670_modify_reg(&cfg->i2c, OV7670_COM2, 0x03, 0x03);

	/* Resolution and timing. */
	/*hstart = 0x22U << 2U;
	vstart = 0x07U << 1U;
	hsize = width + 16U;*/

	/* Set the window size. */
	/*ov7670_write_reg(&cfg->i2c, OV7670_HSTART, hstart >> 2U);
	ov7670_write_reg(&cfg->i2c, OV7670_HSTOP, hsize >> 2U);
	ov7670_write_reg(&cfg->i2c, OV7670_VSTART, vstart >> 1U);
	ov7670_write_reg(&cfg->i2c, OV7670_VSTOP, height >> 1U);
	ov7670_write_reg(&cfg->i2c, OV7670_HREF,
			 ((vstart & 1U) << 6U) |
			 ((hstart & 3U) << 4U) |
			 ((height & 1U) << 2U) |
			 ((hsize & 3U) << 0U));
	ov7670_write_reg(&cfg->i2c, OV7670_EXHCH,
					((height & 1U) << 2U) |
					((width & 3U) << 0U));*/
	return 0;
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

	LOG_INF("Starting the ov7670 camera");

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
		LOG_ERR("Unable to read PID, ret = %d", ret);
		return -ENODEV;
	}

	ret = ov7670_read_reg(&cfg->i2c, OV7670_VER, &ver);
	if (ret) {
		LOG_ERR("Unable to read VER");
		return -ENODEV;
	}

	LOG_INF("PID -> 0x%x, VER -> 0x%x", pid, ver);

	if (OV7670_REVISION != (((uint32_t)pid << 8U) | (uint32_t)ver)) {
		LOG_ERR("OV7670 Get Vision fail\n");
		return -ENODEV;
	}

	LOG_INF("The revision ID is correct");

	/* Device identify OK, perform software reset. */
	ret = ov7670_write_reg(&cfg->i2c, OV7670_COM7, 0x80);
	if (ret) {
		LOG_ERR("Fail to reset the camera. Error: %d", ret);
		return -ENODEV;
	}
	LOG_INF("Reseting camera");

	k_sleep(K_MSEC(2));

	/* set default/init format VGA RGB565 */
	/*fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = 176;
	fmt.height = 144;
	fmt.pitch = fmt.width * 2;
	ret = ov7670_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}*/

	/* Configure Sensor */
	ret = ov7670_write_all(dev, ov7670_init_reg_tb,
				ARRAY_SIZE(ov7670_init_reg_tb));
	if (ret) {
		LOG_ERR("Unable to write ov7670 config");
		return ret;
	}

	LOG_INF("camera ready");

	return 0;
}

static struct ov7670_data ov7670_data_0;

static void vsync_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	static int i = 0;
	i++;
	LOG_INF("Int. cnt: %d", i);
}

static int dvp_interrupts_init(const struct device *dev)
{
	const struct ov7670_config *cfg = dev->config;
	const struct ov7670_data *drv_data = dev->data;
	int ret = 0;

	if (!device_is_ready(cfg->dvp_vsync.port)) {
		LOG_ERR("Vsync GPIO device is not ready");
		return -ENODEV;
	}
	
	LOG_INF("Vsync GPIO device is ready");

	// configure pin as input
	ret = gpio_pin_configure_dt(&cfg->dvp_vsync, GPIO_INPUT);
	if(ret != 0){
		LOG_ERR("Failed to configure gpio pin. Err: %d", ret);
		return ret;
	}

	// initialize the callback
	gpio_init_callback(&drv_data->vsync_cb,
			   vsync_callback,
			   BIT(cfg->dvp_vsync.pin));

	// add the callback to our pin
	ret = gpio_add_callback(cfg->dvp_vsync.port, &drv_data->vsync_cb);
	if (ret != 0) {
		LOG_ERR("Failed to set gpio callback. Err: %d", ret);
		return -EIO;
	}

	// Trigger the interrupt at edge rising level
	ret = gpio_pin_interrupt_configure_dt(&cfg->dvp_vsync,
					GPIO_INT_EDGE_RISING);

	if (ret != 0) {
		LOG_ERR("Failed to configure gpio pin interrupt. Err: %d", ret);
		return ret;
	}

	return 0;
}

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

	// Set CLK signal
	if (!device_is_ready(cfg->pinclk.dev)) {
		LOG_ERR("pwm device is not ready");
		return -ENODEV;
	}
	LOG_INF("PWM channel %d, period: %d", cfg->pinclk.channel, cfg->pinclk.period);
	int ret = pwm_set_dt(&cfg->pinclk, cfg->pinclk.period, cfg->pinclk.period / 2U);
	if (ret != 0){
		LOG_ERR("set period error: %d", ret);
	}

	// Set dvp interrupts
	ret = dvp_interrupts_init(dev);
	if (ret != 0){
		LOG_ERR("Error initializing dvp interrupt. Err: %d", ret);
		return ret;
	}

	return ov7670_init(dev);
}

static const struct ov7670_config ov7670_cfg_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
	.dvp_hsync = GPIO_DT_SPEC_INST_GET_BY_IDX(0, dvp_gpios, 0),
	.dvp_vsync = GPIO_DT_SPEC_INST_GET_BY_IDX(0, dvp_gpios, 1),
	.dvp_pclk = GPIO_DT_SPEC_INST_GET_BY_IDX(0, dvp_gpios, 2),
	.pinclk = PWM_DT_SPEC_GET(DT_INST_PHANDLE(0, pwm)),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif
};


DEVICE_DT_INST_DEFINE(0, &ov7670_init_0, NULL,
		    &ov7670_data_0, &ov7670_cfg_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &ov7670_driver_api);

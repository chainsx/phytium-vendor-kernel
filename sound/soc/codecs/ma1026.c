//SPDX-License-Identifier:GPL-2.0
/*
 * ma1026 ALSA SoC audio driver 
 *
 * Copyright (c) Cubiclattice semiconductor.co.,Ltd
 * Copyright (c) 2022-2024 PhytiumTechnology co.,"Ltd.
 */

#include <generated/uapi/linux/version.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/stddef.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <asm/unaligned.h>

#include "ma1026.h"

#define TEMP_DEBUG 0
#define MA_DBG(fmt...)	printk(fmt)

#define MA1026_LRCK (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define MA1026_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	#define snd_soc_write_ma1026(codec, addr, value)		\
		snd_soc_write(codec, addr, value);
	#define snd_soc_update_ma1026(component, reg, mask, val)	\
		snd_soc_update_bits(component, reg, mask, val)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5, 00, 0))
	#define snd_soc_write_ma1026(codec, addr, value)		\
		snd_soc_component_write(codec, addr, value);
	#define snd_soc_update_ma1026(component, reg, mask, val)	\
		snd_soc_component_update_bits(component, reg, mask, val)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 00, 0))
	#define snd_soc_write_ma1026(codec, addr, value)		\
		snd_soc_component_write(codec, addr, value);
	#define snd_soc_update_ma1026(component, reg, mask, val)	\
		snd_soc_component_update_bits(component, reg, mask, val)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	#define snd_soc_read_ma1026(codec, addr)			\
		snd_soc_read(codec, addr);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5, 00, 0))
	#define snd_soc_read_ma1026(codec, addr)			\
		snd_soc_component_read32(codec, addr);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 00, 0))
	#define snd_soc_read_ma1026(codec, addr)			\
		snd_soc_component_read(codec, addr);
#endif

struct sp_config {
	u8 spc, mmcc, spfs;
	u32 lrckfreq;
};

struct  ma1026_priv{
	struct gpio_desc *gpiod_spk_ctl;
	struct sp_config config;
	struct regmap *regmap;
	struct clk *mclk;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	int shutdwn_delay;
	bool hp_mute;
	bool hp_inserted;
	long force_muted;
	long playback_path;
	long capture_path;
};

struct snd_pcm_constr_list {
	int count;
	u32 *list;
};

struct ma1026_mclk_div {
	u32 mclk, lrckfreq;
	u8 mmcc;
};

static struct ma1026_mclk_div ma1026_mclk_coeffs[] = {
	/* MCLK, LRCK, MMCC[5:0] */
	{5644800, 11025, 0x30},
	{5644800, 22050, 0x20},
	{5644800, 44100, 0x10},

	{6144000, 8000, 0x38},
	{6144000, 12000, 0x30},
	{6144000, 16000, 0x28},
	{6144000, 24000, 0x20},
	{6144000, 32000, 0x18},
	{6144000, 48000, 0x10},
};

struct ma1026_priv *globe_ma1026;
static struct snd_soc_component *ma1026_codec;

struct workqueue_struct *spk_monitor_wq;
struct delayed_work spk_delay_work;


static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count	= ARRAY_SIZE(rates_12288),
	.list	= rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count	= ARRAY_SIZE(rates_112896),
	.list	= rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};

static bool ma1026_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg <= 255)
		return true;
	else
		return false;
}

static bool ma1026_readable_reg(struct device *dev, unsigned int reg)
{
	if (reg <= 255)
		return true;
	else
		return false;
}

static const struct reg_default ma1026_reg_defaults[] = {
	{1, 0xF1}, {2, 0xDF}, {3, 0x3F}, {4, 0x60},
	{5, 0x77}, {6, 0x04}, {7, 0x00}, {8, 0x15},
	{9, 0x06}, {10, 0x00}, {11, 0x00}, {12, 0x00},
	{13, 0x00}, {14, 0x00}, {15, 0x00}, {16, 0x00},
	{17, 0x00}, {18, 0x02}, {19, 0x02}, {20, 0x00},
	{21, 0x00}, {22, 0x00}, {23, 0x7F}, {24, 0x03},
	{25, 0x00}, {26, 0x3F}, {27, 0xE1}, {28, 0x00},
	{29, 0x00}, {30, 0x08}, {31, 0x3F}, {32, 0x3F},
	{33, 0x00}, {34, 0x00}, {35, 0x00}, {36, 0x00},
	{37, 0x3F}, {38, 0x3F}, {94, 0x00},
};

static struct regmap_config ma1026_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MA1026_MAX_REGISTER,
	.reg_defaults = ma1026_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ma1026_reg_defaults),
	.volatile_reg = ma1026_volatile_reg,
	.readable_reg = ma1026_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int ma1026_set_bias_level(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec,
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5, 00, 0))
	struct snd_soc_component *codec,
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(6, 00, 0))
	struct snd_soc_component *codec,
#endif
	enum snd_soc_bias_level level)
{
	MA_DBG("ma1026" "set ma1026 bias level level = %d \n", level);
	if (level == SND_SOC_BIAS_ON) {
		snd_soc_component_update_bits(codec, MA1026_DMMCC,
					      MA1026_MCLKDIS, 0);
		snd_soc_component_update_bits(codec, MA1026_PWRCTRL1,
					      MA1026_PDN, 0);
	}
	return 0;
}

static DECLARE_TLV_DB_SCALE(micpga_tlv, 0, 100, 0);
static DECLARE_TLV_DB_SCALE(ipdig_tlv, -9600, 100, 0);
static DECLARE_TLV_DB_SCALE(adc_20db_tlv, 0, 2000, 0);

static const char * const ma1026_pgaa[] = {"Line 1", "Mic 1"};
static const char * const ma1026_pgab[] = {"Line 2", "Mic 2"};

static const struct soc_enum pgaa_enum =
	SOC_ENUM_SINGLE(MA1026_ADCIPC, 3, ARRAY_SIZE(ma1026_pgaa), ma1026_pgaa);

static const struct soc_enum pgab_enum =
	SOC_ENUM_SINGLE(MA1026_ADCIPC, 7, ARRAY_SIZE(ma1026_pgab), ma1026_pgab);

static const char * const ma1026_ip_swap[] = {
	"Stereo",
	"Mono 1",
	"Mono 2",
	"Swap 1/2"
};

static const struct soc_enum ip_swap_enum =
	SOC_ENUM_SINGLE(MA1026_MIOPC, 6,
			ARRAY_SIZE(ma1026_ip_swap), ma1026_ip_swap);

static const char * const ma1026_ng_delay[] = {
	"50ms",
	"100ms",
	"150ms",
	"200ms"
};

static const struct soc_enum ng_delay_enum =
	SOC_ENUM_SINGLE(MA1026_NGCAB, 0,
			ARRAY_SIZE(ma1026_ng_delay), ma1026_ng_delay);

static const struct snd_kcontrol_new pgaa_mux =
	SOC_DAPM_ENUM("Left Analog Input Capture Mux", pgaa_enum);

static const struct snd_kcontrol_new pgab_mux =
	SOC_DAPM_ENUM("Right Analog Input Capture Mux", pgab_enum);

static const struct snd_kcontrol_new input_left_mixer[] = {
	SOC_DAPM_SINGLE("ADC Left Input", MA1026_PWRCTRL1, 5, 1, 1),
	SOC_DAPM_SINGLE("DMIC Left Input", MA1026_PWRCTRL1, 4, 1, 1),
};

static const struct snd_kcontrol_new input_right_mixer[] = {
	SOC_DAPM_SINGLE("ADC Right Input", MA1026_PWRCTRL1, 7, 1, 1),
	SOC_DAPM_SINGLE("DMIC Right Input", MA1026_PWRCTRL1, 6, 1, 1),
};

static DECLARE_TLV_DB_SCALE(hpd_tlv, -10200, 50, 0);
static DECLARE_TLV_DB_SCALE(hpa_tlv, -5400, 200, 0);

static const unsigned int limiter_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 2, TLV_DB_SCALE_ITEM(-3000, 600, 0),
	3, 7, TLV_DB_SCALE_ITEM(-1200, 300, 0),
};

static const char * const ma1026_playback_path_mode[] = {
	"OPEN SPK",
	"CLOSE SPK",
	"OPEN HP A",
	"CLOSE HP A",
	"OPEN HP B",
	"CLOSE HP B",
	"OPEN HP",
	"CLOSE HP"
};

static const char * const ma1026_capture_path_mode[] = {
	"OPEN DIGITAL MIC",
	"CLOSE DIGITAL MIC",
	"OPEN MIC A",
	"CLOSE MIC A",
	"OPEN MIC B",
	"CLOSE MIC B"
};

static const char * const ma1026_mute_mode[] = {
	"MUTE OFF",
	"MUTE ON",
};

static SOC_ENUM_SINGLE_DECL(ma1026_playback_path_type,
	0, 0, ma1026_playback_path_mode);

static SOC_ENUM_SINGLE_DECL(ma1026_capture_path_type,
	0, 0, ma1026_capture_path_mode);

static SOC_ENUM_SINGLE_DECL(ma1026_mute_type,
	0, 0, ma1026_mute_mode);

static int ma1026_playback_path_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(component);

	MA_DBG("ma1026 %s : playback_path %ld\n",
	       __func__, ma1026->playback_path);

	ucontrol->value.integer.value[0] = ma1026->playback_path;

	return 0;
}

static int ma1026_playback_path_config(struct snd_soc_component *component,
				      long int pre_path, long int target_path)
{
	int reg = 0;
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(component);

	ma1026->playback_path = target_path;

	switch (ma1026->playback_path) {
	case OPEN_SPK_E:
	break;

	case CLOSE_SPK:
	break;

	case OPEN_HP_A:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPA, 1<<5);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL3,
				      MA1026_PDN_HP, 0);
	break;

	case CLOSE_HP_A:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPA, 0<<5);
		reg = snd_soc_read_ma1026(component, MA1026_MICBSVDDA);
		MA_DBG("ma1026" "%s : ma1026_MICBSVDDA = %x\n", __func__, reg);
		if ((reg&MA1026_MICBSVDDA_HPB) == 0)
			snd_soc_update_ma1026(component, MA1026_PWRCTRL3,
					      MA1026_PDN_HP, 1);
	break;

	case OPEN_HP_B:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPB, 1<<6);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL3,
				      MA1026_PDN_HP, 0);
	break;

	case CLOSE_HP_B:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPB, 0<<6);
		reg = snd_soc_read_ma1026(component, MA1026_MICBSVDDA);
		MA_DBG("ma1026" "%s : ma1026_MICBSVDDA = %x\n", __func__, reg);
		if ((reg&MA1026_MICBSVDDA_HPA) == 0)
			snd_soc_update_ma1026(component, MA1026_PWRCTRL3,
					      MA1026_PDN_HP, 1);
	break;

	case OPEN_HP:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPA, 1<<5);
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPB, 1<<6);
	break;

	case CLOSE_HP:
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPB, 0<<6);
		snd_soc_update_ma1026(component, MA1026_MICBSVDDA,
				      MA1026_MICBSVDDA_HPB, 0<<5);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL3,
				      MA1026_PDN_HP, 1);
	break;
	}

	return 0;
}

static int ma1026_playback_path_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(component);
	static int isSet;

	if ((ma1026->playback_path == ucontrol->value.integer.value[0]) &&
	    (isSet == 1)) {
		MA_DBG("ma1026" "%s : playback_path is not changed!\n",
		       __func__);
		return 0;
	}
	isSet = 1;
	return ma1026_playback_path_config(component, ma1026->playback_path,
					  ucontrol->value.integer.value[0]);
}

static int ma1026_capture_path_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026  = snd_soc_component_get_drvdata(component);

	MA_DBG("ma1026 %s : capture_path %ld\n", __func__,
	       ma1026->capture_path);

	ucontrol->value.integer.value[0] = ma1026->capture_path;
	return 0;
}

static int ma1026_capture_path_config(struct snd_soc_component *component,
				      long pre_path, long target_path)
{
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(component);

	ma1026->capture_path = target_path;

	switch (ma1026->capture_path) {
	case OPEN_DIGITAL_MIC:
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_DMICB, 0<<6);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_DMICB, 0<<4);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_ADCB, 1<<7);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_ADCA, 1<<5);
		break;

	case CLOSE_DIGITAL_MIC:
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_DMICB, 0<<6);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_DMICB, 0<<4);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_ADCB, 0<<7);
		snd_soc_update_ma1026(component, MA1026_PWRCTRL1,
				      MA1026_PDN_ADCA, 0<<5);
		break;

	case SELECT_MIC_A:
		snd_soc_update_ma1026(component, MA1026_ADCIPC, 
				      MA1026_ADCIPC_MICB_LINEB, 1<<7);
		break;

	case SELECT_LINE_A:
		snd_soc_update_ma1026(component, MA1026_ADCIPC,
				      MA1026_ADCIPC_MICB_LINEB, 0<<7);
		break;

	case SELECT_MIC_B:
		snd_soc_update_ma1026(component, MA1026_ADCIPC,
				      MA1026_ADCIPC_MICA_LINEA, 1<<3);
		break;

	case SELECT_LINE_B:
		snd_soc_update_ma1026(component, MA1026_ADCIPC,
				      MA1026_ADCIPC_MICA_LINEA, 0<<3);
		break;

	default:
		break;
	}

	return 0;
}

static int ma1026_capture_path_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = 
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026  = snd_soc_component_get_drvdata(component);
	static int isSet;

	if ((ma1026->capture_path == ucontrol->value.integer.value[0]) &&
	    (isSet == 1)) {
		MA_DBG("ma1026" "%s : capture_path is not changed!\n",
		       __func__);
		return 0;
	}
	isSet = 1;
	return ma1026_capture_path_config(component, ma1026->capture_path,
					  ucontrol->value.integer.value[0]);
}

static int ma1026_mute_path_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026  = snd_soc_component_get_drvdata(component);

	MA_DBG("ma1026 %s : force_muted %ld\n", __func__, ma1026->force_muted);

	ucontrol->value.integer.value[0] = ma1026->force_muted;
	return 0;
}

static int ma1026_muted_path_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct ma1026_priv *ma1026  = snd_soc_component_get_drvdata(component);

	if (!ucontrol->value.integer.value[0]) {
		ma1026->force_muted = 0;
		snd_soc_update_ma1026(component, MA1026_HPDC, 1<<1, 0<<1);
		snd_soc_update_ma1026(component, MA1026_HPDC, 1<<0, 0<<0);
	} else {
		ma1026->force_muted = 1;
		snd_soc_update_ma1026(component, MA1026_HPDC, 1<<1, 1<<1);
		snd_soc_update_ma1026(component, MA1026_HPDC, 1<<0, 1<<0);
	}

	return 0;
}

static const DECLARE_TLV_DB_SCALE(attn_tlv, -6300, 100, 1);

static const struct snd_kcontrol_new ma1026_snd_controls[] = {
	/* range 0-30 */
	SOC_DOUBLE_R_S_TLV("HP_A_PlaybackVolume",
			   MA1026_HPAAVOL, MA1026_HPBAVOL, 1,
			   -27, 3, 5, 0, hpa_tlv),

	SOC_DOUBLE_R("HP Analog Playback Mute", MA1026_HPAAVOL,
		     MA1026_HPBAVOL, 7, 1, 1),

	/* range 0-24 */
	SOC_DOUBLE_R_TLV("Input PGA Analog Volume",
			 MA1026_MICPGAAVOL, MA1026_MICPGABVOL, 0,
			 0x18, 0, micpga_tlv),

	/* range 0-108 */
	SOC_DOUBLE_R_S_TLV("Input Path Digital Volume",
			   MA1026_IPADVOL, MA1026_IPBDVOL, 0x00,
			   -96, 12, 7, 0, ipdig_tlv),

	SOC_DOUBLE_R("MIC Preamp Switch", MA1026_MICPGAAVOL,
		     MA1026_MICPGABVOL, 6, 1, 1),

	SOC_SINGLE_TLV("ADC A 20db",
		       MA1026_ADCIPC, 2,
		       0x01, 1, adc_20db_tlv),

	SOC_SINGLE_TLV("ADC B 20db",
		       MA1026_ADCIPC, 6,
		       0x01, 1, adc_20db_tlv),

	SOC_DOUBLE("Input Path Digital Mute", MA1026_ADCIPC, 0, 4, 1, 1),
	SOC_DOUBLE("HP Digital Playback left Mute", MA1026_HPDC, 0, 1, 1, 1),
	SOC_SINGLE("PGA Soft Ramp Switch", MA1026_MIOPC, 3, 1, 0),
	SOC_SINGLE("Analog 0 Cross Switch", MA1026_MIOPC, 2, 1, 0),
	SOC_SINGLE("Digital Soft Ramp Switch", MA1026_MIOPC, 1, 1, 0),
	SOC_SINGLE("Analog Output Soft Ramp Switch", MA1026_MIOPC, 0, 1, 0),

	SOC_DOUBLE("ADC Signal Polarity Switch", MA1026_ADCIPC, 1, 5, 1, 0),

	SOC_SINGLE("HP Limiter Attack Rate", MA1026_LIMATTRATEHP, 0, 0x3F, 0),
	SOC_SINGLE("HP Limiter Release Rate", MA1026_LIMRLSRATEHP, 0, 0x3F, 0),
	SOC_SINGLE("HP Limiter Switch", MA1026_LIMRLSRATEHP, 7, 1, 0),
	SOC_SINGLE("HP Limiter All Channels Switch",
		   MA1026_LIMRLSRATEHP, 6, 1, 0),

	SOC_SINGLE_TLV("HP Limiter Max Threshold Vol", MA1026_LMAXMINHP,
		       5, 7, 1, limiter_tlv),
	SOC_SINGLE_TLV("HP Limiter Min Threshold Vol", MA1026_LMAXMINHP,
		       2, 7, 1, limiter_tlv),

	SOC_SINGLE("ALC Attack Rate Volume", MA1026_ALCARATE, 0, 0x3F, 0),
	SOC_SINGLE("ALC Release Rate Volume", MA1026_ALCRRATE, 0, 0x3F, 0),
	SOC_DOUBLE("ALC Switch", MA1026_ALCARATE, 6, 7, 1, 0),

	SOC_SINGLE_TLV("ALC Max Threshold Volume", MA1026_ALCMINMAX,
		       5, 7, 0, limiter_tlv),
	SOC_SINGLE_TLV("ALC Min Threshold Volume", MA1026_ALCMINMAX,
		       2, 7, 0, limiter_tlv),

	SOC_DOUBLE("NG Enable Switch", MA1026_NGCAB, 6, 7, 1, 0),
	SOC_SINGLE("NG Boost Switch", MA1026_NGCAB, 5, 1, 0),
	SOC_SINGLE("NG Threshold", MA1026_NGCAB, 2, 7, 0),
    SOC_SINGLE("HP Amp CTRL", MA1026_PWRCTRL3, 0, 1, 0),

	SOC_ENUM("NG Delay", ng_delay_enum),

	SOC_DOUBLE_R_TLV("IP2ASP Volume", MA1026_IPA2ASPA, MA1026_IPB2ASPB,
			 0, 0x3F, 1, attn_tlv),
	SOC_DOUBLE_R_TLV("ASP2ASP Volume", MA1026_ASPA2ASPA, MA1026_ASPB2ASPB,
			 0, 0x3F, 1, attn_tlv),
	SOC_DOUBLE_R_TLV("IP2HP Volume", MA1026_IPA2HPA, MA1026_IPB2HPB,
			 0, 0x3F, 1, attn_tlv),
	SOC_DOUBLE_R_TLV("ASP2HP Volume", MA1026_ASPA2HPA, MA1026_ASPB2HPB,
			 0, 0x3F, 1, attn_tlv),

	SOC_ENUM("IP Digital Swap/Mono Select", ip_swap_enum),
	SOC_ENUM_EXT("Playback Opt", ma1026_playback_path_type,
		     ma1026_playback_path_get, ma1026_playback_path_put),
	SOC_ENUM_EXT("Capture MIC Opt", ma1026_capture_path_type,
		     ma1026_capture_path_get, ma1026_capture_path_put),
	SOC_ENUM_EXT("Mute", ma1026_mute_type,
		     ma1026_mute_path_get, ma1026_muted_path_put),
};

static int ma1026_init_regs(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec
#endif
)
{
	MA_DBG("ma1026" " init all regs...\n");
#if TEMP_DEBUG
    snd_soc_write_ma1026(codec, 0x00, 0x68);
    snd_soc_write_ma1026(codec, ma1026_PWRCTRL1, 0x00);
    snd_soc_update_ma1026(codec, ma1026_PWRCTRL1, ma1026_PDN_LDO, 1<<3);
    snd_soc_update_ma1026(codec, ma1026_PWRCTRL1, ma1026_PDN, 0);

	/* Set the main clk division */
    snd_soc_update_ma1026(codec, ma1026_DMMCC, ma1026_FREQ_DIV, 2<<1);
    snd_soc_update_ma1026(codec, ma1026_DMMCC, ma1026_MCLKDIS, 0);

	/* Open I2S input/output */
    snd_soc_update_ma1026(codec, ma1026_PWRCTRL2, ma1026_PDN_ASP_SDO, 0);
    snd_soc_update_ma1026(codec, ma1026_PWRCTRL2, ma1026_PDN_ASP_SDI, 0);
#endif
	snd_soc_write_ma1026(codec, 0x00, 0x68);
	snd_soc_write_ma1026(codec, 0x0A, 0x88);
	snd_soc_write_ma1026(codec, 0x01, 0x00);
	snd_soc_write_ma1026(codec, 0x01, 0x08);
	snd_soc_write_ma1026(codec, 0x05, 0x65);
	snd_soc_write_ma1026(codec, 0x02, 0x1F);
	snd_soc_write_ma1026(codec, 0x06, 0x04);
	snd_soc_write_ma1026(codec, 0x02, 0x13);
	snd_soc_write_ma1026(codec, 0x12, 0x02);
	snd_soc_write_ma1026(codec, 0x13, 0x02);

	snd_soc_write_ma1026(codec, 0x21, 0x00);
	snd_soc_write_ma1026(codec, 0x22, 0x00);
	snd_soc_write_ma1026(codec, 0x23, 0x00);
	snd_soc_write_ma1026(codec, 0x24, 0x00);
	snd_soc_write_ma1026(codec, 0x03, 0x1c);

	snd_soc_write_ma1026(codec, 0x2c, 0xc1);

	return 0;
}

static int ma1026_probe(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec
#endif
)
{
	int ret = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct ma1026_priv *ma1026 = snd_soc_codec_get_drvdata(codec);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(codec);
#endif
	ma1026_codec = codec;

	MA_DBG("ma1026" " alsa start ....\n");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	codec->control_data = ma1026->regmap;
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	snd_soc_component_init_regmap(codec, ma1026->regmap);
#endif

	ma1026_set_bias_level(codec, SND_SOC_BIAS_ON);

	ma1026_init_regs(codec);

	ma1026->mclk = devm_clk_get(codec->dev, "mclk");
	if (PTR_ERR(ma1026->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	MA_DBG("ma1026" "alsa driver finish.... \n");
	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
static int ma1026_remove(
	struct snd_soc_codec *codec
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
static void ma1026_remove(
	struct snd_soc_component *codec
#endif
)
{
	ma1026_set_bias_level(codec, SND_SOC_BIAS_OFF);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	return 0;
#endif
}

static int ma1026_suspend(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
			struct snd_soc_codec *codec
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
			struct snd_soc_component *codec
#endif
	)
{
	MA_DBG("ma1026" " suspend ...\n");
	ma1026_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int ma1026_resume(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
			struct snd_soc_codec *codec
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
			struct snd_soc_component *codec
#endif
	)
{
	MA_DBG("ma1026" " resume ...\n");
	ma1026_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static const struct snd_soc_dapm_widget dit_widgets[] = {
	SND_SOC_DAPM_OUTPUT("HPOUTA"),
	SND_SOC_DAPM_OUTPUT("HPOUTB"),

	SND_SOC_DAPM_INPUT("LINEIN1"),
	SND_SOC_DAPM_INPUT("LINEIN2"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),

	SND_SOC_DAPM_DAC("HP Amp", NULL, MA1026_PWRCTRL3, 0, 0),

	SND_SOC_DAPM_MIXER_NAMED_CTL("Input Left Capture", SND_SOC_NOPM,
			 0, 0, input_left_mixer, ARRAY_SIZE(input_left_mixer)),
	SND_SOC_DAPM_MIXER_NAMED_CTL("Input Right Capture", SND_SOC_NOPM,
			0, 0, input_right_mixer, ARRAY_SIZE(input_right_mixer)),

	SND_SOC_DAPM_ADC("DMIC Left", NULL, MA1026_PWRCTRL1, 6, 0),
	SND_SOC_DAPM_ADC("DMIC Right", NULL, MA1026_PWRCTRL1, 4, 0),
};

static const struct snd_soc_dapm_route dit_routes[] = {
	{"HPOUTA", NULL, "ASP Playback"},
	{"HPOUTA", NULL, "ASP Playback"},
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
static struct snd_soc_codec_driver soc_codec_dev_ma1026 = {
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
static struct snd_soc_component_driver soc_codec_dev_ma1026 = {
#endif
	.probe = ma1026_probe,
	.remove = ma1026_remove,
	.suspend = ma1026_suspend,
	.resume = ma1026_resume,
	.set_bias_level = ma1026_set_bias_level,
	.dapm_widgets		= dit_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(dit_widgets),
	.dapm_routes		= dit_routes,
	.num_dapm_routes	= ARRAY_SIZE(dit_routes),
	.controls = ma1026_snd_controls,
	.num_controls = ARRAY_SIZE(ma1026_snd_controls),
};

static int ma1026_pcm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	if (playback)
		msleep(50);

	return 0;
}


static int ma1026_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec = dai->codec;
	struct ma1026_priv *priv = snd_soc_codec_get_drvdata(codec);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec = dai->component;
	struct ma1026_priv *priv = snd_soc_component_get_drvdata(codec);
#endif
	int lrckfreq = params_rate(params);
	int i, index = -1;
	if (priv->config.mmcc & MA1026_MS_MASTER) {

		for (i = 0; i < ARRAY_SIZE(ma1026_mclk_coeffs); i++) {
			if (ma1026_mclk_coeffs[i].lrckfreq == lrckfreq)
				index = i;
		}

		if (index < 0) {
			MA_DBG("ma1026" " ma1026_pcm_hw_params, mclk = %d\n", index);
			return -EINVAL;
		}

		dev_dbg(codec->dev, "DAI[ ]: lrckfreq %u, MMCC[5:0] = %x\n",
			lrckfreq, ma1026_mclk_coeffs[index].mmcc);

		priv->config.mmcc &= 0xC0;
		priv->config.mmcc |= ma1026_mclk_coeffs[index].mmcc;
		priv->config.spc &= 0xFC;
	} else {
		priv->config.spc &= 0xFC;
		priv->config.spc |= MA1026_MCK_SCLK_64FS;
	}
	priv->config.lrckfreq = lrckfreq;

	snd_soc_write_ma1026(codec, MA1026_ASPMMCC, priv->config.mmcc);

	return 0;
}

static int ma1026_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ma1026_priv *priv = snd_soc_codec_get_drvdata(codec);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec = codec_dai->component;
	struct ma1026_priv *priv = snd_soc_component_get_drvdata(codec);
#endif
	u8 spc, mmcc;

	spc = snd_soc_read_ma1026(codec, MA1026_ASPC);
	mmcc = snd_soc_read_ma1026(codec, MA1026_ASPMMCC);

	if (fmt == SND_SOC_DAIFMT_CBM_CFM)
		mmcc |= MA1026_MS_MASTER;
	else
		mmcc &= ~MA1026_MS_MASTER;

	if (fmt == SND_SOC_DAIFMT_I2S)
		spc &= ~MA1026_SPDIF_PCM;
	else
		spc |= MA1026_SPDIF_PCM;

	if (spc & MA1026_SPDIF_PCM)
		spc &= ~MA1026_PCM_BIT_ORDER;

	priv->config.spc = spc;
	priv->config.mmcc = mmcc;

	return 0;
}

static int ma1026_set_dai_sysclk(struct snd_soc_dai *codec_dai,
			      int clk_id, unsigned int freq, int dir)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ma1026_priv *ma1026 = snd_soc_codec_get_drvdata(codec);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec = codec_dai->component;
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(codec);
#endif
	MA_DBG("ma1026" " ma1026_set_dai_sysclk, freq = %d\n", freq);

	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		ma1026->sysclk_constraints = &constraints_112896;
		ma1026->sysclk = freq;
		return 0;
	case 12288000:
	case 19200000:
	case 16934400:
	case 24576000:
	case 33868800:
		ma1026->sysclk_constraints = &constraints_12288;
		ma1026->sysclk = freq;
		return 0;
	case 12000000:
		ma1026->sysclk_constraints = &constraints_12;
		ma1026->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int ma1026_mute(struct snd_soc_dai *dai, int mute
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 00, 0))
	, int stream
#endif
)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 00, 0))
	struct snd_soc_codec *codec = dai->codec;
	struct ma1026_priv *ma1026 = snd_soc_codec_get_drvdata(codec);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 00, 0))
	struct snd_soc_component *codec = dai->component;
	struct ma1026_priv *ma1026 = snd_soc_component_get_drvdata(codec);
#endif

	MA_DBG("ma1026" " ma1026_mute, mute = %d\n", mute);
	if (ma1026->force_muted)
		return 0;

	if (mute) {
		snd_soc_update_ma1026(codec, MA1026_HPDC, 1<<1, 1<<1);
		snd_soc_update_ma1026(codec, MA1026_HPDC, 1<<0, 1<<0);
	} else {
		snd_soc_update_ma1026(codec, MA1026_HPDC, 1<<1, 0<<1);
		snd_soc_update_ma1026(codec, MA1026_HPDC, 1<<0, 0<<0);
	}
	return 0;
}

#define ma1026_pcm_shutdown NULL

static struct snd_soc_dai_ops ma1026_ops = {
	.startup = ma1026_pcm_startup,
	.hw_params = ma1026_pcm_hw_params,
	.set_fmt = ma1026_set_dai_fmt,
	.set_sysclk = ma1026_set_dai_sysclk,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 00, 0))
	.digital_mute = ma1026_mute,
#else
	.mute_stream = ma1026_mute,
#endif
};

static struct snd_soc_dai_driver ma1026_dai = {
	.name = "ma1026-asp",
	.playback = {
		.stream_name = "ASP Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MA1026_LRCK,
		.formats = MA1026_FORMATS,
	},
	.capture = {
		.stream_name = "ASP Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = MA1026_LRCK,
		.formats = MA1026_FORMATS,
	},
	.ops = &ma1026_ops,
	.symmetric_rates = 1,
};

static int ma1026_i2c_probe(struct i2c_client *i2c_client,
			     const struct i2c_device_id *id)
{
	struct ma1026_priv *ma1026;
	int ret;

	MA_DBG("ma1026" " run driver .... \n");

	ma1026 = devm_kzalloc(&i2c_client->dev, sizeof(struct ma1026_priv),
			      GFP_KERNEL);
	if (!ma1026) {
		MA_DBG("ma1026 " "could not allocate codec\n");
		return -ENOMEM;
	}
	memset(ma1026, 0x00, sizeof(struct ma1026_priv));
	ma1026->regmap = devm_regmap_init_i2c(i2c_client, &ma1026_regmap);
	if (IS_ERR(ma1026->regmap)) {
		ret = PTR_ERR(ma1026->regmap);
		MA_DBG("ma1026 ""regmap_init() failed: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c_client, ma1026);

	globe_ma1026 = ma1026;

	ma1026->hp_mute = 1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	ret =  snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_dev_ma1026, &ma1026_dai, 1);
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(4, 19, 0))
	ret = devm_snd_soc_register_component(&i2c_client->dev,
			&soc_codec_dev_ma1026, &ma1026_dai, 1);
#endif

	if (ret < 0) {
		MA_DBG("ma1026 ""%s() register codec error %d\n",
		       __func__, ret);
		return ret;
	}

	MA_DBG("ma1026" " load end ret=%d\n", ret);

	return 0;
}

static int ma1026_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

static void ma1026_i2c_shutdown(struct i2c_client *client)
{
	if (ma1026_codec != NULL) {
		msleep(20);
		ma1026_set_bias_level(ma1026_codec, SND_SOC_BIAS_OFF);
	}
}


static const struct of_device_id ma1026_of_match[] = {
	{.compatible = "Phytium, ma1026", },
	{}
};

MODULE_DEVICE_TABLE(of, ma1026_of_match);

static const struct i2c_device_id ma1026_id[] = {
	{"ma1026", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ma1026_id);

static struct i2c_driver ma1026_i2c_driver = {
	.driver = {
		.name = "ma1026",
		.of_match_table = ma1026_of_match,
	},
	.probe = ma1026_i2c_probe,
	.remove = ma1026_i2c_remove,
	.shutdown = ma1026_i2c_shutdown,
	.id_table = ma1026_id,
};
module_i2c_driver(ma1026_i2c_driver);

MODULE_DESCRIPTION("Cubiclattice ma1026 driver");
MODULE_AUTHOR("Cubiclattice Ltd");
MODULE_LICENSE("GPL");

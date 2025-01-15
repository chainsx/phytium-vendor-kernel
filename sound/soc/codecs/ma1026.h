#ifndef __MA1026_H__
#define __MA1026_H__

/* I2C Registers */
#define MA1026_PWRCTRL1		0x01    /* Power Ctrl 1. */
#define MA1026_PWRCTRL2		0x02    /* Power Ctrl 2. */
#define MA1026_PWRCTRL3		0x03    /* Power Ctrl 3. */
#define MA1026_CPFCHC		0x04    /* Charge Pump Freq. Class H Ctrl. */
#define MA1026_MICBSVDDA	0x05    /* MIC Bias & VDDA Voltage*/
#define MA1026_DMMCC		0x06    /* Digital MIC & Master Clock Ctrl. */
#define MA1026_ASPC		0x07    /* Audio Serial Port (ASP) Control. */
#define MA1026_ASPMMCC		0x08    /* ASP Master Mode Control. */
#define MA1026_MIOPC		0x09    /* Input & Output Path Control. */
#define MA1026_ADCIPC		0x0A	/* ADC/IP Control. */
#define MA1026_MICPGAAVOL	0x0B	/* MIC 1 PreAmp, PGAA Vol. */
#define MA1026_MICPGABVOL	0x0C	/* MIC 2 PreAmp, PGAB Vol. */
#define MA1026_IPADVOL		0x0D	/* Input Path A Digital Volume. */
#define MA1026_IPBDVOL		0x0E	/* Input Path B Digital Volume. */
#define MA1026_HPDC		0x0F	/* HP Digital Control. */
#define MA1026_HPADVOL		0x10	/* HP A Digital Vol. */
#define MA1026_HPBDVOL		0x11	/* HP B Digital Vol. */
#define MA1026_HPAAVOL		0x12	/* HP A Analog Volume. */
#define MA1026_HPBAVOL		0x13	/* HP B Analog Volume. */
#define MA1026_ANAINVOL		0x14	/* Analog Input Path Advisory Vol. */
#define MA1026_ASPINVOL		0x15	/* ASP Input Path Advisory Vol. */
#define MA1026_LIMATTRATEHP	0x16	/* Limit Attack Rate HP. */
#define MA1026_LIMRLSRATEHP	0x17	/* Limit Release Rate HP. */
#define MA1026_LMAXMINHP	0x18	/* Limit Thresholds HP. */
#define MA1026_ALCARATE		0x19	/* ALC Enable, Attack Rate AB. */
#define MA1026_ALCRRATE		0x1A	/* ALC Release Rate AB.  */
#define MA1026_ALCMINMAX	0x1B	/* ALC Thresholds AB. */
#define MA1026_NGCAB		0x1C	/* Noise Gate Ctrl AB. */
#define MA1026_ALCNG		0x1D	/* ALC & Noise Gate Ctrl. */
#define MA1026_MIXERCTL		0x1E	/* Mixer Control. */
#define MA1026_IPA2HPA		0x1F	/* HP Left Mixer */
#define MA1026_IPB2HPB		0x20	/* HP Right Mixer  */
#define MA1026_ASPA2HPA		0x21	/* HP Left Mixer */
#define MA1026_ASPB2HPB		0x22	/* HP Right Mixer */
#define MA1026_IPA2ASPA		0x23	/* ASP Left Mixer */
#define MA1026_IPB2ASPB		0x24	/* ASP Right Mixer */
#define MA1026_ASPA2ASPA	0x25	/* ASP Left Mixer */
#define MA1026_ASPB2ASPB	0x26	/* ASP Right Mixer */
#define MA1026_IM		0x5E	/* Interrupt Mask  */
#define MA1026_IS		0x60	/* Interrupt Status */
#define MA1026_MAX_REGISTER	0xff	/* Total Registers */
/* Bitfield Definitions */

/* MA1026_PWRCTRL1 */
#define MA1026_PDN_ADCB		    (1 << 7)
#define MA1026_PDN_DMICB		(1 << 6)
#define MA1026_PDN_ADCA		    (1 << 5)
#define MA1026_PDN_DMICA		(1 << 4)
#define MA1026_PDN_LDO			(1 << 3)
#define MA1026_PDN			    (1 << 0)

/* MA1026_PWRCTRL2 */
#define MA1026_PDN_MICBS2		(1 << 7)
#define MA1026_PDN_MICBS1		(1 << 6)
#define MA1026_PDN_ASP_SDO		(1 << 3)
#define MA1026_PDN_ASP_SDI	    (1 << 2)

/* MA1026_PWRCTRL3 */
#define MA1026_PDN_HP			(1 << 0)

#define MA1026_CP_MASK	(0xF0)

/* MA1026_ASPC */
#define	MA1026_SP_3ST			(1 << 7)
#define MA1026_SPDIF_PCM		(1 << 6)
#define MA1026_PCM_BIT_ORDER		(1 << 3)
#define MA1026_MCK_SCLK_64FS		(0 << 0)
#define MA1026_MCK_SCLK_MCLK		(2 << 0)
#define MA1026_MCK_SCLK_PREMCLK		(3 << 0)

/* MA1026_ASPMMCC */
#define MA1026_MS_MASTER		(1 << 7)


/* MA1026_DMMCC */
#define MA1026_MCLKDIS			(1 << 0)
#define MA1026_FREQ_DIV			(2 << 1)
/* IS1, IM1 */
#define MA1026_MIC_SDET		(1 << 6)

/* Analog Softramp */
#define MA1026_ANLGOSFT		(1 << 0)

/* MA1026 MCLK 0 from EXT CLK; 1 from PLL OUTPUT*/
#define MA1026_CLKID  		0

/*MA1026_MICBSVDDA  */
#define MA1026_MICBSVDDA_HPA (1<<5)
#define MA1026_MICBSVDDA_HPB (1<<6)

/*MA1026_ADCIPC */
#define MA1026_ADCIPC_MICB_LINEB (1<<7)
#define MA1026_ADCIPC_MICA_LINEA (1<<3)

enum output
{
    OPEN_SPK_E,
    CLOSE_SPK,

    OPEN_HP_A,
    CLOSE_HP_A,

    OPEN_HP_B,
    CLOSE_HP_B,

    OPEN_HP,
    CLOSE_HP,
} ;
enum input
{
    OPEN_DIGITAL_MIC,
    CLOSE_DIGITAL_MIC,

    SELECT_MIC_A,    
    SELECT_LINE_A,

    SELECT_MIC_B,
    SELECT_LINE_B,

};


#endif	/* __MA1026_H__ */

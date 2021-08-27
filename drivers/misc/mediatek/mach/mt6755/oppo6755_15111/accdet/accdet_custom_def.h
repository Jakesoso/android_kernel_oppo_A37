/*Headset mode MIC define*/
typedef enum
{
	ACCDET_MIC_MODE_ACC = 1,
	ACCDET_MIC_MODE_LOW_COST_WITHOUT_IN_BIAS = 2,
	ACCDET_MIC_MODE_LOW_COST_WITH_IN_BIAS = 6,
} ACCDET_MIC_MODE;
/*Use ACC Mode Temporarily*/
#define ACCDET_MIC_MODE	(1)

/*use accdet + EINT solution*/
/*#define ACCDET_EINT*/   /*ACC mode*/
#ifndef ACCDET_EINT
#define ACCDET_EINT_IRQ  /*DCC mode*/
#endif
/*#define ACCDET_PIN_SWAP*/
/*#define ACCDET_PIN_RECOGNIZATION*/
#if defined(CONFIG_MTK_LEGACY)
#define ACCDET_HIGH_VOL_MODE
#ifdef ACCDET_HIGH_VOL_MODE
/*yongzhi.zhang@Multimedia, 2016/01/30, modify for adjusting micbias vol for APPLE CTIA Headset*/
#define ACCDET_MIC_VOL 7     /*2.7v*/
#else
#define ACCDET_MIC_VOL 2     /*1.9v*/
#endif
#endif
#define ACCDET_SHORT_PLUGOUT_DEBOUNCE
#define ACCDET_SHORT_PLUGOUT_DEBOUNCE_CN 20
/*#define FOUR_KEY_HEADSET*/



/******************************************************************************
*                                                                             *
*                            Module specification                             *
*                                                                             *
*                                Copyright by                                 *
*                                                                             *
*                              Azoteq (Pty) Ltd                               *
*                          Republic of South Africa                           *
*                                                                             *
*                           Tel: +27(0)21 863 0033                            *
*                          E-mail: info@azoteq.com                            *
*                                                                             *
*******************************************************************************
Name             :  IQS316.h
Description      :  Header file of IQS316 memory map addresses
*******************************************************************************/

#define PROD_NUM		0x00
#define VERSION_NUM		0x01

#define UI_FLAGS0		0x10

#define PROX_STAT		0x31
#define TOUCH_STAT 		0x35
#define HALT_STAT		0x39
#define GROUP_NUM		0x3D

#define CUR_SAM_04_HI		0x42
#define CUR_SAM_04_LO		0x43
#define CUR_SAM_15_HI		0x44
#define CUR_SAM_15_LO		0x45
#define CUR_SAM_26_HI		0x46
#define CUR_SAM_26_LO		0x47
#define CUR_SAM_37_HI		0x48
#define CUR_SAM_37_LO		0x49

#define LTA_04_HI		0x83
#define LTA_04_LO		0x84
#define LTA_15_HI		0x85
#define LTA_15_LO		0x86
#define LTA_26_HI		0x87
#define LTA_26_LO		0x88
#define LTA_37_HI		0x89
#define LTA_37_LO		0x8A

#define UI_SETTINGS0 		0xC4
#define POWER_SETTINGS		0xC5
#define PROX_SETTINGS_1		0xC6
#define PROX_SETTINGS_2		0xC7
#define ATI_MULT1		0xC8
#define ATI_MULT2		0xC9
#define ATI_C0			0xCA
#define ATI_C1			0xCB
#define ATI_C2			0xCC
#define ATI_C3			0xCD
#define SHLD_SETTINGS		0xCE
#define INT_CAL_SETTINGS	0xCF
#define PM_CX_SELECT		0xD0
#define DEFAULT_COMMS_PTR	0xD1
#define CHAN_ACTIVE0		0xD2
#define CHAN_ACTIVE1		0xD3
#define CHAN_ACTIVE2		0xD4
#define CHAN_ACTIVE3		0xD5
#define CHAN_ACTIVE4		0xD6
#define CHAN_RESEED0		0xD7
#define CHAN_RESEED1		0xD8
#define CHAN_RESEED2		0xD9
#define CHAN_RESEED3		0xDA
#define CHAN_RESEED4		0xDB
#define AUTO_ATI_TARGET_HI	0xDC
#define AUTO_ATI_TARGET_LO	0xDD

#define DIRECT_ADDR_RW		0xFC
#define DIRECT_DATA_RW		0xFD


// BIT DEFINITIONS

// UI_FLAGS0
#define SHOW_RESET		0x80
#define MODE_INDICATOR		0x40
// unused			0x20
// unused			0x10
// unused			0x08
#define ATI_BUSY		0x04
#define RESEED_BUSY		0x02
#define NOISE			0x01

// TOUCH THRESHOLDS
// with touch LOW range selected
#define TOUCH_THRES_1_32        0x00
#define TOUCH_THRES_1_16        0x40
#define TOUCH_THRES_2_16        0x80
#define TOUCH_THRES_3_16        0xC0
// with touch HIGH range selected
#define TOUCH_THRES_4_16        0x00
#define TOUCH_THRES_6_16        0x40
#define TOUCH_THRES_8_16        0x80
#define TOUCH_THRES_10_16       0xC0

// PROX THRESHOLDS
// with prox LOW range selected
#define PROX_THRES_2            0x00
#define PROX_THRES_3            0x10
#define PROX_THRES_4            0x20
#define PROX_THRES_6            0x30
// with prox HIGH range selected
#define PROX_THRES_8            0x00
#define PROX_THRES_16           0x10
#define PROX_THRES_20           0x20
#define PROX_THRES_30           0x30

// UI_SETTINGS0
#define RESEED			0x80
#define ATI_MODE		0x40
#define PROX_THRES_RANGE	0x20
#define TOUCH_THRES_RANGE	0x10
#define FORCE_PROX_THRES_MODE	0x08
#define FORCE_TOUCH_THRES_MODE	0x04
#define ND			0x02
// unused			0x01

// POWER_SETTINGS
// unused			0x80
// unused			0x40
// unused			0x20
// unused			0x10
#define SLEEP			0x08
#define MAIN_OSC		0x04
#define LP1			0x02
#define LP0			0x01

// PROX_THRES_SETTINGS_1
#define CXVSS			0x80
#define ZC_EN			0x40
#define HALT1			0x20
#define HALT0			0x10
#define AUTO_ATI		0x08
#define CXDIV2			0x04
#define CXDIV1			0x02
#define CXDIV0			0x01

// PROX_THRES_SETTINGS_2
// unused			0x80
#define SHIELD_EN		0x40
#define STOP_COMMS		0x20
#define ACK_RESET		0x10
#define SKIP_CONV		0x08
#define ACF_DISABLE		0x04
#define LTN_DISABLE		0x02
#define WDT_DISABLE		0x01

// CX_CONFIG
#define CX_GPIO_1		0x80
#define CX_GPIO_0		0x40
// unused			0x20
// unused			0x10
#define GROUP4			0x08
#define GROUP3			0x04
#define GROUP2			0x02
#define GROUP1			0x01

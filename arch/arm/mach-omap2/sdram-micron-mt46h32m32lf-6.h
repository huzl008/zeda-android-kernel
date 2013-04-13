/*
 * SDRC register values for the Micron MT46H32M32LF-6
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H32M32LF
#define ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46H32M32LF

// Jack_20111118: Copy micro's memory settings @ 200MHz from <ROWBOAT>/x-loader/include/asm/arch-omap3/mem.h
/* optimized timings good for current shipping parts */
#define SDP_3430_SDRC_RFR_CTRL_100MHz   0x0002da01
#define SDP_3430_SDRC_RFR_CTRL_133MHz   0x0003de01 /* 7.8us/7.5ns - 50=0x3de */
#define SDP_3430_SDRC_RFR_CTRL_165MHz   0x0004e201 /* 7.8us/6ns - 50=0x4e2 */
#define SDP_3430_SDRC_RFR_CTRL_200MHz   0x0005e601 /* 7.8us/5ns - 50=0x5e6 */
#define MICRON_TDAL_200   6
#define MICRON_TDPL_200   3
#define MICRON_TRRD_200   2
#define MICRON_TRCD_200   3
#define MICRON_TRP_200    3
#define MICRON_TRAS_200   8
#define MICRON_TRC_200   11
#define MICRON_TRFC_200  15
#define MICRON_V_ACTIMA_200 ((MICRON_TRFC_200 << 27) | (MICRON_TRC_200 << 22) | (MICRON_TRAS_200 << 18) \
		| (MICRON_TRP_200 << 15) | (MICRON_TRCD_200 << 12) |(MICRON_TRRD_200 << 9) | \
		(MICRON_TDPL_200 << 6) | (MICRON_TDAL_200))
#define MICRON_TWTR_200   2
#define MICRON_TCKE_200   1
#define MICRON_TXP_200    2
#define MICRON_XSR_200   23
#define MICRON_V_ACTIMB_200 ((MICRON_TCKE_200 << 12) | (MICRON_XSR_200 << 0)) | \
				(MICRON_TXP_200 << 8) | (MICRON_TWTR_200 << 16)

#include <plat/sdrc.h>

/* Micron MT46H32M32LF-6 */
/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params mt46h32m32lf6_sdrc_params[] = {
	[0] = {
		.rate	     = 166000000,
		.actim_ctrla = 0x9a9db4c6,
		.actim_ctrlb = 0x00011217,
		.rfr_ctrl    = 0x0004dc01,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 165941176,
		.actim_ctrla = 0x9a9db4c6,
		.actim_ctrlb = 0x00011217,
		.rfr_ctrl    = 0x0004dc01,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 83000000,
		.actim_ctrla = 0x51512283,
		.actim_ctrlb = 0x0001120c,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate	     = 82970588,
		.actim_ctrla = 0x51512283,
		.actim_ctrlb = 0x0001120c,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	/* Added 200MHz's setting. Need to fully test */
	[4] = {
		.rate	     = 200000000,
		.actim_ctrla = MICRON_V_ACTIMA_200,
		.actim_ctrlb = MICRON_V_ACTIMB_200,
		.rfr_ctrl    = SDP_3430_SDRC_RFR_CTRL_200MHz,
		.mr	     = 0x00000032,
	},
	[5] = {
		.rate	     = 0
	},
};

#endif

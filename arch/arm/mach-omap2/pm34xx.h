/*
 * OMAP2+ MPU PM_34XX-specific function prototypes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ARCH_ARM_MACH_OMAP3_PM_34XX_H
#define __ARCH_ARM_MACH_OMAP3_PM_34XX_H

extern void omap3_pm_prm_voltctrl_set(int signal_flag,int ret_flag,int off_lag);
extern void omap3_pm_prm_polctrl_set(int pol_clkreq,int pol_offmode);

#endif

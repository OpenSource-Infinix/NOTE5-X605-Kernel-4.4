/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef MTK_POWER_GS_ARRAY_H
#define MTK_POWER_GS_ARRAY_H

extern void mt_power_gs_sp_dump(void);
extern unsigned int golden_read_reg(unsigned int addr);

extern const unsigned int *AP_PMIC_REG_gs_suspend;
extern unsigned int AP_PMIC_REG_gs_suspend_len;

extern const unsigned int *AP_PMIC_REG_gs_deepidle___lp_mp3_32kless;
extern unsigned int AP_PMIC_REG_gs_deepidle___lp_mp3_32kless_len;

extern const unsigned int *AP_PMIC_REG_gs_sodi3p0;
extern unsigned int AP_PMIC_REG_gs_sodi3p0_len;

extern const unsigned int *AP_PMIC_REG_gs_deepidle___lp_mp3;
extern unsigned int AP_PMIC_REG_gs_deepidle___lp_mp3_len;

extern const unsigned int *AP_PMIC_REG_gs_w_key;
extern unsigned int AP_PMIC_REG_gs_w_key_len;

extern const unsigned int *AP_PMIC_REG_gs_sodi3p0_32kless;
extern unsigned int AP_PMIC_REG_gs_sodi3p0_32kless_len;

extern const unsigned int *AP_PMIC_REG_gs_suspend_32kless;
extern unsigned int AP_PMIC_REG_gs_suspend_32kless_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_dpidle;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_dpidle_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_suspend;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_suspend_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_vp_mjc;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_vp_mjc_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_topck_name;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_topck_name_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_paging;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_paging_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_mp3_play;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_mp3_play_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_clkon;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_clkon_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_dcm_off;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_dcm_off_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_vr;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_vr_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_vp;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_vp_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_access;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_access_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_pdn_ao;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_pdn_ao_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_idle;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_idle_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_talk;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_talk_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_connsys;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_connsys_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_sodi;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_sodi_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_datalink;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_datalink_len;

extern const unsigned int *AP_CG_Golden_Setting_tcl_gs_flight;
extern unsigned int AP_CG_Golden_Setting_tcl_gs_flight_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_dpidle;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_dpidle_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_suspend;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_suspend_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_vp_mjc;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_vp_mjc_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_topck_name;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_topck_name_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_paging;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_paging_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_mp3_play;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_mp3_play_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_clkon;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_clkon_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_dcm_off;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_dcm_off_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_vr;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_vr_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_vp;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_vp_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_access;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_access_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_pdn_ao;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_pdn_ao_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_idle;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_idle_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_talk;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_talk_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_connsys;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_connsys_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_sodi;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_sodi_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_datalink;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_datalink_len;

extern const unsigned int *AP_DCM_Golden_Setting_tcl_gs_flight;
extern unsigned int AP_DCM_Golden_Setting_tcl_gs_flight_len;

#endif

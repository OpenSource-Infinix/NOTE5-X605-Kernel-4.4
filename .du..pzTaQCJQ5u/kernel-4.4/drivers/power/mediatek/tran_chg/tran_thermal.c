/*****************************************************************************
 *
 * Filename:
 * ---------
 *   tran_thermal.c
 *
 * Project:
 * --------
 *   Kernel_Software
 *
 * Description:
 * ------------
 *   This Module is For Custom Transsion Thermal
 *
 * Author:
 * -------
 *
 *
 ****************************************************************************/
#include <linux/thermal.h>
#include <mt-plat/mtk_thermal_monitor.h>
#include "tran_thermal.h"

int TRAN_THERMAL_LOG = THERMAL_LOG_DEBG;

int g_max_temp_delta = 0;
int g_ibat_temp_cur = 0;
int g_pep_conn = 0;
int g_tran_backlight_level;
#define MAX_DELTA(a, b) (((a) > (b)) ? (a) : (b))

extern unsigned int g_charging_call_state;
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
extern bool mtk_is_pep_series_connect(struct charger_manager * info);
extern bool mtk_pe_get_is_connect(struct charger_manager *pinfo);
#endif

void change_current_by_screen_light(void)
{
	int delta_current_b = 0;
	int delta_current_c = 0;
	if(g_charging_call_state > 0){
		delta_current_c = g_ibat_temp_cur-TRAN_CALL_DROP_CURRENT;
		if(delta_current_c<=0){
			delta_current_c=0;
		}
		g_max_temp_delta=MAX_DELTA(delta_current_c,g_max_temp_delta);
	}

	if(g_tran_backlight_level > 0){
		delta_current_b = g_ibat_temp_cur-TRAN_SCREEN_DROP_CURRENT;
		if(delta_current_b<=0){
			delta_current_b=0;
		}
		g_max_temp_delta=MAX_DELTA(delta_current_b,g_max_temp_delta);
	}

	tran_thml_log(THERMAL_LOG_CRTI, "support TRAN_CHARGER_DROP_CUR_BY_CALL OR TRAN_CHARGER_DROP_CUR_BY_SCREEN g_max_temp_delta = %d, delta_current_c = %d, delta_current_b = %d\n", g_max_temp_delta, delta_current_c, delta_current_b);
}

#if defined(CONFIG_TRAN_CHARGER_CUR_FOR_TEMP_SUPPORT)
void change_current_by_pcb_temp(void)
{
	int delta_current = 0;
	int PCB_temperature = 0;

	PCB_temperature = mtk_thermal_get_temp(MTK_THERMAL_SENSOR_AP);
	if(-127000 == PCB_temperature){
		return;
	}

    //calculate the delta of the temperature for delta of charging current
        if((PCB_temperature>TRAN_PCB_DROP_TEMP_STEP_ONE) && (PCB_temperature <=TRAN_PCB_DROP_TEMP_STEP_TWO)){
		delta_current = (PCB_temperature - TRAN_PCB_DROP_TEMP_STEP_ONE)*TRAN_PCB_DROP_CUR_STEP_ONE;
        }else if(PCB_temperature>TRAN_PCB_DROP_TEMP_STEP_TWO){
		delta_current =(TRAN_PCB_DROP_TEMP_STEP_TWO-TRAN_PCB_DROP_TEMP_STEP_ONE)*TRAN_PCB_DROP_CUR_STEP_ONE
			+(PCB_temperature - TRAN_PCB_DROP_TEMP_STEP_TWO)*TRAN_PCB_DROP_CUR_STEP_TWO;
	}
	if(delta_current<=0){
		delta_current=0;
	}

	g_max_temp_delta = MAX_DELTA(delta_current, g_max_temp_delta);

	tran_thml_log(THERMAL_LOG_CRTI, "support TRAN_CHARGER_DROP_CUR_BY_PCB delta_current = %d g_max_temp_delta = %d Tpcb = %d\n", delta_current, g_max_temp_delta, PCB_temperature);

}

void change_current_by_temp(struct charger_manager *info)
{
    int ichg;
    struct charger_data *pdata;
    pdata = &info->chg1_data;

	tran_thml_log(THERMAL_LOG_CRTI,"\n");

	change_current_by_pcb_temp();
	change_current_by_screen_light();

	tran_thml_log(THERMAL_LOG_CRTI, "line:%d [%s] get charging current g_ibat_temp_cur = %d, g_max_temp_delta = %d\n", __LINE__, __func__, g_ibat_temp_cur, g_max_temp_delta);

    //error check
	if(g_max_temp_delta >= g_ibat_temp_cur){
		g_max_temp_delta = g_ibat_temp_cur - TRAN_MIN_CHARGING_CURRENT;
	}else if(g_max_temp_delta <= 0){
		g_max_temp_delta = 0;
	}else{
    //do nothing
	}

	//finnal set chg current and check
	ichg = g_ibat_temp_cur - g_max_temp_delta;
	tran_thml_log(THERMAL_LOG_CRTI, "line:%d [%s] Ibat = %d g_max_temp_delta=%d\n", __LINE__, __func__, ichg,g_max_temp_delta);

	if(ichg < TRAN_MIN_CHARGING_CURRENT){
		ichg = TRAN_MIN_CHARGING_CURRENT;
	}
	tran_thml_log(THERMAL_LOG_CRTI, "line:%d [%s] Final current setting Ibat = %d\n", __LINE__, __func__, ichg);
	if(0==g_pep_conn){
		pdata->input_current_limit=ichg;
	}

	pdata->charging_current_limit=ichg;
}
#endif

void set_jeita_charging_current(struct charger_manager *info,struct charger_data *pdata)
{
	if(info==NULL || pdata==NULL){
		return;
	}

    if(info->chr_type != STANDARD_CHARGER){
        return;
    }

	g_ibat_temp_cur = AC_CHARGER_CURRENT;
	g_pep_conn = 0;
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if(mtk_is_pep_series_connect(info) || mtk_pe_get_is_connect(info)){
		g_ibat_temp_cur = TA_AC_CHARGING_CURRENT;
		g_pep_conn = 1;
	}
#endif

	if(info->sw_jeita.sm == TEMP_BELOW_T0){
		pdata->input_current_limit = JEITA_TEMP_BELOW_T0_AC_CHG_CURRENT_INPUT;
		pdata->charging_current_limit = JEITA_TEMP_BELOW_T0_AC_CHG_CURRENT;
		pr_err("[BATTERY] TEMP_BELOW_T0 JEITA set charging current : %d %d\r\n",
			pdata->input_current_limit,pdata->charging_current_limit);
	}else if(info->sw_jeita.sm == TEMP_T0_TO_T1 || info->sw_jeita.sm == TEMP_T1_TO_T2){
        pdata->charging_current_limit = JEITA_TEMP_T0_TO_T1_AC_CHG_CURRENT;
        pdata->input_current_limit = JEITA_TEMP_T0_TO_T1_AC_CHG_CURRENT_INPUT;
        pr_err("[BATTERY] TEMP_T1_TO_T2 JEITA set charging current : %d %d\r\n",
				pdata->input_current_limit,pdata->charging_current_limit);
	}else if(info->sw_jeita.sm == TEMP_T2_TO_T3){
        pdata->charging_current_limit = JEITA_TEMP_T2_TO_T3_AC_CHG_CURRENT;
        pdata->input_current_limit = JEITA_TEMP_T2_TO_T3_AC_CHG_CURRENT_INPUT;
        if(1==g_pep_conn){
            pdata->charging_current_limit = TA_AC_CHARGING_CURRENT;
        }else{
	     pdata->charging_current_limit = AC_CHARGER_CURRENT;
	 }
        pr_err("[BATTERY] TEMP_T2_TO_T3 JEITA set charging current : %d %d\r\n",
				pdata->input_current_limit,pdata->charging_current_limit);
	}else if(info->sw_jeita.sm == TEMP_T3_TO_T4){
        pdata->charging_current_limit = JEITA_TEMP_T3_TO_T4_AC_CHG_CURRENT;
        pdata->input_current_limit = JEITA_TEMP_T3_TO_T4_AC_CHG_CURRENT_INPUT;
        pr_err("[BATTERY] TEMP_T3_TO_T4 JEITA set charging current : %d %d\r\n",
				pdata->input_current_limit,pdata->charging_current_limit);
	}else if(info->sw_jeita.sm == TEMP_ABOVE_T4){
        pdata->charging_current_limit = JEITA_TEMP_ABOVE_T4_AC_CHG_CURRENT;
        pdata->input_current_limit = JEITA_TEMP_ABOVE_T4_AC_CHG_CURRENT_INPUT;
		pr_err("[BATTERY] TEMP_ABOVE_T4 JEITA set charging current : %d %d\r\n",
				pdata->input_current_limit,pdata->charging_current_limit);
	}else{
		pr_err("[BATTERY] TEMP_POS_default JEITA set charging current : %d %d\r\n",
				pdata->input_current_limit,pdata->charging_current_limit);
	}

#if defined(CONFIG_TRAN_CHARGER_CUR_FOR_TEMP_SUPPORT)
    g_max_temp_delta = g_ibat_temp_cur - pdata->charging_current_limit;
    if((get_boot_mode() != KERNEL_POWER_OFF_CHARGING_BOOT && get_boot_mode() != LOW_POWER_OFF_CHARGING_BOOT)
        && (info->sw_jeita.sm != TEMP_ABOVE_T4)&&(info->sw_jeita.sm != TEMP_BELOW_T0)){
        change_current_by_temp(info);
    }
#endif

	return;
}

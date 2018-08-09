/*****************************************************************************
 *
 * Filename:
 * ---------
 *   tran_charger.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module register a new driver for providing services
 *
 * Author:
 * -------
 *
 *
 ****************************************************************************/
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>	/* platform device */

#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_battery.h>
#include <mtk_battery_internal.h>

/*global variable*/
#if defined(CONFIG_TRAN_CHARGER_AGING_SUPPORT)
int monkey_flag=0;
#endif
unsigned int g_charging_call_state = 0;



/*attributes*/
static ssize_t show_Charging_CallState(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_debug("call state = %d\n", g_charging_call_state);
	return sprintf(buf, "%u\n", g_charging_call_state);
}

static ssize_t store_Charging_CallState(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t size)
{

	if (kstrtouint(buf, 10, &g_charging_call_state) == 0) {
		bm_debug("call state = %d\n", g_charging_call_state);
		return size;
	}

	/* hidden else, for sscanf format error */
	{
		bm_debug("  bad argument, echo [enable] > current_cmd\n");
	}

	return 0;
}

static DEVICE_ATTR(Charging_CallState, 0664, show_Charging_CallState, store_Charging_CallState);

#if defined(CONFIG_TRAN_CHARGER_AGING_SUPPORT)
static ssize_t show_CHG_CAPACITY_TEST(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_debug( "%s: monkey_flag = %d\n", __func__, monkey_flag);
	return sprintf(buf, "%u\n", monkey_flag);
}

static ssize_t store_CHG_CAPACITY_TEST(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val) == 0) {
		monkey_flag = val;
		bm_debug( "%s: monkey_flag = %d\n", __func__, monkey_flag);
		return size;
	}

	/* hidden else, for sscanf format error */
	{
		bm_debug( "store_CHG_CAPACITY_TEST,bad argument\n");
	}

	return 0;
}

static DEVICE_ATTR(CHG_CAPACITY_TEST, 0664, show_CHG_CAPACITY_TEST, store_CHG_CAPACITY_TEST);
#endif

#if defined(CONFIG_TRAN_CHARGER_COULOMB_SUPPORT)
static ssize_t show_fg_coulomb(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned int fg_coulomb=0;
	fg_coulomb = gauge_get_coulomb();

	bm_debug("[FG] show_fg_coulomb : %d\n", fg_coulomb);
	return sprintf(buf, "%d\n", fg_coulomb);
}

static ssize_t store_fg_coulomb(struct device *dev, struct device_attribute *attr,
					       const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(fg_coulomb, 0664, show_fg_coulomb,store_fg_coulomb);
#endif

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT)
static ssize_t show_Pump_Express_VCharger(struct device *dev,struct device_attribute *attr,
							char *buf)
{
	int charger_vol=0;
	charger_vol = battery_meter_get_charger_voltage();
    bm_debug("show_Pump_Express_VCharger charger_vol= %d\n",charger_vol);
    return sprintf(buf, "%u\n", charger_vol);
}
static ssize_t store_Pump_Express_VCharger(struct device *dev,struct device_attribute *attr,
							const char *buf, size_t size)
{
    bm_debug("store_Pump_Express_VCharger\n");
    return size;
}
static DEVICE_ATTR(Pump_Express_VCharger, 0664, show_Pump_Express_VCharger, store_Pump_Express_VCharger);

static ssize_t show_Pump_Express_ICharger(struct device *dev,struct device_attribute *attr,
							char *buf)
{
	int charger_I=0;
	charger_I = (battery_meter_get_battery_current()) / 10;
	if (battery_meter_get_battery_current_sign() == 1) {
		charger_I = charger_I;	/* charging */
	}else{
		charger_I=0;
	}
    bm_debug("show_Pump_Express_ICharger charger_I= %d\n",charger_I);
    return sprintf(buf, "%u\n", charger_I);
}
static ssize_t store_Pump_Express_ICharger(struct device *dev,struct device_attribute *attr,
							const char *buf, size_t size)
{
    bm_debug("store_Pump_Express_ICharger\n");
    return size;
}
static DEVICE_ATTR(Pump_Express_ICharger, 0664, show_Pump_Express_ICharger, store_Pump_Express_ICharger);
#endif


/*probe*/
static int tran_battery_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	bm_err("******** battery_meter_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charging_CallState);
	#if defined(CONFIG_TRAN_CHARGER_AGING_SUPPORT)
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_CHG_CAPACITY_TEST);
	#endif
	#if defined(CONFIG_TRAN_CHARGER_COULOMB_SUPPORT)
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_fg_coulomb);
	#endif
	#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_20_SUPPORT)
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_Pump_Express_VCharger);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_Pump_Express_ICharger);
	#endif
	return 0;
}



struct platform_device tran_battery_device = {
	.name = "tran_battery",
	.id = -1,
};

static struct platform_driver tran_battery_driver = {
	.probe = tran_battery_probe,
	.remove = NULL,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "tran_battery",
	},
};


static int __init tran_battery_driver_init(void)
{
	int ret;
	bm_err("tran_battery_driver_init\n");

#if 0   //CONFIG_OF
		/*	*/
#else
	ret = platform_device_register(&tran_battery_device);
	if (ret) {
		bm_err("%s failed to device register\n",__func__);
		return ret;
	}
#endif

	ret = platform_driver_register(&tran_battery_driver);
	if (ret) {
		bm_err("%s failed to driver register\n",__func__);
		return ret;
	}
	bm_err("[tran_battery_driver_init] done\n");
	return 0;
}

static void __exit tran_battery_driver_exit(void)
{
	platform_driver_unregister(&tran_battery_driver);
}

module_init(tran_battery_driver_init);
module_exit(tran_battery_driver_exit);







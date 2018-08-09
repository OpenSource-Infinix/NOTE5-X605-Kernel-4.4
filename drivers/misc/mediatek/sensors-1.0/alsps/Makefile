# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
#add OSODEV-1176 by haiping.bai 20171123 start
obj-$(CONFIG_MTK_LTR579)        +=  ltr579/
#add OSODEV-1176 by haiping.bai 20171123 end
obj-$(CONFIG_MTK_EPL2182)	+=  epl2182/
#add XLLSHLSS-2715 by haiping.bai 20180409 start
obj-$(CONFIG_MTK_STK3X1X_NEW)        +=  stk3x1x-new/
#add XLLSHLSS-2715 by haiping.bai 20180409 end
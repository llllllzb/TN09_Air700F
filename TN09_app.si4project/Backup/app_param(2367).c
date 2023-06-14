#include "app_param.h"

#include "app_db.h"

bootParam_s bootparam;
systemParam_s sysparam;

void bootParamSaveAll(void)
{
    bootparam.VERSION = BOOT_PARAM_FLAG;
    EEPROM_ERASE(BOOT_USER_PARAM_ADDR, sizeof(bootParam_s));
    EEPROM_WRITE(BOOT_USER_PARAM_ADDR, &bootparam, sizeof(bootParam_s));
}
void bootParamGetAll(void)
{
    EEPROM_READ(BOOT_USER_PARAM_ADDR, &bootparam, sizeof(bootParam_s));
}

void paramSaveAll(void)
{
    EEPROM_ERASE(APP_USER_PARAM_ADDR, sizeof(systemParam_s));
    EEPROM_WRITE(APP_USER_PARAM_ADDR, &sysparam, sizeof(systemParam_s));
}
void paramGetAll(void)
{
    EEPROM_READ(APP_USER_PARAM_ADDR, &sysparam, sizeof(systemParam_s));
}

void paramDefaultInit(uint8_t level)
{
    LogMessage(DEBUG_ALL, "paramDefaultInit");
    if (level == 0)
    {
        memset(&sysparam, 0, sizeof(systemParam_s));
        sysparam.VERSION = APP_PARAM_FLAG;
        strcpy(sysparam.SN, "888888887777777");
        strncpy(sysparam.jt808sn, "888777", 6);

        strcpy(sysparam.Server, "jzwz.basegps.com");
        strcpy(sysparam.hiddenServer, "jzwz.basegps.com");
        strcpy(sysparam.bleServer, "jzwz.basegps.com");
        strcpy(sysparam.jt808Server, "47.106.96.28");

        sysparam.ServerPort = 9998;
        sysparam.bleServerPort = 9998;
        sysparam.hiddenPort = 9998;
        sysparam.jt808Port = 9997;

        sysparam.utc = 8;
        sysparam.AlarmTime[0] = 720;
        sysparam.AlarmTime[1] = 0xFFFF;
        sysparam.AlarmTime[2] = 0xFFFF;
        sysparam.AlarmTime[3] = 0xFFFF;
        sysparam.AlarmTime[4] = 0xFFFF;
        sysparam.MODE = 2;
        sysparam.gpsuploadgap = 10;
        sysparam.gapDay = 1;
        sysparam.poitype = 2;
        sysparam.lowvoltage = 36;
        sysparam.adccal = 3.06;
        strcpy(sysparam.apn, "cmnet");

        strcpy((char *)sysparam.agpsServer, "agps.domilink.com");
        strcpy((char *)sysparam.agpsUser, "123");
        strcpy((char *)sysparam.agpsPswd, "123");
        sysparam.agpsPort = 10188;

    }
    sysparam.runTime = 0;
    sysparam.startUpCnt = 0;
    sysparam.accctlgnss = 1;
    sysparam.accdetmode = 2;
    sysparam.heartbeatgap = 180;
    sysparam.ledctrl = 0;
    sysparam.poitype = 2;
    sysparam.fence = 30;
    sysparam.accOnVoltage = 13.2;
    sysparam.accOffVoltage = 12.8;
    sysparam.noNmeaRstCnt = 0;
    sysparam.sosalm = ALARM_TYPE_NONE;
    paramSaveAll();
}

void paramInit(void)
{
    paramGetAll();
    if (sysparam.VERSION != APP_PARAM_FLAG)
    {
        paramDefaultInit(0);
    }
    sysinfo.lowvoltage = sysparam.lowvoltage / 10.0;
    dbInfoRead();
}

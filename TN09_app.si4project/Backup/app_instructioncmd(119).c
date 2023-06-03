#include <app_protocol.h>
#include "app_instructioncmd.h"

#include "app_peripheral.h"

#include "app_gps.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_server.h"
#include "app_jt808.h"
const instruction_s insCmdTable[] =
{
    {PARAM_INS, "PARAM"},
    {STATUS_INS, "STATUS"},
    {VERSION_INS, "VERSION"},
    {SERVER_INS, "SERVER"},
    {HBT_INS, "HBT"},
    {MODE_INS, "MODE"},
    {POSITION_INS, "123"},
    {APN_INS, "APN"},
    {UPS_INS, "UPS"},
    {LOWW_INS, "LOWW"},
    {LED_INS, "LED"},
    {POITYPE_INS, "POITYPE"},
    {RESET_INS, "RESET"},
    {UTC_INS, "UTC"},
    {DEBUG_INS, "DEBUG"},
    {ACCCTLGNSS_INS, "ACCCTLGNSS"},
    {ACCDETMODE_INS, "ACCDETMODE"},
    {FENCE_INS, "FENCE"},
    {FACTORY_INS, "FACTORY"},
    {ICCID_INS, "ICCID"},
    {SETAGPS_INS, "SETAGPS"},
    {JT808SN_INS, "JT808SN"},
    {HIDESERVER_INS, "HIDESERVER"},
    {BLESERVER_INS, "BLESERVER"},
    {SOS_INS, "SOS"},
    {CENTER_INS, "CENTER"},
    {SOSALM_INS, "SOSALM"},
    {TIMER_INS, "TIMER"},
    {TILTALM_INS, "TILTALM"},
    {LEAVEALM_INS, "LEAVEALM"},
    {BLELINKFAIL_INS, "BLELINKFAIL"},
};

static insMode_e mode123;
static insParam_s param123;
static uint8_t serverType;


static void sendMsgWithMode(uint8_t *buf, uint16_t len, insMode_e mode, void *param)
{
    insParam_s *insparam;
    switch (mode)
    {
        case DEBUG_MODE:
            LogMessage(DEBUG_FACTORY, "----------Content----------");
            LogMessage(DEBUG_FACTORY, (char *)buf);
            LogMessage(DEBUG_FACTORY, "------------------------------");
            break;
        case SMS_MODE:
            if (param != NULL)
            {
                insparam = (insParam_s *)param;
                sendMessage(buf, len, insparam->telNum);
                //startTimer(15, deleteMessage, 0);
            }
            break;
        case NET_MODE:
            if (param != NULL)
            {
                insparam = (insParam_s *)param;
                insparam->data = buf;
                insparam->len = len;
                protocolSend(insparam->link, PROTOCOL_21, insparam);
            }
            break;
        case BLE_MODE:
            appSendNotifyData(buf, len);
            break;
    }
}

static void doParamInstruction(ITEM *item, char *message)
{
    uint8_t i;
    uint8_t debugMsg[15];
    if (sysparam.protocol == JT808_PROTOCOL_TYPE)
    {
        byteToHexString(sysparam.jt808sn, debugMsg, 6);
        debugMsg[12] = 0;
        sprintf(message + strlen(message), "JT808SN:%s;SN:%s;IP:%s:%u;", debugMsg, sysparam.SN, sysparam.jt808Server,
                sysparam.jt808Port);
    }
    else
    {
        sprintf(message + strlen(message), "SN:%s;IP:%s:%d;", sysparam.SN, sysparam.Server, sysparam.ServerPort);
    }
    sprintf(message + strlen(message), "APN:%s;", sysparam.apn);
    sprintf(message + strlen(message), "UTC:%s%d;", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE21:
            if (sysparam.MODE == MODE1)
            {
                sprintf(message + strlen(message), "Mode1:");

            }
            else
            {
                sprintf(message + strlen(message), "Mode21:");

            }
            for (i = 0; i < 5; i++)
            {
                if (sysparam.AlarmTime[i] != 0xFFFF)
                {
                    sprintf(message + strlen(message), " %.2d:%.2d", sysparam.AlarmTime[i] / 60, sysparam.AlarmTime[i] % 60);
                }

            }
            sprintf(message + strlen(message), ", Every %d day;", sysparam.gapDay);
            break;
        case MODE2:
            if (sysparam.gpsuploadgap != 0)
            {
                if (sysparam.gapMinutes == 0)
                {
                    //��⵽�𶯣�n ������
                    sprintf(message + strlen(message), "Mode2: %dS;", sysparam.gpsuploadgap);
                }
                else
                {
                    //��⵽�𶯣�n �����ͣ�δ�𶯣�m�����Զ�����
                    sprintf(message + strlen(message), "Mode2: %dS,%dM;", sysparam.gpsuploadgap, sysparam.gapMinutes);

                }
            }
            else
            {
                if (sysparam.gapMinutes == 0)
                {
                    //�������ߣ�������
                    sprintf(message + strlen(message), "Mode2: online;");
                }
                else
                {
                    //�������ߣ�������𶯣�ÿ��m���ӣ��Զ�����
                    sprintf(message + strlen(message), "Mode2: %dM;", sysparam.gapMinutes);
                }
            }
            break;
        case MODE3:
            sprintf(message + strlen(message), "Mode3: %d minutes;", sysparam.gapMinutes);
            break;
        case MODE23:
            sprintf(message + strlen(message), "Mode23: %d minutes;", sysparam.gapMinutes);
            break;
    }

    sprintf(message + strlen(message), "StartUp:%u;RunTime:%u;", sysparam.startUpCnt, sysparam.runTime);

}
static void doStatusInstruction(ITEM *item, char *message)
{
    gpsinfo_s *gpsinfo;
    moduleGetCsq();
    sprintf(message, "OUT-V=%.2fV;", sysinfo.outsidevoltage);
    //sprintf(message + strlen(message), "BAT-V=%.2fV;", sysinfo.insidevoltage);
    if (sysinfo.gpsOnoff)
    {
        gpsinfo = getCurrentGPSInfo();
        sprintf(message + strlen(message), "GPS=%s;", gpsinfo->fixstatus ? "Fixed" : "Invalid");
        sprintf(message + strlen(message), "PDOP=%.2f;", gpsinfo->pdop);
    }
    else
    {
        sprintf(message + strlen(message), "GPS=Close;");
    }

    sprintf(message + strlen(message), "ACC=%s;", getTerminalAccState() > 0 ? "On" : "Off");
    sprintf(message + strlen(message), "SIGNAL=%d;", getModuleRssi());
    sprintf(message + strlen(message), "BATTERY=%s;", getTerminalChargeState() > 0 ? "Charging" : "Uncharged");
    sprintf(message + strlen(message), "LOGIN=%s;", primaryServerIsReady() > 0 ? "Yes" : "No");
}


static void serverChangeCallBack(void)
{
    jt808ServerReconnect();
    privateServerReconnect();
    if (serverType == JT808_PROTOCOL_TYPE)
    {
        sysparam.protocol = JT808_PROTOCOL_TYPE;
        sysparam.jt808isRegister = 0;
    }
    else
    {
        sysparam.protocol = ZT_PROTOCOL_TYPE;
    }
    paramSaveAll();
}

static void doServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[2][0] != 0 && item->item_data[3][0] != 0)
    {
        serverType = atoi(item->item_data[1]);
        if (serverType == JT808_PROTOCOL_TYPE)
        {
            strncpy((char *)sysparam.jt808Server, item->item_data[2], 50);
            stringToLowwer(sysparam.jt808Server, strlen(sysparam.jt808Server));
            sysparam.jt808Port = atoi((const char *)item->item_data[3]);
            sprintf(message, "Update jt808 domain %s:%d;", sysparam.jt808Server, sysparam.jt808Port);

        }
        else
        {
            strncpy((char *)sysparam.Server, item->item_data[2], 50);
            stringToLowwer(sysparam.Server, strlen(sysparam.Server));
            sysparam.ServerPort = atoi((const char *)item->item_data[3]);
            sprintf(message, "Update domain %s:%d;", sysparam.Server, sysparam.ServerPort);
        }
        startTimer(30, serverChangeCallBack, 0);
    }
    else
    {
        sprintf(message, "Update domain fail,please check your param");
    }

}


static void doVersionInstruction(ITEM *item, char *message)
{
    sprintf(message, "Version:%s;Compile:%s %s;", EEPROM_VERSION, __DATE__, __TIME__);

}
static void doHbtInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "The time of the heartbeat interval is %d seconds;", sysparam.heartbeatgap);
    }
    else
    {
        sysparam.heartbeatgap = (uint8_t)atoi((const char *)item->item_data[1]);
        paramSaveAll();
        sprintf(message, "Update the time of the heartbeat interval to %d seconds;", sysparam.heartbeatgap);
    }

}
static void doModeInstruction(ITEM *item, char *message)
{
    uint8_t workmode, i, j, timecount = 0, gapday = 1;
    uint16_t mode1time[7];
    uint16_t valueofminute;
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Mode%d,%d", sysparam.MODE, sysparam.gpsuploadgap);
    }
    else
    {
        sysinfo.runStartTick = sysinfo.sysTick;
        workmode = atoi(item->item_data[1]);
        gpsRequestClear(GPS_REQUEST_GPSKEEPOPEN_CTL);
        switch (workmode)
        {
            case 1:
            case 21:
                //�������������2��˵����ʱ�������
                if (item->item_cnt > 2)
                {
                    for (i = 0; i < item->item_cnt - 2; i++)
                    {
                        if (strlen(item->item_data[2 + i]) <= 4 && strlen(item->item_data[2 + i]) >= 3)
                        {
                            valueofminute = atoi(item->item_data[2 + i]);
                            mode1time[timecount++] = valueofminute / 100 * 60 + valueofminute % 100;
                        }
                        else
                        {
                            gapday = atoi(item->item_data[2 + i]);
                        }
                    }

                    for (i = 0; i < (timecount - 1); i++)
                    {
                        for (j = 0; j < (timecount - 1 - i); j++)
                        {
                            if (mode1time[j] > mode1time[j + 1]) //��������Ԫ�����Ƚϣ����ǰ��Ԫ�ش��ں��棬���н���
                            {
                                valueofminute = mode1time[j + 1];
                                mode1time[j + 1] = mode1time[j];
                                mode1time[j] = valueofminute;
                            }
                        }
                    }

                }

                for (i = 0; i < 5; i++)
                {
                    sysparam.AlarmTime[i] = 0xFFFF;
                }
                //����д��AlarmTime
                for (i = 0; i < timecount; i++)
                {
                    sysparam.AlarmTime[i] = mode1time[i];
                }
                sysparam.gapDay = gapday;
                if (workmode == MODE1)
                {
                    terminalAccoff();
                    if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    sysparam.MODE = MODE1;
                    portGsensorCtl(0);
                }
                else
                {
                    sysparam.MODE = MODE21;
                    portGsensorCtl(0);
                }
                sprintf(message, "Change to Mode%d,and work on at", workmode);
                for (i = 0; i < timecount; i++)
                {
                    sprintf(message + strlen(message), " %.2d:%.2d", sysparam.AlarmTime[i] / 60, sysparam.AlarmTime[i] % 60);
                }
                sprintf(message + strlen(message), ",every %d day;", gapday);
                portSetNextAlarmTime();
                break;
            case 2:
                portGsensorCtl(1);
                sysparam.gpsuploadgap = (uint8_t)atoi((const char *)item->item_data[2]);
                sysparam.gapMinutes = (uint16_t)atoi((const char *)item->item_data[3]);
                sysparam.MODE = MODE2;
                if (sysparam.gpsuploadgap == 0)
                {
                    gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    //�˶����Զ���GPS
                    if (sysparam.gapMinutes == 0)
                    {

                        sprintf(message, "The device switches to mode 2 without uploading the location");
                    }
                    else
                    {
                        sprintf(message, "The device switches to mode 2 and uploads the position every %d minutes all the time",
                                sysparam.gapMinutes);
                    }
                }
                else
                {
                    if (getTerminalAccState())
                    {
                        if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
                        {
                            gpsRequestSet(GPS_REQUEST_ACC_CTL);
                        }
                        else
                        {
                            gpsRequestClear(GPS_REQUEST_ACC_CTL);
                        }
                    }
                    else
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    if (sysparam.accctlgnss == 0)
                    {
                        gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
                    }
                    if (sysparam.gapMinutes == 0)
                    {
                        sprintf(message, "The device switches to mode 2 and uploads the position every %d seconds when moving",
                                sysparam.gpsuploadgap);

                    }
                    else
                    {
                        sprintf(message,
                                "The device switches to mode 2 and uploads the position every %d seconds when moving, and every %d minutes when standing still",
                                sysparam.gpsuploadgap, sysparam.gapMinutes);
                    }

                }
                break;
            case 3:
            case 23:
                sysparam.gapMinutes = (uint16_t)atoi(item->item_data[2]);
                if (sysparam.gapMinutes < 5)
                {
                    sysparam.gapMinutes = 5;
                }
                if (workmode == MODE3)
                {
                    terminalAccoff();
                    if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                    sysparam.MODE = MODE3;
                    portGsensorCtl(0);
                    sprintf(message, "Change to mode %d and update the startup interval time to %d minutes;", workmode,
                            sysparam.gapMinutes);
                }
                else
                {
                    if (atoi(item->item_data[3]) != 0)
                    {
                        sysparam.gpsuploadgap = (uint16_t)atoi((const char *)item->item_data[3]);
                    }
                    sysparam.MODE = MODE23;
                    portGsensorCtl(1);
                    sprintf(message, "Change to mode %d and update interval time to %d m %d s;", workmode, sysparam.gapMinutes,
                            sysparam.gpsuploadgap);
                }

                break;
            default:
                strcpy(message, "Unsupport mode");
                break;
        }
        paramSaveAll();

    }
}



void dorequestSend123(void)
{
    char message[150];
    uint16_t year;
    uint8_t  month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    gpsinfo_s *gpsinfo;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    sysinfo.flag123 = 0;
    gpsinfo = getCurrentGPSInfo();
    sprintf(message, "(%s)<Local Time:%.2d/%.2d/%.2d %.2d:%.2d:%.2d>http://maps.google.com/maps?q=%s%f,%s%f", sysparam.SN, \
            year, month, date, hour, minute, second, \
            gpsinfo->NS == 'N' ? "" : "-", gpsinfo->latitude, gpsinfo->EW == 'E' ? "" : "-", gpsinfo->longtitude);
    reCover123InstructionId();
    sendMsgWithMode((uint8_t *)message, strlen(message), mode123, &param123);
}

void do123Instruction(ITEM *item, insMode_e mode, void *param)
{
    insParam_s *insparam;
    mode123 = mode;
    if (param != NULL)
    {
        insparam = (insParam_s *)param;
        param123.telNum = insparam->telNum;
    }
    if (sysparam.poitype == 0)
    {
        lbsRequestSet(DEV_EXTEND_OF_MY);
        LogMessage(DEBUG_ALL, "Only LBS reporting");
    }
    else if (sysparam.poitype == 1)
    {
        lbsRequestSet(DEV_EXTEND_OF_MY);
        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        LogMessage(DEBUG_ALL, "LBS and GPS reporting");
    }
    else
    {
        lbsRequestSet(DEV_EXTEND_OF_MY);
        wifiRequestSet(DEV_EXTEND_OF_MY);
        gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        LogMessage(DEBUG_ALL, "LBS ,WIFI and GPS reporting");
    }
    sysinfo.flag123 = 1;
    save123InstructionId();

}


void doAPNInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "APN:%s,APN User:%s,APN Password:%s", sysparam.apn, sysparam.apnuser, sysparam.apnpassword);
    }
    else
    {
        sysparam.apn[0] = 0;
        sysparam.apnuser[0] = 0;
        sysparam.apnpassword[0] = 0;
        if (item->item_data[1][0] != 0 && item->item_cnt >= 2)
        {
            strcpy((char *)sysparam.apn, item->item_data[1]);
        }
        if (item->item_data[2][0] != 0 && item->item_cnt >= 3)
        {
            strcpy((char *)sysparam.apnuser, item->item_data[2]);
        }
        if (item->item_data[3][0] != 0 && item->item_cnt >= 4)
        {
            strcpy((char *)sysparam.apnpassword, item->item_data[3]);
        }

        startTimer(300, moduleReset, 0);
        paramSaveAll();
        sprintf(message, "Update APN:%s,APN User:%s,APN Password:%s", sysparam.apn, sysparam.apnuser, sysparam.apnpassword);
    }

}


void doUPSInstruction(ITEM *item, char *message)
{
    if (item->item_cnt >= 3)
    {
        strcpy((char *)bootparam.updateServer, item->item_data[1]);
        bootparam.updatePort = atoi(item->item_data[2]);
        bootParamSaveAll();
    }
    bootParamGetAll();
    sprintf(message, "The device will download the firmware from %s:%d in 5 seconds", bootparam.updateServer,
            bootparam.updatePort);
    bootparam.updateStatus = 1;
    strcpy(bootparam.SN, sysparam.SN);
    strcpy(bootparam.apn, sysparam.apn);
    strcpy(bootparam.apnuser, sysparam.apnuser);
    strcpy(bootparam.apnpassword, sysparam.apnpassword);
    strcpy(bootparam.codeVersion, EEPROM_VERSION);
    bootParamSaveAll();
    startTimer(40, portSysReset, 0);
}

void doLOWWInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sysinfo.lowvoltage = sysparam.lowvoltage / 10.0;
        sprintf(message, "The low voltage param is %.1fV", sysinfo.lowvoltage);

    }
    else
    {
        sysparam.lowvoltage = atoi(item->item_data[1]);
        sysinfo.lowvoltage = sysparam.lowvoltage / 10.0;
        paramSaveAll();
        sprintf(message, "When the voltage is lower than %.1fV, an alarm will be generated", sysinfo.lowvoltage);
    }
}

void doLEDInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "LED was %s", sysparam.ledctrl ? "open" : "close");

    }
    else
    {
        sysparam.ledctrl = atoi(item->item_data[1]);
        paramSaveAll();
        sprintf(message, "%s", sysparam.ledctrl ? "LED ON" : "LED OFF");
    }
}


void doPOITYPEInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.poitype)
        {
            case 0:
                strcpy(message, "Current poitype is only LBS reporting");
                break;
            case 1:
                strcpy(message, "Current poitype is LBS and GPS reporting");
                break;
            case 2:
                strcpy(message, "Current poitype is LBS ,WIFI and GPS reporting");
                break;
        }
    }
    else
    {
        if (strstr(item->item_data[1], "AUTO") != NULL)
        {
            sysparam.poitype = 2;
        }
        else
        {
            sysparam.poitype = atoi(item->item_data[1]);
        }
        switch (sysparam.poitype)
        {
            case 0:
                strcpy(message, "Set to only LBS reporting");
                break;
            case 1:
                strcpy(message, "Set to LBS and GPS reporting");
                break;
            case 2:
                strcpy(message, "Set to LBS ,WIFI and GPS reporting");
                break;
            default:
                sysparam.poitype = 2;
                strcpy(message, "Unknow type,default set to LBS ,WIFI and GPS reporting");
                break;
        }
        paramSaveAll();

    }
}

void doResetInstruction(ITEM *item, char *message)
{
    sprintf(message, "System will reset after 5 seconds");
    startTimer(50, portSysReset, 0);
}

void doUTCInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "System time zone:UTC %s%d", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
        updateRTCtimeRequest();
    }
    else
    {
        sysparam.utc = atoi(item->item_data[1]);
        updateRTCtimeRequest();
        LogPrintf(DEBUG_ALL, "utc=%d", sysparam.utc);
        if (sysparam.utc < -12 || sysparam.utc > 12)
            sysparam.utc = 8;
        paramSaveAll();
        sprintf(message, "Update the system time zone to UTC %s%d", sysparam.utc >= 0 ? "+" : "", sysparam.utc);
    }
}

void doDebugInstrucion(ITEM *item, char *message)
{
    uint16_t year;
    uint8_t  month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);

    sprintf(message, "Time:%.2d/%.2d/%.2d %.2d:%.2d:%.2d;", year, month, date, hour, minute, second);
    sprintf(message + strlen(message), "sysrun:%.2d:%.2d:%.2d;gpsRequest:%02X;gpslast:%.2d:%.2d:%.2d;",
            sysinfo.sysTick / 3600, sysinfo.sysTick % 3600 / 60, sysinfo.sysTick % 60, sysinfo.gpsRequest,
            sysinfo.gpsUpdatetick / 3600, sysinfo.gpsUpdatetick % 3600 / 60, sysinfo.gpsUpdatetick % 60);
    sprintf(message + strlen(message), "hideLogin:%s;", hiddenServerIsReady() ? "Yes" : "No");
}

void doACCCTLGNSSInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "%s", sysparam.accctlgnss ? "GPS is automatically controlled by the program" :
                "The GPS is always be on");
    }
    else
    {
        sysparam.accctlgnss = (uint8_t)atoi((const char *)item->item_data[1]);
        if (sysparam.MODE == MODE2)
        {
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            else
            {
                gpsRequestClear(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
        }
        sprintf(message, "%s", sysparam.accctlgnss ? "GPS will be automatically controlled by the program" :
                "The GPS will always be on");
        paramSaveAll();
    }

}


void doAccdetmodeInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.accdetmode)
        {
            case ACCDETMODE0:
                sprintf(message, "The device is use acc wire to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE1:
                sprintf(message, "The device is use acc wire first and voltage second to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE2:
                sprintf(message, "The device is use acc wire first and gsensor second to determine whether ACC is ON or OFF.");
                break;
        }

    }
    else
    {
        sysparam.accdetmode = atoi(item->item_data[1]);
        switch (sysparam.accdetmode)
        {
            case ACCDETMODE0:
                sprintf(message, "The device is use acc wire to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE1:
                sprintf(message, "The device is use acc wire first and voltage second to determine whether ACC is ON or OFF.");
                break;
            case ACCDETMODE2:
                sprintf(message, "The device is use acc wire first and gsensor second to determine whether ACC is ON or OFF.");
                break;
            default:
                sysparam.accdetmode = ACCDETMODE2;
                sprintf(message,
                        "Unknow mode,Using acc wire first and voltage second to determine whether ACC is ON or OFF by default");
                break;
        }
        paramSaveAll();
    }
}

void doFenceInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "The static drift fence is %d meters", sysparam.fence);
    }
    else
    {
        sysparam.fence = atol(item->item_data[1]);
        paramSaveAll();
        sprintf(message, "Set the static drift fence to %d meters", sysparam.fence);
    }

}

void doIccidInstrucion(ITEM *item, char *message)
{
    sendModuleCmd(CCID_CMD, NULL);
    sprintf(message, "ICCID:%s", getModuleICCID());
}

void doFactoryInstrucion(ITEM *item, char *message)
{
    if (my_strpach(item->item_data[1], "ZTINFO8888"))
    {
        paramDefaultInit(0);
        sprintf(message, "Factory all successfully");
    }
    else
    {
        paramDefaultInit(1);
        sprintf(message, "Factory Settings restored successfully");

    }

}
void doSetAgpsInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Agps:%s,%d,%s,%s", sysparam.agpsServer, sysparam.agpsPort, sysparam.agpsUser, sysparam.agpsPswd);
    }
    else
    {
        if (item->item_data[1][0] != 0)
        {
            strcpy((char *)sysparam.agpsServer, item->item_data[1]);
        }
        if (item->item_data[2][0] != 0)
        {
            sysparam.agpsPort = atoi(item->item_data[2]);
        }
        if (item->item_data[3][0] != 0)
        {
            strcpy((char *)sysparam.agpsUser, item->item_data[3]);
        }
        if (item->item_data[4][0] != 0)
        {
            strcpy((char *)sysparam.agpsPswd, item->item_data[4]);
        }
        paramSaveAll();
        sprintf(message, "Update Agps info:%s,%d,%s,%s", sysparam.agpsServer, sysparam.agpsPort, sysparam.agpsUser,
                sysparam.agpsPswd);
    }
}

static void doJT808SNInstrucion(ITEM *item, char *message)
{
    char senddata[40];
    uint8_t snlen;
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        byteToHexString(sysparam.jt808sn, (uint8_t *)senddata, 6);
        senddata[12] = 0;
        sprintf(message, "JT808SN:%s", senddata);
    }
    else
    {
        snlen = strlen(item->item_data[1]);
        if (snlen > 12)
        {
            sprintf(message, "SN number too long");
        }
        else
        {
            jt808CreateSn(sysparam.jt808sn, (uint8_t *)item->item_data[1], snlen);
            byteToHexString(sysparam.jt808sn, (uint8_t *)senddata, 6);
            senddata[12] = 0;
            sprintf(message, "Update SN:%s", senddata);
            sysparam.jt808isRegister = 0;
            sysparam.jt808AuthLen = 0;
            jt808RegisterLoginInfo(sysparam.jt808sn, sysparam.jt808isRegister, sysparam.jt808AuthCode, sysparam.jt808AuthLen);
            paramSaveAll();
        }
    }
}

static void doHideServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "hidden server %s:%d was %s", sysparam.hiddenServer, sysparam.hiddenPort,
                sysparam.hiddenServOnoff ? "on" : "off");
    }
    else
    {
        if (item->item_data[1][0] == '1')
        {
            sysparam.hiddenServOnoff = 1;
            if (item->item_data[2][0] != 0 && item->item_data[3][0] != 0)
            {
                strncpy((char *)sysparam.hiddenServer, item->item_data[2], 50);
                stringToLowwer(sysparam.hiddenServer, strlen(sysparam.hiddenServer));
                sysparam.hiddenPort = atoi((const char *)item->item_data[3]);
                sprintf(message, "Update hidden server %s:%d and enable it", sysparam.hiddenServer, sysparam.hiddenPort);
            }
            else
            {
                strcpy(message, "please enter your param");
            }
        }
        else
        {
            sysparam.hiddenServOnoff = 0;
            strcpy(message, "Disable hidden server");
        }
        paramSaveAll();
    }
}


static void doBleServerInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "ble server was %s:%d", sysparam.bleServer, sysparam.bleServerPort);
    }
    else
    {
        if (item->item_data[1][0] != 0 && item->item_data[2][0] != 0)
        {
            strncpy((char *)sysparam.bleServer, item->item_data[1], 50);
            stringToLowwer(sysparam.bleServer, strlen(sysparam.bleServer));
            sysparam.bleServerPort = atoi((const char *)item->item_data[2]);
            sprintf(message, "Update ble server %s:%d", sysparam.bleServer, sysparam.bleServerPort);
            paramSaveAll();
        }
        else
        {
            strcpy(message, "please enter your param");
        }

    }
}


void doTimerInstrucion(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        sprintf(message, "Current timer is %d", sysparam.gpsuploadgap);
    }
    else
    {
        sysparam.gpsuploadgap = (uint16_t)atoi((const char *)item->item_data[1]);
        sprintf(message, "Update timer to %d", sysparam.gpsuploadgap);
        if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
        {
            if (sysparam.gpsuploadgap == 0)
            {
                gpsRequestClear(GPS_REQUEST_ACC_CTL);
            }
            else
            {
                if (getTerminalAccState())
                {
                    if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
                    {
                        gpsRequestSet(GPS_REQUEST_ACC_CTL);
                    }
                    else
                    {
                        gpsRequestClear(GPS_REQUEST_ACC_CTL);
                    }
                }
                else
                {
                    gpsRequestClear(GPS_REQUEST_ACC_CTL);
                }
            }
        }
        paramSaveAll();
    }
}

void doSOSInstruction(ITEM *item, char *messages, insMode_e mode, void *param)
{
    uint8_t i, j, k;
    insParam_s *insparam;

    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        strcpy(messages, "Query sos number:");
        for (i = 0; i < 3; i++)
        {
            sprintf(messages + strlen(messages), " %s", sysparam.sosNum[i][0] == 0 ? "NULL" : (char *)sysparam.sosNum[i]);
        }
    }
    else
    {

        if (mode == SMS_MODE && param != NULL)
        {
            insparam = (insParam_s *)param;
            if (my_strstr(insparam->telNum, (char *)sysparam.centerNum, strlen(insparam->telNum)) == 0)
            {
                strcpy(messages, "Operation failure,please use center number!");
                return ;
            }
        }
        if (item->item_data[1][0] == 'A' || item->item_data[1][0] == 'a')
        {
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 19; j++)
                {
                    sysparam.sosNum[i][j] = item->item_data[2 + i][j];
                }
                sysparam.sosNum[i][19] = 0;
            }
            strcpy(messages, "Add sos number:");
            for (i = 0; i < 3; i++)
            {
                sprintf(messages + strlen(messages), " %s", sysparam.sosNum[i][0] == 0 ? "NULL" : (char *)sysparam.sosNum[i]);
            }
        }
        else if (item->item_data[1][0] == 'D' || item->item_data[1][0] == 'd')
        {
            j = strlen(item->item_data[2]);
            if (j < 20 && j > 0)
            {
                k = 0;
                for (i = 0; i < 3; i++)
                {
                    if (my_strpach((char *)sysparam.sosNum[i], item->item_data[2]))
                    {
                        sprintf(messages + strlen(messages), "Delete %s OK;", (char *)sysparam.sosNum[i]);
                        sysparam.sosNum[i][0] = 0;
                        k = 1;
                    }
                }
                if (k == 0)
                {
                    sprintf(messages, "Delete %s Fail,no this number", item->item_data[2]);
                }
            }
            else
            {
                strcpy(messages, "Invalid sos number");
            }
        }
        else
        {
            strcpy(messages, "Invalid option");
        }
        paramSaveAll();
    }
}

void doCenterInstruction(ITEM *item, char *messages, insMode_e mode, void *param)
{
    uint8_t j, k;
    insParam_s *insparam;
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        strcpy(messages, "Query center number:");
        sprintf(messages + strlen(messages), " %s", sysparam.centerNum[0] == 0 ? "NULL" : (char *)sysparam.centerNum);
    }
    else
    {
        if (mode == SMS_MODE && param != NULL)
        {
            insparam = (insParam_s *)param;
            if (my_strstr(insparam->telNum, (char *)sysparam.centerNum, strlen(insparam->telNum)) == 0)
            {
                strcpy(messages, "Operation failure,please use center number!");
                return;
            }
        }
        if (item->item_data[1][0] == 'A' || item->item_data[1][0] == 'a')
        {
            for (j = 0; j < 19; j++)
            {
                sysparam.centerNum[j] = item->item_data[2][j];
            }
            sysparam.centerNum[19] = 0;
            strcpy(messages, "Add center number:");
            sprintf(messages + strlen(messages), " %s", sysparam.centerNum[0] == 0 ? "NULL" : (char *)sysparam.centerNum);
        }
        else if (item->item_data[1][0] == 'D' || item->item_data[1][0] == 'd')
        {
            j = strlen(item->item_data[2]);
            if (j < 20 && j > 0)
            {
                k = 0;
                if (my_strpach((char *)sysparam.centerNum, item->item_data[2]))
                {
                    sprintf(messages + strlen(messages), "Delete %s OK;", (char *)sysparam.centerNum);
                    sysparam.centerNum[0] = 0;
                    k = 1;
                }
                if (k == 0)
                {
                    sprintf(messages, "Delete %s Fail,no this number", item->item_data[2]);
                }
            }
            else
            {
                strcpy(messages, "Invalid center number");
            }
        }
        else
        {
            strcpy(messages, "Invalid option");
        }
        paramSaveAll();
    }
}

void doSosAlmInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
        switch (sysparam.sosalm)
        {
            case ALARM_TYPE_NONE:
                sprintf(message, "Query:%s", "SOS alarm was closed");
                break;
            case ALARM_TYPE_GPRS:
                sprintf(message, "Query:the device will %s when sos occur", "only send gprs");
                break;
            case ALARM_TYPE_GPRS_SMS:
                sprintf(message, "Query:the device will %s when sos occur", "send gprs and sms");
                break;
            case ALARM_TYPE_GPRS_SMS_TEL:
                sprintf(message, "Query:the device will %s when sos occur", "send gprs,sms and call sos number");
                break;
            default:
                strcpy(message, "Unknow");
                break;
        }
    }
    else
    {
        sysparam.sosalm = atoi(item->item_data[1]);
        switch (sysparam.sosalm)
        {
            case ALARM_TYPE_NONE:
                strcpy(message, "Close sos function");
                break;
            case ALARM_TYPE_GPRS:
                sprintf(message, "The device will %s when sos occur", "only send gprs");
                break;
            case ALARM_TYPE_GPRS_SMS:
                sprintf(message, "The device will %s when sos occur", "send gprs and sms");
                break;
            case ALARM_TYPE_GPRS_SMS_TEL:
                sprintf(message, "The device will %s when sos occur", "send gprs,sms and call sos number");
                break;
            default:
                sprintf(message, "%s", "Unknow status,change to send gprs by default");
                sysparam.sosalm = ALARM_TYPE_GPRS;
                break;
        }
        paramSaveAll();
    }
}


static void doBlelinkfailInstruction(ITEM *item, char *message)
{
    if (item->item_data[1][0] == 0 || item->item_data[1][0] == '?')
    {
    	if (sysparam.bleLinkFailCnt != 0)
    	{
			sprintf(message + strlen(message), "Ble is enable and Ble fail count:%u", sysparam.bleLinkFailCnt);
    	}
    	else
    	{
        	sprintf(message + strlen(message), "Ble is disable");
        }
    }
    else
    {
        sysparam.bleLinkFailCnt = atoi(item->item_data[1]);
        paramSaveAll();
        if (sysparam.bleLinkFailCnt != 0)
        {
			sprintf(message + strlen(message), "Enable ble OK and set ble fail count %d ", sysparam.bleLinkFailCnt);
        }
        else
        {
			sprintf(message + strlen(message), "Disable ble OK");
        }
    }
}


/*--------------------------------------------------------------------------------------*/
static void doinstruction(int16_t cmdid, ITEM *item, insMode_e mode, void *param)
{
    char message[512];
    message[0] = 0;
    switch (cmdid)
    {
        case PARAM_INS:
            doParamInstruction(item, message);
            break;
        case STATUS_INS:
            doStatusInstruction(item, message);
            break;
        case VERSION_INS:
            doVersionInstruction(item, message);
            break;
        case SERVER_INS:
            doServerInstruction(item, message);
            break;
        case HBT_INS:
            doHbtInstruction(item, message);
            break;
        case MODE_INS:
            doModeInstruction(item, message);
            break;
        case POSITION_INS:
            do123Instruction(item, mode, param);
            break;
        case APN_INS:
            doAPNInstruction(item, message);
            break;
        case UPS_INS:
            doUPSInstruction(item, message);
            break;
        case LOWW_INS:
            doLOWWInstruction(item, message);
            break;
        case LED_INS:
            doLEDInstruction(item, message);
            break;
        case POITYPE_INS:
            doPOITYPEInstruction(item, message);
            break;
        case RESET_INS:
            doResetInstruction(item, message);
            break;
        case UTC_INS:
            doUTCInstruction(item, message);
            break;
        case DEBUG_INS:
            doDebugInstrucion(item, message);
            break;
        case ACCCTLGNSS_INS:
            doACCCTLGNSSInstrucion(item, message);
            break;
        case ACCDETMODE_INS:
            doAccdetmodeInstruction(item, message);
            break;
        case FENCE_INS:
            doFenceInstrucion(item, message);
            break;
        case FACTORY_INS:
            doFactoryInstrucion(item, message);
            break;
        case ICCID_INS:
            doIccidInstrucion(item, message);
            break;
        case SETAGPS_INS:
            doSetAgpsInstruction(item, message);
            break;
        case JT808SN_INS:
            doJT808SNInstrucion(item, message);
            break;
        case HIDESERVER_INS:
            doHideServerInstruction(item, message);
            break;
        case BLESERVER_INS:
            doBleServerInstruction(item, message);
            break;
        case TIMER_INS:
            doTimerInstrucion(item, message);
            break;
        case SOS_INS:
            doSOSInstruction(item, message, mode, param);
            break;
        case CENTER_INS:
            doCenterInstruction(item, message, mode, param);
            break;
        case SOSALM_INS:
            doSosAlmInstruction(item, message);
            break;

        case BLELINKFAIL_INS:
			doBlelinkfailInstruction(item, message);
        	break;
        default:
            if (mode == SMS_MODE)
            {
                deleteMessage();
                return ;
            }
            snprintf(message, 50, "Unsupport CMD:%s;", item->item_data[0]);
            break;
    }

    sendMsgWithMode((uint8_t *)message, strlen(message), mode, param);
}

static int16_t getInstructionid(uint8_t *cmdstr)
{
    uint16_t i = 0;
    for (i = 0; i < sizeof(insCmdTable) / sizeof(insCmdTable[0]); i++)
    {
        if (mycmdPatch(insCmdTable[i].cmdstr, cmdstr))
        {
            return insCmdTable[i].cmdid;
        }
    }
    return -1;
}

void instructionParser(uint8_t *str, uint16_t len, insMode_e mode, void *param)
{
    ITEM item;
    int16_t cmdid;
    stringToItem(&item, str, len);
    strToUppper(item.item_data[0], strlen(item.item_data[0]));
    cmdid = getInstructionid((uint8_t *)item.item_data[0]);
    doinstruction(cmdid, &item, mode, param);
}


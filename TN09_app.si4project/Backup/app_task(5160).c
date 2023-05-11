#include <app_protocol.h>
#include "app_task.h"
#include "app_mir3da.h"
#include "app_atcmd.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_net.h"
#include "app_param.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_server.h"
#include "app_jt808.h"
#include "app_central.h"

#define SYS_LED1_ON       LED1_ON
#define SYS_LED1_OFF      LED1_OFF


static SystemLEDInfo sysledinfo;
static motionInfo_s motionInfo;
static bleScanTry_s bleTry;


/**************************************************
@bref		bit0 ��λ������
@param
@return
@note
**************************************************/
void terminalDefense(void)
{
    sysinfo.terminalStatus |= 0x01;
}

/**************************************************
@bref		bit0 ���������
@param
@return
@note
**************************************************/
void terminalDisarm(void)
{
    sysinfo.terminalStatus &= ~0x01;
}
/**************************************************
@bref		��ȡ�˶���ֹ״̬
@param
@return
	>0		�˶�
	0		��ֹ
@note
**************************************************/

uint8_t getTerminalAccState(void)
{
    return (sysinfo.terminalStatus & 0x02);

}

/**************************************************
@bref		bit1 ��λ���˶���accon
@param
@return
@note
**************************************************/

void terminalAccon(void)
{
    sysinfo.terminalStatus |= 0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 1);
}

/**************************************************
@bref		bit1 �������ֹ��accoff
@param
@return
@note
**************************************************/
void terminalAccoff(void)
{
    sysinfo.terminalStatus &= ~0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 0);
}

/**************************************************
@bref		bit2 ��λ�����
@param
@return
@note
**************************************************/

void terminalCharge(void)
{
    sysinfo.terminalStatus |= 0x04;
}
/**************************************************
@bref		bit2 �����δ���
@param
@return
@note
**************************************************/

void terminalunCharge(void)
{
    sysinfo.terminalStatus &= ~0x04;
}

/**************************************************
@bref		��ȡ���״̬
@param
@return
	>0		���
	0		δ���
@note
**************************************************/

uint8_t getTerminalChargeState(void)
{
    return (sysinfo.terminalStatus & 0x04);
}

/**************************************************
@bref		bit 3~5 ������Ϣ
@param
@return
@note
**************************************************/

void terminalAlarmSet(TERMINAL_WARNNING_TYPE alarm)
{
    sysinfo.terminalStatus &= ~(0x38);
    sysinfo.terminalStatus |= (alarm << 3);
}

/**************************************************
@bref		bit6 ��λ���Ѷ�λ
@param
@return
@note
**************************************************/

void terminalGPSFixed(void)
{
    sysinfo.terminalStatus |= 0x40;
}

/**************************************************
@bref		bit6 �����δ��λ
@param
@return
@note
**************************************************/

void terminalGPSUnFixed(void)
{
    sysinfo.terminalStatus &= ~0x40;
}

/**************************************************
@bref		LED1 ��������
@param
@return
@note
**************************************************/

static void sysLed1Run(void)
{
    static uint8_t tick = 0;


    if (sysledinfo.sys_led1_on_time == 0)
    {
        SYS_LED1_OFF;
        return;
    }
    else if (sysledinfo.sys_led1_off_time == 0)
    {
        SYS_LED1_ON;
        return;
    }

    tick++;
    if (sysledinfo.sys_led1_onoff == 1) //on status
    {
        SYS_LED1_ON;
        if (tick >= sysledinfo.sys_led1_on_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 0;
        }
    }
    else   //off status
    {
        SYS_LED1_OFF;
        if (tick >= sysledinfo.sys_led1_off_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 1;
        }
    }
}

/**************************************************
@bref		���õƵ���˸Ƶ��
@param
@return
@note
**************************************************/

static void ledSetPeriod(uint8_t ledtype, uint8_t on_time, uint8_t off_time)
{
    if (ledtype == GPSLED1)
    {
        //ϵͳ�źŵ�
        sysledinfo.sys_led1_on_time = on_time;
        sysledinfo.sys_led1_off_time = off_time;
    }
}

/**************************************************
@bref		����ϵͳ��״̬
@param
@return
@note
**************************************************/

void ledStatusUpdate(uint8_t status, uint8_t onoff)
{
    if (onoff == 1)
    {
        sysinfo.sysLedState |= status;
    }
    else
    {
        sysinfo.sysLedState &= ~status;
    }
    if ((sysinfo.sysLedState & SYSTEM_LED_RUN) == SYSTEM_LED_RUN)
    {

        //����
        ledSetPeriod(GPSLED1, 10, 10);
        if ((sysinfo.sysLedState & SYSTEM_LED_NETOK) == SYSTEM_LED_NETOK)
        {
            //����
            ledSetPeriod(GPSLED1, 1, 9);
            if ((sysinfo.sysLedState & SYSTEM_LED_GPSOK) == SYSTEM_LED_GPSOK)
            {
                //��ͨ�Ƴ���
                ledSetPeriod(GPSLED1, 1, 0);
            }
        }

    }
    else if ((sysinfo.sysLedState & SYSTEM_LED_BLE) == SYSTEM_LED_BLE)
    {
		ledSetPeriod(GPSLED1, 5, 5);
    }
    else
    {
        SYS_LED1_OFF;
        ledSetPeriod(GPSLED1, 0, 1);
    }
}

/**************************************************
@bref		�ƿ�����
@param
@return
@note
**************************************************/

static void ledTask(void)
{
    if (sysinfo.sysTick >= 300 && sysparam.ledctrl == 0)
    {
        SYS_LED1_OFF;
        return;
    }
    sysLed1Run();
}
/**************************************************
@bref		gps��������
@param
@return
@note
**************************************************/
void gpsRequestSet(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestSet==>0x%04X", flag);
    sysinfo.gpsRequest |= flag;
}

/**************************************************
@bref		gps�������
@param
@return
@note
**************************************************/

void gpsRequestClear(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestClear==>0x%04X", flag);
    sysinfo.gpsRequest &= ~flag;
}

uint32_t gpsRequestGet(uint32_t flag)
{
    return sysinfo.gpsRequest & flag;
}

/**************************************************
@bref		gps����״̬���л�
@param
@return
@note
**************************************************/

static void gpsChangeFsmState(uint8_t state)
{
    sysinfo.gpsFsm = state;
}

/**************************************************
@bref		ģ���gps����
@param
@return
@note
**************************************************/

static void gpsConfig(void)
{
    sendModuleCmd(QGPSCFG_CMD, "\"apflash\",1");
    sendModuleCmd(QGPSCFG_CMD, "\"gnssconfig\",1");
}


/**************************************************
@bref		gps���ݽ���
@param
@return
@note
**************************************************/

void gpsUartRead(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[UART_RECV_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > UART_RECV_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            if (sysinfo.nmeaOutPutCtl)
            {
                LogWL(DEBUG_GPS, gpsRestore + begin, i - begin);
                LogWL(DEBUG_GPS, "\r\n", 2);
                
            }
            nmeaParser(gpsRestore + begin, i - begin);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}

/**************************************************
@bref		�л��п�΢NMEA���
@param
@return
@note
**************************************************/

static void gpsCfg(void)
{
    char param[50];
    sprintf(param, "$PCAS03,1,0,1,0,1,0,0,0,0,0,0,0,0,0*03\r\n");
    portUartSend(&usart0_ctl, (uint8_t *)param, strlen(param));
    LogMessage(DEBUG_ALL, "gps config nmea output");
}

/**************************************************
@bref		�л��п�΢������Ϊ115200
@param
@return
@note
**************************************************/

//$PCAS03,1,0,1,1,1,0,0,0,0,0,0,0,0,0*02
static void changeGPSBaudRate(void)
{
    char param[50];
    sprintf(param, "$PCAS01,5*19\r\n");
    portUartSend(&usart0_ctl, (uint8_t *)param, strlen(param));
    portUartCfg(APPUSART0, 1, 115200, gpsUartRead);
    LogMessage(DEBUG_ALL, "gps config baudrate to 115200");
    startTimer(10, gpsCfg, 0);
}

/**************************************************
@bref		����gps
@param
@return
@note
**************************************************/

static void gpsOpen(void)
{
//    GPSPWR_ON;
//    GPSLNA_ON;
//    portUartCfg(APPUSART0, 1, 9600, gpsUartRead);
//    startTimer(30, changeGPSBaudRate, 0);
//    sysinfo.gpsUpdatetick = sysinfo.sysTick;
//    sysinfo.gpsOnoff = 1;
//    gpsChangeFsmState(GPSWATISTATUS);
//    gpsClearCurrentGPSInfo();
//    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
//    moduleSleepCtl(0);
//    LogMessage(DEBUG_ALL, "gpsOpen");

}
/**************************************************
@bref		�ȴ�gps�ȶ�
@param
@return
@note
**************************************************/

static void gpsWait(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 4)
    {
        runTick = 0;
        gpsChangeFsmState(GPSOPENSTATUS);
		//�ȴ�gps������ϣ��л�115200
        agpsRequestSet();
    }
}

/**************************************************
@bref		�ر�gps
@param
@return
@note
**************************************************/

static void gpsClose(void)
{
//    GPSPWR_OFF;
//    GPSLNA_OFF;
//    sysinfo.rtcUpdate = 0;
//    sysinfo.gpsOnoff = 0;
//    gpsClearCurrentGPSInfo();
//    terminalGPSUnFixed();
//    gpsChangeFsmState(GPSCLOSESTATUS);
//    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
//    if (primaryServerIsReady())
//    {
//        moduleSleepCtl(1);
//    }
//    LogMessage(DEBUG_ALL, "gpsClose");
}

/**************************************************
@bref		gps�ڵȴ�ģʽʱ��ָ�ͨ
@param
@return
@note
�ڷ�����gpsָ���EC800M ģ������һ��ʱ��Ŀ��٣�Ϊ���ⷢ��ָ������Ӧ�������ط���ָ����Ҫ�ӳ�һ��
**************************************************/

uint8_t gpsInWait(void)
{
    if (sysinfo.gpsFsm == GPSWATISTATUS)
    {
        return 1;
    }
    return 0;
}

/**************************************************
@bref		gps��������
@param
@return
@note
**************************************************/

static void gpsRequestTask(void)
{
    gpsinfo_s *gpsinfo;
    switch (sysinfo.gpsFsm)
    {
        case GPSCLOSESTATUS:
            //���豸���󿪹�
            if (sysinfo.gpsRequest != 0)
            {
                gpsOpen();
            }
            break;
        case GPSWATISTATUS:
            gpsWait();
            break;
        case GPSOPENSTATUS:
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus)
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 1);
            }
            else
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
            }
            if (sysinfo.gpsRequest == 0 || (sysinfo.sysTick - sysinfo.gpsUpdatetick) >= 20)
            {
                if ((sysinfo.sysTick - sysinfo.gpsUpdatetick) >= 20)
                {
                    LogMessage(DEBUG_ALL, "no nmea ouput");
                }
                gpsClose();
            }
            break;
        default:
            gpsChangeFsmState(GPSCLOSESTATUS);
            break;
    }
}

/**************************************************
@bref		����һ��gpsλ��
@param
@return
@note
**************************************************/

static void gpsUplodOnePointTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t runtick = 0;
    static uint8_t uploadtick = 0;
    //�ж��Ƿ���������¼�
    if (sysinfo.gpsOnoff == 0)
        return;
    if (gpsRequestGet(GPS_REQUEST_UPLOAD_ONE) == 0)
    {
        runtick = 0;
        uploadtick = 0;
        return;
    }
    gpsinfo = getCurrentGPSInfo();
    runtick++;
    if (gpsinfo->fixstatus == 0)
    {
        uploadtick = 0;
        if (runtick >= sysinfo.gpsuploadonepositiontime)
        {
            runtick = 0;
            uploadtick = 0;
            gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
        }
        return;
    }
    runtick = 0;
    if (++uploadtick >= 10)
    {
        uploadtick = 0;
        if (sysinfo.flag123)
        {
            dorequestSend123();
        }
        protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
    }

}

/**************************************************
@bref		������������
@param
@return
@note
**************************************************/
void alarmRequestSet(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestSet==>0x%04X", request);
    sysinfo.alarmRequest |= request;
}
/**************************************************
@bref		�����������
@param
@return
@note
**************************************************/

void alarmRequestClear(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestClear==>0x%04X", request);
    sysinfo.alarmRequest &= ~request;
}

/**************************************************
@bref		��������
@param
@return
@note
**************************************************/

void alarmRequestTask(void)
{
    uint8_t alarm;
    if (primaryServerIsReady() == 0 || sysinfo.alarmRequest == 0)
    {
        return;
    }
    if (getTcpNack() != 0)
    {
        return;
    }
    //�йⱨ��
    if (sysinfo.alarmRequest & ALARM_LIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_LIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Light Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LIGHT);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�͵籨��
    if (sysinfo.alarmRequest & ALARM_LOWV_REQUEST)
    {
        alarmRequestClear(ALARM_LOWV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>LowVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOWV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�ϵ籨��
    if (sysinfo.alarmRequest & ALARM_LOSTV_REQUEST)
    {
        alarmRequestClear(ALARM_LOSTV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>lostVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOSTV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //SOS����
    if (sysinfo.alarmRequest & ALARM_SOS_REQUEST)
    {
        alarmRequestClear(ALARM_SOS_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>SOS Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SOS);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�����ٱ���
    if (sysinfo.alarmRequest & ALARM_ACCLERATE_REQUEST)
    {
        alarmRequestClear(ALARM_ACCLERATE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid Accleration Alarm");
        alarm = 9;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //�����ٱ���
    if (sysinfo.alarmRequest & ALARM_DECELERATE_REQUEST)
    {
        alarmRequestClear(ALARM_DECELERATE_REQUEST);
        LogMessage(DEBUG_ALL,
                   "alarmRequestTask==>Rapid Deceleration Alarm");
        alarm = 10;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //����ת����
    if (sysinfo.alarmRequest & ALARM_RAPIDLEFT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDLEFT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid LEFT Alarm");
        alarm = 11;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //����ת����
    if (sysinfo.alarmRequest & ALARM_RAPIDRIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDRIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid RIGHT Alarm");
        alarm = 12;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

	//��������
	if (sysinfo.alarmRequest & ALARM_BLEALARM_REQUEST)
	{
        alarmRequestClear(ALARM_BLEALARM_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>BLE Alarm");
        alarm = 0x18;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
	}
}



/**************************************************
@bref		�����˶���ֹ״̬
@param
	src 		�����Դ
	newState	��״̬
@note
**************************************************/

static void motionStateUpdate(motion_src_e src, motionState_e newState)
{
    char type[20];


    if (motionInfo.motionState == newState)
    {
        return;
    }
    motionInfo.motionState = newState;
    switch (src)
    {
        case ACC_SRC:
            strcpy(type, "acc");
            break;
        case VOLTAGE_SRC:
            strcpy(type, "voltage");
            break;
        case GSENSOR_SRC:
            strcpy(type, "gsensor");
            break;
        default:
            return;
            break;
    }
    LogPrintf(DEBUG_ALL, "Device %s , detected by %s", newState == MOTION_MOVING ? "moving" : "static", type);

    if (newState)
    {
        netResetCsqSearch();
        if (sysparam.gpsuploadgap != 0)
        {
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
            {
                gpsRequestSet(GPS_REQUEST_ACC_CTL);
            }
        }
        terminalAccon();
        hiddenServerCloseClear();
    }
    else
    {
        if (sysparam.gpsuploadgap != 0)
        {
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            gpsRequestClear(GPS_REQUEST_ACC_CTL);
        }
        terminalAccoff();
        updateRTCtimeRequest();
    }
    if (primaryServerIsReady())
    {
        protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                            sysparam.startUpCnt, sysparam.runTime);
        protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
        jt808SendToServer(TERMINAL_POSITION, getLastFixedGPSInfo());
    }
}


/**************************************************
@bref		���ж�
@param
@note
**************************************************/

void motionOccur(void)
{
    motionInfo.tapInterrupt++;
}
/**************************************************
@bref		ͳ��ÿһ����жϴ���
@param
@note
**************************************************/

static void motionCalculate(void)
{
    motionInfo.ind = (motionInfo.ind + 1) % sizeof(motionInfo.tapCnt);
    motionInfo.tapCnt[motionInfo.ind] = motionInfo.tapInterrupt;
    motionInfo.tapInterrupt = 0;
}
/**************************************************
@bref		��ȡ�����n����𶯴���
@param
@note
**************************************************/

static uint16_t motionGetTotalCnt(void)
{
    uint16_t cnt;
    uint8_t i;
    cnt = 0;
    for (i = 0; i < sizeof(motionInfo.tapCnt); i++)
    {
        cnt += motionInfo.tapCnt[i];
    }
    return cnt;
}

/**************************************************
@bref		��ȡ�˶�״̬
@param
@note
**************************************************/

motionState_e motionGetStatus(void)
{
    return motionInfo.motionState;
}


/**************************************************
@bref		�˶��;�ֹ���ж�
@param
@note
**************************************************/

static void motionCheckTask(void)
{
    static uint16_t gsStaticTick = 0;
    static uint16_t autoTick = 0;
    static uint8_t  accOnTick = 0;
    static uint8_t  accOffTick = 0;
    static uint8_t fixTick = 0;

    static uint8_t  volOnTick = 0;
    static uint8_t  volOffTick = 0;

    gpsinfo_s *gpsinfo;

    uint16_t totalCnt, staticTime;

    motionCalculate();

    if (sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
    {
        staticTime = 300;
    }
    else
    {
        staticTime = 180;
    }


    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
        terminalAccoff();
        if (gpsRequestGet(GPS_REQUEST_ACC_CTL))
        {
            gpsRequestClear(GPS_REQUEST_ACC_CTL);
        }
        gsStaticTick = 0;
        return ;
    }

    //�����˶�״̬ʱ�����gap����Max�����������ϱ�gps
    if (getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    {
        if (++autoTick >= sysparam.gpsuploadgap)
        {
            autoTick = 0;
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        }
    }
    else
    {
        autoTick = 0;
    }


    totalCnt = motionGetTotalCnt();
    if (ACC_READ == ACC_STATE_ON)
    {
        //����Զ�ǵ�һ���ȼ�
        if (++accOnTick >= 10)
        {
            accOnTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_MOVING);
        }
        accOffTick = 0;
        return;
    }
    accOnTick = 0;
    if (sysparam.accdetmode == ACCDETMODE0)
    {
        //����acc�߿���
        if (++accOffTick >= 10)
        {
            accOffTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_STATIC);
        }
        return;
    }

    if (sysparam.accdetmode == ACCDETMODE1)
    {
        //��acc��+��ѹ����
        if (sysinfo.outsidevoltage >= sysparam.accOnVoltage)
        {
            if (++volOnTick >= 15)
            {
                volOnTick = 0;
                motionStateUpdate(VOLTAGE_SRC, MOTION_MOVING);
            }
        }
        else
        {
            volOnTick = 0;
        }

        if (sysinfo.outsidevoltage < sysparam.accOffVoltage)
        {
            if (++volOffTick >= 15)
            {
                volOffTick = 0;
                motionStateUpdate(MOTION_MOVING, MOTION_STATIC);
            }
        }
        else
        {
            volOffTick = 0;
        }
        return;
    }
    //ʣ�µģ���acc��+gsensor����

    if (totalCnt >= 7)
    {
        motionStateUpdate(GSENSOR_SRC, MOTION_MOVING);
    }
    if (totalCnt <= 1)
    {
        if (sysinfo.gpsOnoff)
        {
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus && gpsinfo->speed >= 7)
            {
                if (++fixTick >= 5)
                {
                    gsStaticTick = 0;
                }
            }
            else
            {
                fixTick = 0;
            }
        }
        gsStaticTick++;
        if (gsStaticTick >= staticTime)
        {
            motionStateUpdate(GSENSOR_SRC, MOTION_STATIC);
        }
    }
    else
    {
        gsStaticTick = 0;
    }
}



/**************************************************
@bref		��ѹ�������
@param
@return
@note
**************************************************/

static void voltageCheckTask(void)
{
    static uint16_t lowpowertick = 0;
    static uint8_t  lowwflag = 0;
    float x;

    if (isModeRun() == 0)
    {
		return;
    }
    x = portGetAdcVol(ADC_CHANNEL);
    sysinfo.outsidevoltage = x ;
    sysinfo.insidevoltage = sysinfo.outsidevoltage;
	//LogPrintf(DEBUG_ALL, "x:%.2f, outvol:%.2f", x, sysinfo.outsidevoltage);
    //�͵籨��
//    if (sysinfo.outsidevoltage < sysinfo.lowvoltage)
//    {
//        lowpowertick++;
//        if (lowpowertick >= 30)
//        {
//            if (lowwflag == 0)
//            {
//                lowwflag = 1;
//                LogPrintf(DEBUG_ALL, "power supply too low %.2fV", sysinfo.outsidevoltage);
//                //�͵籨��
//                jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 1);
//                alarmRequestSet(ALARM_LOWV_REQUEST);
//                lbsRequestSet(DEV_EXTEND_OF_MY);
//                wifiRequestSet(DEV_EXTEND_OF_MY);
//                gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//            }
//
//        }
//    }
//    else
//    {
//        lowpowertick = 0;
//    }


//    if (sysinfo.outsidevoltage >= sysinfo.lowvoltage + 0.5)
//    {
//        lowwflag = 0;
//        jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 0);
//    }
}

/**************************************************
@bref		ģʽ״̬���л�
@param
@return
@note
**************************************************/

static void changeModeFsm(uint8_t fsm)
{
    sysinfo.runFsm = fsm;
}

/**************************************************
@bref		���ٹر�
@param
@return
@note
**************************************************/

static void modeShutDownQuickly(void)
{
    static uint8_t delaytick = 0;
    if (sysinfo.wifiExtendEvt == 0 && sysinfo.lbsExtendEvt == 0)
    {
        delaytick++;
        if (delaytick >= 5)
        {
            LogMessage(DEBUG_ALL, "modeShutDownQuickly==>shutdown");
            delaytick = 0;
            changeModeFsm(MODE_STOP); //ִ����ϣ��ػ�
        }
    }
    else
    {
        delaytick = 0;
    }
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

void modeTryToStop(void)
{
    sysinfo.gpsRequest = 0;
    changeModeFsm(MODE_STOP);
}

/**************************************************
@bref		����ɨ��
@param
@return
@note
**************************************************/

static void modeScan(void)
{
//    static uint8_t runTick = 0;
//    scanList_s  *list;
//    if (sysparam.leavealm == 0 || (sysparam.MODE != MODE1 && sysparam.MODE != MODE3))
//    {
//        runTick = 0;
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (runTick == 1)
//    {
//        portFsclkChange(1);
//        bleCentralDiscovery();
//    }
//    else if (runTick >= 20)
//    {
//        runTick = 0;
//        list = scanListGet();
//        if (list->cnt == 0)
//        {
//            alarmRequestSet(ALARM_LEAVE_REQUEST);
//        }
//        changeModeFsm(MODE_START);
//    }
//    runTick++;
}


/**************************************************
@bref		����״̬���л�
@param
@return
@note
**************************************************/

static void bleChangeFsm(modeChoose_e fsm)
{
    bleTry.runFsm = fsm;
    bleTry.runTick = 0;
}

/**************************************************
@bref		ɨ����ɻص�
@param
@return
@note
**************************************************/

void bleScanCallBack(deviceScanList_s *list)
{
    uint8_t i;
    for (i = 0; i < list->cnt; i++)
    {
        if (my_strpach(list->list[i].broadcaseName, "AUTO"))
        {
            LogPrintf(DEBUG_ALL, "Find Ble [%s],rssi:%d", list->list[i].broadcaseName, list->list[i].rssi);
            tmos_memcpy(bleTry.mac, list->list[i].addr, 6);
            bleTry.addrType = list->list[i].addrType;
            bleChangeFsm(BLE_CONN);
            return;
        }
    }
    LogMessage(DEBUG_ALL, "no find my ble");
    bleChangeFsm(BLE_SCAN);
}

/**************************************************
@bref		������ɵ�
@param
@return
@note
**************************************************/

void bleConnCallBack(void)
{
    LogMessage(DEBUG_ALL, "connect success");
    bleChangeFsm(BLE_READY);
    sysparam.bleLinkCnt = 0;
    paramSaveAll();
    tmos_set_event(appCentralTaskId, APP_WRITEDATA_EVENT);
}


/**************************************************
@bref		ģʽѡ�񣬽�ģʽһ�����£������������ܲ�������
@param
@return
@note
**************************************************/

static void modeChoose(void)
{
	static uint8_t flag = 0;
    if (sysparam.MODE != MODE1 && sysparam.MODE != MODE3)
    {
        bleChangeFsm(BLE_IDLE);
        changeModeFsm(MODE_START);
        return;
    }
//    if (sysinfo.alarmRequest != 0)
//    {
//        bleChangeFsm(BLE_IDLE);
//        changeModeFsm(MODE_START);
//        sysparam.startUpCnt++;
//        paramSaveAll();
//        return;
//    }
	if (sysparam.bleLinkFailCnt == 0)
	{
		LogPrintf(DEBUG_ALL, "modeChoose==>Ble disable");
		bleChangeFsm(BLE_IDLE);
        changeModeFsm(MODE_START);
        return;
	}
    if (sysinfo.first == 0)
    {
    	LogPrintf(DEBUG_ALL, "modeChoose==>First start");
        sysinfo.first = 1;
        bleChangeFsm(BLE_IDLE);
        changeModeFsm(MODE_START);
        sysparam.startUpCnt++;
        paramSaveAll();
        return;
    }
    if (flag == 0)
    {
    	flag = 1;
		
    }
    switch (bleTry.runFsm)
    {
        case BLE_IDLE:
            sysparam.startUpCnt++;
            sysparam.bleLinkCnt++;
            paramSaveAll();
            //wakeUpByInt(2, 30);
            ledStatusUpdate(SYSTEM_LED_BLE, 1);
   
            portSetNextAlarmTime();
            bleChangeFsm(BLE_SCAN);
            bleTry.scanCnt = 0;
            paramSaveAll();
            break;
        case BLE_SCAN:
            bleTry.connCnt = 0;
            if (bleTry.scanCnt++ < 3)
            {
                //����ɨ��
                centralStartDisc();
                bleChangeFsm(BLE_SCAN_WAIT);
            }
            else
            {
                bleTry.scanCnt = 0;
                //ɨ��ʧ��
                if (sysparam.bleLinkCnt >= sysparam.bleLinkFailCnt)
                {
                    LogPrintf(DEBUG_ALL, "scan fail==>%d", sysparam.bleLinkCnt);
                    alarmRequestSet(ALARM_BLEALARM_REQUEST);
                    sysparam.bleLinkCnt = 0;
                    paramSaveAll();
                    changeModeFsm(MODE_START);
                    bleChangeFsm(BLE_IDLE);
                    flag = 0;
                }
                else
                {
                    LogPrintf(DEBUG_ALL, "scan cnt==>%d", sysparam.bleLinkCnt);
                    bleChangeFsm(BLE_DONE);
                }
            }
            break;
        case BLE_SCAN_WAIT:
            //�ȴ�ɨ����
            if (++bleTry.runTick >= 12)
            {
                bleChangeFsm(BLE_SCAN);
            }
            break;
        case BLE_CONN:
            //��ʼ��������
            if (bleTry.connCnt++ < 3)
            {
                centralEstablish(bleTry.mac, bleTry.addrType);
                bleChangeFsm(BLE_CONN_WAIT);
            }
            else
            {
                bleTry.connCnt = 0;
                if (sysparam.bleLinkCnt >= sysparam.bleLinkFailCnt)
                {
                    LogPrintf(DEBUG_ALL, "conn fail==>%d", sysparam.bleLinkCnt);
                    sysparam.bleLinkCnt = 0;
                    alarmRequestSet(ALARM_BLEALARM_REQUEST);
                    paramSaveAll();
                    changeModeFsm(MODE_START);
                    bleChangeFsm(BLE_IDLE);
                    flag = 0;
                }
                else
                {
                    bleChangeFsm(BLE_DONE);
                }
            }
            break;
        case BLE_CONN_WAIT:
            //�ȴ����ӽ��
            if (++bleTry.runTick >= 12)
            {
                centralTerminate();
                bleChangeFsm(BLE_CONN);
            }
            break;
        case BLE_READY:
            //�������ӳɹ�
            if (++bleTry.runTick >= 20)
            {
                centralTerminate();
                bleChangeFsm(BLE_DONE);
            }
            break;
        case BLE_DONE:
            ledStatusUpdate(SYSTEM_LED_BLE, 0);
            bleChangeFsm(BLE_IDLE);
            changeModeFsm(MODE_DONE);
            gpsRequestClear(GPS_REQUEST_ALL);
            flag = 0;
            break;
        default:
            bleChangeFsm(BLE_IDLE);
            break;
    }
}




/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeStart(void)
{
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    LogPrintf(DEBUG_ALL, "modeStart==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
    sysinfo.runStartTick = sysinfo.sysTick;
    sysinfo.gpsuploadonepositiontime = 180;
    updateRTCtimeRequest();
    //portFsclkChange(0);
    switch (sysparam.MODE)
    {
        case MODE1:
            portGsensorCtl(0);
            sysparam.startUpCnt++;
            portSetNextAlarmTime();
            break;
        case MODE2:
            portGsensorCtl(1);
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            break;
        case MODE3:
            portGsensorCtl(0);
            sysparam.startUpCnt++;
            break;
        case MODE21:
            portGsensorCtl(1);
            portSetNextAlarmTime();
            break;
        case MODE23:
            portGsensorCtl(1);
            break;
        default:
            sysparam.MODE = MODE2;
            paramSaveAll();
            break;
    }
    lbsRequestSet(DEV_EXTEND_OF_MY);
    wifiRequestSet(DEV_EXTEND_OF_MY);
    gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
    ledStatusUpdate(SYSTEM_LED_RUN, 1);
    modulePowerOn();
    netResetCsqSearch();
    changeModeFsm(MODE_RUNING);
}

static void sysRunTimeCnt(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 180)
    {
        runTick = 0;
        sysparam.runTime++;
        paramSaveAll();
    }
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeRun(void)
{
    static uint8_t runtick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE3:
            //��ģʽ�¹���3�ְ���
            if ((sysinfo.sysTick - sysinfo.runStartTick) >= 150)
            {
                gpsRequestClear(GPS_REQUEST_ALL);
                alarmRequestClear(ALARM_ALL_REQUEST);
                changeModeFsm(MODE_STOP);
            }
            modeShutDownQuickly();
            break;
        case MODE2:
            //��ģʽ��ÿ��3���Ӽ�¼ʱ��
            sysRunTimeCnt();
            gpsUploadPointToServer();
            break;
        case MODE21:
        case MODE23:
            //��ģʽ����gps����ʱ���Զ��ػ�
            sysRunTimeCnt();
            modeShutDownQuickly();
            gpsUploadPointToServer();
            break;
        default:
            LogMessage(DEBUG_ALL, "mode change unknow");
            sysparam.MODE = MODE2;
            break;
    }
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeStop(void)
{
    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
        portGsensorCtl(0);
    }
    ledStatusUpdate(SYSTEM_LED_RUN, 0);
    modulePowerOff();
    changeModeFsm(MODE_DONE);
}

/**************************************************
@bref		�ȴ�����ģʽ
@param
@return
@note
**************************************************/

static void modeDone(void)
{
	if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
    	//��һ�������Ǳ�ִ֤����˯��֮ǰ��IO����
    	//�ڶ��������Ǳ�֤ģ��ػ�
    	if (sysinfo.sleep && isModulePowerOn() == 0)
    	{
			tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
    	}
    }
}

/**************************************************
@bref		��ǰ�Ƿ�Ϊ����ģʽ
@param
@return
	1		��
	0		��
@note
**************************************************/

uint8_t isModeRun(void)
{
    if (sysinfo.runFsm == MODE_RUNING)
        return 1;
    return 0;
}
/**************************************************
@bref		��ǰ�Ƿ�Ϊdoneģʽ
@param
@return
	1		��
	0		��
@note
**************************************************/

uint8_t isModeDone(void)
{
    if (sysinfo.runFsm == MODE_DONE)
        return 1;
    return 0;
}


/**************************************************
@bref		ϵͳ��ʱ�Զ�����
@param
@return
@note
**************************************************/

static void sysAutoReq(void)
{
    uint16_t year;
    uint8_t month, date, hour, minute, second;

    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE21)
    {
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (date == sysinfo.alarmDate && hour == sysinfo.alarmHour && minute == sysinfo.alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            //gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysinfo.kernalRun == 0)
            {
            	changeModeFsm(MODE_CHOOSE);
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
    }
    else
    {
        if (sysparam.gapMinutes != 0)
        {
            sysinfo.sysMinutes++;
            LogPrintf(DEBUG_ALL, "sysAutoReq==>sysMinutes:%d", sysinfo.sysMinutes);
            if ((sysinfo.sysMinutes - sysparam.gapMinutes) == 0)
            {
            	sysinfo.sysMinutes = 0;
                //gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
                LogMessage(DEBUG_ALL, "upload period");
                if (sysinfo.kernalRun == 0)
                {
                	changeModeFsm(MODE_CHOOSE);
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
            }
        }
    }
}

/**************************************************
@bref		ģʽ��������
@param
@return
@note
**************************************************/

static void sysModeRunTask(void)
{
    switch (sysinfo.runFsm)
    {
    	case MODE_CHOOSE:
			modeChoose();
    		break;
        case MODE_START:
            modeStart();
            break;
        case MODE_RUNING:
            modeRun();
            break;
        case MODE_STOP:
            modeStop();
            break;
        case MODE_DONE:
            modeDone();
            break;
    }
}

/**************************************************
@bref		��վ��������
@param
@return
@note
**************************************************/

void lbsRequestSet(uint8_t ext)
{
    sysinfo.lbsRequest = 1;
    sysinfo.lbsExtendEvt |= ext;
}

static void sendLbs(void)
{
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_MY)
    {
        protocolSend(NORMAL_LINK, PROTOCOL_19, NULL);
    }
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_BLE)
    {
        protocolSend(BLE_LINK, PROTOCOL_19, NULL);
    }
    sysinfo.lbsExtendEvt = 0;
}
/**************************************************
@bref		��վ��������
@param
@return
@note
**************************************************/

static void lbsRequestTask(void)
{
    if (sysinfo.lbsRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    sysinfo.lbsRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
        moduleGetLbs();
        startTimer(70, sendLbs, 0);
    }
}

/**************************************************
@bref		����WIFI��������
@param
@return
@note
**************************************************/

void wifiRequestSet(uint8_t ext)
{
    sysinfo.wifiRequest = 1;
    sysinfo.wifiExtendEvt |= ext;
}


/**************************************************
@bref		WIFI��������
@param
@return
@note
**************************************************/

static void wifiRequestTask(void)
{
    if (sysinfo.wifiRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0)
        return;

    sysinfo.wifiRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
        startTimer(80, moduleGetWifiScan, 0);
    }
}

/**************************************************
@bref		�����豸
@param
@return
@note
**************************************************/
void wakeUpByInt(uint8_t     type, uint8_t sec)
{
    switch (type)
    {
        case 0:
            sysinfo.ringWakeUpTick = sec;
            break;
        case 1:
            sysinfo.cmdTick = sec;
            break;
    }

    portSleepDn();
}

/**************************************************
@bref		��ѯ�Ƿ���Ҫ����
@param
@return
@note
**************************************************/

static uint8_t getWakeUpState(void)
{
    //��ӡ������Ϣʱ��������
    if (sysinfo.logLevel == DEBUG_FACTORY)
    {
        return 1;
    }
    //δ������������
    if (primaryServerIsReady() == 0 && isModeRun())
    {
        return 2;
    }
    if (sysinfo.ringWakeUpTick != 0)
    {
        return 4;
    }
    if (sysinfo.cmdTick != 0)
    {
        return 5;
    }
    //��0 ʱǿ�Ʋ�����
    return 0;
}

/**************************************************
@bref		�Զ�����
@param
@return
@note
**************************************************/

void autoSleepTask(void)
{
    static uint8_t flag = 0;
    if (sysinfo.ringWakeUpTick != 0)
    {
        sysinfo.ringWakeUpTick--;
    }
    if (sysinfo.cmdTick != 0)
    {
        sysinfo.cmdTick--;
    }
    if (getWakeUpState())
    {
        portSleepDn();
        if (flag != 0)
        {
            flag = 0;
            //portFsclkChange(0);
            LogMessage(DEBUG_ALL, "disable sleep");
            tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
            sysinfo.sleep = 0;
        }
    }
    else
    {
        portSleepEn();
        if (flag != 1)
        {
            flag = 1;
            //portFsclkChange(1);
            LogMessage(DEBUG_ALL, "enable sleep");
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            sysinfo.sleep = 1;
        }
    }
}

/**************************************************
@bref		ÿ������
@param
@note
**************************************************/

static void rebootEveryDay(void)
{
    sysinfo.sysTick++;
    //    if (sysinfo.sysTick < 86400)
    //        return ;
    //    if (sysinfo.gpsRequest != 0)
    //        return ;
    //    portSysReset();
}

/**************************************************
@bref		1������
@param
@return
@note
**************************************************/

void taskRunInSecond(void)
{
    rebootEveryDay();
    netConnectTask();
    voltageCheckTask();
    alarmRequestTask();
    lbsRequestTask();
    wifiRequestTask();
    sysModeRunTask();
    serverManageTask();
    autoSleepTask();
}


/**************************************************
@bref		���ڵ��Խ���
@param
@return
@note
**************************************************/
void doDebugRecvPoll(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[DEBUG_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > DEBUG_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            atCmdParserFunction(gpsRestore + begin, i - begin + 1);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}

/**************************************************
@bref		ϵͳ����ʱ����
@param
@return
@note
**************************************************/

void myTaskPreInit(void)
{
    tmos_memset(&sysinfo, 0, sizeof(sysinfo));
    //sysinfo.logLevel = 9;
    SetSysClock(CLK_SOURCE_PLL_60MHz);
    portGpioSetDefCfg();
    portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);

    portModuleGpioCfg(1);
    portLedGpioCfg();
    portAdcCfg(1);
    portWdtCfg();
    
    paramInit();
    socketListInit();
    createSystemTask(ledTask, 1);
    createSystemTask(outputNode, 2);
    sysinfo.sysTaskId = createSystemTask(taskRunInSecond, 10);

}

/**************************************************
@bref		tmos ����ص�
@param
@return
@note
**************************************************/

static tmosEvents myTaskEventProcess(tmosTaskID taskID, tmosEvents events)
{
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = tmos_msg_receive(sysinfo.taskId)) != NULL)
        {
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & APP_TASK_KERNAL_EVENT)
    {
        kernalRun();
        portWdtFeed();
        return events ^ APP_TASK_KERNAL_EVENT;
    }

    if (events & APP_TASK_POLLUART_EVENT)
    {
        pollUartData();
        portWdtFeed();
        return events ^ APP_TASK_POLLUART_EVENT;
    }

    if (events & APP_TASK_RUN_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal start");
        
        sysinfo.kernalRun = 1;
        portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);

	    portModuleGpioCfg(1);
	    //portGpsGpioCfg();
	    portLedGpioCfg();
    	portAdcCfg(1);
        tmos_start_reload_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT, MS1_TO_SYSTEM_TIME(100));
        return events ^ APP_TASK_RUN_EVENT;
    }
    if (events & APP_TASK_STOP_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal stop");
        portGpioSetDefCfg();
        portAdcCfg(0);
		portUartCfg(APPUSART2, 0, 115200, NULL);
        // �ֶ�����Boot����
//        GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeOut_PP_5mA);
//        GPIOB_ResetBits(GPIO_Pin_22);
        sysinfo.kernalRun = 0;
        tmos_stop_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT);
        return events ^ APP_TASK_STOP_EVENT;
    }
    if (events & APP_TASK_ONEMINUTE_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task one minutes");
        sysAutoReq();
        return events ^ APP_TASK_ONEMINUTE_EVENT;
    }
    return 0;
}

/**************************************************
@bref		�����ʼ��
@param
@return
@note
**************************************************/

void myTaskInit(void)
{
    sysinfo.taskId = TMOS_ProcessEventRegister(myTaskEventProcess);
    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_ONEMINUTE_EVENT, MS1_TO_SYSTEM_TIME(60000));
}


#include <app_protocol.h>
#include "app_net.h"
#include "app_jt808.h"
#include "app_db.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_server.h"
#include "app_socket.h"
//������ؽṹ��

static moduleState_s  moduleState;
static moduleCtrl_s moduleCtrl;
static cmdNode_s *headNode = NULL;

static void gpsUploadChangeFsm(uint8_t fsm);
static void gpsUploadSetSize(uint32_t size);

//ָ���
const atCmd_s cmdtable[] =
{
    {AT_CMD, "ATE0"},
    {CPIN_CMD, "AT+CPIN"},
    {CSQ_CMD, "AT+CSQ"},
    {CGREG_CMD, "AT+CGREG"},
    {CEREG_CMD, "AT+CEREG"},
    {QICSGP_CMD, "AT+QICSGP"},
    {QIACT_CMD, "AT+QIACT"},
    {QIDEACT_CMD, "AT+QIDEACT"},
    {QIOPEN_CMD, "AT+QIOPEN"},
    {QISEND_CMD, "AT+QISEND"},
    {CIMI_CMD, "AT+CIMI"},
    {CGSN_CMD, "AT+CGSN"},
    {CCID_CMD, "AT+CCID"},
    {ICCID_CMD, "AT+ICCID"},
    {CMGF_CMD, "AT+CMGF"},
    {CMGR_CMD, "AT+CMGR"},
    {CMGD_CMD, "AT+CMGD"},
    {CMGS_CMD, "AT+CMGS"},
    {CPMS_CMD, "AT+CPMS"},
    {CNMI_CMD, "AT+CNMI"},
    {QSCLK_CMD, "AT+QSCLK"},
    {CFUN_CMD, "AT+CFUN"},
    {QICLOSE_CMD, "AT+QICLOSE"},
    {CGDCONT_CMD, "AT+CGDCONT"},
    {QCFG_CMD, "AT+QCFG"},
    {QICFG_CMD, "AT+QICFG"},
    {QPOWD_CMD, "AT+QPOWD"},
    {QGPS_CMD, "AT+QGPS"},
    {QGPSEND_CMD, "AT+QGPSEND"},
    {QGPSCFG_CMD, "AT+QGPSCFG"},
    {QGPSGNMEA_CMD, "AT+QGPSGNMEA"},
    {QISTATE_CMD, "AT+QISTATE"},
    {QAGPS_CMD, "AT+QAGPS"},
    {QURCCFG_CMD, "AT+QURCCFG"},
    {CGATT_CMD, "AT+CGATT"},
    {QIDNSIP_CMD, "AT+QIDNSIP"},
    {QIMUX_CMD, "AT+QIMUX"},
    {QISACK_CMD, "AT+QISACK"},
    {QINDI_CMD, "AT+QINDI"},
    {QIRD_CMD, "AT+QIRD"},
    {ATA_CMD, "ATA"},
    {CBC_CMD, "AT+CBC"},
    {CIPMUX_CMD, "AT+CIPMUX"},
    {CIPSTART_CMD, "AT+CIPSTART"},
    {CIPCLOSE_CMD, "AT+CIPCLOSE"},
    {CIPSEND_CMD, "AT+CIPSEND"},
    {CIPQSEND_CMD, "AT+CIPQSEND"},
    {CIPRXGET_CMD, "AT+CIPRXGET"},
    {CIPACK_CMD, "AT+CIPACK"},
    {CSCLK_CMD, "AT+CSCLK"},
    {CIPSTATUS_CMD, "AT+CIPSTATUS"},
    {CIPSHUT_CMD, "AT+CIPSHUT"},
    {CFGRI_CMD, "AT+CFGRI"},
    {WIFISCAN_CMD, "AT+WIFISCAN"},
    {CFG_CMD, "AT+CFG"},
};

/**************************************************
@bref		����ָ��������У�����˳�����
@param
@return
@note
**************************************************/

uint8_t createNode(char *data, uint16_t datalen, uint8_t currentcmd)
{
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    //�������ͷδ�������򴴽�����ͷ��
    WAKEMODULE;
    if (currentcmd == WIFISCAN_CMD)
    {
		wakeUpByInt(1, 20);
    }
    else
    {
		wakeUpByInt(1, 10);
    }
    if (headNode == NULL)
    {
        headNode = malloc(sizeof(cmdNode_s));
        if (headNode != NULL)
        {
            headNode->currentcmd = currentcmd;
            headNode->data = NULL;
            headNode->data = malloc(datalen);
            if (headNode->data != NULL)
            {
                memcpy(headNode->data, data, datalen);
                headNode->datalen = datalen;
                headNode->nextnode = NULL;
                return 1;
            }
            else
            {
                free(headNode);
                headNode = NULL;
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
    currentnode = headNode;
    do
    {
        nextnode = currentnode->nextnode;
        if (nextnode == NULL)
        {
            nextnode = malloc(sizeof(cmdNode_s));
            if (nextnode != NULL)
            {

                nextnode->currentcmd = currentcmd;
                nextnode->data = NULL;
                nextnode->data = malloc(datalen);
                if (nextnode->data != NULL)
                {
                    memcpy(nextnode->data, data, datalen);
                    nextnode->datalen = datalen;
                    nextnode->nextnode = NULL;
                    currentnode->nextnode = nextnode;
                    nextnode = nextnode->nextnode;
                }
                else
                {
                    free(nextnode);
                    nextnode = NULL;
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        currentnode = nextnode;
    }
    while (nextnode != NULL);

    return 1;
}

/**************************************************
@bref		���ݶ������
@param
@return
@note
**************************************************/

void outputNode(void)
{
    static uint8_t lockFlag = 0;
    static uint8_t lockTick = 0;
    static uint8_t sleepTick = 0;
    static uint8_t tickRange = 50;
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    if (lockFlag)
    {
        if (lockTick++ >= tickRange)
        {
            lockFlag = 0;
            lockTick = 0;
            LogMessage(DEBUG_ALL, "outputNode==>Unlock");
        }
        return ;
    }
    if (headNode == NULL)
    {
        if (sleepTick > 0)
        {
            sleepTick--;
        }
        else
        {
            SLEEPMODULE;
        }
        return ;
    }
    sleepTick = 2;
    currentnode = headNode;
    if (currentnode != NULL)
    {
        nextnode = currentnode->nextnode;
        moduleState.cmd = currentnode->currentcmd;

        //���ݷ���
        portUartSend(&usart3_ctl, (uint8_t *)currentnode->data, currentnode->datalen);
        if (currentnode->data[0] != 0X78 && currentnode->data[0] != 0x79 && currentnode->data[0] != 0x7E)
        {
            LogMessageWL(DEBUG_ALL, currentnode->data, currentnode->datalen);
        }
        free(currentnode->data);
        free(currentnode);
        if (currentnode->currentcmd == QICLOSE_CMD || currentnode->currentcmd == CMGS_CMD ||
                currentnode->currentcmd == WIFISCAN_CMD)
        {
            lockFlag = 1;
            if (currentnode->currentcmd == QICLOSE_CMD)
            {
                tickRange = 20;
            }
            else
            {
                tickRange = 25;
            }
            LogMessage(DEBUG_ALL, "outputNode==>Lock");
        }
    }
    headNode = nextnode;
}

/**************************************************
@bref		ģ��ָ���
@param
@return
@note
**************************************************/

uint8_t  sendModuleCmd(uint8_t cmd, char *param)
{
    uint8_t i;
    int16_t cmdtype = -1;
    char sendData[256];

    if (moduleState.powerState == 0)
    {
        //LogMessage(DEBUG_ALL,"sendModuleCmd==>module was off");
        return 0;
    }

    for (i = 0; i < sizeof(cmdtable) / sizeof(cmdtable[0]); i++)
    {
        if (cmd == cmdtable[i].cmd_type)
        {
            cmdtype = i;
            break;
        }
    }
    if (cmdtype < 0)
    {
        snprintf(sendData, 255, "sendModuleCmd==>No cmd");
        LogMessage(DEBUG_ALL, sendData);
        return 0;
    }
    if (param != NULL && strlen(param) <= 240)
    {
        if (param[0] == '?')
        {
            snprintf(sendData, 255, "%s?\r\n", cmdtable[cmdtype].cmd);

        }
        else
        {
            snprintf(sendData, 255, "%s=%s\r\n", cmdtable[cmdtype].cmd, param);
        }
    }
    else if (param == NULL)
    {
        snprintf(sendData, 255, "%s\r\n", cmdtable[cmdtype].cmd);
    }
    else
    {
        return 0;
    }
    createNode(sendData, strlen(sendData), cmd);
    return 1;
}

void sendModuleDirect(char *param)
{
    createNode(param, strlen(param), 0);
}

/**************************************************
@bref		��ʼ��ģ�����ʹ�ýṹ��
@param
@return
@note
**************************************************/

static void moduleInit(void)
{
    memset(&moduleState, 0, sizeof(moduleState_s));
}

/**************************************************
@bref		��ѯģ�鿪�ػ�
@param
@return
@note
**************************************************/
uint8_t isModulePowerOn(void)
{
	if (moduleState.powerState)
		return 1;
	return 0;
}

/**************************************************
@bref		�Ƿ񿪻�����
@param
@return
@note
**************************************************/
static void modulePressReleaseKey(void)
{
    PWRKEY_HIGH;
    moduleState.powerState = 1;
    LogPrintf(DEBUG_ALL, "PowerOn Done");
}
/**************************************************
@bref		���¿�������
@param
@return
@note
**************************************************/

static void modulePressPowerKey(void)
{
    PWRKEY_LOW;
    startTimer(10, modulePressReleaseKey, 0);
}
/**************************************************
@bref		ģ�鿪��
@param
@return
@note
**************************************************/

void modulePowerOn(void)
{
    LogMessage(DEBUG_ALL, "modulePowerOn");
    portModuleGpioCfg(1);
    moduleInit();
    portUartCfg(APPUSART3, 1, 115200, moduleRecvParser);
    POWER_ON;
    PWRKEY_HIGH;
    RSTKEY_HIGH;
    startTimer(6, modulePressPowerKey, 0);
    moduleState.gpsFileHandle = 1;
    moduleCtrl.scanMode = 0;
    socketDelAll();
}

/**************************************************
@bref		�ػ�,�ɿ���������
@param
@return
@note
**************************************************/

static void modulePressReleaseOffKey(void)
{
    PWRKEY_HIGH;
    moduleInit();
    socketDelAll();
    moduleState.powerState = 0;
    LogPrintf(DEBUG_ALL, "PowerOFF Done");
    portModuleGpioCfg(0);
}

/**************************************************
@bref		�ػ�,���¿�������
@param
@return
@note       �������ٴΰ�������ť����1.5s�ػ�
**************************************************/

static void modulePressPowerOffKey(void)
{
    PWRKEY_LOW;
    startTimer(20, modulePressReleaseOffKey, 0);
}


/**************************************************
@bref		ģ��ػ�
@param
@return
@note
**************************************************/

void modulePowerOff(void)
{
    LogMessage(DEBUG_ALL, "modulePowerOff");
    
    portModuleGpioCfg(1);
    portUartCfg(APPUSART3, 0, 115200, NULL);
    POWER_OFF;
    //RSTKEY_HIGH;
    PWRKEY_HIGH;
    startTimer(6, modulePressPowerOffKey, 0);

}

/**************************************************
@bref		�ͷŸ�λ����
@param
@return
@note
**************************************************/

static void moduleReleaseRstkey(void)
{
    moduleState.powerState = 1;
    RSTKEY_HIGH;
}
/**************************************************
@bref		ģ�鸴λ
@param
@return
@note
**************************************************/

void moduleReset(void)
{
    LogMessage(DEBUG_ALL, "moduleReset");
    moduleInit();
    POWER_OFF;
    RSTKEY_LOW;
    startTimer(15, modulePowerOn, 0);
    socketDelAll();
}

/**************************************************
@bref		�л�����״̬��
@param
@return
@note
**************************************************/
static void changeProcess(uint8_t fsm)
{
    moduleState.fsmState = fsm;
    moduleState.fsmtick = 0;
    if (moduleState.fsmState != NORMAL_STATUS)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
    }
}

/**************************************************
@bref		����socket
@param
@return
@note
**************************************************/

void openSocket(uint8_t link, char *server, uint16_t port)
{
    char param[100];
    sprintf(param, "%d,\"TCP\",\"%s\",%d", link, server, port);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPSTART_CMD, param);
}

/**************************************************
@bref		�ر�socket
@param
@return
@note
**************************************************/

void closeSocket(uint8_t link)
{
    char param[10];
    sprintf(param, "%d", link);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPCLOSE_CMD, param);
}

/**************************************************
@bref		apn����
@param
@return
@note
**************************************************/

static void netSetCgdcong(char *apn)
{

    char param[100];
    sprintf(param, "1,\"IP\",\"%s\"", apn);
    sendModuleCmd(CGDCONT_CMD, param);
}

/**************************************************
@bref		apn����
@param
@return
@note
**************************************************/

static void netSetApn(char *apn, char *apnname, char *apnpassword)
{
    char param[100];
    sprintf(param, "1,1,\"%s\",\"%s\",\"%s\"", apn, apnname, apnpassword);
    sendModuleCmd(QICSGP_CMD, param);
}


/**************************************************
@bref		ģ��������ģʽ
@param
@return
@note
**************************************************/

static void moduleEnterFly(void)
{
    sendModuleCmd(CFUN_CMD, "0");
}

/**************************************************
@bref		ģ���������ģʽ
@param
@return
@note
**************************************************/

static void moduleExitFly(void)
{
    sendModuleCmd(CFUN_CMD, "1");
}

/**************************************************
@bref		����socket��ȡ����ָ��
@param
@return
@note
**************************************************/

static void qirdCmdSend(uint8_t link)
{
    char param[10];
    sprintf(param, "2,%d,1200", link);
    moduleState.curQirdId = link;
    sendModuleCmd(CIPRXGET_CMD, param);
}

/**************************************************
@bref		��ȡ����
@param
@return
@note
**************************************************/

static void queryRecvBuffer(void)
{
    char param[10];
    if (moduleState.normalLinkQird)
    {
        qirdCmdSend(NORMAL_LINK);
    }
    else if (moduleState.agpsLinkQird)
    {
        qirdCmdSend(AGPS_LINK);
    }
    else if (moduleState.bleLinkQird)
    {
        qirdCmdSend(BLE_LINK);
    }
    else if (moduleState.jt808LinkQird)
    {
        qirdCmdSend(JT808_LINK);
    }
    else if (moduleState.hideLinkQird)
    {
        qirdCmdSend(HIDDEN_LINK);
    }
}

/**************************************************
@bref		����׼������
@param
@return
@note
**************************************************/

void netConnectTask(void)
{
    if (moduleState.powerState == 0)
    {
        return;
    }

    moduleState.powerOnTick++;
    switch (moduleState.fsmState)
    {
        case AT_STATUS:
            if (moduleState.atResponOK)
            {
                moduleCtrl.atCount = 0;
                moduleState.atResponOK = 0;
                moduleState.cpinResponOk = 0;
                changeProcess(CPIN_STATUS);
            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    moduleState.powerOnTick = 0;
                    sendModuleCmd(AT_CMD, NULL);
                }
                if (moduleState.fsmtick >= 30)
                {
                    moduleCtrl.atCount++;
                    if (moduleCtrl.atCount >= 2)
                    {
                        moduleCtrl.atCount = 0;
                        modulePowerOn();
                    }
                    else
                    {
                        moduleReset();
                    }
                }
                break;
            }
        case CPIN_STATUS:
            if (moduleState.cpinResponOk)
            {
                moduleState.cpinResponOk = 0;
                moduleState.csqOk = 0;
                sendModuleCmd(AT_CMD, NULL);
                netSetCgdcong((char *)sysparam.apn);
                netSetApn((char *)sysparam.apn, (char *)sysparam.apnuser, (char *)sysparam.apnpassword);
                changeProcess(CSQ_STATUS);

            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    sendModuleCmd(CPIN_CMD, "?");
                }
                if (moduleState.fsmtick >= 30)
                {
                    moduleReset();
                }
                break;
            }
        case CSQ_STATUS:
            if (moduleState.csqOk)
            {
                moduleState.csqOk = 0;
                moduleState.cgregOK = 0;
                moduleCtrl.csqCount = 0;
                sendModuleCmd(CGREG_CMD, "2");
                sendModuleCmd(CEREG_CMD, "2");
                sendModuleCmd(CIPSHUT_CMD, NULL);
                changeProcess(CGREG_STATUS);
                netResetCsqSearch();
            }
            else
            {
                sendModuleCmd(CSQ_CMD, NULL);
                if (moduleCtrl.csqTime == 0)
                {
                    moduleCtrl.csqTime = 90;
                }
                if (moduleState.fsmtick >= moduleCtrl.csqTime)
                {
                    moduleCtrl.csqCount++;
                    if (moduleCtrl.csqCount >= 3)
                    {
                        moduleCtrl.csqCount = 0;
                        //3��������������ʱ�����û��gps������ػ�
                        if (sysinfo.gpsRequest != 0)
                        {
                            moduleReset();
                        }
                        else
                        {
                            modeTryToStop();
                        }
                    }
                    else
                    {
                        moduleEnterFly();
                        startTimer(80, moduleExitFly, 0);
                    }
                    changeProcess(AT_STATUS);
                }
                break;
            }
        case CGREG_STATUS:
            if (moduleState.cgregOK)
            {
                moduleCtrl.cgregCount = 0;
                moduleState.cgregOK = 0;
                changeProcess(CONFIG_STATUS);
            }
            else
            {
                sendModuleCmd(CGREG_CMD, "?");
                sendModuleCmd(CEREG_CMD, "?");
                if (moduleState.fsmtick >= 90)
                {
                    moduleCtrl.cgregCount++;
                    if (moduleCtrl.cgregCount >= 2)
                    {
                        moduleCtrl.cgregCount = 0;
                        //2��ע�᲻�ϻ�վʱ�����û��gps������ػ�
                        if (sysinfo.gpsRequest == 0)
                        {
                            modeTryToStop();
                        }
                        else
                        {
                            moduleReset();
                        }

                        LogMessage(DEBUG_ALL, "Register timeout,try to skip");
                    }
                    else
                    {
                        sendModuleCmd(CGATT_CMD, "1");
                        changeProcess(AT_STATUS);
                    }
                }
                break;
            }
        case CONFIG_STATUS:
            sendModuleCmd(CFGRI_CMD, "1,50,50,3");
            sendModuleCmd(CIPMUX_CMD, "1");
            sendModuleCmd(CIPQSEND_CMD, "1");
            sendModuleCmd(CIPRXGET_CMD, "5");
            sendModuleCmd(CFG_CMD, "\"urcdelay\",100");
            changeProcess(QIACT_STATUS);
            break;
        case QIACT_STATUS:
            if (moduleState.qipactOk)
            {
                moduleState.qipactOk = 0;
                moduleCtrl.qipactCount = 0;
                changeProcess(NORMAL_STATUS);
                sendModuleCmd(AT_CMD, NULL);
                sendModuleCmd(CGSN_CMD, NULL);
            }
            else
            {
                if (moduleState.qipactSet == 0)
                {
                    moduleState.qipactSet = 1;
                    sendModuleCmd(CGATT_CMD, "1");
                }
                else
                {
                    sendModuleCmd(CGATT_CMD, "?");
                }
                if (moduleState.fsmtick >= 45)
                {
                    LogMessage(DEBUG_ALL, "try QIPACT again");
                    moduleState.qipactSet = 0;
                    moduleState.fsmtick = 0;
                    moduleCtrl.qipactCount++;
                    if (moduleCtrl.qipactCount >= 3)
                    {
                        moduleCtrl.qipactCount = 0;
                        moduleReset();
                    }
                    else
                    {
                        changeProcess(CPIN_STATUS);
                    }
                }
                break;
            }
        case NORMAL_STATUS:
            socketSchedule();
            queryRecvBuffer();
            break;
        default:
            changeProcess(AT_STATUS);
            break;
    }
    moduleState.fsmtick++;
}


/**************************************************
@bref		AT+CSQ	ָ�����
@param
@return
@note
**************************************************/

static void csqParser(uint8_t *buf, uint16_t len)
{
    int index, indexa, datalen;
    uint8_t *rebuf;
    uint16_t  relen;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CSQ:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, ',');
        if (indexa > 6)
        {
            datalen = indexa - 6;
            if (datalen > 5)
                return;
            memset(restore, 0, 5);
            strncpy(restore, (char *)rebuf + 6, datalen);
            moduleState.rssi = atoi(restore);
            if (moduleState.rssi >= 6 && moduleState.rssi <= 31)
                moduleState.csqOk = 1;
        }
    }
}

/**************************************************
@bref		AT+CREG	ָ�����
@param
@return
@note
**************************************************/

static void cgregParser(uint8_t *buf, uint16_t len)
{
    int index, datalen;
    uint8_t *rebuf;
    uint16_t  relen, i;
    char restore[50];
    uint8_t cnt;
    uint8_t type = 0;
    index = my_getstrindex((char *)buf, "+CGREG:", len);
    if (index < 0)
    {
        type = 1;
        index = my_getstrindex((char *)buf, "+CEREG:", len);
    }
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        datalen = 0;
        cnt = 0;
        restore[0] = 0;
        for (i = 0; i < relen; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 2:
                            if (restore[0] == '1' || restore[0] == '5')
                            {
                                if (type)
                                {
                                    moduleState.ceregOK = 1;
                                }
                                else
                                {
                                    moduleState.cgregOK = 1;
                                }
                            }
                            else
                            {
                                return ;
                            }
                            break;
                        case 3:

                            moduleState.lac = strtoul(restore + 1, NULL, 16);
                            LogPrintf(DEBUG_ALL, "LAC=%s,0x%X", restore, moduleState.lac);
                            break;
                        case 4:
                            moduleState.cid = strtoul(restore + 1, NULL, 16);
                            LogPrintf(DEBUG_ALL, "CID=%s,0x%X", restore, moduleState.cid);
                            break;
                    }
                    restore[0] = 0;
                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }

            }
        }
    }
}

/**************************************************
@bref		AT+CIMI	ָ�����
@param
@return
@note
**************************************************/

static void cimiParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
    index = getCharIndex(rebuf, relen, '\n');
    if (index < 0)
    {
        return;
    }
    rebuf = rebuf + index + 1;
    relen = relen - index - 1;
    index = getCharIndex(rebuf, relen, '\r');
    if (index == 15)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMSI[i] = rebuf[i];
        }
        moduleState.IMSI[index] = 0;
        moduleState.mcc = (moduleState.IMSI[0] - '0') * 100 + (moduleState.IMSI[1] - '0') * 10 + moduleState.IMSI[2] - '0';
        moduleState.mnc = (moduleState.IMSI[3] - '0') * 10 + moduleState.IMSI[4] - '0';
        LogPrintf(DEBUG_ALL, "IMSI:%s,MCC=%d,MNC=%d", moduleState.IMSI, moduleState.mcc, moduleState.mnc);
    }
}


/**************************************************
@bref		AT+ICCID	ָ�����
@param
@return
@note
**************************************************/

static void iccidParser(uint8_t *buf, uint16_t len)
{
    int16_t index, indexa;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t snlen, i;
    char debug[70];
    index = my_getstrindex((char *)buf, "+ICCID:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, '\r');
        if (indexa > 8)
        {
            snlen = indexa - 8;
            if (snlen == 20)
            {
                for (i = 0; i < snlen; i++)
                {
                    moduleState.ICCID[i] = rebuf[i + 8];
                }
                moduleState.ICCID[snlen] = 0;
                sprintf(debug, "ICCID:%s", moduleState.ICCID);
                LogMessage(DEBUG_ALL, debug);
            }
        }
    }

}

/**************************************************
@bref		AT+QIACT	ָ�����
@param
@return
@note
**************************************************/

static void qipactParser(uint8_t *buf, uint16_t len)
{
    int index, datalen;
    uint8_t *rebuf;
    uint16_t  relen, i;
    char restore[50];
    uint8_t   cnt;

    index = my_getstrindex((char *)buf, "+QIACT:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        datalen = 0;
        cnt = 0;
        restore[0] = 0;
        for (i = 0; i < relen; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 4:
                            if (my_strpach(restore, "\"0.0.0.0\"") == 1)
                            {
                                moduleState.qipactOk = 0;

                            }
                            else if (restore[0] == '"')
                            {
                                moduleState.qipactOk = 1;
                            }
                            else
                            {
                                moduleState.qipactOk = 0;

                            }
                            break;
                    }
                    restore[0] = 0;

                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }

            }
        }
    }
}

/**************************************************
@bref		socket���ݽ��ս���
@param
@return
@note
+QIURC: "closed",0
+QIURC: "pdpdeact",1
+QIURC: "recv",1,10
xx\0\0�U

**************************************************/

static void qiurcParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    char resbuf[513];
    int index, relen, recvLen;
    int sockId, debugLen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+QIURC:", relen);
    if (index < 0)
        return;
    while (index >= 0)
    {
        index += 9;
        rebuf += index;
        relen -= index;
        if (my_strpach(rebuf, "closed"))
        {
            rebuf += 8;
            relen -= 8;
            index = getCharIndex(rebuf, relen, '\r');
            if (index >= 1 && index <= 2)
            {
                memcpy(resbuf, rebuf, index);
                resbuf[index] = 0;
                sockId = atoi(resbuf);
                LogPrintf(DEBUG_ALL, "Socket[%d] Close", sockId);
                socketSetConnState(sockId, SOCKET_CONN_IDLE);
            }
            rebuf += index;
            relen -= index;
        }
        else if (my_strpach(rebuf, "pdpdeact"))
        {
            rebuf += 8;
            relen -= 8;
            LogMessage(DEBUG_ALL, "Socket all closed");
            moduleSleepCtl(0);
            socketResetConnState();
            changeProcess(AT_STATUS);
        }
        else if (my_strpach(rebuf, "recv"))
        {
            rebuf += 6;
            relen -= 6;
            index = getCharIndex(rebuf, relen, ',');
            if (index >= 1 && index <= 2)
            {
                memcpy(resbuf, rebuf, index);
                resbuf[index] = 0;
                sockId = atoi(resbuf);
                LogPrintf(DEBUG_ALL, "Socket[%d] recv data", sockId);
                switch (sockId)
                {
                    case NORMAL_LINK:
                        moduleState.normalLinkQird = 1;
                        break;
                    case BLE_LINK:
                        moduleState.bleLinkQird = 1;
                        break;
                    case JT808_LINK:
                        moduleState.jt808LinkQird = 1;
                        break;
                    case HIDDEN_LINK:
                        moduleState.hideLinkQird = 1;
                        break;
                    case AGPS_LINK:
                        moduleState.agpsLinkQird = 1;
                        break;
                }
            }
        }

        index = my_getstrindex((char *)rebuf, "+QIURC:", relen);
    }
}




/**************************************************
@bref		���Ž���
@param
@return
@note
**************************************************/


static void cmtiParser(uint8_t *buf, uint16_t len)
{
    uint8_t i;
    int16_t index;
    uint8_t *rebuf;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CMTI:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        index = getCharIndex(rebuf, len, ',');
        if (index < 0)
            return;
        rebuf = rebuf + index + 1;
        index = getCharIndex(rebuf, len, '\r');
        if (index > 5 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message index=%d", atoi(restore));
        sendModuleCmd(CMGR_CMD, restore);
    }
}

/**************************************************
@bref		CMGR	ָ�����
@param
@return
@note
**************************************************/

static void cmgrParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t *numbuf;
    uint16_t  relen, i, renumlen;
    char restore[100];
    insParam_s insparam;
    //�ҵ��ض��ַ�����buf��λ��
    index = my_getstrindex((char *)buf, "+CMGR:", len);
    if (index >= 0)
    {
        //�õ��ض��ַ����Ŀ�ʼλ�ú�ʣ�೤��
        rebuf = buf + index;
        relen = len - index;
        //ʶ���ֻ�����
        index = getCharIndexWithNum(rebuf, relen, '"', 3);
        if (index < 0)
            return;
        numbuf = rebuf + index + 1;
        renumlen = relen - index - 1;
        index = getCharIndex(numbuf, renumlen, '"');
        if (index > 100 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = numbuf[i];
        }
        restore[index] = 0;

        if (index > sizeof(moduleState.messagePhone))
            return ;
        strcpy((char *)moduleState.messagePhone, restore);
        LogPrintf(DEBUG_ALL, "Tel:%s", moduleState.messagePhone);
        //�õ���һ��\n��λ��
        index = getCharIndex(rebuf, len, '\n');
        if (index < 0)
            return;
        //ƫ�Ƶ����ݴ�
        rebuf = rebuf + index + 1;
        //�õ������ݴ���ʼ�ĵ�һ��\n������index�������ݳ���
        index = getCharIndex(rebuf, len, '\n');
        if (index > 100 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message:%s", restore);
        insparam.telNum = moduleState.messagePhone;
        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
    }
}

/**************************************************
@bref		AT+QISEND	ָ�����
@param
@return
@note
+QISEND: 212,212,0

**************************************************/

static void qisendParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t i, datalen, cnt, sockId;
    uint16_t relen;
    char restore[51];

    index = my_getstrindex((char *)buf, "ERROR", len);
    if (index >= 0)
    {
        //���ܺܺ����ֵ�����������·���ִ���
        socketResetConnState();
        moduleSleepCtl(0);
        changeProcess(CGREG_STATUS);
        return ;
    }
    index = my_getstrindex((char *)buf, "+QISEND:", len);
    if (index < 0)
    {
        return ;
    }

    rebuf = buf + index + 9;
    relen = len - index - 9;
    index = getCharIndex(rebuf, relen, '\n');
    datalen = 0;
    cnt = 0;
    if (index > 0 && index < 50)
    {
        restore[0] = 0;
        for (i = 0; i < index; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 1:
                            moduleState.tcpTotal = atoi(restore);
                            break;
                        case 2:
                            moduleState.tcpAck = atoi(restore);
                            break;
                        case 3:
                            moduleState.tcpNack = atoi(restore);
                            break;
                    }
                    restore[0] = 0;
                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }

            }
        }
    }
}


/**************************************************
@bref		QWIFISCAN	ָ�����
@param
@return
@note
+QWIFISCAN: (-,-,-74,"F4:6D:2F:7F:0E:A8",6)
+QWIFISCAN: (-,-,-74,"08:6B:D1:0B:50:60",11)
+QWIFISCAN: (-,-,-79,"4A:98:CA:B9:71:9C",1)
+QWIFISCAN: (-,-,-79,"C4:AD:34:C7:0D:01",7)
+QWIFISCAN: (-,-,-83,"70:3A:73:05:79:1C",13)

+WIFISCAN: "50:0f:f5:51:1a:eb",-31,3
+WIFISCAN: "dc:9f:db:1c:1d:76",-50,11
+WIFISCAN: "ec:41:18:0c:82:09",-63,4
+WIFISCAN: "40:22:30:1a:f8:01",-67,1
+WIFISCAN: "08:6b:d1:0b:50:60",-68,11
+WIFISCAN: "14:09:b4:95:94:6d",-72,6
+WIFISCAN: "f8:8c:21:a2:c6:e9",-74,11
+WIFISCAN: "70:3a:73:05:79:1c",-90,13
+WIFISCAN: "2c:9c:6e:28:f5:0e",-91,1
+WIFISCAN: "7c:a7:b0:61:db:14",-92,11

OK

**************************************************/

static void wifiscanParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf, i;
    int16_t relen;
    char restore[20];
    WIFIINFO wifiList;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+WIFISCAN:", relen);
    wifiList.apcount = 0;
    while (index >= 0)
    {
        rebuf += index + 12;
        relen -= index + 12;

        index = getCharIndex(rebuf, relen, '"');
        if (index == 17 && wifiList.apcount < WIFIINFOMAX)
        {
            memcpy(restore, rebuf, index);
            restore[17] = 0;
            LogPrintf(DEBUG_ALL, "WIFI:[%s]", restore);

            wifiList.ap[wifiList.apcount].signal = 0;
            for (i = 0; i < 6; i++)
            {
                changeHexStringToByteArray(wifiList.ap[wifiList.apcount].ssid + i, restore + (3 * i), 1);
            }
            wifiList.apcount++;
        }
        index = getCharIndex(rebuf, relen, '\n');
        rebuf += index;
        relen -= index;

        index = my_getstrindex((char *)rebuf, "+WIFISCAN:", relen);
    }
    if (wifiList.apcount != 0)
    {
        if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_MY)
        {
            protocolSend(NORMAL_LINK, PROTOCOL_F3, &wifiList);
        }
        if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_BLE)
        {
            protocolSend(BLE_LINK, PROTOCOL_F3, &wifiList);
        }
        sysinfo.wifiExtendEvt = 0;
    }
}
static void cgsnParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
    index = getCharIndex(rebuf, relen, '\n');
    rebuf = rebuf + index + 1;
    relen = relen - index - 1;
    index = getCharIndex(rebuf, relen, '\r');
    if (index >= 0 && index < 20)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMEI[i] = rebuf[i];
        }
        moduleState.IMEI[index] = 0;
        LogPrintf(DEBUG_ALL, "module IMEI [%s]", moduleState.IMEI);
        if (tmos_memcmp(moduleState.IMEI, sysparam.SN, 15) == FALSE)
        {
            tmos_memset(sysparam.SN, 0, sizeof(sysparam.SN));
            strncpy(sysparam.SN, moduleState.IMEI, 15);
            jt808CreateSn(sysparam.jt808sn, sysparam.SN + 3, 12);
            sysparam.jt808isRegister = 0;
            sysparam.jt808AuthLen = 0;
            paramSaveAll();
        }
    }

}


/**************************************************
@bref		QIOPEN	ָ�����
@param
@return
@note
	0, CONNECT OK
**************************************************/

static void qiopenParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t sockId;
    int16_t relen;
    uint16_t result;
    char restore[10];
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+QIOPEN:", relen);
    while (index >= 0)
    {
        rebuf += index + 9;
        relen -= index + 9;

        index = getCharIndex(rebuf, relen, ',');
        if (index >= 1 && index <= 2)
        {
            memcpy(restore, rebuf, index);
            restore[index] = 0;
            sockId = atoi(restore);

            rebuf += index + 1;
            relen -= index + 1;

            index = getCharIndex(rebuf, relen, '\r');
            if (index > 0 && index < 9)
            {
                memcpy(restore, rebuf, index);
                restore[index] = 0;
                result = atoi(restore);
                if (result == 0)
                {
                    socketSetConnState(sockId, SOCKET_CONN_SUCCESS);
                }
                else
                {
                    socketSetConnState(sockId, SOCKET_CONN_ERR);
                }
            }
        }
        index = my_getstrindex((char *)rebuf, "+QIOPEN:", relen);
    }
}

/*
+CGATT: 1

OK

*/
void cgattParser(uint8_t *buf, uint16_t len)
{
    uint8_t ret;
    int index;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+CGATT:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    ret = rebuf[8] - '0';
    if (ret == 1)
    {
        moduleState.qipactOk = 1;
    }
    else
    {
        moduleState.qipactOk = 0;
    }
}


/**************************************************
@bref		+RECEIVE	ָ�����
@param
@return
@note
+RECEIVE: 0, 16
xx?		\0U?

**************************************************/

uint8_t receiveParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    char resbuf[513];
    int index, relen, recvLen;
    int sockId, debugLen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+RECEIVE:", relen);
    if (index < 0)
        return 0;
    while (index >= 0)
    {
        rebuf += index + 10;
        relen -= index + 10;
        index = getCharIndex(rebuf, relen, ',');
        if (index >= 1 && index <= 2)
        {
            memcpy(resbuf, rebuf, index);
            resbuf[index] = 0;
            sockId = atoi(resbuf);
            rebuf += index + 2;
            relen -= index + 2;
            index = getCharIndex(rebuf, relen, '\r');
            if (index >= 0 && index <= 5)
            {
                memcpy(resbuf, rebuf, index);
                resbuf[index] = 0;
                recvLen = atoi(resbuf);
                rebuf += index + 2;
                relen -= index + 2;
                if (relen >= recvLen)
                {
                    debugLen = recvLen > 256 ? 256 : recvLen;
                    byteToHexString(rebuf, resbuf, debugLen);
                    resbuf[debugLen * 2] = 0;
                    LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", sockId, recvLen, resbuf);
                    socketRecv(sockId, rebuf, recvLen);
                    rebuf += recvLen;
                    relen -= recvLen;
                }
                else
                {
                    LogMessage(DEBUG_ALL, "TCP data lost");
                    return 1;
                }

            }
        }
        index = my_getstrindex((char *)rebuf, "+RECEIVE:", relen);
    }
    return 0;
}


/**************************************************
@bref		QISACK	ָ�����
@param
@return
@note
	+QISACK: 0, 0, 0
**************************************************/

void qisackParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    int index, relen;
    ITEM item;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+QISACK:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index + 9;
    relen -= index + 9;
    stringToItem(&item, rebuf, relen);
    if (item.item_cnt >= 3)
    {
        moduleState.tcpTotal = atoi(item.item_data[0]);
        moduleState.tcpAck = atoi(item.item_data[1]);
        moduleState.tcpNack = atoi(item.item_data[2]);
        LogPrintf(DEBUG_ALL, "Total:%d,Ack:%d,NAck:%d", moduleState.tcpTotal, moduleState.tcpAck, moduleState.tcpNack);
    }
}

/**************************************************
@bref		CBC	ָ�����
@param
@return
@note
	+CBC: 0,80,3909

	OK
**************************************************/

void cbcParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    int index, relen;
    ITEM item;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+CBC:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index + 6;
    relen -= index + 6;
    stringToItem(&item, rebuf, relen);
    if (item.item_cnt >= 3)
    {
        sysinfo.insidevoltage = atoi(item.item_data[2]) / 1000.0;
        LogPrintf(DEBUG_ALL, "batttery voltage %.2f", sysinfo.insidevoltage);
    }
}


/**************************************************
@bref		QIRD	ָ�����
@param
@return
@note
+QIRD: 47.107.25.39:9998,TCP,10
xx�\0
?

OK

**************************************************/

static uint8_t qirdParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    char resbuf[513];
    int index, relen, recvLen;
    int debugLen;
    uint8_t ret = 0;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+QIRD:", relen);
    if (index < 0)
    {
        return ret;
    }
    while (index >= 0)
    {
        rebuf += index + 7;
        relen -= index + 7;
        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0 && index <= 5)
        {
            memcpy(resbuf, rebuf, index);
            resbuf[index] = 0;
            recvLen = atoi(resbuf);
            rebuf += index + 2;
            relen -= index + 2;
            if (relen >= recvLen)
            {
                if (recvLen == 0)
                {
                    switch (moduleState.curQirdId)
                    {
                        case NORMAL_LINK:
                            moduleState.normalLinkQird = 0;
                            break;
                        case BLE_LINK:
                            moduleState.bleLinkQird = 0;
                            break;
                        case JT808_LINK:
                            moduleState.jt808LinkQird = 0;
                            break;
                        case HIDDEN_LINK:
                            moduleState.hideLinkQird = 0;
                            break;
                        case AGPS_LINK:
                            moduleState.agpsLinkQird = 0;
                            break;
                    }
                    LogPrintf(DEBUG_ALL, "Link[%d] recv Done", moduleState.curQirdId);
                }
                else
                {
                    debugLen = recvLen > 256 ? 256 : recvLen;
                    byteToHexString(rebuf, resbuf, debugLen);
                    resbuf[debugLen * 2] = 0;
                    LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", moduleState.curQirdId, recvLen, resbuf);
                    ret = 0;
                    socketRecv(moduleState.curQirdId, rebuf, recvLen);
                }

                rebuf += recvLen;
                relen -= recvLen;
            }
            else
            {
                ret = 1;
                LogMessage(DEBUG_ALL, "TCP data lost");
            }

        }

        index = my_getstrindex((char *)rebuf, "+QIRD:", relen);
    }
    return ret;
}


/**************************************************
@bref		PDP	ָ�����
@param
@return
@note
+PDP DEACT


**************************************************/
void deactParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+PDP DEACT", len);
    if (index < 0)
    {
        return;
    }
    socketDelAll();
    changeProcess(CPIN_STATUS);
}


/**************************************************
@bref		CMT	ָ�����
@param
@return
@note
+CMT: "1064899195049",,"2022/09/05 17:46:53+32"
PARAM

**************************************************/

void cmtParser(uint8_t *buf, uint16_t len)
{
    uint8_t *rebuf;
    char restore[130];
    int relen;
    int index;
    insParam_s insparam;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+CMT:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    while (index >= 0)
    {
        rebuf += 7;
        relen -= 7;
        index = getCharIndex(rebuf, relen, '\"');
        if (index < 0 || index >= 20)
        {
            return;
        }
        memcpy(moduleState.messagePhone, rebuf, index);
        moduleState.messagePhone[index] = 0;
        LogPrintf(DEBUG_ALL, "TEL:%s", moduleState.messagePhone);
        index = getCharIndex(rebuf, relen, '\n');
        if (index < 0)
        {
            return;
        }
        rebuf += index + 1;
        relen -= index - 1;
        index = getCharIndex(rebuf, relen, '\r');
        if (index < 0 || index >= 128)
        {
            return ;
        }
        memcpy(restore, rebuf, index);
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Content:[%s]", restore);
        insparam.telNum = moduleState.messagePhone;
        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
    }
}

void ringParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "RING", len);
    if (index < 0)
    {
        return;
    }
    sendModuleCmd(ATA_CMD, NULL);
}

void cipstartRspParser(uint8_t *buf, uint16_t len)
{
    int index;
    int relen;
    uint8_t *rebuf;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "CONNECT", relen);
    if (index < 0)
    {
        return;
    }

    index = my_getstrindex(rebuf, "0, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(0, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "1, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(1, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "2, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(2, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "3, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(3, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "4, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(4, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "5, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(5, SOCKET_CONN_SUCCESS);
    }

    index = my_getstrindex(rebuf, "0, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(0, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "1, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(1, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "2, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(2, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "3, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(3, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "4, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(4, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "5, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(5, SOCKET_CONN_ERR);
    }
}

void cipstartParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+CME ERROR:", len);
    if (index < 0)
    {
        return;
    }
    LogMessage(DEBUG_ALL, "socket open err");
    socketDelAll();
    changeProcess(CPIN_STATUS);
    sendModuleCmd(CIPSHUT_CMD, NULL);
}
/*
+CIPRXGET: 1,0
+CIPRXGET: 2,0,20,0
xx\02
xx\02


OK

*/
uint8_t ciprxgetParser(uint8_t *buf, uint16_t len)
{
    int index;
    char restore[513];
    uint8_t *rebuf, type, link, ret = 0;
    int16_t relen;
    uint16_t readLen, unreadLen, debugLen;
    ITEM item;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "+CIPRXGET:", relen);
    while (index >= 0)
    {
        rebuf += index + 11;
        relen -= index + 11;

        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0 && index < 20)
        {
            tmos_memcpy(restore, rebuf, index);
            restore[index] = 0;
            stringToItem(&item, restore, index);
            type = atoi(item.item_data[0]);
            link = atoi(item.item_data[1]);
            if (type == 1)
            {
                LogPrintf(DEBUG_ALL, "Socket[%d] recv data", link);
                switch (link)
                {
                    case NORMAL_LINK:
                        moduleState.normalLinkQird = 1;
                        break;
                    case BLE_LINK:
                        moduleState.bleLinkQird = 1;
                        break;
                    case JT808_LINK:
                        moduleState.jt808LinkQird = 1;
                        break;
                    case HIDDEN_LINK:
                        moduleState.hideLinkQird = 1;
                        break;
                    case AGPS_LINK:
                        moduleState.agpsLinkQird = 1;
                        break;
                }
            }
            else if (type == 2)
            {
                readLen = atoi(item.item_data[2]);
                unreadLen = atoi(item.item_data[3]);
                rebuf += index + 2;
                relen -= index + 2;
                if (relen >= readLen)
                {
                    if (readLen != 0)
                    {
                        debugLen = readLen > 256 ? 256 : readLen;
                        byteToHexString(rebuf, restore, debugLen);
                        restore[debugLen * 2] = 0;
                        LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", link, readLen, restore);
                        socketRecv(link, rebuf, readLen);
                    }
                }
                else
                {
                    ret = 1;
                    break;
                }

                if (unreadLen == 0)
                {
                    switch (link)
                    {
                        case NORMAL_LINK:
                            moduleState.normalLinkQird = 0;
                            break;
                        case BLE_LINK:
                            moduleState.bleLinkQird = 0;
                            break;
                        case JT808_LINK:
                            moduleState.jt808LinkQird = 0;
                            break;
                        case HIDDEN_LINK:
                            moduleState.hideLinkQird = 0;
                            break;
                        case AGPS_LINK:
                            moduleState.agpsLinkQird = 0;
                            break;
                    }
                }
            }
        }

        index = my_getstrindex(rebuf, "+CIPRXGET:", relen);
    }

    return ret;
}

/*
+CIPACK: 42,42,0
*/
void cipackParser(uint8_t *buf, uint16_t len)
{
    int index;
    ITEM item;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+CIPACK:", relen);
    if (index < 0)
    {
        return;
    }

    rebuf += 8;
    relen -= 8;

    if (relen < 0)
    {
        return;
    }
    stringToItem(&item, rebuf, relen);
    moduleState.tcpTotal = atoi(item.item_data[0]);
    moduleState.tcpAck = atoi(item.item_data[1]);
    moduleState.tcpNack = atoi(item.item_data[2]);
    LogPrintf(DEBUG_ALL, "Total:%d,Ack:%d,NAck:%d", moduleState.tcpTotal, moduleState.tcpAck, moduleState.tcpNack);

}



void cipsendParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+CME ERROR:", len);
    if (index < 0)
    {
        return;
    }
    LogMessage(DEBUG_ALL, "socket send error");
    sendModuleCmd(CIPSTATUS_CMD, NULL);
}



/*
C: 0,0,"TCP","47.107.25.39","9998","CONNECTED"
C: 1,,"","","","INITIAL"
C: 2,,"","","","INITIAL"
C: 3,,"","","","INITIAL"
C: 4,0,"TCP","47.106.96.28","10188","CLOSED"
C: 5,,"","","","INITIAL"


STATE: IP INITIAL


*/
void cipstatusParser(uint8_t *buf, uint16_t len)
{
    int index;
    ITEM item;
    uint8_t *rebuf, link;
    int16_t relen;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "IP INITIAL", relen);
    if (index > 0)
    {
        LogMessage(DEBUG_ALL, "All socket lost");
        socketDelAll();
        return;
    }

    index = my_getstrindex(rebuf, "C:", relen);
    while (index >= 0)
    {
        rebuf += index + 3;
        relen -= index + 3;

        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0)
        {
            stringToItem(&item, rebuf, index);
            link = atoi(item.item_data[0]);
            if (my_strstr(item.item_data[5], "CONNECTED", strlen(item.item_data[5])))
            {
                socketSetConnState(link, SOCKET_CONN_SUCCESS);
            }
            else// if (my_strstr(item.item_data[5], "CLOSED", strlen(item.item_data[5])))
            {
                socketSetConnState(link, SOCKET_CONN_ERR);
            }



            rebuf += index;
            relen -= index;
        }

        index = my_getstrindex(rebuf, "C:", relen);
    }

}

/**************************************************
@bref		ģ������ݽ��ս�����
@param
@return
@note
**************************************************/

void moduleRecvParser(uint8_t *buf, uint16_t bufsize)
{
    static uint16_t len = 0;
    static uint8_t dataRestore[MODULE_RX_BUFF_SIZE + 1];
    if (bufsize == 0)
        return;
    if (len + bufsize > MODULE_RX_BUFF_SIZE)
    {
        len = 0;
        bufsize %= MODULE_RX_BUFF_SIZE;
        LogMessage(DEBUG_ALL, "UartRecv Full!!!");
    }
    memcpy(dataRestore + len, buf, bufsize);
    len += bufsize;
    dataRestore[len] = 0;
    if (dataRestore[len - 1] != '\n')
    {
        if (dataRestore[2] != '>')
        {
            return;
        }
    }
    LogPrintf(DEBUG_ALL, "--->>>---0x%X [%d]", dataRestore, len);
    LogMessageWL(DEBUG_ALL, (char *)dataRestore, len);
    LogMessage(DEBUG_ALL, "---<<<---");
    /*****************************************/
    moduleRspSuccess();
    cmtiParser(dataRestore, len);
    cmgrParser(dataRestore, len);
    nmeaParser(dataRestore, len);
    cipstartRspParser(dataRestore, len);
    wifiscanParser(dataRestore, len);
    if (ciprxgetParser(dataRestore, len))
    {
        if (moduleState.cmd == CIPRXGET_CMD)
        {
            return;
        }
    }

    /*****************************************/
    switch (moduleState.cmd)
    {
        case AT_CMD:
            if (distinguishOK((char *)dataRestore))
            {
                moduleState.atResponOK = 1;
            }
            break;
        case CPIN_CMD:
            if (my_strstr((char *)dataRestore, "+CPIN: READY", len))
            {
                moduleState.cpinResponOk = 1;
            }
            break;
        case CSQ_CMD:
            csqParser(dataRestore, len);
            break;
        case CGREG_CMD:
        case CEREG_CMD:
            cgregParser(dataRestore, len);
            break;
        case CIMI_CMD:
            cimiParser(dataRestore, len);
            break;
        case ICCID_CMD:
            iccidParser(dataRestore, len);
            break;
        case CGSN_CMD:
            cgsnParser(dataRestore, len);
            break;
        case CBC_CMD:
            cbcParser(dataRestore, len);
            break;
        case CGATT_CMD:
            cgattParser(dataRestore, len);
            break;
        case CIPACK_CMD:
            cipackParser(dataRestore, len);
            break;
        case CIPSTART_CMD:
            cipstartParser(dataRestore, len);
            break;
        case CIPSTATUS_CMD:
            cipstatusParser(dataRestore, len);
            break;
        case CIPSEND_CMD:
            cipsendParser(dataRestore, len);
            break;
        default:
            break;
    }
    moduleState.cmd = 0;
    len = 0;
}
/*--------------------------------------------------------------*/

/**************************************************
@bref		�����ź�����ʱ��
@param
@return
@note
**************************************************/

void netResetCsqSearch(void)
{
    moduleCtrl.csqTime = 90;
}

/**************************************************
@bref		socket��������
@param
@return
@note
**************************************************/

int socketSendData(uint8_t link, uint8_t *data, uint16_t len)
{
    int ret = 0;
    char param[10];

    if (socketGetConnStatus(link) == 0)
    {
        //��·δ����
        return 0;
    }

    sprintf(param, "%d,%d", link, len);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPSEND_CMD, param);
    createNode((char *)data, len, CIPSEND_CMD);
    if (link == NORMAL_LINK || link == JT808_LINK)
    {
        moduleState.tcpNack = len;
    }
    return len;
}
/**************************************************
@bref		ģ��˯�߿���
@param
@return
@note
**************************************************/
void moduleSleepCtl(uint8_t onoff)
{

}

/**************************************************
@bref		��ȡCSQ
@param
@return
@note
**************************************************/

void moduleGetCsq(void)
{
    sendModuleCmd(CSQ_CMD, NULL);
}

/**************************************************
@bref		��ȡ��վ
@param
@return
@note
**************************************************/

void moduleGetLbs(void)
{
    sendModuleCmd(CGREG_CMD, "?");
    sendModuleCmd(CEREG_CMD, "?");
}
/**************************************************
@bref		��ȡWIFIscan
@param
@return
@note
**************************************************/

void moduleGetWifiScan(void)
{
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(WIFISCAN_CMD, NULL);

}

/**************************************************
@bref		���Ͷ���Ϣ
@param
@return
@note
**************************************************/

void sendMessage(uint8_t *buf, uint16_t len, char *telnum)
{
    char param[60];
    sprintf(param, "\"%s\"", telnum);
    sendModuleCmd(CMGF_CMD, "1");
    sendModuleCmd(CMGS_CMD, param);
    buf[len] = 0x1A;
    createNode((char *)buf, len + 1, CMGS_CMD);
}
/**************************************************
@bref		ɾ�����ж���Ϣ
@param
@return
@note
**************************************************/


void deleteMessage(void)
{
    sendModuleCmd(CMGD_CMD, "0,4");
}

/**************************************************
@bref		��ѯ�����Ƿ������
@param
@return
@note
**************************************************/

void querySendData(uint8_t link)
{
    char param[5];
    sprintf(param, "%d", link);
    sendModuleCmd(CIPACK_CMD, param);
}


/**************************************************
@bref		��ѯģ���ص�ѹ
@param
@return
@note
**************************************************/

void queryBatVoltage(void)
{
    //    sendModuleCmd(AT_CMD, NULL);
    //    sendModuleCmd(CBC_CMD, NULL);
}

/**************************************************
@bref		��ȡ�ź�ֵ
@param
@return
@note
**************************************************/

uint8_t getModuleRssi(void)
{
    return moduleState.rssi;
}

/**************************************************
@bref		��ȡIMSI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMSI(void)
{
    return moduleState.IMSI;
}
/**************************************************
@bref		��ȡIMEI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMEI(void)
{
    return moduleState.IMEI;
}



/**************************************************
@bref		��ȡICCID
@param
@return
@note
**************************************************/

uint8_t *getModuleICCID(void)
{
    return moduleState.ICCID;
}

/**************************************************
@bref		��ȡMCC
@param
@return
@note
**************************************************/

uint16_t getMCC(void)
{
    return moduleState.mcc;
}

/**************************************************
@bref		��ȡMNC
@param
@return
@note
**************************************************/

uint8_t getMNC(void)
{
    return moduleState.mnc;
}

/**************************************************
@bref		��ȡLAC
@param
@return
@note
**************************************************/

uint16_t getLac(void)
{
    return moduleState.lac;
}

/**************************************************
@bref		��ȡCID
@param
@return
@note
**************************************************/

uint32_t getCid(void)
{
    return moduleState.cid;
}

/**************************************************
@bref		��ȡδ�����ֽ������ж��Ƿ��ͳɹ�
@param
@return
@note
**************************************************/

uint32_t getTcpNack(void)
{
    return moduleState.tcpNack;
}

/**************************************************
@bref		ģ���Ƿ�ﵽ����״̬
@param
@return
@note
**************************************************/

uint8_t isModuleRunNormal(void)
{
    if (moduleState.fsmState == NORMAL_STATUS)
        return 1;
    return 0;
}

/**************************************************
@bref		ģ��ﵽ��������
@param
@return
@note
**************************************************/

uint8_t isModulePowerOnOk(void)
{
    if (moduleState.powerOnTick > 10)
        return 1;
    return 0;
}

/**************************************************
@bref		�Ҷϵ绰
@param
@return
@note
**************************************************/

void stopCall(void)
{
    sendModuleDirect("ATH\r\n");
}
/**************************************************
@bref		����绰
@param
@return
@note
**************************************************/

void callPhone(char *tel)
{
    char param[50];
    sprintf(param, "ATD%s;\r\n", tel);
    LogPrintf(DEBUG_ALL, "Call %s", tel);
    sendModuleDirect(param);

}



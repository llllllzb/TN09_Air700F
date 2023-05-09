#ifndef APP_PORT
#define APP_PORT

#include <stdint.h>
#include "config.h"
#define APPUSART0_BUFF_SIZE 10
#define APPUSART1_BUFF_SIZE 10
#define APPUSART2_BUFF_SIZE 128
#define APPUSART3_BUFF_SIZE 768

//4G模组涉及IO
#define SUPPLY_PIN      
#define PORT_SUPPLY_ON  
#define PORT_SUPPLY_OFF 

#define POWER_PIN    	GPIO_Pin_13 //PA13
#define PORT_PWRKEY_H   GPIOA_ResetBits(POWER_PIN)
#define PORT_PWRKEY_L   GPIOA_SetBits(POWER_PIN)

#define RST_PIN			GPIO_Pin_12 //PA12	
#define PORT_RSTKEY_H	GPIOA_ResetBits(RST_PIN)
#define PORT_RSTKEY_L	GPIOA_SetBits(RST_PIN)


#define RING_PIN			//PA14

#define DTR_PIN				//PA15
#define DTR_HIGH		
#define DTR_LOW			

////DA217涉及IO
//#define GSPWR_PIN       GPIO_Pin_9
//#define GSPWR_ON        portGsensorPwrCtl(1)
//#define GSPWR_OFF       portGsensorPwrCtl(0)
//#define GSINT_PIN       GPIO_Pin_12
//IIC 涉及IO
#define SCL_PIN         GPIO_Pin_14
#define SDA_PIN         GPIO_Pin_15

//LED 涉及IO
#define LED1_PIN        GPIO_Pin_7	//PB7
#define LED1_ON         GPIOB_SetBits(LED1_PIN)
#define LED1_OFF        GPIOB_ResetBits(LED1_PIN)
#define LED2_PIN
#define LED2_ON
#define LED2_OFF

 

#define VCARD_ADCPIN	GPIO_Pin_9	//PA9
#define ADC_CHANNEL		CH_EXTIN_13



typedef enum
{
    APPUSART0,
    APPUSART1,
    APPUSART2,
    APPUSART3,
} UARTTYPE;

typedef struct
{
    uint8_t *rxbuf;
    uint8_t init;
    uint16_t rxbufsize;
    __IO uint16_t rxbegin;
    __IO uint16_t rxend;

    void (*rxhandlefun)(uint8_t *, uint16_t len);
    void (*txhandlefun)(uint8_t *, uint16_t len);

} UART_RXTX_CTL;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} system_date_s;

extern UART_RXTX_CTL usart0_ctl;
extern UART_RXTX_CTL usart1_ctl;
extern UART_RXTX_CTL usart2_ctl;
extern UART_RXTX_CTL usart3_ctl;

void pollUartData(void);
void portUartCfg(UARTTYPE type, uint8_t onoff, uint32_t baudrate, void (*rxhandlefun)(uint8_t *, uint16_t len));
void portUartSend(UART_RXTX_CTL *uartctl, uint8_t *buf, uint16_t len);

void portModuleGpioCfg(uint8_t state);

void portLedGpioCfg(void);
void portGpsGpioCfg(void);
void portGpioSetDefCfg(void);

void portSysReset(void);

uint8_t iicReadData(uint8_t addr, uint8_t regaddr, uint8_t *data, uint8_t len);
uint8_t iicWriteData(uint8_t addr, uint8_t reg, uint8_t data);
void portGsensorPwrCtl(uint8 onoff);
void portIICCfg(void);
void portRollCfg(void);
void portGsensorCtl(uint8_t onoff);

void portRtcCfg(void);
void portGetRtcDateTime(uint16_t *year, uint8_t *month, uint8_t *date, uint8_t *hour, uint8_t *minute, uint8_t *second);
void portUpdateRtcDateTime(uint8_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second);
int portSetNextAlarmTime(void);
void portSetNextWakeUpTime(void);

void portLowPowerCfg(void);


void portAdcCfg(uint8_t onoff);
float portGetAdcVol(ADC_SingleChannelTypeDef channel);

void portSleepEn(void);
void portSleepDn(void);

void portWdtCfg(void);
void portWdtFeed(void);
void portFsclkChange(uint8_t type);

#endif

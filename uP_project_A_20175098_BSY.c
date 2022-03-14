/*
 * 2019-03-13   Programmer : �� �� ��
 * MCU : TM4C1294NCPDT
 *
 * GPIO
 * ����ġ ȸ��
 * ���ͷ�Ʈ ��� X , Ǯ������ġ ���, PortN �� PortJ ���
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"

#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"

#include "uartstdio.h"

unsigned int ADC[8];
unsigned int light, sound, outside, sleep;
uint32_t ulPeriod,ulPeriod_Servo, ui32SysClock, timer;
int f_random, n_random, i;
volatile bool buzzer_on, isHumanIn, isAuto, isLightOn, isMorningCallOn, isCurtainOpen, isCurtainOn, isTimerOn, isMicrowaveOn,isMwFinish;
double duty_ratio;
const double LEFT = 0.99;
const double RIGHT = 0.01;

void ADCSeq0Handler(){                      // ����� �ڵ���� ��� ��������

    ADCSequenceDataGet(ADC0_BASE,0,ADC);

    light = ADC[0];

    ADCIntClear(ADC0_BASE,0);
}

void ADCSeq1Handler(){                      // Ŀư ��� �Ҹ�����

    ADCSequenceDataGet(ADC0_BASE,1,ADC);

    sound = ADC[1];

    ADCIntClear(ADC0_BASE,1);
}

void ADCSeq2Handler(){                      // ħ�뿡 ����ִ°��� ���� ���� �Ǻ� ��������

    ADCSequenceDataGet(ADC0_BASE,2,ADC);

    sleep = ADC[2];

    ADCIntClear(ADC0_BASE,2);
}

void ADC1Seq0Handler(){                      // ���� ������ ��ο��� ���� �Ǻ� ��������

    ADCSequenceDataGet(ADC1_BASE,0,ADC);

    outside = ADC[3];

    ADCIntClear(ADC1_BASE,0);
}
void Int_Timer0(){

    if(isTimerOn){
        UARTprintf("Time : %d\n",timer--);
    }
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
}

void Int_GPIOD(){
    if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0)==0X0){
        isTimerOn = true;
        isMicrowaveOn = true;
        timer = timer + 5;
    }
    for(i = 0; i<300000;i++);
    GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_0);
}

void SystemSetup(){

    ui32SysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 25000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_INT_PIN_1| GPIO_INT_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 |  GPIO_INT_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);

    IntMasterEnable();
}

void UARTInit(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    UARTStdioConfig(0, 115200, ui32SysClock);
}

void ADCInit(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1));

    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH18 | ADC_CTL_IE | ADC_CTL_END); // pk2 light cds
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END); // pk3 sound
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END); // pk1 morning call cds
    ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH16 | ADC_CTL_IE | ADC_CTL_END); // pk0 ouside cds

    ADCSequenceEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCSequenceEnable(ADC1_BASE, 0);

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 0);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 0);
}

void PWMInit(){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    ulPeriod = ui32SysClock / 640;
    ulPeriod_Servo = ui32SysClock / 400 ;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ulPeriod_Servo);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ulPeriod*0.5);


    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
}

void TIMERTInit(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    GPIODirModeSet(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE,TIMER_A,ui32SysClock);

    IntEnable(INT_TIMER0A);
    IntEnable(INT_GPIOJ);

    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_0);

    IntEnable(INT_TIMER0A);
    IntEnable(INT_GPIOD);
}

void checkFrontDoorLamp(){

    if(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_2)==4)
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3, 0X08);
    else
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3, 0X00);
}

void checkLightOnOff(){

    if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==2)
        isLightOn = true;
    else
        isLightOn = false;
}

void checkLightAuto(){

    if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)==0)         // ����� ���� ���
        isAuto = false;
    else {
        isAuto = true;                                      // ����� �ڵ� ���

        ADCIntEnable(ADC0_BASE, 0);

        IntEnable(INT_ADC0SS0);

        ADCProcessorTrigger(ADC0_BASE, 0);
    }
}

void lightOn(){

    f_random = rand() % 3 +1;
    n_random = rand() % 2;

    GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0|GPIO_PIN_1, f_random);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4, n_random + 0XF);

    for(i = 0; i< 400000; i++);
}

void lightOff(){

    GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0|GPIO_PIN_1,0x00);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,0x00);
}

void checkCurtainOnOff(){

    if(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_1)==2){
        isCurtainOn = true;

        ADCIntEnable(ADC0_BASE, 1);

        IntEnable(INT_ADC0SS1);

        ADCProcessorTrigger(ADC0_BASE, 1);
    }
    else
        isCurtainOn = false;
}

void CurtainMove(){

    if(isCurtainOpen){
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ulPeriod_Servo*LEFT);
        isCurtainOpen = false;
        sound = 0;
        for(i = 0; i < 10000; i++);
    }
    else{
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ulPeriod_Servo*RIGHT);
        isCurtainOpen = true;
        sound = 0;
        for(i = 0; i < 10000; i++);
    }
}


void checkMorningCallOnOff(){

    if(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_0)==1){
        isMorningCallOn = true;

        ADCIntEnable(ADC0_BASE, 2);
        ADCIntEnable(ADC1_BASE, 0);

        IntEnable(INT_ADC0SS2);
        IntEnable(INT_ADC1SS0);

        ADCProcessorTrigger(ADC0_BASE, 2);
        ADCProcessorTrigger(ADC1_BASE, 0);
    }
    else
        isMorningCallOn = false;

}

void Buzzer(){

    int i;

    if(buzzer_on){


        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<700000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<700000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<2000000;i++);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<700000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<700000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<2000000;i++);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<1000000;i++);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 880); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 500000000000); // x
        for(i=0;i<550000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<400000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 500000000000); // X
        for(i=0;i<550000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<1200000;i++);

/*
//        ����(��)��(��)(��)(��)(��)(��)�ּּ��ļֶ󵵼�
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<1000000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 523); //��
        for(i=0;i<1000000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 493); //������
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 523); //��
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 493); //������
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 440); //������
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 391); //������
        for(i=0;i<1000000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 440); //������
        for(i=0;i<1000000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 493); //������
        for(i=0;i<1200000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<1000000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 880); //��
        for(i=0;i<800000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<1000000;i++);
*/
        /*
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 523); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 587); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 659); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 698); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 739); //��#
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 783); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 880); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 987); //��
        for(i=0;i<1500000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 1046); //��
        for(i=0;i<1500000;i++);*/

        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    }

    if(isMwFinish){
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 1046); //��
        for(i=0;i<600000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 500000000000); // x
        for(i=0;i<550000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 1046); //��
        for(i=0;i<600000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 500000000000); // X
        for(i=0;i<550000;i++);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32SysClock / 1046); //��
        for(i=0;i<1200000;i++);

        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    }

}

void microwaveOn(){

    if(isMicrowaveOn){
        while(timer != -1){
            isMicrowaveOn = false;

            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5, 0X20);
        }

        isTimerOn = false;
        timer = 0;

        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5, 0X0);

        UARTprintf("******* Mw Fnish *******\n");

        isMwFinish = true;
        Buzzer();
        isMwFinish = false;

    }else
        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5, 0X0);

}

int main(void) {

    SystemSetup();
    UARTInit();
    ADCInit();
    PWMInit();
    TIMERTInit();

    srand(time(NULL));

    buzzer_on=false;
    isHumanIn = false;
    isAuto = false;
    isLightOn = false;
    isCurtainOn = false;
    isCurtainOpen = true;
    isMorningCallOn = false;
    isTimerOn = false;
    isMwFinish - false;

    while(1){

        // ������ ������ ����
        checkFrontDoorLamp();

        // ����� ����
        checkLightOnOff();

        if(isLightOn){                  // ����� ���� on/off �Ǻ�
            checkLightAuto();           // �ڵ� ������� ���� ������� �Ǻ�
            if(!isAuto)                 // ���� ���
                lightOn();
            else {                      // �ڵ� ���
                if(light > 1000)        // ��ο����� �� ��
                    lightOn();
                else
                    lightOff();
            }
        }
        else
            lightOff();

        // �ڵ� Ŀư ����
        checkCurtainOnOff();        // Ŀư ����

        if(isCurtainOn){
            if(sound > 1500){       // ���� �Ҹ� �̻�(ex. �ڼ��Ҹ�)�� ���ļ��� �� �� Ŀư ������
                CurtainMove();
            }
        }

        // ����� ����
        checkMorningCallOnOff();    // ����� ����

        if(isMorningCallOn){
            if(outside < 1000 && sleep > 1500){      // ��ħ�ε� �ڰ� ���� �� ����� ��
                buzzer_on=true;
                Buzzer();
                buzzer_on=false;
            }
        }

        // ���ڷ����� ����
        microwaveOn();
    }
}

/** ----------------
 * Triphase inverter code
 * -----------------
 *
 * Inverter code designed to drive a Triphase inverter
 *
 * Designed for a Tiva C Series board
 * With an ARM Cortex-M4F TM4C123GH6PM 80MHz
 *
 */


#include "inverter.h"
#include "sen_table.h"
#include <math.h>


#define PWM_GEN_BADDR(_mod_, _gen_)                                           \
                                ((_mod_) + (_gen_))
#define PWM_GEN_EXT_BADDR(_mod_, _gen_)                                       \
                                ((_mod_) + PWM_GEN_EXT_0 +                    \
                                 ((_gen_) - PWM_GEN_0) * 2)
#define PWM_OUT_BADDR(_mod_, _out_)                                           \
                                ((_mod_) + ((_out_) & 0xFFFFFFC0))
#define PWM_IS_OUTPUT_ODD(_out_)                                              \
                                ((_out_) & 0x00000001)


/**
 * Error handleling routine
 */

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//Global variables
uint16_t PWMfrequency = 5000;
uint16_t DBrise = 0.01;
uint16_t DBfall = 0.01;
uint16_t Updatefrequency = 60;
float senamp = 0.96f;
uint16_t sentablesize;



void SetTimers(uint32_t period)
{
    //Enable the module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    //Configure the timer
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    //Set timer period
    TimerLoadSet(TIMER2_BASE, TIMER_A, period);

    // Enable timers interrupts
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Enable timers
    TimerEnable(TIMER2_BASE, TIMER_A);
}



void TimerUpdatePeriod(uint32_t period)
{
    TimerLoadSet(TIMER2_BASE, TIMER_A, period);
}



void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}



void SetBothCompSen(uint32_t ui32Base, uint32_t ui32PWMOut, uint16_t iterator)
{
    uint32_t baseaddress;
    uint16_t loadval;
    uint16_t compval;

    baseaddress = PWM_OUT_BADDR(ui32Base, ui32PWMOut); // Get the output address

    loadval = HWREG(baseaddress + PWM_O_X_LOAD); //Get the maximum value

    compval = loadval * (float)((senamp*(float)sen_table[iterator] / INT16_MAX)+0.5f); //Calculate the compvalue in respect to the maximum load
    //compval = loadval * iterator/sentablesize;
    HWREG(baseaddress + PWM_O_X_CMPA) = compval;
    HWREG(baseaddress + PWM_O_X_CMPB) = loadval - compval + 1; // +1 to avoid underflow
}



void SetPWMDrivers(uint32_t PWMperiod, float DeadBandRise, float DeadBandFall)
{
    //Division of system clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Each PWM block have 4 PWM generators each with 2 outputs;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Configure pins
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    GPIOPinConfigure(GPIO_PE4_M1PWM2);
    GPIOPinConfigure(GPIO_PE5_M1PWM3);

    //Set the PWM to the pins
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);

    //Configuration of the PWM generators
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_SYNC|PWM_GEN_MODE_DBG_RUN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_SYNC|PWM_GEN_MODE_DBG_RUN);

    //Set period measured in clock ticks
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWMperiod);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMperiod);

    //Activate output
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

    //Enable deadband
    PWMDeadBandEnable(PWM1_BASE, PWM_GEN_0, DeadBandRise*PWMperiod, DeadBandFall*PWMperiod);
    PWMDeadBandEnable(PWM1_BASE, PWM_GEN_1, DeadBandRise*PWMperiod, DeadBandFall*PWMperiod);


    //Enable generators
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}



uint16_t senvalA = 0;
uint16_t senvalB = 0;
void Timer2IntHandler(void)
{
    //Clear interrupt
    ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //SetCMPvalues A
    SetBothCompSen(PWM1_BASE, PWM_OUT_0, senvalA);
    if(senvalA >= (sentablesize - 1));
        senvalA = senvalA % sentablesize;
    senvalA++;

    //SetCMPvalues B
    SetBothCompSen(PWM1_BASE, PWM_OUT_2, senvalB);
    if(senvalB >= (sentablesize - 1));
        senvalB = senvalB % sentablesize;
    senvalB++;

    //Sync
    PWMSyncUpdate(PWM1_BASE,PWM_GEN_0_BIT|PWM_GEN_1_BIT|PWM_GEN_3_BIT);
    PWMSyncTimeBase(PWM1_BASE,PWM_GEN_0_BIT|PWM_GEN_1_BIT|PWM_GEN_3_BIT);
}



int main(void)
{

    //For interrupt float-point instructions
    FPULazyStackingEnable();

    //PLL=200MHz, SYSCTL_SYSDIV_5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);



    sentablesize = sizeof(sen_table)/sizeof(*sen_table);
    uint32_t systemclock = SysCtlClockGet();

    senvalA = 0; //Defasagem de 0
    senvalB = sentablesize/2; //Defasagem de 180

    //Configure
    SetPWMDrivers(systemclock/PWMfrequency, DBrise, DBfall);
    SetTimers(systemclock/(Updatefrequency*sentablesize));
    ConfigureUART();

    UARTprintf(" PWM Rodando \n");

    //Enable interrupt
    IntMasterEnable();


    int i = 0;
    while(1){
        TimerUpdatePeriod(systemclock/(Updatefrequency*sentablesize));
        i = 0;
        while(i<100000)
            i++;
    };

    //return 0;
}

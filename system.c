// Configuration bits: selected in the GUI

// FSEC
#pragma config BWRP = OFF    // Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED    // Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF    // Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF    // General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED    // General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    // Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED    // Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF    // Alternate Interrupt Vector Table bit->Disabled AIVT

// FOSCSEL
#pragma config FNOSC = FRCPLL    // Oscillator Source Selection->Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
#pragma config PLLMODE = PLL96DIV2    // PLL Mode Selection->96 MHz PLL. (8 MHz input)
#pragma config IESO = ON    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFCN = ON    // OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config SOSCSEL = ON    // SOSC Power Selection Configuration bits->SOSC is used in crystal (SOSCI/SOSCO) mode
#pragma config PLLSS = PLL_FRC    // PLL Secondary Selection Configuration bit->PLL is fed by the on-chip Fast RC (FRC) oscillator
#pragma config IOL1WAY = ON    // Peripheral pin select configuration bit->Allow only one reconfiguration
#pragma config FCKSM = CSDCMD    // Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are disabled

// FWDT
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config FWPSA = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config FWDTEN = OFF    // Watchdog Timer Enable bits->WDT and SWDTEN disabled
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config WDTWIN = WIN25    // Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config WDTCMX = WDTCLK    // WDT MUX Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC

// FPOR
#pragma config BOREN = ON    // Brown Out Enable bit->Brown Out Enable Bit
#pragma config LPCFG = OFF    // Low power regulator control->No Retention Sleep
#pragma config DNVPEN = ENABLE    // Downside Voltage Protection Enable bit->Downside protection enabled using ZPBOR when BOR is inactive

// FICD
#pragma config ICS = PGD2    // ICD Communication Channel Select bits->Communicate on PGEC2 and PGED2
#pragma config JTAGEN = OFF    // JTAG Enable bit->JTAG is disabled
#pragma config BTSWP = OFF    // BOOTSWP Disable->BOOTSWP instruction disabled

// FDEVOPT1
#pragma config ALTCMPI = DISABLE    // Alternate Comparator Input Enable bit->C1INC, C2INC, and C3INC are on their standard pin locations
#pragma config TMPRPIN = OFF    // Tamper Pin Enable bit->TMPRN pin function is disabled
#pragma config SOSCHP = ON    // SOSC High Power Enable bit (valid only when SOSCSEL = 1->Enable SOSC high power mode (default)
#pragma config ALTVREF = ALTREFEN    // Alternate Voltage Reference Location Enable bit->VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9

// FBOOT
#pragma config BTMODE = SINGLE    // Boot Mode Configuration bits->Device is in Single Boot (legacy) mode

#include <p24FJ1024GB610.h>
#include <xc.h>

#include "main.h"
#include "system.h"
#include "roboclaw.h"
#include "motion.h"
#include "lcd.h"

#define ERROR_HANDLER __attribute__((interrupt,no_auto_psv))
#define ERROR_HANDLER_NORETURN ERROR_HANDLER __attribute__((noreturn))
#define FAILSAFE_STACK_GUARDSIZE 8


// Init Globals
uint8_t uart1_RxPacket[MAX_RECIEVED_PACKET_LENGTH_ROBOCLAW] = {0};
uint16_t Rx_DataIndex = 0;
bool Roboclaw_DataRecieved = false;
bool b_glb_ThermoScanServoPulseHigh = false;
bool b_glb_ThermoTiltPulseHigh = false;
bool b_glb_DistTOFServoTurnPulseHigh = false;
uint8_t glb_numBytesToWaitForRoboclaw = 0;

//*************************
//  INTERRUPTS
//*************************

// UART 1 Interrupt
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void) // Interrupts on recieving data in RX Register for UART 1
{
    uart1_RxPacket[Rx_DataIndex] = U1RXREG;
    Rx_DataIndex++;

//    if (Rx_DataIndex >= glb_numBytesToWaitForRoboclaw)
//    {
//        Roboclaw_DataRecieved = true;
//        Rx_DataIndex = 0;
//    } 
    if (Rx_DataIndex >= MAX_RECIEVED_PACKET_LENGTH_ROBOCLAW)
    {
        Rx_DataIndex = 0;
    }
    
    IFS0bits.U1RXIF = 0; // clears interrupt
}

//TODO: U1ERR interrupt needed?

// Timer 1 interrupt -- 0.2 sec period HEARTBEAT
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  ) // Interrupts every 0.2 sec to update ThermoScanServo and start Timer 2
{
    TMR1_Stop();
    //setting up the pulse for the first motor
    if (!b_glb_ThermoScanServoPulseHigh && !b_glb_ThermoTiltPulseHigh && !b_glb_DistTOFServoTurnPulseHigh)
    {
        ThermoServoScanIO_SetHigh();
    }
    PR2 = 2 * u16_glb_ThermoScanServoPW;
    b_glb_ThermoScanServoPulseHigh = true;
    TMR2_Start(); // This will trigger _T2Interrupt() which updates the other servos
    IFS0bits.T1IF = false;
   
}
// Timer 2 interrupt
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  ) // Handles servo controls in conjunction with Timer 1
{
    //stopping TMR2 - we got here after the first motor was started
    if (b_glb_ThermoScanServoPulseHigh && !b_glb_ThermoTiltPulseHigh && !b_glb_DistTOFServoTurnPulseHigh)
    {
         ThermoServoScanIO_SetLow();
         b_glb_ThermoScanServoPulseHigh = false;
         TMR2_Stop();
         IFS0bits.T2IF = false; //clearing up the interrupt flag        
        ThermoServoTiltIO_SetHigh();
        PR2 = 2 * u16_glb_ThermoTiltPW;
        b_glb_ThermoTiltPulseHigh = true;
        TMR2_Start();
        return;
    }
    //we get here only after the second motor pulse is taken care of
    else if (b_glb_ThermoTiltPulseHigh && !b_glb_ThermoScanServoPulseHigh && !b_glb_DistTOFServoTurnPulseHigh)
    {
         ThermoServoTiltIO_SetLow();
         b_glb_ThermoTiltPulseHigh = false;
         //done with all motors - stopping timers
         TMR2_Stop(); 
         IFS0bits.T2IF = false; //clearing up the interrupt flag
         DistTOFServoTurnIO_SetHigh();
         PR2 = 2 * u16_glb_TOFServoPW;
         b_glb_DistTOFServoTurnPulseHigh = true;
         TMR2_Start();
         return;
    }
    else if (b_glb_DistTOFServoTurnPulseHigh && !b_glb_ThermoTiltPulseHigh && !b_glb_ThermoScanServoPulseHigh)
    {
        DistTOFServoTurnIO_SetLow();
        b_glb_DistTOFServoTurnPulseHigh = false;
        TMR2_Stop();
        IFS0bits.T2IF = false;
        IFS0bits.T1IF = false;
        TMR1_Start();
        return;
    }
    else //by default stop the two timers
    {        
        TMR2_Stop();
        IFS0bits.T2IF = false;
    }
}

//*********8 EXCEPTIONS : *****************************************
//**********************************************************************************************************************

void __attribute__((naked, noreturn, weak)) TRAPS_halt_on_error(uint16_t code)
{
  //TODO  TRAPS_ERROR_CODE errorCode = code;
#ifdef __DEBUG    
    __builtin_software_breakpoint();
    /* If we are in debug mode, cause a software breakpoint in the debugger */
#endif
    asm ("RESET");
    while(1);
    
}

/**
 * Sets the stack pointer to a backup area of memory, in case we run into
 * a stack error (in which case we can't really trust the stack pointer)
 */
inline static void use_failsafe_stack(void)
{
    static uint8_t failsafe_stack[32];
    asm volatile (
        "   mov    %[pstack], W15\n"
        :
        : [pstack]"r"(failsafe_stack)
    );
/* Controls where the stack pointer limit is, relative to the end of the
 * failsafe stack
 */    
    SPLIM = (uint16_t)(((uint8_t *)failsafe_stack) + sizeof(failsafe_stack) 
            - FAILSAFE_STACK_GUARDSIZE);
}

/** Oscillator Fail Trap vector**/
void ERROR_HANDLER_NORETURN _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_OSC_FAIL);
}

/** Stack Error Trap Vector**/
//void ERROR_HANDLER_NORETURN _SfStackError(void)
//{
//    /* We use a failsafe stack: the presence of a stack-pointer error
//     * means that we cannot trust the stack to operate correctly unless
//     * we set the stack pointer to a safe place.
//     */
//    use_failsafe_stack(); 
//    INTCON1bits.STKERR = 0;  //Clear the trap flag
//    TRAPS_halt_on_error(TRAPS_STACK_ERR);
//}

/** Address Error Trap Vector**/
void ERROR_HANDLER_NORETURN _AddressError(void)
{
    INTCON1bits.ADDRERR = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_ADDRESS_ERR);
}

/** Math Error Trap Vector**/
void ERROR_HANDLER_NORETURN _MathError(void)
{
    INTCON1bits.MATHERR = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_MATH_ERR);
}

// System Reset Function -- Saves all data then hard reboots the controller -- Only done if something very bad happens

void SYSTEM_Reset(void)
{
    //TODO
}

/*
 Calls all Initialize functions defined here
 */  
void SYSTEM_Initialize(void)
{
    PIN_Init();  
    OSCILLATOR_Initialize();
    
    INTCON2bits.GIE = 1;
    
    UART1_Initialize();
    //TMR1_Initialize();
    //TMR2_Initialize();
    //TMR3_Initialize();
    //TMR4_Initialize(); // NOTE: TMR4 IS NOT FUNCTIONAL -- DO NOT UTILIZE
    //TMR5_Initialize();
    //ADC1_Initialize();
    //SCCP4_CAPTURE_Initialize();
    //LCD_Initialize();
    //I2C1_Initialize();

    TMR1_Stop();
    TMR2_Stop();
    TMR3_Stop(); 
    TMR4_Stop();
    TMR5_Stop();
    
    
    //Flush_Rx_Buffer();
    //Stop_Motors();
    //Reset_Quadrature_Encoder_Counters();
    //glb_motionFailed = false;
} 

/*
 Initializes Oscillator
 */
void OSCILLATOR_Initialize(void)
{
//    // CF no clock failure; NOSC FRCPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
//    __builtin_write_OSCCONL((uint8_t) (0x0100 & 0x00FF));
//    // CPDIV 1:1; PLLEN enabled; DOZE 1:8; RCDIV FRC; DOZEN disabled; ROI disabled; 
//    CLKDIV = 0x3020;
//    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
//    OSCTUN = 0x0000;
//    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
//    REFOCONL = 0x0000;
//    // RODIV 0; 
//    REFOCONH = 0x0000;
//    // ROTRIM 0; 
//    //REFOTRIML = 0x0000;
//    // DCOTUN 0; 
//    DCOTUN = 0x0000;
//    // DCOFSEL 8; DCOEN disabled; 
//    DCOCON = 0x0700;
//    // DIV 0; 
//    OSCDIV = 0x0000;
//    // TRIM 0; 
//    OSCFDIV = 0x0000;
//    // WDTO disabled; TRAPR disabled; SLEEP disabled; BOR disabled; CM disabled; SWR disabled; SWDTEN disabled; EXTR disabled; POR disabled; SBOREN disabled; IDLE disabled; IOPUWR disabled; VREGS disabled; 
//    RCON = 0x0000;
        // CPDIV 1:1; PLLEN disabled; DOZE 1:8; RCDIV FRC; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3000;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
    OSCTUN = 0x00;
    // ROEN disabled; ROSWEN disabled; ROSEL FOSC; ROOUT disabled; ROSIDL disabled; ROSLP disabled; 
    REFOCONL = 0x00;
    // RODIV 0; 
    REFOCONH = 0x00;
    // DCOTUN 0; 
    DCOTUN = 0x00;
    // DCOFSEL 8; DCOEN disabled; 
    DCOCON = 0x700;
    // DIV 0; 
    OSCDIV = 0x00;
    // TRIM 0; 
    OSCFDIV = 0x00;
    // AD1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled; 
    PMD1 = 0x00;
    // OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled; 
    PMD2 = 0x00;
    // I2C3MD enabled; PMPMD enabled; U3MD enabled; RTCCMD enabled; CMPMD enabled; CRCMD enabled; I2C2MD enabled; 
    PMD3 = 0x00;
    // U4MD enabled; USB1MD enabled; CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
    PMD4 = 0x00;
    // IC9MD enabled; OC9MD enabled; 
    PMD5 = 0x00;
    // SPI3MD enabled; 
    PMD6 = 0x00;
    // DMA1MD enabled; DMA0MD enabled; 
    PMD7 = 0x00;
    // U5MD enabled; CLC3MD enabled; CLC4MD enabled; CLC1MD enabled; CLC2MD enabled; U6MD enabled; 
    PMD8 = 0x00;
    // CF no clock failure; NOSC FRCPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONH((uint8_t) (0x01));
    __builtin_write_OSCCONL((uint8_t) (0x01));
    // Wait for Clock switch to occur
    while (OSCCONbits.OSWEN != 0);
    while (OSCCONbits.LOCK != 1);
}

/**
 Initializes pins
*/
void PIN_Init(void)
{
    /*
     * LAT AND TRIS SETTINGS
     */
    LATA = 0x00;
    TRISA = 0xC2FE;
    //Pin 17, A0 -- Output -- LED_D3
    //Pin 29, A10 -- Output -- Ultrasound 3 Trigger
    
    LATB = 0x00;
    TRISB = 0xF3F7;
    //Pin 24, B1 -- ADC Channel 1 Input (AN1) -- Microphone Input
    //Pin 26, B6 -- PGC2 -- In-Circuit Debugger Pin
    //Pin 27, B7 -- PGD2 -- In Circuit Debugger Pin
    //Pin 32, B8 -- U1CTS -- UART 1 Clear-to-Send Pin
    //Pin 34, B10 -- Output -- LED_DET
    //Pin 35, B11 -- Output -- LED_MIC
    
    LATC = 0x00;
    TRISC = 0xF010;
    //Pin 6, C1 -- Output -- ThermoScanServoIO
    //Pin 7, C2 -- Output -- ThermoTitltServoIO
    //Pin 8, C3 -- Output -- TOFServoIO
    
    LATD = 0x8400;
    TRISD = 0xFBFF;
    //Pin 76, D1 -- SCCP ICM4 -- SCCP pin input NOTE: NOT in USE
    //Pin 83, D6 -- Input -- Button S3
    //Pin 84, D7 -- Input -- Ultrasound 4 Echo
    //Pin 68, D8 -- Input -- Ultrasound 3 Echo
    //Pin 70, D10 -- U1TX -- UART 1 Transmit Pin
    //Pin 71, D11 -- U1RX -- UART 1 Receive Pin
    //Pin 80, D13 -- Input -- Button S4
    
    LATF = 0x0020;
    TRISF = 0x31FC;
    //Pin 87, F0 -- Output -- Ultrasound 1 Trigger
    //Pin 88, F1 -- Output -- Ultrasound 6 Trigger
    //Pin 40, F12 -- Input -- Ultrasound 6 Echo
    //Pin 39, F13 -- Output -- Ultrasound 1 Echo
    
    LATG = 0x00;
    TRISG = 0xD3CF;
    //Pin 97, G13 -- Output -- Ultrasound 4 Trigger
    
    /*
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     */
    IOCPDA = 0x0000;
    IOCPDB = 0x0000;
    IOCPDC = 0x0000;
    IOCPDD = 0x0000;
    IOCPDE = 0x0000;
    IOCPDF = 0x0000;
    IOCPDG = 0x0000;
    IOCPUA = 0x0000;
    IOCPUB = 0x0000;
    IOCPUC = 0x0000;
    IOCPUD = 0x0000;
    IOCPUE = 0x0000;
    IOCPUF = 0x0000;
    IOCPUG = 0x0000;
    
    /*
     * Open Drain Settings -- ALL 0's
     */
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;
    
    /*
     * Analog/Digital Settings
     */
    ANSA = 0x02C0;
    ANSB = 0xF237;
    ANSC = 0x6010;
    ANSD = 0x0000;
    ANSE = 0x0210;
    ANSG = 0x03C0;
/****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    RPINR18bits.U1RXR = 0x000C;   //RD11->UART1:U1RX;
    RPOR1bits.RP3R = 0x0003;   //RD10->UART1:U1TX;

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    
   
}

/**
 Initializes UART 
 */
void UART1_Initialize(void)
{
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U1MODE = 0x8008;  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1STA = 0x0000;
    // BaudRate = 115200; Frequency = 16000000 Hz; U1BRG 34; 
    U1BRG = 0x22; //0x0044;
    // ADMADDR 0; ADMMASK 0; 
    U1ADMD = 0x0000;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;

    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U1MODEbits.UARTEN = 1;  // enabling UART ON bit
    U1STAbits.UTXEN = 1;
}

/*
 Sends a byte to th e UART Transmit register for transmission
 */
void UART1_SendByte (uint8_t msg)
{
    while (U1STAbits.UTXBF) {}; // waits while transmit buffer is full
    U1TXREG = msg;
    while (!U1STAbits.TRMT) {}; // waits while transmission NOT complete
}

/*
 Sends a series of bytes down the transmit line of UART1
 */
void UART1_SendBytes (uint8_t numBytes, unsigned char *msg)
{
    uint8_t i = 0;
    
    for (i = 0; i <= numBytes - 1; i++)
    {
        UART1_SendByte (msg[i]);
    }
    
}

/*
 Sends a string (char pointer) to the UART Transmit register for transmission
 */
void UART1_SendString (unsigned char *msg)
{
    uint8_t msgLen = 0;
    uint8_t i = 0;
    msgLen = strlen((char*) msg);
    for (i=0; i <= msgLen - 1; i++)
    {
        while (U1STAbits.UTXBF) {} //wait until at least one character 
        U1TXREG = msg[i];         //can be written to the transmit buffer
    }
    //now we wait for transmit shift register to get empty
    while (!U1STAbits.TRMT) {}
}

/**
 * Initializes Timer 1
 */
void TMR1_Initialize (void)
{
    //TMR1 0; 
    TMR1 = 0x0000;
    //Period = 0.02 s; Frequency = 16000000 Hz; PR1 40000; 
    PR1 = 0x9C40;
    //TCKPS 1:8; TON enabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TSYNC disabled; TGATE disabled; 
    //T1CON = 0x8010;
    // TON disabled
    T1CON = 0x0010;
    
    //    TI: T1 - Timer1
    //    Priority: 1
    IPC0bits.T1IP = 1;
    
    IFS0bits.T1IF = false;
    IEC0bits.T1IE = true;
	

}

void TMR1_Start( void )
{
     //TMR1 0; 
    TMR1 = 0x0000;
    /*Enable the interrupt*/
    IEC0bits.T1IE = true;

    /* Start the Timer */
    T1CONbits.TON = 1;
}

void TMR1_Stop( void )
{
    /* Stop the Timer */
    T1CONbits.TON = false;

    /*Disable the interrupt*/
    IEC0bits.T1IE = false;
}

/**
 * Initialized Timer 2
 */
void TMR2_Initialize (void)
{
    //TMR2 0; 
    TMR2 = 0x0000;
    //Period = 0 s; Frequency = 16000000 Hz; PR2 160; 
    //PR2 = 0x0010; //will be setting this up in TMR1 ISR
    //TCKPS 1:1; T32 16 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T2CON = 0x0010; //TMR2 period is 500ns
    
    //    TI: T2 - Timer2
    //    Priority: 1
    IPC1bits.T2IP = 1;
   
    IFS0bits.T2IF = false;
    IEC0bits.T2IE = false;

}

void TMR2_Start( void )
{
    //TMR2 0; 
    TMR2 = 0x0000;
    /*Enable the interrupt*/
    IEC0bits.T2IE = true;

    /* Start the Timer */
    T2CONbits.TON = 1;
}

void TMR2_Stop( void )
{
    /* Stop the Timer */
    T2CONbits.TON = false;

    /*Disable the interrupt*/
    IEC0bits.T2IE = false;
}

void TMR3_Initialize (void)
{
    //TMR3 0; 
    TMR3 = 0x0000;
    //Period = 0 s; Frequency = 16000000 Hz; PR3 0; 
    PR3 = 0x0000;
    //TCKPS 1:8; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T3CON = 0x0010;
}
void TMR3_Start( void )
{
     //TMR3 0; 
    TMR3 = 0x0000;
    /* Start the Timer */
    T3CONbits.TON = 1;
}

void TMR3_Stop( void )
{
    /* Stop the Timer */
    T3CONbits.TON = false;
}

void TMR4_Initialize (void)
{
    //TMR4 0; 
    T4CON = 0x00;
    IEC1bits.T4IE = false; //disabling TMR4 interrupts
    IFS1bits.T4IF = false; //clearing up any flags        
    TMR4 = 0x0000;
    //Period = 0 s; Frequency = 16000000 Hz; PR4 0; 
    PR4 = 0x0000;
    //TCKPS 1:256; T32 16 Bit; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T4CON = 0x0030;

}

void TMR4_Start( void )
{
     //TMR4 0; 
    TMR4 = 0x0000;
    /* Start the Timer */
    T4CONbits.TON = 1;
}

void TMR4_Stop( void )
{
    /* Stop the Timer */
    T4CONbits.TON = false;
}

void TMR5_Initialize (void)
{
    //TMR5 0; 
    TMR5 = 0x0000;
    //Period = 0 s; Frequency = 16000000 Hz; PR5 0; 
    PR5 = 0x0000;
    //TCKPS 1:256; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T5CON = 0x0030;
}

void TMR5_Start( void )
{
    TMR5 = 0x0000;
    /* Start the Timer */
    T5CONbits.TON = 1;
}

void TMR5_Stop( void )
{
    /* Stop the Timer */
    T5CONbits.TON = false;
}

void ADC1_Initialize (void)
{
    // ASAM disabled; DMABM disabled; ADSIDL disabled; DONE disabled; DMAEN disabled; FORM Absolute decimal result, unsigned, right-justified; SAMP disabled; SSRC Clearing sample bit ends sampling and starts conversion; MODE12 10-bit; ADON enabled; 

   AD1CON1 = 0x8000;

    // CSCNA disabled; NVCFG0 AVSS; PVCFG AVDD; ALTS disabled; BUFM disabled; SMPI 1; BUFREGEN disabled; 

   AD1CON2 = 0x0000;

    // SAMC 0; EXTSAM disabled; PUMPEN disabled; ADRC FOSC/2; ADCS 0; 

   AD1CON3 = 0x0A63;

    // CH0SA AN0; CH0SB AN0; CH0NB AVSS; CH0NA AVSS; 

   AD1CHS = 0x0000;

    // CSS25 disabled; CSS24 disabled; CSS23 disabled; CSS22 disabled; CSS21 disabled; CSS20 disabled; CSS30 disabled; CSS19 disabled; CSS18 disabled; CSS29 disabled; CSS17 disabled; CSS28 disabled; CSS16 disabled; 

   AD1CSSH = 0x0000;

    // CSS9 disabled; CSS8 disabled; CSS7 disabled; CSS6 disabled; CSS5 disabled; CSS4 disabled; CSS3 disabled; CSS2 disabled; CSS15 disabled; CSS1 disabled; CSS14 disabled; CSS0 disabled; CSS13 disabled; CSS12 disabled; CSS11 disabled; CSS10 disabled; 

   AD1CSSL = 0x0000;

    // CHH20 disabled; CHH22 disabled; CHH21 disabled; CHH24 disabled; CHH23 disabled; CHH25 disabled; CHH17 disabled; CHH16 disabled; CHH19 disabled; CHH18 disabled; 

   AD1CHITH = 0x0000;

    // CTMEN23 disabled; CTMEN24 disabled; CTMEN21 disabled; CTMEN22 disabled; CTMEN30 disabled; CTMEN20 disabled; CTMEN18 disabled; CTMEN29 disabled; CTMEN19 disabled; CTMEN16 disabled; CTMEN17 disabled; CTMEN28 disabled; CTMEN25 disabled; 

   AD1CTMENH = 0x0000;

    // VBGADC disabled; VBGUSB disabled; VBGEN disabled; VBGCMP disabled; 

   ANCFG = 0x0000;
}

uint16_t ADC1_ReadMic(void)
{
   AD1CHS = MICROPHONE_ADC_CH;
   AD1CON1bits.SAMP = 1; //initiating sampling
   __delay_us(100);
   AD1CON1bits.SAMP = 0; //end sampling, begin conversion
   while (AD1CON1bits.DONE != 1){} //wait for done bit to go high
   return ADC1BUF0;
}

void SCCP4_CAPTURE_Initialize(void)
{
  //CCPON enabled; MOD Every rising and falling edge; CCSEL enabled; CCPSIDL disabled; TMR32 32 Bit; CCPSLP disabled; TMRPS 1:1; CLKSEL FOSC/2; TMRSYNC disabled; 
    CCP4CON1L = 0x8033;
  //RTRGEN disabled; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; IOPS Each IC Event; SYNC None; OPSRC Timer Interrupt Event; 
    CCP4CON1H = 0x0000;
  //ASDGM disabled; SSDG disabled; ASDG None; PWMRSEN disabled; 
    CCP4CON2L = 0x0000;
  //ICGSM Level-Sensitive mode; ICSEL ICM4; AUXOUT Disabled; OCAEN disabled; OENSYNC disabled; 
    CCP4CON2H = 0x0000;
  //OETRIG disabled; OSCNT None; POLACE disabled; PSSACE Tri-state; 
    CCP4CON3H = 0x0000;
  //ICDIS disabled; SCEVT disabled; TRSET disabled; ICOV disabled; ASEVT disabled; TRIG disabled; TRCLR disabled; 
    CCP4STATL = 0x0000;
  //TMR 0; 
    CCP4TMRL = 0x0000;
  //TMR 0; 
    CCP4TMRH = 0x0000;
  //PR 0; 
    CCP4PRL = 0x0000;
  //PR 0; 
    CCP4PRH = 0x0000;
  //CMP 0; 
    CCP4RAL = 0x0000;
  //CMP 0; 
    CCP4RBL = 0x0000;
  //BUF 0; 
    CCP4BUFL = 0x0000;
  //BUF 0; 
    CCP4BUFH = 0x0000;
}

void SCCP4_CAPTURE_Start( void )
{
    /* Start the Timer */
    CCP4CON1Lbits.CCPON = true;
}

void SCCP4_CAPTURE_Stop( void )
{
    /* Stop the Timer */
    CCP4CON1Lbits.CCPON = false;

}

uint32_t SCCP4_CAPTURE_Data32Read( void )
{
    uint32_t captureVal = 0xFFFFFFFF;

    /* get the timer period value and return it */
    captureVal = CCP4BUFL;
    captureVal |= ((uint32_t)CCP4BUFH << 16);
    return(captureVal);
}

void Flush_Rx_Buffer (void)
  {
    uint16_t i;
    for (i = 0; i < MAX_RECIEVED_PACKET_LENGTH_ROBOCLAW; i++)
    {
        uart1_RxPacket[i] = 0;
    }
    Roboclaw_DataRecieved = false;
    Rx_DataIndex = 0;
}

void I2C1_Initialize(void)
{
    uint8_t temp;
    //I2C1BRG = 0x0006; //1MHZ    
    I2C1BRG = 0x004E; //100KHz
    I2C1CONLbits.I2CEN = 0; //Disable to start with
    I2C1CONLbits.DISSLW = 1; //slew rate control disabled
    I2C1STAT = 0x0000;
//    IPC4bits.MI2C1IP = 1; //I2C1 Master events 
//    IPC4bits.SI2C1IP = 1;
    IFS1bits.MI2C1IF = 0; //clearing up any interrupt flags
    I2C1CONLbits.I2CEN = 1; //enabling I2C1 module
    temp = I2C1RCV;
    I2C1_ResetBus(); //setting the bus to idle
    
    // ACKEN disabled; STRICT disabled; STREN disabled; GCEN disabled; SMEN disabled; DISSLW disabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled; 
    //I2C1CONL = 0x8200;
    // BCL disabled; D_nA disabled; R_nW disabled; P disabled; S disabled; I2COV disabled; IWCOL disabled; 
    

    /* MI2C1 - I2C1 Master Events */
    // clear the master interrupt flag
    IFS1bits.MI2C1IF = 0;
    
}
void I2C1_ResetBus(void)
{
    uint8_t i = 0;
    I2C1CONLbits.PEN= 1; //initates a stop bit
    Nop();
    while (I2C1CONLbits.PEN) //waiting for the process to complete:
    {        
        i++;
        if (i>20)
            break;
        
        __delay_us(1);
    }
    I2C1CONLbits.RCEN = 0;
    IFS1bits.MI2C1IF = 0; //resetting interrupt flags
    I2C1STATbits.IWCOL = 0; //resetting write collision bits
    I2C1STATbits.BCL = 0; //resetting bus collision detect bity
    __delay_us(10);
}

void I2C1_Start(void)
{
   uint8_t i = 0;
   I2C1CONLbits.ACKDT = 0; //resetting any previous acks
   __delay_us(10);
   I2C1CONLbits.SEN = 1; //initiating start condition
   Nop();
   while (I2C1CONLbits.SEN) //waiting for the automatic clearing of the start bit
   {       
       i++;
       if (i>20)
           break;
       
       __delay_us(1);
   }
   __delay_us(2);
}

void I2C1_Stop(void)
{
    uint8_t i = 0;
    I2C1CONLbits.PEN= 1; //initates a stop bit
    Nop();
    while (I2C1CONLbits.PEN) //waiting for the process to complete:
    {
        __delay_us(1);
        i++;
        if (i>20)
            break;
    }
}
void I2C1_Restart(void)
{
    uint8_t i = 0;
    I2C1CONLbits.RSEN = 1; //initiating restart condition
    Nop();
    while (I2C1CONLbits.RSEN) //waiting for automatic clearing of the re-start bit
    {         
       i++;
       if (i>20)
           break;
       
        __delay_us(1);
    }
    __delay_us(2);
}

uint8_t I2C1_SendByte(uint8_t data)
{
   uint16_t i = 0;
   for (i=0; i<500; i++)
   {
       if (!I2C1STATbits.TBF) //break out if transmit bit is empty
           break;
       
       if (i==500)
           return 1; //return an error
       
       __delay_us(1);
   }

   IFS1bits.MI2C1IF = 0; //clearing up any pending interrupts
   I2C1TRN = data; //load the data into the transmit register
   
   //waiting for the transmission to finish:
   for (i=0; i<500; i++)
   {
       if (!I2C1STATbits.TRSTAT) //break out if transmission is done
           break;
       
       if (i==500)
           return 1; //return an error
       
       __delay_us(1);
   }
   
   //checking for a no-ack:
   if (I2C1STATbits.ACKSTAT == 1) //i.e no-ack on the bus
   {
       I2C1_ResetBus();
       return 2;
   }
   
   __delay_us(2);
   return 0; //everything good
}

uint8_t I2C1_ReadNoAck(void)
{
    uint16_t i=0;
    uint8_t data = 0;
    
    I2C1CONLbits.RCEN = 1; //setting up to receive
    
    while (I2C1CONLbits.RCEN) //(!I2C2STATbits.RBF) //wait for a buffer full status bit
    {
        i++;
        if (i>2000)
            break;
        __delay_us(1);
    }
    
    data = I2C1RCV; 
    
    return data;
}

uint8_t I2C1_ReadAck(void)
{
    uint16_t i=0;
    uint8_t data = 0;
    
    I2C1CONLbits.RCEN = 1; //setting up to receive
    
    while (I2C1CONLbits.RCEN) //(!I2C2STATbits.RBF) //wait for a buffer full status bit
    {
        i++;
        if (i>2000)
            break;
        __delay_us(1);
    }
    
    data = I2C1RCV; 
    I2C1CONLbits.ACKEN = 1; //initiating the ack sequence
    __delay_us(10);
    return data;
}
/**
 * Reads two bytes at a time from the I2C2 bus
 * @param DevAddr - slave address
 * @param RegAddr - starting register to read from
 * @return 
 */
uint8_t I2C1Read8(uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t val = 0;
    I2C1_Start(); //start of frame 1
    I2C1_SendByte(DevAddr<<1 | 0); //setting up the write bit for slave
    I2C1_SendByte(RegAddr); //writing the resister address to read from
    //__delay_us(10);
    I2C1_Start(); //this will be frame 2 - repeat start
    I2C1_SendByte(DevAddr<<1 | 0x01); //setting up the slave for reads
    val = I2C1_ReadAck(); //msb
    I2C1_ResetBus(); //stop 
    return val;
}

/**
 * Writes 2 bytes at a time to the I2C2 bus
 * @param DevAddr - slave address
 * @param RegAddr - starting register address
 * @param data
 */
void I2C1Write8(uint8_t DevAddr, uint8_t RegAddr, uint8_t data)
{
    I2C1_Start();
    I2C1_SendByte(DevAddr<<1 | 0); //setting up the write bit for slave
    I2C1_SendByte(RegAddr); //writing the resister address to read from
    I2C1_SendByte(data);
    I2C1_ResetBus(); //stop 
}

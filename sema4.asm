    title    "$Id$"
    list     p=16F630
    radix    dec

;**********************************************************************
;                                                                     *
;    Description:   Quad servo driver for model railway use.          *
;                                                                     *
;    Author:        Mike Bolton                                       *
;                                                                     *
;**********************************************************************
;                                                                     *
;   Issue: 1    Rev.: B                                               *
;                                                                     *
;   Gives the pulse output for 4 conventional servos.                 *
;   Input control by on / off switches to ground.                     *
;   PIC has active pull-ups to Vdd.                                   *
;   End limits and travel speed settable with a serial input.         *
;   values stored in EEPROM.                                          *
;                                                                     *
;**********************************************************************
;                                                                     *
;   Processor speed 4MHz using internal clock                         *
;   WDT not used.                                                     *
;   Receive baud rate is 9600.                                        *
;   Command string is a null, a capital letter starting at 'A' and    *
;   a number from 0 to 255 as a three byte ascii.                     *
;                                                                     *
;   29/11/04:                                                         *
;       Working as far as two positons on each servo.                 *
;       Saves settings OK. No mid position or speed control yet.      *
;                                                                     *
;   Servo3 is Servo2 but ready for speed and centre mods.             *
;   Working with revised scheme for current position.                 *
;   Speed setting now works. 0 is maximum, 1 is slowest.              *
;   Increase number for faster. 8 is about max.                       *
;   For mid posiition, just set with PC to 127                        *
;   Defaults are mid on all settings, speed = 0 (max).                *
;   Revised for running either way.                                   *
;   Pulse width set for 1 mSec min, 2 msec max. Setting 127 is mid    *
;   way.                                                              *
;   Tested and working 3/12/04                                        *
;                                                                     *
;   4/12/04:                                                          *
;   Now Servo4. Added reset without save.                             *
;   Allows position to be set via the PC independent of switch        *
;   position.                                                         *
;   Working OK.                                                       *
;                                                                     *
;   12/04/05:                                                         *
;       Rev B. Contains the code to set the oscillator to calibrated  *
;       value.                                                        *
;                                                                     *
;   22 Jun 2008:                                                      *
;       Renamed, restructured, and refactored to decipher             *
;       functionality.                                                *
;                                                                     *
;   23 Jun 2008:                                                      *
;       Added 3 'bounce' positions in either direction.               *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

#include <p16f630.inc>

    __CONFIG   _CPD_OFF & _CP_OFF & _BODEN & _MCLRE_ON & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
; Selection is: Data Code Protection - off, Code Protection - off,
;               Brown Out Detection - on, RA3/!MCLR - !MCLR,
;               Power-up Timer - on, Watchdog Timer - off,
;               Oscillator - Internal RC oscillator (4MHz), no clock output


;**********************************************************************
; Constant definitions                                                *
;**********************************************************************

REGBANK0    EQU    0x00
REGBANK1    EQU    0x80

GETOSCCAL   EQU    0x3FF

FASTOPTIONS EQU    B'00000001' ; Options: PORTA pull-ups enabled,
                               ;          TMR0 from CLKOUT (1MHz),
                               ;          TMR0 prescaler 1:4 (250KHz)

SLOWOPTIONS EQU    B'00000110' ; Options: PORTA pull-ups enabled,
                               ;          TMR0 from CLKOUT (1MHz),
                               ;          TMR0 prescaler 1:128 (7,812.5Hz)

INPORT      EQU    PORTA
PORTADIR    EQU    B'11111111' ; All port bits inputs
PORTAPU     EQU    B'11111101' ; Pull up on port bits except serial input

OUTPORT     EQU    PORTC
PORTCDIR    EQU    B'00000000' ; All port bits outputs

; Serial RX input port bit definition
#define  SERRXIN    INPORT,2

RS232BITS   EQU    8           ; RS232 8 data bits
; RS232 delay in instruction cycles for 9K6 baud at 1MHz clock
RXBITTIME   EQU    104         ; Delay count for 1 serial bit
RXSTARTTIME EQU    156         ; Delay count for 1.5 serial bits

SRVOFFST    EQU    6
SRVONST     EQU    (B'00000001000' | SRVOFFST)
SRVONSTBIT  EQU    3
SRVSTMASK   EQU    B'00000000111'

; Servo control bit definitions (active low)
#define  SRV1IN     inpVal,0
#define  SRV2IN     inpVal,1
#define  SRV3IN     inpVal,4
#define  SRV4IN     inpVal,5

; Servo control output port bit definitions (active high)
#define  SRV1OUT    OUTPORT,0
#define  SRV2OUT    OUTPORT,1
#define  SRV3OUT    OUTPORT,2
#define  SRV4OUT    OUTPORT,3

; Interrupt interval for servo pulse cycle start, 20.096mSec (7,812.5Hz / 157)
CYCLEINT    EQU    (255 - 157)

; Interrupt interval for start portion of servo pulse , 1mSec (250KHz / 250)
MINPULSEINT EQU    (255 - 250)

MINPOINT    EQU    255      ; Minimum pulse length, set servo to start point
MIDPOINT    EQU    127      ; Half pulse length, set servo to middle point
MAXPOINT    EQU    0        ; Maximum pulse length, set servo to end point
MINSPEED    EQU    1        ; Value for minimum speed
MIDSPEED    EQU    127      ; Value for middle speed
MAXSPEED    EQU    255      ; Value for maximum speed

RELOADCMND  EQU    '#'      ; Command character to reload settings from EEPROM
STORECMND   EQU    '@'      ; Command character to store settings to EEPROM
COMMANDBASE EQU    'A'      ; Command character for first setting value

; Number of servo settings to load/save from/to EEPROM
#define  NUMSETTINGS    (1 + (srv4OnRate - srv1Off))

#define  REMOTEIND    sysFlags,0
#define  STOREDIND    sysFlags,1
#define  LOADEDIND    sysFlags,2
#define  RXDATAIND    sysFlags,3


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

    CBLOCK  0x0020  ; General Purpose register data area

; Status and accumulator storage registers
w_isr                       ; W register (accumulator) store during ISR
pclath_isr                  ; PCLATH register store during ISR
status_isr                  ; STATUS register store during ISR

cycleState                  ; Interrupt routine pulse cycle state

sysFlags                    ; System status flags

inpVal                      ; Servo control inputs value

count                       ; General purpose counter
valueEE                     ; Store for EE write data

rxBitCount                  ; Serial RX bit counter
RS232i                      ; Serial input byte

command                     ; Serial input command byte

numHundreds                 ; First ASCII digit of number
numTens                     ; Second ASCII digit of number
numValue                    ; Binary version of number

; Used in generic servo update subroutine
servoSetting
servoRate

srv1Now                     ; Servo 1 target position
srv2Now                     ; Servo 2 target position
srv3Now                     ; Servo 3 target position
srv4Now                     ; Servo 4 target position

srv1State                   ; Servo 1 movement state
srv2State                   ; Servo 2 movement state
srv3State                   ; Servo 3 movement state
srv4State                   ; Servo 4 movement state

;Servo 1
srv1Off                     ; Off position
srv1Off1                    ; Off position first bounce
srv1Off2                    ; Off position second bounce
srv1Off3                    ; Off position third bounce
srv1On                      ; On position
srv1On1                     ; On position first bounce
srv1On2                     ; On position second bounce
srv1On3                     ; On position third bounce
srv1OffRate                 ; Off speed
srv1OnRate                  ; On speed

;Servo 2
srv2Off                     ; Off position
srv2Off1                    ; Off position first bounce
srv2Off2                    ; Off position second bounce
srv2Off3                    ; Off position third bounce
srv2On                      ; On position
srv2On1                     ; On position first bounce
srv2On2                     ; On position second bounce
srv2On3                     ; On position third bounce
srv2OffRate                 ; Off speed
srv2OnRate                  ; On speed

;Servo 3
srv3Off                     ; Off position
srv3Off1                    ; Off position first bounce
srv3Off2                    ; Off position second bounce
srv3Off3                    ; Off position third bounce
srv3On                      ; On position
srv3On1                     ; On position first bounce
srv3On2                     ; On position second bounce
srv3On3                     ; On position third bounce
srv3OffRate                 ; Off speed
srv3OnRate                  ; On speed

;Servo 4
srv4Off                     ; Off position
srv4Off1                    ; Off position first bounce
srv4Off2                    ; Off position second bounce
srv4Off3                    ; Off position third bounce
srv4On                      ; On position
srv4On1                     ; On position first bounce
srv4On2                     ; On position second bounce
srv4On3                     ; On position third bounce
srv4OffRate                 ; Off speed
srv4OnRate                  ; On speed

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

	ORG     0x2100  ; EEPROM data area

; Servo 1
    DE      MIDPOINT - 64   ; Off position
    DE      MIDPOINT - 32   ; Off position first bounce
    DE      MIDPOINT - 48   ; Off position second bounce
    DE      MIDPOINT - 56   ; Off position third bounce
    DE      MIDPOINT + 64   ; On position
    DE      MIDPOINT + 32   ; On position first bounce
    DE      MIDPOINT + 48   ; On position second bounce
    DE      MIDPOINT + 56   ; On position third bounce
    DE      MINSPEED
    DE      MINSPEED

; Servo 2
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; Off position first bounce
    DE      MIDPOINT        ; Off position second bounce
    DE      MIDPOINT        ; Off position third bounce
    DE      MIDPOINT        ; On position
    DE      MIDPOINT        ; On position first bounce
    DE      MIDPOINT        ; On position second bounce
    DE      MIDPOINT        ; On position third bounce
    DE      MAXSPEED
    DE      MAXSPEED

; Servo 3
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; Off position first bounce
    DE      MIDPOINT        ; Off position second bounce
    DE      MIDPOINT        ; Off position third bounce
    DE      MIDPOINT        ; On position
    DE      MIDPOINT        ; On position first bounce
    DE      MIDPOINT        ; On position second bounce
    DE      MIDPOINT        ; On position third bounce
    DE      MAXSPEED
    DE      MAXSPEED

; Servo 4
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; Off position first bounce
    DE      MIDPOINT        ; Off position second bounce
    DE      MIDPOINT        ; Off position third bounce
    DE      MIDPOINT        ; On position
    DE      MIDPOINT        ; On position first bounce
    DE      MIDPOINT        ; On position second bounce
    DE      MIDPOINT        ; On position third bounce
    DE      MAXSPEED
    DE      MAXSPEED


;**********************************************************************
; Delay loop macro                                                    *
;**********************************************************************
DelayLoop  macro    delayCounter, delayValue

           local    loopDelay

#if (6 < delayValue)
    movlw  ((delayValue - 1) / 3)
#else
    movlw  1
#endif
    movwf  delayCounter

loopDelay
    decfsz delayCounter,F
    goto   loopDelay

    endm


;**********************************************************************
; Add ASCII digit to a value macro                                    *
;**********************************************************************
AddAsciiDigitToValue macro    digitValue, placeValue, targetValue

    local   addLoop, addPlace, exitAdd

    movlw   '0'             ; Convert ASCII digit ...
    subwf   digitValue,F    ; ... to integer digit count

    incf    digitValue,F    ; Increment digit count to compensate for zero

    movlw   placeValue

addLoop
    decfsz  digitValue,F    ; Decrement digit count, skip if now zero ...
    goto    addPlace        ; ... otherwise add digit place value to target
    goto    exitAdd

addPlace
    addwf   targetValue,F   ; Add digit place value to target value
    goto    addLoop         ; Loop through adding placeValue

exitAdd

    endm


;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

    ORG     0x0000  ; Processor reset vector

bootVector
    clrf    INTCON          ; Disable interrupts
    clrf    INTCON          ; Ensure interrupts are disabled
    goto    initialise


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

    ORG     0x0004  ; Interrupt vector location

beginISR
    btfss   INTCON,T0IF     ; Skip if TMR0 interrupt flag is set ...
    retfie                  ; ... otherwise not a timer interrupt so return

    bcf     INTCON,T0IF     ; Clear the TMR0 interrupt bitflag

    movwf   w_isr           ; Save off current W register contents
    swapf   STATUS,W        ; Move status register into W register
    BANKSEL REGBANK0        ; Ensure register page 0 is selected
    movwf   status_isr      ; Save off contents of STATUS register
    movf    PCLATH,W        ; Move PCLATH register into W register
    movwf   pclath_isr      ; Save off contents of PCLATH register

    PAGESEL cycleStateTable ; Ensure PCLATH is set for jump code page
    movf    cycleState,W    ; Use cycle state value ...
    addwf   PCL,F           ; ... as index for code jump

cycleStateTable
    goto    endISR
    goto    cycleStart
    goto    srv1Start
    goto    srv1Run
    goto    srv2Start
    goto    srv2Run
    goto    srv3Start
    goto    srv3Run
    goto    srv4Start
    goto    srv4Run
    goto    cycleEnd

#if (high cycleStateTable) != (high $)
    error "Interrupt cycle state jump table spans 8 bit boundary"
#endif

startpulse
    ; Set duration for initial part of pulse
    ; Allow for instruction cycles within interrupt from pulse on to off
    movlw   (MINPULSEINT + 15)

runpulse
    movwf   TMR0            ; Load TMR0
    incf    cycleState,F    ; Advance to next state

endISR
    ; Exit from interrupt service routine
    movf    pclath_isr,W    ; Retrieve copy of PCLATH register
    movwf   PCLATH          ; Restore pre-isr PCLATH register contents
    swapf   status_isr,W    ; Retrieve copy of STATUS register
    movwf   STATUS          ; Restore pre-isr STATUS register contents
    swapf   w_isr,F
    swapf   w_isr,W         ; Restore pre-isr W register contents

    retfie                  ; Return from interrupt

cycleStart
    BANKSEL REGBANK1        ; Ensure register page 1 is selected
    movlw   FASTOPTIONS     ; Set fast interrupt clock
    movwf   OPTION_REG
    BANKSEL REGBANK0        ; Ensure register page 0 is selected
    incf    cycleState,F    ; Advance to next state

srv1Start
    bsf     SRV1OUT         ; Start servo 1 pulse
    goto    startpulse

srv1Run
    movf    srv1Now,W       ; Set duration for position part of pulse
    goto    runpulse

srv2Start
    bcf     SRV1OUT         ; Stop servo 1 pulse
    bsf     SRV2OUT         ; Start servo 2 pulse
    goto    startpulse

srv2Run
    movf    srv2Now,W       ; Set duration for position part of pulse
    goto    runpulse

srv3Start
    bcf     SRV2OUT         ; Stop servo 2 pulse
    bsf     SRV3OUT         ; Start servo 3 pulse
    goto    startpulse

srv3Run
    movf    srv3Now,W       ; Set duration for position part of pulse
    goto    runpulse

srv4Start
    bcf     SRV3OUT         ; Stop servo 3 pulse
    bsf     SRV4OUT         ; Start servo 4 pulse
    goto    startpulse

srv4Run
    movf    srv4Now,W       ; Set duration for position part of pulse
    goto    runpulse

cycleEnd
    bcf     SRV4OUT         ; Stop servo 4 pulse
    clrf    cycleState      ; End of cycle, next cycle state - idle
    goto    endISR


;**********************************************************************
; Servo setting position offset lookup subroutine                     *
;     Servo movement state passed in W                                *
;     Setting offset returned in W                                    *
;**********************************************************************
getServoSettingOffset
    PAGESEL settingOffsetTable ; Ensure PCLATH is set for jump code page
    andlw   0x0F
    addwf   PCL,F

settingOffsetTable
    retlw   (srv1Off  - srv1Off)
    retlw   (srv1Off3 - srv1Off)
    retlw   (srv1Off  - srv1Off)
    retlw   (srv1Off2 - srv1Off)
    retlw   (srv1Off  - srv1Off)
    retlw   (srv1Off1 - srv1Off)
    retlw   (srv1Off  - srv1Off)
    retlw   (srv1Off  - srv1Off)
    retlw   (srv1On   - srv1Off)
    retlw   (srv1On3  - srv1Off)
    retlw   (srv1On   - srv1Off)
    retlw   (srv1On2  - srv1Off)
    retlw   (srv1On   - srv1Off)
    retlw   (srv1On1  - srv1Off)
    retlw   (srv1On   - srv1Off)
    retlw   (srv1On   - srv1Off)

#if (high settingOffsetTable) != (high $)
    error "Servo setting offset lookup table spans 8 bit boundary"
#endif


;**********************************************************************
;    Main program loop                                                *
;**********************************************************************
main
    ; Wait until cycle state is idle, gap between servo pulse output
    movf    cycleState,F
    btfss   STATUS,Z
    goto    main

    bcf     INTCON,GIE      ; Disable interrupts
    bcf     INTCON,GIE      ; Ensure interrupts are disabled

    BANKSEL REGBANK1        ; Ensure register page 1 is selected
    movlw   SLOWOPTIONS     ; Set slow interrupt clock
    movwf   OPTION_REG
    BANKSEL REGBANK0        ; Ensure register page 0 is selected
    movlw   CYCLEINT        ; Set interrupt interval till start of next cycle
    movwf   TMR0            ; Initialise TMR0
    bsf     INTCON,T0IE     ; Enable TMR0 interrupts
    bcf     INTCON,T0IF     ; Clear any pending TMR0 interrupts

    incf    cycleState,F    ; Set cycle state to cycleStart

    movf    INPORT,W        ; Read physical input port
    btfss   REMOTEIND       ; Test if using remote input values ...
    movwf   inpVal          ; ... if not save input values from physical port

    call    updateAllServos ; Update servo target positions

    bsf     INTCON,GIE      ; Enable interrupts

testSerRx
    decf    cycleState,W    ; Test cycle state ...
    btfss   STATUS,Z        ; ... skip if still cycleStart ...
    goto    main            ; ... otherwise abort

    btfsc   SERRXIN         ; Test for serial input connected ...
    goto    testSerRx       ; ... else loop looking for serial input connected

syncSerRx
    decf    cycleState,W    ; Test cycle state ...
    btfss   STATUS,Z        ; ... skip if still cycleStart ...
    goto    main            ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    syncSerRx       ; ... else loop seeking possible serial start bit

    ; Synchronisation byte is a null, eight bits of high (RS232 'space')
    movlw   RS232BITS
    movwf   rxBitCount

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    count, (RXSTARTTIME - 4)

nextSerSyncBit
    btfss   SERRXIN         ; Test for sync byte bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    decfsz  rxBitCount,F    ; Decrement count of serial data bits ...
    goto    continueSerSync ; ... if not zero keep checking data bits ...
    goto    endSerSync      ; ... otherwise look for stop bit

continueSerSync
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    count, (RXBITTIME - 5)
    goto    nextSerSyncBit

endSerSync
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    count, (RXBITTIME - 6)

    btfsc   SERRXIN         ; Test for stop bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    ; Synchronised to null byte, receive command (1 byte) and value (3 bytes)

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    movf    RS232i,W        ; Save received byte ...
    movwf   command         ; ... as command

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    movf    RS232i,W        ; Save received byte ...
    movwf   numHundreds     ; ... as hundreds digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    movf    RS232i,W        ; Save received byte ...
    movwf   numTens         ; ... as tens digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    ; Convert individual digits to a single value

    movlw   '0'             ; Convert received ASCII digit ...
    subwf    RS232i,W       ; ... to integer units ...
    movwf   numValue        ; ... and initialise value

    AddAsciiDigitToValue    numTens,      10, numValue
    AddAsciiDigitToValue    numHundreds, 100, numValue

    ; Decode and action the command
    movlw   COMMANDBASE     ; Convert received command from ASCII ...
    subwf   command,W       ; ... to numerical value
    btfss   STATUS,C        ; Check command character not less than base ...
    goto    storeOrReload   ; ... else command is store or reload

    bcf     REMOTEIND       ; Clear remote settings received indicator

    PAGESEL commandTable    ; Ensure PCLATH is set for jump code page
    addwf   PCL,F           ; Use numerical command as index for code jump

commandTable
    goto    srv1SetOffPosition
    goto    srv1SetOnPosition
    goto    srv1SetOffRate
    goto    srv1SetOnRate
    goto    srv2SetOffPosition
    goto    srv2SetOnPosition
    goto    srv2SetOffRate
    goto    srv2SetOnRate
    goto    srv3SetOffPosition
    goto    srv3SetOnPosition
    goto    srv3SetOffRate
    goto    srv3SetOnRate
    goto    srv4SetOffPosition
    goto    srv4SetOnPosition
    goto    srv4SetOffRate
    goto    srv4SetOnRate
    goto    srv1SetOff1Position
    goto    srv1SetOff2Position
    goto    srv1SetOff3Position
    goto    srv1SetOn1Position
    goto    srv1SetOn2Position
    goto    srv1SetOn3Position
    goto    srv2SetOff1Position
    goto    srv2SetOff2Position
    goto    srv2SetOff3Position
    goto    srv2SetOn1Position
    goto    srv2SetOn2Position
    goto    srv2SetOn3Position
    goto    srv3SetOff1Position
    goto    srv3SetOff2Position
    goto    srv3SetOff3Position
    goto    srv3SetOn1Position
    goto    srv3SetOn2Position
    goto    srv3SetOn3Position
    goto    srv4SetOff1Position
    goto    srv4SetOff2Position
    goto    srv4SetOff3Position
    goto    srv4SetOn1Position
    goto    srv4SetOn2Position
    goto    srv4SetOn3Position

#if (high commandTable) != (high $)
    error "Received command jump table spans 8 bit boundary"
#endif

SetServoPositions  macro    srvNow, srvPos, srvPos1, srvPos2, srvPos3

    movf    numValue,W      ; Store received value ...
    movwf   srvPos          ; ... as servo settings
    movwf   srvPos1
    movwf   srvPos2
    movwf   srvPos3
    movwf   srvNow          ; ... and target position

    endm

srv1SetOffPosition
    SetServoPositions    srv1Now, srv1Off, srv1Off1, srv1Off2, srv1Off3
    bsf     SRV1IN          ; Remote input off, inactive (set bit)
    goto    receivedPosition

srv1SetOff1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1Off1        ; ... as servo settings
    goto    receivedRate

srv1SetOff2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1Off2        ; ... as servo settings
    goto    receivedRate

srv1SetOff3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1Off3        ; ... as servo settings
    goto    receivedRate

srv1SetOnPosition
    SetServoPositions    srv1Now, srv1On, srv1On1, srv1On2, srv1On3
    bcf     SRV1IN          ; Remote input on, active (clear bit)
    goto    receivedPosition

srv1SetOn1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1On1         ; ... as servo settings
    goto    receivedRate

srv1SetOn2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1On2         ; ... as servo settings
    goto    receivedRate

srv1SetOn3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv1On3         ; ... as servo settings
    goto    receivedRate

srv1SetOffRate
    movf    numValue,W      ; Store received value ...
    movwf   srv1OffRate     ; ... as servo settings
    goto    receivedRate

srv1SetOnRate
    movf    numValue,W      ; Store received value ...
    movwf   srv1OnRate      ; ... as servo settings
    goto    receivedRate

srv2SetOffPosition
    SetServoPositions    srv2Now, srv2Off, srv2Off1, srv2Off2, srv2Off3
    bsf     SRV2IN          ; Remote input off, inactive (set bit)
    goto    receivedPosition

srv2SetOff1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2Off1        ; ... as servo settings
    goto    receivedRate

srv2SetOff2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2Off2        ; ... as servo settings
    goto    receivedRate

srv2SetOff3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2Off3        ; ... as servo settings
    goto    receivedRate

srv2SetOnPosition
    SetServoPositions    srv2Now, srv2On, srv2On1, srv2On2, srv2On3
    bcf     SRV2IN          ; Remote input on, active (clear bit)
    goto    receivedPosition

srv2SetOn1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2On1         ; ... as servo settings
    goto    receivedRate

srv2SetOn2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2On2         ; ... as servo settings
    goto    receivedRate

srv2SetOn3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv2On3         ; ... as servo settings
    goto    receivedRate

srv2SetOffRate
    movf    numValue,W      ; Store received value ...
    movwf   srv2OffRate     ; ... as servo settings
    goto    receivedRate

srv2SetOnRate
    movf    numValue,W      ; Store received value ...
    movwf   srv2OnRate      ; ... as servo settings
    goto    receivedRate

srv3SetOffPosition
    SetServoPositions    srv3Now, srv3Off, srv3Off1, srv3Off2, srv3Off3
    bsf     SRV3IN          ; Remote input off, inactive (set bit)
    goto    receivedPosition

srv3SetOff1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3Off1        ; ... as servo settings
    goto    receivedRate

srv3SetOff2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3Off2        ; ... as servo settings
    goto    receivedRate

srv3SetOff3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3Off3        ; ... as servo settings
    goto    receivedRate

srv3SetOnPosition
    SetServoPositions    srv3Now, srv3On, srv3On1, srv3On2, srv3On3
    bcf     SRV3IN          ; Remote input on, active (clear bit)
    goto    receivedPosition

srv3SetOn1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3On1         ; ... as servo settings
    goto    receivedRate

srv3SetOn2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3On2         ; ... as servo settings
    goto    receivedRate

srv3SetOn3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv3On3         ; ... as servo settings
    goto    receivedRate

srv3SetOffRate
    movf    numValue,W      ; Store received value ...
    movwf   srv3OffRate     ; ... as servo settings
    goto    receivedRate

srv3SetOnRate
    movf    numValue,W      ; Store received value ...
    movwf   srv3OnRate      ; ... as servo settings
    goto    receivedRate

srv4SetOffPosition
    SetServoPositions    srv4Now, srv4Off, srv4Off1, srv4Off2, srv4Off3
    bsf     SRV4IN          ; Remote input off, inactive (set bit)
    goto    receivedPosition

srv4SetOff1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4Off1        ; ... as servo settings
    goto    receivedRate

srv4SetOff2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4Off2        ; ... as servo settings
    goto    receivedRate

srv4SetOff3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4Off3        ; ... as servo settings
    goto    receivedRate

srv4SetOnPosition
    SetServoPositions    srv4Now, srv4On, srv4On1, srv4On2, srv4On3
    bcf     SRV4IN          ; Remote input on, active (clear bit)
    goto    receivedPosition

srv4SetOn1Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4On1         ; ... as servo settings
    goto    receivedRate

srv4SetOn2Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4On2         ; ... as servo settings
    goto    receivedRate

srv4SetOn3Position
    movf    numValue,W      ; Store received value ...
    movwf   srv4On3         ; ... as servo settings
    goto    receivedRate

srv4SetOffRate
    movf    numValue,W      ; Store received value ...
    movwf   srv4OffRate     ; ... as servo settings
    goto    receivedRate

srv4SetOnRate
    movf    numValue,W      ; Store received value ...
    movwf   srv4OnRate      ; ... as servo settings
    goto    receivedRate

receivedPosition
    bsf     REMOTEIND       ; Set remote settings received indicator

receivedRate
    bcf     STOREDIND       ; Clear servo settings stored indicator
    bcf     LOADEDIND       ; Clear servo settings loaded indicator
    goto    syncSerRx       ; Loop looking for possible serial data

storeOrReload
    movf    command,W       ; Test if command ...
    xorlw   STORECMND       ; ... is to store settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    reloadSettings  ; ... otherwise test for reload command

    btfsc   STOREDIND       ; Test if settings have already been stored ...
    goto    main            ; ... if so abort

    clrf    count           ; Clear count of settings stored to EEPROM
    movlw   srv1Off         ; Load start address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

storeSetting
    movf    INDF,W          ; Get setting value ...
    movwf   valueEE         ; ... and save as EEPROM write value
    movf    count,W         ; Set count as index into EEPROM
    call    writeEEPROM

    incf    FSR,F           ; Increment to address of next setting
    incf    count,F         ; Increment count of settings stored to EEPROM

    ; Test if all variables have been stored to EEPROM
    movlw   NUMSETTINGS
    subwf   count,W
    btfss   STATUS,Z
    goto    storeSetting    ; Keep looping until all settings have been stored

    bcf     REMOTEIND       ; Clear remote settings received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator
    goto    syncSerRx

reloadSettings
    movf    command,W       ; Test if command ...
    xorlw   RELOADCMND      ; ... is to reload settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    syncSerRx       ; ... otherwise abort

    btfsc   LOADEDIND       ; Test if settings have already been loaded ...
    goto    syncSerRx       ; ... if so abort

    bcf     REMOTEIND       ; Clear remote settings received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator
    goto    loadAllSettings ; Reload all settings from EEPROM


;**********************************************************************
;    System initialisation                                            *
;**********************************************************************
initialise
    BANKSEL REGBANK1        ; Ensure register page 1 is selected

    ; Configure input port
    movlw   PORTADIR
    movwf   TRISA
    movlw   PORTAPU
    movwf   WPUA

    ; Configure output port
    movlw   PORTCDIR
    movwf   TRISC

    ; Set internal oscillator calibration
    call    GETOSCCAL
    movwf   OSCCAL

    BANKSEL REGBANK0        ; Ensure register page 0 is selected

    ; Turn comparator off
    movlw   B'00000111'
    movwf   CMCON

loadAllSettings
    clrf    count           ; Clear count of settings loaded from EEPROM
    movlw   srv1Off         ; Load start address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

loadSetting
    movf    count,W         ; Set count as index into EEPROM
    call    readEEPROM

    movwf   INDF            ; Load setting with value read from EEPROM

    incf    FSR,F           ; Increment to address of next setting
    incf    count,F         ; Increment count of settings loaded from EEPROM

    ; Test if all settings have been loaded from EEPROM
    movlw   NUMSETTINGS
    subwf   count,W
    btfss   STATUS,Z
    goto    loadSetting     ; Keep looping until all settings have been loaded

    ; Initialise servo target positions
    movf    INPORT,W        ; Read physical input port ...
    movwf   inpVal          ; ... and save input values from physical port

    ; Servo 1
    btfsc   SRV1IN          ; Skip if input bit clear, input on (active) ...
    movf    srv1Off,W       ; ... else set target as off position
    btfss   SRV1IN          ; Skip if input bit set, input off (inactive)
    movf    srv1On,W        ; ... else set target as on position
    movwf   srv1Now

    ; Servo 2
    btfsc   SRV2IN          ; Skip if input bit clear, input on (active) ...
    movf    srv2Off,W       ; ... else set target as off position
    btfss   SRV2IN          ; Skip if input bit set, input off (inactive)
    movf    srv2On,W        ; ... else set target as on position
    movwf   srv2Now

    ; Servo 3
    btfsc   SRV3IN          ; Skip if input bit clear, input on (active) ...
    movf    srv3Off,W       ; ... else set target as off position
    btfss   SRV3IN          ; Skip if input bit set, input off (inactive)
    movf    srv3On,W        ; ... else set target as on position
    movwf   srv3Now

    ; Servo 4
    btfsc   SRV4IN          ; Skip if input bit clear, input on (active) ...
    movf    srv4Off,W       ; ... else set target as off position
    btfss   SRV4IN          ; Skip if input bit set, input off (inactive)
    movf    srv4On,W        ; ... else set target as on position
    movwf   srv4Now

    ; Initalise servo movement states
    clrf    srv1State
    clrf    srv2State
    clrf    srv3State
    clrf    srv4State

    clrf    OUTPORT         ; Clear all outputs

    clrf    sysFlags        ; Clear: servo settings stored indicator,
                            ;        servo settings loaded indicator,
                            ;        remote settings received indicator,
                            ;        data byte received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator

    clrf    cycleState      ; Initialise cycle state to idle

    goto    main            ; Execute main program loop


;**********************************************************************
; RS232 input subroutine                                              *
;     Receives data byte into RS232i and sets RXDATAIND if successful *
;**********************************************************************
dataSerRx
    bcf     RXDATAIND       ; Clear data byte received indicator

    decf    cycleState,W    ; Test cycle state ...
    btfss   STATUS,Z        ; ... skip if still cycleStart ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    dataSerRx       ; ... else loop seeking possible serial start bit

    movlw   RS232BITS
    movwf   rxBitCount

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    count, (RXSTARTTIME - 5)

nextSerDataBit
    bcf     STATUS,C        ; Clear carry flag in status
    btfss   SERRXIN         ; Test RX bit on serial input ...
    bsf     STATUS,C        ; ... if not set then set carry flag in status

    rrf     RS232i,F        ; Rotate right carry flag into RS232 received byte

    decfsz  rxBitCount,F    ; Decrement count of serial data bits ...
    goto    continueSerData ; ... if not zero keep receiving data bits ...
    goto    endSerData      ; ... otherwise look for stop bit

continueSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    count, (RXBITTIME - 6)
    goto    nextSerDataBit

endSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    count, (RXBITTIME - 7)

    btfss   SERRXIN         ; Test for stop bit on serial input ...
    bsf     RXDATAIND       ; ... if found set data byte received indicator

    return


;**********************************************************************
; Write to EEPROM subroutine                                          *
;     Address in W, value in valueEE                                  *
;**********************************************************************
writeEEPROM
    BANKSEL EECON1            ; Ensure correct register page is selected
waitWriteEE
    btfsc   EECON1,WR         ; Skip EEPROM write not 'in progress' ...
    goto    waitWriteEE       ; ... else wait for write to complete

    movwf   EEADR             ; Set address of EEPROM location to write
    movf    valueEE,W
    movwf   EEDATA            ; Set EEPROM location value

    bsf     EECON1,WREN       ; Enable EEPROM writes
    bcf     INTCON,GIE        ; Disable interrupts
    bcf     INTCON,GIE        ; Ensure interrupts are disabled
    movlw   0x55
    movwf   EECON2
    movlw   0xAA
    movwf   EECON2
    bsf     EECON1,WR         ; Set EEPROM write status, ...
                              ; ... initiates hardware write cycle
    bcf     EECON1,EEIF       ; Clear EE write complete interrupt flag
    bcf     EECON1,WREN       ; Disable EEPROM writes

    bsf     INTCON,GIE        ; Enable interrupts
    BANKSEL 0                 ; Select register page 0
    return


;**********************************************************************
; Read from EEPROM subroutine                                         *
;     Address in W, value returned in W                               *
;**********************************************************************
readEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitReadEE
    btfsc   EECON1,WR       ; Skip EEPROM write not 'in progress' ...
    goto    waitReadEE      ; ... else wait for write to complete

    movwf   EEADR           ; Set address of EEPROM location to read
    bsf     EECON1,RD       ; Set EEPROM read status
    movf    EEDATA,W

    BANKSEL 0               ; Select register page 0
    return


;**********************************************************************
; Set servo on state macro                                            *
;**********************************************************************
ServoOnState  macro    servoState

    movlw   SRVONST
    btfss   servoState,SRVONSTBIT
    movwf   servoState

    endm


;**********************************************************************
; Set servo off state macro                                           *
;**********************************************************************
ServoOffState  macro    servoState

    movlw   SRVOFFST
    btfsc   servoState,SRVONSTBIT
    movwf   servoState

    endm


;**********************************************************************
; Servo target position update macro                                  *
;**********************************************************************
ServoUpdate  macro    srvState, srvSettings, srvRate, srvNow

    local   keepServoState

    movlw   srvSettings     ; Load servo settings base address
    movwf   FSR             ; ... into indirect addressing register

    movf    srvState,W      ; Get servo settings offset based on state
    call    getServoSettingOffset

    addwf   FSR,F           ; Add offset to servo settings base address
    movf    INDF,W          ; Get indexed setting ...
    movwf   servoSetting    ; ... as setting position

    movf    srvRate,W
    movwf   servoRate

    movlw   srvNow          ; Load servo target position address ...
    movwf   FSR             ; ... into indirect addressing register

    call    updateServo     ; Update servo target position

    btfss   STATUS,Z        ; Check if target and setting positions match ...
    goto    keepServoState  ; ... if so leave servo in current movement state

    ; Target and setting position match so advance movement state

    movlw   SRVSTMASK       ; Mask direction bit ...
    andwf   srvState,W      ; ... from servo movement state

    btfss   STATUS,Z        ; Test if movement complete ...
    decf    srvState,F      ; ... otherwise advance to next movement state

keepServoState

    endm


;**********************************************************************
; Servo target positions update subroutine                            *
;**********************************************************************
updateAllServos

    btfss   SRV1IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv1On    ; ... else perform servo on update

updateSrv1
    ServoOffState  srv1State
    ServoUpdate    srv1State, srv1Off, srv1OffRate, srv1Now
    goto    updateSrv2

updateSrv1On
    ServoOnState   srv1State
    ServoUpdate    srv1State, srv1Off, srv1OnRate, srv1Now

updateSrv2
    btfss   SRV2IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv2On    ; ... else perform servo on update

    ServoOffState  srv2State
    ServoUpdate    srv2State, srv2Off, srv2OffRate, srv2Now
    goto    updateSrv3

updateSrv2On
    ServoOnState   srv2State
    ServoUpdate    srv2State, srv2Off, srv2On, srv2Now

updateSrv3
    btfss   SRV3IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv3On    ; ... else perform servo on update

    ServoOffState  srv3State
    ServoUpdate    srv3State, srv3Off, srv3OffRate, srv3Now
    goto    updateSrv4

updateSrv3On
    ServoOnState   srv3State
    ServoUpdate    srv3State, srv3Off, srv3OnRate, srv3Now

updateSrv4
    btfss   SRV4IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv4On    ; ... else perform servo on update

    ServoOffState  srv4State
    ServoUpdate    srv4State, srv4Off, srv4OffRate, srv4Now
    return

updateSrv4On
    ServoOnState   srv4State
    ServoUpdate    srv4State, srv4Off, srv4OnRate, srv4Now
    return


;**********************************************************************
; Servo target position update subroutine, target accessed via FSR    *
;     Target position accessed via FSR                                *
;     STATUS,Z set when target and setting positions match            *
;**********************************************************************
updateServo
    movf    INDF,W          ; Test target position ...
    subwf   servoSetting,W  ; ... against position setting
    btfss   STATUS,C        ; Skip if target is less than position setting
    goto    decrementServo

    movf    servoRate,W     ; Add speed ...
    btfsc   STATUS,Z        ; ... replacing zero with ...
    movlw   MAXSPEED        ; ... maximum speed ...
    addwf   INDF,F          ; ... to target position

    btfsc   STATUS,C        ; Skip if no overflow ...
    goto    srvFullSetting  ; ... else limit to position setting

    movf    servoSetting,W  ; Subtract position setting ...
    subwf   INDF,W          ; ... from target position

    btfsc   STATUS,C        ; Skip if target less than position setting ...
    goto    srvFullSetting  ; ... else limit to position setting
    goto    servoUpdated

decrementServo
    movf    servoRate,W     ; Subtract off speed ...
    btfsc   STATUS,Z        ; ... replacing zero with ...
    movlw   MAXSPEED        ; ... maximum speed ...
    subwf   INDF,F          ; ... from target position

    btfss   STATUS,C        ; Skip if no overflow ...
    goto    srvFullSetting  ; ... else limit to position setting

    movf    INDF,W          ; Subtract target position ...
    subwf   servoSetting,W  ; ... from position setting

    btfss   STATUS,C        ; Skip if target less than position setting ...
    goto    servoUpdated

srvFullSetting
    movf    servoSetting,W  ; Set position setting ...
    movwf   INDF            ; ... as target position

servoUpdated
    movf    INDF,W          ; Compare target position ...
    subwf   servoSetting,W  ; ... against setting, returns STATUS,Z on match
    return



    end

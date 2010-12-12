    title    "$Id$"
    list     p=12F675
    radix    dec

;**********************************************************************
;                                                                     *
;    Description:   Microchip PIC assembler model railway semaphore   *
;                   servo controller.                                 *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (c)  2009 Monitor Computing Services Ltd               *
;    Unpublished and not for publication                              *
;    All rights reserved                                              *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: Semaphore signal servo control (with bounce) program      *
;           derived from 'sema4', itself derived from Mike Bolton's   *
;           'servo4' MERG (www.merg.org.uk) project.                  *
;                                                                     *
;           GP0 = AN0  - Analogue position feedback                   *
;           GP1 = Vref - A/D reference voltage                        *
;           GP2        - On/^Off input                                *
;           GP3        - Serial Rx                                    *
;                        (9K6, 1 start, 8 data, no parity, 1 stop)    *
;           GP5        - Servo pulse output (1 - 2 mSec)              *
;                                                                     *
;    6 May 2009:                                                      *
;       Added pause states to allow servo to complete movement.       *
;                                                                     *
;    26 Sep 2010:                                                     *
;       New Sema4b commands to set rates. Original command sets rate  *
;       x 16 (by nibble swap) to become properly Servo4 compatible.   *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

#include <P12F675.inc>

    __CONFIG   _CPD_OFF & _CP_ON & _BODEN_ON & _MCLRE_ON & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
; Selection is: Data Code Protection - off, Code Protection - on,
;               Brown Out Detection - on, GP3/!MCLR - !MCLR,
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

GPIODIR     EQU    B'00001111' ; In - GP0 GP1 GP2 GP3, Out - GP4 GP5
GPIOPU      EQU    B'00000100' ; Pull up on GP2

ADSELECT    EQU    B'00110001' ; GP0 = AN0, Conversion clock = Internal RC

ADTRCKTIME  EQU    20          ; A/D tracking time delay count(20 uSec @ 4 MHz)

; Serial RX input port bit definition
#define  SERRXIN   GPIO,GP3

RS232BITS   EQU    B'01111111' ; RS232 8 data bits (right shifted into carry)

; RS232 delay in uSec for 9K6 baud at 4MHz clock
RXBITTIME   EQU    104         ; Delay count for 1 serial bit
RXSTARTTIME EQU    156         ; Delay count for 1.5 serial bits

TIMEFREEZE  EQU    128         ; Number of cycles for setting mode timeout

SRVOFFST    EQU    31
SRVONST     EQU   (B'00100000' | SRVOFFST)
SRVONSTBIT  EQU    5
SRVSTMASK   EQU    B'00011111'
SRVSTFLTR   EQU   (B'00100000' | SRVSTMASK)

; Servo control input bit definition (active low)
#define  SERVOIN   inpVal,GP2

; Servo control output bit definition (active high)
#define  SERVOOUT  GPIO,GP5

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

RESETCMND   EQU    '#'      ; Command character to reset settings from EEPROM
RUNCMND     EQU    '$'      ; Command character to exit setting mode
STORECMND   EQU    '@'      ; Command character to store settings to EEPROM
COMMANDBASE EQU    'A'      ; Command character for first setting value

; Number of servo settings to load/save from/to EEPROM
#define  NUMSETTINGS    (1 + (servoOnRate - servoOff))

#define  STOREDIND    sysFlags,0
#define  LOADEDIND    sysFlags,1
#define  RXDATAIND    sysFlags,2

ASCIIBIT    EQU    7        ; Bit should be clear in any ASCII character
DIGITMASK   EQU    0xF0     ; Mask out lower nibble of ASCII digit character
DIGITTEST   EQU    '0'      ; Test pattern for ASCII digit after applying mask


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

freezeTime                  ; Ignore inputs for setting mode timeout

temp1                       ; General purpose temporary register
temp2                       ; General purpose temporary register
temp3                       ; General purpose temporary register

numHundreds                 ; First ASCII digit of number
numTens                     ; Second ASCII digit of number

servoNowH                   ; Servo 1 target position high byte
servoNowL                   ; Servo 1 target position low byte
servoState                  ; Servo 1 movement state
servoOff                    ; Off position
servoOff1                   ; Off position first bounce
servoOff2                   ; Off position second bounce
servoOff3                   ; Off position third bounce
servoOn                     ; On position
servoOn1                    ; On position first bounce
servoOn2                    ; On position second bounce
servoOn3                    ; On position third bounce
servoOffRate                ; Off speed
servoOnRate                 ; On speed

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

	ORG     0x2100  ; EEPROM data area

    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; Off position first bounce
    DE      MIDPOINT        ; Off position second bounce
    DE      MIDPOINT        ; Off position third bounce
    DE      MIDPOINT        ; On position
    DE      MIDPOINT        ; On position first bounce
    DE      MIDPOINT        ; On position second bounce
    DE      MIDPOINT        ; On position third bounce
    DE      MIDSPEED
    DE      MIDSPEED


;**********************************************************************
; PCLATH setup macro                                                  *
;**********************************************************************
SetPCLATH  macro    codeLabel

    movlw   high codeLabel
    movwf   PCLATH

    endm


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

    SetPCLATH cycleStateTable
    movf    cycleState,W    ; Use cycle state value ...
    addwf   PCL,F           ; ... as index for code jump

cycleStateTable
    goto    endISR
    goto    cycleStart
    goto    servoStart
    goto    servoRun
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

servoStart
    bsf     SERVOOUT        ; Start servo pulse
    goto    startpulse

servoRun
    movf    servoNowH,W      ; Set duration for position part of pulse
    goto    runpulse

cycleEnd
    bcf     SERVOOUT        ; Stop servo pulse
    clrf    cycleState      ; End of cycle, next cycle state - idle
    goto    endISR


;**********************************************************************
; Servo setting position offset lookup subroutine                     *
;     Servo movement state passed in temp3                            *
;     Setting offset returned in W                                    *
;**********************************************************************
getServoSettingOffset
    SetPCLATH settingOffsetTable

    movf    temp3,W
    andlw   SRVSTFLTR
    addwf   PCL,F

settingOffsetTable
    ; Off movement states
    retlw   (servoOff  - servoOff) ; State  0, actually do nothing
    retlw   (servoOff  - servoOff) ; State  1, actually do nothing
    retlw   (servoOff  - servoOff) ; State  2, actually do nothing
    retlw   (servoOff  - servoOff) ; State  3, actually do nothing
    retlw   (servoOff  - servoOff) ; State  4, pause at to off position
    retlw   (servoOff  - servoOff) ; State  5, pause at to off position
    retlw   (servoOff  - servoOff) ; State  6, pause at to off position
    retlw   (servoOff  - servoOff) ; State  7, move back to off position
    retlw   (servoOff3 - servoOff) ; State  8, pause at off 3rd bounce
    retlw   (servoOff3 - servoOff) ; State  9, pause at off 3rd bounce
    retlw   (servoOff3 - servoOff) ; State 10, pause at off 3rd bounce
    retlw   (servoOff3 - servoOff) ; State 11, move to off 3rd bounce
    retlw   (servoOff  - servoOff) ; State 12, pause at off position
    retlw   (servoOff  - servoOff) ; State 13, pause at off position
    retlw   (servoOff  - servoOff) ; State 14, pause at off position
    retlw   (servoOff  - servoOff) ; State 15, move back to off position
    retlw   (servoOff2 - servoOff) ; State 16, pause at off 2nd bounce
    retlw   (servoOff2 - servoOff) ; State 17, pause at off 2nd bounce
    retlw   (servoOff2 - servoOff) ; State 18, pause at off 2nd bounce
    retlw   (servoOff2 - servoOff) ; State 19, move to off 2nd bounce
    retlw   (servoOff  - servoOff) ; State 20, pause at off position
    retlw   (servoOff  - servoOff) ; State 21, pause at off position
    retlw   (servoOff  - servoOff) ; State 22, pause at off position
    retlw   (servoOff  - servoOff) ; State 23, move back to off position
    retlw   (servoOff1 - servoOff) ; State 24, pause at off 1st bounce
    retlw   (servoOff1 - servoOff) ; State 25, pause at off 1st bounce
    retlw   (servoOff1 - servoOff) ; State 26, pause at off 1st bounce
    retlw   (servoOff1 - servoOff) ; State 27, move to off 1st bounce
    retlw   (servoOff  - servoOff) ; State 28, pause at off position
    retlw   (servoOff  - servoOff) ; State 29, pause at off position
    retlw   (servoOff  - servoOff) ; State 30, pause at off position
    retlw   (servoOff  - servoOff) ; State 31, move to off position

    ; On movement states
    retlw   (servoOn   - servoOff) ; State 32, actually do nothing
    retlw   (servoOn   - servoOff) ; State 33, actually do nothing
    retlw   (servoOn   - servoOff) ; State 34, actually do nothing
    retlw   (servoOn   - servoOff) ; State 35, actually do nothing
    retlw   (servoOn   - servoOff) ; State 36, pause at on position
    retlw   (servoOn   - servoOff) ; State 37, pause at on position
    retlw   (servoOn   - servoOff) ; State 38, pause at on position
    retlw   (servoOn   - servoOff) ; State 39, move back to on position
    retlw   (servoOn3  - servoOff) ; State 40, pause at on 3rd bounce
    retlw   (servoOn3  - servoOff) ; State 41, pause at on 3rd bounce
    retlw   (servoOn3  - servoOff) ; State 42, pause at on 3rd bounce
    retlw   (servoOn3  - servoOff) ; State 43, move to on 3rd bounce
    retlw   (servoOn   - servoOff) ; State 44, pause at on position
    retlw   (servoOn   - servoOff) ; State 45, pause at on position
    retlw   (servoOn   - servoOff) ; State 46, pause at on position
    retlw   (servoOn   - servoOff) ; State 47, move back to on position
    retlw   (servoOn2  - servoOff) ; State 48, pause at on 2nd bounce
    retlw   (servoOn2  - servoOff) ; State 49, pause at on 2nd bounce
    retlw   (servoOn2  - servoOff) ; State 50, pause at on 2nd bounce
    retlw   (servoOn2  - servoOff) ; State 51, move to on 2nd bounce
    retlw   (servoOn   - servoOff) ; State 52, pause at on position
    retlw   (servoOn   - servoOff) ; State 53, pause at on position
    retlw   (servoOn   - servoOff) ; State 54, pause at on position
    retlw   (servoOn   - servoOff) ; State 55, move back to on position
    retlw   (servoOn1  - servoOff) ; State 56, pause at on 1st bounce
    retlw   (servoOn1  - servoOff) ; State 57, pause at on 1st bounce
    retlw   (servoOn1  - servoOff) ; State 58, pause at on 1st bounce
    retlw   (servoOn1  - servoOff) ; State 59, move to on 1st bounce
    retlw   (servoOn   - servoOff) ; State 60, pause at on position
    retlw   (servoOn   - servoOff) ; State 61, pause at on position
    retlw   (servoOn   - servoOff) ; State 62, pause at on position
    retlw   (servoOn   - servoOff) ; State 63, move to on position

#if (high settingOffsetTable) != (high $)
    error "Servo setting offset lookup table spans 8 bit boundary"
#endif


;**********************************************************************
; Command decode jump table                                           *
;     Recieved command passed in temp2                                *
;**********************************************************************
decodeCommand
    SetPCLATH commandTable

    movlw   COMMANDBASE     ; Convert received command from ASCII ...
    subwf   temp2,W         ; ... to numerical value
    btfss   STATUS,C        ; Check command character not less than base ...
    goto    receivedCommand ; ... else command is not position or speed setting

    addwf   PCL,F           ; Use numerical command as index for code jump

commandTable
    goto    servoSetOffPosition
    goto    servoSetOnPosition
    goto    servoSetOffRate
    goto    servoSetOnRate
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    servoSetOff1Position
    goto    servoSetOff2Position
    goto    servoSetOff3Position
    goto    servoSetOn1Position
    goto    servoSetOn2Position
    goto    servoSetOn3Position
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    servoSetOffOnly
    goto    servoSetOnOnly
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    servoNewOffRate
    goto    servoNewOnRate
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx
    goto    syncSerRx

#if (high commandTable) != (high $)
    error "Received command jump table spans 8 bit boundary"
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

    call    updateAllServos ; Update servo target positions

    movf    GPIO,W          ; Read physical input port

    movf    freezeTime,F    ; Test position setting mode timeout

    btfsc   STATUS,Z        ; Skip if timeout running, ignore inputs ...
    movwf   inpVal          ; ... otherwise save input values
                            ;     (may be overridden by serial link command)

    btfss   STATUS,Z        ; Skip if setting mode timeout not running ...
    decf    freezeTime,F    ; ... else decrement timeout

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
    movwf   temp3

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    temp1, (RXSTARTTIME - 4)

nextSerSyncBit
    btfss   SERRXIN         ; Test for sync byte bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry
    btfsc   STATUS,C        ; Check if got all serial data bits ...
    goto    continueSerSync ; ... if not zero keep checking data bits ...
    goto    endSerSync      ; ... otherwise look for stop bit

continueSerSync
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    temp1, (RXBITTIME - 5)
    goto    nextSerSyncBit

endSerSync
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    temp1, (RXBITTIME - 6)

    btfsc   SERRXIN         ; Test for stop bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    ; Synchronised to null byte, receive command (1 byte) and value (3 bytes)

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    btfsc   temp3,ASCIIBIT  ; Test received byte is an ASCII character ...
    goto    syncSerRx       ; ... otherwise abort

    movf    temp3,W         ; Save received byte ...
    movwf   temp2           ; ... as command

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    ; Check received byte is an ASCII digit
    movlw   DIGITMASK       ; Mask out lower nibble ...
    andwf   temp3,W         ; ... of recieved byte ...
    xorlw   DIGITTEST       ; ... and test for ASCII digit ...
    btfss   STATUS,Z        ; .. continue if ASCII digit received ...
    goto    syncSerRx       ; ... otherwise abort

    movf    temp3,W         ; Save received byte ...
    movwf   numHundreds     ; ... as hundreds digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    ; Check received byte is an ASCII digit
    movlw   DIGITMASK       ; Mask out lower nibble ...
    andwf   temp3,W         ; ... of recieved byte ...
    xorlw   DIGITTEST       ; ... and test for ASCII digit ...
    btfss   STATUS,Z        ; .. continue if ASCII digit received ...
    goto    syncSerRx       ; ... otherwise abort

    movf    temp3,W         ; Save received byte ...
    movwf   numTens         ; ... as tens digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    ; Check received byte is an ASCII digit
    movlw   DIGITMASK       ; Mask out lower nibble ...
    andwf   temp3,W         ; ... of recieved byte ...
    xorlw   DIGITTEST       ; ... and test for ASCII digit ...
    btfss   STATUS,Z        ; .. continue if ASCII digit received ...
    goto    syncSerRx       ; ... otherwise abort

    ; Convert individual digits to a single value

    movlw   '0'             ; Convert received ASCII digit ...
    subwf   temp3,F         ; ... to integer units

    AddAsciiDigitToValue    numTens,      10, temp3
    AddAsciiDigitToValue    numHundreds, 100, temp3

    ; Decode and action the command
    goto    decodeCommand

servoSetOffPosition
    movf    temp3,W         ; Store received value ...
    movwf   servoOff1       ; ... as ...
    movwf   servoOff2       ; ... servo ...
    movwf   servoOff3       ; ... settings

servoSetOffOnly
    movf    temp3,W         ; Store received value ...
    movwf   servoOff        ; ... as servo setting
    goto    received1OffPosition

servoSetOff1Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOff1       ; ... as servo setting
    goto    received1OffPosition

servoSetOff2Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOff2       ; ... as servo setting
    goto    received1OffPosition

servoSetOff3Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOff3       ; ... as servo setting

received1OffPosition
    movwf   servoNowH       ; Set target position as received setting value
    clrf    servoNowL
    bsf     SERVOIN         ; Servo input off, inactive (set bit)
    clrf    servoState      ; Set movement state as Off movement complete
    goto    receivedSetting

servoSetOnPosition
    movf    temp3,W         ; Store received value ...
    movwf   servoOn1        ; ... as ...
    movwf   servoOn2        ; ... servo ...
    movwf   servoOn3        ; ... settings

servoSetOnOnly
    movf    temp3,W         ; Store received value ...
    movwf   servoOn         ; ... as servo setting
    goto    received1OnPosition

servoSetOn1Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOn1        ; ... as servo setting
    goto    received1OnPosition

servoSetOn2Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOn2        ; ... as servo setting
    goto    received1OnPosition

servoSetOn3Position
    movf    temp3,W         ; Store received value ...
    movwf   servoOn3        ; ... as servo setting

received1OnPosition
    movwf   servoNowH       ; Set target position as received setting value
    clrf    servoNowL
    bcf     SERVOIN         ; Servo input on, active (clear bit)
    clrf    servoState      ; Set movement state as On movement complete
    bsf     servoState,SRVONSTBIT
    goto    receivedSetting

servoNewOffRate
    swapf   temp3,F         ; Negate effect of following nibble swap
servoSetOffRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   servoOffRate    ; ... as servo settings
    goto    receivedRate

servoNewOnRate
    swapf   temp3,F         ; Negate effect of following nibble swap
servoSetOnRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   servoOnRate     ; ... as servo settings
    goto    receivedRate

receivedRate
    bcf     STOREDIND       ; Clear servo settings stored indicator
    bcf     LOADEDIND       ; Clear servo settings loaded indicator
    clrf    freezeTime      ; Clear setting mode timeout (exit mode)
    goto    syncSerRx       ; Loop looking for possible serial data

receivedSetting
    bcf     STOREDIND       ; Clear servo settings stored indicator
    bcf     LOADEDIND       ; Clear servo settings loaded indicator

    movlw   TIMEFREEZE      ; Set setting mode timeout (enter mode)
    movwf   freezeTime

    goto    syncSerRx       ; Loop looking for possible serial data

receivedCommand
    movf    temp2,W         ; Test if command ...
    xorlw   RUNCMND         ; ... is to exit setting mode ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    receivedStore   ; ... otherwise test for reset command

    clrf    freezeTime      ; Clear setting mode timeout (exit mode)
    goto    syncSerRx

receivedStore
    movf    temp2,W         ; Test if command ...
    xorlw   STORECMND       ; ... is to store settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    receivedReset   ; ... otherwise test for reset command

    btfsc   STOREDIND       ; Test if settings have already been stored ...
    goto    syncSerRx

    clrf    temp1           ; Clear count of settings stored to EEPROM
    movlw   servoOff        ; Load start address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

storeSetting
    movf    INDF,W          ; Get setting value ...
    movwf   temp2           ; ... and save as EEPROM write value
    movf    temp1,W         ; Set count as index into EEPROM
    call    writeEEPROM

    incf    FSR,F           ; Increment to address of next setting
    incf    temp1,F         ; Increment count of settings stored to EEPROM

    ; Test if all variables have been stored to EEPROM
    movlw   NUMSETTINGS
    subwf   temp1,W
    btfss   STATUS,Z
    goto    storeSetting    ; Keep looping until all settings have been stored

    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator
    clrf    freezeTime      ; Clear setting mode timeout (exit mode)
    goto    syncSerRx

receivedReset
    movf    temp2,W         ; Test if command ...
    xorlw   RESETCMND       ; ... is to reset settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    syncSerRx       ; ... otherwise abort

    btfsc   LOADEDIND       ; Test if settings have already been loaded ...
    goto    syncSerRx       ; ... if so abort

    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator
    goto    loadAllSettings ; Reset all settings from EEPROM


;**********************************************************************
;    System initialisation                                            *
;**********************************************************************
initialise
    BANKSEL REGBANK1        ; Ensure register page 1 is selected

    ; Configure I/O port
    movlw   GPIODIR
    movwf   TRISIO
    movlw   GPIOPU
    movwf   WPU

    ; Configur A/D
    movlw   ADSELECT        ; Select A/D inputs and conversion clock
    movwf   ANSEL
    bcf     ADCON0,ADFM     ; Left justify conversion result
    bsf     ADCON0,VCFG     ; Voltage reference from Vref input
    bcf     ADCON0,CHS0     ; Select A/D ...
    bcf     ADCON0,CHS1     ; ... channel 0
    bsf     ADCON0,ADON     ; Enable A/D convertor

    ; Set internal oscillator calibration
    call    GETOSCCAL
    movwf   OSCCAL

    BANKSEL REGBANK0        ; Ensure register page 0 is selected

    ; Turn comparator off
    movlw   B'00000111'
    movwf   CMCON

loadAllSettings
    clrf    freezeTime      ; Expire setting mode timeout
    clrf    sysFlags        ; Clear: servo settings stored indicator,
                            ;        servo settings loaded indicator,
                            ;        data byte received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator

    clrf    temp1           ; Clear count of settings loaded from EEPROM
    movlw   servoOff        ; Load start address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

loadSetting
    movf    temp1,W         ; Set count as index into EEPROM
    call    readEEPROM

    movwf   INDF            ; Load setting with value read from EEPROM

    incf    FSR,F           ; Increment to address of next setting
    incf    temp1,F         ; Increment count of settings loaded from EEPROM

    ; Test if all settings have been loaded from EEPROM
    movlw   NUMSETTINGS
    subwf   temp1,W
    btfss   STATUS,Z
    goto    loadSetting     ; Keep looping until all settings have been loaded

    ; Initialise servo target positions
    movf    GPIO,W          ; Read physical input port ...
    movwf   inpVal          ; ... and save input values from physical port

    ; Servo 1
    btfsc   SERVOIN         ; Skip if input bit clear, input on (active) ...
    movf    servoOff,W      ; ... else set target as off position
    btfss   SERVOIN         ; Skip if input bit set, input off (inactive)
    movf    servoOn,W       ; ... else set target as on position
    movwf   servoNowH
    clrf    servoNowL

    ; Initalise servo movement states
    clrf    servoState

    clrf    GPIO            ; Clear all outputs

    clrf    cycleState      ; Initialise cycle state to idle


;**********************************************************************
; RS232 input subroutine                                              *
;     Receives data byte into temp3 and sets RXDATAIND if successful  *
;**********************************************************************
dataSerRx
    bcf     RXDATAIND       ; Clear data byte received indicator

    decf    cycleState,W    ; Test cycle state ...
    btfss   STATUS,Z        ; ... skip if still cycleStart ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    dataSerRx       ; ... else loop seeking possible serial start bit

    movlw   RS232BITS
    movwf   temp3

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXSTARTTIME - 5)

nextSerDataBit
    bcf     STATUS,C        ; Clear carry flag in status
    btfss   SERRXIN         ; Test RX bit on serial input ...
    bsf     STATUS,C        ; ... if not set then set carry flag in status

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry

    btfsc   STATUS,C        ; Check if got all serial data bits ...
    goto    continueSerData ; ... if not keep receiving data bits ...
    goto    endSerData      ; ... otherwise look for stop bit

continueSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXBITTIME - 6)
    goto    nextSerDataBit

endSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXBITTIME - 7)

    btfss   SERRXIN         ; Test for stop bit on serial input ...
    bsf     RXDATAIND       ; ... if found set data byte received indicator

    return


;**********************************************************************
; Write to EEPROM subroutine                                          *
;     Address in W, value in temp2                                    *
;**********************************************************************
writeEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitWriteEE
    btfsc   EECON1,WR       ; Skip EEPROM write not 'in progress' ...
    goto    waitWriteEE     ; ... else wait for write to complete

    movwf   EEADR           ; Set address of EEPROM location to write
    movf    temp2,W
    movwf   EEDATA          ; Set EEPROM location value

    bsf     EECON1,WREN     ; Enable EEPROM writes
    bcf     INTCON,GIE      ; Disable interrupts
    bcf     INTCON,GIE      ; Ensure interrupts are disabled
    movlw   0x55
    movwf   EECON2
    movlw   0xAA
    movwf   EECON2
    bsf     EECON1,WR       ; Set EEPROM write status, ...
                            ; ... initiates hardware write cycle
    bcf     EECON1,EEIF     ; Clear EE write complete interrupt flag
    bcf     EECON1,WREN     ; Disable EEPROM writes

    bsf     INTCON,GIE      ; Enable interrupts
    BANKSEL 0               ; Select register page 0
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
; Read A/D input subroutine                                           *
;     Conversion value returned in W                                  *
;**********************************************************************
readAD
    bsf     ADCON0,GO       ; Start an A/D conversion

    DelayLoop temp1,ADTRCKTIME

waitAD
    btfsc   ADCON0,GO       ; Test if A/D conversion ready ...
    goto    waitAD          ; ... else wait for A/D conversion

    movf    ADRESH,W        ; Get A/D conversion value
    return


;**********************************************************************
; Servo target position update macro                                  *
;**********************************************************************
ServoUpdate  macro    srvRate

    local   skipServoUpdate

    movlw   SRVSTMASK       ; Mask direction bit ...
    andwf   servoState,W    ; ... from servo movement state
    btfsc   STATUS,Z        ; Test if movement not yet complete ...
    goto    skipServoUpdate ; ... otherwise do nothing

    movlw   servoOff        ; Load servo settings base address
    movwf   FSR             ; ... into indirect addressing register

    movf    servoState,W    ; Get servo settings offset based on state
    movwf   temp3
    call    getServoSettingOffset

    addwf   FSR,F           ; Add offset to servo settings base address
    movf    INDF,W          ; Get indexed setting ...
    movwf   temp1           ; ... as setting position

    movf    srvRate,W
    movwf   temp2

    movlw   servoNowH       ; Load servo target position address ...
    movwf   FSR             ; ... into indirect addressing register

    call    updateServo     ; Update servo target position

    btfsc   STATUS,Z        ; Check if target and setting positions match ...
    decf    servoState,F    ; ... if so advance to next movement state

skipServoUpdate

    endm


;**********************************************************************
; Servo target positions update subroutine                            *
;**********************************************************************
updateAllServos

    btfss   SERVOIN         ; Skip if input bit set, input off (inactive) ...
    goto    updateServoOn   ; ... else perform servo on update

    movlw   SRVOFFST
    btfsc   servoState,SRVONSTBIT
    movwf   servoState

    ServoUpdate    servoOffRate
    return

updateServoOn
    movlw   SRVONST
    btfss   servoState,SRVONSTBIT
    movwf   servoState

    ServoUpdate    servoOnRate
    return


;**********************************************************************
; Servo target position update subroutine, target accessed via FSR    *
;     Target position accessed via FSR                                *
;     STATUS,Z set when target and setting positions match            *
;**********************************************************************
updateServo
    movf    temp2,F         ; Test speed ...
    btfsc   STATUS,Z        ; ... replacing zero with ...
    movlw   MAXSPEED        ; ... maximum speed ...

    movf    INDF,W          ; Test target position ...
    subwf   temp1,W         ; ... against position setting (result used later)

    ; Multiply 8 bit speed by 16 to give 16 bit value
    swapf   temp2,F         ; Swap nibbles, times 16 but mixed up
    movlw   0x0F            ; Isolate high byte ...
    andwf   temp2,W         ; ... nibble ...
    movwf   temp3           ; ... and save
    movlw   0xF0            ; Isolate low byte ...
    andwf   temp2,W         ; nibble

    btfss   STATUS,C        ; Skip if target is less than position setting
    goto    decrementServo

    ; Add speed to target position
    incf    FSR,F           ; Target position low byte

    addwf   INDF,F          ; Add speed low to target positon low

    decf    FSR,F           ; Target positon high byte

    btfsc   STATUS,C        ; Check no overflow from low byte addition ...
    incf    temp3,F         ; ... else adjust speed high byte

    movf    temp3,W         ; Get speed high byte

    addwf   INDF,F          ; Add speed high to target position high

    btfsc   STATUS,C        ; Skip if no overflow ...
    goto    srvFullSetting  ; ... else limit target to position setting

    movf    temp1,W         ; Subtract position setting ...
    subwf   INDF,W          ; ... from target position

    btfsc   STATUS,C        ; Skip if target less than position setting ...
    goto    srvFullSetting  ; ... else limit to position setting
    goto    servoUpdated

decrementServo
    ; Subtract speed from target position
    incf    FSR,F           ; Target position low byte

    subwf   INDF,F          ; Subtract speed low from target positon low

    decf    FSR,F           ; Target positon high byte

    btfss   STATUS,C        ; Check no overflow from low byte subtraction ...
    incf    temp3,F         ; ... else adjust speed high byte

    movf    temp3,W         ; Get speed high byte

    subwf   INDF,F          ; Subtract speed high from target position high

    btfss   STATUS,C        ; Skip if no overflow ...
    goto    srvFullSetting  ; ... else limit to position setting

    movf    INDF,W          ; Subtract target position ...
    subwf   temp1,W         ; ... from position setting

    btfss   STATUS,C        ; Skip if target less than position setting ...
    goto    servoUpdated

srvFullSetting
    movf    temp1,W         ; Set position setting ...
    movwf   INDF            ; ... as target position
    incf    FSR,F           ; Target position low byte
    clrf    INDF
    decf    FSR,F           ; Target positon high byte

servoUpdated
    movf    INDF,W          ; Compare target position ...
    subwf   temp1,W         ; ... against setting, returns STATUS,Z on match
    return



    end

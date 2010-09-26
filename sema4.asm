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
;   Issue: 1    Rev.: D                                               *
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
;   24 Jun 2008:                                                      *
;       Servo4 set position command sets corresponding bounces to     *
;       position. New Sema4 command to set only position.             *
;       Servo move immediately to position when setting bounce.       *
;       Physical on/off input not overridden when setting rates.      *
;       Replaced several explicit variables with temp1 and temp2 due  *
;       to running out of registers.                                  *
;                                                                     *
;   26 Jun 2008:                                                      *
;       Replaced RS232i and numValue with temp3.                      *
;                                                                     *
;   27 Jun 2008:                                                      *
;       Fixed incorrect variable being used as src2OnRate when        *
;       Updating position.                                            *
;       Target position made 16 bits, speed still 8 bits but          *
;       multiplied by 16 to update target position.                   *
;       Code protection turned on, just in case ...                   *
;                                                                     *
;   30 Jun 2008:                                                      *
;       Replaced PAGESEL with SetPCLATH.                              *
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

#include <p16f630.inc>

    __CONFIG   _CPD_OFF & _CP & _BODEN & _MCLRE_ON & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
; Selection is: Data Code Protection - off, Code Protection - on,
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
PORTAPU     EQU    B'11111011' ; Pull up on port bits except serial input

OUTPORT     EQU    PORTC
PORTCDIR    EQU    B'00000000' ; All port bits outputs

; Serial RX input port bit definition
#define  SERRXIN   INPORT,2

RS232BITS   EQU    B'01111111' ; RS232 8 data bits (right shifted into carry)
; RS232 delay in instruction cycles for 9K6 baud at 1MHz clock
RXBITTIME   EQU    104         ; Delay count for 1 serial bit
RXSTARTTIME EQU    156         ; Delay count for 1.5 serial bits

TIMEFREEZE  EQU    128         ; Number of cycles for setting mode timeout

SRVOFFST    EQU    31
SRVONST     EQU   (B'00100000' | SRVOFFST)
SRVONSTBIT  EQU    5
SRVSTMASK   EQU    B'00011111'
SRVSTFLTR   EQU   (B'00100000' | SRVSTMASK)

; Servo control bit definitions (active low)
#define  SRV1IN    inpVal,0
#define  SRV2IN    inpVal,1
#define  SRV3IN    inpVal,4
#define  SRV4IN    inpVal,5

; Servo control output port bit definitions (active high)
#define  SRV1OUT   OUTPORT,0
#define  SRV2OUT   OUTPORT,1
#define  SRV3OUT   OUTPORT,2
#define  SRV4OUT   OUTPORT,3

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
#define  NUMSETTINGS    (1 + (srv4OnRate - srv1Off))

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

srv1NowH                    ; Servo 1 target position high byte
srv1NowL                    ; Servo 1 target position low byte
srv2NowH                    ; Servo 2 target position high byte
srv2NowL                    ; Servo 2 target position low byte
srv3NowH                    ; Servo 3 target position high byte
srv3NowL                    ; Servo 3 target position low byte
srv4NowH                    ; Servo 4 target position high byte
srv4NowL                    ; Servo 4 target position low byte

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

; Servo 2
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

; Servo 3
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

; Servo 4
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
    movf    srv1NowH,W      ; Set duration for position part of pulse
    goto    runpulse

srv2Start
    bcf     SRV1OUT         ; Stop servo 1 pulse
    bsf     SRV2OUT         ; Start servo 2 pulse
    goto    startpulse

srv2Run
    movf    srv2NowH,W      ; Set duration for position part of pulse
    goto    runpulse

srv3Start
    bcf     SRV2OUT         ; Stop servo 2 pulse
    bsf     SRV3OUT         ; Start servo 3 pulse
    goto    startpulse

srv3Run
    movf    srv3NowH,W      ; Set duration for position part of pulse
    goto    runpulse

srv4Start
    bcf     SRV3OUT         ; Stop servo 3 pulse
    bsf     SRV4OUT         ; Start servo 4 pulse
    goto    startpulse

srv4Run
    movf    srv4NowH,W      ; Set duration for position part of pulse
    goto    runpulse

cycleEnd
    bcf     SRV4OUT         ; Stop servo 4 pulse
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
    retlw   (srv1Off  - srv1Off) ; State  0, actually do nothing
    retlw   (srv1Off  - srv1Off) ; State  1, actually do nothing
    retlw   (srv1Off  - srv1Off) ; State  2, actually do nothing
    retlw   (srv1Off  - srv1Off) ; State  3, actually do nothing
    retlw   (srv1Off  - srv1Off) ; State  4, pause at to off position
    retlw   (srv1Off  - srv1Off) ; State  5, pause at to off position
    retlw   (srv1Off  - srv1Off) ; State  6, pause at to off position
    retlw   (srv1Off  - srv1Off) ; State  7, move back to off position
    retlw   (srv1Off3 - srv1Off) ; State  8, pause at off 3rd bounce
    retlw   (srv1Off3 - srv1Off) ; State  9, pause at off 3rd bounce
    retlw   (srv1Off3 - srv1Off) ; State 10, pause at off 3rd bounce
    retlw   (srv1Off3 - srv1Off) ; State 11, move to off 3rd bounce
    retlw   (srv1Off  - srv1Off) ; State 12, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 13, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 14, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 15, move back to off position
    retlw   (srv1Off2 - srv1Off) ; State 16, pause at off 2nd bounce
    retlw   (srv1Off2 - srv1Off) ; State 17, pause at off 2nd bounce
    retlw   (srv1Off2 - srv1Off) ; State 18, pause at off 2nd bounce
    retlw   (srv1Off2 - srv1Off) ; State 19, move to off 2nd bounce
    retlw   (srv1Off  - srv1Off) ; State 20, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 21, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 22, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 23, move back to off position
    retlw   (srv1Off1 - srv1Off) ; State 24, pause at off 1st bounce
    retlw   (srv1Off1 - srv1Off) ; State 25, pause at off 1st bounce
    retlw   (srv1Off1 - srv1Off) ; State 26, pause at off 1st bounce
    retlw   (srv1Off1 - srv1Off) ; State 27, move to off 1st bounce
    retlw   (srv1Off  - srv1Off) ; State 28, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 29, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 30, pause at off position
    retlw   (srv1Off  - srv1Off) ; State 31, move to off position

    ; On movement states
    retlw   (srv1On   - srv1Off) ; State 32, actually do nothing
    retlw   (srv1On   - srv1Off) ; State 33, actually do nothing
    retlw   (srv1On   - srv1Off) ; State 34, actually do nothing
    retlw   (srv1On   - srv1Off) ; State 35, actually do nothing
    retlw   (srv1On   - srv1Off) ; State 36, pause at on position
    retlw   (srv1On   - srv1Off) ; State 37, pause at on position
    retlw   (srv1On   - srv1Off) ; State 38, pause at on position
    retlw   (srv1On   - srv1Off) ; State 39, move back to on position
    retlw   (srv1On3  - srv1Off) ; State 40, pause at on 3rd bounce
    retlw   (srv1On3  - srv1Off) ; State 41, pause at on 3rd bounce
    retlw   (srv1On3  - srv1Off) ; State 42, pause at on 3rd bounce
    retlw   (srv1On3  - srv1Off) ; State 43, move to on 3rd bounce
    retlw   (srv1On   - srv1Off) ; State 44, pause at on position
    retlw   (srv1On   - srv1Off) ; State 45, pause at on position
    retlw   (srv1On   - srv1Off) ; State 46, pause at on position
    retlw   (srv1On   - srv1Off) ; State 47, move back to on position
    retlw   (srv1On2  - srv1Off) ; State 48, pause at on 2nd bounce
    retlw   (srv1On2  - srv1Off) ; State 49, pause at on 2nd bounce
    retlw   (srv1On2  - srv1Off) ; State 50, pause at on 2nd bounce
    retlw   (srv1On2  - srv1Off) ; State 51, move to on 2nd bounce
    retlw   (srv1On   - srv1Off) ; State 52, pause at on position
    retlw   (srv1On   - srv1Off) ; State 53, pause at on position
    retlw   (srv1On   - srv1Off) ; State 54, pause at on position
    retlw   (srv1On   - srv1Off) ; State 55, move back to on position
    retlw   (srv1On1  - srv1Off) ; State 56, pause at on 1st bounce
    retlw   (srv1On1  - srv1Off) ; State 57, pause at on 1st bounce
    retlw   (srv1On1  - srv1Off) ; State 58, pause at on 1st bounce
    retlw   (srv1On1  - srv1Off) ; State 59, move to on 1st bounce
    retlw   (srv1On   - srv1Off) ; State 60, pause at on position
    retlw   (srv1On   - srv1Off) ; State 61, pause at on position
    retlw   (srv1On   - srv1Off) ; State 62, pause at on position
    retlw   (srv1On   - srv1Off) ; State 63, move to on position

#if (high settingOffsetTable) != (high $)
    error "Servo setting offset lookup table spans 8 bit boundary"
#endif


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
    clrf    freezeTime      ; Expire setting mode timeout
    clrf    sysFlags        ; Clear: servo settings stored indicator,
                            ;        servo settings loaded indicator,
                            ;        data byte received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator

    clrf    temp1           ; Clear count of settings loaded from EEPROM
    movlw   srv1Off         ; Load start address of servo settings ...
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
    movf    INPORT,W        ; Read physical input port ...
    movwf   inpVal          ; ... and save input values from physical port

    ; Servo 1
    btfsc   SRV1IN          ; Skip if input bit clear, input on (active) ...
    movf    srv1Off,W       ; ... else set target as off position
    btfss   SRV1IN          ; Skip if input bit set, input off (inactive)
    movf    srv1On,W        ; ... else set target as on position
    movwf   srv1NowH
    clrf    srv1NowL

    ; Servo 2
    btfsc   SRV2IN          ; Skip if input bit clear, input on (active) ...
    movf    srv2Off,W       ; ... else set target as off position
    btfss   SRV2IN          ; Skip if input bit set, input off (inactive)
    movf    srv2On,W        ; ... else set target as on position
    movwf   srv2NowH
    clrf    srv2NowL

    ; Servo 3
    btfsc   SRV3IN          ; Skip if input bit clear, input on (active) ...
    movf    srv3Off,W       ; ... else set target as off position
    btfss   SRV3IN          ; Skip if input bit set, input off (inactive)
    movf    srv3On,W        ; ... else set target as on position
    movwf   srv3NowH
    clrf    srv3NowL

    ; Servo 4
    btfsc   SRV4IN          ; Skip if input bit clear, input on (active) ...
    movf    srv4Off,W       ; ... else set target as off position
    btfss   SRV4IN          ; Skip if input bit set, input off (inactive)
    movf    srv4On,W        ; ... else set target as on position
    movwf   srv4NowH
    clrf    srv4NowL

    ; Initalise servo movement states
    clrf    srv1State
    clrf    srv2State
    clrf    srv3State
    clrf    srv4State

    clrf    OUTPORT         ; Clear all outputs

    clrf    cycleState      ; Initialise cycle state to idle


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

    movf    INPORT,W        ; Read physical input port

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
    SetPCLATH commandTable

    movlw   COMMANDBASE     ; Convert received command from ASCII ...
    subwf   temp2,W         ; ... to numerical value
    btfss   STATUS,C        ; Check command character not less than base ...
    goto    receivedCommand ; ... else command is not position or speed setting

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
    goto    srv1SetOffOnly
    goto    srv1SetOnOnly
    goto    srv2SetOffOnly
    goto    srv2SetOnOnly
    goto    srv3SetOffOnly
    goto    srv3SetOnOnly
    goto    srv4SetOffOnly
    goto    srv4SetOnOnly
    goto    srv1NewOffRate
    goto    srv1NewOnRate
    goto    srv2NewOffRate
    goto    srv2NewOnRate
    goto    srv3NewOffRate
    goto    srv3NewOnRate
    goto    srv4NewOffRate
    goto    srv4NewOnRate

#if (high commandTable) != (high $)
    error "Received command jump table spans 8 bit boundary"
#endif

srv1SetOffPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv1Off1        ; ... as ...
    movwf   srv1Off2        ; ... servo ...
    movwf   srv1Off3        ; ... settings

srv1SetOffOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv1Off         ; ... as servo setting
    goto    received1OffPosition

srv1SetOff1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1Off1        ; ... as servo setting
    goto    received1OffPosition

srv1SetOff2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1Off2        ; ... as servo setting
    goto    received1OffPosition

srv1SetOff3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1Off3        ; ... as servo setting

received1OffPosition
    movwf   srv1NowH        ; Set target position as received setting value
    clrf    srv1NowL
    bsf     SRV1IN          ; Servo input off, inactive (set bit)
    clrf    srv1State       ; Set movement state as Off movement complete
    goto    receivedSetting

srv1SetOnPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv1On1         ; ... as ...
    movwf   srv1On2         ; ... servo ...
    movwf   srv1On3         ; ... settings

srv1SetOnOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv1On          ; ... as servo setting
    goto    received1OnPosition

srv1SetOn1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1On1         ; ... as servo setting
    goto    received1OnPosition

srv1SetOn2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1On2         ; ... as servo setting
    goto    received1OnPosition

srv1SetOn3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv1On3         ; ... as servo setting

received1OnPosition
    movwf   srv1NowH        ; Set target position as received setting value
    clrf    srv1NowL
    bcf     SRV1IN          ; Servo input on, active (clear bit)
    clrf    srv1State       ; Set movement state as On movement complete
    bsf     srv1State,SRVONSTBIT
    goto    receivedSetting

srv1NewOffRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv1SetOffRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv1OffRate     ; ... as servo settings
    goto    receivedRate

srv1NewOnRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv1SetOnRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv1OnRate      ; ... as servo settings
    goto    receivedRate

srv2SetOffPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv2Off1        ; ... as ...
    movwf   srv2Off2        ; ... servo ...
    movwf   srv2Off3        ; ... settings

srv2SetOffOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv2Off         ; ... as servo setting
    goto    received2OffPosition

srv2SetOff1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2Off1        ; ... as servo setting
    goto    received2OffPosition

srv2SetOff2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2Off2        ; ... as servo setting
    goto    received2OffPosition

srv2SetOff3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2Off3        ; ... as servo settings

received2OffPosition
    movwf   srv2NowH        ; Set target position as received setting value
    clrf    srv2NowL
    bsf     SRV2IN          ; Servo input off, inactive (set bit)
    clrf    srv2State       ; Set movement state as Off movement complete
    goto    receivedSetting

srv2SetOnPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv2On1         ; ... as ...
    movwf   srv2On2         ; ... servo ...
    movwf   srv2On3         ; ... settings

srv2SetOnOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv2On          ; ... as servo setting
    goto    received2OnPosition

srv2SetOn1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2On1         ; ... as servo setting
    goto    received2OnPosition

srv2SetOn2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2On2         ; ... as servo setting
    goto    received2OnPosition

srv2SetOn3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv2On3         ; ... as servo setting

received2OnPosition
    movwf   srv2NowH        ; Set target position as received setting value
    clrf    srv2NowL
    bcf     SRV2IN          ; Servo input on, active (clear bit)
    clrf    srv2State       ; Set movement state as On movement complete
    bsf     srv2State,SRVONSTBIT
    goto    receivedSetting

srv2NewOffRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv2SetOffRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv2OffRate     ; ... as servo settings
    goto    receivedRate

srv2NewOnRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv2SetOnRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv2OnRate      ; ... as servo settings
    goto    receivedRate

srv3SetOffPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv3Off1        ; ... as ...
    movwf   srv3Off2        ; ... servo ...
    movwf   srv3Off3        ; ... settings

srv3SetOffOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv3Off         ; ... as servo setting
    goto    received3OffPosition

srv3SetOff1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3Off1        ; ... as servo setting
    goto    received3OffPosition

srv3SetOff2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3Off2        ; ... as servo setting
    goto    received3OffPosition

srv3SetOff3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3Off3        ; ... as servo setting

received3OffPosition
    movwf   srv3NowH        ; Set target position as received setting value
    clrf    srv3NowL
    bsf     SRV3IN          ; Servo input off, inactive (set bit)
    clrf    srv3State       ; Set movement state as Off movement complete
    goto    receivedSetting

srv3SetOnPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv3On1         ; ... as ...
    movwf   srv3On2         ; ... servo ...
    movwf   srv3On3         ; ... settings

srv3SetOnOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv3On          ; ... as servo setting
    goto    received3OnPosition

srv3SetOn1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3On1         ; ... as servo setting
    goto    received3OnPosition

srv3SetOn2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3On2         ; ... as servo setting
    goto    received3OnPosition

srv3SetOn3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv3On3         ; ... as servo setting

received3OnPosition
    movwf   srv3NowH        ; Set target position as received setting value
    clrf    srv3NowL
    bcf     SRV3IN          ; Servo input on, active (clear bit)
    clrf    srv3State       ; Set movement state as On movement complete
    bsf     srv3State,SRVONSTBIT
    goto    receivedSetting

srv3NewOffRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv3SetOffRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv3OffRate     ; ... as servo settings
    goto    receivedRate

srv3NewOnRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv3SetOnRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv3OnRate      ; ... as servo settings
    goto    receivedRate

srv4SetOffPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv4Off1        ; ... as ...
    movwf   srv4Off2        ; ... servo ...
    movwf   srv4Off3        ; ... settings

srv4SetOffOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv4Off         ; ... as servo setting
    goto    received4OffPosition

srv4SetOff1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4Off1        ; ... as servo setting
    goto    received4OffPosition

srv4SetOff2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4Off2        ; ... as servo setting
    goto    received4OffPosition

srv4SetOff3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4Off3        ; ... as servo setting

received4OffPosition
    movwf   srv4NowH        ; Set target position as received setting value
    clrf    srv4NowL
    bsf     SRV4IN          ; Servo input off, inactive (set bit)
    clrf    srv4State       ; Set movement state as Off movement complete
    goto    receivedSetting

srv4SetOnPosition
    movf    temp3,W         ; Store received value ...
    movwf   srv4On1         ; ... as ...
    movwf   srv4On2         ; ... servo ...
    movwf   srv4On3         ; ... settings

srv4SetOnOnly
    movf    temp3,W         ; Store received value ...
    movwf   srv4On          ; ... as servo setting
    goto    received4OnPosition

srv4SetOn1Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4On1         ; ... as servo setting
    goto    received4OnPosition

srv4SetOn2Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4On2         ; ... as servo setting
    goto    received4OnPosition

srv4SetOn3Position
    movf    temp3,W         ; Store received value ...
    movwf   srv4On3         ; ... as servo setting

received4OnPosition
    movwf   srv4NowH        ; Set target position as received setting value
    clrf    srv4NowL
    bcf     SRV4IN          ; Servo input on, active (clear bit)
    clrf    srv4State       ; Set movement state as On movement complete
    bsf     srv1State,SRVONSTBIT
    goto    receivedSetting

srv4NewOffRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv4SetOffRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv4OffRate     ; ... as servo settings
    goto    receivedRate

srv4NewOnRate
    swapf   temp3,F         ; Negate effect of following nibble swap
srv4SetOnRate
    swapf   temp3,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv4OnRate      ; ... as servo settings

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
    movlw   srv1Off         ; Load start address of servo settings ...
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

    local   skipServoUpdate

    movlw   SRVSTMASK       ; Mask direction bit ...
    andwf   srvState,W      ; ... from servo movement state
    btfsc   STATUS,Z        ; Test if movement not yet complete ...
    goto    skipServoUpdate ; ... otherwise do nothing

    movlw   srvSettings     ; Load servo settings base address
    movwf   FSR             ; ... into indirect addressing register

    movf    srvState,W      ; Get servo settings offset based on state
    movwf   temp3
    call    getServoSettingOffset

    addwf   FSR,F           ; Add offset to servo settings base address
    movf    INDF,W          ; Get indexed setting ...
    movwf   temp1           ; ... as setting position

    movf    srvRate,W
    movwf   temp2

    movlw   srvNow          ; Load servo target position address ...
    movwf   FSR             ; ... into indirect addressing register

    call    updateServo     ; Update servo target position

    btfsc   STATUS,Z        ; Check if target and setting positions match ...
    decf    srvState,F      ; ... if so advance to next movement state

skipServoUpdate

    endm


;**********************************************************************
; Servo target positions update subroutine                            *
;**********************************************************************
updateAllServos

    btfss   SRV1IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv1On    ; ... else perform servo on update

updateSrv1
    ServoOffState  srv1State
    ServoUpdate    srv1State, srv1Off, srv1OffRate, srv1NowH
    goto    updateSrv2

updateSrv1On
    ServoOnState   srv1State
    ServoUpdate    srv1State, srv1Off, srv1OnRate, srv1NowH

updateSrv2
    btfss   SRV2IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv2On    ; ... else perform servo on update

    ServoOffState  srv2State
    ServoUpdate    srv2State, srv2Off, srv2OffRate, srv2NowH
    goto    updateSrv3

updateSrv2On
    ServoOnState   srv2State
    ServoUpdate    srv2State, srv2Off, srv2OnRate, srv2NowH

updateSrv3
    btfss   SRV3IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv3On    ; ... else perform servo on update

    ServoOffState  srv3State
    ServoUpdate    srv3State, srv3Off, srv3OffRate, srv3NowH
    goto    updateSrv4

updateSrv3On
    ServoOnState   srv3State
    ServoUpdate    srv3State, srv3Off, srv3OnRate, srv3NowH

updateSrv4
    btfss   SRV4IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv4On    ; ... else perform servo on update

    ServoOffState  srv4State
    ServoUpdate    srv4State, srv4Off, srv4OffRate, srv4NowH
    return

updateSrv4On
    ServoOnState   srv4State
    ServoUpdate    srv4State, srv4Off, srv4OnRate, srv4NowH
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

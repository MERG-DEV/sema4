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
MAXSPEED    EQU    255      ; Value for maximum speed

RELOADCHAR  EQU    '#'      ; Command character to reload settings from EEPROM
STORECHAR   EQU    '@'      ; Command character to store settings to EEPROM
COMMANDBASE EQU    'A'      ; Command character for first setting value

RELOADCMND  EQU    low (RELOADCHAR - COMMANDBASE)
STORECMND   EQU    low (STORECHAR  - COMMANDBASE)

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
numUnits                    ; Third ASCII digit of number
numValue                    ; Binary version of number

servoSetting
servoRate

srv1Now                     ; Servo 1 target position
srv2Now                     ; Servo 2 target position
srv3Now                     ; Servo 3 target position
srv4Now                     ; Servo 4 target position

;Servo 1
srv1Off                     ; Off position
srv1On                      ; On position
srv1OffRate                 ; Off speed
srv1OnRate                  ; On speed

;Servo 2
srv2Off                     ; Off position
srv2On                      ; On position
srv2OffRate                 ; Off speed
srv2OnRate                  ; On speed

;Servo 3
srv3Off                     ; Off position
srv3On                      ; On position
srv3OffRate                 ; Off speed
srv3OnRate                  ; On speed

;Servo 4
srv4Off                     ; Off position
srv4On                      ; On position
srv4OffRate                 ; Off speed
srv4OnRate                  ; On speed

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

	ORG     0x2100  ; EEPROM data area

; Servo 1
    DE      MINPOINT        ; Off position
    DE      MAXPOINT        ; On position
    DE      MAXSPEED
    DE      MAXSPEED

; Servo 2
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; On position
    DE      MAXSPEED
    DE      MAXSPEED

; Servo 3
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; On position
    DE      MAXSPEED
    DE      MAXSPEED

; Servo 4
    DE      MIDPOINT        ; Off position
    DE      MIDPOINT        ; On position
    DE      MAXSPEED
    DE      MAXSPEED


;**********************************************************************
; Delay loop macro                                                    *
;**********************************************************************
DelayLoop  macro    delayCounter, delayValue

           local    loopDelay

 if (6 < delayValue)
    movlw  ((delayValue - 1) / 3)
 else
    movlw  1
 endif
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
    error "Interrupt cycle state jump table split across page boundary"
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
    ; Initialise all servo target positions to mid point
    movlw   MIDPOINT
    movwf   srv1Now
    movwf   srv2Now
    movwf   srv3Now
    movwf   srv4Now

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

    clrf    OUTPORT         ; Clear all outputs

    clrf    sysFlags        ; Clear: servo settings stored indicator,
                            ;        servo settings loaded indicator,
                            ;        remote settings received indicator,
                            ;        data byte received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator

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
    goto    main            ; ... otherwise abort
    movf    RS232i,W        ; Save received byte ...
    movwf   command         ; ... as command

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    main            ; ... otherwise abort
    movf    RS232i,W        ; Save received byte ...
    movwf   numHundreds     ; ... as hundreds digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    main            ; ... otherwise abort
    movf    RS232i,W        ; Save received byte ...
    movwf   numTens         ; ... as tens digit of value

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    main            ; ... otherwise abort
    movf    RS232i,W        ; Save received byte ...
    movwf   numUnits        ; ... as units digit of value

    ; Convert individual digits to a single value

    movlw   '0'             ; Convert ASCII digit ...
    subwf   numUnits,W      ; ... to integer units ...
    movwf   numValue        ; ... and initialise value

    AddAsciiDigitToValue    numTens,      10, numValue
    AddAsciiDigitToValue    numHundreds, 100, numValue

    ; Decode and action the command
    movlw   COMMANDBASE     ; Convert received command from ASCII ...
    subwf   command,F       ; ... to numerical value
    btfss   STATUS,C        ; Check command character not less than base ...
    goto    storeOrReload   ; ... else command is store or reload

    movlw   srv1Off         ; Start address of servo settings ...
    addwf   command,W       ; ... plus numerical command value ...
    movwf   FSR             ; ... as indirect address
    movf    numValue,W      ; Store received value ...
    movwf   INDF            ; ... as new servo setting

    bcf     REMOTEIND       ; Clear remote settings received indicator

    PAGESEL commandTable    ; Ensure PCLATH is set for jump code page
    movf    command,W       ; Use numerical command (servo setting offset) ...
    addwf   PCL,F           ; ... as index for code jump

commandTable
    goto    srv1RemoteOff
    goto    srv1RemoteOn
    goto    remoteRate
    goto    remoteRate
    goto    srv2RemoteOff
    goto    srv2RemoteOn
    goto    remoteRate
    goto    remoteRate
    goto    srv3RemoteOff
    goto    srv3RemoteOn
    goto    remoteRate
    goto    remoteRate
    goto    srv4RemoteOff
    goto    srv4RemoteOn
    goto    remoteRate
    goto    remoteRate

#if (high commandTable) != (high $)
    error "Command jump table split across page boundary"
#endif

srv1RemoteOff
    bsf     SRV1IN          ; Remote input off, inactive (set bit)
    goto    remoteDone

srv1RemoteOn
    bcf     SRV1IN          ; Remote input on, active (clear bit)
    goto    remoteDone

srv2RemoteOff
    bsf     SRV2IN          ; Remote input off, inactive (set bit)
    goto    remoteDone

srv2RemoteOn
    bcf     SRV2IN          ; Remote input on, active (clear bit)
    goto    remoteDone

srv3RemoteOff
    bsf     SRV3IN          ; Remote input off, inactive (set bit)
    goto    remoteDone

srv3RemoteOn
    bcf     SRV3IN          ; Remote input on, active (clear bit)
    goto    remoteDone

srv4RemoteOff
    bsf     SRV4IN          ; Remote input off, inactive (set bit)
    goto    remoteDone

srv4RemoteOn
    bcf     SRV4IN          ; Remote input on, active (clear bit)

remoteDone
    bsf     REMOTEIND       ; Set remote settings received indicator

remoteRate
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
    goto    main

reloadSettings
    movf    command,W       ; Test if command ...
    xorlw   RELOADCMND      ; ... is to reload settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    main            ; ... otherwise abort

    btfsc   LOADEDIND    ; Test if settings have already been loaded ...
    goto    main            ; ... if so abort

    bcf     REMOTEIND       ; Clear remote settings received indicator
    bsf     STOREDIND       ; Set servo settings stored indicator
    bsf     LOADEDIND       ; Set servo settings loaded indicator
    goto    loadAllSettings ; Reload all settings


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
; Servo target positions update subroutine                            *
;**********************************************************************
updateAllServos

    btfss   SRV1IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv1On    ; ... else perform servo on update

updateSrv1
    movf    srv1Off,W
    movwf   servoSetting
    movf    srv1OffRate,W
    movwf   servoRate
    movlw   srv1Now
    movwf   FSR
    call    updateServo
    goto    updateSrv2

updateSrv1On
    movf    srv1On,W
    movwf   servoSetting
    movf    srv1OnRate,W
    movwf   servoRate
    movlw   srv1Now
    movwf   FSR
    call    updateServo

updateSrv2
    btfss   SRV2IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv2On    ; ... else perform servo on update

    movf    srv2Off,W
    movwf   servoSetting
    movf    srv2OffRate,W
    movwf   servoRate
    movlw   srv2Now
    movwf   FSR
    call    updateServo
    goto    updateSrv3

updateSrv2On
    movf    srv2On,W
    movwf   servoSetting
    movf    srv2OnRate,W
    movwf   servoRate
    movlw   srv2Now
    movwf   FSR
    call    updateServo

updateSrv3
    btfss   SRV3IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv3On    ; ... else perform servo on update

    movf    srv3Off,W
    movwf   servoSetting
    movf    srv3OffRate,W
    movwf   servoRate
    movlw   srv3Now
    movwf   FSR
    call    updateServo
    goto    updateSrv4

updateSrv3On
    movf    srv3On,W
    movwf   servoSetting
    movf    srv3OnRate,W
    movwf   servoRate
    movlw   srv3Now
    movwf   FSR
    call    updateServo

updateSrv4
    btfss   SRV4IN          ; Skip if input bit set, input off (inactive) ...
    goto    updateSrv4On    ; ... else perform servo on update

    movf    srv3Off,W
    movwf   servoSetting
    movf    srv3OffRate,W
    movwf   servoRate
    movlw   srv3Now
    movwf   FSR
    goto    updateServo

updateSrv4On
    movf    srv4On,W
    movwf   servoSetting
    movf    srv4OnRate,W
    movwf   servoRate
    movlw   srv4Now
    movwf   FSR
    ; Simply fall through into 'updateServo' subroutine


;**********************************************************************
; Servo target position update subroutine, target accessed via FSR    *
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
    return

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
    return

srvFullSetting
    movf    servoSetting,W  ; Set position setting ...
    movwf   INDF            ; ... as target position
    return



    end

   title    "$Id$"
    list     p=16F630
    radix    dec

;**********************************************************************
;                                                                     *
;    Description:   Quad servo driver for model railway use.          *
;                                                                     *
;    Author:        Chris White M819 whitecf@bcs.org.uk               *
;                                                                     *
;**********************************************************************
;                                                                     *
;   Derived from orignal Servo4 by Mike Bolton                        *
;   Gives the pulse output for 4 conventional servos.                 *
;   Input control by on / off switches to ground (active low).        *
;   Port A has active pull-ups to Vdd.                                *
;   End limits, travel speed, and optional bounce positions           *
;   configured via a serial input (RS232 compatible).                 *
;   Setting values stored in EEPROM.                                  *
;   Option selectable by input bit to shutoff drive pulse shortly     *
;   after completion of movement sequence.                            *
;                                                                     *
;**********************************************************************
;                                                                     *
;   Processor speed 4MHz using internal clock                         *
;   WDT not used.                                                     *
;   Receive baud rate is 9600.                                        *
;   Command string is a null, a capital letter starting at 'A' and    *
;   a number from 0 to 255 as a three byte ascii.                     *
;                                                                     *
;   29/11/04 - Mike Bolton:                                           *
;       Working as far as two positions on each servo.                *
;       Saves settings OK. No mid position or speed control yet.      *
;                                                                     *
;   Servo3 is Servo2 but ready for speed and centre mods.             *
;   Working with revised scheme for current position.                 *
;   Speed setting now works. 0 is maximum, 1 is slowest.              *
;   Increase number for faster. 8 is about max.                       *
;   For mid posiition, just set with PC to 127                        *
;   Defaults are mid on all settings, speed = 0 (max).                *
;   Revised for running either way.                                   *
;   Pulse width set for 1 mSec min, 2 mSec max. Setting 127 is mid    *
;   way.                                                              *
;   Tested and working 3/12/04                                        *
;                                                                     *
;   4/12/04 - Mike Bolton:                                            *
;   Now Servo4. Added reset without save.                             *
;   Allows position to be set via the PC independent of switch        *
;   position.                                                         *
;   Working OK.                                                       *
;                                                                     *
;   12/04/05 - Mike Bolton:                                           *
;       Rev B. Contains the code to set the oscillator to calibrated  *
;       value.                                                        *
;                                                                     *
;   22 Jun 2008 - Chris White:                                        *
;       Renamed, restructured, and refactored to decipher             *
;       functionality.                                                *
;                                                                     *
;   23 Jun 2008 - Chris White:                                        *
;       Added 3 'bounce' positions in either direction.               *
;                                                                     *
;   24 Jun 2008 - Chris White:                                        *
;       Servo4 set position command sets corresponding bounces to     *
;       position. New Sema4 command to set only position.             *
;       Servo move immediately to position when setting bounce.       *
;       Physical on/off input not overridden when setting rates.      *
;       Replaced several explicit variables with temp1 and temp2 due  *
;       to running out of registers.                                  *
;                                                                     *
;   26 Jun 2008 - Chris White:                                        *
;       Replaced RS232i and numValue with temp3.                      *
;                                                                     *
;   27 Jun 2008 - Chris White:                                        *
;       Fixed incorrect variable being used as src2OnRate when        *
;       Updating position.                                            *
;       Current position made 16 bits, speed still 8 bits but         *
;       multiplied by 16 to update current position.                  *
;       Code protection turned on, just in case ...                   *
;                                                                     *
;   30 Jun 2008 - Chris White:                                        *
;       Replaced PAGESEL with SetPCLATH.                              *
;                                                                     *
;    6 May 2009 - Chris White:                                        *
;       Added pause states to allow servo to complete movement.       *
;                                                                     *
;    26 Sep 2010 - Chris White:                                       *
;       New Sema4b commands to set rates. Original command sets rate  *
;       x 16 (by nibble swap) to become properly Servo4 compatible.   *
;                                                                     *
;     4 May 2011 - Chris White:                                       *
;       Incorporated Mike Bolton's Servo4 drive shutoff option.       *
;                                                                     *
;     5 May 2011 - Chris White:                                       *
;       Pulse interval timed by timer0.                               *
;       Pulse duration timed by timer1.                               *
;                                                                     *
;     5 May 2011 - Chris White:                                       *
;       Pulse duration by timer1 at 1MHz, finer movement increments.  *
;                                                                     *
;     5 May 2011 - Chris White:                                       *
;       Optional 0.5mSec to 2.5mSec pulse for extended travel servos. *
;                                                                     *
;     6 May 2011 - Chris White:                                       *
;       Modified serial Rx to check for pulse cycle after each bit    *
;       read and abort if Rx broken by a cycle timing interrupt.      *
;       Reading of servo control inputs moved into a subroutine.      *
;       Code protection set in CONFIG.                                *
;       Added output indicating completion of movement for servo1,    *
;       repeats state of input but only once movement completed.      *
;                                                                     *
;    10 May 2011 - Chris White:                                       *
;       Modified output cycle to skip completely the pulse for servo  *
;       if its output is disabled (drive shut off).                   *
;       Set rates initalised in EEPROM to 16 as MIDRATE was a bit too *
;       fast.                                                         *
;       Reduced input skip timeout after receiving a position setting *
;       to about one second.                                          *
;                                                                     *
;    10 Mar 2012 - Chris White:                                       *
;       Removed servo1 indicator. Ouput is now used to echo Rx data.  *
;                                                                     *
;    29 Apr 2012 - Chris White:                                       *
;       Modified speed setting to be compatible with Servo4g          *
;       (0 - fastest, 255 - slowest). Added banner message output on  *
;       startup. Added reboot command, "!".                           *
;                                                                     *
;    16 Oct 2012 - Chris White:                                       *
;       Servo drive out puts written through cache rather than        *
;       directly to port in order to avoid latching noise on pin as   *
;       output value. Sync to null now uses standard serial data Rx.  *
;                                                                     *
;    11 Aug 2013 - Chris White:                                       *
;       Fixed bug, reading switch inputs at initialisation before     *
;       weak pullups enabled.                                         *
;       Drive shutoff selection input read once at initialisation     *
;       then stored in sysFlags.                                      *
;       Serial TX also now written through output cache.              *
;       Adjusted Rx timing delays.                                    *
;       Tweaked out a few instruction bytes here and there.           *
;       Drive shutdown leaves control outputs high.                   *
;       Clear all general purpose registers on initialisation.        *
;                                                                     *
;    12 Aug 2013 - Chris White:                                       *
;       Switch inputs stored to EEProm on change. On startup          *
;       initialise positions to match end of last movement based on   *
;       saved inputs then if changed begin controlled movement. If    *
;       unchanged remain at current position.                         *
;                                                                     *
;    14 Aug 2013 - Chris White:                                       *
;       Cycle time increased to 21 mSec to ensure time for at least   *
;       two sequences of five bytes between servo pulses.             *
;                                                                     *
;    17 Aug 2013 - Chris White:                                       *
;       Defined macros to reduce duplication of code common to all    *
;       servos.                                                       *
;       Defined subroutine to sync to Rx of null to improve sync to   *
;       start of a command sequence.                                  *
;       Inverted sense of drive enable flags to be drive disable.     *
;                                                                     *
;    18 Aug 2013 - Chris White:                                       *
;       Defined more macros to reduce duplication of code common to   *
;       all servos, this time for handling received commands.         *
;                                                                     *
;    26 Aug 2013 - Chris White:                                       *
;       Reduced movement states from 32 to 8 as if bounce was used as *
;       an overshoot pause was noticeable between reaching end and    *
;       starting "bounce" (overshoot).                                *
;                                                                     *
;    27 Jul 2014 - Chris White:                                       *
;       Made change of extended travel flags mark EEPROM as not       *
;       synchronised same as for any setting value in order to ensure *
;       these are written to EEPROM on receipt of "store" command.    *
;       Corrected preservation of extended travel flags when changed  *
;       control input bits written to EEPROM.                         *
;       Added discrete commands to enable and disable extended travel *
;       per servo, as opposed to setting all in a single command.     *
;                                                                     *
;**********************************************************************
;                                                                     *
;                             +---+ +---+                             *
;                   +5V -> VDD|1  |_| 14|VSS <- 0V                    *
;             Servo4 In -> RA5|2      13|RA0 <- Servo1 In             *
;             Servo3 In -> RA4|3      12|RA1 <- Servo2 In             *
;                        !MCLR|4      11|RA2 <- Serial Rx             *
;         Drive Shutoff -> RC5|5      10|RC0 -> Servo1 Out            *
;             Serial Tx <- RC4|6       9|RC1 -> Servo2 Out            *
;            Servo4 Out <- RC3|7       8|RC2 -> Servo3 Out            *
;                             +---------+                             *
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

; Address constants
;**********************************************************************

REGBANK0    EQU    0x00        ; Value used to select register bank 0
REGBANK1    EQU    0x80        ; Value used to select register bank 1

GETOSCCAL   EQU    0x3FF       ; Address of oscillator calibration value

RAMSTART    EQU    0x0020      ; General Purpose register data area
RAMEND      EQU    RAMSTART + 0x3F

#define ENDSETTINGS srv4On3

EESTART     EQU    0x2100      ; EEPROM data area

BOOTVECTOR  EQU    0           ; Processor reset vector

INTVECTOR   EQU    0x0004      ; Interrupt vector location

; I/O port and bit definitions
;**********************************************************************

INPORT      EQU    PORTA
PORTADIR    EQU    B'11111111' ; All bits inputs
PORTAPU     EQU    B'11111011' ; Pull up on port bits except serial input

OUTPORT     EQU    PORTC
PORTCDIR    EQU    B'00100000' ; All bits outputs except 5 (Drive Shutoff)

; Servo control input port bit definitions (active low)
#define  SRV1INP   INPORT,0
#define  SRV2INP   INPORT,1
#define  SRV3INP   INPORT,4
#define  SRV4INP   INPORT,5

; Servo control bit definitions (active high), srvCtrl
#define  SRV1ON    srvCtrl,0
#define  SRV2ON    srvCtrl,1
#define  SRV3ON    srvCtrl,4
#define  SRV4ON    srvCtrl,5

#define INPMASK    B'00110011' ; Mask to isolate servo control input bits

; Drive shutoff option input bit definition (active high)
#define  DRVOFFINP PORTC,5

; Drive disabled bit definitions (active high), held in "sysFlags"
#define  SRV1DIS   0
#define  SRV2DIS   1
#define  SRV3DIS   2
#define  SRV4DIS   3

; Servo control output port bit definitions (active high)
#define  SRV1OUT   portCval,0
#define  SRV2OUT   portCval,1
#define  SRV3OUT   portCval,2
#define  SRV4OUT   portCval,3

#define  OUTMASK   B'00001111' ; Mask to isolate drive output bits

; Servo extended travel selected bit definitions, srvCtrl
#define  SRV1XTND  srvCtrl,2
#define  SRV2XTND  srvCtrl,3
#define  SRV3XTND  srvCtrl,6
#define  SRV4XTND  srvCtrl,7

#define  XTNDMASK  B'11001100' ; Mask to isolate extended travel selected bits

; Serial RX IO port bit definitions
#define  SERRXIN   PORTA,2
#define  SERTXOUT  portCval,4

RS232BITS   EQU    B'01111111' ; RS232 8 data bits (right shifted into carry)

; Timing constants
;**********************************************************************

; RS232 delay in instruction cycles for 9K6 baud at 1MHz clock
SRLBITTIME  EQU    104         ; Delay count for 1 serial bit
RXSTARTTIME EQU     52         ; Delay count for half a serial bit

TIMEFREEZE  EQU    100         ; Number of cycles for setting mode timeout
TIMEDRIVE   EQU    30          ; Number of cycles till drive shutoff
                               ; N.B. Must be less than 32 as timer is also
                               ; low byte of current position of which upper
                               ; three bits are used for pulse duration timing.

; Interrupt interval for servo pulse cycle start, 21.121mSec (165 @ 7,812.5Hz)
CYCLEINT    EQU    (0xFF - 165)

; Interrupt interval for start portion of servo pulse , 1mSec (250 @ 250KHz)
; Value will be scaled at use as clock is actually 1MHz rather than 250KHz
PULSEINT    EQU    250

; Options register values to select fast or slow interrupt timing
;**********************************************************************

TMR1OPTIONS EQU    B'00000000' ; Options: TMR1 Gate disabled,
                               ;          TMR1 prescale, 1:1 (1MHz)
                               ;          TMR1 from CLKOUT (1MHz),

TMR0OPTIONS EQU    B'00000110' ; Options: PORTA pull-ups enabled,
                               ;          TMR0 from CLKOUT (1MHz),
                               ;          TMR0 prescaler, 1:128 (7,812.5Hz)

; Servo movement sequence state constants
;**********************************************************************

SRVOFFST    EQU    7           ; Initial state in "off" movement sequence
SRVOFFEND   EQU    1           ; Initial state in "off" movement drive shutoff
SRVONST     EQU    15          ; Initial state in "on" movement sequence
SRVONEND    EQU    9           ; Initial state in "on" movement drive shutoff
SRVMVMASK   EQU    B'00000111' ; Mask to test if movement sequence completed
SRVONSTBIT  EQU    3           ; Bit indicates "on" or "off" movement sequence

; Servo settings limit values
;**********************************************************************

MINPOINT    EQU    0        ; Minimum pulse length, set servo to start point
MIDPOINT    EQU    125      ; Half pulse length, set servo to middle point
MAXPOINT    EQU    250      ; Maximum pulse length, set servo to end point
MINRATE     EQU    1        ; Value for minimum rate
MIDRATE     EQU    MIDPOINT ; Value for middle rate
MAXRATE     EQU    MAXPOINT ; Value for maximum rate

; System status flags
;**********************************************************************

#define  SYNCEDIND    sysFlags,7    ; Settings and EEPROM in synch indicator
#define  RXDATAIND    sysFlags,6    ; New Rx data received indicator
#define  RUNMAIN      sysFlags,5    ; Main program loop enabled indicator
#define  DRVOFF       sysFlags,4    ; Drive shutoff option indicator

; Received data test masks
;**********************************************************************

ASCIIBIT    EQU    7        ; Bit should be clear in any ASCII character
DIGITMASK   EQU    0xF0     ; Mask out lower nibble of ASCII digit character
DIGITTEST   EQU    '0'      ; Test pattern for ASCII digit after applying mask

; Command characters
;**********************************************************************

RESETCMND   EQU    '#'      ; Reset settings from EEPROM
RUNCMND     EQU    '$'      ; Exit setting mode
REBOOTCMND  EQU    '!'
STORECMND   EQU    '@'      ; Store current settings to EEPROM
SETTINGBASE EQU    'A'      ; Base character for first setting value

LINEFEED    EQU     0x0A
CRG_RTRN    EQU     0x0D


;**********************************************************************
; Variable definitions                                                *
;**********************************************************************

    CBLOCK  RAMSTART        ; General Purpose register data area

; ISR status and accumulator storage registers
;**********************************************************************

w_isr                       ; W register (accumulator) store during ISR
pclath_isr                  ; PCLATH register store during ISR
status_isr                  ; STATUS register store during ISR

; Temporary scratchpad registers
;**********************************************************************

temp1                       ; General purpose temporary register
temp2                       ; General purpose temporary register
temp3                       ; General purpose temporary register
temp4                       ; General purpose temporary register

; System status and control registers
;**********************************************************************

freezeTime                  ; Setting mode timeout to ignore servo inputs

cycleState                  ; Interrupt routine pulse cycle state

sysFlags                    ; System status flags (all active high)
                            ;  bit 7 - Settings and EEPROM synched indicator
                            ;  bit 6 - New Rx data received indicator
                            ;  bit 5 - Main program loop enabled indicator
                            ;  bit 4 - Drive shutoff selected indicator
                            ;  bit 3 - servo 4 drive enabled
                            ;  bit 2 - servo 3 drive enabled
                            ;  bit 1 - servo 2 drive enabled
                            ;  bit 0 - servo 1 drive enabled

srvCtrl                     ; Servo control flags (all active high)
                            ;  bit 7 - Servo 4 extended travel selected
                            ;  bit 6 - Servo 3 extended travel selected
                            ;  bit 5 - servo 4 control input (inverse of port)
                            ;  bit 4 - servo 3 control input (inverse of port)
                            ;  bit 3 - Servo 2 extended travel selected
                            ;  bit 2 - Servo 1 extended travel selected
                            ;  bit 1 - servo 2 control input (inverse of port)
                            ;  bit 0 - servo 1 control input (inverse of port)

; Servo 1 position and rate settings
;**********************************************************************

srv1OffRate                 ; Off rate
srv1Off                     ; Off position
srv1Off1                    ; Off position first bounce
srv1Off2                    ; Off position second bounce
srv1Off3                    ; Off position third bounce
srv1OnRate                  ; On rate
srv1On                      ; On position
srv1On1                     ; On position first bounce
srv1On2                     ; On position second bounce
srv1On3                     ; On position third bounce

; Servo 2 position and rate settings
;**********************************************************************

srv2OffRate                 ; Off rate
srv2Off                     ; Off position
srv2Off1                    ; Off position first bounce
srv2Off2                    ; Off position second bounce
srv2Off3                    ; Off position third bounce
srv2OnRate                  ; On rate
srv2On                      ; On position
srv2On1                     ; On position first bounce
srv2On2                     ; On position second bounce
srv2On3                     ; On position third bounce

; Servo 3 position and rate settings
;**********************************************************************

srv3OffRate                 ; Off rate
srv3Off                     ; Off position
srv3Off1                    ; Off position first bounce
srv3Off2                    ; Off position second bounce
srv3Off3                    ; Off position third bounce
srv3OnRate                  ; On rate
srv3On                      ; On position
srv3On1                     ; On position first bounce
srv3On2                     ; On position second bounce
srv3On3                     ; On position third bounce

; Servo 4 position and rate settings
;**********************************************************************

srv4OffRate                 ; Off rate
srv4Off                     ; Off position
srv4Off1                    ; Off position first bounce
srv4Off2                    ; Off position second bounce
srv4Off3                    ; Off position third bounce
srv4OnRate                  ; On rate
srv4On                      ; On position
srv4On1                     ; On position first bounce
srv4On2                     ; On position second bounce
srv4On3                     ; On position third bounce

; Servo current positions
;**********************************************************************

srv1NowL                    ; Servo 1 current position low byte
                            ; (also used for drive shutoff timer)
srv1NowH                    ; Servo 1 current position high byte

srv2NowL                    ; Servo 2 current position low byte
                            ; (also used for drive shutoff timer)
srv2NowH                    ; Servo 2 current position high byte

srv3NowL                    ; Servo 3 current position low byte
                            ; (also used for drive shutoff timer)
srv3NowH                    ; Servo 3 current position high byte

srv4NowL                    ; Servo 4 current position low byte
                            ; (also used for drive shutoff timer)
srv4NowH                    ; Servo 4 current position high byte

; Servo movement sequence states
;**********************************************************************

srv1State                   ; Servo 1 movement state
srv2State                   ; Servo 2 movement state
srv3State                   ; Servo 3 movement state
srv4State                   ; Servo 4 movement state

; Output port value cache
;**********************************************************************

portCval

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

	ORG     EESTART         ; EEPROM data area

eeDataStart

; Servo extended travel selections and last switch input values
;**********************************************************************

    DE      0

; Servo 1 position and rate settings
;**********************************************************************

    DE      16              ; Off rate
    DE      (MIDPOINT - 20) ; Off position
    DE      (MIDPOINT -  5) ; Off position first bounce
    DE      (MIDPOINT - 10) ; Off position second bounce
    DE      (MIDPOINT - 15) ; Off position third bounce
    DE      16              ; On rate
    DE      MIDPOINT        ; On position
    DE      (MIDPOINT - 15) ; On position first bounce
    DE      (MIDPOINT - 10) ; On position second bounce
    DE      (MIDPOINT -  5) ; On position third bounce

; Servo 2 position and rate settings
;**********************************************************************

    DE      16              ; Off rate
    DE      (MIDPOINT - 20) ; Off position
    DE      (MIDPOINT -  5) ; Off position first bounce
    DE      (MIDPOINT - 10) ; Off position second bounce
    DE      (MIDPOINT - 15) ; Off position third bounce
    DE      16              ; On rate
    DE      MIDPOINT        ; On position
    DE      (MIDPOINT - 15) ; On position first bounce
    DE      (MIDPOINT - 10) ; On position second bounce
    DE      (MIDPOINT -  5) ; On position third bounce

; Servo 3 position and rate settings
;**********************************************************************

    DE      16              ; Off rate
    DE      (MIDPOINT - 20) ; Off position
    DE      (MIDPOINT -  5) ; Off position first bounce
    DE      (MIDPOINT - 10) ; Off position second bounce
    DE      (MIDPOINT - 15) ; Off position third bounce
    DE      16              ; On rate
    DE      MIDPOINT        ; On position
    DE      (MIDPOINT - 15) ; On position first bounce
    DE      (MIDPOINT - 10) ; On position second bounce
    DE      (MIDPOINT -  5) ; On position third bounce

; Servo 4 position and rate settings
;**********************************************************************

    DE      16              ; Off rate
    DE      (MIDPOINT - 20) ; Off position
    DE      (MIDPOINT -  5) ; Off position first bounce
    DE      (MIDPOINT - 10) ; Off position second bounce
    DE      (MIDPOINT - 15) ; Off position third bounce
    DE      16              ; On rate
    DE      MIDPOINT        ; On position
    DE      (MIDPOINT - 15) ; On position first bounce
    DE      (MIDPOINT - 10) ; On position second bounce
    DE      (MIDPOINT -  5) ; On position third bounce

; Number of settings to load/save from/to EEPROM
NUMSETTINGS EQU ($ - eeDataStart)


;**********************************************************************
; Macro: PCLATH setup                                                 *
;**********************************************************************
SetPCLATH  macro    codeLabel

    movlw   high codeLabel
    movwf   PCLATH

    endm


;**********************************************************************
; Macro: Delay loop                                                   *
;**********************************************************************
Delay      macro    delayValue

#if (1 < ((delayValue - 4) / 5))
    movlw  ((delayValue - 4) / 5)
#else
    movlw  1
#endif
    call    delayLoop

    endm


;**********************************************************************
; Macro: Set servo On state                                           *
;**********************************************************************
ServoOnState  macro     State

    movlw   SRVONST             ; Get initial "on" movement state index
    btfss   State,SRVONSTBIT    ; Skip if in "on" movement sequence ...
    movwf   State               ; ... else start "on" movement sequence

    endm


;**********************************************************************
; Macro: Set servo Off state                                          *
;**********************************************************************
ServoOffState  macro    State

    movlw   SRVOFFST            ; Get initial "off" movement state index
    btfsc   State,SRVONSTBIT    ; Skip if in "off" movement sequence ...
    movwf   State               ; ... else start "off" movement sequence

    endm


;**********************************************************************
; Macro: Initalise servo current position and sequences               *
;**********************************************************************
ServoInit  macro    Off, On, ONf, ONb, NowH, NowL, State

    movf    Off,W           ; Set current position as Off setting
    btfsc   ONf,ONb         ; Skip if input is off ...
    movf    On,W            ; ... else set current position as On setting
    movwf   NowH
    clrf    NowL

    btfsc   ONf,ONb             ; Skip if input is off ...
    bsf     State,SRVONSTBIT    ; ... else change state to On shutdown

    endm


;**********************************************************************
; Macro: Servo current position update                                *
;     W holds base address (rate) of settings for direction of travel *
;**********************************************************************
ServoUpdate  macro  State, NowL, DIS

    local   checkTimer, state3or2, loadTimer, runTimer, state1or0, endUpdate

    movwf   FSR             ; Indirectly address servo settings for state

    movlw   SRVMVMASK       ; Mask non movement states bits ...
    andwf   State,W         ; ... from servo movement state
    btfsc   STATUS,Z        ; Skip if all movement not yet complete ...
    goto    checkTimer      ; ... otherwise skip movement update

    ; Servo movement in progress, update current position
    ;******************************************************************

    bcf     sysFlags,DIS    ; Ensure servo drive is enabled

    movf    State,W         ; Servo current movement state in W
    call    getServoTarget  ; temp1 = target position, temp2 = rate for state

    movlw   NowL            ; Load servo current position low byte address ...
    movwf   FSR             ; ... into indirect addressing register

    movf    (NowL + 1),W    ; Load servo current position high byte into W

    call    updateServo     ; Update servo current position

    btfss   STATUS,Z        ; Check if current and target positions match ...
    goto    endUpdate       ; ... if not keep moving

    decf    State,F         ; Advance to next movement state
    movlw   SRVMVMASK       ; Mask non movement states bits ...
    andwf   State,W         ; ... from servo movement state
    movlw   TIMEDRIVE
    btfsc   STATUS,Z        ; Skip if all movement not yet complete ...
    movwf   NowL            ; ... else load drive shutoff timer
    goto    endUpdate

checkTimer
    ; Servo movement complete, if necessary perform drive shutoff
    ;******************************************************************

    movf    NowL,F          ; Test drive shutoff timer ...
    btfsc   STATUS,Z        ; ... skip if not expired ...
    goto    endUpdate       ; ... else nothing to do

    decfsz  NowL,F          ; Decrement timer, skip if expired ...
    goto    endUpdate       ; ... else remain in same state

    btfsc   DRVOFF          ; Skip if drive shutoff not selected ...
    bsf     sysFlags,DIS    ; ... else disable servo drive

endUpdate

    endm


;**********************************************************************
; Macro: Servo position update                                        *
;**********************************************************************
UpdateServo  macro  ONf, ONb, OffRate, OnRate, State, NowL, DIS

    local   updateSrvOn, doUpdate

    btfsc   ONf,ONb         ; Skip if input off ...
    goto    updateSrvOn     ; ... else perform servo On update

    ServoOffState  State

    movlw   OffRate
    goto    doUpdate

updateSrvOn
    ServoOnState   State

    movlw   OnRate

doUpdate
    ServoUpdate    State, NowL, DIS

    endm


;**********************************************************************
; Macro: Interrupt set servo pulse duration                           *
;     Puts high byte of current position into timer1 low byte         *
;     Puts low byte of current position into accumulator              *
;**********************************************************************
ServoPulse  macro  OUTf, OUTb, NowH, NowL, XTNDf, XTNDb

    bsf     OUTf,OUTb       ; Turn on servo control output

    movf    NowH,W          ; Get high byte of duration for position ...
    movwf   TMR1L           ; ... and save in low byte of timer1

    movf    NowL,W          ; Get low byte of duration for position

    btfss   XTNDf,XTNDb     ; Skip if servo extended travel is selected
    goto    nrmlPulse
    goto    xtndPulse

    endm


;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

    ORG     BOOTVECTOR      ; Processor reset vector

bootVector
    clrf    INTCON          ; Disable interrupts
    clrf    INTCON          ; Ensure interrupts are disabled
    goto    initialise


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

    ORG     INTVECTOR       ; Interrupt vector

beginISR
    movwf   w_isr           ; Save off current W register contents
    swapf   STATUS,W        ; Move status register into W register
    BANKSEL REGBANK0        ; Ensure register page 0 is selected
    movwf   status_isr      ; Save off contents of STATUS register
    movf    PCLATH,W        ; Move PCLATH register into W register
    movwf   pclath_isr      ; Save off contents of PCLATH register

    btfsc   PIR1,TMR1IF     ; Skip if timer1 interrupt flag not set ...
    goto    runCycle        ; ... otherwise run servo pulse cycle

    btfss   INTCON,T0IF     ; Skip if timer0 interrupt flag set ...
    goto    endISR          ; ... otherwise exit interrupt service

beginCycle
    bcf     INTCON,T0IF     ; Clear the timer0 interrupt bitflag

    movlw   CYCLEINT
    addwf   TMR0,F          ; Reload interrupt interval till cycle start

runCycle
    bcf     T1CON,TMR1ON    ; Ensure timer1 not running
    bcf     PIR1,TMR1IF     ; Clear the timer1 interrupt bitflag

    movlw   ~OUTMASK
    andwf   portCval,F      ; Turn off all servo control outputs

    movlw   OUTMASK
    andwf   sysFlags,W
    iorwf   portCval,F      ; Turn back on disabled servo control outputs

    incf    cycleState,F    ; Advance to next state

    SetPCLATH cycleStateTable
    movf    cycleState,W    ; Use cycle state value ...
    addwf   PCL,F           ; ... as index for code jump

cycleStateTable
    goto    endCycle
    goto    srv1Pulse
    goto    srv2Pulse
    goto    srv3Pulse
    goto    srv4Pulse
    clrf    cycleState      ; End of cycle, reset cycle state

#if (high cycleStateTable) != (high $)
    error "Interrupt cycle state jump table spans 8 bit boundary"
#endif

    goto    endCycle


; Duration of servo pulses are timed using timer1. This is a 16 bit counter
; configured to increment and generate an interrupt on roll over from
; hexadecimal FFFF to 0000. This allows a range of counts from hexadecimal 1
; to 10000, or as decimal 0 to 65536.
;
; The timer is configured to increment from the instruction cycle clock which
; is the oscillator divided by 4, so with 4MHz oscillator this gives a tick
; rate of 1MHz (each tick = 1uSec).
;
; Position settings are 8 bit numbers giving a range of hexadecimal FF to 00,
; or decimal 255 to 0. This gives a range of intervals from 255 to 0 uSecs
; (0.255 to 0 mSecs).
;
; Multiplying the position setting by 4 gives a range of hexadecimal 3FC to
; 000, or decimal 1020 to 0, a range of intervals from 1020 to 0 uSecs (1.02
; to 0 mSecs).
;
; Multiplying the position setting by 8 gives a range of hexadecimal 7F8 to
; 000, or decimal 2040 to 0, a range of intervals from 2040 to 0 uSecs (2.04
; to 0 mSecs).
;
; As the timer counter increments from hexadecimal 0 to overflow at 10000 to
; set the number of ticks for a position value it should be subtracted from
; hexadecimal 1000. However it's quicker and simpler to pad the value with
; leading zeros and use it directly.
;
; Position value 255:
;  0xFF -> 0xFFFF, x4 = 0xFFFC giving 0x1000 - 0xFFFC = 4 uSecs (0.004 mSecs)
;  0xFF -> 0xFFFF, x8 = 0xFFF8 giving 0x1000 - 0xFFF8 = 8 uSecs (0.008 mSecs)
;
; Position value 0:
;  0x0 -> 0xFF00, x4 = 0xFC00 giving 0x1000 - 0xFC00 = 1024 uSecs (1.024 mSecs)
;  0x0 -> 0xFF00, x8 = 0xF800 giving 0x1000 - 0xF800 = 2048 uSecs (2.048 mSecs)
;
; In practise although the position setpoints are 8 bit values the current are
; 12 bit values allowing a more fine grained movement of the servos.
;
; Lastly the fixed starting portion of the pulse can be added by simply
; subtracting a fixed amount from the value loaded into the timer counter,
; this causing more ticks to be needed before the counter overflows.
; Subtracting decimal 1000 adds 1000 uSecs (1 mSec), subtracting 500 adds
; 500 uSecs (0.5 mSec)

    ; Produce a pulse between 0.5 and 2.5 mSec
    ;******************************************************************

    ; High byte of current position is in timer1 low byte
    ; Low byte of current position is accumulator

xtndPulse
    iorlw   B'00011111'     ; Mask all but duration low byte top three bits ...
    movwf   TMR1H           ; ... and save in high byte of timer1

    ; Duration of position part of pulse x 8 forms first eleven bits of timer1
    ; Puts upper three bits of position duration low byte into timer
    ; Results in a pulse length between 0.008 and 2.048 mSecs
    bsf     STATUS,C        ; Ensure 1 shifted into timer1 high byte
    rlf     TMR1H,F         ; Rotate timer1H <- 1, carry <- position bit 7
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 7, carry <- bit 15
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 15, carry <- bit 6
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 6, carry <- bit 14
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 14, carry <- bit 5
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 5, carry <- bit 13
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 13, carry <- bit 4

    ; Subtract fixed amount, adjusted for cycles in interrupt from on to off,
    ; to lengthen pulse by 0.5 mSec to give between 0.508 and 2.548 mSecs
    movlw   low ((PULSEINT - 50) * 2)
    subwf   TMR1L,F         ; Add pulse start time to low byte of timer1
    btfss   STATUS,C        ; Skip if no borrow from low byte subtraction ...
    decf    TMR1H,F         ; ... else adjust timer1 high byte
    movlw   high ((PULSEINT - 50) * 2)
    subwf   TMR1H,F         ; Add pulse start time to high byte of timer1

    goto    endPulse

    ; Produce a pulse between 1 and 2 mSec
    ;******************************************************************

    ; High byte of current position is in timer1 low byte
    ; Low byte of current position is in accumulator

nrmlPulse
    iorlw   B'00111111'     ; Mask all but duration low byte top two bits ...
    movwf   TMR1H           ; ... and save in high byte of timer1

    ; Subtract fixed amount, adjusted for cycles in interrupt from on to off,
    ; to lengthen pulse by 1 mSec (after multiplication by 4 below)
    movlw   (PULSEINT - 19)
    subwf   TMR1L,F         ; Add pulse start time to low byte of timer1

    ; Duration of position part of pulse x 4 forms first ten bits of timer1
    ; Puts upper two bits of position duration low byte into timer
    ; Also adjusts timer1 high byte for any borrow from low byte subtraction
    ; Results in a pulse length between 1.004 and 2.024 mSecs
    rlf     TMR1H,F         ; Rotate timer1H <- borrow, carry <- position bit 7
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 7, carry <- bit 15
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 15, carry <- bit 6
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 6, carry <- bit 14
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 14, carry <- bit 5

endPulse
    bsf     T1CON,TMR1ON    ; Start timer1 running

endCycle
    movf    portCval,W      ; Write servo control bits ...
    movwf   OUTPORT         ; ... to output port

endISR
    ; Exit from interrupt service routine
    movf    pclath_isr,W    ; Retrieve copy of PCLATH register
    movwf   PCLATH          ; Restore pre-isr PCLATH register contents
    swapf   status_isr,W    ; Retrieve copy of STATUS register
    movwf   STATUS          ; Restore pre-isr STATUS register contents
    swapf   w_isr,F
    swapf   w_isr,W         ; Restore pre-isr W register contents

    retfie                  ; Return from interrupt

    ; Start pulse for servos, jumped to from beggining of ISR
    ;******************************************************************

srv1Pulse
    bcf     RUNMAIN         ; Disable main program loop

    ServoPulse  SRV1OUT, srv1NowH, srv1NowL, SRV1XTND

srv2Pulse
    ServoPulse  SRV2OUT, srv2NowH, srv2NowL, SRV2XTND

srv3Pulse
    ServoPulse  SRV3OUT, srv3NowH, srv3NowL, SRV3XTND

srv4Pulse
    ServoPulse  SRV4OUT, srv4NowH, srv4NowL, SRV4XTND


;**********************************************************************
; ASCII digit tens value subroutine                                   *
;     Digit in temp3                                                  *
;     Value returned in W                                             *
;**********************************************************************
asciiTens
    SetPCLATH asciiTensTable
    movf    temp3,W
    addwf   PCL,F

asciiTensTable
    retlw   0
    retlw  10
    retlw  20
    retlw  30
    retlw  40
    retlw  50
    retlw  60
    retlw  70
    retlw  80
    retlw  90

#if (high asciiTensTable) != (high $)
    error "Ascii tens jump table spans 8 bit boundary"
#endif


;**********************************************************************
; Servo4 speed conversion subroutine                                  *
;     Servo4 speed in temp4                                           *
;     Sema4 speed returned in W                                       *
;**********************************************************************
convertSpeed
    SetPCLATH speedTable
    movlw   0x07
    andwf   temp4,W
    addwf   PCL,F

speedTable
    retlw  0x00
    retlw  0x5F
    retlw  0x7F
    retlw  0x9F
    retlw  0xBF
    retlw  0xCF
    retlw  0xDF
    retlw  0xEF

#if (high speedTable) != (high $)
    error "Speed conversion jump table spans 8 bit boundary"
#endif


;**********************************************************************
; Servo setting position offset for state lookup subroutine           *
;     Servo movement state passed in temp1 (bits 3 to 0)              *
;                                                                     *
;     Setting offset returned in W                                    *
;**********************************************************************
getServoSettingOffset
    SetPCLATH settingOffsetTable

    ; "Off" movement sequence states, state index decrements from 7 to 0
    ; Actual movement completed after state 1, states 0 = drive shutoff
    ;
    ; "On" movement sequence states, state index decrements from 15 to 8
    ; Actual movement completed after state 9, states 8 = drive shutoff

    movf    temp1,W     ; Load state into accumulator
    andlw   SRVMVMASK   ; Isolate bits 0 to 2, makes states 15 to 8 == 7 to 0
    addwf   PCL,F

settingOffsetTable
    retlw   (srv1Off  - srv1OffRate) ; State 0, timeout drive shutoff
    retlw   (srv1Off  - srv1OffRate) ; State 1, final move back to end position
    retlw   (srv1Off3 - srv1OffRate) ; State 2, move to 3rd bounce
    retlw   (srv1Off  - srv1OffRate) ; State 3, move back to end position
    retlw   (srv1Off2 - srv1OffRate) ; State 4, move to 2nd bounce
    retlw   (srv1Off  - srv1OffRate) ; State 5, move back to end position
    retlw   (srv1Off1 - srv1OffRate) ; State 6, move to 1st bounce
    retlw   (srv1Off  - srv1OffRate) ; State 7, intial move to end position

#if (high settingOffsetTable) != (high $)
    error "Servo setting offset lookup table spans 8 bit boundary"
#endif


;**********************************************************************
;    System initialisation                                            *
;**********************************************************************
initialise
    ; Set servo drive outputs to high
    movlw   OUTMASK
    movwf   portCval
    movwf   OUTPORT

    ; Turn comparator off
    movlw   B'00000111'
    movwf   CMCON

    BANKSEL REGBANK1        ; Ensure register page 1 is selected

    ; Set internal oscillator calibration
    call    GETOSCCAL
    movwf   OSCCAL

    ; Configure output port
    movlw   PORTCDIR
    movwf   TRISC

    ; Configure input port
    movlw   PORTADIR
    movwf   TRISA
    movlw   PORTAPU
    movwf   WPUA
    bcf     OPTION_REG,NOT_RAPU

    BANKSEL REGBANK0        ; Ensure register page 0 is selected

    ; Output banner message
    ;******************************************************************

    bsf     RUNMAIN         ; Stop delay loop aborting during serial Tx

    movlw   'S'
    call    dataSrlTx
    movlw   'e'
    call    dataSrlTx
    movlw   'm'
    call    dataSrlTx
    movlw   'a'
    call    dataSrlTx
    movlw   '4'
    call    dataSrlTx
    movlw   'g'
    call    dataSrlTx

    ; Initialise RAM to zero
    ;******************************************************************

    movlw   RAMEND          ; Address of end of RAM
    movwf   RAMSTART        ; Ensure first byte of RAM is non zero
    movwf   FSR             ; Point indirect register to end of RAM

clearRAM
    clrf    INDF            ; Clear byte of RAM addressed by FSR
    decf    FSR,F           ; Decrement FSR to next byte of RAM
    movf    RAMSTART,F      ; Test first byte of RAM
    btfss   STATUS,Z        ; Skip if byte of RAM now zero ...
    goto    clearRAM        ; ... else continue to clear RAM

    ; Initialise settings
    ;******************************************************************

    call    loadAllSettings ; Load all settings from EEPROM

    movlw   TIMEFREEZE      ; Set setting mode timeout (ignore physical inputs)
    movwf   freezeTime

    btfsc   DRVOFFINP       ; Skip if Drive shutoff option input off ...
    bsf     DRVOFF          ; ... else set Drive shutoff option indicator

    ; Set interrupt intervals for output cycles and pulse timing
    ;******************************************************************

    movlw   TMR1OPTIONS     ; Set pulse interrupt clock options
    movwf   T1CON

    BANKSEL REGBANK1        ; Ensure register page 1 is selected

    bsf     PIE1,TMR1IE     ; Enable timer1 interrupts

    movlw   TMR0OPTIONS     ; Set cycle interrupt clock options
    movwf   OPTION_REG

    BANKSEL REGBANK0        ; Ensure register page 0 is selected

    clrf    TMR1H           ; Clear timer1 high byte
    clrf    TMR1L           ; Clear timer1 low byte

    bcf     PIR1,TMR1IF     ; Clear any pending timer1 interrupts
    bsf     INTCON,PEIE     ; Enable peripheral interrupts (needed for timer1)

    movlw   CYCLEINT
    movwf   TMR0            ; Set interrupt interval till cycle start

    bcf     INTCON,T0IF     ; Clear any pending timer0 interrupts
    bsf     INTCON,T0IE     ; Enable timer0 interrupts

    bsf     INTCON,GIE      ; Enable interrupts

;**********************************************************************
;    Main program loop                                                *
;**********************************************************************
main
    ; Wait until output cycle is idle, pause between servo pulse output
    ;
    ; Main loop is run once per output cycle to control servo movement
    ; speed and not during actual pulse as interrupts make serial timing
    ; unreliable
    ;******************************************************************

    bcf     SERTXOUT        ; Set serial TX output = RS232 'mark' (idle)
    call    outputTx

    movf    cycleState,W    ; Test pulse output cycle state
    btfss   STATUS,Z        ; Skip if pulse output cycle not running ...
    goto    main            ; ... otherwise wait for idle

    bsf     RUNMAIN         ; Enable main program loop

    ; Update servo current positions
    ;******************************************************************

    UpdateServo  SRV1ON, srv1OffRate, srv1OnRate, srv1State, srv1NowL, SRV1DIS
    UpdateServo  SRV2ON, srv2OffRate, srv2OnRate, srv2State, srv2NowL, SRV2DIS
    UpdateServo  SRV3ON, srv3OffRate, srv3OnRate, srv3State, srv3NowL, SRV3DIS
    UpdateServo  SRV4ON, srv4OffRate, srv4OnRate, srv4State, srv4NowL, SRV4DIS

    ; Read servo control inputs
    ;******************************************************************

    movf    freezeTime,F    ; Test position setting mode timeout
    btfss   STATUS,Z        ; Skip if timeout not running ...
    decf    freezeTime,F    ; ... else decrement position setting mode timeout
    btfsc   STATUS,Z        ; Skip if timeout running ...
    call    scanServoInputs ; ... else read inputs

    ; Check for reception of serial data
    ;******************************************************************

testSrlRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    main            ; ... otherwise abort

    btfsc   SERRXIN         ; Skip if serial input connected ...
    goto    testSrlRx       ; ... else loop looking for connection

syncSrlRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    main            ; ... otherwise abort

    call    nullSrlRx       ; Receive initial null byte via serial input
    btfss   RXDATAIND       ; Skip if received a null ...
    goto    syncSrlRx       ; ... otherwise abort and restart synchronisation

    ; Synchronised to null byte, receive command (1 byte) and value (3 bytes)
    ;******************************************************************

    ; Receive command code
    ;******************************************************************

    call    dataSrlRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSrlRx       ; ... otherwise abort and restart synchronisation

    btfsc   temp3,ASCIIBIT  ; Test received byte is an ASCII character ...
    goto    syncSrlRx       ; ... otherwise abort

    movf    temp3,W         ; Save received byte ...
    movwf   temp2           ; ... as command

    ; Receive first digit of value for command, hundreds
    ;******************************************************************

    call    digitSrlRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSrlRx       ; ... otherwise abort and restart synchronisation

    ; Initialise command value with hundreds digit, can only be 0, 100, or 200
    clrw                    ; Start with 0
    btfsc   temp3,0         ; Skip if not 100 ...
    movlw   100             ; ... else set to 100
    btfsc   temp3,1         ; Skip if not 200 ...
    movlw   200             ; ... else set to 200
    movwf   temp4           ; Initialise command value

    ; Receive second digit of value for command, tens
    ;******************************************************************

    call    digitSrlRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSrlRx       ; ... otherwise abort and restart synchronisation

    ; Add tens digit to command value
    call    asciiTens
    addwf   temp4,F

    ; Receive third digit of value for command, units
    ;******************************************************************

    call    digitSrlRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSrlRx       ; ... otherwise abort and restart synchronisation

    movf    temp3,W
    addwf   temp4,F         ; Add units digit to command value

    ; Decode and action the command
    ;******************************************************************

    SetPCLATH commandTable

    movlw   SETTINGBASE     ; Convert received command from ASCII ...
    subwf   temp2,W         ; ... to numerical value
    btfss   STATUS,C        ; Check command character not less than base ...
    goto    receivedCommand ; ... else command is not position or rate setting

    addwf   PCL,F           ; Use numerical command as index for code jump

commandTable
    ; Original Servo4 commands
    ;******************************************************************

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

    ; Additional Sema4b commands
    ;******************************************************************

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

    ; Additional Sema4c commands
    ;******************************************************************

    goto    receivedXtnd

    ; Additional Sema4g commands
    ;******************************************************************

    goto    receivedXtndOn
    goto    receivedXtndOff

#if (high commandTable) != (high $)
    error "Received command jump table spans 8 bit boundary"
#endif

    ; Servo 1 specific actions for specific position setting commands
    ;******************************************************************

srv1SetOffPosition
    movlw   srv1Off1        ; Store received value as ...
    call    servo4Bnc       ; ... Off bounce positions (Servo4 compatabillity)

srv1SetOffOnly
    movlw   srv1Off         ; Store servo Off position
    goto    srv1OffPosition

srv1SetOff1Position
    movlw   srv1Off1        ; Store servo Off bounce 1 position
    goto    srv1OffPosition

srv1SetOff2Position
    movlw   srv1Off2        ; Store servo Off bounce 2 position
    goto    srv1OffPosition

srv1SetOff3Position
    movlw   srv1Off3        ; Store servo Off bounce 3 position

srv1OffPosition
    movwf   FSR
    movlw   SRVOFFEND       ; Initial movement state is Off drive shutdown
    bcf     SRV1ON          ; Set servo input off
    goto    srv1Position

srv1SetOnPosition
    movlw   srv1On1         ; Store received value as ...
    call    servo4Bnc       ; ... On bounce positions (Servo4 compatabillity)

srv1SetOnOnly
    movlw   srv1On          ; Store servo On position
    goto    srv1OnPosition

srv1SetOn1Position
    movlw   srv1On1         ; Store servo On bounce 1 position
    goto    srv1OnPosition

srv1SetOn2Position
    movlw   srv1On2         ; Store servo On bounce 2 position
    goto    srv1OnPosition

srv1SetOn3Position
    movlw   srv1On3         ; Store servo On bounce 3 position

srv1OnPosition
    movwf   FSR
    movlw   SRVONEND        ; Initial movement state is On drive shutdown
    bsf     SRV1ON          ; Set servo input on

srv1Position
    movwf   srv1State       ; Set movement state
    movf    temp4,W
    movwf   INDF
    movwf   srv1NowH        ; Set current position as received setting value
    clrf    srv1NowL
    goto    receivedPosition

    ; Servo 2 specific actions for specific position setting commands
    ;******************************************************************

srv2SetOffPosition
    movlw   srv2Off1        ; Store received value as ...
    call    servo4Bnc       ; ... Off bounce positions (Servo4 compatabillity)

srv2SetOffOnly
    movlw   srv2Off         ; Store servo Off position
    goto    srv2OffPosition

srv2SetOff1Position
    movlw   srv2Off1        ; Store servo Off bounce 1 position
    goto    srv2OffPosition

srv2SetOff2Position
    movlw   srv2Off2        ; Store servo Off bounce 2 position
    goto    srv2OffPosition

srv2SetOff3Position
    movlw   srv2Off3        ; Store servo Off bounce 3 position

srv2OffPosition
    movwf   FSR
    movlw   SRVOFFEND       ; Initial movement state is Off drive shutdown
    bcf     SRV2ON          ; Set servo input off
    goto    srv2Position

srv2SetOnPosition
    movlw   srv2On1         ; Store received value as ...
    call    servo4Bnc       ; ... On bounce positions (Servo4 compatabillity)

srv2SetOnOnly
    movlw   srv2On          ; Store servo On position
    goto    srv2OnPosition

srv2SetOn1Position
    movlw   srv2On1         ; Store servo On bounce 1 position
    goto    srv2OnPosition

srv2SetOn2Position
    movlw   srv2On2         ; Store servo On bounce 2 position
    goto    srv2OnPosition

srv2SetOn3Position
    movlw   srv2On3         ; Store servo On bounce 3 position

srv2OnPosition
    movwf   FSR
    movlw   SRVONEND        ; Initial movement state is On drive shutdown
    bsf     SRV2ON          ; Set servo input on

srv2Position
    movwf   srv2State       ; Set movement state
    movf    temp4,W
    movwf   INDF
    movwf   srv2NowH        ; Set current position as received setting value
    clrf    srv2NowL
    goto    receivedPosition

    ; Servo 3 specific actions for specific position setting commands
    ;******************************************************************

srv3SetOffPosition
    movlw   srv3Off1        ; Store received value as ...
    call    servo4Bnc       ; ... Off bounce positions (Servo4 compatabillity)

srv3SetOffOnly
    movlw   srv3Off         ; Store servo Off position
    goto    srv3OffPosition

srv3SetOff1Position
    movlw   srv3Off1        ; Store servo Off bounce 1 position
    goto    srv3OffPosition

srv3SetOff2Position
    movlw   srv3Off2        ; Store servo Off bounce 2 position
    goto    srv3OffPosition

srv3SetOff3Position
    movlw   srv3Off3        ; Store servo Off bounce 3 position

srv3OffPosition
    movwf   FSR
    movlw   SRVOFFEND       ; Initial movement state is Off drive shutdown
    bcf     SRV3ON          ; Set servo input off
    goto    srv3Position

srv3SetOnPosition
    movlw   srv3On1         ; Store received value as ...
    call    servo4Bnc       ; ... On bounce positions (Servo4 compatabillity)

srv3SetOnOnly
    movlw   srv3On          ; Store servo On position
    goto    srv3OnPosition

srv3SetOn1Position
    movlw   srv3On1         ; Store servo On bounce 1 position
    goto    srv3OnPosition

srv3SetOn2Position
    movlw   srv3On2         ; Store servo On bounce 2 position
    goto    srv3OnPosition

srv3SetOn3Position
    movlw   srv3On3         ; Store servo On bounce 3 position

srv3OnPosition
    movwf   FSR
    movlw   SRVONEND        ; Initial movement state is On drive shutdown
    bsf     SRV3ON          ; Set servo input on

srv3Position
    movwf   srv3State       ; Set movement state
    movf    temp4,W
    movwf   INDF
    movwf   srv3NowH        ; Set current position as received setting value
    clrf    srv3NowL
    goto    receivedPosition

    ; Servo 4 specific actions for specific position setting commands
    ;******************************************************************

srv4SetOffPosition
    movlw   srv4Off1        ; Store received value as ...
    call    servo4Bnc       ; ... Off bounce positions (Servo4 compatabillity)

srv4SetOffOnly
    movlw   srv4Off         ; Store servo Off position
    goto    srv4OffPosition

srv4SetOff1Position
    movlw   srv4Off1        ; Store servo Off bounce 1 position
    goto    srv4OffPosition

srv4SetOff2Position
    movlw   srv4Off2        ; Store servo Off bounce 2 position
    goto    srv4OffPosition

srv4SetOff3Position
    movlw   srv4Off3        ; Store servo Off bounce 3 position

srv4OffPosition
    movwf   FSR
    movlw   SRVOFFEND       ; Initial movement state is Off drive shutdown
    bcf     SRV4ON          ; Set servo input off
    goto    srv4Position

srv4SetOnPosition
    movlw   srv4On1         ; Store received value as ...
    call    servo4Bnc       ; ... On bounce positions (Servo4 compatabillity)

srv4SetOnOnly
    movlw   srv4On          ; Store servo On position
    goto    srv4OnPosition

srv4SetOn1Position
    movlw   srv4On1         ; Store servo On bounce 1 position
    goto    srv4OnPosition

srv4SetOn2Position
    movlw   srv4On2         ; Store servo On bounce 2 position
    goto    srv4OnPosition

srv4SetOn3Position
    movlw   srv4On3         ; Store servo On bounce 3 position

srv4OnPosition
    movwf   FSR
    movlw   SRVONEND        ; Initial movement state is On drive shutdown
    bsf     SRV4ON          ; Set servo input on

srv4Position
    movwf   srv4State       ; Set movement state
    movf    temp4,W
    movwf   INDF
    movwf   srv4NowH        ; Set current position as received setting value
    clrf    srv4NowL

    ; Common end action for position setting commands
    ;******************************************************************

receivedPosition
    movlw   TIMEFREEZE      ; Set setting mode timeout (ignore physical inputs)
    movwf   freezeTime

receivedSetting
    bcf     SYNCEDIND       ; Clear servo settings synchronised indicator
    goto    syncSrlRx       ; Loop looking for next command sequence

    ; Servo 1 specific actions for rate setting commands
    ;******************************************************************

srv1SetOffRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv1NewOffRate
    movlw   srv1OffRate     ; Address servo 1 Off rate
    goto    receivedRate

srv1SetOnRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv1NewOnRate
    movlw   srv1OnRate      ; Address servo 1 On rate
    goto    receivedRate

    ; Servo 2 specific actions for rate setting commands
    ;******************************************************************

srv2SetOffRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv2NewOffRate
    movlw   srv2OffRate     ; Address servo 2 Off rate
    goto    receivedRate

srv2SetOnRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv2NewOnRate
    movlw   srv2OnRate      ; Address servo 2 On rate
    goto    receivedRate

    ; Servo 3 specific actions for rate setting commands
    ;******************************************************************

srv3SetOffRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv3NewOffRate
    movlw   srv3OffRate     ; Address servo 3 Off rate
    goto    receivedRate

srv3SetOnRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv3NewOnRate
    movlw   srv3OnRate      ; Address servo 3 On rate
    goto    receivedRate

    ; Servo 4 specific actions for rate setting commands
    ;******************************************************************

srv4SetOffRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv4NewOffRate
    movlw   srv4OffRate     ; Address servo 4 Off rate
    goto    receivedRate

srv4SetOnRate
    call    convertSpeed    ; Convert Servo4 speed to Sema4 speed
    movwf   temp4

srv4NewOnRate
    movlw   srv4OnRate      ; Address servo 4 On rate

    ; Common end action for rate setting commands
    ;******************************************************************

receivedRate
    movwf   FSR             ; Indirectly address the appropriate servo rate
    comf    temp4,W         ; Complement received value ...
    btfsc   STATUS,Z        ; ... skip if complemented speed not zero ...
    movlw   1               ; ... else limit to mininum speed ...
    movwf   INDF            ; ... and store as the appropriate servo rate

    clrf    freezeTime      ; Clear setting mode timeout (read physical inputs)
    goto    receivedSetting

    ; Received servo extended travel selections
    ;******************************************************************

receivedXtnd
    movf    temp4,W         ; Get received extended travel selections
    iorlw   ~XTNDMASK       ; Protect servo control flags other than extended travel
    andwf   srvCtrl,F       ; Clear deselected extended travel flags
    andlw   XTNDMASK        ; Isolate received extended travel selections
    iorwf   srvCtrl,F       ; Set extended travel selections

    goto    receivedSetting

    ; Received set servo extended travel selections
    ;******************************************************************

receivedXtndOn
    movf    temp4,W         ; Get received extended travel selections to set
    andlw   XTNDMASK        ; Protect servo control flags other than extended travel
    iorwf   srvCtrl,F       ; Set selected extended travel flags

    goto    receivedSetting

    ; Received clear servo extended travel selections
    ;******************************************************************

receivedXtndOff
    comf    temp4,W         ; Get received extended travel selections to clear
    iorlw   ~XTNDMASK       ; Protect servo control flags other than extended travel
    andwf   srvCtrl,F       ; Clear deselected extended travel flags

    goto    receivedSetting

    ; Actions for commands other than position, rate, or travel setting
    ;******************************************************************

receivedCommand
    movf    temp2,W         ; Test if command ...
    xorlw   RUNCMND         ; ... is to exit setting mode ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    testForStore    ; ... otherwise test for store command

    clrf    freezeTime      ; Clear setting mode timeout (read physical inputs)
    goto    syncSrlRx       ; Loop looking for next command sequence

testForStore
    movf    temp2,W         ; Test if command ...
    xorlw   STORECMND       ; ... is to store settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    testForReset    ; ... otherwise test for reset command

    btfsc   SYNCEDIND       ; Skip if settings not synchronised with EEPROM ...
    goto    syncSrlRx       ; ... else loop looking for next command sequence

    ; Store settings to EEPROM
    ;******************************************************************

    movlw   NUMSETTINGS
    movwf   temp1           ; Set number of settings to write to EEPROM

    movlw   ENDSETTINGS     ; Load end address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

storeSetting
    movf    INDF,W          ; Get setting value ...
    movwf   temp2           ; ... as value to write to EEPROM

    decf    temp1,W         ; Set index into EEPROM from count
    call    writeEEPROM

    decf    FSR,F           ; Decrement to address of next setting
    decfsz  temp1,F         ; Decrement settings count, skip if zero ...
    goto    storeSetting    ; ... else loop until all settings have been stored

    bsf     SYNCEDIND       ; Set servo settings synchronised indicator

    goto    syncSrlRx       ; Loop looking for next command sequence

testForReset
    movf    temp2,W         ; Test if command ...
    xorlw   RESETCMND       ; ... is to reset settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    testForReboot   ; ... otherwise test for reboot command

    btfss   SYNCEDIND       ; Skip if settings synchronised with EEPROM ...
    call    loadAllSettings ; ... else restore all settings from EEPROM

testForReboot
    movf    temp2,W         ; Test if command ...
    xorlw   REBOOTCMND      ; ... is to reboot ...
    btfsc   STATUS,Z        ; ... if not skip ...
    goto    bootVector      ; ... otherwise reboot

    goto    syncSrlRx       ; Loop looking for next command sequence


;**********************************************************************
; Delay loop subroutine                                               *
;     Loop count in W                                                 *
;**********************************************************************
delayLoop
    movwf   temp1           ; Load delay loop count into temp1

loopDelay
    decfsz  temp1,F         ; Continue until temp1 reaches zero
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort
    goto    loopDelay       ; Loop if main program loop enabled


;**********************************************************************
; Write to EEPROM subroutine                                          *
;     Address in W (index starting at zero)                           *
;     Value in temp2                                                  *
;**********************************************************************
writeEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitWriteEE
    btfsc   EECON1,WR       ; Skip if EEPROM write not in progress ...
    goto    waitWriteEE     ; ... else wait for previous write to complete

    movwf   EEADR           ; Set address of EEPROM location to write to
    movf    temp2,W
    movwf   EEDATA          ; Set value to write to EEPROM location

    bcf     INTCON,GIE      ; Disable interrupts
    bsf     EECON1,WREN     ; Enable EEPROM writes

    ; Unlock EEPROM write
    movlw   0x55
    movwf   EECON2
    movlw   0xAA
    movwf   EECON2

    bsf     EECON1,WR       ; Trigger EEPROM hardware write cycle
    bcf     EECON1,WREN     ; Disable EEPROM writes

    bsf     INTCON,GIE      ; Enable interrupts
    BANKSEL REGBANK0        ; Select register page 0
    return


;**********************************************************************
; Read from EEPROM subroutine                                         *
;     Address in W (index starting at zero)                           *
;                                                                     *
;     Return value in W                                               *
;**********************************************************************
readEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitReadEE
    btfsc   EECON1,WR       ; Skip if EEPROM write not in progress ...
    goto    waitReadEE      ; ... else wait for write to complete before attempting read

    movwf   EEADR           ; Set address of EEPROM location to read from
    bsf     EECON1,RD       ; Trigger EEPROM read
    movf    EEDATA,W        ; Read value from EEPROM location

    BANKSEL REGBANK0        ; Select register page 0
    return


;**********************************************************************
; Load settings from EEPROM subroutine                                *
;**********************************************************************
loadAllSettings
    movlw   NUMSETTINGS
    movwf   temp1           ; Set index of settings to be read from EEPROM

    movlw   ENDSETTINGS     ; Load end address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

loadSetting
    decf    temp1,W         ; Set index into EEPROM from count
    call    readEEPROM

    movwf   INDF            ; Load setting with value read from EEPROM

    decf    FSR,F           ; Decrement to address of next setting
    decfsz  temp1,F         ; Decrement settings count, skip if zero ...
    goto    loadSetting     ; ... else loop until all settings have been read

    bsf     SYNCEDIND       ; Set servo settings synchronised indicator

    ; Initialise servo movement positions and sequences
    ;******************************************************************

    movlw   SRVOFFEND
    movwf   srv1State
    movwf   srv2State
    movwf   srv3State
    movwf   srv4State

    ; Servo 1
    ServoInit    srv1Off, srv1On, SRV1ON, srv1NowH, srv2NowL, srv1State

    ; Servo 2
    ServoInit    srv2Off, srv2On, SRV2ON, srv2NowH, srv2NowL, srv2State

    ; Servo 3
    ServoInit    srv3Off, srv3On, SRV3ON, srv3NowH, srv3NowL, srv3State

    ; Servo 4
    ServoInit    srv4Off, srv4On, SRV4ON, srv4NowH, srv4NowL, srv4State

    return


;**********************************************************************
; Servo control input scan subroutine                                 *
;     Servo control inputs are stored in a register so they can be    *
;     overridden when a position setting command is received          *
;**********************************************************************
scanServoInputs
    comf    INPORT,W        ; Read, inverted, physical input port
    xorwf   srvCtrl,W       ; Exclusive or current and previous value
    andlw   INPMASK         ; Isolate control input bits read from port

    btfsc   STATUS,Z        ; Skip if inputs have changed ...
    return                  ; ... else do nothing

    clrw                    ; Read previous stored control bits ...
    call    readEEPROM      ; ... from EEPROM ..
    movwf   temp2           ; ... and save these

    movlw   ~INPMASK        ; Mask out ...
    andwf   temp2,F         ; ... previous stored  control input bits
    andwf   srvCtrl,F       ; ... current control input bits

    comf    INPORT,W        ; Read, inverted, physical input port
    andlw   INPMASK         ; Isolate control input bits read from port
    iorwf   srvCtrl,F       ; Set current control input bits

    iorwf   temp2,F         ; Write current control input and previous control non input bits ...
    clrw                    ; ... back to ...
    goto    writeEEPROM     ; ... EEPROM


;**********************************************************************
; Servo4 compatability bounce setting subroutine                      *
;     Address of first bounce position in W                           *
;     Position setting in temp4                                       *
;**********************************************************************
servo4Bnc
    movwf   FSR             ; Indirectly address bounce position settings
    movf    temp4,W         ; Load position setting into W
    movwf   INDF            ; Set bounce 1 position
    incf    FSR,F           ; Indirectly address next bounce position setting
    movwf   INDF            ; Set bounce 2 position
    incf    FSR,F           ; Indirectly address next bounce position setting
    movwf   INDF            ; Set bounce 3 position

    return

;**********************************************************************
; RS232 sync subroutine                                               *
;     Receives null byte, sets RXDATAIND if successful                *
;     Rx bit state echoed back as Tx bit                              *
;**********************************************************************
nullSrlRx
    bcf     RXDATAIND       ; Clear data byte received indicator

loopNullRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    loopNullRx      ; ... else loop seeking possible serial start bit

    ; Delay half a serial bit time in order to sample near middle of bit
    Delay        RXSTARTTIME

    bsf     SERTXOUT        ; Set serial TX output = RS232 'space' (start bit)
    call    outputTx

    movlw   RS232BITS
    movwf   temp3

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles since start bit detected
    Delay        (SRLBITTIME - 15)

nextNullBit
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test RX zero on serial input ...
    return                  ; ... otherwise abort

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    Delay        (SRLBITTIME - 8)

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry
    btfsc   STATUS,C        ; Check if got all serial data bits ...
    goto    nextNullBit     ; ... otherwise look for next bit

    bcf     SERTXOUT        ; Clear serial TX output = RS232 'mark' (stop bit)

    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    outputTx        ; ... otherwise abort

    btfss   SERRXIN         ; Test for stop bit on serial input ...
    bsf     RXDATAIND       ; ... if found set data byte received indicator

    goto    outputTx        ; Return via writeTx




;**********************************************************************
; RS232 input subroutine                                              *
;     Receives data byte into temp3, sets RXDATAIND if successful     *
;     Rx bit state echoed back as Tx bit                              *
;**********************************************************************
dataSrlRx
    bcf     RXDATAIND       ; Clear data byte received indicator

loopSrlRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    loopSrlRx       ; ... else loop seeking possible serial start bit

    ; Delay half a serial bit time in order to sample near middle of bit
    Delay        RXSTARTTIME

    bsf     SERTXOUT        ; Set serial TX output = RS232 'space' (start bit)
    call    outputTx

    movlw   RS232BITS
    movwf   temp3

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles since start bit detected
    Delay        (SRLBITTIME - 16)

nextSrlRxBit
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    bcf     STATUS,C        ; Clear carry flag in status
    btfss   SERRXIN         ; Test RX bit on serial input ...
    bsf     STATUS,C        ; ... if not set then set carry flag in status

    ; Echo Rx bit as Tx bit
    bcf     SERTXOUT        ; Clear serial TX output = RS232 'mark'
    btfss   STATUS,C
    bsf     SERTXOUT        ; Set serial TX output = RS232 'space'
    call    outputTx

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry
    btfss   STATUS,C        ; Check if not got all serial data bits ...
    goto    endSrlRx        ; ... otherwise look for stop bit

continueSrlRx
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    Delay        (SRLBITTIME - 20)
    goto    nextSrlRxBit

endSrlRx
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    Delay        (SRLBITTIME - 20)

    bcf     SERTXOUT        ; Clear serial TX output = RS232 'mark' (stop bit)

    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    outputTx        ; ... otherwise abort

    btfss   SERRXIN         ; Test for stop bit on serial input ...
    bsf     RXDATAIND       ; ... if found set data byte received indicator

    goto    outputTx        ; Return via writeTx


;**********************************************************************
; RS232 output subroutine                                             *
;     Transmits data byte from acumulator                             *
;**********************************************************************
dataSrlTx
    movwf   temp3

    movlw   RS232BITS
    movwf   temp2

    bsf     SERTXOUT        ; Set serial TX output = RS232 'space' (start bit)
    call    outputTx

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between TX writes
    Delay        (SRLBITTIME - 13)

nextSrlTxBit
    bsf     SERTXOUT        ; Set serial TX output = RS232 'space'
    rrf     temp3,F         ; Rotate right RS232 transmit byte through carry
    btfsc   STATUS,C
    bcf     SERTXOUT        ; Clear serial TX output = RS232 'mark'
    call    outputTx

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between TX writes
    Delay        (SRLBITTIME - 16)

    rrf     temp2,F         ; Rotate right RS232 transmit count through carry
    btfsc   STATUS,C        ; Check if sent all serial data bits ...
    goto    nextSrlTxBit    ; ... otherwise send next bit

endSrlTx
    bcf     SERTXOUT        ; Clear serial TX output = RS232 'mark' (stop bit)
    call    outputTx

    ; Delay two serial bit times
    Delay        (SRLBITTIME * 2)

    return


;**********************************************************************
; Serial Tx bit write to output port subroutine (6 clock cycles)      *
;**********************************************************************
outputTx

    bcf     INTCON,GIE      ; Disable interrupts
    movf    portCval,W
    movwf   OUTPORT
    bsf     INTCON,GIE      ; Enable interrupts

    return


;**********************************************************************
; ASCII digit input subroutine                                        *
;     Receives ASCII digit into temp3, sets RXDATAIND if successful   *
;**********************************************************************
digitSrlRx
    call    dataSrlRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    return                  ; ... otherwise abort

    ; Check received byte is an ASCII digit
    movlw   DIGITTEST       ; Test for ASCII digit ...
    xorwf   temp3,F         ; ... converting received byte at same time ...
    movlw   DIGITMASK       ; Mask out lower nibble ...
    andwf   temp3,W         ; ... of received byte ...
    btfss   STATUS,Z        ; .. continue if ASCII digit received ...
    bcf     RXDATAIND       ; ... otherwise mark received data as bad

    return


;**********************************************************************
; Servo position setting for state lookup subroutine                  *
;     Servo movement state passed in W                                *
;     Servo settings base address (rate) passed in FSR                *
;                                                                     *
;     Servo target position returned in temp1                         *
;     Servo rate (speed) returned in temp2                            *
;**********************************************************************
getServoTarget
    movwf   temp1           ; Save servo movement state in temp1

    movf    INDF,W          ; Store servo rate (speed) ...
    movwf   temp2           ; ... in temp2

    call    getServoSettingOffset

    addwf   FSR,F           ; Add offset to servo settings base address
    movf    INDF,W          ; Indirectly get servo setting
    movwf   temp1           ; Store servo target position in temp1

    return


;**********************************************************************
; Servo current position update subroutine                            *
;     Target position in temp1                                        *
;     Rate in temp2                                                   *
;     Current position high byte in W                                 *
;     Current position accessed via FSR (Low byte, High byte)         *
;                                                                     *
;     Return STATUS,Z set when target positions reached               *
;**********************************************************************
updateServo
    ; Test target position against current position
    ; (result used later for increment/decrement jump)
    subwf   temp1,W         ; Target position - current position high byte

    ; Multiply 8 bit rate by 16 to give 16 bit value
    ;  Original 8 bit rate: Rate[7-4][3-0]
    ;  Becomes 16 bit rate: temp3 = 0000Rate[7-4], W = Rate[3-0]0000
    swapf   temp2,W         ; Swap nibbles, gives times 16 but mixed up
    andlw   0x0F            ; Isolate high byte nibble ...
    movwf   temp3           ; ... and save
    swapf   temp2,W         ; Swap nibbles, gives times 16 but mixed up
    andlw   0xF0            ; Isolate low byte nibble

    btfss   STATUS,C        ; Skip if current <= target ...
    goto    decrementServo  ; ... else current > target

incrementServo
    ; Add rate to current position
    ;******************************************************************

    addwf   INDF,F          ; Add rate low to current position low

    btfsc   STATUS,C        ; Check no carry from low byte addition ...
    incf    temp3,F         ; ... else adjust rate high byte

    incf    FSR,F           ; Address current position high

    movf    temp3,W         ; Add rate high ...
    addwf   INDF,F          ; ... to current position high

    btfsc   STATUS,C        ; Skip if no overflow on high byte addition ...
    goto    reachedTarget   ; ... else passed 255, limit to target position

    movf    temp1,W         ; Target position in W
    subwf   INDF,W          ; Current high byte - target position
    btfsc   STATUS,C        ; Skip if target > current ...
    goto    reachedTarget   ; ... else limit to target position

    return                  ; Not yet passed, or reached, target

decrementServo
    ; Subtract rate from current position
    ;******************************************************************

    subwf   INDF,F          ; Subtract rate low from current position low

    btfss   STATUS,C        ; Check if no borrow from low byte subtraction ...
    incf    temp3,F         ; ... else adjust rate high byte

    incf    FSR,F           ; Address current position high

    movf    temp3,W         ; Subtract rate high ...
    subwf   INDF,F          ; ... from current position high

    btfss   STATUS,C        ; Skip if no underflow on high byte subtraction ...
    goto    reachedTarget   ; ... else limit to target position

    movf    temp1,W         ; Target position in W
    subwf   INDF,W          ; Current high byte - target position
    bcf     STATUS,Z        ; Clear STATUS,Z == target position not reached
    btfsc   STATUS,C        ; Skip if target > current ...
    return                  ; ... else not yet passed, or reached, target

reachedTarget
    ; Limit current position to target position
    ;******************************************************************

    movf    temp1,W         ; Set target position ...
    movwf   INDF            ; ... as current position high byte
    decf    FSR,F           ; Address current position low byte
    clrf    INDF            ; Clear current position low byte

    return                  ; STATUS,Z set by preceeding clear

#if ($) > (GETOSCCAL)
    error "Program code overwrites OSCAL value"
#endif

    end

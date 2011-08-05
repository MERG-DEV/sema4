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
;   Gives the pulse output for 1 conventional servo.                  *
;   Input control by on / off switch to ground (active low).          *
;   Port A has active pull-ups to Vdd on inputs.                      *
;   End limits, travel speed, and optional bounce positions           *
;   configured via a serial input (RS232 compatible).                 *
;   Setting values stored in EEPROM.                                  *
;   Option selectable by input bit to shutoff drive pulse shortly     *
;   after completion of movement sequence.                            *
;   Seven position indication "bandpass" filters with configurable    *
;   upper and lower bounds. Indicator on if position at upper or      *
;   lower bound or bounds, otherwise off.                             *
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
;    03 Aug 2011 - Chris White:                                       *
;       Customisation for Anthony Cooper. Only one servo but several  *
;       indicator outputs activated by "bandpass" position filters.   *
;                                                                     *
;    04 Aug 2011 - Chris White:                                       *
;       Added subroutine to set indicator 2 pass band as one third    *
;       between indicator 1 upper and indicator 4 lower bounds,       *
;       indicator 3 pass band two thirds of same.                     *
;                                                                     *
;    05 Aug 2011 - Chris White:                                       *
;       Corrected subroutine to set indicator 1, 2, 3, and 4 pass     *
;       band bounds when either on or off position is changed.        *
;                                                                     *
;**********************************************************************
;                                                                     *
;                             +---+ +---+                             *
;                   +5V -> VDD|1  |_| 14|VSS <- 0V                    *
;        Indicator6 Out <- RA5|2      13|RA0 <- Servo1 In             *
;        Indicator5 Out <- RA4|3      12|RA1 -> Indicator1 Out        *
;                        !MCLR|4      11|RA2 <- Serial Rx             *
;         Drive Shutoff -> RC5|5      10|RC0 -> Servo1 Out            *
;        Indicator7 Out <- RC4|6       9|RC1 -> Indicator2 Out        *
;        Indicator4 Out <- RC3|7       8|RC2 -> Indicator3 Out        *
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

EESTART     EQU    0x2100      ; EEPROM data area

BOOTVECTOR  EQU    0           ; Processor reset vector

INTVECTOR   EQU    0x0004      ; Interrupt vector location

; I/O port and bit definitions
;**********************************************************************

INPORT      EQU    PORTA
PORTADIR    EQU    B'11001101' ; Input: 0, 2, 3  Output: 1, 4, 5
PORTAPU     EQU    B'00001001' ; Pull up on inputs except serial input

OUTPORT     EQU    PORTC
PORTCDIR    EQU    B'00100000' ; All bits outputs except 5 (Drive Shutoff)

; Servo control input port bit definitions (active low)
#define  SRV1INP   INPORT,0

; Servo control bit definitions (active high)
#define  SRV1IN    srvCtrl,0

#define INPMASK    B'00000001' ; Mask to isolate servo control input bits

; Drive shutoff option input port bit definition (active high)
#define  DRVOFFINP PORTC,5

; Drive enabled bit definitions (active high), held in "sysFlags"
#define  SRV1EN    0

#define  DRVONMASK B'00000001' ; Mask to isolate drive enabled bits

; Servo control output port bit definitions (active high)
#define  SRV1OUT   OUTPORT,0

; Indicator output port bit definitions (active high)
#define  IND1OUT   PORTA,1
#define  IND2OUT   PORTC,1
#define  IND3OUT   PORTC,2
#define  IND4OUT   PORTC,3
#define  IND5OUT   PORTA,4
#define  IND6OUT   PORTA,5
#define  IND7OUT   PORTC,4

#define  OUTMASK   B'00000001' ; Mask to isolate drive enabled bits

; Servo extended travel selected bit definitions
#define  SRV1XTND  srvCtrl,2

#define  XTNDMASK  B'11001100' ; Mask to isolate extended travel selected bits

; Serial RX input port bit definition
#define  SERRXIN   PORTA,2

RS232BITS   EQU    B'01111111' ; RS232 8 data bits (right shifted into carry)

; Timing constants
;**********************************************************************

; RS232 delay in instruction cycles for 9K6 baud at 1MHz clock
RXBITTIME   EQU    104         ; Delay count for 1 serial bit
RXSTARTTIME EQU    156         ; Delay count for 1.5 serial bits

TIMEFREEZE  EQU    100         ; Number of cycles for setting mode timeout
TIMEDRIVE   EQU    30          ; Number of cycles till drive shutoff (1.2 Sec)
                               ; N.B. Must be less than 32 as timer is also
                               ; low byte of current position of which upper
                               ; three bits are used for pulse duration timing.

; Interrupt interval for servo pulse cycle start, 19.968mSec (156 @ 7,812.5Hz)
CYCLEINT    EQU    (0xFF - 156)

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

SRVOFFST    EQU    31          ; Initial state in "off" movement sequence
SRVOFFEND   EQU    3           ; Initial state in "off" movement drive shutoff
SRVONST     EQU    63          ; Initial state in "on" movement sequence
SRVONEND    EQU    35          ; Initial state in "on" movement drive shutoff
SRVMVMASK   EQU    B'00011100' ; Mask to test if movement sequence completed
SRVONSTBIT  EQU    5           ; Bit indicates "on" or "off" movement sequence
SRVLUMASK   EQU    B'00000111' ; Mask to isolate movement setting offset index
SRVLUDIR    EQU    4           ; "on" or "off" movement bit after right shift

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

; Received data test masks
;**********************************************************************

ASCIIBIT    EQU    7        ; Bit should be clear in any ASCII character
DIGITMASK   EQU    0xF0     ; Mask out lower nibble of ASCII digit character
DIGITTEST   EQU    '0'      ; Test pattern for ASCII digit after applying mask

; Command characters
;**********************************************************************

RESETCMND   EQU    '#'      ; Reset settings from EEPROM
RUNCMND     EQU    '$'      ; Exit setting mode
STORECMND   EQU    '@'      ; Store current settings to EEPROM
COMMANDBASE EQU    'A'      ; Base character for first setting value


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
                            ;  bits 4..1 - Unused
                            ;  bit 0 - Servo 1 drive enabled

srvCtrl                     ; Servo control flags (all active high)
                            ;  bits 7..3 - Unused
                            ;  bit 2 - Servo 1 extended travel selected
                            ;  bit 1 - Unused
                            ;  bit 0 - Servo 1 control input (inverse of port)

; Servo 1 position and rate settings
;**********************************************************************

srv1Off                     ; Off position
srv1Off1                    ; Off position first bounce
srv1Off2                    ; Off position second bounce
srv1Off3                    ; Off position third bounce
srv1On                      ; On position
srv1On1                     ; On position first bounce
srv1On2                     ; On position second bounce
srv1On3                     ; On position third bounce
srv1OffRate                 ; Off rate
srv1OnRate                  ; On rate

; Servo 2 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

ind2LoWtr                   ; Indicator 2 bandpass lower bound
ind1LoWtr                   ; Indicator 1 bandpass lower bound
ind4LoWtr                   ; Indicator 4 bandpass lower bound
ind6LoWtr                   ; Indicator 6 bandpass lower bound
ind2HiWtr                   ; Indicator 2 bandpass upper bound
ind1HiWtr                   ; Indicator 1 bandpass upper bound
ind4HiWtr                   ; Indicator 4 bandpass upper bound
ind6HiWtr                   ; Indicator 6 bandpass upper bound
srv2OffRate                 ; Unused
srv2OnRate                  ; Unused

; Servo 3 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

ind3LoWtr                   ; Indicator 3 bandpass lower bound
ind7LoWtr                   ; Indicator 7 bandpass lower bound
srv3Off2                    ; Unused
srv3Off3                    ; Unused
ind3HiWtr                   ; Indicator 3 bandpass upper bound
ind7HiWtr                   ; Indicator 7 bandpass upper bound
srv3On2                     ; Unused
srv3On3                     ; Unused
srv3OffRate                 ; Unused
srv3OnRate                  ; Unused

; Servo 4 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

ind5LoWtr                   ; Indicator 5 bandpass lower bound
srv4Off1                    ; Unused
srv4Off2                    ; Unused
srv4Off3                    ; Unused
ind5HiWtr                   ; Indicator 5 bandpass upper bound
srv4On1                     ; Unused
srv4On2                     ; Unused
srv4On3                     ; Unused
srv4OffRate                 ; Unused
srv4OnRate                  ; Unused

; Servo current positions
;**********************************************************************

srv1NowL                    ; Servo 1 current position low byte
                            ; (also used for drive shutoff timer)
srv1NowH                    ; Servo 1 current position high byte

; Servo movement sequence states
;**********************************************************************

srv1State                   ; Servo 1 movement state

    ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

	ORG     EESTART         ; EEPROM data area

eeDataStart

; Servo extended travel selections
;**********************************************************************

    DE      0

; Servo 1 position and rate settings
;**********************************************************************

    DE      (MIDPOINT - 20) ; Off position
    DE      (MIDPOINT -  5) ; Off position first bounce
    DE      (MIDPOINT - 10) ; Off position second bounce
    DE      (MIDPOINT - 15) ; Off position third bounce
    DE      MIDPOINT        ; On position
    DE      (MIDPOINT - 15) ; On position first bounce
    DE      (MIDPOINT - 10) ; On position second bounce
    DE      (MIDPOINT -  5) ; On position third bounce
    DE      16              ; Off rate
    DE      16              ; On rate

; Servo 2 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

    DE      (MIDPOINT - 15) ; Indicator 2 bandpass lower bound
    DE      (MIDPOINT - 20) ; Indicator 1 bandpass lower bound
    DE      MIDPOINT        ; Indicator 4 bandpass lower bound
    DE      (MIDPOINT - 10) ; Indicator 6 bandpass lower bound
    DE      (MIDPOINT - 15) ; Indicator 2 bandpass upper bound
    DE      (MIDPOINT - 20) ; Indicator 1 bandpass upper bound
    DE      MIDPOINT        ; Indicator 4 bandpass upper bound
    DE      (MIDPOINT - 10) ; Indicator 6 bandpass upper bound
    DE      16              ; Unused
    DE      16              ; Unused

; Servo 3 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

    DE      (MIDPOINT -  5) ; Indicator 3 bandpass lower bound
    DE      (MIDPOINT -  3) ; Indicator 7 bandpass lower bound
    DE      (MIDPOINT - 10) ; Unused
    DE      (MIDPOINT - 15) ; Unused
    DE      (MIDPOINT -  5) ; Indicator 3 bandpass upper bound
    DE      (MIDPOINT -  3) ; Indicator 7 bandpass upper bound
    DE      (MIDPOINT - 10) ; Unused
    DE      (MIDPOINT -  5) ; Unused
    DE      16              ; Unused
    DE      16              ; Unused

; Servo 4 position and rate settings, repurposed for indicator pass band bounds
;**********************************************************************

    DE      (MIDPOINT - 17) ; Indicator 5 bandpass lower bound
    DE      (MIDPOINT -  5) ; Unused
    DE      (MIDPOINT - 10) ; Unused
    DE      (MIDPOINT - 15) ; Unused
    DE      (MIDPOINT - 17) ; Indicator 5 bandpass upper bound
    DE      (MIDPOINT - 15) ; Unused
    DE      (MIDPOINT - 10) ; Unused
    DE      (MIDPOINT -  5) ; Unused
    DE      16              ; Unused
    DE      16              ; Unused

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
    movlw   ~OUTMASK
    andwf   OUTPORT,F       ; Turn off all servo control outputs

    bcf     T1CON,TMR1ON    ; Ensure timer1 not running

    bcf     PIR1,TMR1IF     ; Clear the timer1 interrupt bitflag

skipPulse
    incf    cycleState,F    ; Advance to next state

    SetPCLATH cycleStateTable
    movf    cycleState,W    ; Use cycle state value ...
    addwf   PCL,F           ; ... as index for code jump

cycleStateTable
    goto    endISR
    goto    srv1Pulse
    goto    cycleEnd

#if (high cycleStateTable) != (high $)
    error "Interrupt cycle state jump table spans 8 bit boundary"
#endif

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
    subwf   TMR1H,F         ; Add pulse start time to low byte of timer1

    goto    endPulse

    ; Produce a pulse between 1 and 2 mSec
    ;******************************************************************

nrmlPulse
    iorlw   B'00111111'     ; Mask all but duration low byte top two bits ...
    movwf   TMR1H           ; ... and save in high byte of timer1

    ; Subtract fixed amount, adjusted for cycles in interrupt from on to off,
    ; to lengthen pulse by 1 mSec (after multiplication by 4 below)
    movlw   (PULSEINT - 19)
    subwf   TMR1L,F         ; Add pulse start time to low byte of timer1

    ; Duration of position part of pulse x 4 forms first ten bits of timer1
    ; Puts upper two bits of position duration low byte into timer
    ; Also adjust timer1 high byte for any borrow from low byte subtraction
    ; Results in a pulse length between 1.004 and 2.024 mSecs
    rlf     TMR1H,F         ; Rotate timer1H <- borrow, carry <- position bit 7
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 7, carry <- bit 15
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 15, carry <- bit 6
    rlf     TMR1L,F         ; Rotate timer1L <- position bit 6, carry <- bit 14
    rlf     TMR1H,F         ; Rotate timer1H <- position bit 14, carry <- bit 5

endPulse
    bsf     T1CON,TMR1ON    ; Start timer1 running

endISR
    ; Exit from interrupt service routine
    movf    pclath_isr,W    ; Retrieve copy of PCLATH register
    movwf   PCLATH          ; Restore pre-isr PCLATH register contents
    swapf   status_isr,W    ; Retrieve copy of STATUS register
    movwf   STATUS          ; Restore pre-isr STATUS register contents
    swapf   w_isr,F
    swapf   w_isr,W         ; Restore pre-isr W register contents

    retfie                  ; Return from interrupt


srv1Pulse
    bcf     RUNMAIN         ; Disable main program loop

    btfss   sysFlags,SRV1EN ; Skip if servo 1 drive is enabled ...
    goto    skipPulse       ; ... else skip pulse for servo 1

    movf    srv1NowH,W      ; Get high byte of duration for position ...
    movwf   TMR1L           ; ... and save in low byte of timer1

    movf    srv1NowL,W      ; Get low byte of duration for position

    bsf     SRV1OUT         ; Start servo 1 pulse

    btfss   SRV1XTND        ; Skip if servo 1 extended travel is selected
    goto    nrmlPulse
    goto    xtndPulse

cycleEnd
    clrf    cycleState      ; End of cycle, reset cycle state

    goto    endISR


;**********************************************************************
; Write to EEPROM subroutine                                          *
;     Address in W                                                    *
;     Value in temp2                                                  *
;**********************************************************************
writeEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitWriteEE
    btfsc   EECON1,WR       ; Skip if EEPROM write not 'in progress' ...
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
    BANKSEL REGBANK0        ; Select register page 0
    return


;**********************************************************************
; Read from EEPROM subroutine                                         *
;     Address in W                                                    *
;                                                                     *
;     Return value in W                                               *
;**********************************************************************
readEEPROM
    BANKSEL EECON1          ; Ensure correct register page is selected
waitReadEE
    btfsc   EECON1,WR       ; Skip if EEPROM write not 'in progress' ...
    goto    waitReadEE      ; ... else wait for write to complete

    movwf   EEADR           ; Set address of EEPROM location to read
    bsf     EECON1,RD       ; Set EEPROM read status
    movf    EEDATA,W

    BANKSEL REGBANK0        ; Select register page 0
    return


;**********************************************************************
; Load settings from EEPROM subroutine                                *
;**********************************************************************
loadAllSettings
    movf    srvCtrl,W       ; Save current servo control flags ...
    movwf   temp4           ; ... in temp4

    movlw   NUMSETTINGS
    movwf   temp1           ; Set index of settings to be read from EEPROM

    movlw   srv4OnRate      ; Load end address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

loadSetting
    decf    temp1,W         ; Set index into EEPROM from count
    call    readEEPROM

    movwf   INDF            ; Load setting with value read from EEPROM

    decf    FSR,F           ; Decrement to address of next setting
    decfsz  temp1,F         ; Decrement settings count, skip if zero ...
    goto    loadSetting     ; ... else loop until all settings have been read

    ; Restore current servo control flags other than restored option selections
    iorlw   XTNDMASK        ; Protect restored servo control option selections
    andwf   srvCtrl,F       ; Clear unset non option selection flags
    andlw   ~XTNDMASK       ; Isolate non option selection flags
    iorwf   srvCtrl,F       ; Set non option selection flags

    bsf     SYNCEDIND       ; Set servo settings synchronised indicator

    ; Initialise servo movement (second entry point for subroutine)
    ;******************************************************************

initialisePositions
    call    scanServoInputs

    ; Initialise servo movement positions
    ;******************************************************************

    ; Servo 1
    movf    srv1Off,W       ; Set current position as Off setting
    btfsc   SRV1IN          ; Skip if input is off ...
    movf    srv1On,W        ; ... else set current position as On setting
    movwf   srv1NowH
    clrf    srv1NowL

    ; Initialise all servo movement sequences to Off drive shutdown
    ;******************************************************************

    movlw   SRVOFFEND
    movwf   srv1State

    btfsc   SRV1IN                  ; Skip if input is off ...
    bsf     srv1State,SRVONSTBIT    ; ... else change state to On shutdown

    movlw   DRVONMASK       ; Ensure all drive outputs are enabled
    iorwf   sysFlags,F

    return


;**********************************************************************
; Macro: Delay loop                                                   *
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
; Servo control input scan                                            *
;     Servo control inputs are stored in a register so they can be    *
;     overridden when a position setting command is received          *
;**********************************************************************
scanServoInputs
    comf    INPORT,W        ; Read, inverted, physical input port
    iorlw   ~INPMASK        ; Protect control bits other than inputs
    andwf   srvCtrl,F       ; Clear inactive control input bits
    andlw   INPMASK         ; Isolate control input bits read from port
    iorwf   srvCtrl,F       ; Set active control input bits

    return


;**********************************************************************
; RS232 input subroutine                                              *
;     Receives data byte into temp3, sets RXDATAIND if successful  *
;**********************************************************************
dataSerRx
    bcf     RXDATAIND       ; Clear data byte received indicator

    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    dataSerRx       ; ... else loop seeking possible serial start bit

    movlw   RS232BITS
    movwf   temp3

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXSTARTTIME - 4)

nextSerDataBit
    bcf     STATUS,C        ; Clear carry flag in status
    btfss   SERRXIN         ; Test RX bit on serial input ...
    bsf     STATUS,C        ; ... if not set then set carry flag in status

    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    return                  ; ... otherwise abort

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry
    btfss   STATUS,C        ; Check if not got all serial data bits ...
    goto    endSerData      ; ... otherwise look for stop bit

continueSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXBITTIME - 6)
    goto    nextSerDataBit

endSerData
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles between RX reads
    DelayLoop    temp1, (RXBITTIME - 8)

    btfss   SERRXIN         ; Test for stop bit on serial input ...
    bsf     RXDATAIND       ; ... if found set data byte received indicator

    return


;**********************************************************************
; ASCII digit input subroutine                                        *
;     Receives ASCII digit into temp3, sets RXDATAIND if successful   *
;**********************************************************************
digitSerRx
    call    dataSerRx       ; Receive byte via serial input
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
; Set default indicator 1 to 4 pass bands subroutine                  *
;**********************************************************************
defaultInds

    clrf    ind1LoWtr       ; Indicator 1 lower bound default = 0
    clrf    ind4HiWtr       ; Indicator 4 upper bound ...
    decf    ind4HiWtr,F     ; ... = 255

    movf    srv1Off,W       ; Subtract Servo1 off position ...
    subwf   srv1On,W        ; ... from on position

    btfss   STATUS,C        ; Skip if off position not greater than on ...
    goto    defaultSwap     ; ... else use off for upper bound and on for lower

    movf    srv1Off,W       ; Set Servo1 off position ...
    movwf   ind1HiWtr       ; ... as indicator 1 upper bound default
    movf    srv1On,W        ; Set Servo1 on position ...
    movwf   ind4LoWtr       ; ... as indicator 4 lower bound default
    goto    default23       ; Set indicator 2 and 3 defaults

defaultSwap
    movf    srv1On,W        ; Set Servo1 on position ...
    movwf   ind1HiWtr       ; ... as indicator 1 upper bound default
    movf    srv1Off,W       ; Set Servo1 off position ...
    movwf   ind4LoWtr       ; ... as indicator 4 lower bound default

default23

    ; Calculate one third the difference between bounds 1 upper and 4 lower
    ;******************************************************************

    clrf    ind3HiWtr       ; Temporarily use to hold result
    decf    ind3HiWtr,F     ; Decrement through zero (see loop below)

    movf    ind1HiWtr,W     ; Subtract indicator 1 upper bound ...
    subwf   ind4LoWtr,W     ; ... from indicator 4 lower bound ...
    movwf   ind3LoWtr       ; ... store result as dividend

    movlw   3               ; Set 3 as divisor

defaultLoop
    incf    ind3HiWtr,F     ; Increment result (first time increments to zero)
    subwf   ind3LoWtr,F     ; Subtract 3 from difference between bounds
    btfsc   STATUS,C        ; Skip if subtraction underflowed through zero ...
    goto    defaultLoop     ; ... else continue with division

    movf    ind1HiWtr,W     ; Indicator 1 upper bound ...
    addwf   ind3HiWtr,W     ; ... plus a third of difference ...
    movwf   ind2LoWtr       ; ... as indicator 2 lower bound default ...
    movwf   ind2HiWtr       ; ... and upper bound default

    movf    ind3HiWtr,W     ; Subtract a third of difference  ...
    subwf   ind4LoWtr,W     ; ... from 4 lower bound for ...
    movwf   ind3LoWtr       ; ... indicator 3 lower bound default ...
    movwf   ind3HiWtr       ; ... and upper bound default

    return


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

    clrf    OUTPORT
    clrf    INPORT

    clrf    cycleState      ; Initialise cycle state
    movlw   DRVONMASK       ; Ensure all drive outputs are on ...
    iorwf   sysFlags,F      ; ... and other system flags clear

    ; Delay an arbitray length of time to allow the inputs to settle
    DelayLoop    temp1, 0xFF

    call    loadAllSettings ; Load all settings from EEPROM

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
    ;******************************************************************

    movf    cycleState,W    ; Test pulse output cycle state
    btfss   STATUS,Z        ; Skip if pulse output cycle not running ...
    goto    main            ; ... otherwise wait for idle

    bsf     RUNMAIN         ; Enable main program loop

    call    updateAllServos ; Update servo current positions

    ; Read servo control inputs
    ;******************************************************************

    movf    freezeTime,F    ; Test position setting mode timeout
    btfss   STATUS,Z        ; Skip if timeout not running ...
    goto    skipInputs      ; ... else skip input read

    call    scanServoInputs ; ... else read inputs

    incf    freezeTime,F    ; Compensate for following timeout decrement ...

skipInputs
    decf    freezeTime,F    ; Decrement position setting mode timeout

    ; Check for reception of serial data
    ;******************************************************************

testSerRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    main            ; ... otherwise abort

    btfsc   SERRXIN         ; Test for serial input connected ...
    goto    testSerRx       ; ... else loop looking for serial input connected

syncSerRx
    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    main            ; ... otherwise abort

    btfss   SERRXIN         ; Test for possible start bit on serial input ...
    goto    syncSerRx       ; ... else loop seeking possible serial start bit

    movlw   RS232BITS
    movwf   temp3

    ; Delay one and a half serial bits time
    ; Adjust delay value to allow for clock cycles from RX bit read to here
    DelayLoop    temp1, (RXSTARTTIME - 4)

    ; Data sequence starts with a null Synchronisation byte,
    ; eight bits of high (RS232 'space')
    ;******************************************************************

nextSerSyncBit
    btfss   SERRXIN         ; Test for sync byte bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    btfss   RUNMAIN         ; Skip if main program loop enabled ...
    goto    main            ; ... otherwise abort

    rrf     temp3,F         ; Rotate right RS232 receive byte through carry
    btfss   STATUS,C        ; Skip if not yet got all serial data bits ...
    goto    endSerSync      ; ... else look for stop bit

    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX bit read to here
    DelayLoop    temp1, (RXBITTIME - 7)
    goto    nextSerSyncBit

endSerSync
    ; Delay one serial bit time
    ; Adjust delay value to allow for clock cycles from RX read to here
    DelayLoop    temp1, (RXBITTIME - 8)

    btfsc   SERRXIN         ; Test for stop bit on serial input ...
    goto    syncSerRx       ; ... otherwise not receiving null, start again

    ; Synchronised to null byte, receive command (1 byte) and value (3 bytes)
    ;******************************************************************

    ; Receive command code
    ;******************************************************************

    call    dataSerRx       ; Receive byte via serial input
    btfss   RXDATAIND       ; Skip if received a byte ...
    goto    syncSerRx       ; ... otherwise abort

    btfsc   temp3,ASCIIBIT  ; Test received byte is an ASCII character ...
    goto    syncSerRx       ; ... otherwise abort

    movf    temp3,W         ; Save received byte ...
    movwf   temp2           ; ... as command

    ; Receive first digit of value for command, hundreds
    ;******************************************************************

    call    digitSerRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSerRx       ; ... otherwise abort

    ; Initialise command value with hundreds digit, can only be 0, 100, or 200
    clrw                    ; Assume 0
    btfsc   temp3,0         ; Skip if not 100 ...
    movlw   100             ; ... else set to 100
    btfsc   temp3,1         ; Skip if not 200 ...
    movlw   200             ; ... else set to 200
    movwf   temp4           ; Initialise command value

    ; Receive second digit of value for command, tens
    ;******************************************************************

    call    digitSerRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSerRx       ; ... otherwise abort

    ; Add tens digit to command value
    call    asciiTens
    addwf   temp4,F

    ; Receive third digit of value for command, units
    ;******************************************************************

    call    digitSerRx      ; Receive digit via serial input
    btfss   RXDATAIND       ; Skip if received a digit ...
    goto    syncSerRx       ; ... otherwise abort

    movf    temp3,W
    addwf   temp4,F         ; Add units digit to command value

    ; Decode and action the command
    ;******************************************************************

    SetPCLATH commandTable

    movlw   COMMANDBASE     ; Convert received command from ASCII ...
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

#if (high commandTable) != (high $)
    error "Received command jump table spans 8 bit boundary"
#endif

    ; Actions for position or rate setting command
    ;******************************************************************

srv1SetOffPosition
    movf    temp4,W         ; Store received value ...
    movwf   srv1Off1        ; ... as ...
    movwf   srv1Off2        ; ... servo ...
    movwf   srv1Off3        ; ... Off bounce positions (Servo4 compatabillity)

srv1SetOffOnly
    movf    temp4,W         ; Store received value ...
    movwf   srv1Off         ; ... as servo Off position
    call    defaultInds     ; Set default pass bands for indicators 1 to 4
    goto    received1OffPosition

srv1SetOff1Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1Off1        ; ... as servo Off bounce 1 position
    goto    received1OffPosition

srv1SetOff2Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1Off2        ; ... as servo Off bounce 2 position
    goto    received1OffPosition

srv1SetOff3Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1Off3        ; ... as servo Off bounce 3 position

received1OffPosition
    bcf     SRV1IN          ; Set servo input off
    movlw   SRVOFFEND       ; Initial movement state is Off drive shutdown
    goto    received1Position

srv1SetOnPosition
    movf    temp4,W         ; Store received value ...
    movwf   srv1On1         ; ... as ...
    movwf   srv1On2         ; ... servo ...
    movwf   srv1On3         ; ... On bounce positions (Servo4 compatabillity)

srv1SetOnOnly
    movf    temp4,W         ; Store received value ...
    movwf   srv1On          ; ... as servo On position
    call    defaultInds     ; Set default pass bands for indicators 1 to 4
    goto    received1OnPosition

srv1SetOn1Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1On1         ; ... as servo On bounce 1 position
    goto    received1OnPosition

srv1SetOn2Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1On2         ; ... as servo On bounce 2 position
    goto    received1OnPosition

srv1SetOn3Position
    movf    temp4,W         ; Store received value ...
    movwf   srv1On3         ; ... as servo On bounce 3 position

received1OnPosition
    bsf     SRV1IN          ; Set servo input on
    movlw   SRVONEND        ; Initial movement state is On drive shutdown

received1Position
    movwf   srv1State       ; Set movement state
    movf    temp4,W         ; Set received value ...
    movwf   srv1NowH        ; ... as current position
    clrf    srv1NowL
    bsf     sysFlags,SRV1EN ; Enable servo drive output
    goto    receivedSetting

srv1NewOffRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv1SetOffRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv1OffRate     ; ... as servo Off rate
    goto    receivedRate

srv1NewOnRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv1SetOnRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv1OnRate      ; ... as servo On rate
    goto    receivedRate

srv2SetOffPosition
    movf    temp4,W         ; Store received value as ...
    movwf   ind1LoWtr       ; ... indicator 1 bandpass lower bound
    movwf   ind4LoWtr       ; ... indicator 4 bandpass lower bound
    movwf   ind6LoWtr       ; ... indicator 6 bandpass lower bound

srv2SetOffOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind2LoWtr       ; ... indicator 2 bandpass lower bound
    goto    receivedSetting

srv2SetOff1Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind1LoWtr       ; ... indicator 1 bandpass lower bound
    goto    receivedSetting

srv2SetOff2Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind4LoWtr       ; ... indicator 4 bandpass lower bound
    goto    receivedSetting

srv2SetOff3Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind6LoWtr       ; ... indicator 6 bandpass lower bound
    goto    receivedSetting

srv2SetOnPosition
    movf    temp4,W         ; Store received value as ...
    movwf   ind1HiWtr       ; ... indicator 1 bandpass upper bound
    movwf   ind4HiWtr       ; ... indicator 4 bandpass upper bound
    movwf   ind6HiWtr       ; ... indicator 6 bandpass upper bound

srv2SetOnOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind2HiWtr       ; ... indicator 2 bandpass upper bound
    goto    receivedSetting

srv2SetOn1Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind1HiWtr       ; ... indicator 1 bandpass upper bound
    goto    receivedSetting

srv2SetOn2Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind4HiWtr       ; ... indicator 4 bandpass upper bound
    goto    receivedSetting

srv2SetOn3Position
    movf    temp4,W         ; Store received value ...
    movwf   ind6HiWtr       ; ... indicator 6 bandpass upper bound
    goto    receivedSetting

srv2NewOffRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv2SetOffRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv2OffRate     ; Unused
    goto    receivedRate

srv2NewOnRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv2SetOnRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv2OnRate      ; Unused
    goto    receivedRate

srv3SetOffPosition
    movf    temp4,W         ; Store received value as ...
    movwf   ind7LoWtr       ; ... indicator 7 bandpass lower bound
    movwf   srv3Off2        ; Unused
    movwf   srv3Off3        ; Unused

srv3SetOffOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind3LoWtr       ; ... indicator 3 bandpass lower bound
    goto    receivedSetting

srv3SetOff1Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind7LoWtr       ; ... indicator 7 bandpass lower bound
    goto    receivedSetting

srv3SetOff2Position
    movf    temp4,W         ; Store received value ...
    movwf   srv3Off2        ; Unused
    goto    receivedSetting

srv3SetOff3Position
    movf    temp4,W         ; Store received value ...
    movwf   srv3Off3        ; Unused
    goto    receivedSetting

srv3SetOnPosition
    movf    temp4,W         ; Store received value as ...
    movwf   ind7HiWtr       ; ... indicator 7 bandpass upper bound
    movwf   srv3On2         ; Unused
    movwf   srv3On3         ; Unused

srv3SetOnOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind3HiWtr       ; ... indicator 3 bandpass upper bound
    goto    receivedSetting

srv3SetOn1Position
    movf    temp4,W         ; Store received value as ...
    movwf   ind7HiWtr       ; ... indicator 7 bandpass upper bound
    goto    receivedSetting

srv3SetOn2Position
    movf    temp4,W         ; Store received value as ...
    movwf   srv3On2         ; Unused
    goto    receivedSetting

srv3SetOn3Position
    movf    temp4,W         ; Store received value as ...
    movwf   srv3On3         ; Unused
    goto    receivedSetting

srv3NewOffRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv3SetOffRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv3OffRate     ; Unused
    goto    receivedRate

srv3NewOnRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv3SetOnRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv3OnRate      ; Unused
    goto    receivedRate

srv4SetOffPosition
    movf    temp4,W         ; Store received value as ...
    movwf   srv4Off1        ; Unused
    movwf   srv4Off2        ; Unused
    movwf   srv4Off3        ; Unused

srv4SetOffOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind5LoWtr       ; ... indicator 3 bandpass lower bound
    goto    receivedSetting

srv4SetOff1Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4Off1        ; Unused
    goto    receivedSetting

srv4SetOff2Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4Off2        ; Unused
    goto    receivedSetting

srv4SetOff3Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4Off3        ; Unused
    goto    receivedSetting

srv4SetOnPosition
    movf    temp4,W         ; Store received value ...
    movwf   srv4On1         ; Unused
    movwf   srv4On2         ; Unused
    movwf   srv4On3         ; Unused

srv4SetOnOnly
    movf    temp4,W         ; Store received value as ...
    movwf   ind5HiWtr       ; ... indicator 5 bandpass lower bound
    goto    receivedSetting

srv4SetOn1Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4On1         ; Unused
    goto    receivedSetting

srv4SetOn2Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4On2         ; Unused
    goto    receivedSetting

srv4SetOn3Position
    movf    temp4,W         ; Store received value ...
    movwf   srv4On3         ; Unused
    goto    receivedSetting

srv4NewOffRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv4SetOffRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv4OffRate     ; Unused
    goto    receivedRate

srv4NewOnRate
    swapf   temp4,F         ; Negate effect of following nibble swap
srv4SetOnRate
    swapf   temp4,W         ; Store received value (x16 by nibble swap) ...
    movwf   srv4OnRate      ; Unused

    ; Common end action for rate setting command
    ;******************************************************************

receivedRate
    bcf     SYNCEDIND       ; Clear servo settings synchronised indicator

    clrf    freezeTime      ; Clear setting mode timeout (read physical inputs)

    goto    syncSerRx       ; Loop looking for more serial data

    ; Common end action for position setting command
    ;******************************************************************

receivedSetting
    bcf     SYNCEDIND       ; Clear servo settings synchronised indicator

    movlw   TIMEFREEZE      ; Set setting mode timeout (ignore physical inputs)
    movwf   freezeTime

    goto    syncSerRx       ; Loop looking for more serial data

    ; Received servo extended travel selections
    ;******************************************************************

receivedXtnd
    movf    temp4,W         ; Get received servo option selections
    iorlw   ~XTNDMASK       ; Protect servo control flags other than options
    andwf   srvCtrl,F       ; Clear deselected servo option flags
    andlw   XTNDMASK        ; Isolate received servo option selections
    iorwf   srvCtrl,F       ; Set servo option selections

    goto    syncSerRx       ; Loop looking for more serial data

    ; Actions for commands other than position or rate setting
    ;******************************************************************

receivedCommand
    movf    temp2,W         ; Test if command ...
    xorlw   RUNCMND         ; ... is to exit setting mode ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    receivedStore   ; ... otherwise test for store command

    clrf    freezeTime      ; Clear setting mode timeout (read physical inputs)
    goto    syncSerRx       ; Loop looking for more serial data

receivedStore
    movf    temp2,W         ; Test if command ...
    xorlw   STORECMND       ; ... is to store settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    receivedReset   ; ... otherwise test for reset command

    btfsc   SYNCEDIND       ; Skip if settings not synchronised with EEPROM ...
    goto    syncSerRx       ; ... else loop looking for more serial data

    ; Store settings to EEPROM
    ;******************************************************************

    movlw   NUMSETTINGS
    movwf   temp1           ; Set index of settings to write to EEPROM

    movlw   srv4OnRate      ; Load end address of servo settings ...
    movwf   FSR             ; ... into indirect addressing register

storeSetting
    movf    INDF,W          ; Get setting value ...
    movwf   temp2           ; ... and save as EEPROM write value

    decf    temp1,W         ; Next EEPROM index
    call    writeEEPROM

    decf    FSR,F           ; Decrement to address of next setting
    decfsz  temp1,F         ; Decrement settings count, skip if zero ...
    goto    storeSetting    ; ... else loop until all settings have been stored

    bsf     SYNCEDIND       ; Set servo settings synchronised indicator

    goto    syncSerRx       ; Loop looking for more serial data

receivedReset
    movf    temp2,W         ; Test if command ...
    xorlw   RESETCMND       ; ... is to reset settings ...
    btfss   STATUS,Z        ; ... if so skip ...
    goto    syncSerRx       ; ... otherwise abort

    btfss   SYNCEDIND       ; Skip if settings synchronised with EEPROM ...
    call    loadAllSettings ; ... else restore all settings from EEPROM

    goto    syncSerRx       ; Loop looking for more serial data


;**********************************************************************
; Macro: Set servo On state                                           *
;**********************************************************************
ServoOnState  macro    servoState

    movlw   SRVONST                 ; Get initial "on" movement state index
    btfss   servoState,SRVONSTBIT   ; Skip if in "on" movement sequence ...
    movwf   servoState              ; ... else start "on" movement sequence

    endm


;**********************************************************************
; Macro: Set servo Off state                                          *
;**********************************************************************
ServoOffState  macro    servoState

    movlw   SRVOFFST                ; Get initial "off" movement state index
    btfsc   servoState,SRVONSTBIT   ; Skip if in "off" movement sequence ...
    movwf   servoState              ; ... else start "off" movement sequence

    endm


;**********************************************************************
; Servo setting position offset for state lookup subroutine           *
;     Servo movement state passed in temp3                            *
;                                                                     *
;     Setting offset returned in W                                    *
;**********************************************************************
getServoSettingOffset
    SetPCLATH settingOffsetTable

    ; "Off" movement sequence states, state index decrements from 31 to 0
    ; Actual movement completed after state 4, states 3 to 0 = drive shutoff
    ; "On" movement sequence states, state index decrements from 63 to 32
    ; Actual movement completed after state 36, states 35 to 32 = drive shutoff
    ; 4 states at each position to allow time for servo to physically catch up

    rrf     temp3,F
    rrf     temp3,W
    andlw   SRVLUMASK
    addwf   PCL,F

settingOffsetTable
    retlw   (srv1Off  - srv1Off) ; State  3 -  0, timeout drive shutoff
    retlw   (srv1Off  - srv1Off) ; State  7 -  4, move back to end position
    retlw   (srv1Off3 - srv1Off) ; State 11 -  8, move to 3rd bounce
    retlw   (srv1Off  - srv1Off) ; State 15 - 12, move back to end position
    retlw   (srv1Off2 - srv1Off) ; State 19 - 16, move to 2nd bounce
    retlw   (srv1Off  - srv1Off) ; State 23 - 20, move back to end position
    retlw   (srv1Off1 - srv1Off) ; State 27 - 24, move to 1st bounce
    retlw   (srv1Off  - srv1Off) ; State 31 - 28, move to end position

#if (high settingOffsetTable) != (high $)
    error "Servo setting offset lookup table spans 8 bit boundary"
#endif


;**********************************************************************
; Servo position setting for state lookup subroutine                  *
;     Servo movement state passed in W                                *
;     Servo settings base address passed in temp1                     *
;                                                                     *
;     Servo target position returned in temp1                         *
;**********************************************************************
getServoTarget
    movwf   temp3               ; Servo movement state in temp2
    call    getServoSettingOffset
    addwf   temp1,W             ; Add servo settings base address to offset
    btfsc   temp3,SRVLUDIR      ; Skip if in Off movement sequence ...
    addlw   (srv1On - srv1Off)  ; ... else adjust setting address
    movwf   FSR                 ; Indirectly get ...
    movf    INDF,W              ; ... servo setting
    movwf   temp1               ; Servo target position in temp1
    return


;**********************************************************************
; Macro: Position indicator update                                    *
;**********************************************************************
PstnIndUpdate   macro   srvPstn, indLoWtr, indHiWtr, indPort, indBit

                local   outOfBand

    movf    indLoWtr,W      ; Subtract lower bound ...
    subwf   srvPstn,W       ; ... from current position

    btfss   STATUS,C        ; Skip if current not less than lower bound ...
    goto    outOfBand       ; ... else current outside pass band

    movf    srvPstn,W       ; Subtract current position ...
    subwf   indHiWtr,W      ; ... from upper bound

    btfsc   STATUS,C        ; Skip if current greater than upper bound ...
    bsf     indPort,indBit  ; ... else current inside pass band, indicator on

    btfss   STATUS,C        ; Skip if current not greater than upper bound ...
outOfBand
    bcf     indPort,indBit  ; ... else current outside pass band, indicator off

    endm


;**********************************************************************
; Macro: Servo current position update                                *
;     Rate (speed) passed in W                                        *
;**********************************************************************
ServoUpdate  macro    srvState, srvSettings, srvNow, SRVEN

    local   checkTimer, state3or2, loadTimer, runTimer, state1or0, endUpdate

    movwf   temp2           ; Store servo rate (speed) in temp2

    movlw   SRVMVMASK       ; Mask non movement states bits ...
    andwf   srvState,W      ; ... from servo movement state
    btfsc   STATUS,Z        ; Skip if movement not yet complete ...
    goto    checkTimer      ; ... otherwise skip movement update

    ; Set indirect addressing for servo settings and update current position
    ;******************************************************************

    bsf     sysFlags,SRVEN  ; Ensure servo drive is enabled

    movlw   srvSettings     ; Servo settings base address ...
    movwf   temp1           ; ... in temp1
    movf    srvState,W      ; Servo current state in W
    call    getServoTarget  ; Get servo target position for current state

    movlw   srvNow          ; Load servo current position address ...
    movwf   FSR             ; ... into indirect addressing register

    movf    (srvNow + 1),W  ; W servo current position high byte

    call    updateServo     ; Update servo current position

    btfsc   STATUS,Z        ; Check if current and target positions match ...
    decf    srvState,F      ; ... if so advance to next movement state
    goto    endUpdate

checkTimer
    ; Servo movement complete, if necessary perform drive shutoff
    ;******************************************************************

    btfss   srvState,1      ; Skip if state 3 or 2
    goto    state1or0

state3or2
    btfss   srvState,0      ; Skip if state 3 - load  timer, go to state 2 ...
    goto    runTimer        ; ... else state 2 - run shutoff timer

loadTimer
    movlw   TIMEDRIVE
    movwf   srvNow          ; Load drive shutoff timer
    decf    srvState,F      ; Advance to next state, 2 or 1

runTimer
    decfsz  srvNow,F        ; Decrement timer, skip if expired ...
    goto    endUpdate       ; ... else remain in same state

    btfss   srvState,0      ; Skip if state 1 - disable drive if selected ...
    goto    loadTimer       ; ... else state 2 - reload timer, go to state 1

    btfsc   DRVOFFINP       ; Skip if drive shutoff not selected ...
    bcf     sysFlags,SRVEN  ; ... else disable servo drive

    decf    srvState,F      ; Advance to state 0 - sequence complete

state1or0
    btfsc   srvState,0      ; Skip if state 0 - sequence complete ...
    goto    runTimer        ; ... else state 1 - run shutoff timer

endUpdate

    endm


;**********************************************************************
; Servo current positions update subroutine                           *
;**********************************************************************
updateAllServos

    btfsc   SRV1IN          ; Skip if input off ...
    goto    updateSrv1On    ; ... else perform servo On update

updateSrv1Off
    ServoOffState  srv1State

    movf    srv1OffRate,W
    goto    ServoUpdate1

updateSrv1On
    ServoOnState   srv1State

    movf    srv1OnRate,W

ServoUpdate1
    ServoUpdate    srv1State, srv1Off, srv1NowL, SRV1EN

    ; Update position indicator outputs

    PstnIndUpdate   srv1NowH, ind1LoWtr, ind1HiWtr, IND1OUT
    PstnIndUpdate   srv1NowH, ind2LoWtr, ind2HiWtr, IND2OUT
    PstnIndUpdate   srv1NowH, ind3LoWtr, ind3HiWtr, IND3OUT
    PstnIndUpdate   srv1NowH, ind4LoWtr, ind4HiWtr, IND4OUT
    PstnIndUpdate   srv1NowH, ind5LoWtr, ind5HiWtr, IND5OUT
    PstnIndUpdate   srv1NowH, ind6LoWtr, ind6HiWtr, IND6OUT
    PstnIndUpdate   srv1NowH, ind7LoWtr, ind7HiWtr, IND7OUT

    return


;**********************************************************************
; Servo current position update subroutine                            *
;     Target position in temp1                                        *
;     Rate in temp2                                                   *
;     Current high byte in W                                          *
;     Current position accessed via FSR (Low byte, High byte)         *
;                                                                     *
;     Return STATUS,Z set when target positions reached               *
;**********************************************************************
updateServo
    movf    temp2,F         ; Test rate ...
    btfsc   STATUS,Z        ; ... skip if not zero ...
    goto    moveImmediate   ; ... else set current position to target

    ; Test target position against current position
    ; (result used later for increment/decrement jump)
    movwf   temp3           ; Current high byte in temp3
    movf    temp1,W         ; Target position in W
    subwf   temp3,W         ; Current high byte - target position

    ; Multiply 8 bit rate by 16 to give 16 bit value
    ;  Original 8 bit rate: Rate[7-4][3-0]
    ;  Becomes 16 bit rate: temp3 = 0000Rate[7-4], W = Rate[3-0]0000
    swapf   temp2,W         ; Swap nibbles, gives times 16 but mixed up
    andlw   0x0F            ; Isolate high byte nibble ...
    movwf   temp3           ; ... and save
    swapf   temp2,W         ; Swap nibbles, gives times 16 but mixed up
    andlw   0xF0            ; Isolate low byte nibble

    btfss   STATUS,C        ; Skip if target position not greater than current
    goto    incrementServo

decrementServo
    ; Subtract rate from current position
    ;******************************************************************

    subwf   INDF,F          ; Subtract rate low from current position low

    btfss   STATUS,C        ; Check no borrow from low byte subtraction ...
    incf    temp3,F         ; ... else adjust rate high byte

    incf    FSR,F           ; Address current position high byte

    movf    temp3,W         ; Subtract rate high byte ...
    subwf   INDF,F          ; ... from current position high

    bcf     STATUS,Z        ; Clear STATUS,Z = target position not reached

    btfss   STATUS,C        ; Skip if no underflow on high byte subtraction ...
    goto    targetPassed    ; ... else limit to target position

    movf    temp1,W         ; Target position in W
    subwf   INDF,W          ; Current high byte - target position
    btfss   STATUS,C        ; Skip if target not greater than current ...
    goto    targetPassed    ; ... else limit to target position

    decf    FSR,F           ; Address current position low byte
    iorwf   INDF,W          ; STATUS,Z set if exactly at target

    return

incrementServo
    ; Add rate to current position
    ;******************************************************************

    addwf   INDF,F          ; Add rate low to current position low

    btfsc   STATUS,C        ; Check no carry from low byte addition ...
    incf    temp3,F         ; ... else adjust rate high byte

    incf    FSR,F           ; Address current position high byte

    movf    temp3,W         ; Add rate high byte ...
    addwf   INDF,F          ; ... to current position high

    btfsc   STATUS,C        ; Skip if no overflow on high byte addition ...
    goto    targetPassed    ; ... else limit to target position

    movf    temp1,W         ; Target position in W
    subwf   INDF,W          ; Current high byte - target position
    btfsc   STATUS,C        ; Skip if target greater than current ...
    goto    targetPassed    ; ... else limit to target position

    return

moveImmediate
    ; Already reached target position
    ;******************************************************************

    incf    FSR,F           ; Address current position high byte

targetPassed
    ; Limit current position to target position
    ;******************************************************************

    movf    temp1,W         ; Set target position ...
    movwf   INDF            ; ... as current position high byte
    decf    FSR,F           ; Address current position low byte
    clrf    INDF            ; Clear current position low byte

    return                  ; STATUS,Z set by preceeding clear



    end

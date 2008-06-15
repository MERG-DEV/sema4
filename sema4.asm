    processor   16F630
    title   "Servo4.asm Issue 1 Rev B   12/04/05"
;**********************************************************************************************
;
;   Quad servo driver for model railway use
;
;   
;   Issue   1   Rev B   
;               
;   Gives the pulse output for 4 conventional servos
;   Input control by on / off switches to ground
;   PIC has active pull-ups to Vdd
;   End limits and travel speed settable with a serial input
;   values stored in EEPROM     
;
;**********************************************************************************************

;   processor speed  4MHz   Uses internal clock
;   WDT not used
;   receive baud rate is 9600
;   serial string is a null, a capital letter starting at A and the number
;   from 0 to 255 as a three byte ascii.
;   working as far as two positons on each servo. Saves settings OK
;   no mid position or speed control yet    29/11/04
;   Servo3 is Servo2 but ready for speed and centre mods
;   Working with revised scheme for current position
;   Speed setting now works
;   Speed 0 is maximum, 1 is slowest. Increase number for faster. 8 is about max.
;   For mid posiition, just set with PC to 127
;   Defaults are mid on all settings, speed = 0 (max)
;   Revised for running either way.
;   Pulse width set for 1 mSec min, 2 msec max. Setting 127 is mid way
;   Tested and working 3/12/04 
;   Now Servo4. Added reset without save
;   Allows position to be set via the PC independent of switch position
;   Working OK  4/12/04
;   Rev B. Contains the code to set the oscillator to calibrated value. 12/04/05

    __CONFIG    h'3FE4' ; PUTimer on, WDT off, BOR on,  internal clock
    #include <p16f630.inc>
;
;******************************************************************
;

;
;   working registers
;




;   other defs
#DEFINE Inport  PORTA
#DEFINE Outport PORTC



RX      equ 2   ;rs232 rx





;
;   end of definitions


    CBLOCK  0x0020      ;start of data block in RAM
    State               ;state for interrupt routine
    S1_low              ;servo 1 short time
    S1_hi               ;servo 1 long time
    S1_rat1             ;servo 1 speed 1
    S1_rat2             ;servo 1 speed 2
    S2_low              ;servo 2 short time
    S2_hi               ;servo 2 long time
    S2_rat1             ;servo 2 speed 1
    S2_rat2             ;servo 2 speed 2
    S3_low              ;servo 3 short time
    S3_hi               ;servo 3 long time
    S3_rat1             ;servo 3 speed 1
    S3_rat2             ;servo 3 speed 2
    S4_low              ;servo 4 short time
    S4_hi               ;servo 4 long time
    S4_rat1             ;servo 4 speed 1
    S4_rat2             ;servo 4 speed 2
    S1_now              ;servo1 current position
    S2_now
    S3_now
    S4_now
    RS232i              ;serial input byte
    Bitcnt
    Count
    Intcnt              ;count  interrupts
    Syncnt              ;counter for sync byte
    Command             ;serial input command byte
    Num1                ;first ascii num
    Num2                ;second ascii num
    Num3                ;third ascii num
    Val                 ;binary version of number
    Temp
    Temp1
    Intemp              ;temporary storage for input
    Inrun               ;input bits set by serial
    Inflag
    EEtemp              ;temp for EE write data
    W_temp
    S_temp
    PCH_tmp
    Save
    Reset

    ENDC
;**************************************************************************
;
;   start of program
;
;*************************************************************************
;
    goto    setup       ;reset vector
;
    org 04      ;interrupt vector
    goto    intret      ;
;                             
    org 10      ;start of prog


;
;   end of setup
;
 
;*************************************************************

;   interrupt routine

intret  movwf   W_temp
        swapf   STATUS,W
        movwf   S_temp
        movf    PCLATH,W
        movwf   PCH_tmp
        bcf     STATUS,RP0      ;*
        
        bcf     INTCON,T0IF
        movf    State,W
        addwf   PCL,F
        goto    state0
        goto    state1
        goto    state2
        goto    state3
        goto    state4
        goto    state5
        goto    state6
        goto    state7
        goto    state8
        goto    state9
        goto    state10

state0  goto    back

state1  bsf     STATUS,RP0
        movlw   1
        movwf   OPTION_REG
        bcf     STATUS,RP0
        incf    State,F
        goto    back
state2  bsf     Outport,0       ;servo 1 up
        movlw   .19
        movwf   TMR0
        incf    State,F
        goto    back
state3  movf    S1_now,W
        movwf   TMR0            ;reload
        incf    State,F
        goto    back

state4  bcf     Outport,0       ;servo 1 down
        bsf     Outport,1       ;servo 2 up
        movlw   .19
        movwf   TMR0            ;reload
        incf    State,F
        goto    back

state5  movf    S2_now,W
        movwf   TMR0            ;reload
        incf    State,F
        goto    back

state6  bcf     Outport,1       ;servo 2 down
        bsf     Outport,2       ;servo 3 up
        movlw   .19
        movwf   TMR0            ;reload
        incf    State,F
        goto    back

state7  movf    S3_now,W
        movwf   TMR0            ;reload
        incf    State,F
        goto    back

state8  bcf     Outport,2       ;servo 3 down
        bsf     Outport,3       ;servo 4 up
        movlw   .19
        movwf   TMR0            ;reload
        incf    State,F
        goto    back
    
state9  movf    S4_now,W
        movwf   TMR0
        incf    State,F
        goto    back

state10 bcf     Outport,3       ;servo 4 down
        clrf    State
        goto    back



back    movf    PCH_tmp,W
        movwf   PCLATH
        swapf   S_temp,W
        movwf   STATUS
        swapf   W_temp,F
        swapf   W_temp,W
        retfie

;**********************************************************

bitsort addwf   PCL,F
        goto    onehi
        goto    onelo
        goto    do1
        goto    do1
        goto    twohi
        goto    twolo
        goto    do1
        goto    do1
        goto    trehi
        goto    trelo
        goto    do1
        goto    do1
        goto    fouhi
        goto    foulo
        goto    do1
        goto    do1

onehi   bsf     Inrun,0
        goto    do1
onelo   bcf     Inrun,0
        goto    do1
twohi   bsf     Inrun,1
        goto    do1
twolo   bcf     Inrun,1
        goto    do1
trehi   bsf     Inrun,4
        goto    do1
trelo   bcf     Inrun,4
        goto    do1
fouhi   bsf     Inrun,5
        goto    do1
foulo   bcf     Inrun,5
        goto    do1
    
;
;*************************************************************
;
;   Main  routine
;   
;
main    movf    State,F
        btfss   STATUS,Z
        goto    main
        bcf     INTCON,GIE  ;stop interrupts
        ;call   dely2
        bsf     STATUS,RP0
        movlw   6
        movwf   OPTION_REG
        bcf     STATUS,RP0
        movlw   .100
        movwf   TMR0
        incf    State,F
        btfsc   Inflag,0
        goto    scan1
        movf    Inport,W
        movwf   Intemp
scan    call    tone            ;scan   for now
        bsf     INTCON,GIE
        goto    rs1
scan1   movf    Inrun,W
        movwf   Intemp
        goto    scan    

rs1     btfss   Inport,RX
        goto    sync
        decf    State,W     ;is state other than 1?
        btfss   STATUS,Z
        goto    main        ;yes
        goto    rs1
sync    bsf     Outport,4   ;timing test, port c 4
        bcf     Outport,4
        btfsc   Inport,RX   ;look for a start bit of sync byte  
        goto    sync2
        decf    State,W     ;is state other than 1?
        btfss   STATUS,Z
        goto    main        ;yes
        goto    sync
sync2   movlw   8
        movwf   Syncnt
        movlw   .50
        movwf   Count
sync3   decfsz  Count,F
        goto    sync3
sync3a  bsf     Outport,4   ;timing test, port c 4
        bcf     Outport,4
        btfss   Inport,RX
        goto    sync
        decfsz  Syncnt,F
        goto    sync4
        goto    sync_s      ;look for stop bit
sync4   movlw   .31
        movwf   Count
sync5   decfsz  Count,F
        goto    sync5
        goto    sync3a
sync_s  movlw   .31
        movwf   Count
sync_s1 decfsz  Count,F
        goto    sync_s1
        bsf     Outport,4   ;timing test, port c 4
        bcf     Outport,4
        btfsc   Inport,RX
        goto    sync        ;no stop bit so try again
sync_e  nop     ;bsf        Outport,5   ;timing test, port c 5
        nop     ;bcf        Outport,5
        decf    State,W     ;is state other than 1?
        btfss   STATUS,Z
        goto    main        ;yes
        call    serbyt
        movf    RS232i,W
        movwf   Command
        call    serbyt
        movf    RS232i,W
        movwf   Num1
        call    serbyt
        movf    RS232i,W
        movwf   Num2
        call    serbyt
        movf    RS232i,W
        movwf   Num3
        movlw   0x30
        subwf   Num3,W
        movwf   Val
        movlw   0x30
        subwf   Num2,W
        movwf   Count
        incf    Count,F
        movlw   .10
tens    decfsz  Count,F
        goto    tenadd
        goto    hun
tenadd  addwf   Val,F
        goto    tens
hun     movlw   0x30
        subwf   Num1,W
        movwf   Count
        incf    Count,F
        movlw   .100
huns    decfsz  Count,F
        goto    hunadd
        goto    numdun
hunadd  addwf   Val,F
        goto    huns
numdun  movf    Val,W
    
do_cmd  movlw   S1_low
        movwf   FSR
        movlw   0x41
        subwf   Command,W
        btfss   STATUS,C
        goto    store
        movwf   Temp        ;hold command number
    
        addwf   FSR,F
        movf    Val,W
        movwf   INDF
        movf    Temp,W
        goto    bitsort

do1     bsf     Inflag,0        
        clrf    Save
        clrf    Reset
        goto    rs1

store   movf    Command,W
        sublw   "@"
        btfss   STATUS,Z
        goto    reset
        
        btfsc   Save,0
        goto    main
    ;   bsf     Outport,5
    ;   bcf     Outport,5
        movlw   .16
        movwf   Count
        clrf    Temp
        movlw   S1_low
        movwf   FSR
store1  movf    INDF,W
        movwf   EEtemp
        movf    Temp,W
        call    eewrite
        incf    Temp,F
        incf    FSR,F
        decfsz  Count,F
        goto    store1
        bsf     Save,0          ;do once
        bcf     Inflag,0
        goto    main

reset   movf    Command,W
        sublw   0x23
        btfss   STATUS,Z
        goto    main
        
        movf    Reset,F
        btfss   STATUS,Z
        goto    main    
        bsf     Reset,0
        bcf     Inflag,0
        goto    load1

;*******************************************

setup   movlw   0xFF
        bsf     STATUS,RP0      ;page 1
        movwf   TRISA           ;port a all inputs
        movlw   B'11111011'     ;no pullup on serial input
        movwf   WPUA            ;enable pullups on port a
        clrf    TRISC           ;port c all outputs
        movlw   1
        movwf   OPTION_REG      ;weak pullups, TMR0 int clock / 4
        call    0x3FF           ;get calibration value
        movwf   OSCCAL          ;put in
        bcf STATUS,RP0      ;page 0
        movlw   7
        movwf   CMCON           ;turn comparator off
        clrf    PORTC           ;all low
        
        clrf    Save
        clrf    Reset
load1   movlw   S1_low
        movwf   FSR
        clrf    Count
        movlw   .127
        movwf   S1_now
        movwf   S2_now
        movwf   S3_now
        movwf   S4_now
load    movf    Count,W
        call    eeread
        movwf   INDF
        incf    FSR,F
        incf    Count,F
        movlw   .16
        subwf   Count,W
        btfss   STATUS,Z
        goto    load
        bcf     Inflag,0
        
        call    one             ;set start pos

start   movlw   .20
        movwf   Intcnt          ;20 full counts of Timer 0
        bsf     INTCON,T0IE
        clrf    State
        goto    main
;
;*********************************************************  
;   Subroutines

;**********************************************************

;   RS232 input routine
;   here if a start bit RA2
;   gets a complete byte 

serbyt  movlw   8
        movwf   Bitcnt 
stbit   bsf     Outport,4   ;timing test, port c 4
        bcf     Outport,4
        btfsc   Inport,RX   ;look for a start bit of sync byte  
        goto    stbit2
        decf    State,W     ;is state other than 1?
        btfss   STATUS,Z
        goto    main        ;yes
        goto    stbit   

stbit2  movlw   .50     ;delay till half of 1st bit *
        call    dely
tstbt   bsf     Outport,4   ;timing test, port c 4
        bcf     Outport,4
        btfss   Inport,RX
        goto    onebit
        bcf     STATUS,C
        goto    rsrol1
onebit  bsf STATUS,C
        nop
rsrol1  rrf RS232i,F    ;roll in data 
        decfsz  Bitcnt,F
        goto    rsnxt
        movlw   .28     ;31  * 
        call    dely
sbbit   btfsc   Inport,RX   ;stop bit lo?   
        goto    sbbit

        return          ;OK

rsnxt   movlw   .28     ;30
        call    dely
        
        
        goto    tstbt

;**********************************************************

;   Read EEPROM,  address in W, result returned in W

eeread  bsf     STATUS,RP0
        movwf   EEADR
        bsf     EECON1,RD
        movf    EEDATA,W
        bcf     STATUS,RP0
        return

;write to EEPROM,   address in W, data in EEtemp

eewrite bsf     STATUS,RP0
        movwf   EEADR
        movf    EEtemp,W
        movwf   EEDATA
        bsf     EECON1,WREN
        bcf     INTCON,GIE          ;*
        movlw   0x55
        movwf   EECON2
        movlw   0xAA
        movwf   EECON2
        bsf     EECON1,WR
        bsf     INTCON,GIE          ;*
        bcf     STATUS,RP0
eedone  btfss   PIR1,EEIF           ;wait for complete
        goto    eedone
        bcf     PIR1,EEIF
        return

;**********************************************************

one     btfss   Inport,0
        goto    one_low
        movf    S1_low,W
        movwf   S1_now
        goto    two
one_low movf    S1_hi,W
        movwf   S1_now
two     btfss   Inport,1
        goto    two_low
        movf    S2_low,W
        movwf   S2_now
        goto    tre
two_low movf    S2_hi,W
        movwf   S2_now
tre     btfss   Inport,4
        goto    tre_low
        movf    S3_low,W
        movwf   S3_now
        goto    fou
tre_low movf    S3_hi,W
        movwf   S3_now
fou     btfss   Inport,5
        goto    fou_low
        movf    S4_low,W
        movwf   S4_now
        return
fou_low movf    S4_hi,W
        movwf   S4_now
        return

tone    btfss   Intemp,0
        goto    tne_low
        movf    S1_rat1,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_1_lo
        movf    S1_now,W
        subwf   S1_low,W        ;got there?
        btfsc   STATUS,Z
        goto    ttwo    
        btfss   STATUS,C
        goto    one_r1
        movf    S1_rat1,W
        movwf   Count
inc_1h  incf    S1_now,F
        movf    S1_now,W
        subwf   S1_low,W
        btfsc   STATUS,Z
        goto    ttwo
        decfsz  Count,F
        goto    inc_1h
        goto    ttwo
one_r1  movf    S1_rat1,W
        movwf   Count
dec_1h  decf    S1_now,F
        movf    S1_now,W
        subwf   S1_low,W
        btfsc   STATUS,Z
        goto    ttwo
        decfsz  Count,F
        goto    dec_1h
        goto    ttwo

no_1_lo movf    S1_low,W
        movwf   S1_now
        goto    ttwo

tne_low movf    S1_rat2,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_1_hi
        movf    S1_now,W
        subwf   S1_hi,W     ;got there?
        btfsc   STATUS,Z
        goto    ttwo    
        btfss   STATUS,C
        goto    one_r2
        movf    S1_rat2,W
        movwf   Count
inc_1l  incf    S1_now,F
        movf    S1_now,W
        subwf   S1_hi,W
        btfsc   STATUS,Z
        goto    ttwo
        decfsz  Count,F
        goto    inc_1l
        goto    ttwo
one_r2  movf    S1_rat2,W
        movwf   Count
dec_1l  decf    S1_now,F
        movf    S1_now,W
        subwf   S1_hi,W
        btfsc   STATUS,Z
        goto    ttwo
        decfsz  Count,F
        goto    dec_1l
        goto    ttwo
        
no_1_hi movf    S1_hi,W
        movwf   S1_now

ttwo    btfss   Intemp,1
        goto    tto_low
        movf    S2_rat1,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_2_lo
        movf    S2_now,W
        subwf   S2_low,W        ;got there?
        btfsc   STATUS,Z
        goto    ttre    
        btfss   STATUS,C
        goto    two_r1
        movf    S2_rat1,W
        movwf   Count
inc_2h  incf    S2_now,F
        movf    S1_now,W
        subwf   S2_low,W
        btfsc   STATUS,Z
        goto    ttre
        decfsz  Count,F
        goto    inc_2h
        goto    ttre
two_r1  movf    S2_rat1,W
        movwf   Count
dec_2h  decf    S2_now,F
        movf    S2_now,W
        subwf   S2_low,W
        btfsc   STATUS,Z
        goto    ttre
        decfsz  Count,F
        goto    dec_2h
        goto    ttre
    

no_2_lo movf    S2_low,W
        movwf   S2_now
        goto    ttre

tto_low movf    S2_rat2,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_2_hi
        movf    S2_now,W
        subwf   S2_hi,W     ;got there?
        btfsc   STATUS,Z
        goto    ttre    
        btfss   STATUS,C
        goto    two_r2
        movf    S2_rat2,W
        movwf   Count
inc_2l  incf    S2_now,F
        movf    S2_now,W
        subwf   S2_hi,W
        btfsc   STATUS,Z
        goto    ttre
        decfsz  Count,F
        goto    inc_2l
        goto    ttre
two_r2  movf    S2_rat2,W
        movwf   Count
dec_2l  decf    S2_now,F
        movf    S2_now,W
        subwf   S2_hi,W
        btfsc   STATUS,Z
        goto    ttre
        decfsz  Count,F
        goto    dec_2l
        goto    ttre

no_2_hi movf    S2_hi,W
        movwf   S2_now

        
ttre    btfss   Intemp,4
        goto    tte_low
        movf    S3_rat1,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_3_lo
        movf    S3_now,W
        subwf   S3_low,W        ;got there?
        btfsc   STATUS,Z
        goto    tfou    
        btfss   STATUS,C
        goto    tre_r1
        movf    S3_rat1,W
        movwf   Count
inc_3h  incf    S3_now,F
        movf    S3_now,W
        subwf   S3_low,W
        btfsc   STATUS,Z
        goto    tfou
        decfsz  Count,F
        goto    inc_3h
        goto    tfou
tre_r1  movf    S3_rat1,W
        movwf   Count
dec_3h  decf    S3_now,F
        movf    S3_now,W
        subwf   S3_low,W
        btfsc   STATUS,Z
        goto    tfou
        decfsz  Count,F
        goto    dec_3h
        goto    tfou
        

no_3_lo movf    S3_low,W
        movwf   S3_now
        goto    tfou

tte_low movf    S3_rat2,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_3_hi
        movf    S3_now,W
        subwf   S3_hi,W     ;got there?
        btfsc   STATUS,Z
        goto    tfou    
        btfss   STATUS,C
        goto    tre_r2
        movf    S3_rat2,W
        movwf   Count
inc_3l  incf    S3_now,F
        movf    S3_now,W
        subwf   S3_hi,W
        btfsc   STATUS,Z
        goto    tfou
        decfsz  Count,F
        goto    inc_3l
        goto    tfou
tre_r2  movf    S3_rat2,W
        movwf   Count
dec_3l  decf    S3_now,F
        movf    S3_now,W
        subwf   S3_hi,W
        btfsc   STATUS,Z
        goto    tfou
        decfsz  Count,F
        goto    dec_3l
        goto    tfou

    

no_3_hi movf    S3_hi,W
        movwf   S3_now      

        
tfou    btfss   Intemp,5
        goto    tfu_low
        movf    S4_rat1,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_4_lo
        movf    S4_now,W
        subwf   S4_low,W        ;got there?
        btfsc   STATUS,Z
        return  
        btfss   STATUS,C
        goto    fou_r1
        movf    S4_rat1,W
        movwf   Count
inc_4h  incf    S4_now,F
        movf    S4_now,W
        subwf   S4_low,W
        btfsc   STATUS,Z
        return
        decfsz  Count,F
        goto    inc_4h
        return
fou_r1  movf    S4_rat1,W
        movwf   Count
dec_4h  decf    S4_now,F
        movf    S4_now,W
        subwf   S4_low,W
        btfsc   STATUS,Z
        return
        decfsz  Count,F
        goto    dec_4h
        return


        

no_4_lo movf    S4_low,W
        movwf   S4_now
        return

tfu_low movf    S4_rat2,F       ;max speed?
        btfsc   STATUS,Z
        goto    no_4_hi
        movf    S4_now,W
        subwf   S4_hi,W     ;got there?
        btfsc   STATUS,Z
        return
        btfss   STATUS,C
        goto    fou_r2
        movf    S4_rat2,W
        movwf   Count
inc_4l  incf    S4_now,F
        movf    S4_now,W
        subwf   S4_hi,W
        btfsc   STATUS,Z
        return
        decfsz  Count,F
        goto    inc_4l
        return
fou_r2  movf    S4_rat2,W
        movwf   Count
dec_4l  decf    S4_now,F
        movf    S4_now,W
        subwf   S4_hi,W
        btfsc   STATUS,Z
        return
        decfsz  Count,F
        goto    dec_4l
        return

no_4_hi movf    S4_hi,W
        movwf   S4_now

        return





;   delay
;
dely    movwf   Count
del1    decfsz  Count,F
    goto    del1
    return 
dely1   movwf   Count
dely1a  nop
    nop
    decfsz  Count,F
    goto    dely1a
    return 

dely2   movlw .15
    movwf   Temp
dely2a  movlw   .255
    call    dely1
    decfsz  Temp,F
    goto    dely2a
    return

dely3   clrf    Temp
dely3a  decfsz  Temp,f
    goto    dely3a
    return

;********************************************************************

;   Start of EEPROM

    org 0x2100      ;default to centre position, full speed
    de      .127
    de      .127
    de      .0
    de      .0
    de      .127
    de      .127
    de      .0
    de      .0
    de      .127
    de      .127
    de      .0
    de      .0
    de      .127
    de      .127
    de      .0
    de      .0
    end

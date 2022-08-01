//R2D2 control for R2Duino VBS 2022
//Copyright (c) 2022 djulien@thejuliens.net

//usage:
//1. open and compile with Arduino IDE
//to see preprocessor output, extract "Compiling sketch..." command line from "Copy error messages"
// then add "-E" and look at output file
//2. upload to R2Duino; no longer need to reboot + remove BT < upload (no more serial port conflict)
//3. upload WAV/OGG files to SoundFX, file names as follows:
//   first char = group (sequentially selected via BT remote)
//   second char = sort by (if > 1 in group), "L" or "!" for latch on/off, "R" or "?" for random order

//history:
//version date      description
//  1.0   7/22/22   first version (Beachpoint VBS 2022): BT motor and head control
//  1.1   7/24/22   use SoftwareSerial lib to allow BT HC06 *and* SoundFX when USB is connected
//  1.2   7/31/22   add SoundFX, BT sound menu (in progress), DEBUG detail level

//TODO: replace HC-06 with BT HID
//TODO: cli app bigger buttons, more space between, and/or 2-button UI
//TODO: PIR -> headrot + random talk
//TODO: redo BT/IR command menu
//--TODO: audio (add Arduino -> SoundFX -> Speaker); Arduino-based sound not good enough
//TODO: L/R alignment/calibration
//TODO: BT connect timeout
//TODO: partial head rot?
//TODO: obstacle halt
//TODO: random action on timeout
//TODO: line/object follow
//TODO: use easing for motors?
//TODO: add servos for body tools, head tools
//TODO: add LEDs to body + head

//config options:
#define WANT_EASE //comment out to bypass servo easing
#define DEBUG_LEVEL  10+10 //comment out to remove debug
//#define RAND_HEADROT  random((int)8e3, (int)56e3) //(int)10e3 //comment out for no periodic head rot
#define WANT_STATIONARY  //comment out for motion


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// config
//

//FTDI: GND (blk), CTS# (brn), VCC (red), TXD (org), RXD (yel), RTS# (grn)
//SoundFX: Vin, Gnd, (BUS), TX, RX, UG, (CS), Gnd, R, L, Gnd
//FTDI -> SoundFX (test):
//https://learn.adafruit.com/adafruit-audio-fx-sound-board/serial-audio-control
//+5 -> +5
//GND -> GND, UG (to enable UART)
//TX -> RX
//RX -> TX
//putty /dev/ttyUSB0 @9600


//pin ass'ts:
//from ~/.arduino15/packages/arduino/hardware/avr/1.8.3/variants/standard/pins_arduino.h:
//// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|.  f|28  PC5 (AI 5)
//      (D 0) PD0  2|T  h|27  PC4 (AI 4)
//      (D 1) PD1  3|R  h|26  PC3 (AI 3)
//      (D 2) PD2  4|H  S|25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|f  U|24  PC1 (AI 1)
//      (D 4) PD4  6|H  U|23  PC0 (AI 0)
//            VCC  7|.  .|22  GND
//            GND  8|.  .|21  AREF
//            PB6  9|.  .|20  AVCC
//            PB7 10|.  L|19  PB5 (D 13)
// PWM+ (D 5) PD5 11|H IR|18  PB4 (D 12)
// PWM+ (D 6) PD6 12|H LT|17  PB3 (D 11) PWM
//      (D 7) PD7 13|H LT|16  PB2 (D 10) PWM
//      (D 8) PB0 14|H LT|15  PB1 (D 9) PWM

//NOTE: built-in USB upload/output on pins D0, D1
//NOTE: built-in LED is on pin D13

//D0, D1 = upload/debug RX/TX
//D2, D4-8 = H-bridge
//D9-11 = (reserved for) line-tracking
//D12 = IR
//D13 = BT RX + built-in LED (shared)
//A0-1 = (reserved for) ultrasonic
//A2 = head rot servo
//A3-4 = (reserved for) H-bridge shadow tween
//D3, A5 = SoundFX TX/RX

int IR_RECV_PIN = 12; //TODO: IR

#define HEADROT_SERVO_PIN  A2
#define START_DEGREE_VALUE  90 //look straight ahead

//shadow pins to use H-bridge motors via PWM:
#define WHEEL_LPWM  A3 //TODO
#define WHEEL_RPWM  A4 //TODO

#define BT_RX  13 //BT rcv; NOTE: also built-in LED
#define FX_TX  3 //FX send
#define FX_RX  A5 //FX rcv (status)

#define WHEELS 0x4287 //TODO
//CAUTION: front/back motors are mounted in different directions
int pinLF = 4;           // left front motor
int pinLB = 2;           // left rear motor
int pinRF = 8;           // right front motor
int pinRB = 7;           // right rear motor
#define Lpwm_pin  5     //left H-bridge speed 
#define Rpwm_pin  6    //right H-bridge speed


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// helpers
//

//#include <Vector.h>

#include <IRremote.h>
//#define _SS_MAX_RX_BUFF  128 //64 // RX buffer size
#include <SoftwareSerial.h> //https://docs.arduino.cc/tutorials/communication/SoftwareSerialExample

#ifdef WANT_EASE
//#include <Tween.h> //https://github.com/hideakitai/Tween broken (examples have compile errors)
//#include "EasingLibrary.h" //https://andybrown.me.uk/2010/12/05/animation-on-the-arduino-with-easing-functions/
//https://github.com/hideakitai/Easing
 #define DISABLE_COMPLEX_FUNCTIONS     // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
 #define MAX_EASING_SERVOS 1
 #define ENABLE_EASE_CUBIC
 #include "ServoEasing.hpp" //https://github.com/ArminJo/ServoEasing
#else
 #include <Servo.h>
 #define ServoEasing  Servo
#endif //def WANT_EASE

//https://stackoverflow.com/questions/6707148/foreach-macro-on-macros-arguments/13459454#13459454
//from https://github.com/swansontec/map-macro
//no worky: adapted from https://forum.arduino.cc/t/serial-debug-macro/64259
//Use PROGMEM for the strings?
//#define TOSTR(x) #x
// #define DEBUG(...)  DEBUG_MORE("[", millis(), "] ", __VA_ARGS__, " @", __PRETTY_FUNCTION__, ":", __LINE__)
// #define DEBUG_FIRST(arg, ...)  (Serial.print(arg), DEBUG_MORE(__VA_ARGS__))
// #define DEBUG_MORE(...)  UPTO_10ARGS(__VA_ARGS__, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, DEBUG_FIRST, Serial.println) (__VA_ARGS__)
//#include "map.h" //only handles 4 args
//#define DEBUG2(...)  MAP_LIST(Serial.print, "[", millis(), "] ", __VA_ARGS__, " @", __PRETTY_FUNCTION__, ":", __LINE__, "\n")
#ifndef DEBUG_LEVEL
 #define DEBUG_LEVEL  -1
#endif
//use expr syntax so DEBUG() can be inserted inline in misc stmts
#define _DEBUG(level, ...)  ((level <= DEBUG_LEVEL)? _DEBUG_ARGS("[", millis(), "] ", __VA_ARGS__, " @", __PRETTY_FUNCTION__, ":", __LINE__): _DEBUG_0ARGS())
//no worky :(  #define _DEBUG_ARGS(...)  SKIP_20ARGS(, ##__VA_ARGS__, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_MULTI, _DEBUG_1ARG, _DEBUG_0ARGS) (__VA_ARGS__)
#define _DEBUG_ARGS(...)  SKIP_24ARGS(, ##__VA_ARGS__, _DEBUG_24ARGS, _DEBUG_23ARGS, _DEBUG_22ARGS, _DEBUG_21ARGS, _DEBUG_20ARGS, _DEBUG_19ARGS, _DEBUG_18ARGS, _DEBUG_17ARGS, _DEBUG_16ARGS, _DEBUG_15ARGS, _DEBUG_14ARGS, _DEBUG_13ARGS, _DEBUG_12ARGS, _DEBUG_11ARGS, _DEBUG_10ARGS, _DEBUG_9ARGS, _DEBUG_8ARGS, _DEBUG_7ARGS, _DEBUG_6ARGS, _DEBUG_5ARGS, _DEBUG_4ARGS, _DEBUG_3ARGS, _DEBUG_2ARGS, _DEBUG_1ARG, _DEBUG_0ARGS) (__VA_ARGS__)
//no worky :(  #define _DEBUG_MULTI(first, ...)  Serial.print(first); _DEBUG_ARGS(__VA_ARGS__)
//kludge: need a new macro for each level:
#define _DEBUG_24ARGS(first, ...)  (Serial.print(first), _DEBUG_23ARGS(__VA_ARGS__))
#define _DEBUG_23ARGS(first, ...)  (Serial.print(first), _DEBUG_22ARGS(__VA_ARGS__))
#define _DEBUG_22ARGS(first, ...)  (Serial.print(first), _DEBUG_21ARGS(__VA_ARGS__))
#define _DEBUG_21ARGS(first, ...)  (Serial.print(first), _DEBUG_20ARGS(__VA_ARGS__))
#define _DEBUG_20ARGS(first, ...)  (Serial.print(first), _DEBUG_19ARGS(__VA_ARGS__))
#define _DEBUG_19ARGS(first, ...)  (Serial.print(first), _DEBUG_18ARGS(__VA_ARGS__))
#define _DEBUG_18ARGS(first, ...)  (Serial.print(first), _DEBUG_17ARGS(__VA_ARGS__))
#define _DEBUG_17ARGS(first, ...)  (Serial.print(first), _DEBUG_16ARGS(__VA_ARGS__))
#define _DEBUG_16ARGS(first, ...)  (Serial.print(first), _DEBUG_15ARGS(__VA_ARGS__))
#define _DEBUG_15ARGS(first, ...)  (Serial.print(first), _DEBUG_14ARGS(__VA_ARGS__))
#define _DEBUG_14ARGS(first, ...)  (Serial.print(first), _DEBUG_13ARGS(__VA_ARGS__))
#define _DEBUG_13ARGS(first, ...)  (Serial.print(first), _DEBUG_12ARGS(__VA_ARGS__))
#define _DEBUG_12ARGS(first, ...)  (Serial.print(first), _DEBUG_11ARGS(__VA_ARGS__))
#define _DEBUG_11ARGS(first, ...)  (Serial.print(first), _DEBUG_10ARGS(__VA_ARGS__))
#define _DEBUG_10ARGS(first, ...)  (Serial.print(first), _DEBUG_9ARGS(__VA_ARGS__))
#define _DEBUG_9ARGS(first, ...)  (Serial.print(first), _DEBUG_8ARGS(__VA_ARGS__))
#define _DEBUG_8ARGS(first, ...)  (Serial.print(first), _DEBUG_7ARGS(__VA_ARGS__))
#define _DEBUG_7ARGS(first, ...)  (Serial.print(first), _DEBUG_6ARGS(__VA_ARGS__))
#define _DEBUG_6ARGS(first, ...)  (Serial.print(first), _DEBUG_5ARGS(__VA_ARGS__))
#define _DEBUG_5ARGS(first, ...)  (Serial.print(first), _DEBUG_4ARGS(__VA_ARGS__))
#define _DEBUG_4ARGS(first, ...)  (Serial.print(first), _DEBUG_3ARGS(__VA_ARGS__))
#define _DEBUG_3ARGS(first, ...)  (Serial.print(first), _DEBUG_2ARGS(__VA_ARGS__))
#define _DEBUG_2ARGS(first, ...)  (Serial.print(first), _DEBUG_1ARG(__VA_ARGS__))
#define _DEBUG_1ARG(arg)  Serial.println(arg)
#define _DEBUG_0ARGS()  0 //noop
//#ifdef WANT_DEBUG
 #define DEBUG(...)  _DEBUG(__VA_ARGS__)
//#else
// #define DEBUG(...)  //noop
//#endif //def WANT_DEBUG

//var arg macro helpers:
//from https://stackoverflow.com/questions/4054085/appending-to-va-args/9204947#9204947
#define VA_COMMA(...)  SKIP_24ARGS(, ##__VA_ARGS__, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, )
#define SKIP_24ARGS(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16, arg17, arg18, arg19, arg20, arg21, arg22, arg23, arg24, keep, ...)  keep
#define COMMA  ,

#define SIZEOF(thing)  (sizeof(thing) / sizeof((thing)[0]))
#define toupper(ch)  (((ch) >= 'a' && (ch) <= 'z')? ch - 'a' + 'A': ch)


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// state
//

//motor speeds:
#define MAX_PWM  255
#define FAST  200
#define MEDIUM  (FAST + SLOW) / 2
#define SLOW  100

#define STEP_LEN  (int)1e3 / 2 //limit in case connection slow or lost
#define SPIN_LEN  (int)1e3 / 2 //limit in case connection slow or lost
#define HEAD_LEN  (int)1e3 / 2 //limit in case connection slow or lost

//IR keyboard:
//no pattern?
#define IR_Up  0xff629d //0110 0010 1001 1101
#define IR_Right  0xffc23d //1100 0010 0011 1101
#define IR_Left  0xff22dd //0010 0010 1101 1101
#define IR_Down  0xffa857 //1010 1000 0101 0111
#define IR_OK  0xff02fd //0000 0010 1111 1101
#define IR_1  0xff6897 //0110 1000 1001 0111
#define IR_2  0xff9867 //1001 1000 0110 0111
#define IR_3  0xffb04f //1011 0000 0100 1111
#define IR_4  0xff30cf //0011 0000 1100 1111
#define IR_5  0xff18e7 //0001 1000 1110 0111
#define IR_6  0xff7a85 //0111 1010 1000 0101
#define IR_7  0xff10ef //0001 0000 1110 1111
#define IR_8  0xff38c7 //0011 1000 1100 0111
#define IR_9  0xff5aa5 //0101 1010 1010 0101
#define IR_Star  0xff42bd //0100 0010 1011 1101
#define IR_0  0xff4ab5 //0100 1010 1011 0101
#define IR_Hash  0xff52ad //0101 0010 1010 1101
#define IR_None  0xffffffff

//commands:
#define IR_Fwd      IR_Up //IR_Go 0x00ff629d
#define IR_Back    IR_Down //0x00ffa857
//#define IR_Left    0x00ff22dd
//#define IR_Right   0x00ffc23d
#define IR_Stop    IR_OK //0x00ff02fd
#define IR_ESC     IR_Hash //0x00ff52ad


//adapted from Tween custom_class example:
/*
struct Hbridge
{
    int Left, Right;
    Hbridge(): Left(0), Right(0) {};
    Hbridge(const int L, const int R): Left(L), Right(R) {};

    Hbridge operator+(const Hbridge& rhs) const { return Hbridge(Left + rhs.Left, Right + rhs.Right); }
    Hbridge operator-(const Hbridge& rhs) const { return Hbridge(Left - rhs.Left, Right - rhs.Right); }
    Hbridge operator*(const float f) const { return Hbridge(Left * f, Right * f); }
//TODO: other overloads?
};

struct State: Hbridge
{
    int head; //head
//    Hbridge wheels;
    State(): head(0) {};
    State(const int L, const int R, const int head): Hbridge(L, R), head(head) {};

    State operator+(const State& rhs) const { return State(Left + rhs.Left, Right + rhs.Right, head + rhs.head); }
    State operator-(const State& rhs) const { return State(Left - rhs.Left, Right - rhs.Right, head - rhs.head); }
    State operator*(const float f) const { return State(Left * f, Right * f, head * f); }
//TODO: other overloads?
};
*/

//state info:
//always set to same speed?
//unsigned char Lpwm_val = 0; //(FAST + SLOW) / 2; //150;
//unsigned char Rpwm_val = 0; //(FAST + SLOW) / 2; //150;
//int Car_state = 0; //current IR command
//int pos = 0;    // variable to store the servo position
//int speed = 0, target; //current, desired
//int Ltarget = 0, Rtarget = 0; //desired
//int head = 0;
//Hbridge wheels(0, 0);

//Tween::Timeline timeline;
//SineEase sine;
//State state; //current state of head + L/R wheels
struct State
{
    int head, Left, Right;
    uint32_t latest_mv, latest_vcc;
} state;
#ifndef RAND_HEADROT
 #define RAND_HEADROT  (uint32_t)1e8 //never
#endif
uint32_t next_headrot = RAND_HEADROT; //(int)1e6; //random((int)8e3, (int)56e3)

//Servo myservo;  // create servo object to control a servo
ServoEasing myservo;
// twelve servo objects can be created on most boards

IRrecv irrecv(IR_RECV_PIN);
//decode_results results;

//RX and TX can be the same according to https://forum.arduino.cc/t/using-only-tx-of-softwareserial-leaving-rx-pin-free/109623/32
SoftwareSerial btSerial(BT_RX, BT_RX); //read BT, TX not used
SoftwareSerial fxSerial(FX_RX, FX_TX); //read, write SoundFX


void blink(int count = 1)
{
    DEBUG(10, "blink ", count);
#if BUILTIN_LED != BT_RX
    for (;;)
    {
        digitalWrite(LED_BUILTIN, HIGH); //LED on (HIGH is the voltage level)
        delay((int)1e3 / 10); //msec
        digitalWrite(LED_BUILTIN, LOW); //LED off (voltage LOW)
        if (count-- < 1) return;
        delay((int)1e3 / 10); //msec
    }
#endif
}

#define lowBattery()  (readVcc() < 4950) //USB 4.85, battery ~ 5.05 - 5.15

//adapted from https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc(int pause = 0)
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
    if (pause) delay(pause); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
    state.latest_vcc = millis();
    long result = (high << 8) | low;
    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts; USB seems to be ~4.85V, battery is > 5V
}


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// sound
//

#define R2_HELLO  0
#define R2_etc  1

#define SEQ_MODE 'S' //'+'
#define RANDOM_MODE  'R' //'?'
#define LATCH_MODE  'L' //'!'

struct SoundTrack
{
//    char grp, mode;
//    uint8_t track;
//    uint8_t next_grp, next_trk;
    char name[8]; //for sort; first char = grp, second char = mode or name
    uint8_t grp_next;
};
int numgrp = 0, numtr = 0;
uint8_t grp_first[10];
SoundTrack tracks[30];
//uint8_t sorted[SIZEOF(tracks)];

void getline(char* buf, int buflen)
{
    uint32_t timeout = millis() + (int)1e3;
    for (;;)
    {
        while (fxSerial.available())
        {
            *buf = fxSerial.read();
            if (*buf == '\r') continue;
            if (*buf == '\n') { *buf = '\0'; return; }
            if (*buf == '\t') *buf = ' ';
            if (--buflen > 0) ++buf; //keep reading + discard if buf full
        }
        if (millis() > timeout) { *buf = '\0'; return; }
    }
}

//get list of sounds available
//see info in https://github.com/adafruit/Adafruit_Soundboard_library/blob/master/Adafruit_Soundboard.cpp
//track #s are in order copied to SD, starting at 0
//uint8_t sounds[10][10];
char buf[100], *bp = buf;
void flush(int len)
{
    if (len && bp < buf + sizeof(buf) - len - 1) return; //!full
    if (!len && bp == buf) return; //empty
    *bp = '\0';
    DEBUG(10, "fxlist[", numtr++, "] ", bp - buf, ":'", buf, "'");
    bp = buf;
}

void enum_sounds(void)
{
    fxSerial.println("L"); //get list of files
//    uint32_t last_io = millis();
#if 0 //no worky - truncates output (too slow?)
    numtr = 0;
    for (uint32_t lastio = millis(); millis() < lastio + (int)2e3; ) //2 sec timeout after last char
    {
        while (fxSerial.available())
        {
            char ch = fxSerial.read();
            if (ch == '\t') { flush(2); *bp++ = '\\'; *bp++ = 't'; }
            else if (ch == '\r') { flush(2); *bp++ = '\\'; *bp++ = 'r'; }
            else if (ch == '\n') { flush(2); *bp++ = '\\'; *bp++ = 'n'; }
            else if (ch < 0x20 || ch > 0x7f)
            {
                flush(8);
                sprintf(bp, "\\x%x ", ch); bp += strlen(bp);
            }
            else { flush(1); *bp++ = ch; }
            lastio = millis();
        }
        flush(2);
        *bp++ = *bp++ = '#';
        delay((int)1e3);
    }
    flush(0);
    return;
#endif
    for (uint32_t lastio = millis(); millis() < lastio + (int)2e3; ) //2 sec timeout after last char
    {
        String str = fxSerial.readString(); //8.3 filename \t file size, timeout applies
        DEBUG(10, "fxlist[", numtr++, "]: ", str.length(), ":'", str.c_str(), "'");
        if (str.length()) lastio = millis();
    }
    return;

    for (numgrp = numtr = 0;; ++numtr) //int count = 0;; ++count)
    {
        char linebuf[8+3 + 1 + 10 + 2]; //, *bp = linebuf;
//        while (fxSerial.available())
//        int ch = fxSerial.read(); //read FX charf
//        millis() < now + (int)1e3; )
//        String str = fxSerial.readString(); //8.3 filename \t file size, timeout applies
        getline(linebuf, sizeof(linebuf));
//filenames:
//first char = group (sorted)
//second char = "!" for latch, "?" for random, any else is sequential (sorted)
        if (strlen(linebuf) < 8+3) break; //eof?
        char group = linebuf[0], order = linebuf[1]; //str.charAt(0), order = str.charAt(1);
        const char* mode;
        switch (order)
        {
            case LATCH_MODE: mode = " (latch)"; break;
            case RANDOM_MODE: mode = " (random)"; break;
            default: order = SEQ_MODE; mode = " (seq)"; break;
        }
        DEBUG(10, "fx[", numtr, "]: group ", group, ", order ", order, mode, ", name+size ", strlen(linebuf), ":'", linebuf, "'");
        if (numtr >= (int)SIZEOF(tracks)) continue; //count but don't save overflow
        strncpy(tracks[numtr].name, linebuf, sizeof(tracks[numtr].name));
#if 0 //TODO
        for (int i = 0; i < numgrp; ++i)
            if (tracks[grp_first[i]].name[0] > group && numgrp < SIZEOF(grp_first)) //start new group
            {
                
            }
        if (numgrp < SIZEOF(grp_first)) //add new group
        {
            
        }
        tracks[numtr].grp = str.charAt(0);
        tracks[numtr].mode = mode_char;
//        tracks[numtr].track = numtr;
#endif
    }
    DEBUG(10, numtr, " tracks found, limit ", SIZEOF(tracks));
}

void playback(int track, int pause = 0)
{
    DEBUG(20, "playback", track, ", pause ", pause);
//TODO: send pause/cancel in case previous sound still playing?
    fxSerial.print("#");
    fxSerial.println(track);
//TODO: wait for play/done status?
    if (pause) delay(pause);
    String str = fxSerial.readString(); //"play" ... "done"
    DEBUG(10, "playback ", str.length(), ":'", str.c_str(), "'");
}

//adjust volume from 0 (silent) to 204 (loudest) in 2-increments
void volume(int updown, int pause = 0)
{
    DEBUG(20, "volume ", updown, ", pause ", pause);
#if 0
    if (updown > 0) fxSerial.println("+"); //+2
    if (updown < 0) fxSerial.println("-"); //-2
#else
    if (updown > 0) fxSerial.println("+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n+\n"); //+2
    if (updown < 0) fxSerial.println("-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n"); //-2
#endif
    if (!updown) fxSerial.println("q"); //stop; TODO: use pause ("=") instead?
    if (pause) delay(pause);
    String str = fxSerial.readString(); //5-digit # (current volume)
    DEBUG(10, "volume ", str.length(), ":'", str.c_str(), "'");
}

//TODO? pause/resume (no newline)


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// movement
//

//void update()
//{
//    int Ldelta = Ltarget - Lspeed, Rdelta = Rtarget - Rspeed;
//        Lspeed += (Ltarget - Lspeed) 
//}

void headrot(int pause = 0)
{
    static int zz = 0;
    ++zz;
    next_headrot = RAND_HEADROT;
    int start = (zz & 1)? 180: 0, finish = (zz & 1)? 0: 180;
    DEBUG(20, "headrot 90 => ", start, " => ", finish, " => 90, zz ", zz, ", busy? ", myservo.isMoving(), ", next ", next_headrot, ", pause ", pause); //, pos);
//    for (/*int*/ pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees in steps of 1 degree
//    {
//        myservo.write(pos);              // tell servo to go to position in variable 'pos'
//        delay(15);                       // waits 15ms for the servo to reach the position
//    }
//    for (/*int*/ pos = 180; pos >= 0; pos -= 1) // goes from 180 degrees to 0 degrees
//    {
//        myservo.write(pos);              // tell servo to go to position in variable 'pos'
//        delay(15);                       // waits 15ms for the servo to reach the position
//    }
//    timeline
//        .append(state, State(state.Left, state.Right, 180), (int)1e3, cb) //0 => 180 degrees
//        .append(state, State(state.Left, state.Right, 0), (int)1e3, cb); //180 => 0 degrees
//    if (!timeline.isRunning()) timeline.restart();
    while (myservo.isMoving()) blink(); //wait for prev to finish
    myservo.startEaseTo(start); //(zz & 1)? 180: 0); //, 40 /*deg per sec*/, START_UPDATE_BY_INTERRUPT); //non-blocking
    state.latest_mv = millis();
    while (myservo.isMoving()); // blink();
    myservo.startEaseTo(finish); //(zz & 1)? 0: 180); //, 40 /*deg per sec*/, START_UPDATE_BY_INTERRUPT); //non-blocking
    while (myservo.isMoving()); // blink();
    myservo.startEaseTo(90); //, 40 /*deg per sec*/, START_UPDATE_BY_INTERRUPT); //non-blocking
    if (pause) delay(pause);
}

void cb(void)
{
    bool battlow = lowBattery();
//    DEBUG("cb: L ", state.Left, ", R ", state.Right, ", head ", state.head);
//    myservo.write(state.head);              // tell servo to go to position in variable 'pos'
//CAUTION: front motors are mounted backwards
    DEBUG(20, "LF = ", (state.Left < 0)? LOW: HIGH, ", LB = ", (state.Left <= 0)? HIGH: LOW, ", RF = ", (state.Right < 0)? LOW: HIGH, ", RB = ", (state.Right <= 0)? HIGH: LOW, ", Lpwm = ", abs(state.Left), ", Rpwm = ", abs(state.Right), ", battery low? ", battlow);
#ifndef WANT_STATIONARY
    if (!battlow)
    {
        digitalWrite(pinLF, (state.Left < 0)? LOW: HIGH); 
        digitalWrite(pinLB, (state.Left <= 0)? HIGH: LOW); // making motor move towards left rear
        digitalWrite(pinRF, (state.Right < 0)? LOW: HIGH);
        digitalWrite(pinRB, (state.Right <= 0)? HIGH: LOW); // making motor move towards right rear
        analogWrite(Lpwm_pin, abs(state.Left));
        analogWrite(Rpwm_pin, abs(state.Right));
    }
#endif
    state.latest_mv = millis();
}

void Set_Speed(const char* desc, int Left, int Right, int pause = 0)
{
    DEBUG(30, desc, ": set speed L ", state.Left, " => ", Left, ", R ", state.Right, " => ", Right, ", pause ", pause);
//    analogWrite(Lpwm_pin, Left);
//    analogWrite(Rpwm_pin, Right);
//    Lpwm_val = Left;
//    Rpwm_val = Right;
//    timeline.add(motors)
//        .then(Hbridge(L, R), (int)1e3)
//        .start();
//    timeline.append(state, State(Left, Right, state.head), (int)1e3, cb);
// this is same as above (manually)
//    timeline[a].then(0, 5000);
//    timeline.update_duration();
//    timeline.append(a, 2000);  // same as timline[a].hold(2000)
//    timeline.clear();
//      timeline.add(a).init(0);
//        if (b_changed) {
//    if (!timeline.isRunning()) timeline.restart();
    int deltaL = Left - state.Left;
    int deltaR = Right - state.Right;
    if (!deltaL && !deltaR) return; //no change      
    if ((Left > 0 && state.Left < 0) || (Left < 0 && state.Left > 0) || (Right > 0 && state.Right < 0) || (Right < 0 && state.Right > 0)) //need pause
    {
        state.Left = 0; state.Right = 0; cb();
        delay((int)1e3);
    }
//    if (Left == state.Left;
    state.Left = Left; state.Right = Right; cb();
    if (pause) delay(pause);
}


#define turnL(...)  Set_Speed("turnL", +FAST, -FAST VA_COMMA(__VA_ARGS__) __VA_ARGS__)
#define turnR(...)  Set_Speed("turnR", -FAST, +FAST VA_COMMA(__VA_ARGS__) __VA_ARGS__)
#define turn(dir, ...)  (dir < 0)? turnL(__VA_ARGS__): (dir > 0)? turnR(__VA_ARGS__): stopp(__VA_ARGS__)
#define forward(...)  Set_Speed("forward", +FAST, +FAST VA_COMMA(__VA_ARGS__) __VA_ARGS__)
#define back(...)  Set_Speed("back", -FAST, -FAST VA_COMMA(__VA_ARGS__) __VA_ARGS__)
#define movee(dir, ...)  (dir < 0)? forward(__VA_ARGS__): (dir > 0)? back(__VA_ARGS__): stopp(__VA_ARGS__)
#define stopp(...)  Set_Speed("stop", 0, 0 VA_COMMA(__VA_ARGS__) __VA_ARGS__)


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// animation
//

//1-button animations:
//TODO: make user-configurable

void auto_fwd()
{
    forward((int)1e3 / 2);
//  turnL((int)1e3 / 2);
//  turnR((int)1e3 / 2);
    stopp();
    headrot((int)2e3);
    back((int)1e3 / 2);
    stopp();
}

void auto_spin()
{
    static int zz = 0;
    ++zz;
    turn(2 * (zz & 1) - 1, (int)1e3 / 2);
    stopp();
    headrot((int)2e3);
    turn(1 - 2 * (zz & 1), (int)1e3 / 2);
    stopp();
}

void self_test()
{
//    static bool tested = false;
//    if (tested) return;
//    delay((int)5e3);
//    tested = true;
    auto_fwd();
    delay((int)5e3);
    auto_spin();
    playback(R2_HELLO);
    delay((int)5e3);
/*
// timeline mode settings
   timeline.mode(Tween::Mode::ONCE);  // default
// timeline.mode(Tween::Mode::REPEAT_TL);
// timeline.mode(Tween::Mode::REPEAT_SQ);
   timeline
//      .add(head)
//      .init(0)
//      .add(wheels)
//      .init(Hbridge(0, 0));
        .add(state)
        .init(State(0, 0, 0)) //nothing moving yet
        .then(State(0, 0, 0), (int)1e3);
//        .start();
    timeline.start();
*/
}

#if 0 //hard-coded test
void test1(void)
{
  headrot((int)5e3);

  forward((int)1e3);
  back((int)1e3);
  stopp((int)5e3);

  turnL((int)1e3);
  turnR((int)1e3);
  stopp((int)5e3);
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// init
//

void motor_setup(void)
{
    DEBUG(10, "init motor control");
//motors:
    pinMode(pinLF, OUTPUT); // pin4
    pinMode(pinLB, OUTPUT); // pin2
    pinMode(pinRF, OUTPUT); // pin8
    pinMode(pinRB, OUTPUT); // pin7 
    pinMode(Lpwm_pin, OUTPUT); // pin5 (PWM) 
    pinMode(Rpwm_pin, OUTPUT); // pin6 (PWM)   
//   Set_Speed(MEDIUM, MEDIUM); //Lpwm_val, Rpwm_val);
}

void head_setup(void)
{
    if (myservo.attach(HEADROT_SERVO_PIN, START_DEGREE_VALUE) == INVALID_SERVO)
    {
        Serial.println("head servo error");
        blink(-1); //show error
    }
    myservo.setEasingType(EASE_CUBIC_IN_OUT); //default is LINEAR
    myservo.setSpeed(90); //set default speed if not passed later (deg per sec)
}

void sound_setup(void)
{
    fxSerial.begin(9600);
    fxSerial.setTimeout((int)5e3); //default is 1 sec
    enum_sounds();
    volume(-2);
}

void setup()
{ 
//#ifdef DEBUG
//https://www.arduino.cc/reference/en/language/functions/communication/serial/
//    Serial.begin(9600); //115200); //CAUTION: Bluetooth and uploads both use serial port; disconnect BT while uploading
    Serial.begin(115200); //only upload+debug use h/w serial port now
    while (!Serial); //wait for serial port to connect (native USB only) - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//    return;
//#endif
    long vcc = readVcc(5);
    DEBUG(1, "startup, vcc: ", vcc);
    sound_setup();
    delay((int)5e3); //allow power to stabilize, time to re-attach head, etc.
    motor_setup();
    head_setup();
    irrecv.enableIRIn(); // Start the receiver
//    pinMode(LED_BUILTIN, OUTPUT);
    btSerial.begin(9600); //NOTE: reuses built-in LED pin
//TODO: line following sensors
//TODO: ultrasonic sensor
//    Serial.println("Hello, world?");
    self_test();
//   stopp();  
    blink(3); //ready signal
    DEBUG(1, "ready");
}
//void loop(void) {}


/////////////////////////////////////////////////////////////////////////////////////////////////
////
/// main logic
//

//NOTE: IR Remote repeats code while button is pressed
void IR_Control(void)
{
    int recv = 0;
    decode_results results;
    while (irrecv.decode(&results)) //recevied IR command
    {
        ++recv;
        blink(1);
        unsigned long cmd = results.value;
        irrecv.resume(); //rcv next cmd (while motors/servos still running)
        switch (cmd)
        {
            case IR_Fwd: DEBUG(5, "IR fwd"); forward(STEP_LEN); break;
            case IR_Back: DEBUG(5, "IR bk"); back(STEP_LEN); break;
            case IR_Left: DEBUG(5, "IR L"); turnL(SPIN_LEN); break;
            case IR_Right: DEBUG(5, "IR R"); turnR(SPIN_LEN); break;
            case IR_Stop: DEBUG(5, "IR stop"); stopp(); break;
            case IR_1: DEBUG(5, "IR F!"); auto_fwd(); break;
            case IR_2: DEBUG(5, "IR P!"); auto_spin(); break;
    #ifdef DEBUG
            case IR_3: DEBUG(15, "IR 3 noop"); break;
            case IR_4: DEBUG(15, "IR 4 noop"); break;
            case IR_5: DEBUG(15, "IR 5 noop"); break;
            case IR_6: DEBUG(15, "IR 6 noop"); break;
            case IR_7: DEBUG(15, "IR 7 noop"); break;
            case IR_8: DEBUG(15, "IR 8 noop"); break;
            case IR_9: DEBUG(15, "IR 9 noop"); break;
            case IR_Star: DEBUG(15, "IR * noop"); break;
            case IR_0: DEBUG(15, "0 noop"); break;
    #endif
            case IR_Hash: _DEBUG(5, "IR hd"); headrot(HEAD_LEN); break;
            case IR_None: break; //delim?
            default: _DEBUG(1, "IR unknown: ", cmd); break;
        } 
    }
//    if (!Serial.available()) stopp();
    if (recv) stopp(); //stop after all commands processed
}

char getBT(void)
{
    while (!btSerial.available());
    int ch = btSerial.read(); //read BT command
    return toupper(ch);
}

//NOTE: Serial contains 64 byte buffer
void BT_Control(void)
{
    int recv = 0, track;
    while (btSerial.available()) //received command from BT uart
    {
        ++recv;
        blink(1);
        int cmd = getBT(); //read BT command
        switch (cmd)
        {
            case 'F': DEBUG(5, "BT fwd"); forward(STEP_LEN); break;
            case 'B': DEBUG(5, "BT bk"); back(STEP_LEN); break;
            case 'L': DEBUG(5, "BT L"); turnL(SPIN_LEN); break;
            case 'R': DEBUG(5, "BT R"); turnR(SPIN_LEN); break;
            case 'S': DEBUG(5, "BT stop"); stopp(); break;
            case 'H': DEBUG(5, "BT hd"); headrot(HEAD_LEN); break;
            case '1': DEBUG(5, "BT F!"); auto_fwd(); break;
            case '2': DEBUG(5, "BT P!"); auto_spin(); break;
//            case '3': DEBUG(5, "BT H!"); auto_head = spin(); break;
            case 'P': track = getBT() - '0'; DEBUG(5, "BT P#", track); playback(track); break;
//            case 'V': track = getBT(); DEBUG(5, "BT V", (track == '+')? "+": (track == '-')? "-": "="); volume((track == '+')? +1: (track == '-')? -1: 0); break;
            case 'V': track = getBT(); DEBUG(5, "BT V", (track == '+')? "+": (track == '-')? "-": "="); volume((track == '+')? 200: (track == '-')? 50: 0); break;
            default: DEBUG(1, "BT unknown: ", cmd); break;
        }
//        if (!btSerial.available()) stopp();
    }
    if (recv) stopp(); //stop after all commands processed
}

void loop() 
{
   IR_Control();
   BT_Control();

    uint32_t now = millis();
    if (now - state.latest_vcc > (int)5e3) DEBUG(5, "vcc ", readVcc());
    if (now - state.latest_mv >= next_headrot) headrot(HEAD_LEN);
}

//eof

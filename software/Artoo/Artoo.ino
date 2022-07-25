//R2D2 control for R2Duino VBS 2022
//Copyright (c) 2022 djulien@thejuliens.net

//usage:
//1. open and compile with Arduino IDE
//2. upload to R2Duino
//NOTE: need to reboot + remove BT < upload (serial port conflict)

//history:
//version date      description
//  1.0   7/22/22   first version (Beachpoint VBS 2022): BT motor and head control

//TODO: resolve BT/upload/SoundFX conflict on serial port
//TODO: cli app bigger buttons, more space between, and/or 2-button UI
//TODO: PIR -> headrot + random talk
//TODO: redo BT/IR command menu
//TODO: audio (add Arduino -> SoundFX -> Speaker); Arduino-based sound not good enough
//TODO: L/R alignment/calibration
//TODO: BT connect timeout
//TODO: partial head rot?
//TODO: obstacle halt
//TODO: random action on timeout
//TODO: line/object follow
//TODO: use easing for motors?


#define WANT_EASE //comment out to bypass servo easing
//#define WANT_DEBUG //comment out to remove debug
#define RAND_HEADROT  random((int)8e3, (int)56e3) //(int)10e3 //comment out for no periodic head rot

#include <IRremote.h>

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
#include "map.h"
#define DEBUG2(...)  MAP_LIST(Serial.print, "[", millis(), "] ", __VA_ARGS__, " @", __PRETTY_FUNCTION__, ":", __LINE__, "\n")
#ifdef WANT_DEBUG
 #define DEBUG(...)  DEBUG2(__VA_ARGS__)
#else
 #define DEBUG(...)  //noop
#endif //def WANT_DEBUG

//var arg macro helpers:
//from https://stackoverflow.com/questions/4054085/appending-to-va-args/9204947#9204947
/*
 * VA_COMMA() expands to nothing if given no arguments and a comma if
 * given 1 to 4 arguments.  Bad things happen if given more than 4
 * arguments.  Don't do it.
 */
#define VA_COMMA(...)  GET_16TH_ARG(, ##__VA_ARGS__, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, COMMA, )
#define GET_16TH_ARG(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16, ...)  arg16
#define COMMA  ,
/* EXAMPLES */
//#define MACRO(api, ...)  bool ret = api(__VA_ARGS__ VA_COMMA(__VA_ARGS__) 456)
//MACRO(foo)                       /* bool ret = foo( 456)              */
//MACRO(foo,1)                     /* bool ret = foo(1 , 456)           */
//MACRO(foo,1,2,3,4)               /* bool ret = foo(1,2,3,4 , 456)     */
/* uh oh, too many arguments: */
//MACRO(foo,1,2,3,4,5)             /* bool ret = foo(1,2,3,4,5 5 456)   */
//MACRO(foo,1,2,3,4,5,6)           /* bool ret = foo(1,2,3,4,5,6 5 456) */

#if 0 //broken
//from https://stackoverflow.com/questions/5588855/standard-alternative-to-gccs-va-args-trick
#define FIRST_ARG(...) FIRST_HELPER(__VA_ARGS__, ignored)
#define FIRST_HELPER(first, ...) first
//#define OTHER_ARGS(...) REST_HELPER(NUM_ARGS(__VA_ARGS__), __VA_ARGS__)
//#define REST_HELPER(qty, ...) REST_HELPER2(qty, __VA_ARGS__)
//#define REST_HELPER2(qty, ...) REST_HELPER_##qty(__VA_ARGS__)
//#define REST_HELPER_ONE(first)  //expand to nothing if only one arg
//#define REST_HELPER_TWOORMORE(first, ...) , __VA_ARGS__ //expand to comma followed by args if > 1 arg
//#define NUM_ARGS(...) \
//    SELECT_UPTO10(__VA_ARGS__, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE,\
//                TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, TWOORMORE, ONE, ZERO) //add more args as needed
#define SELECT_UPTO10(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, keep11, ...)  keep11

#define APPEND_ARGS(...)  SELECT_UPTO10(__VA_ARGS__, 1010 APPEND_ANY, 99 APPEND_ANY, 88 APPEND_ANY, 77 APPEND_ANY, 66 APPEND_ANY, 55 APPEND_ANY, 44 APPEND_ANY, 33 APPEND_ANY, 22 APPEND_ANY, 11 APPEND_ANY, 00 APPEND_NONE) (__VA_ARGS__)
#define APPEND_ANY(...)  , any __VA_ARGS__ //with comma
#define APPEND_NONE(...)  none __VA_ARGS__ //without comma
void test()
{
    Serial.print(FIRST_ARG()); Serial.print(APPEND_ARGS());
    Serial.print(FIRST_ARG(1)); Serial.print(APPEND_ARGS(1));
    Serial.print(FIRST_ARG(2, 2)); Serial.print(APPEND_ARGS(2, 2));
}
#endif

//pin assts:
//from pins_arduino.h:
//// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM

int RECV_PIN = 12; //IR

#define ULTRASONIC_SERVO_PIN  A2
#define START_DEGREE_VALUE  90 //look straight ahead

//shadow pins to use H-bridge motors via PWM:
//TODO: #define WHEEL_LPWM  A3
//TODO: #define WHEEL_RPWM  A4

#define WHEELS 0x4287 //TODO
//CAUTION: front/back motors are mounted in different directions
int pinLF = 4;           // left front motor
int pinLB = 2;           // left rear motor
int pinRF = 8;           // right front motor
int pinRB = 7;           // right rear motor
#define Lpwm_pin  5     //left H-bridge speed 
#define Rpwm_pin  6    //right H-bridge speed

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
    uint32_t latest;
} state;
#ifndef RAND_HEADROT
 #define RAND_HEADROT  (int)1e8 //never
#endif
uint32_t next_headrot = RAND_HEADROT; //(int)1e6; //random((int)8e3, (int)56e3)

//Servo myservo;  // create servo object to control a servo
ServoEasing myservo;
// twelve servo objects can be created on most boards

IRrecv irrecv(RECV_PIN);
//decode_results results;


void blink(int count = 1)
{
    DEBUG("blink ", count);
    for (;;)
    {
        digitalWrite(LED_BUILTIN, HIGH); //LED on (HIGH is the voltage level)
        delay((int)1e3 / 10); //msec
        digitalWrite(LED_BUILTIN, LOW); //LED off (voltage LOW)
        if (count-- < 1) return;
        delay((int)1e3 / 10); //msec
    }
}

//void update()
//{
//    int Ldelta = Ltarget - Lspeed, Rdelta = Rtarget - Rspeed;
//        Lspeed += (Ltarget - Lspeed) 
//}

void headrot(int pause = 0)
{
    static int zz = 0;
    ++zz;
    int start = (zz & 1)? 180: 0, finish = (zz & 1)? 0: 180;
    DEBUG("headrot 90 => ", start, " => ", finish, " => 90, zz ", zz, ", busy? ", myservo.isMoving(), ", pause ", pause); //, pos);
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
    state.latest = millis();
    while (myservo.isMoving()); // blink();
    myservo.startEaseTo(finish); //(zz & 1)? 0: 180); //, 40 /*deg per sec*/, START_UPDATE_BY_INTERRUPT); //non-blocking
    while (myservo.isMoving()); // blink();
    myservo.startEaseTo(90); //, 40 /*deg per sec*/, START_UPDATE_BY_INTERRUPT); //non-blocking
    next_headrot = RAND_HEADROT;
    if (pause) delay(pause);
}

void cb(void)
{
//    DEBUG("cb: L ", state.Left, ", R ", state.Right, ", head ", state.head);
//    myservo.write(state.head);              // tell servo to go to position in variable 'pos'
//CAUTION: front motors are mounted backwards
    DEBUG("LF = ", (state.Left < 0)? LOW: HIGH, ", LB = ", (state.Left <= 0)? HIGH: LOW, ", RF = ", (state.Right < 0)? LOW: HIGH, ", RB = ", (state.Right <= 0)? HIGH: LOW, ", Lpwm = ", abs(state.Left), ", Rpwm = ", abs(state.Right));
    digitalWrite(pinLF, (state.Left < 0)? LOW: HIGH); 
    digitalWrite(pinLB, (state.Left <= 0)? HIGH: LOW); // making motor move towards left rear
    digitalWrite(pinRF, (state.Right < 0)? LOW: HIGH);
    digitalWrite(pinRB, (state.Right <= 0)? HIGH: LOW); // making motor move towards right rear
    analogWrite(Lpwm_pin, abs(state.Left));
    analogWrite(Rpwm_pin, abs(state.Right));
    state.latest = millis();
}

void Set_Speed(const char* desc, int Left, int Right, int pause = 0)
{
    DEBUG(desc, ": set speed L ", state.Left, " => ", Left, ", R ", state.Right, " => ", Right, ", pause ", pause);
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


void M_Control_IO_config(void)
{
//    DEBUG("init motor control");
//motors:
    pinMode(pinLF, OUTPUT); // pin4
    pinMode(pinLB, OUTPUT); // pin2
    pinMode(pinRF, OUTPUT); // pin8
    pinMode(pinRB, OUTPUT); // pin7 
    pinMode(Lpwm_pin, OUTPUT); // pin5 (PWM) 
    pinMode(Rpwm_pin, OUTPUT); // pin6 (PWM)   
}

void setup()
{ 
    delay((int)5e3);
//#ifdef DEBUG
    Serial.begin(9600); //115200); //CAUTION: Bluetooth and uploads both use serial port; disconnect BT while uploading
//#endif
    pinMode(LED_BUILTIN, OUTPUT);
    if (myservo.attach(ULTRASONIC_SERVO_PIN, START_DEGREE_VALUE) == INVALID_SERVO) { Serial.println("servo att err"); blink(-1); }  // attaches the servo on pin 9 to the servo object
    myservo.setEasingType(EASE_CUBIC_IN_OUT); //default is LINEAR
    myservo.setSpeed(90); //deg per sec // This speed is taken if no further speed argument is given.
    M_Control_IO_config();
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
//   Set_Speed(MEDIUM, MEDIUM); //Lpwm_val, Rpwm_val);
    irrecv.enableIRIn(); // Start the receiver
//   stopp();  
    blink(3); //ready signal
}

//NOTE: IR Remote repeats code while button is pressed
void IR_Control(void)
{
//   unsigned long Key;
    decode_results results;
    int recv = 0;
    while (irrecv.decode(&results)) //return; //judging if serial port receives data   
    {
        ++recv;
        blink(1);
        unsigned long Key = results.value;
        irrecv.resume(); // Receive the next value (while motors/servos still running)
        switch (Key)
        {
            case IR_Fwd: DEBUG2("IR fwd"); forward(STEP_LEN); break;
            case IR_Back: DEBUG2("IR bk"); back(STEP_LEN); break;
            case IR_Left: DEBUG2("IR L"); turnL(SPIN_LEN); break;
            case IR_Right: DEBUG2("IR R"); turnR(SPIN_LEN); break;
            case IR_Stop: DEBUG2("IR stop"); stopp(); break;
            case IR_1: DEBUG2("IR F!"); auto_fwd(); break;
            case IR_2: DEBUG2("IR P!"); auto_spin(); break;
    #ifdef DEBUG
            case IR_3: DEBUG2("IR 3 noop"); break;
            case IR_4: DEBUG2("IR 4 noop"); break;
            case IR_5: DEBUG2("IR 5 noop"); break;
            case IR_6: DEBUG2("IR 6 noop"); break;
            case IR_7: DEBUG2("IR 7 noop"); break;
            case IR_8: DEBUG2("IR 8 noop"); break;
            case IR_9: DEBUG2("IR 9 noop"); break;
            case IR_Star: DEBUG2("IR * noop"); break;
            case IR_0: DEBUG2("0 noop"); break;
    #endif
            case IR_Hash: DEBUG2("IR hd"); headrot(HEAD_LEN); break;
            case IR_None: break; //delim?
            default: DEBUG2("IR unknown: ", Key); break;
        } 
    }
//    if (!Serial.available()) stopp();
    if (recv) stopp(); //stop after all commands processed
}

#define toupper(ch)  (((ch) >= 'a' && (ch) <= 'z')? ch - 'a' + 'A': ch)
//NOTE: Serial contains 64 byte buffer
void BT_Control(void)
{
    int recv = 0;
    while (Serial.available()) //to judge whether the serial port receives the data.
    {
        ++recv;
        blink(1);
        int Bluetooth_val = Serial.read();  //reading (Bluetooth) data of serial port,giving the value of val;
        switch (toupper(Bluetooth_val))
        {
            case 'F': DEBUG2("BT fwd"); forward(STEP_LEN); break;
            case 'B': DEBUG2("BT bk"); back(STEP_LEN); break;
            case 'L': DEBUG2("BT L"); turnL(SPIN_LEN); break;
            case 'R': DEBUG2("BT R"); turnR(SPIN_LEN); break;
            case 'S': DEBUG2("BT stop"); stopp(); break;
            case 'H': DEBUG2("BT hd"); headrot(HEAD_LEN); break;
            case '1': DEBUG2("BT F!"); auto_fwd(); break;
            case '2': DEBUG2("BT P!"); auto_spin(); break;
//            case '3': DEBUG2("BT H!"); auto_head = spin(); break;
            default: DEBUG2("BT unknown: ", Bluetooth_val); break;
        }
//        if (!Serial.available()) stopp();
    }
    if (recv) stopp(); //stop after all commands processed
}

void self_test()
{
    static bool tested = false;
    if (tested) return;
//    delay((int)5e3);
    tested = true;
    auto_fwd();
    delay((int)5e3);
    auto_spin();
    delay((int)5e3);
}

void loop() 
{
    self_test();
#if 1
   IR_Control();
   BT_Control();
#endif

//#ifdef RAND_HEADROT
//   timeline.update();
//    static uint32_t prev;
//    static uint32_t next = (int)20e3; //wait before first
    uint32_t now = millis();
//   if (now < state.latest + (int)20e3) return;
//   prev = now;
//   state.latest = now;
//   DEBUG("tween: L ", state.Left, ", R ", state.Right, ", state.head, ", state.head);
    if (now - state.latest < next_headrot) return;
    headrot(HEAD_LEN);
//    next_headrot = RAND_HEADROT;
//#endif

#if 0 //hard-coded test
  headrot();
  delay((int)5e3);

  forward();
  delay((int)1e3);
  back();
  delay((int)1e3);
  stopp();
  delay((int)5e3);

  turnL();
  delay((int)1e3);
  turnR();
  delay((int)1e3);
  stopp();
  delay((int)5e3);
  
  return;
#endif
}

//eof

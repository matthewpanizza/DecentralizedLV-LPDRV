/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mligh/OneDrive/Particle/DecentralizedLV-LPDRV/src/DecentralizeLPDRV.ino"
/*
 * Project DecentralizeLPDRV
 * Description: Code for the Decentralized Low-Voltge System Low-Power Driver Boards
 * Author: Matthew Panizza
 * Date: May 7, 2022
 */
#include "neopixel.h"   //Neopixel Library for Addressable Strips

/////////////////////////////////////////////////////////
// BOARD DEFINITION - UNCOMMENT BOARD TO BE PROGRAMMED //
/////////////////////////////////////////////////////////

//#define BDFL      //Front-Left Board
//#define BDFR      //Front-Right Board
//#define BDRL      //Rear-Left Board
void initialize(uint8_t pinNum);
void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk);
void autoCAN();
void write(bool cmp);
void dwrite(bool cmp);
void initialize(uint8_t pinNum);
void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk);
void autoCAN();
void duty(uint8_t cmp);
void initialize(uint8_t pinNum);
void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk);
void autoCAN();
void write(bool cmp);
void initialize(uint8_t pinNum, uint8_t pType);
void autoCAN();
bool pwmCANFlag();
uint8_t getCANWatchValue();
void dWrite(bool cmp);
void aWrite(uint8_t cmp);
bool dRead();
void autoCAN();
void setup();
void loop();
void animationHandler();
void animationHandler2();
void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7);
#line 16 "c:/Users/mligh/OneDrive/Particle/DecentralizedLV-LPDRV/src/DecentralizeLPDRV.ino"
#define BDRR        //Rear-Right Board

SYSTEM_MODE(SEMI_AUTOMATIC);    //Disable Wi-Fi for this system

/////////////////////////////////////////////////////////////////////////
// Neopixel Board Params - Allows Per-Board Pixel Count and Pin Config //
/////////////////////////////////////////////////////////////////////////

#ifdef BDFL
    #define PIXEL_COUNT 36
    #define PIXEL_PIN RX
#endif
#ifdef BDFR
    #define PIXEL_COUNT 36
    #define PIXEL_PIN RX
#endif
#ifdef BDRL
    #define PIXEL_COUNT 46
    #define PIXEL_PIN A2
#endif
#ifdef BDRR
    #define PIXEL_COUNT 1
    #define PIXEL_PIN A1
#endif

//////////////////////////////////////////////////////////////////////////////
// Neopixel Setup Params - Allows Control of LED Type, and if LEDs are used //
//////////////////////////////////////////////////////////////////////////////

#define USING_NEOPIXEL      //Uncomment if Neopixels are being used
#define PIXEL_TYPE WS2812B

//RGB color definitions for various LED colors
#define AMBER 255,60,0      // R = 255, G = 60, B = 0
#define WHITE 255,200,180
#define DARK 0,0,0

#define FLASH_PIXEL_WIDTH   14      //Number of pixels wide the BPS flash indicator should be
#define SOL_PUL_CYCLES      500     //Delay between solar animation pulses
#define SOL_PUL_WIDTH       1       //Number of pixels in the animation at full-brightness
#define SOL_FADE_WID        3       //Number of pixels in the animation where there is fade-out
#define SOL_FADE_OFFSET     30      //Amount of dimming that occurs for each fade step from the center
#define STRT_FADE_OFFSET    60

#ifdef USING_NEOPIXEL
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);    //Create Neopixel Strip Object
#endif

////////////////////////////
// CAN Message Formatting //
////////////////////////////

#define CAN_SNS 0x100   //CAN ID to receive main message from sensor board

CANChannel can(CAN_D1_D2);  //Create CAN object
CANMessage inputMessage;    //Input message for receiving from multiple addresses and filtering

CANMessage pinStatus; //Main transmit message for switch data
// byte 0: Right Turn PWM 0-255
// byte 1: Left Turn PWM 0-255
// byte 2: Brake PWM 0-255
// byte 3: Reverse PWM 0-255
// byte 4: b0: Headlight, b1: High-Beam, b2: Trunk-Release, b3: Driver-Fan, b4: Power-Steer-Relay, b5: reverse, b6: brake press
// byte 5: Power Mode: 0-Acc, 1-Ign, 2-LPAcc, 3-LPIgn
// byte 6: Pump Mode: 0-Off, 1-LPRun, 2-Run, 4-Boost

CANMessage replyAddr;   //Reply address for the CAN boards

bool ignition = 0;      //Ignition flag based on CAN message from sensor board

bool flashOn = true;        //Flash flag if there is a BMS fault
uint16_t faultBMS = 0;      //Flag set to 1 if the BMS was the source of the fault
uint16_t faultSwitch = 0;   //Flag set to 1 if the kill-switch was the source of the fault
uint16_t carCharge;         //Flag set to 1 if the car is in wall-charge mode
bool solarCharge;           //Flag set to 1 if the car is in solar-charge mode
bool startupHDL;            //Flag set to 1 when car is first turned on to play animations

////////////////////////////
// Pin Object Definitions //
////////////////////////////

class LPDRVPin{     //Low-Power Drive Pin - Typically Controls On-Off Mosfet Pin (5-Amp output)
    public:
    uint8_t devicePin;          //Pin number of the pin this object corresponds to
    uint8_t canByte;            //Byte of the CAN Message to monitor (0-7)
    uint8_t canBitMsk;          //Bit number in the byte to control output on-off
    CANMessage *watchID;        //Pointer to the CAN message to watch
    //Function to initialize the pin
    void initialize(uint8_t pinNum){    
        devicePin = pinNum;     //Copy in the pin number to the object
        pinMode(pinNum,OUTPUT); //Configure pin as an output
    }
    //Function to configure automatic CAN Control - select the bit and byte of the message
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk){
        watchID = wtch;         //Copy in the pointer to the message to respond to
        canByte = cbyte;        //Copy in the byte number (0-7)
        canBitMsk = cbitmsk;    //Copy in the bit number (0-7)
    }
    //Function called to automatically update the pin state based on the received CAN message
    void autoCAN(){
        digitalWrite(devicePin,(watchID->data[canByte]&(1 << canBitMsk)));  //Get data out of the CAN message
    }
    //Function to manually change the PWM value of the pin (not supported on all pins)
    void write(bool cmp){
        analogWrite(devicePin,cmp);
    }
    //Function to manually turn on or off the pin (0 or 1)
    void dwrite(bool cmp){
        digitalWrite(devicePin,cmp);
    }
};

class LPDRVPinPWM{      //Low-Power Drive Pin PWM - Typically Controls Mosfet with Variable Output (0-255)
    public:
    uint8_t devicePin;          //Pin number of the pin this object corresponds to
    uint8_t canByte;            //Byte of the CAN Message to monitor (0-7)
    uint8_t canBitMsk;          //Bit number in the byte to control output on-off
    CANMessage *watchID;        //Pointer to the CAN message to watch
    //Function to initialize the pin
    void initialize(uint8_t pinNum){    
        devicePin = pinNum;     //Copy in the pin number to the object
        pinMode(pinNum,OUTPUT); //Configure pin as an output
    }
    //Function to configure automatic CAN Control - select the bit and byte of the message
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk){
        watchID = wtch;         //Copy in the pointer to the message to respond to
        canByte = cbyte;        //Copy in the byte number (0-7)
        canBitMsk = cbitmsk;    //Copy in the bit mask (0-255)
    }
    //Function called to automatically update the pin state based on the received CAN message
    void autoCAN(){
        analogWrite(devicePin,(watchID->data[canByte])&canBitMsk);
    }
    //Function to manually change the pin output (0-255 duty cycle)
    void duty(uint8_t cmp){
        analogWrite(devicePin,cmp);
    }
};

class HPDRVPin{     //High-Power Drive Pin - Typically Controls On-Off Relay Pin (>5-Amp output)
    public:
    uint8_t devicePin;          //Pin number of the pin this object corresponds to
    uint8_t canByte;            //Byte of the CAN Message to monitor (0-7)
    uint8_t canBitMsk;          //Bit number in the byte to control output on-off
    CANMessage *watchID;        //Pointer to the CAN message to watch
    //Function to initialize the pin
    void initialize(uint8_t pinNum){    
        devicePin = pinNum;     //Copy in the pin number to the object
        pinMode(pinNum,OUTPUT); //Configure pin as an output
    }
    //Function to configure automatic CAN Control - select the bit and byte of the message
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk){
        watchID = wtch;         //Copy in the pointer to the message to respond to
        canByte = cbyte;        //Copy in the byte number (0-7)
        canBitMsk = cbitmsk;    //Copy in the bit number (0-7)
    }
    //Function called to automatically update the pin state based on the received CAN message
    void autoCAN(){
        digitalWrite(devicePin,watchID->data[canByte] & (1<<canBitMsk));
    }
    //Function to manually turn on or off the pin (0 or 1)
    void write(bool cmp){
        analogWrite(devicePin,cmp);
    }
};

class IOPin{        //Input/Output Pin - mappable to 6 different functions (LEDs, switch inputs, sensors, etc.)
    public:
    uint8_t devicePin;          //Pin number of the pin this object corresponds to
    uint8_t canByte;            //Byte of the CAN Message to monitor (0-7)
    uint8_t canBitMsk;          //Bit number in the byte to control output on-off
    CANMessage *watchID;        //Pointer to the CAN message to watch
    uint16_t *outVar;           //Pointer to a variable that gets updated from the received CAN packet
    uint8_t pinType;            //Variable to hold the IO pin type from intialization
    bool invRet;                //Flag which can invert the value in the returned variable
    //Function intitializes the pin to the correct type and copies in the pin type so autoCAN works
    void initialize(uint8_t pinNum, uint8_t pType){
        devicePin = pinNum;
        pinType = pType;
        switch (pType){
        case 0:     //Standard GPIO Pin
            pinMode(pinNum, OUTPUT);
            break;
        case 1:     //Input for Analog
            pinMode(pinNum, INPUT);
            break;
        case 2:     //Input for Digital Button
            pinMode(pinNum, INPUT_PULLDOWN);
            break;
        case 3:     //Input for Digital Button
            pinMode(pinNum, INPUT_PULLUP);
            break;
        case 4:     //PWM Out Pin
            pinMode(pinNum, OUTPUT);
            break;
        case 5:     //Neopixel
            break;
        case 6:
            pinMode(pinNum, INPUT);
            break;
        }
    }
    //Function to set which CAN message, byte, and bit to watch for.
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk, uint16_t *watchVar = nullptr, bool inv = false){
        outVar = watchVar;      //Copy in pointer to return variable
        watchID = wtch;         //Copy in pointer to CAN message
        if(cbyte > 6 && pinType == 1) return;   //Don't assign a byte in the message if it is too far away to work
        canByte = cbyte;        //Copy in byte to monitor
        canBitMsk = cbitmsk;    //Copy in bit to monitor
        invRet = inv;           //Copy in flag for inverting returned value
    }
    //Function to automatically update the hardware based on the contents of the received CAN Message
    void autoCAN(){
        uint16_t rVal;
        switch (pinType){
        case 0:     //If the pin is a standard input-output
            digitalWrite(devicePin,watchID->data[canByte]&canBitMsk);
            break;
        case 1:     //If the pin is an analog input, fill CAN message with the value to be sent out
            rVal = analogRead(devicePin);
            *outVar = rVal ^ invRet;
            watchID->data[canByte] = rVal%255;//Lower 8 bits
            watchID->data[canByte+1] = (uint8_t)(rVal >> 8); //Upper 8 bits
            break;
        case 2:     //If the pin is an digital input, fill CAN message with the value to be sent out
            rVal = digitalRead(devicePin);
            *outVar = rVal ^ invRet;
            watchID->data[canByte] &= ~(1 << canBitMsk);
            watchID->data[canByte] |= rVal << canBitMsk;
            break;
        case 3:     //If the pin is an inverted digital input, fill CAN message with the value to be sent out
            rVal = !digitalRead(devicePin);
            *outVar = rVal ^ invRet;
            watchID->data[canByte] &= ~(1 << canBitMsk);
            watchID->data[canByte] |= rVal << canBitMsk; //Read true if pulled to ground
            break;
        case 4:     //If the pin is an analog output, set output to value found in CAN message
            analogWrite(devicePin,watchID->data[canByte]&canBitMsk);
            break;
        case 6:
            rVal = digitalRead(devicePin);
            *outVar = rVal ^ invRet;
            watchID->data[canByte] &= ~(1 << canBitMsk);
            watchID->data[canByte] |= rVal << canBitMsk;
        }
        
    }
    //Function returns true or false if the monitoring bit is set true or not (similar to autoCAN, but doesnt update hardware)
    bool pwmCANFlag(){
        if((watchID->data[canByte]) & (1 << canBitMsk)) return true;
        return false;
    }
    //Function returns an entire byte value from the CAN message
    uint8_t getCANWatchValue(){
        return watchID->data[canByte];
    }
    //Function to manually set a pin high (3.3V) or low (0V)
    void dWrite(bool cmp){
        digitalWrite(devicePin,cmp);
    }
    //Function to manually set output PWM value (for pins that support PWM)
    void aWrite(uint8_t cmp){
        analogWrite(devicePin, cmp);
    }
    //Function to read the value of the pin (High or Low)
    bool dRead(){
        return digitalRead(devicePin);
    }
};

class WatchVar{
    public:
    uint8_t canByte;
    uint8_t canBitMsk;
    bool *outVar;
    CANMessage *watchID;
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk, bool *watchVar = nullptr){
        outVar = watchVar;
        watchID = wtch;
        canByte = cbyte;
        canBitMsk = cbitmsk;
    }
    void autoCAN(){
        *outVar = watchID->data[canByte] & (1 << canBitMsk);
    }
};


LPDRVPin LP0;
LPDRVPin LP1;
LPDRVPin LP2;
LPDRVPin LP3;

LPDRVPinPWM LP4;
LPDRVPinPWM LP5;
LPDRVPinPWM LP6;
LPDRVPinPWM LP7;

HPDRVPin HP0;
HPDRVPin HP1;
HPDRVPin HP2;

IOPin IP0;
IOPin IP1;
IOPin IP2;
IOPin IP3;

WatchVar WV0;
WatchVar WV1;

bool startupToggle = true;
uint8_t animationMode;
uint16_t animationTick;

Timer pTimer(10, animationHandler);
Timer pTimer2(250, animationHandler2);

void boardConfig();
void autoBoardCAN();

void setup() {
    can.begin(500000);
    can.addFilter(0x100,0x7FF);
    boardConfig();
    #ifdef USING_NEOPIXEL
    strip.begin();
    #endif
    animationMode = 0;
    animationTick = 0;
    autoBoardCAN(); 
    pTimer.start();
    
}


void loop() {
    if(can.receive(inputMessage)){
        if(inputMessage.id == CAN_SNS) pinStatus = inputMessage;
        autoBoardCAN();
        delay(5);
    }
}

void animationHandler(){
    #ifdef BDFL //Front-Left Driver Board Config
        if(animationMode == 0){    //Neopixel animated, turn signal not animated
            if(IP0.pwmCANFlag()){
                if(animationTick == PIXEL_COUNT) return;
                strip.setPixelColor(animationTick++,AMBER);
                strip.show();
            }
            else{
                strip.clear();
                strip.show();
                animationTick = 0;
            }
        }
        else if(animationMode == 1){
            if(IP0.pwmCANFlag()){
                byte gCol = animationTick*0.85;
                byte bCol = animationTick*0.75;
                for(int i = 0; i < PIXEL_COUNT; i++) strip.setPixelColor(i, animationTick,gCol, bCol);
                strip.show();
                if(animationTick < 255) animationTick++;
            }
            else{
                animationTick = 0;
            }
        }
        else if(animationMode == 2){
            uint16_t tempTick  = (animationTick >> 2);
            if(tempTick < (PIXEL_COUNT) + SOL_FADE_WID + 1){
                strip.setPixelColor(tempTick-SOL_FADE_WID-1,0,0,0);
                for(int i = 0; i < SOL_PUL_WIDTH; i++){
                    strip.setPixelColor(tempTick-1+i,255,255,255);
                }
                for(int i = 0; i < SOL_FADE_WID; i++){
                    strip.setPixelColor(tempTick+SOL_PUL_WIDTH-1+i,255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1));
                    strip.setPixelColor(tempTick-i,255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1));
                }
                strip.show();
                animationTick++;
            }
            else if(tempTick < (PIXEL_COUNT << 1) + SOL_FADE_WID + 1){
                uint8_t pixNum = PIXEL_COUNT-1-(tempTick-(PIXEL_COUNT));
                strip.setPixelColor(pixNum,255,255,255);
                strip.show();
                animationTick++;
            }
            else{
                startupToggle = false;
                animationMode = 0;
                animationTick = 0;
            }
        }
    #endif
    #ifdef BDFR //Front-Right Driver Board Config
        if(animationMode == 0){    //Neopixel animated, turn signal not animated
            if(IP0.pwmCANFlag()){
                if(animationTick == PIXEL_COUNT) return;
                strip.setPixelColor(animationTick++,AMBER);
                strip.show();
            }
            else{
                strip.clear();
                strip.show();
                animationTick = 0;
            }
        }
        else if(animationMode == 1){
            if(IP0.pwmCANFlag()){
                byte gCol = animationTick*0.85;
                byte bCol = animationTick*0.75;
                for(int i = 0; i < PIXEL_COUNT; i++) strip.setPixelColor(i, animationTick,gCol, bCol);
                strip.show();
                if(animationTick < 255) animationTick++;
            }
            else{
                animationTick = 0;
            }
            
        }
        else if(animationMode == 2){
            uint16_t tempTick  = (animationTick >> 2);
            if(tempTick < (PIXEL_COUNT) + SOL_FADE_WID + 1){
                strip.setPixelColor(tempTick-SOL_FADE_WID-1,0,0,0);
                for(int i = 0; i < SOL_PUL_WIDTH; i++){
                    strip.setPixelColor(tempTick-1+i,255,255,255);
                }
                for(int i = 0; i < SOL_FADE_WID; i++){
                    strip.setPixelColor(tempTick+SOL_PUL_WIDTH-1+i,255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1));
                    strip.setPixelColor(tempTick-i,255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1),255-STRT_FADE_OFFSET*(i+1));
                }
                strip.show();
                animationTick++;
            }
            else if(tempTick < (PIXEL_COUNT << 1) + SOL_FADE_WID + 1){
                uint8_t pixNum = PIXEL_COUNT-1-(tempTick-(PIXEL_COUNT));
                strip.setPixelColor(pixNum,255,255,255);
                strip.show();
                animationTick++;
            }
            else{
                startupToggle = false;
                animationMode = 0;
                animationTick = 0;
            }
        }
        uint16_t battCurrent = analogRead(A3);
        CANSend(0x102,battCurrent & 255, battCurrent >> 8,0,0,0,0,0,0);
    #endif
    #ifdef BDRL
        if(!faultBMS && !faultSwitch){   //No fault
            if(IP2.pwmCANFlag()){
                if(flashOn){
                    for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                        strip.setPixelColor((PIXEL_COUNT>>1)+i,255,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-1-i,255,0,0);
                    }
                    strip.show();
                    flashOn = 0;
                }
                if(animationTick < (PIXEL_COUNT>>1)){
                    strip.setPixelColor((PIXEL_COUNT>>1)+1+animationTick,255,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-animationTick,255,0,0);
                    strip.show();
                    animationTick++;
                }
            }
            else{
                if(flashOn){
                    for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                        strip.setPixelColor((PIXEL_COUNT>>1)+i,0,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-1-i,0,0,0);
                    }
                    strip.show();
                    flashOn = 0;
                }
                if((carCharge || solarCharge) && !ignition){
                    uint16_t tempTick  = (animationTick >> 2);
                    if(tempTick < (PIXEL_COUNT >> 1) + SOL_FADE_WID + 1){
                        strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-SOL_FADE_WID-1,0,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-tempTick+SOL_FADE_WID,0,0,0);
                        if(carCharge && !solarCharge){
                            for(int i = 0; i < SOL_PUL_WIDTH; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-1+i,100,0,100);
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-i,100,0,100);
                            }
                            for(int i = 0; i < SOL_FADE_WID; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick+SOL_PUL_WIDTH-1+i,100-SOL_FADE_OFFSET*(i+1),0,100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-i,100-SOL_FADE_OFFSET*(i+1),0,100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-SOL_PUL_WIDTH-i,100-SOL_FADE_OFFSET*(i+1),0,100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-1+i,100-SOL_FADE_OFFSET*(i+1),0,100-SOL_FADE_OFFSET*(i+1));
                            }
                        }
                        else if(solarCharge && !carCharge){
                            for(int i = 0; i < SOL_PUL_WIDTH; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-1+i,0,100,0);
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-i,0,100,0);
                            }
                            for(int i = 0; i < SOL_FADE_WID; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick+SOL_PUL_WIDTH-1+i,0,100-SOL_FADE_OFFSET*(i+1),0);
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-i,0,100-SOL_FADE_OFFSET*(i+1),0);
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-SOL_PUL_WIDTH-i,0,100-SOL_FADE_OFFSET*(i+1),0);
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-1+i,0,100-SOL_FADE_OFFSET*(i+1),0);
                            }
                        }
                        else{
                            for(int i = 0; i < SOL_PUL_WIDTH; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-1+i,100,0,100);
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-i,100,0,100);
                            }
                            for(int i = 0; i < SOL_FADE_WID; i++){
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick+SOL_PUL_WIDTH-1+i,100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-i,100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-SOL_PUL_WIDTH-i,100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1));
                                strip.setPixelColor((PIXEL_COUNT>>1)-tempTick-1+i,100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1),100-SOL_FADE_OFFSET*(i+1));
                            }
                        }
                        strip.show();
                        animationTick++;
                    }
                    else{
                        if(animationTick < SOL_PUL_CYCLES) animationTick++;
                        else animationTick = 0;
                    }
                    return;
                }
                if(animationTick >= 0){
                    strip.setPixelColor((PIXEL_COUNT>>1)+1+animationTick,0,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-animationTick,0,0,0);
                    strip.show();
                    if(animationTick)animationTick--;
                }
            }
        }
        else{
            if(IP2.pwmCANFlag()){
                for(int i = FLASH_PIXEL_WIDTH>>1; i < PIXEL_COUNT; i++){
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,255,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,255,0,0);
                }
            }
            else{
                for(int i = FLASH_PIXEL_WIDTH>>1; i < PIXEL_COUNT; i++){
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,0,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,0,0,0);
                }
            }
            if(flashOn){
                for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,255,200,170);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,255,200,170);
                }
            }
            else{
                for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,0,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,0,0,0);
                }
            }
            strip.show();
            
        }
        CANSend(0x103,(int) flashOn, faultBMS, faultSwitch,carCharge,0,0,0,0);
    #endif
    #ifdef BDRR
        
    #endif
}

void animationHandler2(){
    flashOn = !flashOn;

}

void boardConfig(){
    //Low Power Outputs (J10)
    LP7.initialize(D0);         //LP Output 7 (PWM)
    LP6.initialize(D3);         //LP Output 6 (PWM)
    LP5.initialize(A4);         //LP Output 5 (PWM)
    LP4.initialize(A5);         //LP Output 4 (PWM)
    LP3.initialize(D4);         //LP Output 3 
    LP2.initialize(D5);         //LP Output 2
    LP1.initialize(D6);         //LP Output 1
    LP0.initialize(D7);         //LP Output 0 

    HP2.initialize(TX);         //HP Output 2
    HP1.initialize(A7);         //HP Output 1
    HP0.initialize(A6);         //HP Output 0

    #ifdef BDFL //Front-Left Driver Board Config
        replyAddr.id = 0x101;   //Reply on 0x101
        
        LP3.dwrite(1);   //Power supply pin for neopixel 5V reg, have on by default.
        
        LP5.configCANWatch(&pinStatus,1,255);   //Byte 1, 8 bits - Left turn signal PWM
        LP2.configCANWatch(&pinStatus,4,1);     //Byte 4, Bit 1 - High-beam
        LP1.configCANWatch(&pinStatus,4,0);     //Byte 4, Bit 0 - Headlight
        LP0.configCANWatch(&pinStatus,4,4);     //Byte 4, Bit 4 - Steering pump relay
        
        IP3.initialize(A3,1);                   //Unused
        IP2.initialize(A2,1);                   //Unused
        IP1.initialize(A1,5);                   //Neopixel output
        IP0.initialize(RX,5);                   //Unused

        IP0.configCANWatch(&pinStatus,1,0);   //Setup watch for neopixel animation

        HP2.configCANWatch(&pinStatus,7,2);     //Byte 7, Bit 2 - iBooster Power
        HP1.configCANWatch(&pinStatus,7,1);     //Byte 7, Bit 1 - RMS Pump
        HP0.configCANWatch(&pinStatus,7,0);     //Byte 7, Bit 0 - Radiator Fan

        WV0.configCANWatch(&pinStatus,5,3,&startupHDL);

    #endif
    #ifdef BDFR //Front-Right Driver Board Config
        replyAddr.id = 0x102;   //Reply on 0x102

        LP2.dwrite(1);   //Power supply pin for hall current sensor, always have on.
        LP3.dwrite(1);   //Power supply pin for neopixel 5V reg, have on by default.
        
        LP5.configCANWatch(&pinStatus,0,255);   //Byte 0, 8 bits - Right turn signal PWM
        LP1.configCANWatch(&pinStatus,4,1);     //Byte 4, Bit 0 - Headlight
        LP0.configCANWatch(&pinStatus,4,0);     //Byte 4, Bit 4 - Steering pump relay

        //IP3.initialize(A3,5);                   //Unused
        IP2.initialize(A2,5);                   //Neopixel output
        IP1.initialize(A1,1);                   
        IP0.initialize(RX,5);                   //Current Sensor Analog Input

        //IP3.configCANWatch(&replyAddr,0,255);   //Current Sensor Data Reply
        IP0.configCANWatch(&pinStatus,0,0);   //Setup watch for neopixel animation

        WV0.configCANWatch(&pinStatus,5,3,&startupHDL);

    #endif
    #ifdef BDRL
        replyAddr.id = 0x103;   //Reply on 0x103

        LP2.dwrite(1);   //Power supply pin for neopixel 5V reg, have on by default.
        LP4.duty(255);

        //LP6.configCANWatch(&pinStatus,3,255);     //Reverse
        LP6.configCANWatch(&pinStatus,3,255);   //Fan PWM
        LP7.configCANWatch(&pinStatus,1,255);
        LP4.configCANWatch(&pinStatus,2,255);
        LP1.configCANWatch(&pinStatus,4,5);
        //LP0.configCANWatch(&pinStatus,7,3);

        HP0.configCANWatch(&pinStatus,7,4);

        IP3.initialize(A3,6);   //BMS Fault Sense input
        IP2.initialize(A2,5);   //Neopixel output
        IP1.initialize(A1,6);   //Kill-switch Fault Sense input
        IP0.initialize(RX,2);   //Solar Charge Switch sense input

        IP2.configCANWatch(&pinStatus,4,6); //Brake Press

        IP3.configCANWatch(&replyAddr,0,0,&faultBMS, true); //BMS fault reply - used for BPS Flash
        
        IP1.configCANWatch(&replyAddr,1,0,&faultSwitch, true); //Kill-switch fault reply - used for BPS Flash
        IP0.configCANWatch(&replyAddr,2,0,&carCharge); //Solar Charge Enable reply

        WV0.configCANWatch(&pinStatus,4,3,&solarCharge);
        WV1.configCANWatch(&pinStatus,5,0,&ignition);

    #endif
    #ifdef BDRR
        replyAddr.id = 0x104;   //Reply on 0x104

        LP6.configCANWatch(&pinStatus,3,255);   //Fan PWM
        //LP6.configCANWatch(&pinStatus,3,255);   //Reverse signal PWM
        LP7.configCANWatch(&pinStatus,0,255);   //Right turn signal PWM
        LP4.configCANWatch(&pinStatus,2,255);   //Brake PWM
        LP1.configCANWatch(&pinStatus,4,5);     //Reverse on-off
        //LP1.configCANWatch(&pinStatus,7,3);     //Battery Fan Power
        
        HP0.configCANWatch(&pinStatus,4,2);     //Trunk Release Switch

        IP3.initialize(A3,1);   //unused
        IP2.initialize(A2,4);   //Fan PWM Output
        IP1.initialize(A1,5);   //neopixel output, but unused currently
        IP0.initialize(RX,1);   //unused

        IP2.aWrite(255); //Start at 100% fan duty cycle

    #endif


    pinMode(A0, INPUT_PULLUP);  //General-purpose button
}

void autoBoardCAN(){
    #ifdef BDFL //Front-Left Driver Board Config
        LP5.autoCAN();   //Byte 1, 8 bits - Left turn signal PWM
        LP2.autoCAN();   //Byte 4, Bit 1 - High-beam
        LP1.autoCAN();   //Byte 4, Bit 0 - Headlight
        LP0.autoCAN();   //Byte 4, Bit 4 - Steering pump relay

        HP2.autoCAN();   //Byte 7, Bit 2 - iBooster Power
        HP1.autoCAN();   //Byte 7, Bit 1 - RMS Pump
        HP0.autoCAN();   //Byte 7, Bit 0 - Radiator Fan

        WV0.autoCAN();

        if(startupHDL && startupToggle && animationMode != 2){
            animationMode = 2;
            startupToggle = false;
        }
        if(!startupToggle && !startupHDL){
            startupToggle = true;
        }
    #endif
    #ifdef BDFR //Front-Right Driver Board Config
        LP5.autoCAN();   //Byte 0, 8 bits - Right turn signal PWM
        LP1.autoCAN();   //Byte 4, Bit 0 - Headlight
        LP0.autoCAN();   //Byte 4, Bit 4 - Steering pump relay

        WV0.autoCAN();

        if(startupHDL && startupToggle && animationMode != 2){
            animationMode = 2;
            startupToggle = false;
        }
        if(!startupToggle && !startupHDL){
            startupToggle = true;
        }
    #endif
    #ifdef BDRL
        LP6.autoCAN();
        LP7.autoCAN();
        LP4.autoCAN();
        LP1.autoCAN();
        LP0.autoCAN();

        HP0.autoCAN();

        IP3.autoCAN();
        IP1.autoCAN();
        IP0.autoCAN();

        if(digitalRead(A1)){
            RGB.control(true);
            RGB.color(0,255,0);
        }

        WV0.autoCAN();
        WV1.autoCAN();

        if(faultBMS || faultSwitch){
            if(!pTimer2.isActive()){
                pTimer2.start();
                flashOn = 1;
            }
        }
        else{
            if(pTimer2.isActive()){
                pTimer2.stop();
                flashOn = 1;
            }
        }
    #endif
    #ifdef BDRR
        LP6.autoCAN();
        LP7.autoCAN();
        LP4.autoCAN();
        LP1.autoCAN();
        
        HP0.autoCAN();
    #endif
}

void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7){
    replyAddr.len = 8;
    replyAddr.data[0] = data0;
    replyAddr.data[1] = data1;
    replyAddr.data[2] = data2;
    replyAddr.data[3] = data3;
    replyAddr.data[4] = data4;
    replyAddr.data[5] = data5;
    replyAddr.data[6] = data6;
    replyAddr.data[7] = data7;
    
    can.transmit(replyAddr);
}

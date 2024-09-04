/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mligh/OneDrive/Documents/GitHub/DecentralizedLV-LPDRV/src/DecentralizeLPDRV.ino"
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
#line 13 "c:/Users/mligh/OneDrive/Documents/GitHub/DecentralizedLV-LPDRV/src/DecentralizeLPDRV.ino"
#define BDFL      //Front-Left Board
//#define BDFR      //Front-Right Board
//#define BDRL      //Rear-Left Board
//#define BDRR        //Rear-Right Board

SYSTEM_MODE(SEMI_AUTOMATIC);    //Disable Wi-Fi for this system

/////////////////////////////////////////////////////////////////////////
// Neopixel Board Params   Allows Per-Board Pixel Count and Pin Config //
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
#define TRN_DELAY           30      //Milliseconds between publishes of transmission gear

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
uint16_t trnCount = 0;
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

class WatchVar{     //WatchVar is like a "software pin" where a variable can have its value changed by CAN as opposed to a hardware pin
    public:
    uint8_t canByte;            //Byte of the CAN Message to monitor (0-7)
    uint8_t canBitMsk;          //Bit number in the byte to control output on-off
    CANMessage *watchID;        //Pointer to the CAN message to watch
    bool *outVar;               //Pointer to the variable that is updated based on the CAN message
    //Configure which bit and byte to monitor in the CAN message
    void configCANWatch(CANMessage *wtch, uint8_t cbyte, uint8_t cbitmsk, bool *watchVar = nullptr){
        outVar = watchVar;
        watchID = wtch;
        canByte = cbyte;
        canBitMsk = cbitmsk;
    }
    //Function updates whatever variable outVar points to with the value contained in the CAN message
    void autoCAN(){
        *outVar = watchID->data[canByte] & (1 << canBitMsk);
    }
};

/////////////////////////////
// Pin Object Declarations //
/////////////////////////////

//Standard board has 4 low-power outputs without PWM (LP Outputs 0-3)
LPDRVPin LP0;
LPDRVPin LP1;
LPDRVPin LP2;
LPDRVPin LP3;

//Standard board has 4 low-power outputs with PWM (LP Outputs 4-7)
LPDRVPinPWM LP4;
LPDRVPinPWM LP5;
LPDRVPinPWM LP6;
LPDRVPinPWM LP7;

//Standard board has 3 high-power outputs (Relays)
HPDRVPin HP0;
HPDRVPin HP1;
HPDRVPin HP2;

//Standard board has 4 Input/Output pins for neopixels/sensors/switches
IOPin IP0;
IOPin IP1;
IOPin IP2;
IOPin IP3;

//Created some WatchVars - Useful for handling fault signals like the BMS and kill switch
WatchVar WV0;
WatchVar WV1;

bool startupToggle = true;  //Flag for playing neat animations on startup
uint8_t animationMode;      //Global for the current animation mode
uint16_t animationTick;     //A counter that makes animation have very little overhead - change the state of the LED based on the counter value

Timer pTimer(10, animationHandler);     //Create a timer that happens ever 10 milliseconds for updating the animations
Timer pTimer2(250, animationHandler2);  //A timer for the LV board that handles flashing the BPS fault light

void boardConfig();     //Function prototype
void autoBoardCAN();

//Function runs once at power up to configure the device.
void setup() {
    can.begin(500000);          //Start CAN communication at 500kbps
    can.addFilter(0x100,0x7FF); //Adds a filter to accept messages with CAN ID 0x100
    boardConfig();              //Call function to configure the pins depending on which board this is
    #ifdef USING_NEOPIXEL       //If neopixel is being used on this board, initialize the strip
    strip.begin();
    #endif
    animationMode = 0;          //Reset animation mode to default, set the tick to 0
    animationTick = 0;
    autoBoardCAN();             //Call autoBoardCAN, which just calls autoCAN() on each of pins
    pTimer.start();             //start the timer for the animations
    
}

//Loop continuously executes as fast as possible
void loop() {
    //All of the fancy pin logic may be complicated but makes life in the loop real EZ
    if(can.receive(inputMessage)){      //If a CAN Message was received from another board matching the filter (ID 0x100)
        if(inputMessage.id == CAN_SNS) pinStatus = inputMessage;    //Copy the message into the main message listened to by all of the pins
        autoBoardCAN();     //Call autoCAN on all the pins to update based on the new message, and you're done, the pins automatically update state
        delay(5);           //Delay for a few milliseconds so we aren't blazing around the loop
    }
}

//AnimationHandler updates the states of LEDs and other things that play animations over time - using an interrupt allows loop() to continue to run and get updates
void animationHandler(){
    #ifdef BDFL //Front-Left Driver Board Config
        if(animationMode == 0){    //Neopixel animated, turn signal not animated
            if(IP0.pwmCANFlag()){   //Flag set true when there is a left turn signal active
                if(animationTick == PIXEL_COUNT) return;    //If we've reached the length of the strip, we've lighted all of the pixels
                strip.setPixelColor(animationTick++,AMBER); //Light pixel # animation tick AMBER, then increment the counter so the next pixel is lit the next time the interrupt runs
                strip.show();   //Call show to update the LED strip
            }
            else{   //If the turn signal is turned off, turn off all of the pixels in the strip
                strip.clear();
                strip.show();
                animationTick = 0;  //Reset the counter so we start at the beginning of the strip again
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
        if(animationMode == 0){    //Chase animation - fills strip starting at pixel 0
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
        else if(animationMode == 1){    //Fade-up entire strip animation
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
        else if(animationMode == 2){    //Startup animation - small group of pixels travelling across strip
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
            if(IP2.pwmCANFlag()){   //If the brake pedal is pressed
                if(flashOn){ //If the white section of the BPS fault is ON, turn back to RED since fault was cleared
                    for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                        strip.setPixelColor((PIXEL_COUNT>>1)+i,255,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-1-i,255,0,0);
                    }
                    strip.show();
                    flashOn = 0;
                }
                if(animationTick < (PIXEL_COUNT>>1)){   //Do animation where the pixels start from the center and fill going outwards
                    strip.setPixelColor((PIXEL_COUNT>>1)+1+animationTick,255,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-animationTick,255,0,0);
                    strip.show();
                    animationTick++;
                }
            }
            else{   //Not braking
                if(flashOn){    //If the white section of the BPS fault is ON, turn OFF since fault was cleared
                    for(int i = 0; i < FLASH_PIXEL_WIDTH>>1; i++){
                        strip.setPixelColor((PIXEL_COUNT>>1)+i,0,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-1-i,0,0,0);
                    }
                    strip.show();
                    flashOn = 0;
                }
                if((carCharge || solarCharge) && !ignition){    //Animation for car charging/solar charging mode
                    uint16_t tempTick  = (animationTick >> 2);
                    if(tempTick < (PIXEL_COUNT >> 1) + SOL_FADE_WID + 1){
                        strip.setPixelColor((PIXEL_COUNT>>1)+tempTick-SOL_FADE_WID-1,0,0,0);
                        strip.setPixelColor((PIXEL_COUNT>>1)-tempTick+SOL_FADE_WID,0,0,0);
                        if(carCharge && !solarCharge){      //Charging from both solar and wall
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
                        else if(solarCharge && !carCharge){     //Solar charging but not car charging
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
                        else{   //Wall charging but not solar charging
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
        else{   //Animation if there is a fault happening
            if(IP2.pwmCANFlag()){   //Is the brake pressed at the same time? 
                for(int i = FLASH_PIXEL_WIDTH>>1; i < PIXEL_COUNT; i++){    //Light up pixels surrounding BPS flash area as RED
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,255,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,255,0,0);
                }
            }
            else{   //Otherwise turn off LEDs surrounding the BPS flasher
                for(int i = FLASH_PIXEL_WIDTH>>1; i < PIXEL_COUNT; i++){
                    strip.setPixelColor((PIXEL_COUNT>>1)+i,0,0,0);
                    strip.setPixelColor((PIXEL_COUNT>>1)-1-i,0,0,0);
                }
            }
            if(flashOn){    //If the flasher variable is true, light up the section white. Variable toggled every 0.25 seconds by interrupt
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
    flashOn = !flashOn; //Simply toggle the variable every time the interupt happens to make the section flash

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
        IP1.initialize(A1,1);                   //Unused
        IP0.initialize(RX,5);                   //Neopixel output

        IP0.configCANWatch(&pinStatus,1,0);     //Setup watch for neopixel animation

        HP2.configCANWatch(&pinStatus,7,2);     //Byte 7, Bit 2 - Unused
        HP1.configCANWatch(&pinStatus,7,1);     //Byte 7, Bit 1 - RMS Pump
        HP0.configCANWatch(&pinStatus,7,0);     //Byte 7, Bit 0 - Radiator Fan

        WV0.configCANWatch(&pinStatus,5,3,&startupHDL); //Populate the variable startupHDL transmitted from the sensor board

        
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
        LP6.configCANWatch(&pinStatus,3,255);       //Fan PWM
        LP7.configCANWatch(&pinStatus,1,255);       //Left turn signal PWM
        LP4.configCANWatch(&pinStatus,2,255);       //Brake PWM
        LP1.configCANWatch(&pinStatus,4,5);         //Backup camera output
        //LP0.configCANWatch(&pinStatus,7,3);

        HP0.configCANWatch(&pinStatus,7,4);     //Relay for MPPT

        IP3.initialize(A3,6);   //BMS Fault Sense input
        IP2.initialize(A2,5);   //Neopixel output
        IP1.initialize(A1,6);   //Kill-switch Fault Sense input
        IP0.initialize(RX,2);   //Solar Charge Switch sense input

        IP2.configCANWatch(&pinStatus,4,6); //Brake Press

        IP3.configCANWatch(&replyAddr,0,0,&faultBMS, true); //BMS fault reply - used for BPS Flash
        
        IP1.configCANWatch(&replyAddr,1,0,&faultSwitch, true); //Kill-switch fault reply - used for BPS Flash
        IP0.configCANWatch(&replyAddr,2,0,&carCharge); //Solar Charge Enable reply

        WV0.configCANWatch(&pinStatus,4,3,&solarCharge);    //Flag set by the sensor board if the solar charge switch is on - used for animations
        WV1.configCANWatch(&pinStatus,5,0,&ignition);       //Flag set by the sensor board if the ignition switch is on - used for animations

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

//Function to call autoCAN() on all of the ACTIVE pins for a particular board - don't call autoCAN on an unitialized pin as you'll crash the code
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

        if(millis() - trnCount > TRN_DELAY){
            uint16_t sns1 = (analogRead(A3) >> 4);
            uint16_t sns2 = (analogRead(A2) >> 4);

            CANSend(0x101,(uint8_t)sns1,(uint8_t)sns2,0,0,0,0,0,0);
            trnCount = millis();
        }

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

        //Kind of a lazy spot to put this
        if(faultBMS || faultSwitch){    //Check if one of the fault sources has been triggered (switch or BMS)
            if(!pTimer2.isActive()){    //Start the timer if it isn't active
                pTimer2.start();        //Start the timer for the white flash signal
                flashOn = 1;            //Instantly turn on the BPS LED - interrupt toggles this
            }
        }
        else{
            if(pTimer2.isActive()){     //Stop the timer if there is no fault and it is running
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

//Function takes in data from its arguments and sends out a CAN message to the system
void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7){
    replyAddr.len = 8;      //Send 8 bytes of data
    replyAddr.data[0] = data0;  //Copy in each of the bytes
    replyAddr.data[1] = data1;
    replyAddr.data[2] = data2;
    replyAddr.data[3] = data3;
    replyAddr.data[4] = data4;
    replyAddr.data[5] = data5;
    replyAddr.data[6] = data6;
    replyAddr.data[7] = data7;
    
    can.transmit(replyAddr);    //Transmit out the CAN data
}

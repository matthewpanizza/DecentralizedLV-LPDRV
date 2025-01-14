/*
 * Project DecentralizeLPDRV
 * Description: Code for the Decentralized Low-Voltge System Low-Power Driver Boards
 * Author: Matthew Panizza
 * Date: May 7, 2022
 */
#include "neopixel.h"   //Neopixel Library for Addressable Strips
#include "DecentralizedLV-Boards/DecentralizedLV-Boards.h"

/////////////////////////////////////////////////////////
// BOARD DEFINITION - UNCOMMENT BOARD TO BE PROGRAMMED //
/////////////////////////////////////////////////////////

//#define BDFL      //Front-Left Board
//#define BDFR      //Front-Right Board
//#define BDRL      //Rear-Left Board
#define BDRR        //Rear-Right Board

/////////////////////////////////////////////////////////
/////////////      PIN MAPPINGS      ////////////////////
/////////////////////////////////////////////////////////

#define LP7     D0      //Low Power Output 7 (PWM)
#define LP6     D3      //Low Power Output 6 (PWM)
#define LP5     A4      //Low Power Output 5 (PWM)
#define LP4     A5      //Low Power Output 4 (PWM)
#define LP3     D4      //Low Power Output 3 
#define LP2     D5      //Low Power Output 2
#define LP1     D6      //Low Power Output 1
#define LP0     D7      //Low Power Output 0 

#define HP2     TX      //High Power Output 2
#define HP1     A7      //High Power Output 1
#define HP0     A6      //High Power Output 0

#define IP3     RX      //Multipurpose IO Pin 3
#define IP2     A1      //Multipurpose IO Pin 2
#define IP1     A2      //Multipurpose IO Pin 1 
#define IP0     A3      //Multipurpose IO Pin 0

////////////////////////////
// CAN Message Formatting //
////////////////////////////

CAN_Controller canController;

LPDRV_RearLeft_CAN rearLeftDriver(REAR_LEFT_DRIVER);

DashController_CAN dashController(DASH_CONTROL_ADDR);

PowerController_CAN powerController(POWER_CONTROL_ADDR);

LV_CANMessage inputMessage;    //Input message for receiving from multiple addresses and filtering

Timer pTimer(10, animationHandler);     //Create a timer that happens ever 10 milliseconds for updating the animations
Timer pTimer2(250, animationHandler2);  //A timer for the LV board that handles flashing the BPS fault light

//Misc control-flow variables
uint32_t loop_time = 0;

void boardConfig();     //Function prototype
void transmitLPDRVBoards();
void updateOutputPins();
void updateInputPins();
void updateAnimations();

SYSTEM_MODE(SEMI_AUTOMATIC);    //Disable Wi-Fi for this system

//Function runs once at power up to configure the device.
void setup() {
    canController.begin(500000);
    canController.addFilter(powerController.boardAddress);   //Allow incoming messages from Power Controller
    canController.addFilter(dashController.boardAddress);    //Allow incoming messages from Dash Controller
    boardConfig();              //Call function to configure the pins depending on which board this is
    updateOutputPins();         //Update the pin state based on the data received from other boards
    updateInputPins();          //Update this board's variables that are set by reading the input pins
    pTimer.start();             //start the timer for the animations
    
}

//Loop continuously executes as fast as possible
void loop() {
    //All of the fancy pin logic may be complicated but makes life in the loop real EZ
    while(canController.receive(inputMessage)){      //If a CAN Message was received from another board matching the filter (ID 0x100)
        dashController.receiveCANData(inputMessage);
        powerController.receiveCANData(inputMessage);
    }
    updateOutputPins();         //Update the pin state based on the data received from other boards
    updateInputPins();          //Read the multipurpose pins that need to be transmitted from this board
    updateAnimations();

    while(millis()-loop_time < 10) delayMicroseconds(1);    //Fancy delay mechanism which takes into account time to read CAN bus frames
    loop_time = millis();

    transmitLPDRVBoards();      //Transmits data from this board to the CAN Bus.
}

/// @brief Call the sendCANData for the respective LPDRV board you've programmed.
void transmitLPDRVBoards(){
    #ifdef BDFL //Front-Left Driver Board
        //Haven't made a Front Left Driver class in Boards API yet. Will need to make one if you want to transmit data from this board.
    #endif
    #ifdef BDFR //Front-Right Driver Board
        //Haven't made a Front Right Driver class in Boards API yet. Will need to make one if you want to transmit data from this board.
    #endif
    #ifdef BDRL
        rearLeftDriver.sendCANData(canController);
    #endif
    #ifdef BDRR
        //Haven't made a Rear Right Driver class in Boards API yet. Will need to make one if you want to transmit data from this board.
    #endif
    
}

//AnimationHandler updates the states of LEDs and other things that play animations over time - using an interrupt allows loop() to continue to run and get updates
void animationHandler(){
    #ifdef BDFL //Front-Left Driver Board Config

    #endif
    #ifdef BDFR //Front-Right Driver Board Config

    #endif
    #ifdef BDRL

    #endif
    #ifdef BDRR
        
    #endif
}

void animationHandler2(){
    //flashOn = !flashOn; //Simply toggle the variable every time the interupt happens to make the section flash

}

void boardConfig(){
    //Low Power Outputs (J10)
    pinMode(LP7, OUTPUT);
    pinMode(LP6, OUTPUT);
    pinMode(LP5, OUTPUT);
    pinMode(LP4, OUTPUT);
    pinMode(LP3, OUTPUT);
    pinMode(LP2, OUTPUT);
    pinMode(LP1, OUTPUT);
    pinMode(LP0, OUTPUT);

    pinMode(HP2, OUTPUT);
    pinMode(HP1, OUTPUT);
    pinMode(HP0, OUTPUT);

    #ifdef BDFL //Front-Left Driver Board Config
        
        //pinMode(IP3, INPUT);      //Don't set this value if using it for the neopixels. Pin RX is being used for neopixel currently.
        pinMode(IP2, INPUT);
        pinMode(IP1, INPUT);
        pinMode(IP0, INPUT);
        
    #endif
    #ifdef BDFR //Front-Right Driver Board Config
       
        //pinMode(IP3, INPUT);      //Don't set this value if using it for the neopixels. Pin RX is being used for neopixel currently.
        pinMode(IP2, INPUT);
        pinMode(IP1, INPUT);
        pinMode(IP0, INPUT);

    #endif
    #ifdef BDRL

        //pinMode(IP3, INPUT);      //Don't set this value if using it for the neopixels. Pin RX is being used for neopixel currently.
        pinMode(IP2, INPUT);
        pinMode(IP1, INPUT);
        pinMode(IP0, INPUT);

    #endif
    #ifdef BDRR

        pinMode(IP3, INPUT);
        pinMode(IP2, OUTPUT);       //Fan PWM Output
        //pinMode(IP1, INPUT);      //Don't set this value if using it for the neopixels. Pin A1 is being used for neopixel currently.
        pinMode(IP0, INPUT);

    #endif
    
    pinMode(A0, INPUT_PULLUP);  //General-purpose button
}

//Update the state of the pins in this function. Do reads/writes to and from the DecentralizedLV API objects.
void updateOutputPins(){
    #ifdef BDFL //Front-Left Driver Board Config
        
        analogWrite(LP5, dashController.leftTurnPWM);   //Left turn signal PWM. analogWrite creates PWM pulse based on duty cycle (0-255)
        digitalWrite(LP2, dashController.highbeam);     //High-beam output
        digitalWrite(LP1, dashController.headlight);    //Headlight output
        
        digitalWrite(HP2, powerController.Ign);         //iBooster Power. Turn this on whenever ignition is on
        digitalWrite(HP1, dashController.radiatorPump); //RMS Pump
        digitalWrite(HP0, dashController.radiatorFan);  //RMS radiator fan

    #endif
    #ifdef BDFR //Front-Right Driver Board Config

        analogWrite(LP5, dashController.rightTurnPWM);  //Right turn signal PWM. analogWrite creates PWM pulse based on duty cycle (0-255)
        digitalWrite(LP2, dashController.highbeam);     //High-beam output
        digitalWrite(LP1, dashController.headlight);    //Headlight output

    #endif
    #ifdef BDRL

        analogWrite(LP7, dashController.leftTurnPWM);   //Left turn signal PWM. analogWrite creates PWM pulse based on duty cycle (0-255)
        analogWrite(LP6, dashController.batteryFanPWM); //Battery box has a fan on the left and right side. Use this pin to power the left side fan

        if(powerController.BrakeSense) analogWrite(LP4, 255);   //If the brake pedal is pressed, turn on brake lights to full brightness (255)
        else if(dashController.headlight) analogWrite(LP4, 80); //If the brake is not pressed and the headlights are on, then dim the brake lights like a regular car
        else analogWrite(LP4, 0);                               //If brake pedal is not pressed and the headlights are not on, thne the brake lights should be fully off.
        
        digitalWrite(LP1, dashController.reversePress);         //If we're in reverse, then turn on the backup lights
        digitalWrite(LP0, dashController.reversePress);         //If we're in reverse, turn on the backup camera power

        digitalWrite(HP0, powerController.Acc);                 //We want to turn on the MPPT whenever the Accessory bus is on. Use the Power Controller's accessory flag as a virtual power rail.

        rearLeftDriver.bmsFaultInput = digitalRead(IP3);        //Read in the state of the general purpose IO 3. This is the BMS fault switch. Update the flag for this controller
        rearLeftDriver.switchFaultInput = digitalRead(IP1);     //Read in the state of the general purpose IO 1. This is the BMS fault switch. Update the flag for this controller

    #endif
    #ifdef BDRR

        analogWrite(LP7, dashController.rightTurnPWM);  //Right turn signal PWM. analogWrite creates PWM pulse based on duty cycle (0-255)
        analogWrite(LP6, dashController.batteryFanPWM); //Battery box has a fan on the left and right side. Use this pin to power the right side fan

        if(powerController.BrakeSense) analogWrite(LP4, 255);   //If the brake pedal is pressed, turn on brake lights to full brightness (255)
        else if(dashController.headlight) analogWrite(LP4, 80); //If the brake is not pressed and the headlights are on, then dim the brake lights like a regular car
        else analogWrite(LP4, 0);                               //If brake pedal is not pressed and the headlights are not on, thne the brake lights should be fully off.

        digitalWrite(LP1, dashController.reversePress);         //If we're in reverse, then turn on the backup lights

    #endif
}

/// @brief Read any data this board collects (i.e. switches from the 4X low power inputs) and populate the Boards API variables
void updateInputPins(){
    #ifdef BDFL //Front-Left Driver Board Config
        
        //TO-DO: Read in IP0, IP1, IP2, IP3 here for the functions of the front-right board!
        
    #endif
    #ifdef BDFR //Front-Right Driver Board Config
       
       //TO-DO: Read in IP0, IP1, IP2, IP3 here for the functions of the front-right board!

    #endif
    #ifdef BDRL

        //TO-DO: Read in IP0, IP1, IP2, IP3 here for the functions of the front-right board!
        //An example below of how it was done on SPX
        //rearLeftDriver.bmsFaultInput = digitalRead(IP1);
        //rearLeftDriver.switchFaultInput = digitalRead(IP0);
    
    #endif
    #ifdef BDRR

        //TO-DO: Read in IP0, IP1, IP2, IP3 here for the functions of the front-right board!

    #endif
}

void updateAnimations(){
    #ifdef BDFL

    #endif
    #ifdef BDFR

    #endif
    #ifdef BDRL

    #endif
    #ifdef BDRR

    #endif
}

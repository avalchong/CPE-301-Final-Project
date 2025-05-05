//Authors: Ava Chong, Ernest Velasquez, Jasmine Kong
//Date: 4/29/2025
//Purpose: CPE 301 Final Project
//Description: This project creates an evaporation cooling system (a swamp cooler). Utilizing both hardware and programming, this is achieved. 

#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>   //library for clock
#include <string.h>  //for strings if we need it

#define DHTPIN 7
#define DHTTYPE DHT11
DHT thsensor(DHTPIN, DHTTYPE);                                      //Temp/humidity DHT11 sensor setup

#define RDA       0x80
#define TBE       0x20

#define STEPS 2048
Stepper stepper(STEPS, 8, 9, 10, 13);       //stepper motor setup
#define POTENTIOMETER_PIN A1
int lastSteppedPosition = 0;

const int RS = 11, EN = 12, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);                          //LCD setup

//set up port with arbitrary nums, change later
volatile unsigned char *PORT_B  = (unsigned char *) 0x99;
volatile unsigned char *DDR_B   = (unsigned char *) 0x99;
volatile unsigned char *PIN_B   = (unsigned char *) 0x99;
#define FAN 0x10 //may need to change num later


//basic setups stuff for UART0
volatile unsigned char *myUCSR0A = (unsigned char *) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *) 0x00C6;

//basic setups stuff for ADC
volatile unsigned char *myADMUX = (unsigned char*) 0x7C;
volatile unsigned char *myADCSRB = (unsigned char*) 0x7B;
volatile unsigned char *myADCSRA = (unsigned char*) 0x7A;
volatile unsigned int *myADCDATA = (unsigned int*) 0x78;

//Set up states  FOR turning on and off system
enum State {
    DISABLED,
    IDLE,
    ERROR,
    RUNNING,
};

#define WTR_THRESHOLD 999//arbitrary water threshold, we will need to find that with our particular circuit and parts
#define TEMP_THRESHOLD 99 //arbitrary temp, we will need to find that with our particular circuit and parts
unsigned int wtrLevel = 0;
//Start initial state of program
State currState = DISABLED;
State prevState = DISABLED;
//### Functions Pseudo Code ###//

//We will need functions for:
//adc setup
void adc_init() {
    *myADCSRA = 0x80;
    *myADCSRB = 0x00;
    *myADMUX = 0x40;
  }

  unsigned int adc_read(unsigned char adc_channel) {
    *myADCSRB &= 0xF7; // Reset the MUX
    *myADCSRB |= (adc_channel & 0x08); // Set the MUX
    *myADMUX &= 0xF8; // Reset the MUX
    *myADMUX |= (adc_channel & 0x07); // Set the MUX

    *myADCSRA |= 0x40; // Start conversion
    while (*myADCSRA & 0x40) {} 
    return *myADCDATA; // Return coverted num
  }
  
  //setup serial print

  void U0init(unsigned long U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

//get character
unsigned char U0getchar(){
  return *myUDR0;
}
//put character to serial print
void U0putchar(unsigned char U0pdata){
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}

//this would be in loop maybe instead of a function actually?
wtrLevel = adc_read(0); //might need to change that 0 value
if(waterLvl <= WTR_THRESHOLD){ //if water level is at threshold or is too low, then go to error. in error state change led and print alert
    currState = ERROR;
}

//NOTE**: THIS WOULD BE IN THE ERROR CASE BELOW
lcd.print("Water level is too low!!");
//For monitoring water levels in a reservoir << float likely, to return water level as described by a float
    //print alert when level is low << via LCD
    //MUST USE: water level sensor form kit. 
    //CANNOT use ADC library to perform sampling.
    //Threshold detection can use either an interrupt from the comparator or via a sample using the ADC.
    //MUST use the real-time clock module for event reporting << dunno if this function counts as reporting, but it is here just in case. 
//Ava



// turn on and off fan motor
void controlFanMotor(bool set){
    if(set){
      *PORT_B |= ~FAN;
    }else {
      *PORT_B &= ~FAN;
    }
  }

  float temp = thsensor.readTemperature();
  //This would likely ALSO BE IN THE STATES CASE
  //for IDLE
  if(temp >= TEMP_THRESHOLD){
    currState = RUNNING; //in RUNNING state we would probably call controlFanMotor(true);
  }
  //for RUNNING case
  if(temp < TEMP_THRESHOLD){
    currState = IDLE;
    //in IDLE state we would have controlFanMotor(false);
  }
//To start and stop the fan motor when the temperature falls out of a specified range (if it is too HIGH or too LOW) << takes input from the above function as a float, then stop or start the fan. probably bool (?) for check purposes
    // The Kit motor and fan blade MUST be used for the fan motor. 
    //Be sure to use the included seperate power supply board
    //Connecting the fan directly to the arduino can result in damage to the arduino output circuitry.
//Ava

void currentAirTempAndHumidity(){
    float temp = thsensor.readTemperature();
    float humidity = thsensor.readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature: ");
    lcd.print(temp);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print("%");
}
//Monitor and display current air temp and humidity on LCD screen << void (?)
    //The LCD Display must be used for the required messages
    //You MAY use the arduino library for the LCD. 
    //MUST use the Temp/humidity sensor DHT11 for temp and humidity readings. 
    //There is an arduino library for that sensor, and it is OKAY to use.
//Ernest

bool adjustAngleForVent(){
    int potentiometerValue = analogRead(POTENTIOMETER_PIN);     //idk if i can use analogRead
    int stepsMade = map(potentiometerValue, 0, 1023, 0, STEPS);
    int movement = stepsMade - lastSteppedPosition;
    if(abs(movement) > 3){     //check for a reasonable change in potentiometer value 
        stepper.step(movement);
        lastSteppedPosition = stepsMade;
        return true;
    }
    return false;
}
//Allow a user to use a control to adjust the angle of an output vent from the system << can be void, but probably best to make it a bool to check to ensure action has been done
    //simplified: function to adjust angle of output vent
    //MUST be implemented via the stepper motor
    //Can use either buttons of a potentio meter to control the direction of the vent
    //you MAY use the arduino libraries for the stepper motor
//Ernest

//Turning on and off the system with a button << Bool function

//Jasmine

//Record the time and date every time the motor is turned on or off. << Need input of above function, but this will be a void most likely.
    //Info should be transmitted to host computer via USB << Not sure how to do this one, look into it
    //MUST use the real-time clock module for event reporting
//Jasmine

const int buttonPin = 22;
int buttonState = 0;

void setup() {
    adc_init();
    pinMode(buttonPin, INPUT);
    dht.begin();
    lcd.begin(16, 2);
    stepper.setSpeed(10);

}
  
void loop() {
    buttonState = digitalRead(digitalPin);
    if(buttonState = HIGH){
        //insert functions here
    } else if(buttonState = LOW){
        // turn off circuit
    }

}
  
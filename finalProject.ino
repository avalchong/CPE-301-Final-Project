//Authors: Ava Chong, Ernest Velasquez, Jasmine Kong
//Date: 4/29/2025
//Purpose: CPE 301 Final Project
//Description: This project creates an evaporation cooling system (a swamp cooler). Utilizing both hardware and programming, this is achieved. 

#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

#define DHTPIN 7
#define DHTTYPE DHT11
DHT thsensor(DHTPIN, DHTTYPE);                                      //Temp/humidity DHT11 sensor setup

#define STEPS 2048
Stepper stepper(STEPS, 8, 9, 10, 13);       //stepper motor setup
#define POTENTIOMETER_PIN A1
int lastSteppedPosition = 0;

const int RS = 11, EN = 12, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);                          //LCD setup

//### Functions Pseudo Code ###//

//We will need functions for:

//For monitoring water levels in a reservoir << float likely, to return water level as described by a float
    //print alert when level is low << via LCD
    //MUST USE: water level sensor form kit. 
    //CANNOT use ADC library to perform sampling.
    //Threshold detection can use either an interrupt from the comparator or via a sample using the ADC.
    //MUST use the real-time clock module for event reporting << dunno if this function counts as reporting, but it is here just in case. 
//Ava

//To start and stop the fan motor when the temperature falls out of a specified range (if it is too HIGH or too LOW) << takes input from the above function as a float, then stop or start the fan. probably bool (?) for check purposes
    // The Kit motor and fan blade MUST be used for the fan motor. 
    //Be sure to use the eincluded seperate power supply board
    //Connecting the fan directly to the arduino can result in damage to the arduino output circuitry.
//Ava

void currentAirTempAndHumidity(){
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

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
  
//Authors: Ava Chong, Ernest Velasquez, Jasmine Kong
//Date: 4/29/2025
//Purpose: CPE 301 Final Project
//Description: This project creates an evaporation cooling system (a swamp cooler). Utilizing both hardware and programming, this is achieved. 

//### Functions Pseudo Code ###//

//We will need functions for:

//For monitoring water levels in a reservoir << float likely, to return water level as described by a float
    //print alert when level is low << via LCD
    //MUST USE: water level sensor form kit. 
    //CANNOT use ADC library to perform sampling.
    //Threshold detection can use either an interrupt from the comparator or via a sample using the ADC.
    //MUST use the real-time clock module for event reporting << dunno if this function counts as reporting, but it is here just in case. 

//To start and stop the fan motor when the temperature falls out of a specified range (if it is too HIGH or too LOW) << takes input from the above function as a float, then stop or start the fan. probably bool (?) for check purposes
    // The Kit motor and fan blade MUST be used for the fan motor. 
    //Be sure to use the eincluded seperate power supply board
    //Connecting the fan directly to the arduino can result in damage to the arduino output circuitry.

//Monitor and display current air temp and humidity on LCD screen << void (?)
    //The LCD Display must be used for the required messages
    //You MAY use the arduino library for the LCD. 
    //MUST use the Temp/humidity sensor DHT11 for temp and humidity readings. 
    //There is an arduino library for that sensor, and it is OKAY to use.

//Allow a user to use a control to adjust the angle of an output vent from the system << can be void, but probably best to make it a bool to check to ensure action has been done
    //simplified: function to adjust angle of output vent
    //MUST be implemented via the stepper motor
    //Can use either buttons of a potentio meter to control the direction of the vent
    //you MAY use the arduino libraries for the stepper motor

//Turning on and off the system with a button << Bool function

//Record the time and date every time the motor is turned on or off. << Need input of above function, but this will be a void most likely.
    //Info should be transmitted to host computer via USB << Not sure how to do this one, look into it
    //MUST use the real-time clock module for event reporting

void setup() {
    // put your setup code here, to run once:

}
  
void loop() {
    // put your main code here, to run repeatedly:

}
  
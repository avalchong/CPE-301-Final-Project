//Authors: Ava Chong, Ernest Velasquez, Jasmine Kong
//Date: 5/8/2025
//Purpose: CPE 301 Final Project
//Description: This project creates an evaporation cooling system (a swamp cooler). Utilizing both hardware and programming, this is achieved. 

#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

#define DHTPIN 8
#define DHTTYPE DHT11
DHT thsensor(DHTPIN, DHTTYPE);                                      //Temp/humidity DHT11 sensor setup

#define MAX_STR_SIZE 500

#define RDA       0x80
#define TBE       0x20

#define STEPS 2048
Stepper stepper(STEPS, 7, 9, 10, 13);       //stepper motor setup

int lastSteppedPosition = 0;


const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);                          //LCD setup

//set up port for LEDS
volatile unsigned char *PORT_A  = (unsigned char *) 0x22;
volatile unsigned char *DDR_A   = (unsigned char *) 0x21;
volatile unsigned char *PIN_A   = (unsigned char *) 0x20;

//set up port for button
volatile unsigned char *PORT_D  = (unsigned char *) 0x2B;
volatile unsigned char *DDR_D  = (unsigned char *) 0x2A;
volatile unsigned char *PIN_D   = (unsigned char *) 0x29;

//set up port for motor
volatile unsigned char *PORT_C  = (unsigned char *) 0x28;
volatile unsigned char *DDR_C   = (unsigned char *) 0x27;
volatile unsigned char *PIN_C   = (unsigned char *) 0x26;

//set up port for potentiometer
volatile unsigned char *PORT_F  = (unsigned char *) 0x31;
volatile unsigned char *DDR_F   = (unsigned char *) 0x30;
volatile unsigned char *PIN_F   = (unsigned char *) 0x2F;

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

//Set up pin numbers
//Below correspond to PORTA
#define BLUE_PIN 0
#define RED_PIN 1
#define GREEN_PIN 2
#define YELLOW_PIN 3
//Below correspond to PORTC

#define POTENTIOMETER_PIN 2
#define POTENTIOMETER_TEMP 36 //set as this rn in case it flags problem, but will be 1, associated with PORT_C
#define MOTOR_PIN 2 //in relation to PORTC, pin 35
//below correspond to PORTB
#define btn_PIN 18 //pin 18
#define BUTTON_PIN 3

//Set up states  FOR turning on and off system
enum State {
    DISABLED, IDLE, ERROR, RUNNING
};


RTC_DS1307 rtc;
bool motorState = false;


#define WTR_THRESHOLD 100
const float temp_thresh = 25.0;
unsigned int wtrLevel = 0;
//Start initial state of program
State currState = DISABLED;
State prevState = DISABLED;


void setup() {
  //set up adc
  adc_init();
  //set up serial port
  U0init(9600);
  //initialize leds, buttons, potentiometer,and motor
  parts_init();
  thsensor.begin();
  lcd.begin(16, 2);
  stepper.setSpeed(10);
  Serial.begin(9600);

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //attach interrupt to button
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleToggleButton, FALLING);
}

void loop() {
  float temp = thsensor.readTemperature();
  wtrLevel = adc_read(0);
  handleToggleButton();
  updateLed(currState);
  switch (currState){
    case DISABLED:
      motorState = false;
      toggleMotor(false);
      lcd.setCursor(0, 0);
      lcd.print("System DISABLED!");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      break;
    case RUNNING:
      motorState = true;
      currentAirTempAndHumidity();
      toggleMotor(true);
      if(temp < temp_thresh){
        currState = IDLE;
      }
      if(wtrLevel <= WTR_THRESHOLD){ //if water level is at threshold or is too low, then go to error. in error state change led and print alert
        currState = ERROR;
      }
      
      break;
    case IDLE:
      motorState = false;
      
      currentAirTempAndHumidity();
      toggleMotor(false);
      if(temp >= temp_thresh){
        currState = RUNNING; //in RUNNING state we would probably call controlFanMotor(true);
      }
      if(wtrLevel < WTR_THRESHOLD){ //if water level is at threshold or is too low, then go to error. in error state change led and print alert
        currState = ERROR;
      }
      break;  
    case ERROR:
      motorState = false;
      toggleMotor(false);
      lcd.setCursor(0, 0);
      lcd.print("Water level low!");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print("default is being flagged");
      break;
  }
  //log motor change
  if(currState != DISABLED){
    adjustAngleForVent();
  }
  if(prevState != currState){
    logMotorState();
  }
  //logMotorState();
  prevState = currState;
}
 
void parts_init(){
  *DDR_A |= 0x01 << BLUE_PIN;
  *DDR_A |= 0x01 << RED_PIN;
  *DDR_A |= 0x01 << GREEN_PIN;
  *DDR_A |= 0x01 << YELLOW_PIN;

  *DDR_D &= ~(0x01 << BUTTON_PIN);
  *PORT_D |= (0x01 << BUTTON_PIN);
  *DDR_F &= ~(0x01 << POTENTIOMETER_PIN);
  
  *DDR_C |= 0x01 << MOTOR_PIN;
}

void updateLed(State cState){
  //reset and turn off all leds
  *PORT_A &= ~(0x01 << BLUE_PIN);
  *PORT_A &= ~(0x01 << RED_PIN);
  *PORT_A &= ~(0x01 << GREEN_PIN);
  *PORT_A &= ~(0x01 << YELLOW_PIN);

  //turn on appropriate led
  switch(cState){
    case DISABLED:
      *PORT_A |= 0x01 << YELLOW_PIN;
      break;
    case IDLE:
      *PORT_A |= 0x01 << GREEN_PIN;
      break;
    case ERROR:
      *PORT_A |= 0x01 << RED_PIN;
      break;
    case RUNNING:
      *PORT_A |= 0x01 << BLUE_PIN;
      break;
    default:
      char str[64];
      snprintf(str,64, "ERROR CHANGING LED\n");
      printStringSerial(str);
      break;
  }
}
void printStringSerial(char s[MAX_STR_SIZE]){
  for(int i = 0; s[i] != '\0'; i++){
    U0putchar(s[i]);
  }
}


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


void currentAirTempAndHumidity(){
  float temp = thsensor.readTemperature();
  float humidity = thsensor.readHumidity();

  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C   ");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}


bool adjustAngleForVent(){
  static unsigned long lastReadTime = 0;
  if(millis() - lastReadTime < 50){
    return false;
  }
  lastReadTime = millis();
  
  uint16_t potentiometerValue = adc_read(POTENTIOMETER_PIN);     
  int stepsMade = map(potentiometerValue, 0, 1023, 0, STEPS);
  int movement = stepsMade - lastSteppedPosition;
  if(abs(movement) > 5){     //check for a reasonable change in potentiometer value 
      stepper.step(movement);
      lastSteppedPosition = stepsMade;
      return true;
  }
  return false;
}


void handleToggleButton() {
  bool pressed = *PIN_D & (0x01 << BUTTON_PIN);
  if((!pressed)){
    switch(currState){
      case DISABLED: 
      case ERROR:
        currState = IDLE;
        break;
      case IDLE:
      case RUNNING:
        currState = DISABLED;
        break;
      default:
        break;
    }

  }
}
// turn on and off fan motor
void toggleMotor(bool motorOn) {
  if(motorOn){
    *PORT_C |= (0x01 << MOTOR_PIN);    
  }else{
    *PORT_C &= ~(0x01 << MOTOR_PIN);
  }
}

void logMotorState() {
  DateTime time = rtc.now();
  char dateC[MAX_STR_SIZE];
  String dateS = time.timestamp(DateTime::TIMESTAMP_FULL);
  dateS.toCharArray(dateC, MAX_STR_SIZE);
  if(currState == RUNNING){
    printStringSerial("MOTOR ON at ");
    printStringSerial(dateC);
    printStringSerial("\n");
  } else if(((prevState == RUNNING) && (currState == ERROR))||((prevState == RUNNING) && (currState == IDLE))||((prevState == RUNNING) && (currState == DISABLED))){
    printStringSerial("MOTOR OFF at ");
    printStringSerial(dateC);
    printStringSerial("\n");
  }
}

//CPE 301 Final Project Summer 2023
//Group Members:-  Saransh Beniwal, Michael Nia, and Wyatt Young
//Swamp Cooler Code

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <DHT.h>

//Stepper Motor pins
#define STEPPER_FAN_MOTOR_PIN1 10
#define STEPPER_FAN_MOTOR_PIN2 11
#define STEPPER_FAN_MOTOR_PIN3 12
#define STEPPER_FAN_MOTOR_PIN4 13

//LED pins
#define LED_PIN_RED 3
#define LED_PIN_YELLOW 3
#define LED_PIN_GREEN 5
#define LED_PIN_BLUE 5

//Humidity and temperature sensor pins
#define DHT_PIN 7
#define DHT_TYPE DHT11
#define WATER_LEVEL 5

//Fan motor pins
#define FAN_MOTOR_PIN 6

//Pins for the switch
#define SWITCH_TOP 2
#define SWITCH_BOTTOM 3
#define SWITCH_STEPPER_HIGH 4
#define SWITCH_STEPPER_LOW 5

//Set to 18 for Interrupt
#define SWITCH_STATUS 0

//When in error mode need to be able to reset to idle state
#define SWITCH_RESET 1

#define RDA 0x80
#define TBE 0x20

//Initializing the library with the interface pins
LiquidCrystal lcd(30, 31, 32, 33, 34, 35);

//Real Time Clock
RTC_DS1307 RTC;

//Temperature and Humidity
DHT dht(DHT_PIN, DHT_TYPE);

//Stepper Motor object
Stepper stepper(2048, STEPPER_FAN_MOTOR_PIN1, STEPPER_FAN_MOTOR_PIN2, STEPPER_FAN_MOTOR_PIN3, STEPPER_FAN_MOTOR_PIN4);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

//Machine States
enum PossibleMyMachineStates {
  IDLE, DISABLED, RUNNING, ERROR, START,
};

//Temperature sensor threshold
#define TEMPERATURE_THRESH 20
//20 Works

//Water sensor threshold
#define WATER_THRESH 320
//100 Works
//320

//Defining the initial state for the state machine
PossibleMyMachineStates currentMyMachineState = DISABLED;
PossibleMyMachineStates prevMyMachineState = START;

void setup() {
  //Initializing the pins that are used for the communication with the RTC module
  PORTD |= (1 << PD0) | (1 << PD1);
  RTC.begin();

  //Set the correct time on the RTC module
  DateTime now = DateTime(23, 7, 30, 0, 0, 0);
  RTC.adjust(now);

  DDRE |= (0x01 << LED_PIN_GREEN | 0x01 << LED_PIN_YELLOW);
  DDRG |= (0x01 << LED_PIN_BLUE);
  DDRH |= (0x01 << LED_PIN_RED | 0x01 << WATER_LEVEL | 0x01 << FAN_MOTOR_PIN);

  //Read
  adc_init();

  //Initialize the serial port on USART0
  U0init(9600);
  dht.begin();

  //Setting LCD's columns and rows
  lcd.begin(16, 2);
  lcd.print("System is Starting!Woohoo!");

  //Limiting the switch setting
  DDRA &= ~(0x01 << SWITCH_RESET | 0x01 << SWITCH_STATUS | 0x01 << SWITCH_TOP | 0x01 << SWITCH_BOTTOM | 0x01 << SWITCH_STEPPER_HIGH | 0x01 << SWITCH_STEPPER_LOW);

  //Attaching an interrupt to the switch pin
  attachInterrupt(digitalPinToInterrupt(SWITCH_STATUS), handleMovement, RISING);
  stepper.setSpeed(12);
}

//Looping  the updated values
int lastTemperaturePrint = 0;
double temperature = 0;
double humidity = 0;

//Stepper Motor Speed
int stepperMotorRating = 2048;

//Machine Flags
bool fanMyMachineState = false;
int LEDCurrent = -1;
bool printTemperatureHumidity = false;
bool stepperMotorMyMachineState = false;
bool WaterLevelMonitoring = false;

void loop() {
  //Reading the time from the RTC module
  DateTime now = RTC.now();
  if (printTemperatureHumidity) {
    //Reading the temperature and humidity from the DHT sensor
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
  }

  currentMyMachineState = currentMyMachineStateOfSystem(temperature, humidity, currentMyMachineState);

  if (readMyPin(SWITCH_STATUS)) {
    handleMovement();
  }

  //When state is switched
  if (currentMyMachineState != prevMyMachineState) {
    transitionMyMachineState(now, prevMyMachineState, currentMyMachineState);

    //Check the current state of the state machine
    switch (currentMyMachineState) {

      case DISABLED:

        fanMyMachineState = false;
        LEDCurrent = 3;
        printTemperatureHumidity = false;
        stepperMotorMyMachineState = true;
        WaterLevelMonitoring = false;
        break;

      case IDLE:
        fanMyMachineState = false;
        LEDCurrent = 2;
        printTemperatureHumidity = true;
        stepperMotorMyMachineState = true;
        WaterLevelMonitoring = true;
        break;

      case RUNNING:
        fanMyMachineState = true;
        LEDCurrent = 1;
        printTemperatureHumidity = true;
        stepperMotorMyMachineState = true;
        WaterLevelMonitoring = true;
        break;

      case ERROR:
        lcd.clear();
        lcd.print("Error! Water level is low.");
        fanMyMachineState = false;
        LEDCurrent = 0;
        printTemperatureHumidity = true;
        stepperMotorMyMachineState = false;
        WaterLevelMonitoring = true;
        break;

      case START:
        break;
    }
  }

  //Seting stepper motor rate
  if (stepperMotorMyMachineState) {
    //Determining direction, the limits and set that speed
    int stepperMotorWorking = stepperMotorRating*(readMyPin(SWITCH_STEPPER_HIGH)?1:readMyPin(SWITCH_STEPPER_LOW)?-1:0);
    //Checking motor limits
    stepperMotorWorking = (readMyPin(SWITCH_TOP)?min(stepperMotorWorking, 0):(readMyPin(SWITCH_BOTTOM)?max(stepperMotorWorking, 0):stepperMotorWorking));
    if (stepperMotorWorking != 0) {
      stepperMotorMovement(now, prevMyMachineState, currentMyMachineState);
    }
    setStepperMotor(stepperMotorWorking);
  }

  //Setting fan rate
  setFanMotor(fanMyMachineState);

  //Setting LED
  turnLEDS(LEDCurrent);

  //Displaying temperature and humidity
  if (printTemperatureHumidity && abs(lastTemperaturePrint - now.minute()) >= 1) {

    lcd.clear();
    //Updated pervious values
    lastTemperaturePrint = now.minute();
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    lcd.print("Temperature, Humidity");
    delay(900);
    lcd.clear();
    lcd.print(temperature);
    lcd.print(humidity);
  }

  prevMyMachineState = currentMyMachineState;

  //Check water level
  if (WaterLevelMonitoring) {
    //Calculate water level
    int waterLevelCheck = adc_read(WATER_LEVEL);
    if (waterLevelCheck <= WATER_THRESH) {
      currentMyMachineState = ERROR;
    }
  }
  delay(600);
}

int waterLevelRead(int pin) {
  return PINH & (0x01 << pin);
}

int readMyPin(int pin) {
  //Read the value of the pin using the PIN register
  return PINA & (0x01 << pin);
}

bool switchStateOfMachine = true;

//If handle is off wwhen switch is pressed
void handleMovement() {
  prevMyMachineState = currentMyMachineState;
  bool myButtonWorks = readMyPin(SWITCH_STATUS);

  if (switchStateOfMachine && myButtonWorks) {
    currentMyMachineState = IDLE;
    switchStateOfMachine = false;
  } else if (myButtonWorks) {
    currentMyMachineState = DISABLED;
    switchStateOfMachine = true;
  }
}

//Writing states to the LCD
void stepperMotorMovement(DateTime now, PossibleMyMachineStates prevMyMachineState, PossibleMyMachineStates currentMyMachineState) {
  U0putchar('S'); U0putchar('T'); U0putchar('E'); U0putchar('P'); U0putchar('P'); U0putchar('E'); U0putchar('R');
  U0putchar(' ');
  transitionMyMachineState(now, prevMyMachineState, currentMyMachineState);
}

//Writing states to the LCD
void transitionMyMachineState(DateTime now, PossibleMyMachineStates prevMyMachineState, PossibleMyMachineStates currentMyMachineState) {
  unsigned char presentMyMachineStateMachine = (prevMyMachineState == DISABLED ? 'd' : (prevMyMachineState == IDLE ? 'i' : (prevMyMachineState == RUNNING ? 'r' : (prevMyMachineState == ERROR ? 'e' : 'u'))));
  unsigned char currentMyMachineStateMachine = (currentMyMachineState == DISABLED ? 'd' : (currentMyMachineState == IDLE ? 'i' : (currentMyMachineState == RUNNING ? 'r' : (currentMyMachineState == ERROR ? 'e' : 'u'))));

  U0putchar(presentMyMachineStateMachine);
  U0putchar(':');
  U0putchar(currentMyMachineStateMachine);
  U0putchar(' ');

  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char digits[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

  int dateOnesYear = year % 10;
  int dateTensYear = year / 10 % 10;

  int dateOnesMonth = month % 10;
  int dateTensMonth = month / 10 % 10;

  int dateOnesDay = day % 10;
  int dateTensDay = day / 10 % 10;

  int timeOnesHour = hour % 10;
  int timeTensHour = hour / 10 % 10;

  int timeOnesMinute = minute % 10;
  int timeTensMinute = minute / 1010;

  int timeOnesSecond = second % 10;
  int timeTensSecond = second / 10 % 10;

  //Printing Time and Date Sequentially
  U0putchar('M'); U0putchar(':'); U0putchar('D'); U0putchar(':'); U0putchar('Y');
  U0putchar(' ');

  U0putchar('H'); U0putchar(':'); U0putchar('M'); U0putchar(':'); U0putchar('S');
  U0putchar(' ');

  U0putchar(digits[dateTensMonth]);  U0putchar(digits[dateOnesMonth]);
  U0putchar(':');

  U0putchar(digits[dateTensDay]); U0putchar(digits[dateOnesDay]);
  U0putchar(':');

  U0putchar(digits[dateTensYear]); U0putchar(digits[dateOnesYear]);
  U0putchar(' ');

  U0putchar(digits[timeTensHour]); U0putchar(digits[timeOnesHour]);
  U0putchar(':');

  U0putchar(digits[timeTensMinute]); U0putchar(digits[timeOnesMinute]);
  U0putchar(':');

  U0putchar(digits[timeTensSecond]); U0putchar(digits[timeOnesSecond]);

  U0putchar('\n');
}

PossibleMyMachineStates currentMyMachineStateOfSystem(double temperature, int waterLevelCheck, PossibleMyMachineStates currentMyMachineState) {
  PossibleMyMachineStates myMachineState;

  if (temperature <= TEMPERATURE_THRESH && currentMyMachineState == RUNNING) {
    myMachineState = IDLE;
  } else if (temperature > TEMPERATURE_THRESH && currentMyMachineState == IDLE) {
    myMachineState = RUNNING;
  } else if (currentMyMachineState == ERROR && readMyPin(SWITCH_RESET) && waterLevelCheck > WATER_THRESH) {
    myMachineState = IDLE;
  } else {
    myMachineState = currentMyMachineState;
  }
  return myMachineState;
}

//Turn on and off the specific LED
void turnLEDS(int ledPin) {
  //Turning off all of the LEDs using a bitwise AND operation
  PORTH &= ~(0x01 << LED_PIN_RED);
  PORTG &= ~(0x01 << LED_PIN_BLUE);
  PORTE &= ~(0x01 << LED_PIN_GREEN);
  PORTE &= ~(0x01 << LED_PIN_YELLOW);

  //Turning the specified LED using a bitwise OR
  switch (ledPin) {
    case 0:
      PORTH |= 0x01 << LED_PIN_RED;
      break;
    case 1:
      PORTG |= 0x01 << LED_PIN_BLUE;
      break;
    case 2:
      PORTE |= 0x01 << LED_PIN_GREEN;
      break;
    case 3:
      PORTE |= 0x01 << LED_PIN_YELLOW;
      break;
  }
}

//Setting stepper motor speed
void setStepperMotor(int maxStepperSpeed) {
  stepper.step(maxStepperSpeed);
}

//Turning fan motor fan motor on/off
void setFanMotor(bool fanWorking) {
  if (fanWorking) {
    PORTH |= (0x01 << FAN_MOTOR_PIN);
  } else {
    PORTH &= ~(0x01 << FAN_MOTOR_PIN);
  }
}

void adc_init() {
  ADCSRA = 0x80;
  ADCSRB = 0x00;
  ADMUX = 0x40;
}

unsigned int adc_read(unsigned char read_adc) {
  ADCSRB &= 0xF7;
  ADCSRB |= (read_adc & 0x08);

  ADMUX &= 0xF8;
  ADMUX |= (read_adc & 0x07);

  //Start the fan
  ADCSRA |= 0x40;
  while (ADCSRA & 0x40) {}
  //Return the fan number
  return ADC;
}

void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;

  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

// Read USART0 RDA status bit and return nfanWorking-zero true if set
unsigned char U0kbhit() {
  return (RDA & *myUCSR0A);
}

// Read input character from USART0 input buffer
unsigned char U0getchar() {
  return *myUDR0;
}

// Wait for USART0 TBE to be set then write character to transmit buffer
void U0putchar(unsigned char U0pdata) {
  while (!(TBE & *myUCSR0A))
    ;
  *myUDR0 = U0pdata;
}

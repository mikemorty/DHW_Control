
/*
----------------------------------------------------

   DHW                     10K
1.2 to 1.5v o---/\/\/--.--/\/\/---o GND Arduino
                  |         |
 Pin 0 o-----------         ------o GND DHW

----------------------------------------------------
*/

const int onBoardLedPin = 13;       // the pin numbers for the LEDs
const int On_off_Pin = 10;
const int powervent_Pin = 7;     // the number of the powervent relay pin
const int buttonPin = 4;            // the pin number for the button

const int voltPin = A2;     // the number of the voltPin input pin
const int tempPin = A1;     // the number of the tempPin input pin
const int dhw_Pin = A0;     // the number of the demand hot water input pin

const int buttonInterval = 1000; // number of millisecs between button readings
const int blinkDuration = 500; // number of millisecs that Led's are on - all three leds use this
const int onBoardLedInterval = 500; // number of millisecs between blinks
const int temp_delay = 1000;           // number of millisecs in the delay
const int volt_delay = 1000;           // number of millisecs in the delay
const int powervent_delay = 5000;      // number of millisecs in the delay

byte On_off_State = LOW;
byte onBoardLedState = LOW;             // used to record whether the LEDs are on or off
byte powervent_State = LOW;                   // used to record whether the relays are on or off

unsigned long previousTempSample = 0;
unsigned long previousVoltSample = 0;
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long previousOnBoardLedMillis = 0;   // will store last time the LED was updated
unsigned long previousButtonMillis = 0; // time when button press last checked
unsigned long powerventStart;

void setup() {

  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(On_off_Pin, OUTPUT);
  pinMode(onBoardLedPin, OUTPUT);
  pinMode(powervent_Pin, OUTPUT);  // initialize the powervent_Pin as an output:
}

//========================================

void loop() {

  currentMillis = millis();
  readButton();               // call the functions that do the work
  updateOnBoardLedState();
  powervent();         // call the functions that do the work
  switchRelays();
  temp();
  volts();
}

void switchRelays() {
  digitalWrite(onBoardLedPin, onBoardLedState);
  digitalWrite(On_off_Pin, On_off_State);
  digitalWrite(powervent_Pin, powervent_State);     // this is the code that actually switches the relay on and off
}

//========================================

void updateOnBoardLedState() {

  if (onBoardLedState == LOW) {
    // if the Led is off, we must wait for the interval to expire before turning it on
    if (currentMillis - previousOnBoardLedMillis >= onBoardLedInterval) {
      onBoardLedState = HIGH;
      previousOnBoardLedMillis += onBoardLedInterval;
    }
  }
  else {  // i.e. if onBoardLedState is HIGH

    // if the Led is on, we must wait for the duration to expire before turning it off
    if (currentMillis - previousOnBoardLedMillis >= blinkDuration) {
      // time is up, so change the state to LOW
      onBoardLedState = LOW;
      // and save the time when we made the change
      previousOnBoardLedMillis += blinkDuration;
    }
  }
}

//========================================

void powervent() {

  int val = analogRead(dhw_Pin);
  if (val >= 180 )       // check if the input is greater than 180
  {
    powervent_State = HIGH;   // turn powervent_Pin, HIGH
    powerventStart = millis();
  }
  if ( millis() - powerventStart >= powervent_delay) {
    powervent_State = LOW;
  }
}

//========================================

void temp() {
  
  int reading = analogRead(tempPin);
  if (currentMillis - previousTempSample >= temp_delay)
  {
    previousTempSample += temp_delay;
    // add code to take temperature reading here
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    // now print out the temperature
    float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
    //to degrees ((voltage - 500mV) times 100)
    Serial.print(temperatureC); Serial.println(" degrees C");
  }
}

//========================================

void readButton() {

  if (millis() - previousButtonMillis >= buttonInterval) {

    if (digitalRead(buttonPin) == LOW) {
      On_off_State = ! On_off_State; // this changes it to LOW if it was HIGH
      Serial.print(On_off_State);
      //   and to HIGH if it was LOW
      previousButtonMillis += buttonInterval;
    }
  }
}

//========================================

void volts() {
  
  #define NUM_SAMPLES 10
  int sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number

  if (currentMillis - previousVoltSample >= volt_delay)
  {
    previousVoltSample += volt_delay;
    // add code to take temperature reading here
    float voltage = 0.0;            // calculated voltage
    // take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
      sum += analogRead(voltPin);
      sample_count++;
    }
    voltage = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;
    Serial.print(voltage);
    Serial.println (" Volts");
    sample_count = 0;
    sum = 0;
  }
}

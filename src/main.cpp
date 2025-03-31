#include <Arduino.h>
#include <fft.h>
#include <ESP32Servo.h>
#include <SimpleKalmanFilter.h>
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>

const uint16_t samples = 4096; // MAKE SURE that this is a power of 2 (very important)
const double samplingFrequency = 38000; // fine tuned no touchy pls

boolean clipping = 0;

//data storage variables
uint16_t newData = 0;
uint16_t prevData = 0;
unsigned int elapsedTime = 0;//keeps elapsedTime and sends vales to store in elapsedTimer[] occasionally
int elapsedTimer[10];//sstorage for timing of events
int slope[10];//storage for slope of events
unsigned int totalelapsedTimer;//used to calculate period
unsigned int period;//storage for period of wave
uint16_t currentIndex = 0;//current storage currentIndex
float frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for decided whether you have a match
uint16_t noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
uint slopeTol = 3;//slope tolerance- adjust this if you need
int elapsedTimerTol = 10;//elapsedTimer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampelapsedTimer = 0;
uint maxAmp = 0;
uint checkMaxAmp;
uint ampThreshold = 30;//raise if you have a very noisy signal


// ESP32 pin
#define ADC_PIN 34 
#define STRING_SELECTOR_PIN 27  
#define BUTTON_CONFIG_PIN 12    
#define BUTTON_TUNE_PIN 13      
#define TUNER_SERVO_PIN 14  

// signal parameters
#define TOLERANCE 2  
#define REFERENCE_SPEED 343.2 

bool isTuningEnabled = false;
bool isDisplayingTuningMode = false;
int currentConfig = 0;
int selectedString = 0;

// BMP280, Servo, Kalman Filter, and LCD objects
Adafruit_BMP280 bmp;
Servo tuningServo;
SimpleKalmanFilter kalmanFilter(1, 1, 0.01);
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

struct Tuning {
  const char name[16];
  double frequencies[6];
};

// can totally add more :)
Tuning tuningConfigs[] = {
  {"Standard", {82.41, 110.00, 146.83, 196.00, 246.94, 329.63}},
  {"Half Step Down", {77.78, 104.00, 139.69, 185.00, 233.08, 311.13}},
  {"Drop D", {73.42, 110.00, 146.83, 196.00, 246.94, 329.63}},
  {"Open G", {82.41, 98.00, 123.47, 196.00, 246.94, 329.63}},
  {"Open D", {73.42, 98.00, 146.83, 185.00, 246.94, 329.63}}
};

// debouncing things
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 100;

void handleButtons();
void getFrequency();
double getPeriod();
double getSpeedOfSound();
double compensateFrequency(double measuredFreq);
void lcdDisplay(double frequency, double targetFreq);
void reset();
// void adjustServo(double frequency, double targetFreq);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  lcd.begin(16, 2);
  lcd.print("Tuner Ready!");
  tuningServo.attach(TUNER_SERVO_PIN);

  // pulldowns
  pinMode(BUTTON_CONFIG_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_TUNE_PIN, INPUT_PULLDOWN);
  pinMode(STRING_SELECTOR_PIN, INPUT_PULLDOWN);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 sensor not found!");
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby elapsedTime. */
}

void loop() {
  handleButtons();
  
  if (!isDisplayingTuningMode && isTuningEnabled) { 
    getFrequency();
    double compensatedFreq = compensateFrequency(frequency);
    float temp = bmp.readTemperature();
    double targetFreq = tuningConfigs[currentConfig].frequencies[selectedString];
    lcdDisplay(frequency, targetFreq);
    // adjustServo(frequency, targetFreq);
  }
  Serial.print("Detected Frequency: ");
  Serial.println(frequency, 2);

}

void handleButtons() {
  unsigned long currentMillis = millis();

  if (digitalRead(BUTTON_CONFIG_PIN) == HIGH && currentMillis - lastButtonPress > debounceDelay) {
    currentConfig = (currentConfig + 1) % (sizeof(tuningConfigs) / sizeof(tuningConfigs[0]));
    isDisplayingTuningMode = true;
    Serial.print("Selected Tuning: ");
    Serial.println(tuningConfigs[currentConfig].name);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tuning Mode:");
    lcd.setCursor(0, 1);
    lcd.print(tuningConfigs[currentConfig].name);
    
    lastButtonPress = currentMillis;
  }

  if (digitalRead(BUTTON_TUNE_PIN) == HIGH && currentMillis - lastButtonPress > debounceDelay) {
    isTuningEnabled = !isTuningEnabled;
    isDisplayingTuningMode = false;
    Serial.print("Tuning: ");
    Serial.println(isTuningEnabled ? "ON" : "OFF");
    lastButtonPress = currentMillis;
  }

  if (digitalRead(STRING_SELECTOR_PIN) == HIGH && currentMillis - lastButtonPress > debounceDelay) {
    selectedString = (selectedString + 1) % 6;
    Serial.print("Selected String: ");
    Serial.println(selectedString + 1);
    lastButtonPress = currentMillis;
  }
}

void getFrequency() {
  frequency = 38462/float(frequency);
}

double getSpeedOfSound() {
  double temperature = bmp.readTemperature();
  double humidity = 50.0;  
  double speedOfSound = 331.3 + (0.606 * temperature) + (0.0124 * humidity);
  return speedOfSound;
}

double compensateFrequency(double measuredFreq) {
  double actualSpeed = getSpeedOfSound();
  return measuredFreq * (actualSpeed / REFERENCE_SPEED);
}

void lcdDisplay(double frequency, double targetFreq) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  lcd.print(frequency, 2);
  lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print("Match: ");
  lcd.print(targetFreq, 2);
  lcd.print(" Hz");
}

// void adjustServo(double frequency, double targetFreq) {
//   int servo_angle = tuningServo.read();
//   int step = 2;

//   while (abs(frequency - targetFreq) > TOLERANCE) {
//     if (frequency < targetFreq - TOLERANCE) {
//       servo_angle += step; // tighten peg
//     } else if (frequency > targetFreq - TOLERANCE) {
//       servo_angle -= step; // loosen peg
//     }

//     servo_angle = constrain(servo_angle, 0, 180); // keep within 0 - 180
//     tuningServo.write(servo_angle);
//     delay(50); // wait a bit

//     // recheck
//     frequency = getFrequency();

//     if (abs(frequency - targetFreq) > 5) {
//       step = 1;
//     }
//   }
//   Serial.println("string in tune yayyy");
// }

double getPeriod() {
  
  prevData = newData; //store previous value
  newData = analogRead(34); // data from PZ sensor

  if (prevData < 2047 && newData >= 2047) { // if increasing and crossing midpoint
    newSlope = newData - prevData; // calculate slope
    if (abs(newSlope - maxSlope) < slopeTol) { // if slopes are equal:

      //record new data and reset elapsedTime
      slope[currentIndex] = newSlope;
      elapsedTimer[currentIndex] = elapsedTime;
      elapsedTime = 0;
      if (currentIndex == 0){//new max slope just reset
        noMatch = 0;
        currentIndex++;//increment currentIndex
      }
      else if (abs(elapsedTimer[0] - elapsedTimer[currentIndex]) < elapsedTimerTol && abs(slope[0] - newSlope) < slopeTol) { // if elapsedTimer duration and slopes match
        //sum elapsedTimer values
        totalelapsedTimer = 0;
        for (byte i=0; i < currentIndex; i++){
          totalelapsedTimer += elapsedTimer[i];
        }
        period = totalelapsedTimer;//set period

        //reset new zero currentIndex values to compare with
        elapsedTimer[0] = elapsedTimer[currentIndex];
        slope[0] = slope[currentIndex];
        currentIndex = 1; //set currentIndex to 1
        noMatch = 0;
      }
      else { // crossing midpoint but not match
        currentIndex++; // increment currentIndex
        if (currentIndex > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope) { // if new slope is much larger than max slope
      maxSlope = newSlope;
      elapsedTime = 0;//reset clock
      noMatch = 0;
      currentIndex = 0;//reset currentIndex
    }
    else{ // slope not steep enough
      noMatch++; // increment no match counter
      if (noMatch > 9) {
        reset();
      }
    }
  }
    
  if (newData == 0 || newData == 4095) { //if clipping
    clipping = 1; // currently clipping
  }
  
  elapsedTime++; // increment elapsedTimer at rate of 38.5kHz
  
  ampelapsedTimer++; // increment amplitude elapsedTimer
  if (abs(2045 - analogRead(34)) > maxAmp) {
    maxAmp = abs(2045 - analogRead(34));
  }
  if (ampelapsedTimer == 1000) {
    ampelapsedTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }

  return period;
}

void reset(){ // clean  out some variables
  currentIndex = 0; // reset currentIndex
  noMatch = 0; // reset match couner
  maxSlope = 0; // reset slope
}

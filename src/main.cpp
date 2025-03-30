#include <Arduino.h>
#include <fft.h>
#include <ESP32Servo.h>
#include <SimpleKalmanFilter.h>
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>

const uint16_t samples = 4096; // MAKE SURE that this is a power of 2 (very important)
const double samplingFrequency = 38000; // fine tuned no touchy pls

// ESP32 pin
#define ADC_PIN 34 
#define STRING_SELECTOR_PIN 27  
#define BUTTON_CONFIG_PIN 12    
#define BUTTON_TUNE_PIN 13      
#define TUNER_SERVO_PIN 14  

// signal parameters
#define SIGNAL_THRESHOLD 200
#define TOLERANCE 2  
#define REFERENCE_SPEED 343.2 

bool isTuningEnabled = false;
bool isDisplayingTuningMode = false;
int currentConfig = 0;
int selectedString = 0;

// real and imag samples for FFT
float vReal[samples];
float vImag[samples];

// BMP280, Servo, FFT, Kalman Filter, and LCD objects
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
double getFrequency();
double getSpeedOfSound();
double compensateFrequency(double measuredFreq);
void lcdDisplay(double frequency, double targetFreq);
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
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  handleButtons();
  
  if (!isDisplayingTuningMode && isTuningEnabled) { 
    double frequency = getFrequency();
    double compensatedFreq = compensateFrequency(frequency);
    float temp = bmp.readTemperature();
    double targetFreq = tuningConfigs[currentConfig].frequencies[selectedString];
    lcdDisplay(frequency, targetFreq);
    // adjustServo(frequency, targetFreq);
  }
  double f = getFrequency();
  double t = bmp.readTemperature();
  Serial.print("Detected Frequency: ");
  Serial.println(f, 2);
  Serial.print("Temp: ");
  Serial.println(t);
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

double getFrequency() {
  for (uint16_t i = 0; i < samples; i++) {
    int rawSensorValue = analogRead(ADC_PIN);
    vReal[i] = kalmanFilter.updateEstimate(rawSensorValue);
    vImag[i] = 0.0;
  }
  // Perform FFT
  float twiddle_factors[samples];
  fft_config_t* fft_config = fft_init(samples, FFT_REAL, FFT_FORWARD, vReal, vImag);
  fft_execute(fft_config);

  // Calculate magnitudes and find the major peak
  double maxMag = 0;
  int peakIndex = 0;
  for (int i = 1; i < samples / 2; i++) {
    float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
    if (magnitude > maxMag) {
      maxMag = magnitude;
      peakIndex = i;
    }
  }

  double peakFrequency = peakIndex * samplingFrequency / samples;
  fft_destroy(fft_config); // Clean up
  return peakFrequency;
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
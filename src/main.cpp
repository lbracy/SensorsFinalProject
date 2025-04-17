#include <Arduino.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>
#include <SimpleKalmanFilter.h>
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
// #include <ThingSpeak.h>

// wifi connection
const char* ssid = "PAWS-Secure";
const char* password = "";

// ESP32 pins
#define ADC_PIN 34 
#define STRING_SELECTOR_PIN 27  
#define BUTTON_CONFIG_PIN 12    
#define BUTTON_TUNE_PIN 13      
#define TUNER_SERVO_PIN 14  

// signal parameters
#define TOLERANCE 2  
#define REFERENCE_SPEED 343.2 
float frequency = 0;


bool isTuningEnabled = false;
bool isDisplayingTuningMode = false;
int currentConfig = 0;
int selectedString = 0;

const uint16_t SAMPLES = 4096;   // Number of FFT samples (power of 2)
const double SAMPLING_FREQUENCY = 8000; // Sampling frequency in Hz 
float vReal[SAMPLES];  // Real part of FFT input
float vImag[SAMPLES];  // Imaginary part of FFT input

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

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
void waitForStableSignal();
double getFrequencyFFT();
double getSpeedOfSound();
double compensateFrequency(double measuredFreq);
void lcdDisplay(double frequency, double targetFreq);
void reset();
void adjustServo(double frequency, double targetFreq);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  //   }
  // Serial.println("Connected to WiFi");
  // WiFiClient client;
  // unsigned long myChannelNumber = 2824966;
  // const char * myWriteAPIKey = "FPEIQA6RV3WTZD12";

  // ThingSpeak.begin(client);

  lcd.begin(16, 2);
  lcd.print("Tuner Ready!");
  tuningServo.attach(TUNER_SERVO_PIN);

  // pulldowns
  pinMode(BUTTON_CONFIG_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_TUNE_PIN, INPUT_PULLDOWN);
  pinMode(STRING_SELECTOR_PIN, INPUT_PULLDOWN);

  tuningServo.write(90);

  if (!bmp.begin(0x77)) {
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
    double compensatedFreq = compensateFrequency(frequency);
    float temp = bmp.readTemperature();
    double targetFreq = tuningConfigs[currentConfig].frequencies[selectedString];
    frequency = getFrequencyFFT();
    lcdDisplay(compensatedFreq + 3, targetFreq);
    // adjustServo(frequency, targetFreq);

  }

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

double getFrequencyFFT() {
  double fundamental = 0;
  double prev_freq = 0;

  // 1) Sample
  unsigned long microsBetween = 1000000UL / SAMPLING_FREQUENCY;
  unsigned long lastMicros = micros();

  if (analogRead(ADC_PIN) >= 200) {

    Serial.println("ADC Value:");
    Serial.println(analogRead(34));
    Serial.println("Starting sample");

    for (int i = 0; i < SAMPLES; i++) {
      while (micros() - lastMicros < microsBetween);
      lastMicros += microsBetween;
      int a = analogRead(ADC_PIN);
      vReal[i] = a;
      vImag[i] = 0.0;
    }

    Serial.println("Finished sampling");

    // 2) FFT
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    // 3) Find top-3 peaks
    double peakMag[3]  = {0, 0, 0};
    double peakFreq[3] = {0, 0, 0};
    double binWidth = SAMPLING_FREQUENCY / double(SAMPLES);
    int half = SAMPLES / 2;

    for (int i = 1; i < half; i++) {
      double mag = vReal[i];
      double f = i * binWidth;

      if (f < 70.0 || f > 370.0) continue;

      if (mag > peakMag[0]) {
        peakMag[2] = peakMag[1]; peakFreq[2] = peakFreq[1];
        peakMag[1] = peakMag[0]; peakFreq[1] = peakFreq[0];
        peakMag[0] = mag;        peakFreq[0] = f;
      }
      else if (mag > peakMag[1]) {
        peakMag[2] = peakMag[1]; peakFreq[2] = peakFreq[1];
        peakMag[1] = mag;        peakFreq[1] = f;
      }
      else if (mag > peakMag[2]) {
        peakMag[2] = mag;
        peakFreq[2] = f;
      }
    }

    // 4) Choose fundamental (lowest of top 3)
    fundamental = peakFreq[0];
    for (int k = 1; k < 3; k++) {
      if (peakFreq[k] > 0 && peakFreq[k] < fundamental) {
        fundamental = peakFreq[k];
      }
    }

    // 5) Harmonic check (div by 2 or 3)
    double adjustedFreq = fundamental;
    int binFund = round(fundamental / binWidth);
    int binDiv2 = round((fundamental / 2.0) / binWidth);
    int binDiv3 = round((fundamental / 3.0) / binWidth);

    if (binDiv2 > 1 && vReal[binDiv2] > (0.5 * vReal[binFund])) {
      adjustedFreq = fundamental / 2.0;
    } else if (binDiv3 > 1 && vReal[binDiv3] > (0.5 * vReal[binFund])) {
      adjustedFreq = fundamental / 3.0;
    }

    prev_freq = adjustedFreq;
    Serial.print("Corrected Frequency: ");
    Serial.println(adjustedFreq);

  } else {
    lcdDisplay(prev_freq, tuningConfigs[currentConfig].frequencies[selectedString]);
  }

  return prev_freq;
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

void adjustServo(double currentFreq, double targetFreq) {
    static int servo_angle = tuningServo.read();

    double error = targetFreq - currentFreq;  // Frequency error
    double K = 25.0;  // Experimentally determined gain (adjust this value)
    
    // Use logarithmic function to determine peg rotation needed
    double deltaTheta = K * log(1 + abs(error) / targetFreq);

    if (abs(error) <= TOLERANCE) {
        Serial.println("Frequency matched! Stopping tuning...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("In tune!");
        return;  // Exit the function early to stop adjusting the servo
    }
    
    // Adjust tuning peg direction based on frequency error
    if (error > 0) {
        servo_angle += deltaTheta; // Increase tension (tighten)
    } else if (error < 0) {
        servo_angle -= deltaTheta; // Decrease tension (loosen)
    }

    // Constrain the servo range
    servo_angle = constrain(servo_angle, 0, 180);
    tuningServo.write(servo_angle);
    delay(10);

    Serial.print("Servo Angle: ");
    Serial.println(servo_angle);

    if (servo_angle == 180 || servo_angle == 0) {
        Serial.println("Servo at limit! Resetting...");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Servo Resetting...");
        delay(3000);  // Wait for 3 seconds

        tuningServo.write(90);  // Reset servo to middle position
        servo_angle = 90;       // Update stored angle
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Reseting");
        lcd.setCursor(0, 1);
        lcd.print("Tuning...");
        delay(2000);  // Wait for 2 seconds
      } else {
      Serial.println("String in tune yayyy!");
    }
}

// void uploadThingSpeak() {
//   ThingSpeak.setField(1, analogRead(34));
//   int x = ThingSpeak.writeFields(2824966, "FPEIQA6RV3WTZD12");
//   if (x == 200){
//     Serial.println("Channel update successful.");
//   }
//   else {
//     Serial.println("Problem updating channel. HTTP error code " + String(x));
//   }
// }
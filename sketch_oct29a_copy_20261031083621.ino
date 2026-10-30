
/*
 FALL DETECTOR  -  Arduino Uno R4 wifi + MPU6050 + momentary push button + Blynk IoT


 Wiring
 ------
 MPU6050 : VCC->5V, GND->GND, SDA->A4, SCL->A5 (these are the wire connections from breadboard + MPU6050 to Arduino UNO r4 wifi)
 Button  : connect one wire to 7 (for line 45 to work) - (not pressed = LOW, pressed = HIGH)


Libraries
Wire.h: allows Arduino to communicate with I2C devices—such e.g. sensors, MPU6050 (when connected to SDA & SCL pins), and OLED/LCD displays.
SoftwareSerial.h: turns Arduino pins into extra USB-like communication channels (e.g. Bluetooth module, a GPS, or a Wi-Fi chip)
WiFiS3.h: connects Arduino to the internet using Wi-Fi - as the model I am using is Arduino UNO R4 Wifi (it has a built-in ESP32-S3 Wi-Fi chip)
BlynkSimpleWifi.h = links Arduino to the BLYNK app over Wi-Fi (evident in lines 27-31)

 BLYNK 
 Create Event on app/website with event code:  fall.alert (enable notifications for it)
 ALSO - ensure you go to developer zone > datastream > then make 2 seperate datastreams. One will have pin: V1 (integer) and other will be pin: V2 (integer) 
 ^ this ensures data comes through to BLYNK IoT 
*/


// Blynk credentials (must come BEFORE the libraries included) 
#define BLYNK_TEMPLATE_ID   "TMPL6cuxxpts_"
#define BLYNK_TEMPLATE_NAME "fall detection1"
#define BLYNK_AUTH_TOKEN    "2FuRb7DCnWi04rMtxlmOhB8Xd7YqmR7e"
//#define BLYNK_PRINT Serial  

// these are the libraries needed
#include <Wire.h>
#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <BlynkSimpleWifi.h>


// wifi details
char ssid[] = "Optus_9182DE";
char pass[] = "grebemashyXqnuE";

// Pins - for push button to transmit data to Arduino
const int BUTTON_PIN = 7;            // pressed = HIGH

// MPU6050 - gyroscope and accelerometer - converts MPU6050 raw readings into units (g = acceleration, degrees/sec = rotation, and radians to degrees)
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB_PER_G  = 4096.0; // +/-8g range
const float GYRO_LSB_PER_DPS = 65.5; // +/-500 deg/s range
const float RAD2DEG = 57.2958;


// Timing - e.g. how long for the countdown, how long for arduino to recognise fall/ other physical activity
const unsigned long SAMPLE_PERIOD_MS   = 20;      // 50 Hz
const unsigned long WINDOW_MS          = 1000;    // activity classification window
const unsigned long FREEFALL_TIMEOUT   = 500;     // max wait for impact after free-fall
const unsigned long MONITOR_MS         = 2000;    // post-impact observation time
const unsigned long SETTLE_MS          = 500;     // ignore the impact bounce
const unsigned long COUNTDOWN_MS       = 30000UL; // 30 s to cancel
const unsigned long BUTTON_DEBOUNCE_MS = 50;


// Thresholds (tune these for where/how the device is worn - mine will be worn on a waistband). This details the gravity values 
const float FREEFALL_THRESH  = 0.4;   // g  : below = weightlessness
const float IMPACT_THRESH    = 2.5;   // g  : above = hard impact
const float WALK_MIN_PEAK    = 1.2;   // g  : walking
const float RUN_MIN_PEAK     = 1.8;   // g  : running
const float STILLNESS_THRESH = 0.50;  // g  : lying still
const float TILT_THRESH      = 45.0;  // deg: posture change to confirm a fall


// 
const bool PLOT_MODE = false;


// Physical state of the person
enum FallState { IDLE, FREEFALL, MONITOR };
FallState fallState = IDLE;


const char* activityName = "Still";


unsigned long lastSampleTime = 0;
unsigned long stateTime = 0;
float angleBeforeFall = 0.0;
float maxDevAfterImpact = 0.0;


float pitch = 0.0, roll = 0.0, tiltAngle = 0.0;
float lastG = 1.0;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;


// activity window
float winMax = 0.0, winMin = 10.0;
unsigned long winStart = 0;


// countdown / alert
bool countdownActive = false;
unsigned long countdownStart = 0;
bool alertPending = false;
unsigned long lastAlertTry = 0;


// button debounce
bool btnDown = false;
unsigned long btnDownTime = 0;


unsigned long lastPublish = 0;


//  MPU6050 
void mpuWrite(uint8_t reg, uint8_t val) {
 Wire.beginTransmission(MPU_ADDR);
 Wire.write(reg);
 Wire.write(val);
 Wire.endTransmission(true);
}

// checks accelerometer for latest data and sees if sensor is actually plugged in and working.
bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
 Wire.beginTransmission(MPU_ADDR);
 Wire.write(0x3B);
 if (Wire.endTransmission(false) != 0) return false;
 if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;


 int16_t v[7];                        // AX AY AZ TEMP GX GY GZ
 for (uint8_t i = 0; i < 7; i++) {
   uint8_t hi = Wire.read();          // explicit order (avoids undefined evaluation order)
   uint8_t lo = Wire.read();
   v[i] = (int16_t)((hi << 8) | lo);
 }
 ax = v[0] / ACC_LSB_PER_G;
 ay = v[1] / ACC_LSB_PER_G;
 az = v[2] / ACC_LSB_PER_G;
 gx = v[4] / GYRO_LSB_PER_DPS;
 gy = v[5] / GYRO_LSB_PER_DPS;
 gz = v[6] / GYRO_LSB_PER_DPS;
 return true;
}

// checking if gyroscope works 
void calibrateGyro() {
 Serial.println(F("Calibrating gyro - keep the device still..."));
 float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
 int n = 0;
 for (int i = 0; i < 200; i++) {
   if (readMPU(ax, ay, az, gx, gy, gz)) {
     sx += gx; sy += gy; sz += gz; n++;
     pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG;  // start angles from gravity
     roll  = atan2(ay, az) * RAD2DEG;
   }
   delay(5);
 }
 if (n > 0) { gyroOffX = sx / n; gyroOffY = sy / n; gyroOffZ = sz / n; }
}


//  wakes up computer connection, sets up a button and configures motion sensor's sensitivity limits.
//  Setup - line 166 = baud value
void setup() {
 Serial.begin(115200);
 pinMode(BUTTON_PIN, INPUT_PULLUP);          

// 
 Wire.begin();
 mpuWrite(0x6B, 0x00);                // wake up
 mpuWrite(0x1A, 0x03);                // low-pass filter ~44 Hz
 mpuWrite(0x1C, 0x10);                // accel +/-8g
 mpuWrite(0x1B, 0x08);                // gyro  +/-500 deg/s

//
 float a, b, c, d, e, f;
 if (!readMPU(a, b, c, d, e, f)) {
   Serial.println(F("MPU6050 not found - check wiring (SDA=A4, SCL=A5)."));
   while (1) { delay(10); }
 }
 calibrateGyro();

// these are the progress comments in serial monitor when you upload code to the connected arduino device (if wiring is correct).
 Serial.println(F("Connecting to WiFi + Blynk..."));
 Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
 Serial.println(F("System active: monitoring."));


 lastSampleTime = winStart = millis();
}


//  Main loop (non-blocking)
void loop() {
  Blynk.run();
  unsigned long now = millis() ; 

  // TEST: type 't' in the Serial Monitor and press Send to start the countdown (making sure button is working - if not your wiring might be incorrect)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' && !countdownActive) {
      setActivity("FALL");
      startCountdown(now);
    }
  }
// timer that reads and processes sensor data at exact intervals while updating countdowns, managing alerts, and publishing status updates.
  if (now - lastSampleTime >= SAMPLE_PERIOD_MS) {
    float dt = (now - lastSampleTime) / 1000.0;
    lastSampleTime = now;
    sampleAndProcess(dt, now);
  }

  handleCountdown(now);
  retryPendingAlert(now);
  publishStatus(now);
}

//  Sensor sampling + orientation (MPU6050)
void sampleAndProcess(float dt, unsigned long now) {
 float ax, ay, az, gx, gy, gz;
 if (!readMPU(ax, ay, az, gx, gy, gz)) return;


 gx -= gyroOffX; gy -= gyroOffY; gz -= gyroOffZ;


 float g       = sqrt(ax * ax + ay * ay + az * az);   // magnitude in g
 float gyroMag = sqrt(gx * gx + gy * gy + gz * gz);   // deg/s
 lastG = g;


 // only trust the accelerometer when it is close to 1 g
 // (during free-fall / impact it does not point at gravity).
 float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG;
 float accelRoll  = atan2(ay, az) * RAD2DEG;
 float k = (g > 0.7 && g < 1.3) ? 0.02 : 0.0;
 pitch = (1.0 - k) * (pitch + gy * dt) + k * accelPitch;
 roll  = (1.0 - k) * (roll  + gx * dt) + k * accelRoll;
 tiltAngle = sqrt(pitch * pitch + roll * roll);

// sends data to computer for live graphing, then passes motion readings into background logic to check for falls and track activity.
 if (PLOT_MODE) {
   Serial.print(g); Serial.print(',');
   Serial.print(gyroMag / 100.0); Serial.print(',');
   Serial.println(tiltAngle / 45.0);
 }


 processFallStateMachine(g, now);
 updateActivityWindow(g, now);
}


//  Fall detection state - tells arduino what categorises certain physical levels to differenciate a fall vs standing, etc + the messages to write in monitor 
void processFallStateMachine(float g, unsigned long now) {
 switch (fallState) {


   case IDLE:
     if (countdownActive) break;                       // already handling a fall
     if (g < FREEFALL_THRESH) {
       angleBeforeFall = tiltAngle;
       fallState = FREEFALL;
       stateTime = now;
     } else if (g > IMPACT_THRESH) {                   // impact without a seen free-fall
       angleBeforeFall = tiltAngle;
       startMonitor(now);
     }
     break;


   case FREEFALL:
     if (g > IMPACT_THRESH) {
       startMonitor(now);
     } else if (now - stateTime > FREEFALL_TIMEOUT) {  // long float, no impact
       fallState = IDLE;
     }
     break;


   case MONITOR:
     if (now - stateTime > SETTLE_MS) {                // ignore the bounce right after impact
       float dev = fabs(g - 1.0);
       if (dev > maxDevAfterImpact) maxDevAfterImpact = dev;
     }
     if (now - stateTime > MONITOR_MS) {
       float angleChange = fabs(tiltAngle - angleBeforeFall);


       if (maxDevAfterImpact < STILLNESS_THRESH) {
         Serial.println(F("[!!!] FALL CONFIRMED: posture changed and user is not moving."));
         setActivity("FALL");
         startCountdown(now);
       } else if (angleChange < TILT_THRESH) {
         Serial.println(F("Impact with no posture change -> Jump."));
         setActivity("Jumping");
         winStart = now; winMax = 0; winMin = 10;      // hold "Jumping" for one window
       } else {
         Serial.println(F("Impact, posture changed but user is moving -> recovered."));
         setActivity("Recovered");
       }
       fallState = IDLE;
     }
     break;
 }
}


void startMonitor(unsigned long now) {
 fallState = MONITOR;
 stateTime = now;
 maxDevAfterImpact = 0.0;
}


//  Activity classification (walking / running / still)
void updateActivityWindow(float g, unsigned long now) {
 if (g > winMax) winMax = g;
 if (g < winMin) winMin = g;


 if (now - winStart < WINDOW_MS) return;


 // only classify when nothing fall-related is going on
 if (fallState == IDLE && !countdownActive) {
   if (strcmp(activityName, "Jumping") == 0 || strcmp(activityName, "Recovered") == 0) {
     // keep the label for exactly one window, then fall through to normal classification
     setActivity("Still");
   } else if (winMax >= RUN_MIN_PEAK)      setActivity("Running");
   else if (winMax >= WALK_MIN_PEAK)       setActivity("Walking");
   else                                    setActivity("Still");
 }
 winStart = now;
 winMax = 0.0;
 winMin = 10.0;
}


void setActivity(const char* name) {
 if (strcmp(activityName, name) != 0) {
   activityName = name;
   Serial.print(F("Activity: "));
   Serial.println(activityName);
   if (Blynk.connected()) Blynk.virtualWrite(V0, activityName);
 }
}


//  30-second countdown, button cancel, Blynk alert - in case of a false alarm, person can press button to stop the notification being sent
void startCountdown(unsigned long now) {
  countdownActive = true;
  countdownStart = now;
  btnDown = false;
  Serial.println(F("Countdown started: press the button within 30 s to cancel."));
}


bool buttonPressed(unsigned long now) {
 if (digitalRead(BUTTON_PIN) == LOW) {
   if (!btnDown) { btnDown = true; btnDownTime = now; }
   return (now - btnDownTime >= BUTTON_DEBOUNCE_MS);
 }
 btnDown = false;
 return false;
}


void handleCountdown(unsigned long now) {
 if (!countdownActive) return;


 unsigned long elapsed = now - countdownStart;

// this includes the comments in monitor if button is pressed or not
 if (buttonPressed(now)) {
   countdownActive = false;
   Serial.println(F("Button pressed: countdown cancelled. False alarm cleared."));
   setActivity("Still");
   if (Blynk.connected()) Blynk.virtualWrite(V1, 0);
 }
 else if (elapsed >= COUNTDOWN_MS) {
   countdownActive = false;
   Serial.println(F("Countdown expired: sending Blynk alert."));
   alertPending = true;
   lastAlertTry = 0;                    // send immediately
   if (Blynk.connected()) Blynk.virtualWrite(V1, 0);
 }
 else {
   static unsigned long lastPrint = 0;
   if (now - lastPrint >= 1000) {
     lastPrint = now;
     int remaining = (COUNTDOWN_MS - elapsed) / 1000;
     Serial.print(F("Seconds remaining to cancel: "));
     Serial.println(remaining);
     if (Blynk.connected()) Blynk.virtualWrite(V1, remaining);
   }
 }
}


// retries every 5 sec until BLYNK is connected and the event has gone out
void retryPendingAlert(unsigned long now) {
 if (!alertPending) return;
 if (lastAlertTry != 0 && now - lastAlertTry < 5000) return;
 lastAlertTry = now;
 if (lastAlertTry == 0) lastAlertTry = 1;


 if (Blynk.connected()) {
   Blynk.logEvent("fall.alert", "Emergency! A fall was detected and the user did not respond.");
   alertPending = false;
   Serial.println(F("Alert sent to Blynk."));
 } else {
   Serial.println(F("Blynk offline - will retry alert..."));
 }
}


// In the BLYNK app, you can customise how often you would like the notification to come through (I have done every 1 second)
void publishStatus(unsigned long now) {
 if (now - lastPublish < 1000) return;
 lastPublish = now;
 if (Blynk.connected()) Blynk.virtualWrite(V2, lastG);
}




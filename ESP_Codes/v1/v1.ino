#include <Wire.h>
#include <INA226.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <math.h>

// ==========================================
// I2C PINS
// ==========================================
#define I2C_SDA 21
#define I2C_SCL 22

// ==========================================
// BTS7960 MOTOR PINS & SETTINGS
// ==========================================
#define LEFT_LPWM    25
#define LEFT_RPWM    26
#define RIGHT_LPWM   27
#define RIGHT_RPWM   32

#define CH_LEFT_LPWM  0
#define CH_LEFT_RPWM  1
#define CH_RIGHT_LPWM 2
#define CH_RIGHT_RPWM 3

#define PWM_FREQ 15000
#define PWM_RES  8     // 8-bit resolution (0-255)

#define OFFSET_LEFT   1
#define OFFSET_RIGHT  1

// Global targets updated by the USB command task
float targetLeft = 0.0;
float targetRight = 0.0;

// ==========================================
// SENSOR OBJECTS & VARIABLES
// ==========================================
INA226 ina(0x40);
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

float voltage = 0.0;
float current = 0.0;

bool magReady = false;
bool inaReady = false;
bool magHeadingValid = false;
float magHeading = 0.0;
float curHdg = 0.0;
unsigned long lastMagRead = 0;

float MAG_DECLINATION_DEG = 0.0;
float MAG_MOUNT_OFFSET_DEG = 0.0;
float MAG_X_OFFSET = 0.0;
float MAG_Y_OFFSET = 0.0;
const float MAG_FILTER_ALPHA = 0.25;
const double TO_DEG = 57.29577951308232;

// ==========================================
// MATH HELPER FUNCTIONS
// ==========================================
float normalizeAngle180(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

float normalizeAngle360(float angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
}

// ==========================================
// MOTOR DRIVER FUNCTIONS
// ==========================================
void setupBTS7960Motor(int lpwmPin, int rpwmPin, int lpwmChannel, int rpwmChannel) {
    ledcSetup(lpwmChannel, PWM_FREQ, PWM_RES);
    ledcSetup(rpwmChannel, PWM_FREQ, PWM_RES);

    ledcAttachPin(lpwmPin, lpwmChannel);
    ledcAttachPin(rpwmPin, rpwmChannel);

    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);
}

void driveBTS7960(int lpwmChannel, int rpwmChannel, float val, int offset) {
    val *= offset;
    int pwm = constrain(abs((int)val), 0, 255);

    if (val > 10) {
        ledcWrite(lpwmChannel, pwm);
        ledcWrite(rpwmChannel, 0);
    } else if (val < -10) {
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, pwm);
    } else {
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);
    }
}

// ==========================================
// SENSOR READ FUNCTION
// ==========================================
void updateSensors() {
    if (inaReady) {
        voltage = ina.getBusVoltage();
        current = ina.getCurrent();
    }

    if (!magReady) {
        magHeadingValid = false;
        return;
    }

    if (millis() - lastMagRead < 100) return;
    lastMagRead = millis();

    sensors_event_t event;
    mag.getEvent(&event);

    float mx = event.magnetic.x - MAG_X_OFFSET;
    float my = event.magnetic.y - MAG_Y_OFFSET;

    if (abs(mx) < 0.01 && abs(my) < 0.01) {
        magHeadingValid = false;
        return;
    }

    float rawHeading = atan2(my, mx) * TO_DEG;
    rawHeading += MAG_DECLINATION_DEG;
    rawHeading += MAG_MOUNT_OFFSET_DEG;
    rawHeading = normalizeAngle360(rawHeading);

    if (!magHeadingValid) {
        magHeading = rawHeading;
        magHeadingValid = true;
    } else {
        float diff = normalizeAngle180(rawHeading - magHeading);
        magHeading = normalizeAngle360(magHeading + diff * MAG_FILTER_ALPHA);
    }
    
    curHdg = magHeading;
}

// ==========================================
// SERIAL READ FUNCTION (CORE 0)
// ==========================================
// ==========================================
// SERIAL READ FUNCTION (CORE 0)
// ==========================================
void commsTask(void *pvParameters) {
    String inBuffer = ""; // Persistent buffer to hold incoming chunks

    for (;;) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            
            // Wait until the user presses Enter (Newline or Carriage Return)
            if (c == '\n' || c == '\r') {
                if (inBuffer.length() > 0) {
                    inBuffer.trim();
                    inBuffer.toUpperCase();

                    int spaceIndex = inBuffer.indexOf(' ');
                    
                    if (spaceIndex > 0) {
                        float speed = inBuffer.substring(0, spaceIndex).toFloat();
                        String dir = inBuffer.substring(spaceIndex + 1);
                        
                        speed = constrain(speed, 0, 255); 
                        bool isValid = true;

                        if (dir == "FWD") {
                            targetLeft = speed;
                            targetRight = speed;
                        } else if (dir == "REV") {
                            targetLeft = -speed;
                            targetRight = -speed;
                        } else if (dir == "LFT") {
                            targetLeft = -speed;
                            targetRight = speed;
                        } else if (dir == "RGT") {
                            targetLeft = speed;
                            targetRight = -speed;
                        } else if (dir == "STOP") {
                            targetLeft = 0;
                            targetRight = 0;
                        } else {
                            isValid = false;
                            Serial.println("ERR: Unknown Direction (Use FWD, REV, LFT, RGT, STOP)");
                        }
                        
                        // Only print the target if the command was actually successful
                        if (isValid) {
                            Serial.printf("ACK | Target Speed: %.0f | Dir: %s\n", speed, dir.c_str());
                        }
                    } 
                    else if (inBuffer == "STOP") {
                        targetLeft = 0;
                        targetRight = 0;
                        Serial.println("ACK | EMERGENCY STOP");
                    } 
                    else {
                        Serial.println("ERR: Invalid format. Use 'SPEED DIR' (e.g., '150 FWD')");
                    }
                    
                    // Clear the buffer to get ready for the next command
                    inBuffer = ""; 
                }
            } else {
                // Stack normal characters into the buffer
                inBuffer += c;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    
    // Motor Pin Setup
    setupBTS7960Motor(LEFT_LPWM, LEFT_RPWM, CH_LEFT_LPWM, CH_LEFT_RPWM);
    setupBTS7960Motor(RIGHT_LPWM, RIGHT_RPWM, CH_RIGHT_LPWM, CH_RIGHT_RPWM);
    
    // Spawn the USB listening task
    xTaskCreatePinnedToCore(
        commsTask,   "USB_Task",  4096, NULL, 1, NULL, 0
    );

    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize INA226
    if (ina.begin()) {
        inaReady = true;
        ina.setMaxCurrentShunt(30.0, 0.002);
        Serial.println("[INA226 READY]");
    } else {
        inaReady = false;
        Serial.println("[INA226 NOT FOUND]");
    }

    // Initialize LSM303DLH
    if (mag.begin()) {
        magReady = true;
        Serial.println("[LSM303DLHC MAG READY]");
    } else {
        magReady = false;
        Serial.println("[LSM303DLHC MAG NOT FOUND]");
    }
}

// ==========================================
// MAIN LOOP (CORE 1)
// ==========================================
void loop() {
    // 1. Read I2C Data
    updateSensors();

    // 2. Drive Motors
    driveBTS7960(CH_LEFT_LPWM, CH_LEFT_RPWM, targetLeft, OFFSET_LEFT);
    driveBTS7960(CH_RIGHT_LPWM, CH_RIGHT_RPWM, targetRight, OFFSET_RIGHT);

    // 3. Print Telemetry (Only if sensors are alive)
    static unsigned long lastPrint = 0;
    if ((millis() - lastPrint > 1000) && magReady && inaReady) {
        lastPrint = millis();
        Serial.printf("Voltage: %.2fV | Current: %.2fA  | Heading: %.1f deg\n", voltage, current, curHdg);
    }
    
    delay(20); // Run loop at 50Hz for smooth motor updates
}
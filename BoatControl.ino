#include <ArduinoBLE.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <sbus.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

#include "KalmanFilter.h"

#define GPS_RX 5
#define GPS_TX 6
#define MOTOR1 7
#define MOTOR2 8
#define MOTOR_TEST false

#define EVENT_PERIOD 100

BLEService boatService("fff0");

// Please don't change
const int MAXCOORDS = 32;

// Bluetooth setup
BLEStringCharacteristic debugCharacteristic("45c1eda2-4473-42a3-8143-dc79c30a64bf", BLENotify | BLERead, 100);
BLEUnsignedIntCharacteristic statusCharacteristic("6f04c0a3-f201-4091-a13d-5ecafc3dc54b", BLENotify | BLERead);
BLECharCharacteristic commandCharacteristic("05c6cc87-7888-4588-b794-92bdf9a29330", BLEWrite);
BLECharacteristic coordsCharacteristic("3794c841-1b53-4029-aebb-12319386fd28", BLEWrite, 16*MAXCOORDS, true);
BLECharacteristic telemetryCharacteristic("ccc03716-4f66-4cb8-b6fd-9b2278587add", BLENotify, 100, false);
BLECharacteristic kalmanCharacteristic("933963ae-cc8e-4704-bd3c-dc53721ba956", BLENotify, 100, false);

// Status setup
enum status_e {
    GPS_AVAIL   = 1 << 0,
    RC_AVAIL    = 1 << 1,
    INITIALISED = 1 << 2,
};
volatile uint32_t g_status = 0;

// GPS setup
TinyGPSPlus gps;
Uart gpsSerial (&sercom0, GPS_RX, GPS_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler(){
    gpsSerial.IrqHandler();
}

// Frsky setup
bfs::SbusRx sbus(&Serial1);
bfs::SbusData data;

// Motor setup
constexpr float MOTOR_IDLE = 1475;
bool g_motorsInitialised = false;

// Waypoint data
int g_numWaypoints = 0;
int g_waypointPointer = 0;
double g_waypointCoords[10] = {};

// Homing data
bool g_homeSet = false;
double g_homeLat;
double g_homeLng;

// Frsky Control
const int DEADZONE = 100;

// Will do rc control
bool g_manualControl = true;

// IMU data (not used)
float rx, ry;

// Telemetry data
const unsigned int TELEM_SIZE = sizeof(double)*4 + sizeof(float)*3;
byte g_telemOutput[TELEM_SIZE];
double* p_x          = (double*) (g_telemOutput);
double* p_y          = (double*) (g_telemOutput + 8);
double* p_lat        = (double*) (g_telemOutput + 16);
double* p_lng        = (double*) (g_telemOutput + 24);
float*  p_powerLeft  = (float*)  (g_telemOutput + 32); // -1..1
float*  p_powerRight = (float*)  (g_telemOutput + 36); // -1..1
float*  p_rz         = (float*)  (g_telemOutput + 40);

// Kalman data
const unsigned int KALMAN_SIZE = sizeof(float)*6;
float g_kalmanOutput[6];

// Kalman filter
KalmanFilter kf(((float) EVENT_PERIOD) / 1000);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    if (!BLE.begin()){
        // Module failed
        while(1){
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
        }
    }

    Serial.begin(9600);

    gpsSerial.begin(9600);

    pinPeripheral(GPS_RX, PIO_SERCOM_ALT);
    pinPeripheral(GPS_TX, PIO_SERCOM_ALT);

    sbus.Begin();

    if (!IMU.begin()){
        Serial.println("IMU failed");
    }

    BLE.setLocalName("Boat");
    BLE.setAdvertisedService(boatService);
    boatService.addCharacteristic(debugCharacteristic);
    boatService.addCharacteristic(statusCharacteristic);
    boatService.addCharacteristic(commandCharacteristic);
    boatService.addCharacteristic(coordsCharacteristic);
    boatService.addCharacteristic(telemetryCharacteristic);
    boatService.addCharacteristic(kalmanCharacteristic);
    BLE.addService(boatService);

    BLE.advertise();

    delay(2000);

    Serial.println("Starting up");

    // Will block for bluetooth
    BLEDevice central = BLE.central();
    Serial.println("Waiting for central");
    while (!central) { central = BLE.central(); } // Wait for bluetooth
    Serial.println("Connected");
    while (!commandCharacteristic.written()){
        central.connected();
        debugCharacteristic.writeValue("Waiting for char...");
        delay(100);
    } // Wait for write
    commandCharacteristic.value(); // Throw away value
}

void loop() {
    // put your main code here, to run repeatedly:
    BLEDevice central = BLE.central();
    static unsigned long prevMillis = 0;

    if (central){
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Central connect");

        while (central.connected()){
            unsigned long currentMillis = millis();
            unsigned long dt = currentMillis - prevMillis;
            if (dt > EVENT_PERIOD){
                prevMillis = currentMillis;
                processInputsAndSensors();
            }
        }

        digitalWrite(LED_BUILTIN, LOW);
    }

    // Only allow exclusive rc control once motors are Initialised
    // This is because the motor initialisation is currently
    // done through bluetooth
    // (Subject to change)
    if (g_motorsInitialised) {
        if (g_manualControl){
            unsigned long currentMillis = millis();
            unsigned long dt = currentMillis - prevMillis;
            if (dt > EVENT_PERIOD){
                getRCControl();
                prevMillis = currentMillis;
            }
        }
    }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setStatus(uint32_t status) {
    // Set bit
    uint32_t new_status = g_status | status;
    if (new_status != g_status) {
        g_status = new_status;
        statusCharacteristic.writeValue(new_status);
    }
}

void unsetStatus(uint32_t status) {
    // Unset bit
    uint32_t new_status = g_status & ~status;
    if (new_status != g_status) {
        g_status = new_status;
        statusCharacteristic.writeValue(new_status);
    }
}

class MotorHandler {
    Servo* mpLeft = nullptr;
    Servo* mpRight = nullptr;

    bool mLeftReversed = false;
    bool mRightReversed = false;

    // Ramp up motor until input is sent over bluetooth
    void rampMotor(Servo* motor) {
        BLEDevice central = BLE.central();
        long power = MOTOR_IDLE;
        while (!commandCharacteristic.written()) {
            power += 8;
            if (power > 1900)
                power = 1900;
            motor->writeMicroseconds(power);
            central.connected();
            delay(100); 
        }
        motor->writeMicroseconds(MOTOR_IDLE);
    }

public:
    // Initialises motors
    MotorHandler() {
        Servo* firstMotor = new Servo;
        Servo* secondMotor = new Servo;
        firstMotor->attach(MOTOR1);
        secondMotor->attach(MOTOR2);
        firstMotor->writeMicroseconds(MOTOR_IDLE);
        secondMotor->writeMicroseconds(MOTOR_IDLE);

        // Motor initialisation (waits for another input)
        debugCharacteristic.writeValue("Waiting for motor init");
        BLEDevice central = BLE.central();
        while(!commandCharacteristic.written()) {
            central.connected();
            delay(100);
        }
        commandCharacteristic.value();
        Serial.println("Starting motor init");

        // Find which is left/right motors
        Serial.println("Sending debug");
        debugCharacteristic.writeValue("Which motor is runnig (l, r)");
        Serial.println("Sent debug");
        rampMotor(firstMotor);
        Serial.println("Got response");
        if (commandCharacteristic.value() == 'l') {
            mpLeft = firstMotor;
            mpRight = secondMotor;
        } else {
            mpRight = firstMotor;
            mpLeft = secondMotor;
        }

        // Test left motor reversed
        debugCharacteristic.writeValue("Running left (f, b)");
        rampMotor(mpLeft);
        if (commandCharacteristic.value() == 'b') {
            debugCharacteristic.writeValue("Reversing left");
            mLeftReversed = true;
        }

        // Test right motor reversed
        debugCharacteristic.writeValue("Running right (f, b)");
        rampMotor(mpRight);
        if (commandCharacteristic.value() == 'b') {
            debugCharacteristic.writeValue("Reversing right");
            mRightReversed = true;
        }

        debugCharacteristic.writeValue("Finished motor init");
        g_motorsInitialised = true;
        setStatus(INITIALISED);
        kf.setHome();
    }

    // Run motors according to global powers
    void run() {
        // Copy from global
        long power_left  = (long) (*p_powerLeft * 400.); // -400..400
        long power_right = (long) (*p_powerRight * 400.); // -400..400

        // Swap motor direction if needed
        if(mLeftReversed){
            power_left *= -1;
        }
        if(mRightReversed){
            power_right *= -1;
        }

        power_left = constrain(power_left, -400l, 400l);
        power_right = constrain(power_right, -400l, 400l);

        power_left += MOTOR_IDLE;
        power_right += MOTOR_IDLE;

        #if(MOTOR_TEST)
        Serial.print("Left Motor: ");
        Serial.print(power_left);
        Serial.print(", Right Motor: ");
        Serial.println(power_right);

        Serial.print("Left reversed: ");
        Serial.print(mLeftReversed);
        Serial.print(", Right reversed: ");
        Serial.println(mRightReversed);
        delay(100);
        #else
        mpLeft->writeMicroseconds(power_left);
        mpRight->writeMicroseconds(power_right);
        #endif
    }
};

void processCommand(){
    char cmd = commandCharacteristic.value();
    switch(cmd){
        //case 'i':
            //debugCharacteristic.writeValue("Initialising motors");
            //g_initialiseMotors = true;
            //g_startInitMotorTime = millis();
            //break;
        case 'h':
            debugCharacteristic.writeValue("Home reset");
            g_homeSet = false;
            break;
        default :
            debugCharacteristic.writeValue("Something else");
            break;
    }
}

// Get coordinates from coords characteristic
void processCoords(){
    g_numWaypoints = MAXCOORDS;
    for (int i = 0;i < MAXCOORDS*2; i++){
        double* pCoords = (double*) coordsCharacteristic.value();
        g_waypointPointer = 0;
        if (abs(pCoords[i]) < 0.01){
            // Check odd coords sent
            if (i % 2 == 1){
                debugCharacteristic.writeValue("Invalid coord count");
            } else {
                g_numWaypoints = i / 2;
            }
            break; // We are done reading
        }else{
            // Add waypoint
            g_waypointCoords[i] = pCoords[i];
        }
    }

    String output = "Coords written: ";
    output += g_numWaypoints;
    debugCharacteristic.writeValue(output);
}

// Poll gps once ready
void processGPS(){
    if (gps.location.isValid()){
        if (g_homeSet == false){
            g_homeLat = gps.location.lat();
            g_homeLng = gps.location.lng();
            g_homeSet = true;
            debugCharacteristic.writeValue("Home set");
        } else {
            double lat = gps.location.lat();
            double lng = gps.location.lng();
            double dist = TinyGPSPlus::distanceBetween(
                g_homeLat,
                g_homeLng,
                lat,
                lng);
            double angle = TinyGPSPlus::courseTo(
                g_homeLat,
                g_homeLng,
                lat,
                lng);
            double x = dist*sin(angle*DEG_TO_RAD);
            double y = dist*cos(angle*DEG_TO_RAD);
            memcpy(p_x, &x, 8);
            memcpy(p_y, &y, 8);
            memcpy(p_lat, &lat, 8);
            memcpy(p_lng, &lng, 8);
        }
        setStatus(GPS_AVAIL);
    } else {
        unsetStatus(GPS_AVAIL);
    }
}

// Runs motors according to rc controller
void getRCControl(){
    float x = 0;
    float y = 0;
    float m = 0;

    static bool frskyZeroSet = false;
    static int frskyZeroX = 0;
    static int frskyZeroY = 0;

    if (sbus.Read()){
        data = sbus.data();
        
        // Check frame loss
        if (data.lost_frame)
            unsetStatus(RC_AVAIL);
        else
            setStatus(RC_AVAIL);

        if (data.failsafe) {
            *p_powerLeft = 0;
            *p_powerRight = 0;
        }

        if (frskyZeroSet){
            x = data.ch[1] - frskyZeroX; // -800..800
            y = data.ch[2] - frskyZeroY; // -800..800

            m = data.ch[4]; // 0..1800
        } else {
            // First time connection
            frskyZeroX = data.ch[1];
            frskyZeroY = data.ch[2];
            frskyZeroSet = true;
            debugCharacteristic.writeValue("RC controller connected");
        }
    } else {
        return; // Not ready
    }

    // Deadzone
    if (abs(x) < DEADZONE){
        x = 0;
    }
    if (abs(y) < DEADZONE){
        y = 0;
    }

    float power_left  = mapfloat((y + x)*m, -1600.*1800., 1600.*1800., -1., 1.);
    float power_right = mapfloat((y - x)*m, -1600.*1800., 1600.*1800., -1., 1.);

    *p_powerLeft = constrain(power_left, -1, 1);
    *p_powerRight = constrain(power_right, -1, 1);
}

void updateKalmanFilter() {
    kf.predict(*p_powerLeft, *p_powerRight);
    sensor_data sensors;
    sensors.gpsX = *p_x;
    sensors.gpsY = *p_y;
    sensors.rz = *p_rz;
    kf.update(sensors);
    kf.copyTo(g_kalmanOutput);
    kalmanCharacteristic.writeValue(g_kalmanOutput, KALMAN_SIZE);
}

void processInputsAndSensors(){
    // Only runs if connected to device
    // Runs every 0.1 seconds

    if (commandCharacteristic.written()){
        processCommand();
    }
    if (coordsCharacteristic.written()){
        processCoords();
    }

    while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read())){
            processGPS();
        }
    }

    if (g_manualControl){
        getRCControl();
    }

    if (IMU.gyroscopeAvailable()){
        IMU.readGyroscope(rx, ry, *p_rz);
    }

    // Run motor
    static MotorHandler handler;
    handler.run();

    telemetryCharacteristic.writeValue((byte*) g_telemOutput, TELEM_SIZE);

    // Kalman filter
    updateKalmanFilter();
}

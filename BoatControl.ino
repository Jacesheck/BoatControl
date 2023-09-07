#include <ArduinoBLE.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <sbus.h>
#include "wiring_private.h"
#include <Arduino_LSM6DS3.h>

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
BLECharCharacteristic commandCharacteristic("05c6cc87-7888-4588-b794-92bdf9a29330", BLEWrite);
BLECharacteristic coordsCharacteristic("3794c841-1b53-4029-aebb-12319386fd28", BLEWrite, 16*MAXCOORDS, true);
BLECharacteristic telemetryCharacteristic("ccc03716-4f66-4cb8-b6fd-9b2278587add", BLENotify, 100, false);

TinyGPSPlus gps;
Uart gpsSerial (&sercom0, GPS_RX, GPS_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler(){
    gpsSerial.IrqHandler();
}

// Frsky setup
bfs::SbusRx sbus(&Serial1);
bfs::SbusData data;

// Motor setup
constexpr int MOTOR_IDLE = 1475;
bool g_motorsInitialised = false;
long g_powerLeft = MOTOR_IDLE;
long g_powerRight = MOTOR_IDLE;

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

// IMU data
float rx, ry;

// Telemetry data
const unsigned int TELEM_SIZE = sizeof(double)*4 + sizeof(long)*2 + sizeof(float);
byte g_telemOutput[100];
double* p_x    = (double*) (g_telemOutput);
double* p_y    = (double*) (g_telemOutput + 8);
double* p_lat  = (double*) (g_telemOutput + 16);
double* p_lng  = (double*) (g_telemOutput + 24);
long* p_power1 = (long*)   (g_telemOutput + 32);
long* p_power2 = (long*)   (g_telemOutput + 36);
float* p_rz    = (float*)  (g_telemOutput + 40);

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
    boatService.addCharacteristic(commandCharacteristic);
    boatService.addCharacteristic(coordsCharacteristic);
    boatService.addCharacteristic(telemetryCharacteristic);
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
    }

    // Run motors according to global powers
    void run() {
        // Copy from global
        long power_left = g_powerLeft;
        long power_right = g_powerRight;

        // Swap motor direction if needed
        if(mLeftReversed){
            power_left = (2 * MOTOR_IDLE) - power_left;
        }
        if(mRightReversed){
            power_right = (2 * MOTOR_IDLE) - power_right;
        }

        power_left = constrain(power_left, 1100, 1900);
        power_right = constrain(power_right, 1100, 1900);

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
        memcpy(p_power1, &power_left, sizeof(long));
        memcpy(p_power2, &power_right, sizeof(long));
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
    } else {
        static uint32_t i = 0;
        if (i++ % 20 == 0) {
            debugCharacteristic.writeValue("Location not avail");
        }
    }
}

// Runs motors according to rc controller
void getRCControl(){
    int x = 0;
    int y = 0;
    int m = 0;

    static uint8_t timer = 0;

    static bool frskyZeroSet = false;
    static float frskyZeroX = 0;
    static float frskyZeroY = 0;

    if (sbus.Read()){
        data = sbus.data();
        if (frskyZeroSet){
            x = data.ch[1] - frskyZeroX;
            y = data.ch[2] - frskyZeroY;

            m = data.ch[4];
        } else {
            // First time connection
            frskyZeroX = data.ch[1];
            frskyZeroY = data.ch[2];
            frskyZeroSet = true;
            debugCharacteristic.writeValue("RC controller connected");
        }
        timer = 0;
    } else {
        if (timer++ % 100 == 0) {
            debugCharacteristic.writeValue("No RC controller connected");
        }
    }

    // Deadzone
    if (abs(x) < DEADZONE){
        x = 0;
    }
    if (abs(y) < DEADZONE){
        y = 0;
    }

    g_powerLeft = y + x;
    g_powerRight = y - x;

    g_powerLeft = g_powerLeft*m/4000 + MOTOR_IDLE;
    g_powerRight = g_powerRight*m/4000 + MOTOR_IDLE;
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
}

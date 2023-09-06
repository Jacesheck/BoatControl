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

constexpr int MOTOR_IDLE = 1475;

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
Servo g_motor1;
Servo g_motor2;

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

    g_motor1.attach(MOTOR1);
    g_motor2.attach(MOTOR2);

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

    Serial.println("Starting up");
}

void loop() {
    // put your main code here, to run repeatedly:
    BLEDevice central = BLE.central();
    static unsigned long prevMillis = 0;

    if (central){
        digitalWrite(LED_BUILTIN, HIGH);

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

    if (g_manualControl){
        unsigned long currentMillis = millis();
        unsigned long dt = currentMillis - prevMillis;
        if (dt > EVENT_PERIOD){
            getRCControl();
            prevMillis = currentMillis;
        }
    }
}


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
    int dir1 = 0;
    int dir2 = 0;

    static bool frskyZeroSet = false;
    static float frskyZeroX = 0;
    static float frskyZeroY = 0;

    if (sbus.Read()){
        data = sbus.data();
        if (frskyZeroSet){
            x = data.ch[1] - frskyZeroX;
            y = data.ch[2] - frskyZeroY;

            m = data.ch[4];
            dir1 = data.ch[5];
            dir2 = data.ch[6];
        } else {
            frskyZeroX = data.ch[1];
            frskyZeroY = data.ch[2];
            frskyZeroSet = true;
        }
    } else {
        static uint8_t i = 0;
        if (i++ % 100 == 0) {
            debugCharacteristic.writeValue("No RC controller connected");
        }
        return;
    }

    // Deadzone
    if (abs(x) < DEADZONE){
        x = 0;
    }
    if (abs(y) < DEADZONE){
        y = 0;
    }

    long power1 = y + x;
    long power2 = y - x;

    // Swap motor direction
    if(dir1 < 300){
        power1*=-1;
    }
    if(dir2 < 300){
        power2*=-1;
    }

    power1 = power1*m/4000 + MOTOR_IDLE;
    power2 = power2*m/4000 + MOTOR_IDLE;

    power1 = constrain(power1, 1100, 1900);
    power2 = constrain(power2, 1100, 1900);

    #if(MOTOR_TEST)
    Serial.print("Motor1:");
    Serial.print(power1);
    Serial.print(",Motor2:");
    Serial.print(power2);

    Serial.print(",Dir1:");
    Serial.print(dir1);
    Serial.print(",Dir2:");
    Serial.println(dir2);
    delay(100);
    #else
    g_motor1.writeMicroseconds(power1);
    g_motor2.writeMicroseconds(power2);
    memcpy(p_power1, &power1, 4);
    memcpy(p_power2, &power2, 4);
    #endif
}

void processInputsAndSensors(){
    // Only runs if connected to device

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

    telemetryCharacteristic.writeValue((byte*) g_telemOutput, TELEM_SIZE);
}

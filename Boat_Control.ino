#include <ArduinoBLE.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include "sbus.h"
#include "wiring_private.h"

#define GPS_RX 5
#define GPS_TX 6
#define MOTOR1 7
#define MOTOR2 8
#define MOTOR_TEST false

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
Servo g_motor1;
Servo g_motor2;

unsigned long g_prevMillis = 0;

byte g_telemOutput[100];

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

bool g_manualControl = true;
bool g_frskyZeroSet = false;
float g_frskyZeroX;
float g_frskyZeroY;

const unsigned int TELEM_SIZE = sizeof(double)*4;
double *p_x   = (double*) (g_telemOutput);
double *p_y   = (double*) (g_telemOutput + 8);
double *p_lat = (double*) (g_telemOutput + 16);
double *p_lng = (double*) (g_telemOutput + 24);

void setup() {
    if (!BLE.begin()){
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

    pinMode(LED_BUILTIN, OUTPUT);

    BLE.setLocalName("Boat");
    BLE.setAdvertisedService(boatService);
    boatService.addCharacteristic(debugCharacteristic);
    boatService.addCharacteristic(commandCharacteristic);
    boatService.addCharacteristic(coordsCharacteristic);
    boatService.addCharacteristic(telemetryCharacteristic);
    BLE.addService(boatService);

    BLE.advertise();
}

void loop() {
    // put your main code here, to run repeatedly:
    BLEDevice central = BLE.central();

    if (central){
        digitalWrite(LED_BUILTIN, HIGH);

        while (central.connected()){
            unsigned long currentMillis = millis();
            if (currentMillis - g_prevMillis > 100){
                processEvents();
            }
        }

        digitalWrite(LED_BUILTIN, LOW);
    }
    if (g_manualControl){
        getRCControl();
    }

}


void processCommand(){
    char cmd = commandCharacteristic.value();
    switch(cmd){
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
            double x = dist*cos(angle*DEG_TO_RAD);
            double y = dist*sin(angle*DEG_TO_RAD);
            memcpy(p_x, &x, 8);
            memcpy(p_y, &y, 8);
            memcpy(p_lat, &lat, 8);
            memcpy(p_lng, &lng, 8);
        }
    } else {
        debugCharacteristic.writeValue("Location not available");
    }
}

void getRCControl(){
    int x;
    int y;
    int m;
    int dir1;
    int dir2;

    if (sbus.Read()){
        data = sbus.data();
        if (g_frskyZeroSet){
            x = data.ch[1] - g_frskyZeroX;
            y = data.ch[2] - g_frskyZeroY;
            m = data.ch[4];
            dir1 = data.ch[5];
            dir2 = data.ch[6];
        } else {
            g_frskyZeroX = data.ch[1];
            g_frskyZeroY = data.ch[2];
            g_frskyZeroSet = true;
        }
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

    if(dir1 < 300){
        power1*=-1;
    }
    if(dir2 < 300){
        power2*=-1;
    }

    power1 = power1*m/4000 + 1500;
    power2 = power2*m/4000 + 1500;

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
    #endif
}

void processEvents(){
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
    telemetryCharacteristic.writeValue((byte*) g_telemOutput, TELEM_SIZE);
}

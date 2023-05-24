#include <ArduinoBLE.h>
#include <TinyGPS++.h>

BLEService boatService("fff0");

const int MAXCOORDS = 32;

BLEStringCharacteristic debugCharacteristic("45c1eda2-4473-42a3-8143-dc79c30a64bf", BLENotify | BLERead, 100);
BLECharCharacteristic commandCharacteristic("05c6cc87-7888-4588-b794-92bdf9a29330", BLEWrite);
BLECharacteristic coordsCharacteristic("3794c841-1b53-4029-aebb-12319386fd28", BLEWrite, 16*MAXCOORDS, true);
BLECharacteristic telemetryCharacteristic("ccc03716-4f66-4cb8-b6fd-9b2278587add", BLENotify, 100, false);

TinyGPSPlus gps;

unsigned long prevMillis = 0;

int numWaypoints = 0;
int waypointPointer = 0;
double waypointCoords[10] = {};

byte telemOutput[100];

bool homeSet = false;
double homeLat;
double homeLng;

const unsigned int telemSize = sizeof(double)*4;
double *p_x   = (double*) (telemOutput);
double *p_y   = (double*) (telemOutput + 8);
double *p_lat = (double*) (telemOutput + 16);
double *p_lng = (double*) (telemOutput + 24);

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
    Serial1.begin(9600);

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
            if (currentMillis - prevMillis > 100){
                processEvents();
            }
        }

        digitalWrite(LED_BUILTIN, LOW);
    }
}


void processCommand(){
    char cmd = commandCharacteristic.value();
    switch(cmd){
        case 'h':
            debugCharacteristic.writeValue("Home reset");
            homeSet = false;
            break;
        default :
            debugCharacteristic.writeValue("Something else");
            break;
    }
}

void processCoords(){
    numWaypoints = MAXCOORDS;
    for (int i = 0;i < MAXCOORDS*2; i++){
        double* pCoords = (double*) coordsCharacteristic.value();
        waypointPointer = 0;
        if (abs(pCoords[i]) < 0.01){
            // Check odd coords sent
            if (i % 2 == 1){
                debugCharacteristic.writeValue("Invalid coord count");
            } else {
                numWaypoints = i / 2;
            }
            break; // We are done reading
        }else{
            // Add waypoint
            waypointCoords[i] = pCoords[i];
        }
    }

    String output = "Coords written: ";
    output += numWaypoints;
    debugCharacteristic.writeValue(output);
}

void processGPS(){
    Serial.print("Valid ");
    Serial.println(gps.location.isValid());
    if (gps.location.isValid()){
        if (homeSet == false){
            homeLat = gps.location.lat();
            homeLng = gps.location.lng();
            homeSet = true;
        } else {
            double lat = gps.location.lat();
            double lng = gps.location.lng();
            double dist = TinyGPSPlus::distanceBetween(
                homeLat,
                homeLng,
                lat,
                lng);
            double angle = TinyGPSPlus::courseTo(
                homeLat,
                homeLng,
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

void processEvents(){
    // Only runs if connected to device

    if (commandCharacteristic.written()){
        processCommand();
    }
    if (coordsCharacteristic.written()){
        processCoords();
    }

    while (Serial1.available() > 0){
        if (gps.encode(Serial1.read())){
            processGPS();
        }
    }
    telemetryCharacteristic.writeValue((byte*) telemOutput, telemSize);
}

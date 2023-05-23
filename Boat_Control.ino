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

void outputTelemetry(){
    double gpsX = 100.;
    double gpsY = 200.;

    int i = 0;
    memcpy(&telemOutput[i], &gpsX, sizeof(gpsX));
    i += sizeof(gpsX);
    memcpy(&telemOutput[i], &gpsY, sizeof(gpsY));

    telemetryCharacteristic.writeValue((byte*) telemOutput, 16);
}

void processCommand(){
    char cmd = commandCharacteristic.value();
    switch(cmd){
        case 'a':
            debugCharacteristic.writeValue("Command a\n");
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
    static bool gpsFound = false;
    if (gps.location.isValid()){
        if (gpsFound == false){
            // Run once
            debugCharacteristic.writeValue("Found GPS");
            gpsFound = true;
        }
    }
}

void processEvents(){
    if (commandCharacteristic.written()){
        processCommand();
    }
    if (coordsCharacteristic.written()){
        processCoords();
    }

    while (Serial.available()){
        if (gps.encode(Serial.read()))
            processGPS();
    }
    outputTelemetry();
}

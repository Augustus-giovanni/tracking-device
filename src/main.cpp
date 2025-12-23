// Improved ESP32 GPS + BLE sketch
// - Non-blocking notification timing
// - BLE connection handling
// - GPS validity checks

#include <Arduino.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPS on UART1
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

BLECharacteristic* pCharacteristic = nullptr;
BLEServer* pServer = nullptr;
volatile bool deviceConnected = false;
BLEAdvertising *pAdvertising = nullptr;

const int GPS_RX = 16; // connect GPS TX -> ESP32 RX (GPIO16)
const int GPS_TX = 17; // connect GPS RX -> ESP32 TX (GPIO17)
const unsigned long NOTIFY_INTERVAL_MS = 1500;

unsigned long lastNotifyMs = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    // restart advertising so client can reconnect
    if (pAdvertising) pAdvertising->start();
  }
};

void setupBLE() {
  BLEDevice::init("Device_001");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Waiting for GPS...");

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) yield();
  Serial.println("System Starting...");

  // Initialize UART1 for GPS (RX, TX)
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  setupBLE();

  lastNotifyMs = millis();
  Serial.println("BLE Ready, GPS Waiting for fix...");
}

void tryNotifyLocation() {
  // Only notify at interval and when a client is connected
  unsigned long now = millis();
  if (!deviceConnected) return;
  if (now - lastNotifyMs < NOTIFY_INTERVAL_MS) return;

  if (gps.location.isValid()) {
    char payload[80];
    int n = snprintf(payload, sizeof(payload), "Lat:%.6f,Lng:%.6f,Alt:%.2f",
                     gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    if (n > 0) {
      pCharacteristic->setValue((uint8_t*)payload, (size_t)n);
      pCharacteristic->notify();
      Serial.print("Notified: ");
      Serial.println(payload);
    }
  } else {
    Serial.println("Waiting for valid GPS fix...");
  }

  lastNotifyMs = now;
}

void loop() {
  // Read incoming GPS bytes as they arrive (non-blocking)
  while (SerialGPS.available()) {
    char c = (char)SerialGPS.read();
    gps.encode(c);
  }

  // Periodically try to send location over BLE (if connected)
  tryNotifyLocation();

  // Optional: print a status line occasionally when not connected
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    if (!deviceConnected) {
      if (gps.location.isValid()) {
        Serial.print("GPS valid: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
      } else {
        Serial.println("GPS: waiting for fix...");
      }
    }
    lastStatus = millis();
  }
}

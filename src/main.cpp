#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <map>
#include <cmath>
#include <vector>

// Function prototypes
void startScan();
void exploreService(BLERemoteService* service);
String fallbackConvert(const std::string& rawValue);
bool writeControlRegister();
void displayFoundDevices();
void processUserSelection();
bool connectToDevice();
String getUuidName(BLEUUID uuid);

// Service UUID Definitions
static BLEUUID DIS_UUID((uint16_t)0x180A);  // Device Information Service
static BLEUUID TEMP_UUID("B1F8799E-4999-4F4A-AF05-B5A6FB6AB55D"); // Temperature Service
static BLEUUID CSCP_UUID((uint16_t)0x1816); // Cycling Speed and Cadence Profile
static BLEUUID USER_UUID("B1F879A7-4999-4F4A-AF05-B5A6FB6AB55D"); // User Service
static BLEUUID BATTERY_UUID((uint16_t)0x180F); // Battery Service
static BLEUUID CONTROL_UUID("B1F879B4-4999-4F4A-AF05-B5A6FB6AB55D"); // Control Service
static BLEUUID CONTROL_REG_UUID("B1F879B5-4999-4F4A-AF05-B5A6FB6AB55D"); // Control Register

// Descriptor UUID for Characteristic Presentation Format (CPF)
static BLEUUID CPF_DESC_UUID((uint16_t)0x2904);

// Target Device Configuration
static const char* TARGET_DEVICE_PREFIX = "Skp";
BLEScan* pBLEScan;
BLEClient* pClient = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;
bool deviceFound = false;
bool isConnected = false;
bool scanCompleted = false;
bool waitingForUserInput = false;

// Store all matching devices and their signal strengths
std::map<std::string, std::pair<BLEAdvertisedDevice*, int>> foundDevices;
std::vector<std::pair<std::string, int>> sortedDevices; // For displaying sorted list

// Service/Characteristic Name Map
std::map<std::string, std::string> uuidNames = {
  {DIS_UUID.toString(), "Device Information Service"},
  {TEMP_UUID.toString(), "Temperature Service"},
  {CSCP_UUID.toString(), "Cycling Speed and Cadence"},
  {USER_UUID.toString(), "User Service"},
  {BATTERY_UUID.toString(), "Battery Service"},
  {CONTROL_UUID.toString(), "Control Service"},
  {CONTROL_REG_UUID.toString(), "Control Register"},
  {"0x2A29", "Manufacturer Name String"},
  {"0x2A24", "Model Number String"},
  {"0x2A5B", "CSC Measurement"}
};

// Unit mapping for CPF descriptor (unit UUID -> unit string)
std::map<uint16_t, String> unitMap = {
  {0x2700, ""},         // unitless
  {0x2763, "km/h"},     // kilometres per hour
  {0x27AD, "rpm"},      // revolutions per minute
  {0x2701, "m"},        // metres
  {0x27B1, "°C"},       // degrees Celsius
  {0x27B3, "%"},        // percentage
  {0x27AE, "V"},        // volts
  {0x27AC, "A"}         // amperes
};

// Convert raw binary data using CPF descriptor parameters
String convertRawValue(const std::string& rawValue, uint8_t format, int8_t exponent, uint16_t unitUUID) {
  String unit = (unitMap.find(unitUUID) != unitMap.end()) ? unitMap[unitUUID] : "";
  double scaledValue = 0;

  switch (format) {
    case 0x01: // Boolean
      return rawValue.empty() ? "false" : (rawValue[0] ? "true" : "false");

    case 0x04: { // uint32
      if (rawValue.size() < 4) break;
      uint32_t val = *(reinterpret_cast<const uint32_t*>(rawValue.data()));
      scaledValue = val * pow(10, exponent);
      return String(scaledValue, 2) + unit;
    }
    case 0x06: { // uint16
      if (rawValue.size() < 2) break;
      uint16_t val = *(reinterpret_cast<const uint16_t*>(rawValue.data()));
      scaledValue = val * pow(10, exponent);
      return String(scaledValue, 2) + unit;
    }
    case 0x08: { // int32
      if (rawValue.size() < 4) break;
      int32_t val = *(reinterpret_cast<const int32_t*>(rawValue.data()));
      scaledValue = val * pow(10, exponent);
      return String(scaledValue, 2) + unit;
    }
    case 0x0A: { // int16
      if (rawValue.size() < 2) break;
      int16_t val = *(reinterpret_cast<const int16_t*>(rawValue.data()));
      scaledValue = val * pow(10, exponent);
      return String(scaledValue, 2) + unit;
    }
    case 0x0E: { // float32
      if (rawValue.size() < 4) break;
      float val = *(reinterpret_cast<const float*>(rawValue.data()));
      return String(val * pow(10, exponent), 2) + unit;
    }
  }
  return "";
}

// Fallback conversion: attempt ASCII, otherwise decimal bytes.
String fallbackConvert(const std::string& rawValue) {
  if (rawValue.empty()) return "";
  
  bool isAscii = true;
  for (char c : rawValue) {
    if (c < 32 || c > 126) {
      isAscii = false;
      break;
    }
  }
  if (isAscii) {
    return String(rawValue.c_str());
  }
  
  String out = "";
  for (uint8_t c : rawValue) {
    out += String(c) + " ";
  }
  return out;
}

void exploreService(BLERemoteService* service) {
  BLEUUID serviceUUID = service->getUUID();
  Serial.printf("\nService: %s\nUUID: %s\n", 
                (uuidNames.find(serviceUUID.toString()) != uuidNames.end() ? 
                  uuidNames[serviceUUID.toString()].c_str() : serviceUUID.toString().c_str()),
                serviceUUID.toString().c_str());

  for (auto& chr : *service->getCharacteristics()) {
    BLERemoteCharacteristic* pChar = chr.second;
    BLEUUID charUUID = pChar->getUUID();
    Serial.printf("  Characteristic: %s\n  UUID: %s\n",
                  (uuidNames.find(charUUID.toString()) != uuidNames.end() ? 
                   uuidNames[charUUID.toString()].c_str() : charUUID.toString().c_str()),
                  charUUID.toString().c_str());
    
    if (pChar->canRead()) {
      std::string rawValue = pChar->readValue();
      String formattedValue = "";
      uint16_t unitUUID = 0x2700; // Default: unitless
      bool descriptorUsed = false;

      auto descriptors = pChar->getDescriptors();
      if (descriptors) {
        for (auto& desc : *descriptors) {
          if (desc.second->getUUID().equals(CPF_DESC_UUID)) {
            std::string cpfData = desc.second->readValue();
            if (cpfData.size() >= 7) {
              uint8_t format = cpfData[0];
              int8_t exponent = cpfData[1];
              unitUUID = *(reinterpret_cast<const uint16_t*>(&cpfData[2]));
              formattedValue = convertRawValue(rawValue, format, exponent, unitUUID);
            }
            descriptorUsed = true;
            break;
          }
        }
      }
      
      // If descriptor conversion failed, use fallback conversion.
      if (formattedValue.length() == 0) {
        formattedValue = fallbackConvert(rawValue);
      }
      
      Serial.printf("  Value: %s\n", formattedValue.c_str());
    }
  }
}

// Function to write magic word to Control Register
bool writeControlRegister() {
  if (!pClient || !pClient->isConnected()) {
    Serial.println("Cannot write to control register: not connected");
    return false;
  }

  Serial.println("Accessing Control Service...");
  BLERemoteService* pControlService = pClient->getService(CONTROL_UUID);
  if (!pControlService) {
    Serial.println("Control Service not found");
    return false;
  }

  Serial.println("Accessing Control Register characteristic...");
  BLERemoteCharacteristic* pControlReg = pControlService->getCharacteristic(CONTROL_REG_UUID);
  if (!pControlReg) {
    Serial.println("Control Register characteristic not found");
    return false;
  }

  if (pControlReg->canWrite()) {
    // Write as hex byte array (big-endian)
    Serial.println("Writing magic word 0x337412E4 (big-endian)...");
    uint8_t magicWordBytesBE[] = {0x33, 0x74, 0x12, 0xE4};
    pControlReg->writeValue(magicWordBytesBE, sizeof(magicWordBytesBE), true);
    Serial.println("Magic word successfully written to Control Register");
    return true;
  } else {
    Serial.println("Control Register is not writable");
    return false;
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    Serial.println("Connected to device");
    isConnected = true;
  }

  void onDisconnect(BLEClient* pclient) override {
    Serial.println("Disconnected from device");
    isConnected = false;
    if (targetDevice) {
      delete targetDevice;
      targetDevice = nullptr;
    }
    // Restart scanning after disconnection
    startScan();
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveName() && 
        advertisedDevice.getName().find(TARGET_DEVICE_PREFIX) == 0) {
      
      // Get device address
      std::string address = advertisedDevice.getAddress().toString();
      
      // Get RSSI (signal strength)
      int rssi = advertisedDevice.getRSSI();
      
      // Create a copy of the device to store
      BLEAdvertisedDevice* deviceCopy = new BLEAdvertisedDevice(advertisedDevice);
      
      // Store device with its signal strength
      foundDevices[address] = std::make_pair(deviceCopy, rssi);
      
      Serial.print("Found device: ");
      Serial.print(advertisedDevice.getName().c_str());
      Serial.print(" - Address: ");
      Serial.print(address.c_str());
      Serial.print(" - RSSI: ");
      Serial.println(rssi);
    }
  }
};

String getUuidName(BLEUUID uuid) {
  std::string uuidStr = uuid.toString();
  return (uuidNames.find(uuidStr) != uuidNames.end()) ? 
         String(uuidNames[uuidStr].c_str()) : 
         String(uuidStr.c_str());
}

// Function to connect to the selected device
bool connectToDevice();

// Function to display found devices sorted by signal strength
void displayFoundDevices() {
  sortedDevices.clear();
  
  // Convert map to vector for sorting
  for (auto& item : foundDevices) {
    sortedDevices.push_back(std::make_pair(item.first, item.second.second));
  }
  
  // Sort by RSSI (higher values = stronger signal)
  std::sort(sortedDevices.begin(), sortedDevices.end(), 
            [](const std::pair<std::string, int>& a, const std::pair<std::string, int>& b) {
              return a.second > b.second;
            });
  
  Serial.println("\n===== Found Devices =====");
  Serial.println("Num | Device Name | Address | RSSI");
  Serial.println("----------------------------------------");
  
  for (size_t i = 0; i < sortedDevices.size(); i++) {
    auto& deviceInfo = foundDevices[sortedDevices[i].first];
    
    Serial.print(i+1);
    Serial.print(" | ");
    Serial.print(deviceInfo.first->getName().c_str());
    Serial.print(" | ");
    Serial.print(sortedDevices[i].first.c_str());
    Serial.print(" | ");
    Serial.println(sortedDevices[i].second);
  }
  
  Serial.println("----------------------------------------");
  Serial.println("Enter device number to connect (1-" + String(sortedDevices.size()) + "):");
  waitingForUserInput = true;
}

// Process user selection
void processUserSelection() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    int selection = input.toInt();
    
    if (selection > 0 && selection <= sortedDevices.size()) {
      Serial.print("Connecting to device #");
      Serial.println(selection);
      
      // Get the selected device address
      std::string selectedAddress = sortedDevices[selection-1].first;
      
      // Set the target device
      targetDevice = new BLEAdvertisedDevice(*foundDevices[selectedAddress].first);
      deviceFound = true;
      waitingForUserInput = false;
      
      // Connect to the device
      if (!connectToDevice()) {
        Serial.println("Connection failed. Restarting scan...");
        delete targetDevice;
        targetDevice = nullptr;
        startScan();
      }
    } else {
      Serial.println("Invalid selection. Please try again.");
    }
  }
}

bool connectToDevice() {
  if (!targetDevice) return false;
  
  Serial.print("Connecting to ");
  Serial.println(targetDevice->getAddress().toString().c_str());

  if (pClient) {
    pClient->disconnect();
    delete pClient;
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(targetDevice)) {
    Serial.println("Connection failed");
    return false;
  }

  Serial.println("Connection established. Discovering services...");

  // Discover services
  auto services = pClient->getServices();
  if (!services) {
    Serial.println("Failed to get services");
    return false;
  }

  // Print info about all services and characteristics
  for (auto& service : *services) {
    BLEUUID serviceUUID = service.second->getUUID();
    if (serviceUUID.equals(DIS_UUID) || serviceUUID.equals(TEMP_UUID) ||
        serviceUUID.equals(CSCP_UUID) || serviceUUID.equals(USER_UUID) ||
        serviceUUID.equals(BATTERY_UUID) || serviceUUID.equals(CONTROL_UUID)) {
      exploreService(service.second);
    }
  }
  
  // After exploring services, write the magic word to the Control Register
  Serial.println("\nAttempting to write magic word to Control Register...");
  bool writeResult = writeControlRegister();
  if (writeResult) {
    Serial.println("Control Register write completed successfully");
  } else {
    Serial.println("Control Register write failed");
  }
  
  return true;
}

void startScan() {
  Serial.println("Starting BLE scan for devices with prefix: " + String(TARGET_DEVICE_PREFIX) + "...");
  
  // Clear previous scan results
  pBLEScan->clearResults();
  
  // Clear stored devices from previous scan
  for (auto& item : foundDevices) {
    delete item.second.first;  // Delete the BLEAdvertisedDevice pointer
  }
  foundDevices.clear();
  
  // Start scan for 5 seconds
  scanCompleted = false;
  waitingForUserInput = false;
  
  pBLEScan->start(5, [](BLEScanResults results) {
    Serial.print("Scan complete. Found ");
    Serial.print(foundDevices.size());
    Serial.println(" matching devices.");
    
    if (foundDevices.empty()) {
      Serial.println("No devices found with prefix '" + String(TARGET_DEVICE_PREFIX) + "'. Restarting scan...");
      delay(2000);  // Wait 2 seconds before restarting scan
      startScan();
    } else {
      // Display the devices and wait for user input
      displayFoundDevices();
    }
    
    scanCompleted = true;
  }, false);
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_NONE);
  
  Serial.println("\nBLE Scanner with User Selection");
  Serial.println("==============================");
  
  BLEDevice::init("ESP32");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  // Start initial scan
  startScan();
}

void loop() {
  // Process user input if waiting for a selection
  if (waitingForUserInput) {
    processUserSelection();
  }
  
  // Handle disconnection
  if (isConnected && pClient && !pClient->isConnected()) {
    isConnected = false;
    pClient->disconnect();
    delete pClient;
    pClient = nullptr;
    Serial.println("Device disconnected. Restarting scan...");
    startScan();
  }

  delay(100); // shorter delay for more responsive input processing
}
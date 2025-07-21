#include <hyundaiPID.h>
#include <kiaPID.h>

// Service 01 PIDs (more detail: https://en.wikipedia.org/wiki/OBD-II_PIDs)
#define PID_COOLANT_TEMP 0x05
#define PID_ENGINE_RPM 0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_RUNTIME 0x1F
#define PID_FUEL_LEVEL 0x2F
#define PID_ODOMETER 0xA6
//----------------------------------------------
#define CAN_ID_PID 0x7DF //OBD-II CAN frame ID
#define KIA_CAN_ID 0x7C6
#define HYUNDAI_CAN_ID 0x7C6 

#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2                              // Set INT to pin 2  <--------- CHANGE if using different pin number
MCP_CAN CAN0(10);                               // Set CS to pin 10 <--------- CHANGE if using different pin number

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

String carMake = "";

bool isPIDSupported(unsigned char pid) {
  int attempts = 3; // Check PID support by sending requests multiple times
  while (attempts--) {
    sendPID(pid);
    delay(500);
    if (!digitalRead(CAN0_INT)) {
      CAN0.readMsgBuf(&rxId, &len, rxBuf);
      if (len >= 3 && rxBuf[2] == pid) {
        return true;
      }
    }
  }
  return false; // If no valid response is received
}

void sendPID(unsigned char __pid)
{
  unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};

  byte sndStat = CAN0.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);

  if (sndStat == CAN_OK) {
    Serial.print("PID sent: 0x");
    Serial.println(__pid, HEX);
  }
  else {
    Serial.println("Error Sending Message...");
  }
}

void receivePID(unsigned char __pid)
{
    if (!digitalRead(CAN0_INT)) {                      // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    sprintf(msgString, "Standard ID: 0x%.3lX, DLC: %1d, Data: ", rxId, len);
    Serial.print(msgString);

    for (byte i = 0; i < len; i++) {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
      Serial.print(msgString);
    }
    Serial.println("");


    switch (__pid) {
      case PID_VEHICLE_SPEED:
        if(rxBuf[2] == PID_VEHICLE_SPEED){
          uint8_t speed;
          speed = rxBuf[3];
          Serial.print("Vehicle speed (km/h): ");
          Serial.println(speed, DEC);
        } else {
          Serial.println("PID mismatch");
        }
      break;

      case PID_COOLANT_TEMP:
        if(rxBuf[2] == PID_COOLANT_TEMP){
          uint8_t temp;
          temp = rxBuf[3] - 40;
          Serial.print("Engine Coolant Temp (degC): ");
          Serial.println(temp, DEC);
        } else {
          Serial.println("PID mismatch");
        }
      break;

      case PID_FUEL_LEVEL:
        if(rxBuf[2] == PID_FUEL_LEVEL){
          float fuel;
          fuel = (100.0 / 255.0) * rxBuf[3];
          Serial.print("Fuel level (%): ");
          Serial.println(fuel, DEC);
        } else {
          Serial.println("PID mismatch");
        }
      break;

      case PID_ODOMETER:
        if(rxBuf[2] == PID_ODOMETER){
          uint32_t odometer;
          odometer = (static_cast<uint32_t>(rxBuf[3]) << 24) | 
                    (static_cast<uint32_t>(rxBuf[4]) << 16) | 
                    (static_cast<uint32_t>(rxBuf[5]) << 8)  | 
                    static_cast<uint32_t>(rxBuf[6]);
          float distance = odometer * 0.1; 
          Serial.print("Odometer (km): ");
          Serial.println(distance, 1); 
        } else {
          Serial.println("PID mismatch");
        }
      break;

      case PID_RUNTIME:
        if(rxBuf[2] == PID_RUNTIME){
          uint16_t runtime;
          runtime = 256 * rxBuf[3] + rxBuf[4];
          uint16_t hours = runtime / 3600;  
          uint16_t minutes = (runtime % 3600) / 60;  
          uint16_t seconds = runtime % 60;  
          Serial.print("Run time (s): ");
          Serial.print(runtime, DEC);
          Serial.print(" (");
          if (hours < 10) Serial.print("0"); 
          Serial.print(hours);
          Serial.print(":");
          if (minutes < 10) Serial.print("0"); 
          Serial.print(minutes);
          Serial.print(":");
          if (seconds < 10) Serial.print("0");   
          Serial.print(seconds);
          Serial.println(")");
        } else {
          Serial.println("PID mismatch");
        }
      break;

      case PID_ENGINE_RPM:
        if(rxBuf[2] == PID_ENGINE_RPM){
          uint16_t rpm;
          rpm = ((256 * rxBuf[3]) + rxBuf[4]) / 4;
          Serial.print("Engine Speed (rpm): ");
          Serial.println(rpm, DEC);
        } else {
          Serial.println("PID mismatch");
        }
      break;
    }
  }
}

void sendExtendedPID(unsigned long __pid, unsigned int canId)
{
  unsigned char tmp[8] = {0x03, 0x22, (unsigned char)((__pid >> 16) & 0xFF), (unsigned char)((__pid >> 8) & 0xFF), (unsigned char)(__pid & 0xFF), 0, 0, 0};

  byte sndStat = CAN0.sendMsgBuf(canId, 0, 8, tmp);

  if (sndStat == CAN_OK) {
    Serial.print("Extended PID sent: 0x");
    Serial.println(__pid, HEX);
  }
  else {
    Serial.println("Error Sending Extended Message...");
  }
}

void receiveExtendedPID(unsigned long __pid)
{
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    sprintf(msgString, "Standard ID: 0x%.3lX, DLC: %1d, Data: ", rxId, len);
    Serial.print(msgString);

    for (byte i = 0; i < len; i++) {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
      Serial.print(msgString);
    }
    Serial.println("");
  }
}


unsigned char getPIDFromName(String name) {
  if (name == "PID_COOLANT_TEMP") return PID_COOLANT_TEMP;
  if (name == "PID_ENGINE_RPM") return PID_ENGINE_RPM;
  if (name == "PID_VEHICLE_SPEED") return PID_VEHICLE_SPEED;
  if (name == "PID_RUNTIME") return PID_RUNTIME;
  if (name == "PID_FUEL_LEVEL") return PID_FUEL_LEVEL;
  if (name == "PID_ODOMETER") return PID_ODOMETER;
  return 0xFF; // Invalid PID
}

unsigned long getExtendedPID(String name)
{
  if (carMake == "kia") {
    if (name == "PID_FUEL_LEVEL") return KIA_PID_FUEL_LEVEL;
    if (name == "PID_COOLANT_TEMP") return KIA_PID_COOLANT_TEMP;
    if (name == "PID_ENGINE_RPM") return KIA_PID_ENGINE_RPM;
    if (name == "PID_VEHICLE_SPEED") return KIA_PID_VEHICLE_SPEED;
    if (name == "PID_RUNTIME") return KIA_PID_RUNTIME;
    if (name == "PID_ODOMETER") return KIA_PID_ODOMETER;
  } else if (carMake == "hyundai") {
    if (name == "PID_FUEL_LEVEL") return HYUNDAI_PID_FUEL_LEVEL;
    if (name == "PID_COOLANT_TEMP") return HYUNDAI_PID_COOLANT_TEMP;
    if (name == "PID_ENGINE_RPM") return HYUNDAI_PID_ENGINE_RPM;
    if (name == "PID_VEHICLE_SPEED") return HYUNDAI_PID_VEHICLE_SPEED;
    if (name == "PID_RUNTIME") return HYUNDAI_PID_RUNTIME;
    if (name == "PID_ODOMETER") return HYUNDAI_PID_ODOMETER;
  }
  return 0xFFFFFFFF; // Invalid PID
}


void printSupportedPIDs() {
  Serial.println("Supported PIDs:");
  Serial.println("PID_COOLANT_TEMP: Engine Coolant Temperature (°C)");
  Serial.println("PID_ENGINE_RPM: Engine Speed (rpm)");
  Serial.println("PID_VEHICLE_SPEED: Vehicle Speed (km/h)");
  Serial.println("PID_RUNTIME: Engine Run Time (hh:mm:ss)");
  Serial.println("PID_FUEL_LEVEL: Fuel Level (%)");
  Serial.println("PID_ODOMETER: Odometer (km)");
}

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) { //< -------- - CHANGE if using different board
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }

  //initialise mask and filter to allow only receipt of 0x7xx CAN IDs
  CAN0.init_Mask(0, 0, 0x07000000);              // Init first mask...
  CAN0.init_Mask(1, 0, 0x07000000);              // Init second mask...

  
  for (uint8_t i = 0; i < 6; ++i) {
    CAN0.init_Filt(i, 0, 0x07000000);           //Init filters
  }
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                    // Configuring pin for /INT input

  Serial.println("Enter the car manufacturer (kia or hyundai):");
  while (carMake == "") {
    if (Serial.available()) {
      carMake = Serial.readStringUntil('\n');
      carMake.trim();
      Serial.print("Car make set to: ");
      Serial.println(carMake);
    }
  }
  
  Serial.println("Enter a PID name to request(PID NAME) or type 'pid' to see supported PIDs:");
}

void loop()
{
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "pid") {
      printSupportedPIDs();
    } else {
      unsigned char pid = getPIDFromName(input);
      if (pid != 0xFF) {
        if (isPIDSupported(pid)) {
          sendPID(pid);
          delay(500);
          receivePID(pid); 
        } else {
          Serial.println("Standard PID not supported. Trying extended PID...");
          unsigned long extendedPID = getExtendedPID(input);
          if(extendedPID != 0xFFFFFFFF) {
            sendExtendedPID(extendedPID, (carMake == "kia" ? KIA_CAN_ID : HYUNDAI_CAN_ID));
            delay(500);
            receiveExtendedPID(extendedPID);
          } else {
            Serial.println("Invalid extended PID or car manufacturer");
          }
        }
      } else {
        Serial.println("Invalid PID name. Type 'pid' to see supported PIDs.");
      }
    }
    Serial.println("Enter another PID name to request(PID NAME):");
  }
}
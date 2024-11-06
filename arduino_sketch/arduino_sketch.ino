#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10        // sensors signal pin
#define SERIAL_BAUD_RATE 9600  // serial speed
#define CAMERA_1_FAN_PIN 5     // camera 1 fan pin
#define CAMERA_2_FAN_PIN 6     // camera 2 fan pin
#define CAMERA_3_FAN_PIN 7     // camera 3 fan pin
#define THRESHOLD_TEMP_C 30    // threshold temp value
#define TIMEOUT_MILLIS 5000    // sensors check timeout

// sensor type enum
enum SensorTypes
{
  INTR,
  EXTR
};

// cameras enum
enum Cameras
{
  CAMERA_1,
  CAMERA_2,
  CAMERA_3
};

// creating OneWire object
OneWire oneWire(ONE_WIRE_BUS);

// creating DallasTemperature object
DallasTemperature sensors(&oneWire);

// temperature sensors device addresses
DeviceAddress CAMERA_1_INTERNAL_SENSOR = {0x28, 0xE6, 0x8F, 0x83, 0x00, 0x00, 0x00, 0x70};
DeviceAddress CAMERA_1_EXTERNAL_SENSOR = {0x00, 0xE6, 0x8F, 0x30, 0x00, 0x00, 0x00, 0x70};
DeviceAddress CAMERA_2_INTERNAL_SENSOR = {0x28, 0xF7, 0xC6, 0x84, 0x00, 0x00, 0x00, 0x80};
DeviceAddress CAMERA_2_EXTERNAL_SENSOR = {0x00, 0xF7, 0xC6, 0x84, 0x00, 0x00, 0x00, 0x80};
DeviceAddress CAMERA_3_INTERNAL_SENSOR = {0x28, 0x29, 0x0C, 0x88, 0x00, 0x00, 0x00, 0xDA};
DeviceAddress CAMERA_3_EXTERNAL_SENSOR = {0x00, 0x29, 0x0C, 0x88, 0x00, 0x00, 0x00, 0xDA};

// current temperature
static float currTempC;

const char *sensorTypeToString(SensorTypes sensorType)
{
  switch (sensorType)
  {
  case SensorTypes::INTR:
    return "INTERNAL";
  case SensorTypes::EXTR:
    return "EXTERNAL";
  default:
    return "UNKNOWN";
  }
}

const char *cameraToString(Cameras camera)
{
  switch (camera)
  {
  case Cameras::CAMERA_1:
    return "CAMERA_1";
  case Cameras::CAMERA_2:
    return "CAMERA_2";
  case Cameras::CAMERA_3:
    return "CAMERA_3";
  default:
    return "UNKNOWN";
  }
}

// log temperature
void printTemp(DeviceAddress deviceAddress, SensorTypes sensorType, Cameras camera)
{
  currTempC = sensors.getTempC(deviceAddress);
  Serial.print(cameraToString(camera));
  Serial.print(";");
  Serial.print(sensorTypeToString(sensorType));
  Serial.print("_SENSOR;");
  if (currTempC != DEVICE_DISCONNECTED_C)
  {
    Serial.print(currTempC);
  }
  else
  {
    Serial.print("N/A");
  }
  Serial.println();
}

// start fan if stopped
void startFan(uint8_t fanIdx, Cameras camera)
{
  if (digitalRead(fanIdx) == LOW)
  {
    Serial.print(cameraToString(camera));
    Serial.print(";FAN_STATUS;STARTED");
    digitalWrite(fanIdx, HIGH);
    Serial.println();
  }
}

// stop fan if started
void stopFan(uint8_t fanIdx, Cameras camera)
{
  if (digitalRead(fanIdx) == HIGH)
  {
    Serial.print(cameraToString(camera));
    Serial.print(";FAN_STATUS;STOPPED");
    digitalWrite(fanIdx, LOW);
    Serial.println();
  }
}

// check temperature & fan control
void checkTemp(DeviceAddress deviceAddress, uint8_t fanIdx, Cameras camera)
{
  if (sensors.hasAlarm(deviceAddress))
  {
    currTempC = sensors.getTempC(deviceAddress);
    if (currTempC != DEVICE_DISCONNECTED_C)
    {
      if (currTempC >= sensors.getHighAlarmTemp(deviceAddress))
      {
        startFan(fanIdx, camera);
      }
      else
      {
        stopFan(fanIdx, camera);
      }
    }
    else
    {
      stopFan(fanIdx, camera);
    }
  }
  else
  {
    stopFan(fanIdx, camera);
  }
}

void setup(){

  // start serial
  Serial.begin(SERIAL_BAUD_RATE);

  // wait serial
  while (!Serial)
    ;

  // init sensors
  sensors.begin();

  // set all sensors resolution to 12 bit
  sensors.setResolution(CAMERA_1_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_1_EXTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_2_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_2_EXTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_3_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_3_EXTERNAL_SENSOR, 12);

  // set alarms for internal sensors
  sensors.setHighAlarmTemp(CAMERA_1_INTERNAL_SENSOR, THRESHOLD_TEMP_C);
  sensors.setHighAlarmTemp(CAMERA_2_INTERNAL_SENSOR, THRESHOLD_TEMP_C);
  sensors.setHighAlarmTemp(CAMERA_3_INTERNAL_SENSOR, THRESHOLD_TEMP_C);

  // fan pins set as output
  pinMode(CAMERA_1_FAN_PIN, OUTPUT);
  pinMode(CAMERA_2_FAN_PIN, OUTPUT);
  pinMode(CAMERA_3_FAN_PIN, OUTPUT);
}

void loop()
{
  // update temperatures
  sensors.requestTemperatures();

  // log all temperatures to serial
  printTemp(CAMERA_1_INTERNAL_SENSOR, SensorTypes::INTR, Cameras::CAMERA_1);
  printTemp(CAMERA_1_EXTERNAL_SENSOR, SensorTypes::EXTR, Cameras::CAMERA_1);
  printTemp(CAMERA_2_INTERNAL_SENSOR, SensorTypes::INTR, Cameras::CAMERA_2);
  printTemp(CAMERA_2_EXTERNAL_SENSOR, SensorTypes::EXTR, Cameras::CAMERA_2);
  printTemp(CAMERA_3_INTERNAL_SENSOR, SensorTypes::INTR, Cameras::CAMERA_3);
  printTemp(CAMERA_3_EXTERNAL_SENSOR, SensorTypes::EXTR, Cameras::CAMERA_3);

  // check temperatures inside cameras
  checkTemp(CAMERA_1_INTERNAL_SENSOR, CAMERA_1_FAN_PIN, Cameras::CAMERA_1);
  checkTemp(CAMERA_2_INTERNAL_SENSOR, CAMERA_2_FAN_PIN, Cameras::CAMERA_2);
  checkTemp(CAMERA_3_INTERNAL_SENSOR, CAMERA_3_FAN_PIN, Cameras::CAMERA_3);

  delay(TIMEOUT_MILLIS);
}

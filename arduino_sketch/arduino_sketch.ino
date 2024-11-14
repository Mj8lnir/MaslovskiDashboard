#include <OneWire.h>
#include <DallasTemperature.h>
#include <HX711.h>

#define ONE_WIRE_BUS 4              // sensors signal pin
#define SERIAL_BAUD_RATE 9600       // serial speed
#define CAMERA_1_FAN_PIN 5          // camera 1 fan pin
#define CAMERA_2_FAN_PIN 6          // camera 2 fan pin
#define CAMERA_3_FAN_PIN 7          // camera 3 fan pin
#define CAMERA_1_HX711_SCK_PIN 8    // camera 1 weight sensor SCK pin
#define CAMERA_1_HX711_DT_PIN  9    // camera 1 weight sensor DT pin
#define CAMERA_2_HX711_SCK_PIN 10   // camera 2 weight sensor SCK pin
#define CAMERA_2_HX711_DT_PIN  11   // camera 2 weight sensor DT pin
#define CAMERA_3_HX711_SCK_PIN 12   // camera 3 weight sensor SCK pin
#define CAMERA_3_HX711_DT_PIN  13   // camera 3 weight sensor DT pin
#define THRESHOLD_TEMP_C 80         // threshold temp value
#define TIMEOUT_MILLIS 50           // sensors check timeout

// sensor type enum
enum SensorTypes {
  INTR,
  EXTR
};

// cameras enum
enum Cameras {
  CAMERA_1,
  CAMERA_2,
  CAMERA_3
};

// OneWire object
OneWire oneWire(ONE_WIRE_BUS);

// DallasTemperature object
DallasTemperature sensors(&oneWire);

// camera 1 HX711 object
HX711 camera_1_weight_sensor;

// camera 2 HX711 object
HX711 camera_2_weight_sensor;

// camera 3 HX711 object
HX711 camera_3_weight_sensor;

// temperature sensors device addresses
DeviceAddress CAMERA_1_INTERNAL_SENSOR = {0x28, 0x64, 0xAE, 0x35, 0x00, 0x00, 0x00, 0x59};
DeviceAddress CAMERA_1_EXTERNAL_SENSOR = {0x28, 0xFC, 0x36, 0x43, 0x00, 0x00, 0x00, 0x61};
DeviceAddress CAMERA_2_INTERNAL_SENSOR = {0x28, 0x3C, 0x6B, 0x35, 0x00, 0x00, 0x00, 0x26};
DeviceAddress CAMERA_2_EXTERNAL_SENSOR = {0x28, 0x72, 0x0A, 0x34, 0x00, 0x00, 0x00, 0x13};
DeviceAddress CAMERA_3_INTERNAL_SENSOR = {0x28, 0xEA, 0xF5, 0x36, 0x00, 0x00, 0x00, 0xCD};
DeviceAddress CAMERA_3_EXTERNAL_SENSOR = {0x28, 0xFB, 0xCD, 0x38, 0x00, 0x00, 0x00, 0x0E};

// current temperature
static float correctedValueC, currTempC, RawHigh, RawLow, ReferenceHigh, ReferenceLow, RawRange, ReferenceRange;

// current weight in units and gramms
static float units, gramms;

// camera 1 weight sensor calibration factor
const float CAMERA_1_HX711_CAL_FACTOR = 25.95;

// camera 2 weight sensor calibration factor
const float CAMERA_2_HX711_CAL_FACTOR = 25.03;

// camera 3 weight sensor calibration factor
const float CAMERA_3_HX711_CAL_FACTOR = 25.77;

const char *sensorTypeToString(SensorTypes sensorType) {
  switch (sensorType) {
  case SensorTypes::INTR:
    return "INTERNAL";
  case SensorTypes::EXTR:
    return "EXTERNAL";
  default:
    return "UNKNOWN";
  }
}

const char *cameraToString(Cameras camera) {
  switch (camera) {
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

// reading corrected temperature
float readTempCorrected(DeviceAddress deviceAddress) {
  currTempC = sensors.getTempC(deviceAddress);
  if (deviceAddress == CAMERA_1_INTERNAL_SENSOR) {
    RawHigh = 96.62;
    RawLow = 3.03;
    ReferenceHigh = 96.7;
    ReferenceLow = 5.20;
  } else if (deviceAddress == CAMERA_1_EXTERNAL_SENSOR) {
    RawHigh = 97.31;
    RawLow = 2.19;
    ReferenceHigh = 96.8;
    ReferenceLow = 4.30;
  } else if (deviceAddress == CAMERA_2_INTERNAL_SENSOR) {
    RawHigh = 96.87;
    RawLow = 3.03;
    ReferenceHigh = 97.0;
    ReferenceLow = 5.0;
  } else if (deviceAddress == CAMERA_2_EXTERNAL_SENSOR) {
    RawHigh = 96.44;
    RawLow = 2.13;
    ReferenceHigh = 96.2;
    ReferenceLow = 4.52;
  } else if (deviceAddress == CAMERA_3_INTERNAL_SENSOR) {
    RawHigh = 97.12;
    RawLow = 2.75;
    ReferenceHigh = 95.6;
    ReferenceLow = 4.90;
  } else if (deviceAddress == CAMERA_3_EXTERNAL_SENSOR) {
    RawHigh = 97.0;
    RawLow = 2.44;
    ReferenceHigh = 96.2;
    ReferenceLow = 4.40;
  } else {
    return DEVICE_DISCONNECTED_C;
  }
  if (currTempC != DEVICE_DISCONNECTED_C) {
    RawRange = RawHigh - RawLow;
    ReferenceRange = ReferenceHigh - ReferenceLow;
    return (((currTempC - RawLow) * ReferenceRange) / RawRange) + ReferenceLow;
  } else {
    return DEVICE_DISCONNECTED_C;
  }
}

// log temperature
void printTemp(DeviceAddress deviceAddress, SensorTypes sensorType, Cameras camera) {
  correctedValueC = readTempCorrected(deviceAddress);
  Serial.print(cameraToString(camera));
  Serial.print(";");
  Serial.print(sensorTypeToString(sensorType));
  Serial.print("_SENSOR;");
  if (correctedValueC != DEVICE_DISCONNECTED_C) {
    Serial.print(correctedValueC);
  } else {
    Serial.print("N/A");
  }
  Serial.println();
}

// start fan if stopped
void startFan(uint8_t fanIdx, Cameras camera) {
  if (digitalRead(fanIdx) == LOW) {
    Serial.print(cameraToString(camera));
    Serial.print(";FAN_STATUS;STARTED");
    digitalWrite(fanIdx, HIGH);
    Serial.println();
  }
}

// stop fan if started
void stopFan(uint8_t fanIdx, Cameras camera) {
  if (digitalRead(fanIdx) == HIGH) {
    Serial.print(cameraToString(camera));
    Serial.print(";FAN_STATUS;STOPPED");
    digitalWrite(fanIdx, LOW);
    Serial.println();
  }
}

// check temperature & fan control
void checkTemp(DeviceAddress deviceAddress, uint8_t fanIdx, Cameras camera) {
    correctedValueC = readTempCorrected(deviceAddress);
    if (correctedValueC != DEVICE_DISCONNECTED_C) {
      if (correctedValueC >= THRESHOLD_TEMP_C) {
        startFan(fanIdx, camera);
      } else{
        stopFan(fanIdx, camera);
      }
    } else {
      stopFan(fanIdx, camera);
    }
}

// log weight
void printWeight(HX711 hx711, Cameras camera) {
  if (hx711.is_ready()) {
    for (int i = 0; i < 10; i ++)
    {
      units = + hx711.get_units(), 10;
    }
    units = units / 10;
    if (units < 0)
    {
      units = 0.00;
    }
    gramms = units * 0.35274;
    Serial.print(cameraToString(camera));
    Serial.print(";WEIGHT_SENSOR;");
    Serial.print(gramms);
    Serial.println();
    units = 0.00;
  } else {
    Serial.print(cameraToString(camera));
    Serial.print(";WEIGHT_SENSOR;N/A");
    Serial.println();
  }
}

void setup() {

  // start serial
  Serial.begin(SERIAL_BAUD_RATE);

  // wait serial
  while (!Serial)
    ;

  // init temperature sensors
  sensors.begin();

  // set all sensors resolution to 12 bit
  sensors.setResolution(CAMERA_1_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_1_EXTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_2_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_2_EXTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_3_INTERNAL_SENSOR, 12);
  sensors.setResolution(CAMERA_3_EXTERNAL_SENSOR, 12);

  // fan pins set as output
  pinMode(CAMERA_1_FAN_PIN, OUTPUT);
  pinMode(CAMERA_2_FAN_PIN, OUTPUT);
  pinMode(CAMERA_3_FAN_PIN, OUTPUT);

  // init HX711 sensors
  camera_1_weight_sensor.begin(CAMERA_1_HX711_DT_PIN, CAMERA_1_HX711_SCK_PIN);
  camera_2_weight_sensor.begin(CAMERA_2_HX711_DT_PIN, CAMERA_2_HX711_SCK_PIN);
  camera_3_weight_sensor.begin(CAMERA_3_HX711_DT_PIN, CAMERA_3_HX711_SCK_PIN);

  camera_1_weight_sensor.set_scale();
  camera_2_weight_sensor.set_scale();
  camera_3_weight_sensor.set_scale();

  camera_1_weight_sensor.tare();
  camera_2_weight_sensor.tare();
  camera_3_weight_sensor.tare();

  camera_1_weight_sensor.set_scale(CAMERA_1_HX711_CAL_FACTOR);
  camera_2_weight_sensor.set_scale(CAMERA_2_HX711_CAL_FACTOR);
  camera_3_weight_sensor.set_scale(CAMERA_3_HX711_CAL_FACTOR);

  // update temperatures
  sensors.requestTemperatures();
}

void loop() {

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

  // log weight to serial
  printWeight(camera_1_weight_sensor, Cameras::CAMERA_1);
  printWeight(camera_2_weight_sensor, Cameras::CAMERA_2);
  printWeight(camera_3_weight_sensor, Cameras::CAMERA_3);

  delay(TIMEOUT_MILLIS);
}

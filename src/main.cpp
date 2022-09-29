#include <Arduino.h>
#include <SimpleFOC.h>

#define I2CADDR 0x01

MbedSPI spi(4,3,2);
MbedI2C wire(16,17);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(8, 11, 9, 12, 10, 13);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

float target_voltage = 0;

void rcv(int r) {
  // Serial.println("rcv");
  while (wire.available() > 0) {
    if (wire.read() == 0x02)
      wire.readBytesUntil(0x04, (uint8_t*)&target_voltage, 4);
  }
}

uint8_t req_buf[6];
float req_angle;
void req() {
  // Serial.println("req");
  req_angle = sensor.getAngle();
  req_buf[0] = 0x02;
  memcpy(req_buf+1, &req_angle, 4);
  req_buf[5] = 0x04;
  wire.write(req_buf, 6);
}

void setup() {
  wire.begin(I2CADDR);
  wire.onRequest(req);
  wire.onReceive(rcv);

  sensor.init(&spi);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  Serial.begin(115200);
  // motor.useMonitoring(Serial);
  
  motor.init();
  // motor.initFOC();
  motor.initFOC(2.36, Direction::CW);
}

void loop() {
  motor.loopFOC();
  motor.move(target_voltage);
  // motor.monitor();
  // Serial.println("go");
}
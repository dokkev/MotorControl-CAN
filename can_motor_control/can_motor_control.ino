#include <CAN.h>
#include <SimpleFOC.h>

#define TX_GPIO_NUM 26 // Connects to CTX
#define RX_GPIO_NUM 27 // Connects to CRX

#define HSPI_MISO 19
#define HSPI_MOSI 23
#define HSPI_SCLK 18
#define HSPI_SS 5

#define PWM_A 15
#define PWM_B 2
#define PWM_C 4
#define EN 16

#define CAN_RX_ID = 0x11;
#define CAN_TX_ID = 0x21;

#define MOTOR_POLE_PAIRS 8
#define MOTOR_PHASE_RESISTANCE 2.240

const float torque_constant = 0.0557; // Nm/A


// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, HSPI_SS);

// for esp 32, it has 2 spi interfaces VSPI (default) and HPSI as the second one
// to enable it instatiate the object
SPIClass SPI_2(HSPI);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_A, PWM_B, PWM_C, EN);
Commander command = Commander(Serial);
// target variable
float target_torque = 0;
void doTarget(char *cmd) { command.scalar(&target_torque, cmd); }


//==================================================================================//

void setup() {
  Serial.begin(115200);
  // while (!Serial)
  //   ;
  delay(1000);

  Serial.println("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 250 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  } else {
    Serial.println("CAN Initialized");
  }

  command.add('T', doTarget, "target");
  // Start Motor
  configMotor();
}

//==================================================================================//

void loop() {
  // user communication
  // command.run();

  target_torque = canReceive(CAN_RX_ID);

  motor.loopFOC();

  if (target_torque > 0.5)
    target_torque = 0.5;
  else if (target_torque < -0.5)
    target_torque = -0.5;

  // float target_current = target_torque / torque_constant // Nm/A
  motor.move(target_torque);
  Serial.print("target_torque: ");
  Serial.println(target_torque);

  sensor.update();
  Serial.print("current position: ");
  Serial.println(sensor.getAngle());
  // send angle over CAN
  
}

//==================================================================================//

void canSender(double data) {

  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(CAN_TX_ID ); // sets the ID and clears the transmit buffer
  // CAN.beginExtendedPacket(0xabcdef);
  // write data to buffer. data is not sent until endPacket() is
  // called.
  uint8_t dataBuffer[8];
  memcpy(dataBuffer, &data, sizeof(double));
  CAN.write(dataBuffer, sizeof(dataBuffer));
  CAN.endPacket();
  Serial.print("packet end ... ");


  Serial.println("done");
}


//==================================================================================//

double canReceive(int id) {
  // Try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // Received a packet
    if (!CAN.packetExtended() && CAN.packetId() == id) {
      // Check if the packet size matches the expected size of a float (4 bytes)
      if (packetSize == sizeof(double)) {
        double receivedData; // Declare the received float variable

        // Read the 8 bytes of the double from the CAN packet and store them in
        // receivedData
        CAN.readBytes((char *)&receivedData, sizeof(double));
        Serial.print("Received data: ");
        Serial.println(receivedData);
        canSender(sensor.getAngle());
        return receivedData; // Return the received float
      }
    }
  }

  // Return a default value (you can choose a meaningful default)
  return 0.0;
}

//==================================================================================//

void configMotor() {
  // start the newly defined spi communication
  SPI_2.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); // SCLK, MISO, MOSI, SS
  // initialise magnetic sensor hardware
  sensor.init(&SPI_2);
  motor.linkSensor(&sensor);

  // Set Driver
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 5;
  driver.init();
  motor.linkDriver(&driver);

  // FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque; // it sends voltage without phase resistance defined
                                                // make sure to set phase resistance or current sense to command current

  // Set Motor Limits
  motor.velocity_limit = 3;
  motor.voltage_limit = 5;

  // aligning voltage
  motor.voltage_sensor_align = 3;

  // Init Motor
  motor.init();
  motor.initFOC();

  // Serial.println("Motor Init");
}
//==================================================================================//

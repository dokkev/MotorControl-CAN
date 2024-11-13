```c++
//==================================================================================//
#include <CAN.h>
#define TX_GPIO_NUM 17 // Connects to CTX
#define RX_GPIO_NUM 16 // Connects to CRX
//==================================================================================//
void setup() {
 Serial.begin (115200);
 while (!Serial);
 delay (1000);
 Serial.println ("CAN Receiver/Receiver");
 // Set the pins
 CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
 // start the CAN bus at 500 kbps (Baudrate is cut in half)
 if (!CAN.begin (1000E3)) {
 Serial.println ("Starting CAN failed!");
 while (1);
 }
 else {
 Serial.println ("CAN Initialized");
 }
}
//==================================================================================//
void loop() {
 canSender();
 canReceiver();
}
//==================================================================================//
void canSender() {
 // send packet: id is 11 bits, packet can contain up to 8 bytes of data
 Serial.print ("Sending packet ... ");
 CAN.beginPacket (0x12); //sets the ID and clears the transmit buffer
 // CAN.beginExtendedPacket(0xabcdef);
 CAN.write ('h'); //write data to buffer. data is not sent until endPacket() is called.
 CAN.write ('e');
 CAN.write ('l');
 CAN.write ('l');
 CAN.write ('o');
 CAN.endPacket();
 Serial.print ("packet end ... ");
 //RTR packet with a requested data length
 CAN.beginPacket (0x12, 3, true);
 CAN.endPacket();
 Serial.println ("done");
 delay (1000);
}
//==================================================================================//
void canReceiver() {
 // try to parse packet
 int packetSize = CAN.parsePacket();
 if (packetSize) {
 // received a packet
 Serial.print ("Received ");
 if (CAN.packetExtended()) {
 Serial.print ("extended ");
 }
 if (CAN.packetRtr()) {
 // Remote transmission request, packet contains no data
 Serial.print ("RTR ");
 }
 Serial.print ("packet with id 0x");
 Serial.print (CAN.packetId(), HEX);
 if (CAN.packetRtr()) {
 Serial.print (" and requested length ");
 Serial.println (CAN.packetDlc());
 } else {
 Serial.print (" and length ");
 Serial.println (packetSize);
 // only print packet data for non-RTR packets
 while (CAN.available()) {
 Serial.print ((char) CAN.read());
 }
 Serial.println();
 }
 Serial.println();
 }
}
//==================================================================================//
```
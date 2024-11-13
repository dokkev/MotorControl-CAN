#include "CANInterface.hpp"

namespace can_interface {

CANInterface::CANInterface(uint32_t baudrate) : baudrate_(baudrate) {}

CANInterface::~CANInterface() {
    // End CAN on object destruction
    CAN.end();
}

bool CANInterface::init() {
    // Start CAN bus with specified baudrate
    if (!CAN.begin(baudrate_)) {
        Serial.println("CANInterface::init: Starting CAN Failed");
        return false;
    }
    Serial.println("CANInterface::init: CAN Started started with baudrate: " + String(baudrate_));

    return true;
}

bool CANInterface::set_can_filter(const uint16_t filter_id1, const uint16_t filter_id2) {
    // Set two filters for the CAN bus; returns true if successful
    bool allowRollover = true;
    uint16_t mask0 = 0x7FF;        // Match all bits for filter 0 and filter 1
    uint16_t filter0 = filter_id1;      // Accept only messages with ID 0x1
    uint16_t filter1 = filter_id2;      // Accept only messages with ID 0x456

    uint16_t mask1 = 0x7FF;        // Match all bits for filter 2, 3, 4, 5
    uint16_t filter2 = 0x00;      // Unused filters (set to 0 if not needed)
    uint16_t filter3 = 0x00;      // Unused filters
    uint16_t filter4 = 0x00;      // Unused filters
    uint16_t filter5 = 0x00;      // Unused filters

    if (CAN.setFilterRegisters(mask0, filter0, filter1, mask1, filter2, filter3, filter4, filter5, allowRollover)) {
        Serial.println("CANInterface::set_can_filter : Filters applied successfully for ID: 0x" + String(filter_id1, HEX) + " and 0x" + String(filter_id2, HEX));
    } 
    else {
        Serial.println("CANInterface::set_can_filter : Failed to apply filters!");
        return false;
    }
    return true;
}

bool CANInterface::send_cb(const CANMsg &msg) {
    // Start packet transmission on the specified ID
    if (CAN.beginPacket(msg.ID)) {
        // Send each byte in DATA array, up to LEN
        for (int i = 0; i < msg.LEN; i++) {
            CAN.write(msg.DATA[i]);
        }
        // End packet transmission
        CAN.endPacket();
        // delay(1); 
        // Serial.println("CANInterface::send_cb : Sent message with ID: 0x" + String(msg.ID, HEX));
        return true;
    }
    return false;
}

bool CANInterface::receive_cb(CANMsg &msg) {
    // Check if a CAN packet is available
    int packet_size = CAN.parsePacket();
    if (packet_size > 0) {
        // Store packet ID and length
        msg.ID = CAN.packetId();
        msg.LEN = packet_size;

        // Read packet data into msg.DATA
        for (int i = 0; i < packet_size; i++) {
            int r = CAN.read();
            if (r == -1) {
                Serial.println("Error reading CAN data");
                return false; // Return false if there's an error reading
            }
            msg.DATA[i] = static_cast<uint8_t>(r);
        }
        return true; // Return true if packet was successfully read
    }
    return false; // No packet available
}

void CANInterface::print_message(const CANMsg &msg) {
    // Print the message ID and data
    Serial.print("ID: 0x");
    Serial.print(msg.ID, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < msg.LEN; i++) {
        Serial.print(msg.DATA[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

}  // namespace can_interface

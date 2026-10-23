#pragma once

#include <stdint.h>

// Send one 11-bit (standard ID) CAN data frame, bypassing DroneCAN.
//
// libArduinoDroneCAN only sends 29-bit frames and only through bxCAN transmit
// mailbox 0, so this writes mailbox 1 or 2 (whichever is free) directly;
// DroneCAN traffic is not affected. Nothing waits: if both still hold earlier
// frames (for example because no other node on the bus acknowledges them)
// the call returns false and the frame is dropped.
//
// STM32L431 (Micro Node) only. Returns false on other boards.
bool raw_can_send_std(uint16_t id, const uint8_t *data, uint8_t len);

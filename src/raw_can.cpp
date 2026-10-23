#include "raw_can.h"

#include <Arduino.h>
#include <RawCanFrame.h>

#if defined(CANL431)

// Transmit mailboxes used for these frames. libArduinoDroneCAN's CANSend()
// (canL431.cpp) uses mailbox 0 only; if a future version starts using the
// others, this needs revisiting. Two mailboxes let two frames go out back to
// back (the filtered and unfiltered frames of one sample).
static const uint32_t MAILBOX_EMPTY[] = {CAN_TSR_TME1, CAN_TSR_TME2};
static const uint8_t MAILBOX_INDEX[] = {1, 2};

bool raw_can_send_std(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (id == 0 || id > RAW_CAN_ID_MAX || len > 8) {
        return false;
    }

    int8_t slot = -1;
    const uint32_t tsr = CAN1->TSR;
    for (uint8_t i = 0; i < sizeof(MAILBOX_INDEX); i++) {
        if (tsr & MAILBOX_EMPTY[i]) {
            slot = (int8_t)MAILBOX_INDEX[i];
            break;
        }
    }
    if (slot < 0) {
        return false; // both still waiting for the bus
    }

    uint8_t b[8] = {};
    memcpy(b, data, len);

    CAN_TxMailBox_TypeDef &mb = CAN1->sTxMailBox[slot];
    mb.TDTR = (mb.TDTR & ~CAN_TDT0R_DLC) | len;
    mb.TDLR = ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
    mb.TDHR = ((uint32_t)b[7] << 24) | ((uint32_t)b[6] << 16) | ((uint32_t)b[5] << 8) | b[4];
    // Standard identifier: IDE and RTR clear. Setting TXRQ starts the transfer.
    mb.TIR = ((uint32_t)id << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ;
    return true;
}

#else

bool raw_can_send_std(uint16_t, const uint8_t *, uint8_t)
{
    return false;
}

#endif

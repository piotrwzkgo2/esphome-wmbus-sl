#include "rf_cc1101.h"

namespace esphome {
namespace wmbus {

  static const char *TAG = "rxLoop";

  bool RxLoop::init(uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t cs,
                    uint8_t gdo0, uint8_t gdo2, float freq, bool syncMode) {
    bool retVal = false;
    this->syncMode = syncMode;
    this->gdo0 = gdo0;
    this->gdo2 = gdo2;
    pinMode(this->gdo0, INPUT);
    pinMode(this->gdo2, INPUT);
    ELECHOUSE_cc1101.setSpiPin(clk, miso, mosi, cs);

    ELECHOUSE_cc1101.Init();

    for (uint8_t i = 0; i < TMODE_RF_SETTINGS_LEN; i++) {
      ELECHOUSE_cc1101.SpiWriteReg(TMODE_RF_SETTINGS_BYTES[i << 1],
                                   TMODE_RF_SETTINGS_BYTES[(i << 1) + 1]);
    }

    uint32_t freq_reg = uint32_t(freq * 65536 / 26);
    uint8_t freq2 = (freq_reg >> 16) & 0xFF;
    uint8_t freq1 = (freq_reg >> 8) & 0xFF;
    uint8_t freq0 = freq_reg & 0xFF;

    ESP_LOGD(TAG, "Set CC1101 frequency to %3.3fMHz [%02X %02X %02X]",
             freq/1.0, freq2, freq1, freq0);
             // don't use setMHZ() -- seems to be broken, or used in wrong place
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ2, freq2);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ1, freq1);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ0, freq0);

    ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);

    byte cc1101Version = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);

    if ((cc1101Version != 0) && (cc1101Version != 255)) {
      retVal = true;
      ESP_LOGD(TAG, "CC1101 version '%d'", cc1101Version);
      ELECHOUSE_cc1101.SetRx();
      ESP_LOGD(TAG, "CC1101 initialized");
      delay(4);
    }
    else {
      ESP_LOGE(TAG, "CC1101 initialization FAILED!");
    }

    return retVal;
  }

  bool RxLoop::task() {
    do {
      switch (rxLoop.state) {
        case INIT_RX:
          start();
          return false;

        // RX active, waiting for SYNC
        case WAIT_FOR_SYNC:
          if (digitalRead(this->gdo2)) { // assert when SYNC detected
            rxLoop.state = WAIT_FOR_DATA;
            sync_time_ = millis();
          }
          break;

        // waiting for enough data in Rx FIFO buffer
        case WAIT_FOR_DATA:
          if (digitalRead(this->gdo0)) { // assert when Rx FIFO buffer threshold reached
            uint8_t preamble[2];
            // Read the 3 first bytes,
            ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, 3);
            rxLoop.bytesRx = 3;
            const uint8_t *currentByte = rxLoop.pByteIndex;
            // Mode C
            if (*currentByte == WMBUS_MODE_C_PREAMBLE) {
              currentByte++;
              data_in.mode = 'C';
              // Block A
              if (*currentByte == WMBUS_BLOCK_A_PREAMBLE) {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + packetSize(rxLoop.lengthField);
                data_in.block = 'A';
              }
              // Block B
              else if (*currentByte == WMBUS_BLOCK_B_PREAMBLE) {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + 1 + rxLoop.lengthField;
                data_in.block = 'B';
              }
              // Unknown type, reinit loop
              else {
                rxLoop.state = INIT_RX;
                return false;
              }
              // don't include C "preamble"
              *(rxLoop.pByteIndex) = rxLoop.lengthField;
              rxLoop.pByteIndex += 1;
            }
            // Mode T Block A
            else if (decode3OutOf6(rxLoop.pByteIndex, preamble)) {
              rxLoop.lengthField  = preamble[0];
              data_in.lengthField = rxLoop.lengthField;
              rxLoop.length  = byteSize(packetSize(rxLoop.lengthField));
              data_in.mode   = 'T';
              data_in.block  = 'A';
              rxLoop.pByteIndex += 3;
            }
            // Unknown mode, reinit loop
            else {
              rxLoop.state = INIT_RX;
              return false;
            }

            rxLoop.bytesLeft = rxLoop.length - 3;

            if (rxLoop.length < MAX_FIXED_LENGTH) {
              // Set CC1101 into length mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)rxLoop.length);
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            else {
              // Set CC1101 into infinite mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)(rxLoop.length%MAX_FIXED_LENGTH));
            }

            rxLoop.state = READ_DATA;
            max_wait_time_ += extra_time_;

            ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
          }
          break;

        // waiting for more data in Rx FIFO buffer
        case READ_DATA:
          if (digitalRead(this->gdo0)) { // assert when Rx FIFO buffer threshold reached
            if ((rxLoop.bytesLeft < MAX_FIXED_LENGTH) && (rxLoop.cc1101Mode == INFINITE)) {
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            // Do not empty the Rx FIFO (See the CC1101 SWRZ020E errata note)
            uint8_t bytesInFIFO = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x7F;
            ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, bytesInFIFO - 1);

            rxLoop.bytesLeft  -= (bytesInFIFO - 1);
            rxLoop.pByteIndex += (bytesInFIFO - 1);
            rxLoop.bytesRx    += (bytesInFIFO - 1);
            max_wait_time_    += extra_time_;
          }
          break;
      }

      uint8_t overfl = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x80;
      // end of packet in length mode
      if ((!overfl) && (!digitalRead(gdo2))  && (rxLoop.state > WAIT_FOR_DATA)) {
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, (uint8_t)rxLoop.bytesLeft);
        rxLoop.bytesRx += rxLoop.bytesLeft;
        data_in.length  = rxLoop.bytesRx;
        this->returnFrame.rssi  = (int8_t)ELECHOUSE_cc1101.getRssi();
        this->returnFrame.lqi   = (uint8_t)ELECHOUSE_cc1101.getLqi();
        ESP_LOGV(TAG, "Have %d bytes from CC1101 Rx, RSSI: %d dBm LQI: %d", rxLoop.bytesRx, this->returnFrame.rssi, this->returnFrame.lqi);
        if (rxLoop.length != data_in.length) {
          ESP_LOGE(TAG, "Length problem: req(%d) != rx(%d)", rxLoop.length, data_in.length);
        }
        if (this->syncMode) {
          ESP_LOGV(TAG, "Synchronus mode enabled.");
        }
        if (mBusDecode(data_in, this->returnFrame)) {
          rxLoop.complete = true;
          this->returnFrame.mode  = data_in.mode;
          this->returnFrame.block = data_in.block;
        }
        rxLoop.state = INIT_RX;
        return rxLoop.complete;
      }
      start(false);
    } while ((this->syncMode) && (rxLoop.state > WAIT_FOR_SYNC));
    return rxLoop.complete;
  }

  WMbusFrame RxLoop::get_frame() {
    return this->returnFrame;
  }

bool RxLoop::start(bool force) {
  // waiting too long for next part of data?
  bool reinit_needed = ((now_ms_() - this->sync_time_) > this->max_wait_time_) ? true : false;
  if (!force) {
    if (!reinit_needed) {
      // already in RX?
      if (this->spi_read_status(0x35) == MARCSTATE_RX) {  // MARCSTATE
        return false;
      }
    }
  }

  // init RX here, each time we're idle
  this->rxLoop.state = INIT_RX;
  this->sync_time_ = now_ms_();
  this->max_wait_time_ = this->extra_time_;

  this->spi_strobe(0x36);  // SIDLE
  while ((this->spi_read_status(0x35) & 0x1F) != MARCSTATE_IDLE) {
    delay_ms_(1);
  }
  this->spi_strobe(0x3B);  // SFTX - flush Tx FIFO
  this->spi_strobe(0x3A);  // SFRX - flush Rx FIFO

  // Initialize RX info variable
  this->rxLoop.lengthField = 0;
  this->rxLoop.length = 0;
  this->rxLoop.bytesLeft = 0;
  this->rxLoop.bytesRx = 0;
  this->rxLoop.pByteIndex = this->data_in.data;
  this->rxLoop.complete = false;
  this->rxLoop.cc1101Mode = INFINITE;

  this->returnFrame.frame.clear();
  this->returnFrame.rssi = 0;
  this->returnFrame.lqi = 0;
  this->returnFrame.mode = 'x';
  this->returnFrame.block = 'x';

  std::fill(std::begin(this->data_in.data), std::end(this->data_in.data), 0);
  this->data_in.length = 0;
  this->data_in.lengthField = 0;
  this->data_in.mode = 'x';
  this->data_in.block = 'x';

  // Set Rx FIFO threshold
  this->spi_write_reg(0x03, RX_FIFO_START_THRESHOLD);  // FIFOTHR
  // Set infinite length
  this->spi_write_reg(0x08, INFINITE_PACKET_LENGTH);   // PKTCTRL0

  this->spi_strobe(0x34);  // SRX
  while ((this->spi_read_status(0x35) & 0x1F) != MARCSTATE_RX) {
    delay_ms_(1);
  }

  this->rxLoop.state = WAIT_FOR_SYNC;

  return true;  // this will indicate we just have re-started Rx
}
  }

}
}

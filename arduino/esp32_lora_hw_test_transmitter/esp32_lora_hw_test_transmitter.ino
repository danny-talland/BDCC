#include "Arduino.h"
#include "LoRaWan_APP.h"

namespace {

constexpr uint32_t RF_FREQUENCY = 868000000;
constexpr int8_t TX_OUTPUT_POWER = 5;
constexpr uint8_t LORA_BANDWIDTH = 0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 1;
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
constexpr bool LORA_FIXED_LENGTH = false;
constexpr bool LORA_IQ_INVERSION = false;
constexpr uint32_t LORA_TX_TIMEOUT_MS = 3000;
constexpr uint32_t SEND_INTERVAL_MS = 1000;

constexpr uint8_t TX_BUFFER_SIZE = 40;

static RadioEvents_t radioEvents;
bool loraIdle = true;
uint16_t sequence = 0;
uint32_t lastSendMs = 0;
char txPacket[TX_BUFFER_SIZE] = {};

void sendTestFrame() {
  snprintf(txPacket, sizeof(txPacket), "BDCC,%u,%lu", sequence++, static_cast<unsigned long>(millis()));

  Serial.printf("sending packet \"%s\", length %d\n", txPacket, strlen(txPacket));
  Radio.Send(reinterpret_cast<uint8_t *>(txPacket), strlen(txPacket));
  loraIdle = false;
}

void onTxDone() {
  Serial.println("TX done");
  loraIdle = true;
}

void onTxTimeout() {
  Serial.println("TX timeout");
  Radio.Sleep();
  loraIdle = true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  radioEvents.TxDone = onTxDone;
  radioEvents.TxTimeout = onTxTimeout;
  Radio.Init(&radioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIXED_LENGTH,
                    true, 0, 0, LORA_IQ_INVERSION, LORA_TX_TIMEOUT_MS);

  Serial.println("ESP32 LoRa hardware test transmitter");
}

void loop() {
  Radio.IrqProcess();

  const uint32_t now = millis();
  if (loraIdle && (now - lastSendMs) >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendTestFrame();
  }
}

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <string.h>

namespace {

constexpr uint32_t RF_FREQUENCY = 868000000;
constexpr uint8_t LORA_BANDWIDTH = 0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 1;
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
constexpr uint16_t LORA_SYMBOL_TIMEOUT = 0;
constexpr bool LORA_FIXED_LENGTH = false;
constexpr bool LORA_IQ_INVERSION = false;
constexpr uint32_t LINK_TIMEOUT_MS = 3000;
constexpr uint32_t DISPLAY_UPDATE_MS = 150;

constexpr uint16_t TEST_MAGIC = 0xBEEF;
constexpr uint8_t TEST_VERSION = 1;

struct TestFrame {
  uint16_t magic;
  uint8_t version;
  uint16_t sequence;
  uint32_t uptimeMs;
  uint8_t checksum;
};

#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

static RadioEvents_t radioEvents;
volatile bool pendingFrameReady = false;
TestFrame pendingFrame{};
bool loraIdle = true;
bool linkSeen = false;
uint16_t lastSequence = 0;
uint16_t receivedCount = 0;
uint16_t droppedCount = 0;
int16_t lastRssi = 0;
int8_t lastSnr = 0;
uint32_t lastRxMs = 0;
uint32_t lastDisplayMs = 0;

uint8_t checksumFrame(const TestFrame &frame) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
  uint8_t x = 0;
  for (uint8_t i = 0; i < sizeof(TestFrame) - 1; ++i) {
    x ^= bytes[i];
  }
  return x;
}

void drawNoLinkIcon() {
  display.drawLine(48, 44, 64, 16);
  display.drawCircle(64, 16, 4);
  display.drawCircle(64, 16, 14);
  display.drawCircle(64, 16, 24);
  display.drawLine(36, 12, 92, 56);
  display.drawLine(92, 12, 36, 56);
}

void drawLinkIcon() {
  display.drawCircle(64, 34, 8);
  display.drawCircle(64, 34, 20);
  display.drawCircle(64, 34, 31);
  display.fillCircle(64, 34, 4);
}

void drawPacketFlash() {
  display.drawRect(34, 19, 36, 26);
  display.drawLine(34, 19, 52, 34);
  display.drawLine(70, 19, 52, 34);
  display.drawLine(82, 32, 108, 19);
  display.drawLine(82, 32, 108, 45);
  display.drawLine(108, 19, 108, 45);
  display.fillRect(98, 26, 10, 13);
}

void drawDropIcon() {
  display.drawCircle(64, 34, 25);
  display.fillRect(60, 15, 8, 29);
  display.fillCircle(64, 52, 4);
}

void updateDisplay(bool packetFlash) {
  const uint32_t now = millis();
  if ((now - lastDisplayMs) < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayMs = now;

  const bool linkAlive = linkSeen && (now - lastRxMs) <= LINK_TIMEOUT_MS;
  display.clear();
  display.setColor(WHITE);

  if (!linkAlive) {
    drawNoLinkIcon();
  } else if (droppedCount > 0 && (now / 700) % 2 == 0) {
    drawDropIcon();
  } else if (packetFlash) {
    drawPacketFlash();
  } else {
    drawLinkIcon();
  }

  display.display();
}

void processPendingFrame() {
  if (!pendingFrameReady) {
    return;
  }

  TestFrame frame{};
  noInterrupts();
  memcpy(&frame, &pendingFrame, sizeof(frame));
  pendingFrameReady = false;
  interrupts();

  if (frame.magic != TEST_MAGIC ||
      frame.version != TEST_VERSION ||
      frame.checksum != checksumFrame(frame)) {
    ++droppedCount;
    return;
  }

  if (linkSeen) {
    const uint16_t expected = static_cast<uint16_t>(lastSequence + 1);
    if (frame.sequence != expected) {
      droppedCount += static_cast<uint16_t>(frame.sequence - expected);
    }
  }

  linkSeen = true;
  lastSequence = frame.sequence;
  lastRxMs = millis();
  ++receivedCount;

  Serial.printf("RX seq=%u rssi=%d snr=%d received=%u dropped=%u\n",
                frame.sequence,
                lastRssi,
                lastSnr,
                receivedCount,
                droppedCount);
}

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size == sizeof(TestFrame)) {
    memcpy(&pendingFrame, payload, sizeof(TestFrame));
    pendingFrameReady = true;
    lastRssi = rssi;
    lastSnr = snr;
  } else {
    ++droppedCount;
  }

  Radio.Sleep();
  loraIdle = true;
}

void startReceiveIfNeeded() {
  if (!loraIdle) {
    return;
  }
  loraIdle = false;
  Radio.Rx(0);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  display.init();
  display.clear();
  drawNoLinkIcon();
  display.display();

  radioEvents.RxDone = onRxDone;
  Radio.Init(&radioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODING_RATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIXED_LENGTH,
                    0, true, 0, 0, LORA_IQ_INVERSION, true);

  Serial.println("ESP32 LoRa hardware test receiver");
}

void loop() {
  startReceiveIfNeeded();
  Radio.IrqProcess();

  const bool hadPendingFrame = pendingFrameReady;
  processPendingFrame();
  updateDisplay(hadPendingFrame);
}

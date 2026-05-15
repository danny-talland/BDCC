#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp32-hal-rmt.h"
#include <stdarg.h>
#include <string.h>

namespace {

constexpr uint8_t PIN_DCC_OUT_A = 3;
constexpr uint8_t PIN_DCC_OUT_B = 4;
constexpr uint8_t PIN_HBRIDGE_EN = 5;
constexpr uint8_t PIN_STATUS_LED = 8;
constexpr bool ENABLE_HBRIDGE_EN_PIN = false;
constexpr bool STATUS_LED_ACTIVE_HIGH = true;
constexpr bool DEBUG_SERIAL = true;

constexpr uint8_t ESPNOW_CHANNEL = 6;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint16_t PROTOCOL_MAGIC = 0xBDCC;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint8_t MAX_PACKET_SLOTS = 24;
constexpr uint8_t MAX_LOCOS = 20;
constexpr uint8_t RECEIVER_CONFIG_ADDRESS = 99;

constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;
constexpr uint8_t DCC_PREAMBLE_BITS = 14;
constexpr uint8_t RMT_MAX_PACKET_ITEMS = 80;
constexpr uint16_t DCC_1_HALFPERIOD = 58;
constexpr uint16_t DCC_0_HALFPERIOD = 100;
constexpr uint32_t PACKET_SLOT_EXPIRE_MS = 10000;
constexpr uint32_t PAYLOAD_ACTIVITY_MS = 120;
constexpr uint32_t STARTUP_LED_MS = 1000;
constexpr uint32_t DEBUG_FRAME_INTERVAL_MS = 500;
constexpr uint32_t DEBUG_STATS_INTERVAL_MS = 5000;
constexpr uint16_t EEPROM_SIZE = 64;
constexpr uint16_t EEPROM_CONFIG_ADDRESS = 0;
constexpr uint16_t CONFIG_MAGIC = 0xBDCC;
constexpr uint8_t CONFIG_VERSION = 1;

constexpr uint16_t CV_LINK_TIMEOUT_TICKS = 901;
constexpr uint16_t CV_LINK_DOWN_BRAKE_TICKS = 902;
constexpr uint16_t CV_LINK_UP_ACCEL_TICKS = 903;
constexpr uint16_t CV_MAX_RESUME_SPEED = 904;
constexpr uint16_t CV_LINK_UP_BEHAVIOR = 905;
constexpr uint16_t CV_CONFIG_COMMAND = 908;
constexpr uint8_t CV_COMMAND_SAVE = 1;
constexpr uint8_t CV_COMMAND_FACTORY_RESET = 8;
constexpr uint8_t POM_WRITE_BYTE_MASK = 0xFC;
constexpr uint8_t POM_WRITE_BYTE_OPCODE = 0xEC;

enum FrameType : uint8_t {
  FRAME_TYPE_DCC_PACKET = 1,
  FRAME_TYPE_KEEPALIVE = 2
};

struct __attribute__((packed)) RadioFrame {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t sequence;
  uint8_t flags;
  uint8_t length;
  uint8_t data[MAX_DCC_PACKET_BYTES];
  uint8_t checksum;
};

struct CachedPacket {
  bool valid;
  uint8_t length;
  uint8_t data[MAX_DCC_PACKET_BYTES];
  uint32_t lastSeenMs;
};

struct ReceiverConfig {
  uint16_t magic;
  uint8_t version;
  uint8_t linkDownTicks100ms;
  uint8_t linkDownBrakeTicks100ms;
  uint8_t linkUpAccelTicks100ms;
  uint8_t maxResumeSpeedStep;
  uint8_t holdAfterLinkUp;
  uint8_t checksum;
};

CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};
ReceiverConfig receiverConfig = {};
RadioFrame pendingFrame = {};
volatile bool pendingFrameReady = false;

const uint8_t idlePacket[3] = {0xFF, 0x00, 0xFF};
uint32_t startupMs = 0;
uint32_t lastRadioMs = 0;
uint32_t lastPayloadMs = 0;
uint32_t lastKeepaliveMs = 0;
uint32_t lastConfigMs = 0;
uint32_t lastLedMs = 0;
uint32_t lastDebugFrameMs = 0;
uint32_t lastDebugStatsMs = 0;
uint32_t receivedFrameCount = 0;
uint32_t acceptedFrameCount = 0;
uint32_t payloadFrameCount = 0;
uint32_t keepaliveFrameCount = 0;
uint32_t configFrameCount = 0;
uint32_t invalidFrameCount = 0;
uint32_t staleSequenceCount = 0;
uint8_t lastSequence = 0;
uint8_t roundRobinIndex = 0;
bool lastSequenceValid = false;
bool hasSeenPayload = false;
bool ledState = false;
bool lastLinkAliveState = true;

void debugPrintln(const char *message) {
  if (DEBUG_SERIAL) {
    Serial.println(message);
  }
}

void debugPrintln() {
  if (DEBUG_SERIAL) {
    Serial.println();
  }
}

void debugPrintf(const char *format, ...) {
  if (!DEBUG_SERIAL) {
    return;
  }

  va_list args;
  va_start(args, format);
  Serial.vprintf(format, args);
  va_end(args);
}

void setLed(bool on) {
  ledState = on;
  const uint8_t level = STATUS_LED_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_STATUS_LED, level);
#if defined(LED_BUILTIN)
  if (LED_BUILTIN != PIN_STATUS_LED) {
    digitalWrite(LED_BUILTIN, level);
  }
#endif
}

void configureStatusLeds() {
  pinMode(PIN_STATUS_LED, OUTPUT);
#if defined(LED_BUILTIN)
  if (LED_BUILTIN != PIN_STATUS_LED) {
    pinMode(LED_BUILTIN, OUTPUT);
  }
#endif
}

uint8_t checksumFrame(const RadioFrame &frame) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
  uint8_t x = 0;
  for (uint8_t i = 0; i < sizeof(RadioFrame) - 1; ++i) {
    x ^= bytes[i];
  }
  return x;
}

uint8_t checksumConfig(const ReceiverConfig &config) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&config);
  uint8_t x = 0;
  for (uint8_t i = 0; i < sizeof(ReceiverConfig) - 1; ++i) {
    x ^= bytes[i];
  }
  return x;
}

uint8_t clampU8(uint8_t value, uint8_t low, uint8_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

ReceiverConfig defaultConfig() {
  ReceiverConfig config{};
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  config.linkDownTicks100ms = 20;
  config.linkDownBrakeTicks100ms = 30;
  config.linkUpAccelTicks100ms = 20;
  config.maxResumeSpeedStep = 126;
  config.holdAfterLinkUp = 1;
  config.checksum = checksumConfig(config);
  return config;
}

void sanitizeConfig() {
  receiverConfig.magic = CONFIG_MAGIC;
  receiverConfig.version = CONFIG_VERSION;
  receiverConfig.linkDownTicks100ms = clampU8(receiverConfig.linkDownTicks100ms, 1, 100);
  receiverConfig.maxResumeSpeedStep = clampU8(receiverConfig.maxResumeSpeedStep, 0, 126);
  receiverConfig.holdAfterLinkUp = receiverConfig.holdAfterLinkUp ? 1 : 0;
  receiverConfig.checksum = checksumConfig(receiverConfig);
}

void saveConfig() {
  sanitizeConfig();
  EEPROM.put(EEPROM_CONFIG_ADDRESS, receiverConfig);
  EEPROM.commit();
}

void resetConfig() {
  receiverConfig = defaultConfig();
  saveConfig();
}

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_CONFIG_ADDRESS, receiverConfig);
  if (receiverConfig.magic != CONFIG_MAGIC ||
      receiverConfig.version != CONFIG_VERSION ||
      receiverConfig.checksum != checksumConfig(receiverConfig)) {
    resetConfig();
    return;
  }
  sanitizeConfig();
}

bool isLinkAlive() {
  return (millis() - lastRadioMs) <= (static_cast<uint32_t>(receiverConfig.linkDownTicks100ms) * 100UL);
}

bool isNewerSequence(uint8_t sequence) {
  if (!lastSequenceValid) {
    lastSequence = sequence;
    lastSequenceValid = true;
    return true;
  }
  if (sequence == lastSequence) {
    return false;
  }
  const uint8_t delta = static_cast<uint8_t>(sequence - lastSequence);
  if (delta < 128) {
    lastSequence = sequence;
    return true;
  }
  return false;
}

void setDccBit(rmt_data_t &item, bool one, bool inverted) {
  const uint16_t halfPeriod = one ? DCC_1_HALFPERIOD : DCC_0_HALFPERIOD;
  item.level0 = inverted ? 0 : 1;
  item.duration0 = halfPeriod;
  item.level1 = inverted ? 1 : 0;
  item.duration1 = halfPeriod;
}

void buildDccPacketItems(const uint8_t *data, uint8_t length, rmt_data_t *items, uint8_t &count, bool inverted) {
  count = 0;
  for (uint8_t i = 0; i < DCC_PREAMBLE_BITS && count < RMT_MAX_PACKET_ITEMS; ++i) {
    setDccBit(items[count++], true, inverted);
  }
  setDccBit(items[count++], false, inverted);

  for (uint8_t byteIndex = 0; byteIndex < length && count < RMT_MAX_PACKET_ITEMS; ++byteIndex) {
    for (int8_t bit = 7; bit >= 0 && count < RMT_MAX_PACKET_ITEMS; --bit) {
      setDccBit(items[count++], (data[byteIndex] >> bit) & 0x01, inverted);
    }
    if (count < RMT_MAX_PACKET_ITEMS) {
      setDccBit(items[count++], byteIndex == length - 1, inverted);
    }
  }
}

void initializeDccOutput() {
  pinMode(PIN_DCC_OUT_A, OUTPUT);
  pinMode(PIN_DCC_OUT_B, OUTPUT);
  if (ENABLE_HBRIDGE_EN_PIN) {
    pinMode(PIN_HBRIDGE_EN, OUTPUT);
    digitalWrite(PIN_HBRIDGE_EN, HIGH);
  }

  rmtInit(PIN_DCC_OUT_A, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_RESOLUTION_HZ);
  rmtInit(PIN_DCC_OUT_B, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_RESOLUTION_HZ);
  rmtSetEOT(PIN_DCC_OUT_A, LOW);
  rmtSetEOT(PIN_DCC_OUT_B, HIGH);
}

bool writeDccPacket(const uint8_t *data, uint8_t length) {
  if (length == 0 || length > MAX_DCC_PACKET_BYTES) {
    return false;
  }

  rmt_data_t itemsA[RMT_MAX_PACKET_ITEMS] = {};
  rmt_data_t itemsB[RMT_MAX_PACKET_ITEMS] = {};
  uint8_t countA = 0;
  uint8_t countB = 0;
  buildDccPacketItems(data, length, itemsA, countA, false);
  buildDccPacketItems(data, length, itemsB, countB, true);
  if (countA == 0 || countA != countB) {
    return false;
  }

  if (!rmtWriteAsync(PIN_DCC_OUT_A, itemsA, countA)) {
    return false;
  }
  if (!rmtWriteAsync(PIN_DCC_OUT_B, itemsB, countB)) {
    return false;
  }
  while (!rmtTransmitCompleted(PIN_DCC_OUT_A) || !rmtTransmitCompleted(PIN_DCC_OUT_B)) {
    delay(0);
  }
  return true;
}

uint8_t functionGroupId(uint8_t instruction) {
  if ((instruction & 0xE0) == 0x80) {
    return 1;
  }
  if ((instruction & 0xF0) == 0xB0) {
    return 2;
  }
  if ((instruction & 0xF0) == 0xA0) {
    return 3;
  }
  return 0;
}

bool packetsShareSlot(const RadioFrame &candidate, const CachedPacket &existing) {
  if (candidate.length < 2 || existing.length < 2 || candidate.data[0] != existing.data[0]) {
    return false;
  }
  if (candidate.length == existing.length && memcmp(candidate.data, existing.data, candidate.length) == 0) {
    return true;
  }
  if (candidate.data[0] == 0) {
    return candidate.length == existing.length && candidate.data[1] == existing.data[1];
  }
  if (candidate.data[0] >= 1 && candidate.data[0] <= MAX_LOCOS && candidate.data[1] == 0x3F &&
      existing.length >= 4 && existing.data[1] == 0x3F) {
    return true;
  }
  if (candidate.data[0] >= 1 && candidate.data[0] <= MAX_LOCOS &&
      candidate.length >= 3 && existing.length >= 3 &&
      (candidate.data[1] & 0xC0) == 0x40 && candidate.data[1] != 0x3F &&
      (existing.data[1] & 0xC0) == 0x40 && existing.data[1] != 0x3F) {
    return true;
  }
  const uint8_t group = functionGroupId(candidate.data[1]);
  return group != 0 && group == functionGroupId(existing.data[1]);
}

int8_t findMatchingSlot(const RadioFrame &frame) {
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].valid && packetsShareSlot(frame, packetSlots[i])) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

int8_t findFreeSlot() {
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (!packetSlots[i].valid) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

int8_t findOldestSlot() {
  uint32_t oldest = UINT32_MAX;
  int8_t oldestIndex = 0;
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].lastSeenMs < oldest) {
      oldest = packetSlots[i].lastSeenMs;
      oldestIndex = static_cast<int8_t>(i);
    }
  }
  return oldestIndex;
}

bool applyConfigCv(uint16_t cv, uint8_t value) {
  switch (cv) {
    case CV_LINK_TIMEOUT_TICKS:
      receiverConfig.linkDownTicks100ms = clampU8(value, 1, 100);
      break;
    case CV_LINK_DOWN_BRAKE_TICKS:
      receiverConfig.linkDownBrakeTicks100ms = value;
      break;
    case CV_LINK_UP_ACCEL_TICKS:
      receiverConfig.linkUpAccelTicks100ms = value;
      break;
    case CV_MAX_RESUME_SPEED:
      receiverConfig.maxResumeSpeedStep = clampU8(value, 0, 126);
      break;
    case CV_LINK_UP_BEHAVIOR:
      receiverConfig.holdAfterLinkUp = value ? 1 : 0;
      break;
    case CV_CONFIG_COMMAND:
      if (value == CV_COMMAND_FACTORY_RESET) {
        resetConfig();
      } else if (value == CV_COMMAND_SAVE) {
        saveConfig();
      } else {
        return false;
      }
      return true;
    default:
      return false;
  }
  saveConfig();
  return true;
}

bool handleConfigPomWrite(const RadioFrame &frame) {
  if (frame.length < 5 || frame.data[0] != RECEIVER_CONFIG_ADDRESS) {
    return false;
  }
  if ((frame.data[1] & POM_WRITE_BYTE_MASK) != POM_WRITE_BYTE_OPCODE) {
    return false;
  }
  const uint16_t cv = static_cast<uint16_t>((static_cast<uint16_t>(frame.data[1] & 0x03) << 8) |
                                            frame.data[2]) + 1;
  if (applyConfigCv(cv, frame.data[3])) {
    lastConfigMs = millis();
    ++configFrameCount;
    debugPrintf("Config CV%u=%u applied\n", cv, frame.data[3]);
  }
  return true;
}

void storePacket(const RadioFrame &frame) {
  if (frame.data[0] == RECEIVER_CONFIG_ADDRESS) {
    handleConfigPomWrite(frame);
    return;
  }

  int8_t slot = findMatchingSlot(frame);
  if (slot < 0) {
    slot = findFreeSlot();
  }
  if (slot < 0) {
    slot = findOldestSlot();
  }

  packetSlots[slot].valid = true;
  packetSlots[slot].length = frame.length;
  packetSlots[slot].lastSeenMs = millis();
  for (uint8_t i = 0; i < frame.length && i < MAX_DCC_PACKET_BYTES; ++i) {
    packetSlots[slot].data[i] = frame.data[i];
  }
  hasSeenPayload = true;
}

void expireOldPackets() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].valid && (now - packetSlots[i].lastSeenMs) > PACKET_SLOT_EXPIRE_MS) {
      packetSlots[i].valid = false;
    }
  }
}

bool getNextRawPacket(const uint8_t *&data, uint8_t &length) {
  for (uint8_t n = 0; n < MAX_PACKET_SLOTS; ++n) {
    const uint8_t index = static_cast<uint8_t>((roundRobinIndex + n) % MAX_PACKET_SLOTS);
    if (packetSlots[index].valid) {
      roundRobinIndex = static_cast<uint8_t>((index + 1) % MAX_PACKET_SLOTS);
      data = packetSlots[index].data;
      length = packetSlots[index].length;
      return true;
    }
  }
  return false;
}

void serviceDccOutput() {
  const uint8_t *packetData = nullptr;
  uint8_t packetLength = 0;
  if (!isLinkAlive()) {
    packetData = idlePacket;
    packetLength = sizeof(idlePacket);
  } else if (!getNextRawPacket(packetData, packetLength)) {
    packetData = idlePacket;
    packetLength = sizeof(idlePacket);
  }
  writeDccPacket(packetData, packetLength);
}

void onEspNowReceive(const esp_now_recv_info_t *, const uint8_t *data, int len) {
  ++receivedFrameCount;
  if (len != static_cast<int>(sizeof(RadioFrame))) {
    ++invalidFrameCount;
    return;
  }
  memcpy(&pendingFrame, data, sizeof(RadioFrame));
  pendingFrameReady = true;
}

void processPendingFrame() {
  if (!pendingFrameReady) {
    return;
  }

  RadioFrame frame{};
  noInterrupts();
  memcpy(&frame, &pendingFrame, sizeof(frame));
  pendingFrameReady = false;
  interrupts();

  if (frame.magic != PROTOCOL_MAGIC ||
      frame.version != PROTOCOL_VERSION ||
      frame.length > MAX_DCC_PACKET_BYTES ||
      frame.checksum != checksumFrame(frame)) {
    ++invalidFrameCount;
    return;
  }
  if (!isNewerSequence(frame.sequence)) {
    ++staleSequenceCount;
    return;
  }

  ++acceptedFrameCount;
  lastRadioMs = millis();
  if (frame.type == FRAME_TYPE_KEEPALIVE) {
    lastKeepaliveMs = lastRadioMs;
    ++keepaliveFrameCount;
    return;
  }
  if (frame.type == FRAME_TYPE_DCC_PACKET) {
    lastPayloadMs = lastRadioMs;
    ++payloadFrameCount;
    if (DEBUG_SERIAL && (millis() - lastDebugFrameMs) >= DEBUG_FRAME_INTERVAL_MS) {
      lastDebugFrameMs = millis();
      debugPrintf("Payload addr=%u len=%u seq=%u rx=%lu ok=%lu invalid=%lu stale=%lu\n",
                  frame.data[0],
                  frame.length,
                  frame.sequence,
                  static_cast<unsigned long>(receivedFrameCount),
                  static_cast<unsigned long>(acceptedFrameCount),
                  static_cast<unsigned long>(invalidFrameCount),
                  static_cast<unsigned long>(staleSequenceCount));
    }
    storePacket(frame);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t result = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  debugPrintf("WiFi STA MAC: %s\n", WiFi.macAddress().c_str());
  debugPrintf("ESP-NOW channel set result: %d\n", static_cast<int>(result));

  result = esp_now_init();
  if (result != ESP_OK) {
    debugPrintf("ESP-NOW init failed: %d\n", static_cast<int>(result));
    while (true) {
      setLed(!ledState);
      delay(100);
    }
  }
  debugPrintln("ESP-NOW init OK");
  esp_now_register_recv_cb(onEspNowReceive);
  debugPrintln("ESP-NOW receive callback registered");
}

void updateDebugStats() {
  if (!DEBUG_SERIAL) {
    return;
  }

  const bool linkAlive = isLinkAlive();
  if (linkAlive != lastLinkAliveState) {
    lastLinkAliveState = linkAlive;
    debugPrintf("Link %s at %lu ms\n", linkAlive ? "UP" : "DOWN", static_cast<unsigned long>(millis()));
  }

  const uint32_t now = millis();
  if ((now - lastDebugStatsMs) < DEBUG_STATS_INTERVAL_MS) {
    return;
  }
  lastDebugStatsMs = now;
  debugPrintf("Stats rx=%lu ok=%lu payload=%lu keepalive=%lu config=%lu invalid=%lu stale=%lu link=%s\n",
              static_cast<unsigned long>(receivedFrameCount),
              static_cast<unsigned long>(acceptedFrameCount),
              static_cast<unsigned long>(payloadFrameCount),
              static_cast<unsigned long>(keepaliveFrameCount),
              static_cast<unsigned long>(configFrameCount),
              static_cast<unsigned long>(invalidFrameCount),
              static_cast<unsigned long>(staleSequenceCount),
              linkAlive ? "UP" : "DOWN");
}

void updateStatusLed() {
  const uint32_t now = millis();
  if ((now - startupMs) < STARTUP_LED_MS) {
    setLed(true);
    return;
  }
  if (!isLinkAlive()) {
    setLed(true);
    return;
  }
  if ((now - lastPayloadMs) < PAYLOAD_ACTIVITY_MS || (now - lastConfigMs) < 500) {
    setLed(true);
    return;
  }

  const uint16_t period = hasSeenPayload ? 1000 : 250;
  const uint16_t onTime = hasSeenPayload ? 40 : 80;
  if ((now - lastLedMs) >= period) {
    lastLedMs = now;
  }
  setLed((now - lastLedMs) < onTime);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  debugPrintln();
  debugPrintln("BDCC ESP32-C3 ESP-NOW receiver boot");
  debugPrintf("Build: %s %s\n", __DATE__, __TIME__);
  debugPrintf("DCC output GPIOs: A=%u B=%u, status LED GPIO: %u\n",
              PIN_DCC_OUT_A,
              PIN_DCC_OUT_B,
              PIN_STATUS_LED);
#if defined(LED_BUILTIN)
  debugPrintf("LED_BUILTIN GPIO: %d\n", LED_BUILTIN);
#else
  debugPrintln("LED_BUILTIN not defined by board package");
#endif

  configureStatusLeds();
  setLed(true);

  loadConfig();
  debugPrintf("Config: linkTimeout=%u00ms brake=%u00ms accel=%u00ms maxResume=%u hold=%u\n",
              receiverConfig.linkDownTicks100ms,
              receiverConfig.linkDownBrakeTicks100ms,
              receiverConfig.linkUpAccelTicks100ms,
              receiverConfig.maxResumeSpeedStep,
              receiverConfig.holdAfterLinkUp);
  initializeDccOutput();
  debugPrintln("DCC RMT output initialized");
  setupEspNow();

  startupMs = millis();
  lastRadioMs = startupMs;
  lastKeepaliveMs = startupMs;
  lastLinkAliveState = isLinkAlive();
  debugPrintln("Setup complete");
}

void loop() {
  processPendingFrame();
  expireOldPackets();
  serviceDccOutput();
  updateStatusLed();
  updateDebugStats();
}

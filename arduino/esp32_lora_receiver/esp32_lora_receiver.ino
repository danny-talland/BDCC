#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include "driver/rmt.h"
#include "esp32-hal-matrix.h"
#include "soc/gpio_sig_map.h"
#include <EEPROM.h>
#include <Wire.h>
#include <string.h>

namespace {

constexpr uint8_t PIN_DCC_OUT_A = 3;
constexpr uint8_t PIN_DCC_OUT_B = 4;
constexpr uint8_t PIN_HBRIDGE_EN = 5;
constexpr bool ENABLE_HBRIDGE_EN_PIN = false;

constexpr uint32_t RF_FREQUENCY = 868000000;
constexpr int8_t TX_OUTPUT_POWER = 10;
constexpr uint8_t LORA_BANDWIDTH = 0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 1;
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
constexpr uint16_t LORA_SYMBOL_TIMEOUT = 0;
constexpr bool LORA_FIXED_LENGTH = false;
constexpr bool LORA_IQ_INVERSION = false;

constexpr uint8_t PROTOCOL_VERSION = 2;
constexpr uint16_t PROTOCOL_MAGIC = 0xBDCC;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint8_t MAX_PACKET_SLOTS = 24;
constexpr uint8_t MAX_LOCOS = 20;
constexpr uint8_t RECEIVER_CONFIG_ADDRESS = 99;
constexpr uint8_t RMT_CHANNEL = 0;
constexpr uint8_t RMT_MEM_BLOCKS = 2;
constexpr uint8_t RMT_CLOCK_DIVIDER = 80;
constexpr uint8_t DCC_PREAMBLE_BITS = 14;
constexpr uint8_t RMT_MAX_DATA_ITEMS = 64;
constexpr uint16_t DCC_1_HALFPERIOD = 58;
constexpr uint16_t DCC_0_HALFPERIOD = 100;
constexpr uint32_t PACKET_SLOT_EXPIRE_MS = 10000;
constexpr uint32_t PAYLOAD_ACTIVITY_MS = 200;
constexpr uint32_t KEEPALIVE_ACTIVITY_MS = 200;
constexpr uint32_t DISPLAY_UPDATE_MS = 150;
constexpr uint8_t RADIO_TEXT_BUFFER_SIZE = 40;
constexpr uint16_t EEPROM_SIZE = 64;
constexpr uint16_t EEPROM_CONFIG_ADDRESS = 0;
constexpr uint16_t CONFIG_MAGIC = 0xBDCC;
constexpr uint8_t CONFIG_VERSION = 2;

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

enum DisplayScenario : uint8_t {
  SCENARIO_STARTUP,
  SCENARIO_LINK_DOWN,
  SCENARIO_PAYLOAD,
  SCENARIO_KEEPALIVE,
  SCENARIO_FAILSAFE,
  SCENARIO_CONFIG,
  SCENARIO_NORMAL
};

struct RadioFrame {
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

#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

static RadioEvents_t radioEvents;
portMUX_TYPE dccMux = portMUX_INITIALIZER_UNLOCKED;

CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};
ReceiverConfig receiverConfig = {};
char pendingTextFrame[RADIO_TEXT_BUFFER_SIZE] = {};
volatile bool pendingFrameReady = false;
volatile int16_t lastRssi = 0;
volatile int8_t lastSnr = 0;

rmt_item32_t rmtPreamble[DCC_PREAMBLE_BITS + 2] = {};
rmt_item32_t rmtIdle[28] = {};
rmt_item32_t rmtData[RMT_MAX_DATA_ITEMS] = {};
uint8_t rmtPreambleLen = 0;
uint8_t rmtIdleLen = 0;
volatile uint8_t rmtDataLen = 0;
volatile uint8_t rmtDataRepeat = 0;
volatile bool rmtDataReady = false;

const uint8_t idlePacket[3] = {0xFF, 0x00, 0xFF};
uint32_t startupMs = 0;
uint32_t lastRadioMs = 0;
uint32_t lastPayloadMs = 0;
uint32_t lastKeepaliveMs = 0;
uint32_t lastConfigMs = 0;
uint32_t lastDisplayMs = 0;
uint16_t lastPayloadAddress = 0;
uint8_t lastSequence = 0;
uint8_t roundRobinIndex = 0;
bool lastSequenceValid = false;
bool loraIdle = true;
bool hasSeenPayload = false;

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

void setDccBit1(rmt_item32_t &item) {
  item.level0 = 1;
  item.duration0 = DCC_1_HALFPERIOD;
  item.level1 = 0;
  item.duration1 = DCC_1_HALFPERIOD;
}

void setDccBit0(rmt_item32_t &item) {
  item.level0 = 1;
  item.duration0 = DCC_0_HALFPERIOD;
  item.level1 = 0;
  item.duration1 = DCC_0_HALFPERIOD;
}

void setRmtEot(rmt_item32_t &item) {
  item.val = 0;
}

void IRAM_ATTR onRmtTxEnd(rmt_channel_t channel, void *) {
  if (channel != static_cast<rmt_channel_t>(RMT_CHANNEL)) {
    return;
  }

  if (!rmtDataReady && rmtDataRepeat == 0) {
    rmt_fill_tx_items(static_cast<rmt_channel_t>(RMT_CHANNEL), rmtIdle, rmtIdleLen, rmtPreambleLen - 1);
    return;
  }

  if (rmtDataReady) {
    rmt_fill_tx_items(static_cast<rmt_channel_t>(RMT_CHANNEL), rmtData, rmtDataLen, rmtPreambleLen - 1);
    rmtDataReady = false;
  }
  if (rmtDataRepeat > 0) {
    --rmtDataRepeat;
  }
}

void buildRmtPreambleAndIdle() {
  rmtPreambleLen = DCC_PREAMBLE_BITS + 2;
  for (uint8_t i = 0; i < DCC_PREAMBLE_BITS; ++i) {
    setDccBit1(rmtPreamble[i]);
  }
  setDccBit0(rmtPreamble[DCC_PREAMBLE_BITS]);
  setRmtEot(rmtPreamble[DCC_PREAMBLE_BITS + 1]);

  rmtIdleLen = 0;
  for (uint8_t byteIndex = 0; byteIndex < sizeof(idlePacket); ++byteIndex) {
    for (int8_t bit = 7; bit >= 0; --bit) {
      if ((idlePacket[byteIndex] >> bit) & 0x01) {
        setDccBit1(rmtIdle[rmtIdleLen++]);
      } else {
        setDccBit0(rmtIdle[rmtIdleLen++]);
      }
    }
    if (byteIndex == sizeof(idlePacket) - 1) {
      setDccBit1(rmtIdle[rmtIdleLen++]);
    } else {
      setDccBit0(rmtIdle[rmtIdleLen++]);
    }
  }
  setRmtEot(rmtIdle[rmtIdleLen++]);
}

void initializeDccOutput() {
  pinMode(PIN_DCC_OUT_A, OUTPUT);
  pinMode(PIN_DCC_OUT_B, OUTPUT);
  if (ENABLE_HBRIDGE_EN_PIN) {
    pinMode(PIN_HBRIDGE_EN, OUTPUT);
    digitalWrite(PIN_HBRIDGE_EN, HIGH);
  }

  buildRmtPreambleAndIdle();

  rmt_config_t config;
  memset(&config, 0, sizeof(config));
  config.rmt_mode = RMT_MODE_TX;
  config.channel = static_cast<rmt_channel_t>(RMT_CHANNEL);
  config.gpio_num = static_cast<gpio_num_t>(PIN_DCC_OUT_A);
  config.mem_block_num = RMT_MEM_BLOCKS;
  config.clk_div = RMT_CLOCK_DIVIDER;
  config.tx_config.loop_en = true;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED));

  pinMode(PIN_DCC_OUT_B, OUTPUT);
  pinMatrixOutAttach(PIN_DCC_OUT_B, RMT_SIG_OUT0_IDX + RMT_CHANNEL, true, false);

  rmt_register_tx_end_callback(onRmtTxEnd, nullptr);
  rmt_set_tx_intr_en(static_cast<rmt_channel_t>(RMT_CHANNEL), true);
  rmt_fill_tx_items(static_cast<rmt_channel_t>(RMT_CHANNEL), rmtPreamble, rmtPreambleLen, 0);
  rmt_fill_tx_items(static_cast<rmt_channel_t>(RMT_CHANNEL), rmtIdle, rmtIdleLen, rmtPreambleLen - 1);
  rmt_tx_start(static_cast<rmt_channel_t>(RMT_CHANNEL), true);
}

bool stageDccPacket(const uint8_t *data, uint8_t length) {
  if (length == 0 || length > MAX_DCC_PACKET_BYTES) {
    return false;
  }

  rmt_item32_t scratch[RMT_MAX_DATA_ITEMS] = {};
  uint8_t count = 0;
  for (uint8_t byteIndex = 0; byteIndex < length && count < RMT_MAX_DATA_ITEMS; ++byteIndex) {
    for (int8_t bit = 7; bit >= 0 && count < RMT_MAX_DATA_ITEMS; --bit) {
      if ((data[byteIndex] >> bit) & 0x01) {
        setDccBit1(scratch[count++]);
      } else {
        setDccBit0(scratch[count++]);
      }
    }
    if (count < RMT_MAX_DATA_ITEMS) {
      if (byteIndex == length - 1) {
        setDccBit1(scratch[count++]);
      } else {
        setDccBit0(scratch[count++]);
      }
    }
  }
  if (count >= RMT_MAX_DATA_ITEMS) {
    return false;
  }
  setRmtEot(scratch[count++]);

  portENTER_CRITICAL(&dccMux);
  if (rmtDataReady || rmtDataRepeat > 0) {
    portEXIT_CRITICAL(&dccMux);
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    rmtData[i] = scratch[i];
  }
  rmtDataLen = count;
  rmtDataRepeat = 1;
  rmtDataReady = true;
  portEXIT_CRITICAL(&dccMux);
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
  lastPayloadAddress = frame.data[0];
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
  stageDccPacket(packetData, packetLength);
}

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  const uint16_t copyLength = size < (RADIO_TEXT_BUFFER_SIZE - 1) ? size : (RADIO_TEXT_BUFFER_SIZE - 1);
  memcpy(pendingTextFrame, payload, copyLength);
  pendingTextFrame[copyLength] = '\0';
  pendingFrameReady = true;
  lastRssi = rssi;
  lastSnr = snr;
  Radio.Sleep();
  loraIdle = true;
}

int8_t parseHexNibble(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

bool parseHexBytes(const char *hex, uint8_t *data, uint8_t &length) {
  const size_t hexLength = strlen(hex);
  if (hexLength == 0 || (hexLength % 2) != 0 || hexLength > (MAX_DCC_PACKET_BYTES * 2)) {
    return false;
  }
  length = static_cast<uint8_t>(hexLength / 2);
  for (uint8_t i = 0; i < length; ++i) {
    const int8_t high = parseHexNibble(hex[i * 2]);
    const int8_t low = parseHexNibble(hex[i * 2 + 1]);
    if (high < 0 || low < 0) {
      return false;
    }
    data[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

bool parseTextFrame(const char *text, RadioFrame &frame) {
  frame = RadioFrame{};
  unsigned int version = 0;
  unsigned int sequence = 0;
  char frameType = 0;

  if (sscanf(text, "B%u,%u,%c", &version, &sequence, &frameType) != 3 ||
      version != PROTOCOL_VERSION ||
      sequence > 255) {
    return false;
  }

  frame.version = static_cast<uint8_t>(version);
  frame.sequence = static_cast<uint8_t>(sequence);
  if (frameType == 'K') {
    frame.type = FRAME_TYPE_KEEPALIVE;
    return true;
  }
  if (frameType != 'D') {
    return false;
  }

  const char *payloadStart = strchr(text, 'D');
  if (payloadStart == nullptr || payloadStart[1] != ',') {
    return false;
  }
  payloadStart += 2;

  char *endPtr = nullptr;
  const unsigned long flags = strtoul(payloadStart, &endPtr, 10);
  if (endPtr == payloadStart || *endPtr != ',' || flags > 255) {
    return false;
  }
  frame.type = FRAME_TYPE_DCC_PACKET;
  frame.flags = static_cast<uint8_t>(flags);
  return parseHexBytes(endPtr + 1, frame.data, frame.length);
}

void processPendingFrame() {
  if (!pendingFrameReady) {
    return;
  }

  char text[RADIO_TEXT_BUFFER_SIZE] = {};
  noInterrupts();
  memcpy(text, pendingTextFrame, sizeof(text));
  pendingFrameReady = false;
  interrupts();

  RadioFrame frame{};
  if (!parseTextFrame(text, frame) || !isNewerSequence(frame.sequence)) {
    return;
  }

  lastRadioMs = millis();
  if (frame.type == FRAME_TYPE_KEEPALIVE) {
    lastKeepaliveMs = lastRadioMs;
    return;
  }
  if (frame.type == FRAME_TYPE_DCC_PACKET) {
    lastPayloadMs = lastRadioMs;
    storePacket(frame);
  }
}

void startReceiveIfNeeded() {
  if (!loraIdle) {
    return;
  }
  loraIdle = false;
  Radio.Rx(0);
}

void drawAntenna(int x, int y, bool crossed) {
  display.drawLine(x, y + 24, x + 13, y + 4);
  display.drawCircle(x + 13, y + 4, 3);
  display.drawCircle(x + 13, y + 4, 10);
  display.drawCircle(x + 13, y + 4, 17);
  if (crossed) {
    display.drawLine(x - 5, y, x + 32, y + 32);
    display.drawLine(x + 32, y, x - 5, y + 32);
  }
}

void drawPowerIcon() {
  display.drawCircle(64, 34, 20);
  display.fillRect(60, 8, 9, 25);
}

void drawPayloadAddress() {
  char addressText[6] = {};
  snprintf(addressText, sizeof(addressText), "%u", lastPayloadAddress);
  display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  display.setFont(ArialMT_Plain_24);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, addressText);
}

void drawHeartbeatIcon() {
  display.drawLine(18, 34, 38, 34);
  display.drawLine(38, 34, 45, 20);
  display.drawLine(45, 20, 55, 49);
  display.drawLine(55, 49, 65, 25);
  display.drawLine(65, 25, 75, 34);
  display.drawLine(75, 34, 110, 34);
}

void drawFailsafeIcon() {
  display.drawCircle(64, 34, 24);
  display.drawCircle(64, 34, 20);
  display.fillRect(60, 16, 8, 27);
  display.fillCircle(64, 51, 4);
}

void drawConfigIcon() {
  display.drawCircle(64, 34, 18);
  display.drawCircle(64, 34, 7);
  for (uint8_t i = 0; i < 8; ++i) {
    const float a = i * 0.785398f;
    const int x1 = 64 + static_cast<int>(cosf(a) * 21);
    const int y1 = 34 + static_cast<int>(sinf(a) * 21);
    const int x2 = 64 + static_cast<int>(cosf(a) * 28);
    const int y2 = 34 + static_cast<int>(sinf(a) * 28);
    display.drawLine(x1, y1, x2, y2);
  }
}

void drawNormalIcon() {
  display.drawCircle(64, 34, 8);
  display.drawCircle(64, 34, 18);
  display.drawCircle(64, 34, 28);
  display.fillCircle(64, 34, 4);
}

DisplayScenario currentScenario() {
  const uint32_t now = millis();
  if ((now - startupMs) < 1200) {
    return SCENARIO_STARTUP;
  }
  if (!isLinkAlive()) {
    return hasSeenPayload ? SCENARIO_FAILSAFE : SCENARIO_LINK_DOWN;
  }
  if ((now - lastConfigMs) < 1000) {
    return SCENARIO_CONFIG;
  }
  if ((now - lastPayloadMs) < PAYLOAD_ACTIVITY_MS) {
    return SCENARIO_PAYLOAD;
  }
  if ((now - lastKeepaliveMs) < KEEPALIVE_ACTIVITY_MS) {
    return SCENARIO_KEEPALIVE;
  }
  return SCENARIO_NORMAL;
}

void updateDisplay() {
  const uint32_t now = millis();
  if ((now - lastDisplayMs) < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayMs = now;

  display.clear();
  display.setColor(WHITE);
  switch (currentScenario()) {
    case SCENARIO_STARTUP:
      drawPowerIcon();
      break;
    case SCENARIO_LINK_DOWN:
      drawAntenna(48, 17, true);
      break;
    case SCENARIO_PAYLOAD:
      drawPayloadAddress();
      break;
    case SCENARIO_KEEPALIVE:
      drawHeartbeatIcon();
      break;
    case SCENARIO_FAILSAFE:
      drawFailsafeIcon();
      break;
    case SCENARIO_CONFIG:
      drawConfigIcon();
      break;
    case SCENARIO_NORMAL:
      drawNormalIcon();
      break;
  }
  display.display();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  loadConfig();

  display.init();
  display.clear();
  display.display();

  initializeDccOutput();

  radioEvents.RxDone = onRxDone;
  Radio.Init(&radioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODING_RATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIXED_LENGTH,
                    0, true, 0, 0, LORA_IQ_INVERSION, true);

  startupMs = millis();
  lastRadioMs = startupMs;
  lastKeepaliveMs = startupMs;
}

void loop() {
  startReceiveIfNeeded();
  Radio.IrqProcess();
  processPendingFrame();
  expireOldPackets();
  serviceDccOutput();
  updateDisplay();
}

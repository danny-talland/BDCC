#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

namespace {

constexpr uint8_t PIN_DCC_IN = 2;
constexpr uint8_t PIN_STATUS_LED = 8;
constexpr bool STATUS_LED_ACTIVE_HIGH = true;

constexpr uint8_t ESPNOW_CHANNEL = 6;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint16_t PROTOCOL_MAGIC = 0xBDCC;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint8_t MAX_PACKET_SLOTS = 24;
constexpr uint8_t TX_QUEUE_SIZE = 10;
constexpr uint8_t HALF_BIT_BUFFER_SIZE = 128;
constexpr uint8_t ADDRESS_MIN = 1;
constexpr uint8_t ADDRESS_MAX = 20;
constexpr uint8_t RECEIVER_CONFIG_ADDRESS = 99;

constexpr uint16_t DCC_ONE_MIN_US = 40;
constexpr uint16_t DCC_ONE_MAX_US = 80;
constexpr uint16_t DCC_ZERO_MIN_US = 90;
constexpr uint16_t PACKET_GAP_US = 5000;
constexpr uint32_t STATE_REFRESH_INTERVAL_MS = 2000;
constexpr uint32_t STATE_REFRESH_MIN_SPACING_MS = 50;
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 1000;
constexpr uint32_t DCC_SOURCE_TIMEOUT_MS = 1500;
constexpr uint32_t LED_HEALTH_BLINK_MS = 1000;
constexpr uint32_t LED_TX_FLASH_MS = 40;

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
  bool refreshable;
  uint8_t length;
  uint8_t data[MAX_DCC_PACKET_BYTES];
  uint32_t lastSourceMs;
  uint32_t lastSentMs;
};

enum DecodeState : uint8_t {
  WAIT_PREAMBLE,
  READ_BYTE_BITS,
  READ_DELIMITER
};

const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile uint32_t lastEdgeMicros = 0;
volatile uint16_t halfBitBuffer[HALF_BIT_BUFFER_SIZE] = {};
volatile uint8_t halfBitHead = 0;
volatile uint8_t halfBitTail = 0;
volatile bool halfBitOverflow = false;

DecodeState decodeState = WAIT_PREAMBLE;
uint8_t preambleCount = 0;
uint8_t currentByte = 0;
uint8_t bitIndex = 0;
uint8_t packetBytes[MAX_DCC_PACKET_BYTES] = {};
uint8_t packetLength = 0;
uint8_t sequenceCounter = 0;
uint8_t refreshIndex = 0;
bool pendingHalfBitValid = false;
uint8_t pendingHalfBitValue = 0;
uint32_t lastPacketMicros = 0;
uint32_t lastDccSourceMs = 0;
uint32_t lastKeepaliveMs = 0;
uint32_t lastTxMs = 0;
uint32_t lastLedToggleMs = 0;
bool ledState = false;

CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};
RadioFrame txQueue[TX_QUEUE_SIZE] = {};
uint8_t txQueueHead = 0;
uint8_t txQueueTail = 0;
volatile bool espNowBusy = false;

void setLed(bool on) {
  ledState = on;
  digitalWrite(PIN_STATUS_LED, STATUS_LED_ACTIVE_HIGH ? on : !on);
}

uint8_t checksumBytes(const uint8_t *data, uint8_t length) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < length; ++i) {
    x ^= data[i];
  }
  return x;
}

uint8_t checksumFrame(const RadioFrame &frame) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
  uint8_t x = 0;
  for (uint8_t i = 0; i < sizeof(RadioFrame) - 1; ++i) {
    x ^= bytes[i];
  }
  return x;
}

bool queueFull() {
  return static_cast<uint8_t>((txQueueHead + 1) % TX_QUEUE_SIZE) == txQueueTail;
}

bool queueEmpty() {
  return txQueueHead == txQueueTail;
}

void enqueueFrame(const RadioFrame &frame) {
  if (queueFull()) {
    txQueueTail = static_cast<uint8_t>((txQueueTail + 1) % TX_QUEUE_SIZE);
  }
  txQueue[txQueueHead] = frame;
  txQueueHead = static_cast<uint8_t>((txQueueHead + 1) % TX_QUEUE_SIZE);
}

void makeFrame(uint8_t type, uint8_t flags, const uint8_t *data, uint8_t length, RadioFrame &frame) {
  frame = RadioFrame{};
  frame.magic = PROTOCOL_MAGIC;
  frame.version = PROTOCOL_VERSION;
  frame.type = type;
  frame.sequence = sequenceCounter++;
  frame.flags = flags;
  frame.length = length;
  for (uint8_t i = 0; i < length && i < MAX_DCC_PACKET_BYTES; ++i) {
    frame.data[i] = data[i];
  }
  frame.checksum = checksumFrame(frame);
}

void queueDccPacket(const uint8_t *bytes, uint8_t length, bool highPriority) {
  uint8_t flags = 0x01;
  if (bytes[0] == 0) {
    flags |= 0x02;
  }
  if (highPriority) {
    flags |= 0x04;
  }
  if (bytes[0] == RECEIVER_CONFIG_ADDRESS) {
    flags |= 0x10;
  }

  RadioFrame frame{};
  makeFrame(FRAME_TYPE_DCC_PACKET, flags, bytes, length, frame);
  enqueueFrame(frame);
}

void queueKeepalive() {
  RadioFrame frame{};
  makeFrame(FRAME_TYPE_KEEPALIVE, 0x08, nullptr, 0, frame);
  enqueueFrame(frame);
}

void onEspNowSent(const esp_now_send_info_t *, esp_now_send_status_t) {
  espNowBusy = false;
  lastTxMs = millis();
}

void pumpEspNowTx() {
  if (espNowBusy || queueEmpty()) {
    return;
  }

  RadioFrame frame = txQueue[txQueueTail];
  txQueueTail = static_cast<uint8_t>((txQueueTail + 1) % TX_QUEUE_SIZE);
  if (esp_now_send(broadcastAddress, reinterpret_cast<uint8_t *>(&frame), sizeof(frame)) == ESP_OK) {
    espNowBusy = true;
  }
}

void IRAM_ATTR onDccEdge() {
  const uint32_t now = micros();
  const uint32_t delta = now - lastEdgeMicros;
  lastEdgeMicros = now;
  if (delta > 10000) {
    return;
  }

  const uint8_t nextHead = static_cast<uint8_t>((halfBitHead + 1) % HALF_BIT_BUFFER_SIZE);
  if (nextHead == halfBitTail) {
    halfBitOverflow = true;
    return;
  }

  halfBitBuffer[halfBitHead] = static_cast<uint16_t>(delta);
  halfBitHead = nextHead;
}

bool classifyHalfBit(uint16_t durationMicros, uint8_t &bitValue) {
  if (durationMicros >= DCC_ONE_MIN_US && durationMicros <= DCC_ONE_MAX_US) {
    bitValue = 1;
    return true;
  }
  if (durationMicros >= DCC_ZERO_MIN_US) {
    bitValue = 0;
    return true;
  }
  return false;
}

bool isRelevantPacket(const uint8_t *bytes, uint8_t length) {
  if (length < 3) {
    return false;
  }
  if (bytes[0] == 0) {
    return true;
  }
  if (bytes[0] >= ADDRESS_MIN && bytes[0] <= ADDRESS_MAX) {
    return true;
  }
  return bytes[0] == RECEIVER_CONFIG_ADDRESS;
}

bool is128SpeedPacket(const uint8_t *bytes, uint8_t length) {
  return length >= 4 && bytes[0] >= ADDRESS_MIN && bytes[0] <= ADDRESS_MAX && bytes[1] == 0x3F;
}

bool is28SpeedPacket(const uint8_t *bytes, uint8_t length) {
  if (length < 3 || bytes[0] < ADDRESS_MIN || bytes[0] > ADDRESS_MAX) {
    return false;
  }
  return (bytes[1] & 0xC0) == 0x40 && bytes[1] != 0x3F;
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

bool packetsShareSlot(const uint8_t *candidate,
                      uint8_t candidateLength,
                      const uint8_t *existing,
                      uint8_t existingLength) {
  if (candidateLength < 2 || existingLength < 2 || candidate[0] != existing[0]) {
    return false;
  }
  if (candidate[0] == RECEIVER_CONFIG_ADDRESS) {
    return true;
  }
  if (candidateLength == existingLength && memcmp(candidate, existing, candidateLength) == 0) {
    return true;
  }
  if (candidate[0] == 0) {
    return candidateLength == existingLength && candidate[1] == existing[1];
  }
  if (is128SpeedPacket(candidate, candidateLength) && is128SpeedPacket(existing, existingLength)) {
    return true;
  }
  if (is28SpeedPacket(candidate, candidateLength) && is28SpeedPacket(existing, existingLength)) {
    return true;
  }
  const uint8_t group = functionGroupId(candidate[1]);
  return group != 0 && group == functionGroupId(existing[1]);
}

bool validateDccChecksum(const uint8_t *bytes, uint8_t length) {
  return checksumBytes(bytes, length - 1) == bytes[length - 1];
}

int8_t findSlotForPacket(const uint8_t *bytes, uint8_t length) {
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].valid &&
        packetsShareSlot(bytes, length, packetSlots[i].data, packetSlots[i].length)) {
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
    if (packetSlots[i].lastSentMs < oldest) {
      oldest = packetSlots[i].lastSentMs;
      oldestIndex = static_cast<int8_t>(i);
    }
  }
  return oldestIndex;
}

bool storePacketIfChanged(const uint8_t *bytes, uint8_t length) {
  int8_t slot = findSlotForPacket(bytes, length);
  bool changed = true;
  if (slot < 0) {
    slot = findFreeSlot();
  }
  if (slot < 0) {
    slot = findOldestSlot();
  } else if (packetSlots[slot].valid &&
             packetSlots[slot].length == length &&
             memcmp(packetSlots[slot].data, bytes, length) == 0) {
    changed = false;
  }

  packetSlots[slot].valid = true;
  packetSlots[slot].refreshable = bytes[0] != RECEIVER_CONFIG_ADDRESS;
  packetSlots[slot].length = length;
  packetSlots[slot].lastSourceMs = millis();
  for (uint8_t i = 0; i < length && i < MAX_DCC_PACKET_BYTES; ++i) {
    packetSlots[slot].data[i] = bytes[i];
  }
  return changed;
}

void markPacketSent(const uint8_t *bytes, uint8_t length) {
  const int8_t slot = findSlotForPacket(bytes, length);
  if (slot >= 0) {
    packetSlots[slot].lastSentMs = millis();
  }
}

bool isDccSourceAlive(uint32_t now) {
  return lastDccSourceMs != 0 && (now - lastDccSourceMs) <= DCC_SOURCE_TIMEOUT_MS;
}

void maybeRefreshCachedPacket() {
  static uint32_t lastRefreshMs = 0;
  const uint32_t now = millis();
  if (!isDccSourceAlive(now) || (now - lastRefreshMs) < STATE_REFRESH_MIN_SPACING_MS) {
    return;
  }

  for (uint8_t n = 0; n < MAX_PACKET_SLOTS; ++n) {
    const uint8_t index = static_cast<uint8_t>((refreshIndex + n) % MAX_PACKET_SLOTS);
    if (packetSlots[index].valid &&
        packetSlots[index].refreshable &&
        (now - packetSlots[index].lastSentMs) >= STATE_REFRESH_INTERVAL_MS) {
      queueDccPacket(packetSlots[index].data, packetSlots[index].length, false);
      packetSlots[index].lastSentMs = now;
      refreshIndex = static_cast<uint8_t>((index + 1) % MAX_PACKET_SLOTS);
      lastRefreshMs = now;
      return;
    }
  }
}

void maybeSendKeepalive() {
  const uint32_t now = millis();
  if (!isDccSourceAlive(now)) {
    return;
  }
  if ((now - lastKeepaliveMs) >= KEEPALIVE_INTERVAL_MS) {
    queueKeepalive();
    lastKeepaliveMs = now;
  }
}

void resetDecoder() {
  decodeState = WAIT_PREAMBLE;
  preambleCount = 0;
  currentByte = 0;
  bitIndex = 0;
  packetLength = 0;
  pendingHalfBitValid = false;
}

void finishPacket() {
  if (packetLength >= 3 &&
      validateDccChecksum(packetBytes, packetLength) &&
      isRelevantPacket(packetBytes, packetLength)) {
    lastDccSourceMs = millis();
    if (storePacketIfChanged(packetBytes, packetLength)) {
      queueDccPacket(packetBytes, packetLength, true);
      markPacketSent(packetBytes, packetLength);
    }
  }
  resetDecoder();
}

void processDccBit(uint8_t bitValue) {
  switch (decodeState) {
    case WAIT_PREAMBLE:
      if (bitValue == 1) {
        if (preambleCount < 20) {
          ++preambleCount;
        }
      } else if (preambleCount >= 10) {
        currentByte = 0;
        bitIndex = 0;
        decodeState = READ_BYTE_BITS;
      } else {
        preambleCount = 0;
      }
      break;
    case READ_BYTE_BITS:
      currentByte = static_cast<uint8_t>((currentByte << 1) | (bitValue & 0x01));
      ++bitIndex;
      if (bitIndex == 8) {
        if (packetLength < MAX_DCC_PACKET_BYTES) {
          packetBytes[packetLength++] = currentByte;
          decodeState = READ_DELIMITER;
        } else {
          resetDecoder();
        }
      }
      break;
    case READ_DELIMITER:
      if (bitValue == 0) {
        currentByte = 0;
        bitIndex = 0;
        decodeState = READ_BYTE_BITS;
      } else {
        finishPacket();
      }
      break;
  }
}

void handleHalfBitDuration(uint16_t duration) {
  uint8_t halfBitValue = 0;
  if (!classifyHalfBit(duration, halfBitValue)) {
    resetDecoder();
    return;
  }

  lastPacketMicros = micros();
  if (!pendingHalfBitValid) {
    pendingHalfBitValue = halfBitValue;
    pendingHalfBitValid = true;
    return;
  }
  if (pendingHalfBitValue == halfBitValue) {
    pendingHalfBitValid = false;
    processDccBit(halfBitValue);
    return;
  }

  resetDecoder();
  pendingHalfBitValue = halfBitValue;
  pendingHalfBitValid = true;
}

void processHalfBitStream() {
  if ((micros() - lastPacketMicros) > PACKET_GAP_US) {
    resetDecoder();
  }

  if (halfBitOverflow) {
    noInterrupts();
    halfBitOverflow = false;
    halfBitTail = halfBitHead;
    interrupts();
    resetDecoder();
    return;
  }

  while (true) {
    noInterrupts();
    if (halfBitHead == halfBitTail) {
      interrupts();
      break;
    }
    const uint16_t duration = halfBitBuffer[halfBitTail];
    halfBitTail = static_cast<uint8_t>((halfBitTail + 1) % HALF_BIT_BUFFER_SIZE);
    interrupts();
    handleHalfBitDuration(duration);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) {
      setLed(!ledState);
      delay(100);
    }
  }

  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void updateStatusLed() {
  const uint32_t now = millis();
  if ((now - lastTxMs) < LED_TX_FLASH_MS) {
    setLed(true);
    return;
  }

  if ((now - lastLedToggleMs) >= LED_HEALTH_BLINK_MS) {
    lastLedToggleMs = now;
    setLed(!ledState);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STATUS_LED, OUTPUT);
  setLed(true);

  pinMode(PIN_DCC_IN, INPUT);
  setupEspNow();

  attachInterrupt(digitalPinToInterrupt(PIN_DCC_IN), onDccEdge, CHANGE);
  resetDecoder();
  lastEdgeMicros = micros();
}

void loop() {
  processHalfBitStream();
  maybeRefreshCachedPacket();
  maybeSendKeepalive();
  pumpEspNowTx();
  updateStatusLed();
}

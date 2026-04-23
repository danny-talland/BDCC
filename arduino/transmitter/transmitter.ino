#include <SPI.h>
#include <RF24.h>

namespace {

constexpr uint8_t PIN_DCC_IN = 2;
constexpr uint8_t PIN_STATUS_LED = 4;
constexpr uint8_t PIN_NRF_CE = 9;
constexpr uint8_t PIN_NRF_CSN = 10;

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint8_t MAX_PACKET_SLOTS = 24;
constexpr uint8_t ADDRESS_MIN = 1;
constexpr uint8_t ADDRESS_MAX = 20;

constexpr uint16_t DCC_ONE_MIN_US = 40;
constexpr uint16_t DCC_ONE_MAX_US = 80;
constexpr uint16_t DCC_ZERO_MIN_US = 90;
constexpr uint16_t PACKET_GAP_US = 5000;
constexpr uint32_t RESEND_INTERVAL_MS = 30;
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 250;

struct RadioPacket {
  uint8_t version;
  uint8_t flags;
  uint8_t length;
  uint8_t sequence;
  uint8_t data[MAX_DCC_PACKET_BYTES];
};

struct CachedPacket {
  bool valid;
  uint8_t key;
  uint8_t length;
  uint8_t data[MAX_DCC_PACKET_BYTES];
  uint32_t lastSentMs;
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kRadioAddress[6] = "DCC01";

volatile uint32_t lastEdgeMicros = 0;
volatile uint16_t halfBitMicros = 0;
volatile bool halfBitReady = false;

uint8_t sequenceCounter = 0;

enum DecodeState : uint8_t {
  WAIT_PREAMBLE,
  READ_BYTE_BITS,
  READ_DELIMITER
};

DecodeState decodeState = WAIT_PREAMBLE;
uint8_t preambleCount = 0;
uint8_t currentByte = 0;
uint8_t bitIndex = 0;
uint8_t packetBytes[MAX_DCC_PACKET_BYTES] = {};
uint8_t packetLength = 0;
uint32_t lastPacketMicros = 0;
uint32_t lastKeepaliveMs = 0;
uint8_t resendIndex = 0;
CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};

void onDccEdge() {
  const uint32_t now = micros();
  const uint32_t delta = now - lastEdgeMicros;
  lastEdgeMicros = now;

  if (delta > 10000) {
    return;
  }

  halfBitMicros = static_cast<uint16_t>(delta);
  halfBitReady = true;
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

bool isRelevantPacket(const uint8_t *bytes, uint8_t length, uint8_t &flags) {
  flags = 0x01;
  if (length < 3) {
    return false;
  }

  const uint8_t address = bytes[0];
  if (address == 0) {
    flags |= 0x02;
    return true;
  }

  if (address >= ADDRESS_MIN && address <= ADDRESS_MAX) {
    return true;
  }

  return false;
}

uint8_t makePacketKey(const uint8_t *bytes, uint8_t length) {
  if (length < 2) {
    return 0xFF;
  }
  return static_cast<uint8_t>((bytes[0] << 3) ^ bytes[1]);
}

bool validateChecksum(const uint8_t *bytes, uint8_t length) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < length - 1; ++i) {
    x ^= bytes[i];
  }
  return x == bytes[length - 1];
}

void writeRadioPacket(const RadioPacket &payload) {
  digitalWrite(PIN_STATUS_LED, HIGH);
  radio.write(&payload, sizeof(payload));
  digitalWrite(PIN_STATUS_LED, LOW);
}

void sendKeepalive() {
  RadioPacket payload{};
  payload.version = PROTOCOL_VERSION;
  payload.flags = 0x08;
  payload.length = 0;
  payload.sequence = sequenceCounter++;
  writeRadioPacket(payload);
}

int8_t findSlotByKey(uint8_t key) {
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].valid && packetSlots[i].key == key) {
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

void storePacketForResend(const uint8_t *bytes, uint8_t length) {
  const uint8_t key = makePacketKey(bytes, length);
  if (key == 0xFF) {
    return;
  }

  int8_t slot = findSlotByKey(key);
  if (slot < 0) {
    slot = findFreeSlot();
  }
  if (slot < 0) {
    slot = findOldestSlot();
  }

  packetSlots[slot].valid = true;
  packetSlots[slot].key = key;
  packetSlots[slot].length = length;
  packetSlots[slot].lastSentMs = millis();
  for (uint8_t i = 0; i < length && i < MAX_DCC_PACKET_BYTES; ++i) {
    packetSlots[slot].data[i] = bytes[i];
  }
}

void sendRadioPacket(const uint8_t *bytes, uint8_t length, bool highPriority) {
  uint8_t flags = 0;
  if (!isRelevantPacket(bytes, length, flags)) {
    return;
  }

  if (highPriority) {
    flags |= 0x04;
  }

  RadioPacket payload{};
  payload.version = PROTOCOL_VERSION;
  payload.flags = flags;
  payload.length = length;
  payload.sequence = sequenceCounter++;
  for (uint8_t i = 0; i < length && i < MAX_DCC_PACKET_BYTES; ++i) {
    payload.data[i] = bytes[i];
  }

  writeRadioPacket(payload);
}

void maybeResendCachedPacket() {
  static uint32_t lastResendMs = 0;
  const uint32_t now = millis();
  if ((now - lastResendMs) < RESEND_INTERVAL_MS) {
    return;
  }

  for (uint8_t n = 0; n < MAX_PACKET_SLOTS; ++n) {
    const uint8_t index = (resendIndex + n) % MAX_PACKET_SLOTS;
    if (packetSlots[index].valid) {
      sendRadioPacket(packetSlots[index].data, packetSlots[index].length, false);
      packetSlots[index].lastSentMs = now;
      resendIndex = static_cast<uint8_t>((index + 1) % MAX_PACKET_SLOTS);
      lastResendMs = now;
      return;
    }
  }
}

void maybeSendKeepalive() {
  const uint32_t now = millis();
  if ((now - lastKeepaliveMs) >= KEEPALIVE_INTERVAL_MS) {
    sendKeepalive();
    lastKeepaliveMs = now;
  }
}

void resetPacketDecoder() {
  decodeState = WAIT_PREAMBLE;
  preambleCount = 0;
  currentByte = 0;
  bitIndex = 0;
  packetLength = 0;
}

void finishPacket() {
  if (packetLength >= 3 && validateChecksum(packetBytes, packetLength)) {
    storePacketForResend(packetBytes, packetLength);
    sendRadioPacket(packetBytes, packetLength, true);
  }
  resetPacketDecoder();
}

void processDccBit(uint8_t bitValue) {
  switch (decodeState) {
    case WAIT_PREAMBLE:
      if (bitValue == 1) {
        if (preambleCount < 20) {
          ++preambleCount;
        }
      } else {
        if (preambleCount >= 10) {
          currentByte = 0;
          bitIndex = 0;
          decodeState = READ_BYTE_BITS;
        } else {
          preambleCount = 0;
        }
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
          resetPacketDecoder();
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

void processHalfBitStream() {
  if (!halfBitReady) {
    if ((micros() - lastPacketMicros) > PACKET_GAP_US) {
      resetPacketDecoder();
    }
    return;
  }

  noInterrupts();
  const uint16_t duration = halfBitMicros;
  halfBitReady = false;
  interrupts();

  uint8_t bitValue = 0;
  if (!classifyHalfBit(duration, bitValue)) {
    resetPacketDecoder();
    return;
  }

  lastPacketMicros = micros();

  processDccBit(bitValue);
}

}  // namespace

void setup() {
  pinMode(PIN_DCC_IN, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openWritingPipe(kRadioAddress);
  radio.stopListening();

  attachInterrupt(digitalPinToInterrupt(PIN_DCC_IN), onDccEdge, CHANGE);
  resetPacketDecoder();
  lastEdgeMicros = micros();
}

void loop() {
  processHalfBitStream();
  maybeResendCachedPacket();
  maybeSendKeepalive();
}

#include <SPI.h>
#include <RF24.h>

namespace {

constexpr uint8_t PIN_LINK_LED = 3;
constexpr uint8_t PIN_DCC_OUT_A = 5;
constexpr uint8_t PIN_DCC_OUT_B = 6;
constexpr uint8_t PIN_HBRIDGE_EN = 8;
constexpr uint8_t PIN_NRF_CE = 9;
constexpr uint8_t PIN_NRF_CSN = 10;

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint8_t MAX_PACKET_SLOTS = 24;
constexpr uint16_t HALF_ONE_US = 58;
constexpr uint16_t HALF_ZERO_US = 100;
constexpr uint32_t RADIO_TIMEOUT_MS = 400;

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
  uint32_t lastSeenMs;
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kRadioAddress[6] = "DCC01";

CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};
uint32_t lastRadioMs = 0;
bool outputPolarity = false;
uint8_t roundRobinIndex = 0;

const uint8_t idlePacket[3] = {0xFF, 0x00, 0xFF};

uint8_t makePacketKey(const RadioPacket &packet) {
  if (packet.length < 2) {
    return 0xFF;
  }
  return static_cast<uint8_t>((packet.data[0] << 3) ^ packet.data[1]);
}

void setBridgePolarity(bool polarity) {
  outputPolarity = polarity;
  digitalWrite(PIN_DCC_OUT_A, polarity ? HIGH : LOW);
  digitalWrite(PIN_DCC_OUT_B, polarity ? LOW : HIGH);
}

void sendHalfBit(uint16_t durationUs) {
  setBridgePolarity(!outputPolarity);
  delayMicroseconds(durationUs);
}

void sendDccBit(bool oneBit) {
  const uint16_t duration = oneBit ? HALF_ONE_US : HALF_ZERO_US;
  sendHalfBit(duration);
  sendHalfBit(duration);
}

void sendDccByte(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; --bit) {
    sendDccBit((value >> bit) & 0x01);
  }
}

void sendDccPacket(const uint8_t *data, uint8_t length) {
  for (uint8_t i = 0; i < 14; ++i) {
    sendDccBit(true);
  }
  sendDccBit(false);

  for (uint8_t i = 0; i < length; ++i) {
    sendDccByte(data[i]);
    sendDccBit(i == (length - 1));
  }
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
    if (packetSlots[i].lastSeenMs < oldest) {
      oldest = packetSlots[i].lastSeenMs;
      oldestIndex = static_cast<int8_t>(i);
    }
  }
  return oldestIndex;
}

void storePacket(const RadioPacket &packet) {
  const uint8_t key = makePacketKey(packet);
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
  packetSlots[slot].length = packet.length;
  packetSlots[slot].lastSeenMs = millis();
  for (uint8_t i = 0; i < packet.length && i < MAX_DCC_PACKET_BYTES; ++i) {
    packetSlots[slot].data[i] = packet.data[i];
  }
}

void pollRadio() {
  while (radio.available()) {
    RadioPacket payload{};
    radio.read(&payload, sizeof(payload));
    if (payload.version != PROTOCOL_VERSION) {
      continue;
    }
    if ((payload.flags & 0x01) == 0 || payload.length < 2 || payload.length > MAX_DCC_PACKET_BYTES) {
      continue;
    }

    storePacket(payload);
    lastRadioMs = millis();
    digitalWrite(PIN_LINK_LED, HIGH);
  }

  if ((millis() - lastRadioMs) > RADIO_TIMEOUT_MS) {
    digitalWrite(PIN_LINK_LED, LOW);
  }
}

bool getNextPacket(const uint8_t *&data, uint8_t &length) {
  for (uint8_t n = 0; n < MAX_PACKET_SLOTS; ++n) {
    const uint8_t index = (roundRobinIndex + n) % MAX_PACKET_SLOTS;
    if (packetSlots[index].valid) {
      roundRobinIndex = static_cast<uint8_t>((index + 1) % MAX_PACKET_SLOTS);
      data = packetSlots[index].data;
      length = packetSlots[index].length;
      return true;
    }
  }

  data = idlePacket;
  length = sizeof(idlePacket);
  return false;
}

}  // namespace

void setup() {
  pinMode(PIN_LINK_LED, OUTPUT);
  pinMode(PIN_DCC_OUT_A, OUTPUT);
  pinMode(PIN_DCC_OUT_B, OUTPUT);
  pinMode(PIN_HBRIDGE_EN, OUTPUT);

  digitalWrite(PIN_LINK_LED, LOW);
  digitalWrite(PIN_HBRIDGE_EN, HIGH);
  setBridgePolarity(false);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openReadingPipe(1, kRadioAddress);
  radio.startListening();

  lastRadioMs = millis();
}

void loop() {
  pollRadio();

  const uint8_t *packetData = nullptr;
  uint8_t packetLength = 0;
  getNextPacket(packetData, packetLength);
  sendDccPacket(packetData, packetLength);
}

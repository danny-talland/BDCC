#include <SPI.h>
#include <RF24.h>
#include <avr/interrupt.h>

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
constexpr uint8_t MAX_LOCOS = 20;
constexpr uint16_t HALF_ONE_US = 58;
constexpr uint16_t HALF_ZERO_US = 100;
constexpr uint16_t TIMER1_PRESCALER = 8;
constexpr uint16_t HALF_ONE_TICKS = HALF_ONE_US * 2;
constexpr uint16_t HALF_ZERO_TICKS = HALF_ZERO_US * 2;
constexpr uint32_t SIGNAL_LOSS_TIMEOUT_MS = 2000;
constexpr uint32_t RAMP_STEP_INTERVAL_MS = 80;
constexpr uint8_t RAMP_STEP_DELTA = 1;
constexpr uint32_t PACKET_SLOT_EXPIRE_MS = 4000;
constexpr uint8_t MAX_DCC_BITS = 80;

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

enum SpeedMode : uint8_t {
  SPEED_MODE_UNKNOWN = 0,
  SPEED_MODE_28 = 1,
  SPEED_MODE_128 = 2
};

struct LocoState {
  bool seen;
  bool directionForward;
  SpeedMode speedMode;
  uint8_t commandedStep;
  uint8_t outputStep;
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kRadioAddress[6] = "DCC01";

CachedPacket packetSlots[MAX_PACKET_SLOTS] = {};
LocoState locoStates[MAX_LOCOS + 1] = {};
uint32_t lastRadioMs = 0;
uint32_t lastRampMs = 0;
uint8_t roundRobinIndex = 0;
uint8_t locoRoundRobinIndex = 1;
bool preferSpeedPacket = true;

const uint8_t idlePacket[3] = {0xFF, 0x00, 0xFF};
uint8_t dccBitBuffers[2][MAX_DCC_BITS] = {};
volatile uint8_t activeBufferIndex = 0;
volatile uint8_t activeBitCount = 0;
volatile uint8_t activeBitIndex = 0;
volatile uint8_t activeHalfPhase = 0;
volatile uint8_t stagedBufferIndex = 1;
volatile uint8_t stagedBitCount = 0;
volatile bool stagedReady = false;
volatile bool outputPolarity = false;

inline void setBridgePolarityFast(bool polarity) {
  outputPolarity = polarity;
  if (polarity) {
    PORTD |= _BV(PD5);
    PORTD &= static_cast<uint8_t>(~_BV(PD6));
  } else {
    PORTD &= static_cast<uint8_t>(~_BV(PD5));
    PORTD |= _BV(PD6);
  }
}

ISR(TIMER1_COMPA_vect) {
  setBridgePolarityFast(!outputPolarity);

  const uint8_t currentBit = dccBitBuffers[activeBufferIndex][activeBitIndex];
  OCR1A = currentBit ? HALF_ONE_TICKS : HALF_ZERO_TICKS;

  if (activeHalfPhase == 0) {
    activeHalfPhase = 1;
    return;
  }

  activeHalfPhase = 0;
  ++activeBitIndex;
  if (activeBitIndex >= activeBitCount) {
    if (stagedReady) {
      activeBufferIndex = stagedBufferIndex;
      activeBitCount = stagedBitCount;
      stagedReady = false;
    }
    activeBitIndex = 0;
  }
}

uint8_t makePacketKey(const RadioPacket &packet) {
  if (packet.length < 2) {
    return 0xFF;
  }
  return static_cast<uint8_t>((packet.data[0] << 3) ^ packet.data[1]);
}

uint8_t buildDccBitSequence(const uint8_t *data, uint8_t length, uint8_t *bitBuffer) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < 14 && count < MAX_DCC_BITS; ++i) {
    bitBuffer[count++] = 1;
  }
  if (count < MAX_DCC_BITS) {
    bitBuffer[count++] = 0;
  }

  for (uint8_t i = 0; i < length && count < MAX_DCC_BITS; ++i) {
    for (int8_t bit = 7; bit >= 0 && count < MAX_DCC_BITS; --bit) {
      bitBuffer[count++] = static_cast<uint8_t>((data[i] >> bit) & 0x01);
    }
    if (count < MAX_DCC_BITS) {
      bitBuffer[count++] = static_cast<uint8_t>(i == (length - 1));
    }
  }

  return count;
}

void initializeDccOutput() {
  uint8_t initialBitCount = buildDccBitSequence(idlePacket, sizeof(idlePacket), dccBitBuffers[0]);

  noInterrupts();
  activeBufferIndex = 0;
  activeBitCount = initialBitCount;
  activeBitIndex = 0;
  activeHalfPhase = 0;
  stagedReady = false;
  stagedBufferIndex = 1;
  stagedBitCount = 0;

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = HALF_ONE_TICKS;
  TCCR1B |= _BV(WGM12);
  TCCR1B |= _BV(CS11);
  TIMSK1 |= _BV(OCIE1A);
  interrupts();
}

bool stageDccPacket(const uint8_t *data, uint8_t length) {
  if (length == 0 || length > MAX_DCC_PACKET_BYTES) {
    return false;
  }

  uint8_t bitScratch[MAX_DCC_BITS] = {};
  const uint8_t bitCount = buildDccBitSequence(data, length, bitScratch);

  noInterrupts();
  if (stagedReady) {
    interrupts();
    return false;
  }

  const uint8_t targetBuffer = static_cast<uint8_t>(activeBufferIndex ^ 0x01);
  for (uint8_t i = 0; i < bitCount; ++i) {
    dccBitBuffers[targetBuffer][i] = bitScratch[i];
  }
  stagedBufferIndex = targetBuffer;
  stagedBitCount = bitCount;
  stagedReady = true;
  interrupts();
  return true;
}

bool isLinkAlive() {
  return (millis() - lastRadioMs) <= SIGNAL_LOSS_TIMEOUT_MS;
}

bool isShortAddressPacket(const RadioPacket &packet, uint8_t &address) {
  if (packet.length < 3) {
    return false;
  }
  address = packet.data[0];
  return address >= 1 && address <= MAX_LOCOS;
}

bool decode128SpeedPacket(const RadioPacket &packet, LocoState &state) {
  if (packet.length < 4 || packet.data[1] != 0x3F) {
    return false;
  }

  const uint8_t raw = packet.data[2];
  state.directionForward = (raw & 0x80) != 0;
  state.speedMode = SPEED_MODE_128;
  state.commandedStep = (raw & 0x7F) <= 1 ? 0 : static_cast<uint8_t>((raw & 0x7F) - 1);
  return true;
}

bool decode28SpeedPacket(const RadioPacket &packet, LocoState &state) {
  if (packet.length < 3) {
    return false;
  }

  const uint8_t instruction = packet.data[1];
  if ((instruction & 0xC0) != 0x40 || instruction == 0x3F) {
    return false;
  }

  const uint8_t speedCode = static_cast<uint8_t>(((instruction & 0x0F) << 1) | ((instruction >> 4) & 0x01));
  state.directionForward = (instruction & 0x20) != 0;
  state.speedMode = SPEED_MODE_28;
  if (speedCode <= 1) {
    state.commandedStep = 0;
  } else {
    const uint8_t mapped = static_cast<uint8_t>(speedCode - 1);
    state.commandedStep = mapped > 28 ? 28 : mapped;
  }
  return true;
}

bool packetUpdatesSpeed(const RadioPacket &packet, uint8_t &address) {
  if (!isShortAddressPacket(packet, address)) {
    return false;
  }

  LocoState &state = locoStates[address];
  if (decode128SpeedPacket(packet, state) || decode28SpeedPacket(packet, state)) {
    state.seen = true;
    return true;
  }

  return false;
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
  uint8_t address = 0;
  if (packetUpdatesSpeed(packet, address)) {
    return;
  }

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

void expireOldPackets() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < MAX_PACKET_SLOTS; ++i) {
    if (packetSlots[i].valid && (now - packetSlots[i].lastSeenMs) > PACKET_SLOT_EXPIRE_MS) {
      packetSlots[i].valid = false;
    }
  }
}

void pollRadio() {
  while (radio.available()) {
    RadioPacket payload{};
    radio.read(&payload, sizeof(payload));
    if (payload.version != PROTOCOL_VERSION) {
      continue;
    }
    if (payload.flags & 0x08) {
      lastRadioMs = millis();
      digitalWrite(PIN_LINK_LED, HIGH);
      continue;
    }
    if ((payload.flags & 0x01) == 0 || payload.length < 2 || payload.length > MAX_DCC_PACKET_BYTES) {
      continue;
    }

    storePacket(payload);
    lastRadioMs = millis();
    digitalWrite(PIN_LINK_LED, HIGH);
  }

  if (!isLinkAlive()) {
    digitalWrite(PIN_LINK_LED, LOW);
  }
}

uint8_t encode128SpeedByte(const LocoState &state) {
  if (state.outputStep == 0) {
    return static_cast<uint8_t>(state.directionForward ? 0x80 : 0x00);
  }
  return static_cast<uint8_t>((state.directionForward ? 0x80 : 0x00) | (state.outputStep + 1));
}

uint8_t encode28InstructionByte(const LocoState &state) {
  uint8_t speedCode = 0;
  if (state.outputStep != 0) {
    speedCode = static_cast<uint8_t>(state.outputStep + 1);
    if (speedCode > 29) {
      speedCode = 29;
    }
  }

  const uint8_t lower = static_cast<uint8_t>((speedCode >> 1) & 0x0F);
  const uint8_t cBit = static_cast<uint8_t>((speedCode & 0x01) << 4);
  const uint8_t dirBit = state.directionForward ? 0x20 : 0x00;
  return static_cast<uint8_t>(0x40 | dirBit | cBit | lower);
}

uint8_t appendChecksum(uint8_t *packet, uint8_t payloadLength) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < payloadLength; ++i) {
    x ^= packet[i];
  }
  packet[payloadLength] = x;
  return static_cast<uint8_t>(payloadLength + 1);
}

bool buildSyntheticSpeedPacket(uint8_t *packet, uint8_t &length) {
  for (uint8_t n = 0; n < MAX_LOCOS; ++n) {
    const uint8_t address = static_cast<uint8_t>(((locoRoundRobinIndex - 1 + n) % MAX_LOCOS) + 1);
    const LocoState &state = locoStates[address];
    if (!state.seen || state.speedMode == SPEED_MODE_UNKNOWN) {
      continue;
    }

    packet[0] = address;
    if (state.speedMode == SPEED_MODE_128) {
      packet[1] = 0x3F;
      packet[2] = encode128SpeedByte(state);
      length = appendChecksum(packet, 3);
    } else {
      packet[1] = encode28InstructionByte(state);
      length = appendChecksum(packet, 2);
    }

    locoRoundRobinIndex = static_cast<uint8_t>((address % MAX_LOCOS) + 1);
    return true;
  }

  return false;
}

bool getNextRawPacket(const uint8_t *&data, uint8_t &length) {
  for (uint8_t n = 0; n < MAX_PACKET_SLOTS; ++n) {
    const uint8_t index = (roundRobinIndex + n) % MAX_PACKET_SLOTS;
    if (packetSlots[index].valid) {
      roundRobinIndex = static_cast<uint8_t>((index + 1) % MAX_PACKET_SLOTS);
      data = packetSlots[index].data;
      length = packetSlots[index].length;
      return true;
    }
  }

  return false;
}

void updateRampTargets() {
  const uint32_t now = millis();
  if ((now - lastRampMs) < RAMP_STEP_INTERVAL_MS) {
    return;
  }
  lastRampMs = now;

  const bool linkAlive = isLinkAlive();
  for (uint8_t address = 1; address <= MAX_LOCOS; ++address) {
    LocoState &state = locoStates[address];
    if (!state.seen) {
      continue;
    }

    const uint8_t target = linkAlive ? state.commandedStep : 0;
    if (state.outputStep < target) {
      const uint8_t delta = static_cast<uint8_t>(target - state.outputStep);
      state.outputStep = static_cast<uint8_t>(state.outputStep + (delta > RAMP_STEP_DELTA ? RAMP_STEP_DELTA : delta));
    } else if (state.outputStep > target) {
      const uint8_t delta = static_cast<uint8_t>(state.outputStep - target);
      state.outputStep = static_cast<uint8_t>(state.outputStep - (delta > RAMP_STEP_DELTA ? RAMP_STEP_DELTA : delta));
    }
  }
}

bool getNextPacket(const uint8_t *&data, uint8_t &length, uint8_t *scratch) {
  preferSpeedPacket = !preferSpeedPacket;
  if (preferSpeedPacket) {
    if (buildSyntheticSpeedPacket(scratch, length)) {
      data = scratch;
      return true;
    }
    if (getNextRawPacket(data, length)) {
      return true;
    }
  } else {
    if (getNextRawPacket(data, length)) {
      return true;
    }
    if (buildSyntheticSpeedPacket(scratch, length)) {
      data = scratch;
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
  setBridgePolarityFast(false);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openReadingPipe(1, kRadioAddress);
  radio.startListening();

  lastRadioMs = millis();
  lastRampMs = millis();
  initializeDccOutput();
}

void loop() {
  pollRadio();
  expireOldPackets();
  updateRampTargets();

  const uint8_t *packetData = nullptr;
  uint8_t packetLength = 0;
  uint8_t scratchPacket[MAX_DCC_PACKET_BYTES] = {};
  getNextPacket(packetData, packetLength, scratchPacket);
  stageDccPacket(packetData, packetLength);
}

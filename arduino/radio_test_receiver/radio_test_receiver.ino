#include <SPI.h>
#include <RF24.h>

namespace {

constexpr uint8_t PIN_STATUS_LED = 3;
constexpr uint8_t PIN_NRF_CE = 9;
constexpr uint8_t PIN_NRF_CSN = 10;

constexpr bool ENABLE_SERIAL_DEBUG = true;
constexpr bool ENABLE_PACKET_DEBUG = false;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint32_t STARTUP_LED_SOLID_MS = 4000;
constexpr uint32_t LINK_TIMEOUT_MS = 2000;
constexpr uint32_t PAYLOAD_FLASH_MS = 80;
constexpr uint32_t NO_LINK_BLINK_PERIOD_MS = 1000;
constexpr uint32_t SERIAL_REPORT_INTERVAL_MS = 1000;
constexpr uint8_t LED_BRIGHTNESS_OFF = 0;
constexpr uint8_t LED_BRIGHTNESS_DIM = 1;
constexpr uint8_t LED_BRIGHTNESS_FULL = 255;

struct RadioPacket {
  uint8_t version;
  uint8_t flags;
  uint8_t length;
  uint8_t sequence;
  uint8_t data[MAX_DCC_PACKET_BYTES];
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kRadioAddress[6] = "DCC01";

uint32_t startupMs = 0;
uint32_t lastRadioMs = 0;
uint32_t lastPayloadMs = 0;
uint32_t receivedPackets = 0;
uint32_t droppedPackets = 0;
uint32_t lastSerialReportMs = 0;
uint8_t lastSequence = 0;
bool lastSequenceValid = false;
bool hasReceivedRadio = false;

bool isLinkAlive() {
  if (!hasReceivedRadio) {
    return false;
  }
  return (millis() - lastRadioMs) <= LINK_TIMEOUT_MS;
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

bool validateChecksum(const RadioPacket &packet) {
  if (packet.length < 3) {
    return false;
  }

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < packet.length - 1; ++i) {
    checksum ^= packet.data[i];
  }
  return checksum == packet.data[packet.length - 1];
}

void updateStatusLed() {
  const uint32_t now = millis();

  if ((now - startupMs) < STARTUP_LED_SOLID_MS) {
    analogWrite(PIN_STATUS_LED, LED_BRIGHTNESS_FULL);
    return;
  }

  if (!isLinkAlive()) {
    const bool blinkOn = (now % NO_LINK_BLINK_PERIOD_MS) < (NO_LINK_BLINK_PERIOD_MS / 2);
    analogWrite(PIN_STATUS_LED, blinkOn ? LED_BRIGHTNESS_FULL : LED_BRIGHTNESS_OFF);
    return;
  }

  if (lastPayloadMs != 0 && (now - lastPayloadMs) < PAYLOAD_FLASH_MS) {
    analogWrite(PIN_STATUS_LED, LED_BRIGHTNESS_OFF);
    return;
  }

  analogWrite(PIN_STATUS_LED, LED_BRIGHTNESS_DIM);
}

void pollRadio() {
  while (radio.available()) {
    RadioPacket payload{};
    radio.read(&payload, sizeof(payload));

    if (payload.version != PROTOCOL_VERSION) {
      ++droppedPackets;
      if (ENABLE_PACKET_DEBUG) {
        Serial.println(F("drop version"));
      }
      continue;
    }

    if (!isNewerSequence(payload.sequence)) {
      ++droppedPackets;
      if (ENABLE_PACKET_DEBUG) {
        Serial.println(F("drop sequence"));
      }
      continue;
    }

    hasReceivedRadio = true;
    lastRadioMs = millis();

    if (payload.flags & 0x08) {
      ++receivedPackets;
      if (ENABLE_PACKET_DEBUG) {
        Serial.print(F("rx keepalive seq="));
        Serial.println(payload.sequence);
      }
      continue;
    }

    if ((payload.flags & 0x01) == 0 ||
        payload.length < 2 ||
        payload.length > MAX_DCC_PACKET_BYTES ||
        !validateChecksum(payload)) {
      ++droppedPackets;
      if (ENABLE_PACKET_DEBUG) {
        Serial.print(F("drop payload seq="));
        Serial.println(payload.sequence);
      }
      continue;
    }

    ++receivedPackets;
    lastPayloadMs = lastRadioMs;
    if (ENABLE_PACKET_DEBUG) {
      Serial.print(F("rx payload seq="));
      Serial.print(payload.sequence);
      Serial.print(F(" flags=0x"));
      Serial.print(payload.flags, HEX);
      Serial.print(F(" len="));
      Serial.println(payload.length);
    }
  }
}

void reportStatus() {
  if (!ENABLE_SERIAL_DEBUG) {
    return;
  }

  const uint32_t now = millis();
  if ((now - lastSerialReportMs) < SERIAL_REPORT_INTERVAL_MS) {
    return;
  }
  lastSerialReportMs = now;

  Serial.print(F("link="));
  Serial.print(isLinkAlive() ? F("up") : F("down"));
  Serial.print(F(" rx="));
  Serial.print(receivedPackets);
  Serial.print(F(" dropped="));
  Serial.print(droppedPackets);
  Serial.print(F(" seq="));
  Serial.println(lastSequence);
}

}  // namespace

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  analogWrite(PIN_STATUS_LED, LED_BRIGHTNESS_FULL);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(115200);
    Serial.println(F("BDCC radio test receiver"));
  }

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openReadingPipe(1, kRadioAddress);
  radio.startListening();

  startupMs = millis();
}

void loop() {
  pollRadio();
  updateStatusLed();
  reportStatus();
}

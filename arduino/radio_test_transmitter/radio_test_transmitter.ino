#include <SPI.h>
#include <RF24.h>

namespace {

constexpr uint8_t PIN_STATUS_LED = 4;
constexpr uint8_t PIN_NRF_CE = 9;
constexpr uint8_t PIN_NRF_CSN = 10;

constexpr bool ENABLE_SERIAL_DEBUG = true;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t MAX_DCC_PACKET_BYTES = 6;
constexpr uint32_t PAYLOAD_INTERVAL_MS = 100;
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 500;
constexpr uint32_t LED_FLASH_MS = 40;
constexpr uint32_t SERIAL_REPORT_INTERVAL_MS = 1000;

struct RadioPacket {
  uint8_t version;
  uint8_t flags;
  uint8_t length;
  uint8_t sequence;
  uint8_t data[MAX_DCC_PACKET_BYTES];
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kRadioAddress[6] = "DCC01";

uint8_t sequenceCounter = 0;
uint8_t speedStep = 1;
uint8_t packetPattern = 0;
uint32_t lastPayloadMs = 0;
uint32_t lastKeepaliveMs = 0;
uint32_t ledOffMs = 0;
uint32_t lastSerialReportMs = 0;
uint32_t sentPayloads = 0;
uint32_t sentKeepalives = 0;

uint8_t appendChecksum(uint8_t *packet, uint8_t payloadLength) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < payloadLength; ++i) {
    checksum ^= packet[i];
  }
  packet[payloadLength] = checksum;
  return static_cast<uint8_t>(payloadLength + 1);
}

void flashStatusLed() {
  digitalWrite(PIN_STATUS_LED, HIGH);
  ledOffMs = millis() + LED_FLASH_MS;
}

void writeRadioPacket(const RadioPacket &payload) {
  radio.write(&payload, sizeof(payload));
  flashStatusLed();
}

void sendKeepalive() {
  RadioPacket payload{};
  payload.version = PROTOCOL_VERSION;
  payload.flags = 0x08;
  payload.length = 0;
  payload.sequence = sequenceCounter++;
  writeRadioPacket(payload);
  ++sentKeepalives;
}

void sendDccLikePayload() {
  uint8_t dccPacket[MAX_DCC_PACKET_BYTES] = {};
  uint8_t dccLength = 0;
  uint8_t flags = 0x01;

  switch (packetPattern) {
    case 0:
      dccPacket[0] = 3;
      dccPacket[1] = 0x3F;
      dccPacket[2] = static_cast<uint8_t>(0x80 | (speedStep + 1));
      dccLength = appendChecksum(dccPacket, 3);
      flags |= 0x04;
      speedStep = static_cast<uint8_t>((speedStep % 28) + 1);
      break;

    case 1:
      dccPacket[0] = 3;
      dccPacket[1] = static_cast<uint8_t>(0x80 | ((sequenceCounter >> 1) & 0x1F));
      dccLength = appendChecksum(dccPacket, 2);
      break;

    default:
      dccPacket[0] = 0;
      dccPacket[1] = 0x41;
      dccLength = appendChecksum(dccPacket, 2);
      flags |= 0x02 | 0x04;
      break;
  }

  packetPattern = static_cast<uint8_t>((packetPattern + 1) % 3);

  RadioPacket payload{};
  payload.version = PROTOCOL_VERSION;
  payload.flags = flags;
  payload.length = dccLength;
  payload.sequence = sequenceCounter++;
  for (uint8_t i = 0; i < dccLength; ++i) {
    payload.data[i] = dccPacket[i];
  }

  writeRadioPacket(payload);
  ++sentPayloads;

  if (ENABLE_SERIAL_DEBUG) {
    Serial.print(F("tx payload seq="));
    Serial.print(payload.sequence);
    Serial.print(F(" flags=0x"));
    Serial.print(payload.flags, HEX);
    Serial.print(F(" len="));
    Serial.println(payload.length);
  }
}

void updateStatusLed() {
  if (ledOffMs != 0 && static_cast<int32_t>(millis() - ledOffMs) >= 0) {
    digitalWrite(PIN_STATUS_LED, LOW);
    ledOffMs = 0;
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

  Serial.print(F("tx payloads="));
  Serial.print(sentPayloads);
  Serial.print(F(" keepalives="));
  Serial.print(sentKeepalives);
  Serial.print(F(" nextSeq="));
  Serial.println(sequenceCounter);
}

}  // namespace

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(115200);
    Serial.println(F("BDCC radio test transmitter"));
  }

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(108);
  radio.openWritingPipe(kRadioAddress);
  radio.stopListening();

  lastPayloadMs = millis();
  lastKeepaliveMs = millis();
  ledOffMs = millis() + 500;
}

void loop() {
  const uint32_t now = millis();

  if ((now - lastPayloadMs) >= PAYLOAD_INTERVAL_MS) {
    sendDccLikePayload();
    lastPayloadMs = now;
  }

  if ((now - lastKeepaliveMs) >= KEEPALIVE_INTERVAL_MS) {
    sendKeepalive();
    lastKeepaliveMs = now;
  }

  updateStatusLed();
  reportStatus();
}

# ESP32 LoRa Sketches

Deze map bevat een eerste ESP32-S3/SX1262 variant naast de bestaande Nano/NRF24 sketches:

- `arduino/esp32_lora_transmitter/esp32_lora_transmitter.ino`
- `arduino/esp32_lora_receiver/esp32_lora_receiver.ino`

Doel: hetzelfde state-change concept testen via LoRa op `868 MHz`.

## Hardwaredoel

De sketches zijn gecompileerd voor:

```text
Heltec-esp32:esp32:heltec_wifi_lora_32_V3
```

Dit past bij ESP32-S3 boards met SX1262 LoRa en 0.96" OLED.

## Radio-opzet

Er is één transmitter. Die zendt LoRa-frames als broadcast uit. Alle receivers luisteren continu mee en verwerken dezelfde radiostroom.

De radioframes zijn bewust ASCII-tekst, niet binaire C++ structs. Dat volgt hetzelfde gedrag als de Heltec voorbeeldsketches en voorkomt problemen door struct-padding, endianess of lengteverschillen tussen sketches.

```text
B2,<sequence>,K
B2,<sequence>,D,<flags>,<dcc-packet-hex>
```

Voorbeeld:

```text
B2,17,K
B2,18,D,5,033F7E42
```

De transmitter:

- decodeert DCC op `PIN_DCC_IN`
- stuurt alleen relevante state-changes
- stuurt lage-rate state-refresh ongeveer elke `2 s`
- stuurt keepalive elke `1 s` zolang er recente DCC-brondata is
- gebruikt geen acknowledgements

De receiver:

- ontvangt broadcast LoRa-frames
- cachet ontvangen DCC-packets
- genereert lokaal continu DCC op twee output-pinnen
- gebruikt idle-packets als er geen bruikbare state is
- toont diagnose als iconen op de OLED

## DCC-timing

De ESP32 receiver gebruikt RMT voor de DCC-output, gebaseerd op de ESP32-implementatie in DCC-EX:

- DCC `1` halfperiode: `58 us`
- DCC `0` halfperiode: `100 us`
- RMT clock divider: `80`, dus RMT-ticks zijn `1 us`
- één RMT-output wordt naar `PIN_DCC_OUT_A` gestuurd
- dezelfde RMT-output wordt via de GPIO matrix geïnverteerd naar `PIN_DCC_OUT_B`

DCC-EX beveelt de ESP32-WROOM specifiek aan. Heltec WiFi LoRa 32 V3 gebruikt ESP32-S3; die heeft minder RMT-geheugen per kanaal. Deze sketch gebruikt daarom twee RMT memory blocks voor één DCC-output. De huidige packetgrootte past daarin, maar het signaal moet nog met scope/logic analyzer worden gevalideerd.

## OLED-iconen

De receiver gebruikt geen LED event mask. Scenario's worden altijd als grote iconen getoond:

| Scenario | Icoon |
|---|---|
| startup | power-symbool |
| link down | antenne met kruis |
| payload | groot DCC-adres van het laatst ontvangen packet |
| keepalive | heartbeat |
| failsafe | dubbele cirkel met uitroepteken |
| config | tandwiel |
| normaal | concentrische link-cirkels |

Er wordt bewust geen tekst gebruikt, zodat de status op afstand sneller herkenbaar is.

## Belangrijke pinnen

Controleer deze constants bovenin de sketches voordat je hardware aansluit:

```cpp
constexpr uint8_t PIN_DCC_IN = 2;
constexpr uint8_t PIN_DCC_OUT_A = 3;
constexpr uint8_t PIN_DCC_OUT_B = 4;
constexpr uint8_t PIN_HBRIDGE_EN = 5;
constexpr bool ENABLE_HBRIDGE_EN_PIN = false;
```

De DCC-outputpinnen zijn testdefaults voor de Heltec V3 variant. Kies op het echte board vrije GPIO's die niet door OLED, LoRa, flash, USB of bootstrapping gebruikt worden.

## Configuratie via PoM

De ESP32 LoRa receiver ondersteunt dezelfde basis-CV's voor PoM-configuratie op adres `99`, behalve de oude LED-mask CV. De OLED-iconen zijn vast gedrag.

| CV | Betekenis |
|---|---|
| `901` | link-down timeout in `100 ms` |
| `902` | afremtijd link-down in `100 ms` |
| `903` | optrektijd link-up in `100 ms` |
| `904` | max snelheid na link-up |
| `905` | link-up gedrag |
| `908` | command: `1` save, `8` factory reset |

## Compile check

Gecontroleerd op Heltec WiFi LoRa 32 V3:

```text
esp32_lora_transmitter:
Sketch uses 357019 bytes (10%) flash
Global variables use 25228 bytes (7%) RAM

esp32_lora_receiver:
Sketch uses 396855 bytes (11%) flash
Global variables use 25316 bytes (7%) RAM
```

## Beperkingen

- Dit is een eerste hardwaretestbare LoRa-port, geen vervanging van de bestaande Nano/NRF24 referentie.
- DCC-output op ESP32 gebruikt RMT met DCC-EX timingwaarden; timing moet nog met scoop/logic analyzer gevalideerd worden.
- Failsafe genereert nu idle wanneer de link down is. Remcurve-integratie kan later gelijkgetrokken worden met de Nano receiver.
- LoRa duty-cycle blijft relevant. Houd keepalive/state-refresh laag.

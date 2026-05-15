# ESP32-C3 ESP-NOW Sketches

Deze variant gebruikt ESP32-C3 boards aan beide kanten:

- `arduino/esp32c3_espnow_central/esp32c3_espnow_central.ino`
- `arduino/esp32c3_espnow_receiver/esp32c3_espnow_receiver.ino`

Doel: dezelfde state-change aanpak als de LoRa/NRF varianten, maar via ESP-NOW op 2.4 GHz. Dit vermijdt de 868 MHz duty-cycle beperking en is daarom beter geschikt als meerdere locs regelmatig snelheidsupdates krijgen.

## Centrale Kant

DCC-EX blijft ongewijzigd op de Arduino Mega draaien. De centrale ESP32-C3 leest het bestaande DCC-signaal mee op `PIN_DCC_IN`.

Efficiënte praktische koppeling:

```text
DCC-EX Mega DCC-output of rail-signaal
-> optocoupler of comparator/level-shifter
-> ESP32-C3 GPIO PIN_DCC_IN
```

Dit is voor nu efficiënter dan een DCC-EX plugin, omdat:

- DCC-EX op de Mega niet aangepast hoeft te worden.
- De C3 exact dezelfde DCC-stroom ziet als een decoder.
- De radio-bridge los van DCC-EX getest kan worden.
- Het later nog steeds vervangen kan worden door een seriële/plugin-koppeling als we minder DCC-decodeerwerk willen.

Sluit het DCC-signaal niet direct op de C3 aan. De ESP32-C3 is 3.3V-only.

## Radio Protocol

ESP-NOW gebruikt een compact binair broadcast-frame:

```cpp
magic, version, type, sequence, flags, length, data[6], checksum
```

Frame types:

| Type | Betekenis |
|---|---|
| `1` | DCC packet |
| `2` | keepalive |

Alle receivers luisteren mee naar dezelfde broadcast. Er is geen pairing of ACK per loc.

## State-Change Gedrag

De centrale-C3:

- decodeert DCC-packets vanaf de Mega
- filtert relevante adressen `1..20`, broadcast en configadres `99`
- stuurt een packet direct bij state-change
- ververst gecachete state ongeveer elke `2 s`
- stuurt keepalive elke `1 s` zolang er recente DCC-brondata is

De receiver-C3:

- ontvangt ESP-NOW broadcast
- cachet DCC state-packets
- genereert continu lokaal DCC naar de H-brug
- stuurt idle-packets als de link down is of er nog geen state is
- ondersteunt PoM-config-CV's op adres `99`

## Pinnen

Controleer deze constants bovenin de sketches:

```cpp
// centrale
constexpr uint8_t PIN_DCC_IN = 2;
constexpr uint8_t PIN_STATUS_LED = 8;

// receiver
constexpr uint8_t PIN_DCC_OUT_A = 3;
constexpr uint8_t PIN_DCC_OUT_B = 4;
constexpr uint8_t PIN_HBRIDGE_EN = 5;
constexpr uint8_t PIN_STATUS_LED = 8;
constexpr bool ENABLE_HBRIDGE_EN_PIN = false;
```

Let op: veel ESP32-C3 boards gebruiken GPIO8 als onboard LED, maar niet allemaal. Pas `PIN_STATUS_LED` aan per board.

De sketches sturen zowel `PIN_STATUS_LED` als `LED_BUILTIN` aan. Als het board package `LED_BUILTIN` op dezelfde pin zet als `PIN_STATUS_LED`, wordt die pin maar één keer aangestuurd. De ingebouwde LED en externe statusled tonen dus hetzelfde patroon.

## DCC Output

De receiver gebruikt twee RMT TX-kanalen:

- `PIN_DCC_OUT_A`: normale DCC golfvorm
- `PIN_DCC_OUT_B`: geïnverteerde DCC golfvorm

Dit past bij een H-brug met twee logic inputs zoals DRV8871 `IN1/IN2`. De timingwaarden zijn:

| DCC bit | Halfperiode |
|---|---:|
| `1` | `58 us` |
| `0` | `100 us` |

De C3 heeft minder RMT-marge dan S3, maar voor één DCC-output zonder OLED is dit precies de juiste toepassing.

## Statusled

Centrale:

| Status | LED |
|---|---|
| startup | aan |
| ESP-NOW init fout | snel knipperen |
| TX activiteit | korte flash |
| normaal | trage heartbeat |

Receiver:

| Status | LED |
|---|---|
| startup | aan |
| ESP-NOW init fout | snel knipperen |
| link down | continu aan |
| payload/config ontvangen | korte flash |
| link gezond | korte heartbeat |
| nog geen payload | sneller knipperen |

## Configuratie Via PoM

De receiver ondersteunt dezelfde basis-CV's op adres `99`:

| CV | Betekenis |
|---|---|
| `901` | link-down timeout in `100 ms` |
| `902` | afremtijd link-down in `100 ms` |
| `903` | optrektijd link-up in `100 ms` |
| `904` | max snelheid na link-up |
| `905` | link-up gedrag |
| `908` | command: `1` save, `8` factory reset |

De huidige C3 receiver gebruikt deze waarden vooral voor link-timeout en configuratieconsistentie. Rem/optrekcurve-integratie kan later gelijkgetrokken worden met de Nano receiver.

## Compile Check

Gecontroleerd op:

```text
esp32:esp32:esp32c3
```

Voor native USB-serial op veel ESP32-C3 boards moet in de Arduino IDE staan:

```text
Tools -> USB CDC On Boot -> Enabled
Serial Monitor -> 115200 baud
```

Als `USB CDC On Boot` uit staat, zie je vaak alleen de ROM bootregel zoals `ESP-ROM:esp32c3-api1-20210207` en geen `Serial` output van de sketch.

De receiver print bij boot nu onder andere:

```text
BDCC ESP32-C3 ESP-NOW receiver boot
Config: linkTimeout=...
DCC RMT output initialized
ESP-NOW init OK
ESP-NOW receive callback registered
Setup complete
```

Tijdens bedrijf print de receiver beperkt:

```text
Payload addr=3 len=3 seq=...
Stats rx=... ok=... payload=... keepalive=... invalid=... stale=... link=UP
Link DOWN at ... ms
```

De centrale print nu extra ESP-NOW diagnose, inclusief keepalive-zendingen. In debugmodus staat `SEND_KEEPALIVE_WITHOUT_DCC_SOURCE = true`; daardoor stuurt de centrale keepalives ook als er nog geen DCC van de Mega wordt gedecodeerd. Dat maakt testen eenvoudiger:

- Receiver wordt `link=UP`: ESP-NOW werkt, DCC-input/decode is dan de volgende verdachte als `dcc=NO` blijft.
- Receiver blijft `link=DOWN`: ESP-NOW kanaal, boardselectie, upload of receiver-code is de verdachte.

Voorbeeld central output:

```text
Keepalive queued seq=12 dccSource=NO queuedKeepalive=12
Stats dcc=NO decoded=0 payloadQ=0 refreshQ=0 keepaliveQ=12 sendStart=12 sentOk=12 fail=0 drop=0 queue=0 busy=0 lastType=2 lastSeq=11
```

Bij werkende DCC-input verschijnt ook:

```text
DCC source ACTIVE at ... ms
DCC packet addr=3 len=3 decoded=...
```

Resultaat:

```text
esp32c3_espnow_central:
Sketch uses 956713 bytes (72%) flash
Global variables use 37012 bytes (11%) RAM

esp32c3_espnow_receiver:
Sketch uses 950509 bytes (72%) flash
Global variables use 36164 bytes (11%) RAM
```

## Open Punten

- DCC-input vanaf de Mega moet elektrisch netjes worden geïsoleerd of naar 3.3V worden gevormd.
- DCC-output timing moet met scope/logic analyzer gecontroleerd worden op het gekozen C3 board.
- ESP-NOW bereik moet op het echte traject worden gemeten; dit blijft 2.4 GHz.
- Als later directe DCC-EX integratie gewenst is, is een seriële/plugin-koppeling de logische tweede stap.

# LGB Accu DCC Bridge - Eerste Ontwerp

## Doel

Een centrale zender leest het DCC-signaal van de centrale en stuurt alleen de relevante DCC-packets voor locadressen `1..20` draadloos door. Een ontvanger in een externe accuwagon reconstrueert daaruit weer een continue DCC-stroom op ongeveer `20V`, waarna de aangesloten `Massoth/LGB` decoder zelf zijn eigen adres eruit filtert.

## Architectuur

```text
DCC centrale
  ->
DCC input frontend (optocoupler / comparator)
  ->
Arduino Nano zender
  ->
NRF24L01+ broadcast
  ->
Arduino Nano ontvanger
  ->
DCC packet cache
  ->
DCC generator
  ->
H-brug op 20V
  ->
Massoth/LGB decoder rail input
```

## Systeemaanname

- Locadressen blijven beperkt tot `1..20`
- Alleen standaard loc-packets in versie 1
- Decoder blijft motor, licht en geluid zelf aansturen
- Accuwagon levert een "virtuele rails" aan de decoder
- Service mode CV lezen/schrijven valt buiten versie 1

## Pinplan

### Zender (`Arduino Nano` naast de centrale)

| Pin | Functie |
|---|---|
| `D2` | `DCC_IN` interrupt-ingang vanaf optocoupler/comparator |
| `D4` | `STATUS_LED` |
| `D9` | `NRF_CE` |
| `D10` | `NRF_CSN` |
| `D11` | `NRF_MOSI` |
| `D12` | `NRF_MISO` |
| `D13` | `NRF_SCK` |
| `5V` | voeding Nano / NRF adapter |
| `GND` | common ground |

Opmerking:
- `DCC_IN` moet uit een nette frontend komen. Niet direct de DCC-baan op de Nano zetten.
- Gebruik bij voorkeur galvanische scheiding met optocoupler of comparatortrap.

### Ontvanger (`Arduino Nano` in accuwagon)

| Pin | Functie |
|---|---|
| `D3` | `LINK_LED` |
| `D5` | `DCC_OUT_A` naar H-brug `IN1` |
| `D6` | `DCC_OUT_B` naar H-brug `IN2` |
| `D8` | `HBRIDGE_EN` optioneel enable-pin |
| `D9` | `NRF_CE` |
| `D10` | `NRF_CSN` |
| `D11` | `NRF_MOSI` |
| `D12` | `NRF_MISO` |
| `D13` | `NRF_SCK` |
| `5V` | voeding Nano / NRF adapter |
| `GND` | common ground |

## Vermogenspad ontvanger

```text
2S / 3S / 4S LiPo
  ->
zekering 5A
  ->
hoofdschakelaar
  ->
buffer + TVS
  ->
split:

1. Buck -> 5V -> Nano + NRF24
2. Boost -> 20V -> H-brug VM
```

## Draadloos protocol

De zender verstuurt geen ruwe bitstream maar gedecodeerde DCC-packets.

### Payloadformaat

```text
byte 0  protocol version
byte 1  flags
byte 2  packet length (2..6)
byte 3  sequence
byte 4  packet byte 0
byte 5  packet byte 1
byte 6  packet byte 2
byte 7  packet byte 3
byte 8  packet byte 4
byte 9  packet byte 5
```

### Flags

- `bit0`: packet is geldig
- `bit1`: packet is broadcast (`address 0`)
- `bit2`: resend/high-priority
- `bit3`: keepalive zonder DCC-payload

### Filtering zender

Doorsturen:
- korte locadressen `1..20`
- broadcast stop / address `0`

Niet doorsturen in versie 1:
- accessory packets
- service mode programmeerpackets
- lange adressen

## Werking zender

1. Meet DCC flanktijden via interrupt.
2. Decodeer preamble, databits en XOR-byte.
3. Valideer packetlengte en checksum.
4. Filter alleen relevante adressen `1..20` plus broadcast.
5. Verstuur nieuwe packets direct via `NRF24`.
6. Herhaal gecachte packets periodiek voor multi-ontvanger gedrag.
7. Stuur een keepalive als er tijdelijk geen nieuw DCC-packet langskomt.

## Werking ontvanger

1. Ontvang packet via `NRF24`.
2. Sla functie- en broadcast-pakketten per slot op.
3. Decodeer snelheidspakketten per locadres en bewaar die als lokale toestand.
4. Herhaal alle actieve slots cyclisch als DCC-packets.
5. Genereer snelheidspakketten synthetisch uit de lokale toestand.
6. Vul op met idle-packets wanneer nodig.
7. Bij radio-timeout remt de ontvanger lokaal af naar `0`.
8. Zodra radioverkeer terugkeert trekt de ontvanger weer op naar de laatst bekende snelheid.

## Fail-safe constants

In de ontvanger zitten drie `const` waarden om mee te testen:

- `SIGNAL_LOSS_TIMEOUT_MS`
- `RAMP_STEP_INTERVAL_MS`
- `RAMP_STEP_DELTA`

Daarmee kun je de veilige marge en rem/optrekvertraging tunen zonder de structuur te wijzigen.

## Belangrijke beperking van deze eerste versie

- Nog geen volledige NMRA-decode van alle packetvarianten
- Alleen korte adressen
- Speed-packets zijn in deze versie gericht op `28` en `128` speed steps
- Geen bevestigde radio-link
- DCC-uitgang gebeurt in software; voor hogere robuustheid kan later Timer1 worden ingezet
- Sketches zijn een eerste prototypebasis en nog niet op hardware gecompileerd/getest

## Eerste testvolgorde

1. Ontvanger standalone, zonder radio:
   vaste DCC idle en een test-speedpacket genereren.
2. Decoder op `20V` virtuele rails aansluiten.
3. Verlichting en motor testen.
4. Daarna zender activeren en live packets doorgeven.

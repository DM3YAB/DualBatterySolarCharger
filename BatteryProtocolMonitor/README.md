# BatteryProtocolMonitor

## Overview

The **BatteryProtocolMonitor** is the passive communication and diagnostic monitor of the **DualBatterySolarCharger** system.

It observes the UART communication between:

- **BatterySourceCharger**
- **BatteryAssistCharger**

The monitor does not participate in charging control and does not transmit commands to either charger.

Its purpose is to make the communication between both controllers visible, verifiable and recordable during development and long-term operation.

```text
BatterySourceCharger                  BatteryAssistCharger
        │                                      │
        │          MASTER_REQUEST              │
        ├─────────────────────────────────────►│
        │                                      │
        │          CLIENT_RESPONSE             │
        │◄─────────────────────────────────────┤
        │                                      │
        │                 │                    │
        │                 ▼                    │
        │       BatteryProtocolMonitor         │
        │          passive monitor             │
        │                                      │
```

A failure, reset or complete removal of the BatteryProtocolMonitor has no influence on the charging system.


## Role in the System

The BatterySourceCharger and BatteryAssistCharger exchange information required for coordinated operation and SolarShare.

The ProtocolMonitor observes this exchange independently.

Its principal tasks are:

- receive both communication directions,
- identify complete protocol frames,
- verify protocol structure,
- verify CRC,
- check sequence numbers,
- check payload plausibility,
- detect incomplete frames,
- detect communication timeouts,
- display decoded operating information,
- record communication data,
- record rejected or corrupted frames separately.

The monitor therefore provides a diagnostic view of the complete Source–Assist system without becoming part of its control loop.


## Passive Monitoring Principle

The monitor receives the transmit signal from both controllers independently.

Conceptually:

```text
SOURCE TX ───────────────┐
                        │
                        ▼
                 ┌───────────────┐
                 │               │
                 │   Battery     │
                 │   Protocol    │
                 │   Monitor     │
                 │               │
                 └───────────────┘
                        ▲
                        │
ASSIST TX ───────────────┘
```

The monitor does not need to be inserted into the communication path.

This is an important design principle:

> **The monitor observes communication but must never be required for communication to work.**


## Protocol

The monitored protocol uses fixed-length binary frames.

Current basic protocol parameters are:

| Parameter | Value |
|---|---:|
| Start byte 1 | `0xA5` |
| Start byte 2 | `0x5A` |
| Protocol version | `0x01` |
| Payload length | 21 bytes |
| Complete frame length | 30 bytes |
| CRC | CRC16-CCITT |
| CRC polynomial | `0x1021` |
| CRC initial value | `0xFFFF` |

The detailed protocol definition is documented in:

[Communication Protocol](../Documentation/CommunicationProtocol.md)


## Communication Directions

The monitor distinguishes the two communication directions.

### Source → Assist

The BatterySourceCharger transmits:

`MASTER_REQUEST`

Important information includes:

- 24 V battery voltage,
- 24 V battery current,
- source power,
- safe source power,
- SolarShare power grant,
- Source battery SOC field,
- selected source,
- MPPT state,
- DC/DC state,
- Source warnings,
- Source errors.


### Assist → Source

The BatteryAssistCharger responds with:

`CLIENT_RESPONSE`

Important information includes:

- 12 V battery voltage,
- 12 V battery current,
- requested input power,
- actual input power,
- Assist battery SOC field,
- charge state,
- operating state,
- 12 V battery capacity,
- battery information code,
- Assist warnings,
- Assist errors.


## Frame Validation

The monitor does not simply decode every received byte sequence as valid data.

A frame passes through several validation steps.

```text
Receive data
     │
     ▼
Find A5 5A header
     │
     ▼
Check protocol version
     │
     ▼
Check message type
     │
     ▼
Check payload length
     │
     ▼
Check CRC
     │
     ▼
Check payload plausibility
     │
     ▼
Check sequence relationship
     │
     ▼
Valid protocol data
```

This makes it possible to distinguish a communication problem from a valid frame containing an abnormal operating condition.


## CRC Validation

The protocol uses:

```text
CRC16-CCITT
Polynomial     0x1021
Initial value  0xFFFF
```

The CRC is calculated over:

```text
Protocol Version
Message Type
Sequence
Payload Length
Payload
```

The start bytes:

```text
A5 5A
```

are not included in the CRC calculation.

A frame with an incorrect CRC is rejected and can be recorded for later analysis.


## Payload Plausibility

A frame can have a correct structure and CRC while still containing data that should not be accepted as valid system information.

The monitor therefore also performs plausibility checks.

Examples for Source data include:

| Field | Accepted range |
|---|---:|
| 24 V battery voltage | ≤ 36.00 V |
| Battery current | -50 ... +50 A |
| Source power | ≤ 1500 W |
| Safe source power | ≤ 1500 W |
| Assist grant | ≤ 500 W |
| Battery SOC | 0 ... 100 % |
| Source mode | 0 ... 3 |
| MPPT state | 0 ... 7 |
| DC/DC state | 0 ... 4 |

Examples for Assist data include:

```text
Battery SOC             0 ... 100 %
Charge state            0 ... 4
Battery capacity        0 ... 2000 Ah     Fix!Me
Battery information     0 ... 7
```

These values are **protocol plausibility limits**.

They are not electrical protection limits of the chargers.


## MPPT State Range

During development, the Source MPPT state was extended with additional low-solar conditions.

The monitor therefore accepts:

```text
mpptState = 0 ... 7
```

including:

```text
6 = LOW_SOLAR
7 = LOW_PAUSE
```

This is important because these are valid Source operating states and must not be rejected as implausible protocol data.


## Sequence Monitoring

Each `MASTER_REQUEST` contains an 8-bit sequence number.

The Assist echoes this sequence number in its corresponding `CLIENT_RESPONSE`.

Example:

```text
SOURCE                         ASSIST

MASTER_REQUEST Seq=73
       ───────────────────────►

CLIENT_RESPONSE Seq=73
       ◄───────────────────────


MASTER_REQUEST Seq=74
       ───────────────────────►

CLIENT_RESPONSE Seq=74
       ◄───────────────────────
```

The Source increments the sequence number:

```text
0 → 1 → 2 → ... → 254 → 255 → 0
```

The ProtocolMonitor can therefore identify conditions such as:

- missing requests,
- missing responses,
- unexpected sequence changes,
- request/response mismatches.


## Communication Timing

Current system communication parameters include:

```text
Communication interval       1000 ms
RX partial-frame timeout       20 ms     Fix!Me
Communication link timeout   3000 ms     Fix!Me
```

The monitor can use communication timing in addition to frame validation.

For example, a valid Source frame without the expected Assist response is different from a corrupted UART frame.

These conditions should therefore remain distinguishable in diagnostics.


## Rejected Frames

Invalid frames are not silently discarded.

The monitor can classify rejected frames using categories including:

```text
VERSION_ERROR
UNKNOWN_TYPE
LENGTH_ERROR
CRC_ERROR
DATA_IMPLAUSIBLE
FRAME_TIMEOUT
```

Where possible, the raw received data is retained together with the rejection reason.

This is particularly useful for diagnosing intermittent communication problems.


## REJECT.CSV

Rejected frames can be stored separately from normal communication data.

The current logging concept uses:

```text
/LOG/YYYYMMDD/REJECT.CSV
```

The reject log allows later investigation of:

- CRC failures,
- malformed frames,
- incomplete frames,
- invalid protocol versions,
- unexpected message types,
- implausible data.

Keeping rejected data separate prevents normal operating logs from becoming difficult to evaluate while still preserving diagnostic evidence.


## Communication Pair Monitoring

The monitor also observes the relationship between a Source request and the corresponding Assist response.

Conceptually:

```text
MASTER_REQUEST
      │
      ▼
Wait for matching CLIENT_RESPONSE
      │
      ├── received correctly
      │       → communication pair OK
      │
      └── missing / late
              → pair timeout
```

A pair timeout is a communication event rather than a malformed-frame parser error.

This distinction helps identify whether:

- a frame was transmitted but corrupted,
- a frame was incomplete,
- or a controller simply did not respond.


## Parser Noise

Bytes that do not form the expected:

```text
A5 5A
```

frame header are treated as header noise.

This allows the monitor to count unexpected UART activity without interpreting every unrelated byte as a complete rejected protocol frame.


## Diagnostic Counters

The monitor maintains diagnostic information in RAM for communication problems such as:

- CRC errors,
- format errors,
- data plausibility errors,
- timeouts,
- sequence anomalies.

These counters provide a quick indication of communication quality without requiring every individual log entry to be inspected.


## Information Display

The monitor decodes protocol values into human-readable operating information.

The purpose is to make the relationship between both chargers visible at the same time.

Typical Source information includes:

```text
24 V battery
battery current
source power
safe source power
Assist grant
source mode
MPPT state
DC/DC state
warnings / errors
```

Typical Assist information includes:

```text
12 V battery
battery current
actual input power
battery capacity
charge state
operating state
battery information
warnings / errors
```

This makes the monitor useful not only for finding protocol errors but also for understanding system behaviour during normal operation.


## Battery Capacity Display

The Assist reports its configured 12 V battery capacity as:

`batteryCapacity_Ah`

The monitor decodes and displays this field in Ah.

The field uses:

```text
uint16_t
1 Ah / bit
```

with:

```text
0 Ah = unknown / not configured
```

The current plausibility ceiling is:

`2000 Ah`

and is marked `Fix!Me`.


## Battery Information

The Assist transmits its current battery observation as:

`batteryInfoCode`

Current values are:

| Value | Meaning |
|---:|---|
| 0 | `NORMAL` |
| 1 | `LOW` |
| 2 | `LOAD_ACTIVE` |
| 3 | `RECOVERY` |
| 4 | `FAST_DISCHARGE` |
| 5 | `SUPPORT_REQUEST` |
| 6 | `CHECK_BATTERY` |
| 7 | `EXTERNAL_CHARGE` |

The ProtocolMonitor displays this information separately from warning and error flags.

This distinction is intentional:

```text
INFO
    Current operating observation

WARNING
    Suspicious or abnormal condition

ERROR
    Protection or fault condition
```


## Warning and Error Display

The monitor decodes warning and error flags from both chargers.

### BatterySourceCharger

Current warnings:

| Mask | Meaning |
|---:|---|
| `0x0001` | Thermal derating |
| `0x0002` | Recovery unusually long |

Current errors:

| Mask | Meaning |
|---:|---|
| `0x0001` | DC/DC pre-charge failure |
| `0x0002` | Battery overvoltage |
| `0x0004` | Overtemperature |


### BatteryAssistCharger

Current warnings:

| Mask | Meaning |
|---:|---|
| `0x0001` | Low battery |
| `0x0002` | Fast discharge |
| `0x0004` | Thermal derating |

Current defined errors:

| Mask | Meaning |
|---:|---|
| `0x0001` | DC/DC pre-charge failure |
| `0x0002` | Battery overvoltage |
| `0x0004` | Overtemperature |

The monitor reports the transmitted flags but does not make charging decisions based on them.


## Data Logging

The ProtocolMonitor is intended for both immediate observation and longer-term recording.

Logging allows the behaviour of the complete system to be reconstructed later.

This is useful when investigating events such as:

- changing solar conditions,
- MPPT transitions,
- SolarShare behaviour,
- battery recovery,
- external charging,
- support operation,
- thermal derating,
- communication errors.

The monitor therefore acts as an independent diagnostic observer of the complete system.


## Display Extension

A local display is planned as an extension of the ProtocolMonitor.

The intended display is an:

**EA W320-8K3**

with a resolution of:

`320 × 240`

The display is intended primarily for a clear text-based overview rather than graphical history.

The planned display should show the current Source and Assist states with clear separators and readable status information.

Conceptually:

```text
┌──────────────────────────────────────────────┐
│ SOURCE 24 V                                  │
│ Battery   Current   Solar   Safe   Grant     │
│ Source    MPPT      DCDC                     │
├──────────────────────────────────────────────┤
│ ASSIST 12 V                                  │
│ Battery   Current   Power   Capacity         │
│ Charge    State     Info                     │
├──────────────────────────────────────────────┤
│ COMMUNICATION                                │
│ Sequence   CRC   Rejects   Timeouts          │
└──────────────────────────────────────────────┘
```

The display is not intended to become part of the protocol parser itself.

The preferred software architecture is:

```text
UART receivers
      │
      ▼
Protocol parser
      │
      ▼
Common monitor state
      │
      ├────────► Serial / logging
      │
      └────────► Display
```

This keeps communication processing independent from the user interface.

> **Development status:** The display extension is planned separately. The exact hardware interface and GPIO assignment should only be documented after the final implementation has been verified.


## Display Power Control

The display is intended to be normally switchable rather than permanently illuminated.

The planned concept uses:

- a physical `DisplayON` button,
- controlled high-side display power,
- a P-channel switching device driven through an N-MOSFET or NPN stage.

This allows the monitor itself to continue observing and recording communication while the display is switched off.

The final circuit and GPIO assignment belong in the Hardware documentation after verification.


## Firmware

The current firmware is located in:

```text
BatteryProtocolMonitor/
└── Firmware/
    └── BatteryProtocolMonitor.ino
```

The monitor firmware is separate from both charger firmwares.

Changes to the monitor must never be required for the chargers themselves to continue operating.


## Hardware

Hardware documentation belongs in:

```text
BatteryProtocolMonitor/
└── Hardware/
```

This directory is intended for:

- controller schematic,
- UART input circuitry,
- SD-card interface,
- display interface,
- display power control,
- component placement,
- PCB documentation,
- hardware revisions,
- photographs and measurement notes.

The W320 display hardware should be added only when the final interface has been established and tested.


## Related Documentation

System-level documentation is available in:

- [System Architecture](../Documentation/SystemArchitecture.md)
- [Charging Strategy](../Documentation/ChargingStrategy.md)
- [Communication Protocol](../Documentation/CommunicationProtocol.md)
- [Parameters](../Documentation/Parameters.md)


## Development Status

The BatteryProtocolMonitor already provides the fundamental passive protocol-monitoring architecture.

Current functions include:

- independent observation of Source and Assist communication,
- protocol-frame decoding,
- CRC validation,
- payload plausibility checking,
- sequence monitoring,
- timeout detection,
- rejected-frame classification,
- diagnostic counters,
- logging of rejected communication.

The monitor follows a fundamental rule:

> **Observe everything required for diagnosis, but never influence charging.**

Future extensions such as the local W320 display should consume the already decoded monitor state and remain separate from the protocol parser.

This keeps the BatteryProtocolMonitor useful both as a development instrument and as a long-term diagnostic observer of the DualBatterySolarCharger system.

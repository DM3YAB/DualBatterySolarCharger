# BatteryProtocolMonitor Firmware

## Overview

This directory contains the firmware for the **BatteryProtocolMonitor**, the passive communication and diagnostic monitor of the DualBatterySolarCharger system.

Firmware file:

```text
BatteryProtocolMonitor.ino
```

The firmware observes the UART communication between:

- BatterySourceCharger
- BatteryAssistCharger

Its principal functions are:

- independent reception of both communication directions
- protocol frame detection
- frame validation
- CRC verification
- payload plausibility checking
- sequence monitoring
- request/response pair monitoring
- communication timeout detection
- decoding of Source and Assist operating data
- diagnostic counters
- normal communication logging
- separate logging of rejected frames

The monitor is deliberately passive.

It does not transmit charging commands and does not participate in battery or power control.


## Development Environment

The ProtocolMonitor is implemented on a separate microcontroller with multiple hardware UART interfaces.

The current firmware was developed for the STM32-based monitor hardware.

The Source and Assist transmit signals are connected to separate UART receivers so that both communication directions can be observed independently.

Conceptually:

```text
SOURCE TX ──────────────► UART RX 1
                              │
                              │
                              ▼
                     BatteryProtocolMonitor
                              ▲
                              │
                              │
ASSIST TX ──────────────► UART RX 2
```

The monitor is not inserted electrically into the communication path.

A reset, failure or removal of the monitor must therefore have no effect on communication between Source and Assist.


## Firmware Structure

The firmware separates the main tasks conceptually into:

```text
UART reception
      │
      ▼
Frame collection
      │
      ▼
Protocol validation
      │
      ▼
Payload decoding
      │
      ▼
Common monitor data
      │
      ├────────► Serial output
      │
      ├────────► Logging
      │
      └────────► Future local display
```

Protocol reception and validation should remain independent of presentation and display functions.

This separation is especially important for the planned local display extension.


## Monitored Protocol

The current protocol uses fixed-length binary frames.

Basic parameters:

```text
Start byte 1            0xA5
Start byte 2            0x5A
Protocol version        0x01

Payload length          21 bytes
Complete frame length   30 bytes

CRC                     CRC16-CCITT
Polynomial              0x1021
Initial value           0xFFFF
```

Current message types include:

```text
0x0001   MASTER_REQUEST
0x0002   CLIENT_RESPONSE
0x0003   COMMAND
0x0004   COMMAND_RESPONSE
0x0005   DIAGNOSTIC
```

The current Source–Assist communication uses:

```text
MASTER_REQUEST
CLIENT_RESPONSE
```

The remaining message types are reserved for protocol expansion.


## Frame Structure

The complete frame is:

```text
Byte  0      0xA5
Byte  1      0x5A
Byte  2      Protocol Version

Byte  3      Message Type High
Byte  4      Message Type Low

Byte  5      Sequence
Byte  6      Payload Length

Byte  7..27  Payload

Byte 28      CRC High
Byte 29      CRC Low
```

Multi-byte protocol values use:

```text
Big-endian byte order
```

Example:

```text
0x1234

Wire:
12 34
```

Signed current values are transported in a 16-bit field using two's-complement representation.


## CRC Validation

CRC16-CCITT is calculated over:

```text
Byte 2 ... Byte 27
```

This includes:

```text
Protocol Version
Message Type
Sequence
Payload Length
Payload
```

The frame header:

```text
A5 5A
```

is not included in the CRC calculation.

A frame with an incorrect CRC is not accepted as valid operating data.


## Source → Assist Monitoring

The Source transmits:

```text
MASTER_REQUEST
```

Current payload layout:

| Field | Type | Resolution |
|---|---|---|
| batteryVoltage_10mV | U16 | 10 mV |
| batteryCurrent_10mA | I16 | 10 mA |
| sourcePower_W | U16 | 1 W |
| safeSourcePower_W | U16 | 1 W |
| grantedAssistPower_W | U16 | 1 W |
| batterySoc_percent | U8 | 1 % |
| sourceMode | U16 | state |
| mpptState | U16 | state |
| dcdcState | U16 | state |
| warningFlags | U16 | bit field |
| errorFlags | U16 | bit field |

Total payload:

```text
21 bytes
```


## Source Modes

Current Source input modes are:

```text
0   NONE
1   MAIN
2   SOLAR
3   HELP
```

The monitor accepts:

```text
sourceMode = 0 ... 3
```


## Source MPPT States

The Source protocol can transmit MPPT states:

```text
0 ... 7
```

The range includes the additional low-solar states:

```text
6   LOW_SOLAR
7   LOW_PAUSE
```

These are valid Source operating conditions and must not be rejected as implausible protocol data.


## Source DC/DC States

Current Source DC/DC states are:

```text
0   OFF
1   START
2   CHARGE_ON
3   RAMP_CURRENT
4   READY
```

The monitor therefore accepts:

```text
dcdcState = 0 ... 4
```


## Assist → Source Monitoring

The Assist transmits:

```text
CLIENT_RESPONSE
```

Current payload layout:

| Field | Type | Resolution |
|---|---|---|
| batteryVoltage_10mV | U16 | 10 mV |
| batteryCurrent_10mA | I16 | 10 mA |
| requestedInputPower_W | U16 | 1 W |
| actualInputPower_W | U16 | 1 W |
| batterySoc_percent | U8 | 1 % |
| chargeState | U16 | state |
| operatingState | U16 | state |
| batteryCapacity_Ah | U16 | 1 Ah |
| batteryInfoCode | U16 | state |
| warningFlags | U16 | bit field |
| errorFlags | U16 | bit field |

Total payload:

```text
21 bytes
```


## Battery Capacity

The Assist reports its configured 12 V battery capacity using:

```text
batteryCapacity_Ah
```

Encoding:

```text
Type          uint16_t
Resolution    1 Ah / bit
0             unknown / not configured
```

Current protocol plausibility limit:

```text
0 ... 2000 Ah    Fix!Me
```

The technical U16 field itself can represent a larger range.

The 2000 Ah value is only a current plausibility limit used by the monitor.


## Battery Information

The Assist reports its current battery observation through:

```text
batteryInfoCode
```

Current values:

| Value | Meaning |
|---:|---|
| 0 | NORMAL |
| 1 | LOW |
| 2 | LOAD_ACTIVE |
| 3 | RECOVERY |
| 4 | FAST_DISCHARGE |
| 5 | SUPPORT_REQUEST |
| 6 | CHECK_BATTERY |
| 7 | EXTERNAL_CHARGE |

The monitor treats this information separately from warning and error flags.

Conceptually:

```text
INFO
    Current operating observation

WARNING
    Abnormal or suspicious condition

ERROR
    Protection or fault condition
```


## Frame Validation

A received frame is not accepted solely because the start bytes were found.

The monitor validates the frame in several stages:

```text
Receive bytes
     │
     ▼
Find A5 5A
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
Accept decoded data
```

This allows communication faults to be distinguished from valid frames containing unusual charger conditions.


## Exact Payload Length

The current protocol requires:

```text
Payload length = 21 bytes
```

The monitor requires this exact value for the current Source and Assist messages.

A different payload length is classified as a protocol-format error rather than being silently accepted.


## Payload Plausibility

After CRC validation, decoded values are checked for plausible ranges.

Current Source checks include approximately:

```text
Battery voltage          ≤ 36.00 V
Battery current          -50 ... +50 A
Source power             ≤ 1500 W
Safe source power        ≤ 1500 W
Assist power grant       ≤ 500 W
Battery SOC              0 ... 100 %
Source mode              0 ... 3
MPPT state               0 ... 7
DC/DC state              0 ... 4
```

Current Assist checks include:

```text
Battery SOC              0 ... 100 %
Charge state             0 ... 4
Battery capacity         0 ... 2000 Ah    Fix!Me
Battery information      0 ... 7
```

These are diagnostic plausibility limits.

They are not charger protection limits.


## Rejected Frames

Invalid or incomplete frames are classified instead of simply disappearing from the diagnostic data.

Current reject categories include:

```text
VERSION_ERROR
UNKNOWN_TYPE
LENGTH_ERROR
CRC_ERROR
DATA_IMPLAUSIBLE
FRAME_TIMEOUT
```

Where possible, the monitor retains:

- communication direction
- timestamp
- rejection reason
- received raw frame data
- additional diagnostic information

This makes intermittent communication faults easier to investigate later.


## Reject Queue

Rejected frames are temporarily stored in a small RAM queue before they are written to storage.

Current queue size:

```text
8 entries
```

If the queue itself overflows, this condition is recorded as a diagnostic event.

The queue prevents SD-card access from becoming part of the time-critical UART parser path.


## REJECT.CSV

Rejected communication is logged separately from normal protocol data.

Current directory concept:

```text
/LOG/YYYYMMDD/REJECT.CSV
```

Typical reject information includes:

```text
Time
Direction
Reject reason
Frame information
Raw received bytes
```

This allows a normal operating log to remain readable while still preserving malformed communication for later investigation.


## Frame Timeout

An incomplete frame must not remain permanently active in the parser.

If reception begins but the complete expected frame does not arrive within the allowed time, the parser generates:

```text
FRAME_TIMEOUT
```

The available partial frame data can then be retained for diagnosis.

This is different from a complete valid Source request for which no Assist response is received.


## Request / Response Pair Monitoring

The Source uses an 8-bit sequence number.

The Assist echoes the sequence number of the valid `MASTER_REQUEST` in its `CLIENT_RESPONSE`.

Example:

```text
SOURCE                         ASSIST

MASTER_REQUEST
Sequence 42
       ───────────────────────►

CLIENT_RESPONSE
Sequence 42
       ◄───────────────────────
```

The next request uses the next sequence value.

The sequence wraps:

```text
0 → 1 → ... → 254 → 255 → 0
```

The monitor can therefore observe the relationship between requests and responses.


## Pair Timeout

A missing response to a valid request is treated separately from parser errors.

Conceptually:

```text
Valid MASTER_REQUEST
        │
        ▼
Wait for matching response
        │
        ├──── matching CLIENT_RESPONSE
        │            │
        │            ▼
        │         Pair OK
        │
        └──── no matching response
                     │
                     ▼
                Pair timeout
```

A pair timeout is a communication event.

It is not classified as a malformed UART frame because no invalid frame necessarily existed.


## Header Noise

UART bytes that do not form the expected:

```text
A5 5A
```

header are treated as header noise.

This allows unexpected activity on a monitored UART line to be counted without generating a complete reject record for every individual unrelated byte.


## Diagnostic Counters

The monitor maintains communication statistics in RAM.

These include counters for conditions such as:

```text
CRC errors
Format errors
Data plausibility errors
Frame timeouts
Sequence anomalies
Pair timeouts
Header noise
Reject queue overflow
```

The counters provide a quick indication of communication quality even when detailed SD logs are not being inspected.


## Normal Data Output

Valid Source and Assist frames are decoded into human-readable values.

Typical Source information includes:

```text
24 V battery voltage
24 V battery current
Source power
Safe source power
SolarShare grant
Source mode
MPPT state
DC/DC state
Warning flags
Error flags
```

Typical Assist information includes:

```text
12 V battery voltage
12 V battery current
Requested input power
Actual input power
Battery capacity
Charge state
Operating state
Battery information
Warning flags
Error flags
```

The monitor therefore provides both:

```text
Protocol diagnosis
```

and:

```text
System operating diagnosis
```


## Logging

Normal valid communication can be logged for later evaluation.

This makes it possible to reconstruct system behaviour around events such as:

- changing solar conditions
- MPPT state changes
- SolarShare operation
- 24 V BMS recovery
- 12 V Battery Support
- external battery charging
- thermal derating
- warnings and errors
- communication interruptions

The ProtocolMonitor therefore also serves as an independent long-term observer of the complete charging system.


## Warning and Error Decoding

### BatterySourceCharger

Current warning flags:

```text
0x0001   THERMAL_DERATING
0x0002   RECOVERY_LONG
```

Current error flags:

```text
0x0001   DCDC_PRECHARGE
0x0002   BATTERY_OVERVOLTAGE
0x0004   OVERTEMPERATURE
```


### BatteryAssistCharger

Current warning flags:

```text
0x0001   LOW_BATTERY
0x0002   FAST_DISCHARGE
0x0004   THERMAL_DERATING
```

Current defined error flags:

```text
0x0001   DCDC_PRECHARGE
0x0002   BATTERY_OVERVOLTAGE
0x0004   OVERTEMPERATURE
```

The monitor only reports these flags.

It does not react to them by controlling either charger.


## Independence from Charger Control

The ProtocolMonitor follows a fundamental design rule:

> **Monitoring must never become a requirement for charging operation.**

Therefore the monitor does not:

- grant SolarShare power
- select charging sources
- control either DC/DC converter
- acknowledge charger faults automatically
- change battery parameters
- perform MPPT
- provide timing required by either charger

Source and Assist must remain fully operational when the monitor is disconnected.


## Future Local Display

A local display is planned as a separate output function of the ProtocolMonitor.

The intended display is:

```text
EA W320-8K3
320 × 240
monochrome
```

The display is intended primarily for a structured text overview of:

- Source operating values
- Assist operating values
- communication state
- warning/error information
- diagnostic counters

No graphical history is currently required.


## Display Software Architecture

The display should not be integrated directly into the protocol parsing functions.

Preferred architecture:

```text
UART reception
      │
      ▼
Protocol parser
      │
      ▼
Decoded monitor state
      │
      ├────────► Serial output
      │
      ├────────► SD logging
      │
      └────────► W320 display
```

This allows the display code to be added, modified or disabled without affecting protocol reception.

The exact display GPIO assignment and hardware interface should only be added after the final interface has been implemented and tested.


## Display Power

The planned display hardware includes a physical:

```text
DisplayON
```

button.

The display itself is intended to be switchable while the ProtocolMonitor continues operating and logging in the background.

The final high-side power-control circuit and GPIO assignment belong in the Hardware documentation after practical verification.


## Important Before Use

When the Source/Assist protocol is modified, the ProtocolMonitor must be checked for corresponding changes to:

```text
Protocol version
Frame length
Payload length
Message types
Payload field order
Field scaling
State ranges
Warning flags
Error flags
Plausibility limits
Sequence handling
```

A protocol change should not be considered complete until all three firmware projects agree:

```text
BatterySourceCharger
BatteryAssistCharger
BatteryProtocolMonitor
```


## Related Documentation

Device overview:

[BatteryProtocolMonitor](../README.md)

System documentation:

- [System Architecture](../../Documentation/SystemArchitecture.md)
- [Charging Strategy](../../Documentation/ChargingStrategy.md)
- [Communication Protocol](../../Documentation/CommunicationProtocol.md)
- [Parameters](../../Documentation/Parameters.md)


## Development Status

The current ProtocolMonitor firmware provides the fundamental diagnostic functions required for development and observation of the DualBatterySolarCharger system.

Implemented functions include:

- independent Source and Assist UART reception
- fixed-frame protocol parsing
- CRC16-CCITT validation
- exact payload-length validation
- payload plausibility checking
- sequence monitoring
- request/response pair monitoring
- frame timeout detection
- header-noise detection
- rejected-frame classification
- reject queue
- separate `REJECT.CSV` logging
- diagnostic counters
- decoding of Source and Assist operating information

The planned W320 local display is an extension of this architecture and should use the already decoded monitor data rather than becoming part of the protocol parser.

The firmware therefore has one primary purpose:

> **Make the communication and interaction between Source and Assist observable without influencing it.**

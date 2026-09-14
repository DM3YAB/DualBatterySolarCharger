# Communication Protocol

## 1. Purpose

The **BatterySourceCharger** and **BatteryAssistCharger** communicate through a compact binary UART protocol.

The communication is used to exchange:

- battery voltage and current,
- available and safe source power,
- SolarShare power grant,
- battery capacity,
- charger and operating states,
- battery information,
- warning flags,
- error flags.

The **BatterySourceCharger** acts as communication master.

The **BatteryAssistCharger** responds to valid requests from the Source.

The **BatteryProtocolMonitor** passively observes both communication directions and does not participate in the communication.


## 2. Communication Principle

The normal communication cycle is:

```text
BatterySourceCharger                  BatteryAssistCharger

       MASTER_REQUEST
             ───────────────────────►
             Sequence = N

                                      Validate frame
                                      Process Source data

       CLIENT_RESPONSE
             ◄───────────────────────
             Sequence = N
```

The Source periodically transmits a `MASTER_REQUEST`.

The Assist validates the received frame and responds with a `CLIENT_RESPONSE` using the **same sequence number**.

The Source increments the sequence number for subsequent requests.

The sequence number is an unsigned 8-bit value and therefore wraps:

```text
0 → 1 → 2 → ... → 254 → 255 → 0
```


## 3. Protocol Constants

Current protocol constants:

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


## 4. Message Types

The following message types are defined:

| Message | Value | Current use |
|---|---:|---|
| `MASTER_REQUEST` | `0x0001` | Source → Assist |
| `CLIENT_RESPONSE` | `0x0002` | Assist → Source |
| `COMMAND` | `0x0003` | Reserved / not currently used |
| `COMMAND_RESPONSE` | `0x0004` | Reserved / not currently used |
| `DIAGNOSTIC` | `0x0005` | Reserved / not currently used |

The current charging system uses:

```text
0x0001 MASTER_REQUEST
0x0002 CLIENT_RESPONSE
```


## 5. Frame Structure

Every frame has a fixed length of **30 bytes**.

```text
Byte
 0       Start 1             0xA5
 1       Start 2             0x5A
 2       Protocol Version
 3       Message Type High
 4       Message Type Low
 5       Sequence
 6       Payload Length      21
 7       Payload[0]
 ...
27       Payload[20]
28       CRC High
29       CRC Low
```

As a compact representation:

```text
┌────┬────┬─────┬──────────┬─────┬─────┬──────────────────┬───────────┐
│ A5 │ 5A │ Ver │ Type U16 │ Seq │ Len │ Payload 21 bytes │ CRC16 U16 │
└────┴────┴─────┴──────────┴─────┴─────┴──────────────────┴───────────┘
  0    1    2      3..4       5     6        7..27           28..29
```

The payload length for the current protocol must be exactly:

`21`


## 6. Byte Order

Multi-byte values are transmitted **most significant byte first**.

Example:

```text
uint16_t value = 0x1234

Wire:
0x12 0x34
```

Therefore the protocol uses big-endian byte order for its 16-bit fields.


## 7. Signed Current Values

Battery current is transmitted as a signed 16-bit value.

The bit pattern is placed in the same two-byte big-endian representation as an unsigned 16-bit value and interpreted by the receiver as `int16_t`.

Example conceptually:

```text
Positive current:
 +123 → int16_t

Negative current:
 -123 → int16_t two's-complement representation
```

This allows both charging and reverse/discharging current to be represented.


## 8. CRC16

The protocol uses:

```text
CRC16-CCITT
Polynomial:     0x1021
Initial value:  0xFFFF
```

The CRC is calculated over bytes:

```text
Byte 2 ... Byte 27
```

Therefore the CRC includes:

```text
Protocol Version
Message Type
Sequence
Payload Length
Payload
```

The two start bytes are **not** included.

```text
A5 5A | Version ... Payload | CRC
─────   ───────────────────   ───
 not          CRC input
included
```

The CRC itself is transmitted:

```text
CRC High
CRC Low
```


# MASTER_REQUEST

## 9. Source → Assist

`MASTER_REQUEST` is transmitted by the BatterySourceCharger.

Message type:

`0x0001`

The payload has exactly 21 bytes.


## 10. MASTER_REQUEST Payload

| Payload bytes | Field | Type | Resolution |
|---|---|---|---|
| 0..1 | `batteryVoltage_10mV` | U16 | 10 mV |
| 2..3 | `batteryCurrent_10mA` | I16 | 10 mA |
| 4..5 | `sourcePower_W` | U16 | 1 W |
| 6..7 | `safeSourcePower_W` | U16 | 1 W |
| 8..9 | `grantedAssistPower_W` | U16 | 1 W |
| 10 | `batterySoc_percent` | U8 | 1 % |
| 11..12 | `sourceMode` | U16 | Enum |
| 13..14 | `mpptState` | U16 | Enum |
| 15..16 | `dcdcState` | U16 | Enum |
| 17..18 | `warningFlags` | U16 | Bit field |
| 19..20 | `errorFlags` | U16 | Bit field |

Total:

`21 bytes`


## 11. MASTER_REQUEST Battery Values

### `batteryVoltage_10mV`

24 V battery voltage.

Resolution:

`10 mV / bit`

Example:

```text
Value = 2534

2534 × 10 mV = 25.34 V
```


### `batteryCurrent_10mA`

Signed 24 V battery current.

Resolution:

`10 mA / bit`

Example:

```text
Value = 725
Current = 7.25 A
```

A negative value represents current flowing in the opposite direction according to the firmware current convention.


## 12. Source Power Values

### `sourcePower_W`

Measured/current source power in watts.

Resolution:

`1 W / bit`


### `safeSourcePower_W`

Power that the Source currently considers safely usable after its own source and operating limits have been considered.

Resolution:

`1 W / bit`


### `grantedAssistPower_W`

Maximum SolarShare power currently granted to the BatteryAssistCharger.

Resolution:

`1 W / bit`

The Assist must not interpret the 24 V bus voltage itself as permission to draw additional SolarShare power.

It must remain within this grant.

> **Current development status:** The protocol field is implemented, but the final capacity- and battery-state-dependent SolarShare allocation algorithm in the Source is still under development. The current Source firmware therefore does not yet provide the final dynamic grant behaviour.


## 13. Source Battery SOC

### `batterySoc_percent`

Estimated state of charge of the Source battery.

Range:

```text
0 ... 100 %
```

> **Current development status:** SOC calculation is not yet implemented in the current Source firmware. The field is reserved for this purpose and is currently transmitted as `0`.


## 14. Source Mode

`sourceMode` describes the currently selected Source input.

Current values:

| Value | Meaning |
|---:|---|
| 0 | `INPUT_SOURCE_NONE` |
| 1 | `INPUT_SOURCE_MAIN` |
| 2 | `INPUT_SOURCE_SOLAR` |
| 3 | `INPUT_SOURCE_HELP` |


## 15. MPPT State

`mpptState` describes the current Source solar/MPPT operating condition.

The current protocol accepts values:

```text
0 ... 7
```

Values `6` and `7` are used for the additional low-solar operating conditions:

| Value | Meaning |
|---:|---|
| 6 | `LOW_SOLAR` |
| 7 | `LOW_PAUSE` |

The remaining values correspond to the internal Source MPPT states.

The ProtocolMonitor deliberately accepts the complete range `0 ... 7`.


## 16. Source DC/DC State

The field formerly referred to as `powerManagerState` represents the actual Source DC/DC state and is therefore documented as:

`dcdcState`

Current values:

| Value | Meaning |
|---:|---|
| 0 | `DCDC_STATE_OFF` |
| 1 | `DCDC_STATE_START` |
| 2 | `DCDC_STATE_CHARGE_ON` |
| 3 | `DCDC_STATE_RAMP_CURRENT` |
| 4 | `DCDC_STATE_READY` |


## 17. Source Warning Flags

Current Source warning bits:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x0001` | Thermal derating |
| 1 | `0x0002` | Recovery unusually long |

Warning flags describe conditions where operation may continue.


## 18. Source Error Flags

Current Source error bits:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x0001` | DC/DC pre-charge failure |
| 1 | `0x0002` | Battery overvoltage |
| 2 | `0x0004` | Overtemperature |

Errors represent conditions requiring protection or intervention.


# CLIENT_RESPONSE

## 19. Assist → Source

`CLIENT_RESPONSE` is transmitted by the BatteryAssistCharger after receiving a valid `MASTER_REQUEST`.

Message type:

`0x0002`

The response uses the **same sequence number** as the corresponding request.

The payload has exactly 21 bytes.


## 20. CLIENT_RESPONSE Payload

| Payload bytes | Field | Type | Resolution |
|---|---|---|---|
| 0..1 | `batteryVoltage_10mV` | U16 | 10 mV |
| 2..3 | `batteryCurrent_10mA` | I16 | 10 mA |
| 4..5 | `requestedInputPower_W` | U16 | 1 W |
| 6..7 | `actualInputPower_W` | U16 | 1 W |
| 8 | `batterySoc_percent` | U8 | 1 % |
| 9..10 | `chargeState` | U16 | Enum |
| 11..12 | `operatingState` | U16 | Enum |
| 13..14 | `batteryCapacity_Ah` | U16 | 1 Ah |
| 15..16 | `batteryInfoCode` | U16 | Enum |
| 17..18 | `warningFlags` | U16 | Bit field |
| 19..20 | `errorFlags` | U16 | Bit field |

Total:

`21 bytes`


## 21. Assist Battery Values

### `batteryVoltage_10mV`

12 V battery voltage.

Resolution:

`10 mV / bit`

Example:

```text
Value = 1278

1278 × 10 mV = 12.78 V
```


### `batteryCurrent_10mA`

Signed 12 V battery current.

Resolution:

`10 mA / bit`


## 22. Assist Power Values

### `requestedInputPower_W`

Power requested by the Assist.

Resolution:

`1 W / bit`

> **Current development status:** This field is currently transmitted as `0`. It remains available for future power-management development.


### `actualInputPower_W`

Actual measured Assist input power.

Resolution:

`1 W / bit`

This gives the Source information about the power actually being consumed by the Assist.


## 23. Assist Battery SOC

### `batterySoc_percent`

Estimated state of charge of the 12 V battery.

Range:

```text
0 ... 100 %
```

> **Current development status:** SOC calculation is not yet implemented. The field is currently transmitted as `0`.


## 24. Assist Charge State

`chargeState` describes the current 12 V charging state.

The current protocol accepts:

```text
0 ... 4
```

The additional state introduced during development allows **Float** to be represented explicitly.

The exact symbolic state names are defined by the BatteryAssistCharger firmware.


## 25. Assist Operating State

`operatingState` describes the current operating mode of the BatteryAssistCharger.

It is separate from the charging state.

This allows the protocol to distinguish, for example, between the battery charging phase and the broader system operating condition.


## 26. Battery Capacity

### `batteryCapacity_Ah`

Configured capacity of the 12 V battery.

Type:

`uint16_t`

Resolution:

`1 Ah / bit`

Special value:

```text
0 = capacity unknown / not configured
```

The Assist owns this configuration parameter and reports it to the Source.

This avoids configuring the 12 V battery capacity independently in both controllers.

Example:

```text
batteryCapacity_Ah = 100

→ 12 V battery capacity = 100 Ah
```

The current Assist configuration uses:

`100 Ah`

and the value is marked `Fix!Me` because it must match the battery actually installed.

The protocol itself can technically represent:

```text
0 ... 65535 Ah
```

The current plausibility limit used for system diagnostics is:

`2000 Ah`

This plausibility limit is also considered a `Fix!Me` parameter.


## 27. Battery Information Code

### `batteryInfoCode`

This field describes the current battery observation or operating information.

It was formerly named `reserved`.

The current information codes are:

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

This field represents **current information**.

It should not be confused with the warning flags, which can represent persistent or latched warning history.


## 28. Assist Warning Flags

Current Assist warning bits:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x0001` | Low battery |
| 1 | `0x0002` | Fast discharge |
| 2 | `0x0004` | Thermal derating |

Warnings indicate abnormal or noteworthy conditions where operation may continue.


## 29. Assist Error Flags

Current Assist error bits:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x0001` | DC/DC pre-charge failure |
| 1 | `0x0002` | Battery overvoltage |
| 2 | `0x0004` | Overtemperature |

> **Implementation note:** The battery-overvoltage error bit is defined in the protocol, but the final 12 V battery-overvoltage threshold/protection behaviour has not yet been fully defined in the current Assist firmware.


# VALIDATION

## 30. Frame Validation

A received frame is only accepted when the relevant protocol checks succeed.

These include:

```text
Start bytes
    │
    ▼
Protocol version
    │
    ▼
Known message type
    │
    ▼
Payload length = 21
    │
    ▼
CRC valid
    │
    ▼
Payload plausible
    │
    ▼
Sequence valid where required
    │
    ▼
Accept frame
```


## 31. MASTER_REQUEST Plausibility

The Assist checks Source data for plausible ranges.

Current relevant limits include:

| Field | Accepted range |
|---|---:|
| Battery voltage | ≤ 36.00 V |
| Battery current | -50.00 ... +50.00 A |
| Source power | ≤ 1500 W |
| Safe source power | ≤ 1500 W |
| Granted Assist power | ≤ 500 W |
| Battery SOC | ≤ 100 % |
| Source mode | 0 ... 3 |
| MPPT state | 0 ... 7 |
| DC/DC state | 0 ... 4 |

These limits are protocol plausibility checks and should not be confused with the actual electrical operating limits of the charger.


## 32. CLIENT_RESPONSE Plausibility

The Source and ProtocolMonitor validate Assist data before treating it as valid system information.

Important current checks include:

```text
Battery SOC             ≤ 100 %
Charge state            0 ... 4
Battery capacity        0 ... 2000 Ah     Fix!Me
Battery information     0 ... 7
```

Additional electrical plausibility checks are implemented in the firmware.

Again, protocol plausibility limits are not charger operating limits.


## 33. Communication Timing

The current communication timing parameters are:

```text
Communication interval       1000 ms
RX partial-frame timeout       20 ms     Fix!Me
Communication link timeout   3000 ms     Fix!Me
```

The Source normally initiates one communication cycle per second.

A partial frame that does not complete within the RX timeout is rejected.

Loss of valid communication for the configured link timeout is treated as a communication failure.


## 34. Sequence Handling

The Source increments the request sequence number.

The Assist echoes the sequence number of the valid request.

Example:

```text
SOURCE                         ASSIST

MASTER_REQUEST Seq=41
       ───────────────────────►

CLIENT_RESPONSE Seq=41
       ◄───────────────────────


MASTER_REQUEST Seq=42
       ───────────────────────►

CLIENT_RESPONSE Seq=42
       ◄───────────────────────
```

A response with an unexpected sequence number can therefore be detected.

The ProtocolMonitor can also use the sequence progression to detect missing or unexpected communication.


## 35. Communication Failure and SolarShare

The Assist must not use stale Source information as authorization for normal SolarShare charging.

Therefore:

```text
Valid recent Source data
          +
grantedAssistPower_W > 0
          +
no blocking Source error
          │
          ▼
SolarShare may be used
```

If valid Source communication is lost, the previous grant must not remain indefinitely active.

Local battery and converter protection remains autonomous.


# BATTERY PROTOCOL MONITOR

## 36. Passive Monitoring

The BatteryProtocolMonitor receives both communication directions independently.

```text
BatterySourceCharger TX ───────┐
                               ├──► BatteryProtocolMonitor
BatteryAssistCharger TX ───────┘
```

It does not transmit commands to either charger and is not required for normal system operation.


## 37. Monitor Diagnostics

The monitor checks and records communication problems including:

- protocol version errors,
- unknown message types,
- payload-length errors,
- CRC errors,
- implausible payload data,
- incomplete-frame timeouts,
- sequence anomalies,
- communication-pair timeouts.

Rejected frames can be stored separately for later investigation.


## 38. Rejected Frame Categories

Current rejected-frame categories include:

```text
VERSION_ERROR
UNKNOWN_TYPE
LENGTH_ERROR
CRC_ERROR
DATA_IMPLAUSIBLE
FRAME_TIMEOUT
```

The monitor can preserve the raw frame data together with the rejection reason.

This is useful for distinguishing electrical communication problems from valid frames containing implausible data.


# PROTOCOL SUMMARY

## 39. Complete Exchange

```text
SOURCE                                                   ASSIST
24 V system                                              12 V system

batteryVoltage_10mV ───────────────────────────────────►
batteryCurrent_10mA
sourcePower_W
safeSourcePower_W
grantedAssistPower_W
batterySoc_percent
sourceMode
mpptState
dcdcState
warningFlags
errorFlags

                         MASTER_REQUEST
                         Sequence = N


                    ◄─────────────────────────────────── batteryVoltage_10mV
                                                        batteryCurrent_10mA
                                                        requestedInputPower_W
                                                        actualInputPower_W
                                                        batterySoc_percent
                                                        chargeState
                                                        operatingState
                                                        batteryCapacity_Ah
                                                        batteryInfoCode
                                                        warningFlags
                                                        errorFlags

                         CLIENT_RESPONSE
                         Sequence = N
```


## 40. Design Principle

The communication protocol follows a simple principle:

```text
Source:
    Reports the condition of the 24 V / solar system.
    Defines how much SolarShare power may be used.

Assist:
    Reports the condition and capacity of the 12 V system.
    Uses only a valid Source grant for normal SolarShare.

Monitor:
    Observes both directions.
    Validates and records communication.
    Never participates in charging control.
```

The protocol is deliberately small and fixed-length so that it remains easy to inspect, debug and implement on the embedded controllers.

Fields reserved for future functionality are retained where useful, but functionality that is not yet active is explicitly documented as such.

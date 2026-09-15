# BatteryAssistCharger Firmware

## Overview

This directory contains the firmware for the **BatteryAssistCharger**, the 12 V battery controller of the DualBatterySolarCharger system.

Firmware file:

```text
BatteryAssistCharger.ino
```

The firmware is responsible for the local control of the 12 V charger, including:

- 12 V battery charging
- input-source selection
- DC/DC converter control
- pre-charge verification
- current and voltage monitoring
- thermal management
- battery observation
- external charging detection
- reverse-current handling
- limited 12 V Battery Support from the 24 V system
- SolarShare power handling
- communication with the BatterySourceCharger


## Development Environment

The firmware is intended for:

```text
MCU:          ATmega328P
Framework:    Arduino
Development:  VisualMicro / Microchip Studio
```

The source is kept as a single `.ino` project with a deliberately flat program structure.

The general organization is:

```text
Parameters / constants
        │
        ▼
setup()
        │
        ▼
loop()
        │
        ▼
Functions grouped by task
```

Functions are grouped by purpose rather than collecting large parts of the program into monolithic functions.

This is intended to keep the firmware understandable and maintainable even after long periods without working on the project.


## Configuration

Important operating parameters are located near the beginning of the firmware and grouped by function.

Typical parameter groups include:

- battery charging
- input sources
- DC/DC converter
- pre-charge
- reverse current
- battery observation
- Battery Support
- thermal management
- communication

Parameters that still require practical verification are marked directly in the source with:

`Fix!Me`

These markers are intentional and should not be removed simply because the firmware compiles or appears to operate correctly.


## Power Inputs

The firmware distinguishes three input sources:

```text
HIGH_POWER
    Main / external DC source

MID_POWER
    24 V battery bus
    SolarShare / Battery Support

LOW_POWER
    Auxiliary low-power source
```

Normal source priority:

```text
HIGH_POWER → MID_POWER → LOW_POWER
```

`MID_POWER` must not be interpreted as a direct solar-panel input.

Solar MPPT is performed exclusively by the BatterySourceCharger.


## 12 V Charging

The current battery strategy distinguishes between:

```text
Battery sufficiently charged
    → no charging

Battery moderately discharged
    → Float maintenance

Battery requires complete charging
    → Bulk
    → Absorption
    → Float
```

Current principal values include:

| Parameter | Value |
|---|---:|
| Full-charge decision | ≤ 12.6 V |
| Float-maintenance decision | < 12.8 V |
| Absorption voltage | 14.2 V |
| Float voltage | 13.2 V |
| Minimum absorption time | 15 min |
| Maximum absorption time | 3 h |
| Full-current threshold | 100 mA |

Parameters requiring practical verification are marked `Fix!Me` in the firmware.


## SolarShare

During SolarShare operation, the Assist receives its permitted power budget from the BatterySourceCharger:

```text
grantedAssistPower_W
```

The Assist converts this power budget into an appropriate charging-current limit while respecting its own:

- current limits,
- DC/DC power limit,
- thermal limits,
- battery charging state,
- protection conditions.

Normal SolarShare requires recent valid communication from the Source.

Stale Source data must not continue to authorize SolarShare.

> **Current development status:** The Assist-side handling of `grantedAssistPower_W` is implemented. The final dynamic SolarShare allocation algorithm in the BatterySourceCharger is still under development.


## Battery Support

Battery Support is separate from normal SolarShare.

It allows a small, deliberately limited amount of energy to be transferred from the 24 V battery system when the 12 V battery approaches damaging undervoltage.

Current provisional values include:

```text
12 V support start       11.8 V
12 V support stop        12.4 V

24 V support enable      24.5 V
24 V support stop        24.0 V

Support power             8 W
Maximum support current 800 mA
```

These values are marked `Fix!Me`.

Battery Support is not intended as normal charging from the 24 V battery.


## Battery Capacity

The Assist owns the configuration of the 12 V battery capacity.

Current configuration:

```text
100 Ah    Fix!Me
```

The value is transmitted to the BatterySourceCharger as:

```text
batteryCapacity_Ah
```

If the 12 V battery is replaced by a battery with a different capacity, this parameter must be updated in the Assist firmware.


## DC/DC Control

The firmware controls the DC/DC output through the DAC.

Current converter calibration:

```text
DCDC_DAC_OFFSET_mV       16150
DCDC_DAC_SLOPE_uV_CODE    1303

DCDC_VOUT_MIN_mV         10500
DCDC_VOUT_MAX_mV         16150
```

These values describe the currently used Assist DC/DC hardware and must not be changed without checking the actual converter calibration.


## Pre-Charge

Before connecting the DC/DC converter to the battery, the firmware checks that the converter output can be controlled.

Important current parameters include:

```text
DCDC_TEST_MIN_OUTPUT_mV          10000
DCDC_TEST_RAISE_OFFSET_mV           80
DCDC_TEST_RISE_MIN_mV               40
DCDC_TEST_MATCH_TOL_mV               20
DCDC_CONNECT_OFFSET_mV               30
DCDC_CONNECT_SETTLE_MS              300
DCDC_TEST_STABLE_COUNT                3
DCDC_TEST_PHASE_TIMEOUT_MS         5000
```

The basic sequence is:

```text
Battery disconnected
        │
        ▼
Measure battery
        │
        ▼
Match DC/DC output
        │
        ▼
Raise output by 80 mV
        │
        ▼
Verify at least 40 mV rise
        │
        ▼
Set connection level
battery + 30 mV
        │
        ▼
Connect battery
```


## Reverse Current

Reverse current may occur when another charger raises the 12 V battery voltage above the local DC/DC output.

Current parameters:

```text
Reverse-current threshold     -50 mA
Persistence time                4 s
Retry interval                 60 s    Fix!Me
```

Persistent reverse current causes the Assist to disconnect its own DC/DC output and retry later.

Reverse current itself is treated as an operating condition rather than automatically as a charger fault.


## Current Limits

Current configured charge-current values:

| Input | Setpoint | Maximum |
|---|---:|---:|
| HIGH_POWER | 5 A | 9 A |
| MID_POWER | 5 A | 9 A |
| LOW_POWER | 2 A | 2 A |

Current configured input-current limits:

| Input | Maximum |
|---|---:|
| HIGH_POWER | 9 A |
| MID_POWER | 9 A |
| LOW_POWER | 2 A |

These are software operating limits and not absolute hardware ratings.


## DC/DC Power and Thermal Management

Current configured continuous DC/DC power limit:

```text
350 W    Fix!Me
```

The firmware monitors:

- MOSFET / power-switch temperature
- DC/DC inductor temperature

The hotter sensor determines the thermal limitation.

Current provisional derating:

| Temperature | Maximum power |
|---|---:|
| ≤ 65 °C | 350 W |
| 70 °C | 260 W |
| 75 °C | 175 W |
| 80 °C | 100 W |
| ≥ 85 °C | Charging OFF |

Restart:

```text
Both temperatures ≤ 55 °C    Fix!Me
```

The fan operates while charging and continues running after charging until both monitored temperatures have fallen to approximately:

```text
35 °C
```


## Battery Observation

The firmware observes the 12 V battery independently of the normal charging state.

Current battery information codes include:

```text
NORMAL
LOW
LOAD_ACTIVE
RECOVERY
FAST_DISCHARGE
SUPPORT_REQUEST
CHECK_BATTERY
EXTERNAL_CHARGE
```

The observation system is used to distinguish current battery behaviour from warnings and errors.


## Communication

The BatteryAssistCharger communicates with the BatterySourceCharger using the system binary UART protocol.

The Source is the communication master.

```text
BatterySourceCharger                  BatteryAssistCharger

       MASTER_REQUEST
             ───────────────────────►

                                      Validate request
                                      Process Source data

       CLIENT_RESPONSE
             ◄───────────────────────
             same sequence number
```

Current timing parameters:

```text
Communication interval       1000 ms
RX partial-frame timeout       20 ms    Fix!Me
Communication link timeout   3000 ms    Fix!Me
```

The Assist only responds to a valid `MASTER_REQUEST`.


## Protocol Information Sent by the Assist

The current `CLIENT_RESPONSE` includes:

```text
batteryVoltage_10mV
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
```

The complete protocol definition is documented in:

[Communication Protocol](../../Documentation/CommunicationProtocol.md)


## Warnings and Errors

Current warning flags:

```text
LOW_BATTERY
FAST_DISCHARGE
THERMAL_DERATING
```

Current defined error flags:

```text
DCDC_PRECHARGE
BATTERY_OVERVOLTAGE
OVERTEMPERATURE
```

The firmware distinguishes:

```text
STATUS
    current operating information

WARNING
    abnormal condition where operation may continue

ERROR
    condition requiring protection or intervention
```

> **Development note:** The `BATTERY_OVERVOLTAGE` error bit is defined, but the final 12 V overvoltage threshold and protection behaviour have not yet been fully defined.


## Important Before Use

Before using the firmware on another installation, check at least:

```text
Battery capacity
Battery charging thresholds
Input-current limits
DC/DC calibration
DC/DC power limit
Temperature limits
Battery Support thresholds
Communication timing
```

All `Fix!Me` markers should be reviewed for the actual hardware and battery installation.


## Related Documentation

Device overview:

[BatteryAssistCharger](../README.md)

System documentation:

- [System Architecture](../../Documentation/SystemArchitecture.md)
- [Charging Strategy](../../Documentation/ChargingStrategy.md)
- [Communication Protocol](../../Documentation/CommunicationProtocol.md)
- [Parameters](../../Documentation/Parameters.md)


## Development Status

The firmware contains the current BatteryAssistCharger implementation for the DualBatterySolarCharger system.

Implemented functions include:

- 12 V battery charging
- Float maintenance strategy
- source selection
- protected DC/DC connection
- reverse-current handling
- battery observation
- external charging detection
- limited Battery Support
- thermal management
- Source/Assist communication
- Assist-side SolarShare power limiting
- battery-capacity reporting

The principal system-level function still under development is the final dynamic SolarShare allocation performed by the BatterySourceCharger.

The firmware should therefore be considered part of an actively developed system, with provisional values explicitly identified by `Fix!Me`.

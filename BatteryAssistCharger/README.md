# BatteryAssistCharger

## Overview

The **BatteryAssistCharger** is the 12 V battery charger of the **DualBatterySolarCharger** system.

It is responsible for the local management, charging and protection of the 12 V battery.

The charger can operate from three different DC power sources:

- an external high-power DC source,
- the 24 V battery bus of the BatterySourceCharger system,
- an auxiliary low-power DC source.

The connection to the 24 V battery bus allows available solar energy to be transferred from the 24 V system to the 12 V battery through **SolarShare**.

The same electrical connection can also provide a small, deliberately limited amount of energy from the 24 V battery when the 12 V battery approaches damaging undervoltage.

The BatteryAssistCharger does **not** perform solar MPPT.

Solar MPPT and SolarShare power allocation are responsibilities of the BatterySourceCharger.


## Role in the System

```text
                       External DC
                       HIGH_POWER
                           │
                           ▼
                   ┌─────────────────────┐
24 V Battery Bus ─►│ BatteryAssistCharger│
   MID_POWER       │                     │
                   │ 12 V charge control │
 Auxiliary DC ────►│ Battery protection  │
   LOW_POWER       │ DC/DC control       │
                   └──────────┬──────────┘
                              │
                              ▼
                        ┌─────────────┐
                        │12 V Battery │
                        └─────────────┘
```

The normal input-source priority is:

```text
HIGH_POWER → MID_POWER → LOW_POWER
```

The presence of an input source does not automatically initiate a complete charging cycle.

The Assist first evaluates the condition of the 12 V battery and determines which charging strategy is appropriate.


## Power Inputs

### HIGH_POWER

`HIGH_POWER` is the Main / external DC input.

It can charge the 12 V battery independently of the solar system.

When available, this source has the highest normal priority.


### MID_POWER

`MID_POWER` is connected to the **24 V battery bus**.

It has two different functions:

```text
Normal operation:
    SolarShare

Exceptional maintenance operation:
    Limited 12 V Battery Support
```

During normal SolarShare operation, the Assist may only use the power granted by the BatterySourceCharger.

The presence of 24 V on this input alone is **not** permission to draw arbitrary charging power.


### LOW_POWER

`LOW_POWER` is an auxiliary low-power DC input.

It has the lowest normal source priority.


## 12 V Charging Strategy

The BatteryAssistCharger is designed to avoid unnecessary complete charging cycles, particularly during long periods when the vehicle is not being used.

The present battery decision is:

| Resting battery voltage | Action |
|---|---|
| **≥ 12.8 V** | No charging required |
| **< 12.8 V** | Float maintenance charging |
| **≤ 12.6 V** | Complete charging cycle |

The 12.8 V and 12.6 V decision thresholds are currently marked `Fix!Me` and require practical verification.


## Full Charge Cycle

When a complete charge is required, the sequence is:

```text
Bulk
  │
  ▼
Absorption
  │
  ▼
Float
```

Current principal charging values:

| Parameter | Value |
|---|---:|
| Absorption voltage | 14.2 V |
| Float voltage | 13.2 V |
| Minimum absorption time | 15 min |
| Maximum absorption time | 3 h |
| Full-current threshold | 100 mA |

The full-current threshold is marked `Fix!Me`.

The maximum absorption time prevents permanent Absorption operation when an external load prevents the measured battery current from falling below the normal completion threshold.


## SolarShare

SolarShare transfers part of the currently available solar energy from the 24 V system to the 12 V battery.

```text
Solar Panels
     │
     ▼
BatterySourceCharger
     │
     │ MPPT
     ▼
24 V Battery / Bus
     │
     ▼
BatteryAssistCharger
     │
     ▼
12 V Battery
```

The BatterySourceCharger determines how much solar power may be used by the Assist and communicates this value as:

`grantedAssistPower_W`

The Assist treats this value as a maximum permitted power budget.

It converts the granted power into a suitable charging-current limit while still respecting:

- battery charge-current limits,
- input-current limits,
- DC/DC power limits,
- thermal limits,
- battery charging state,
- local protection conditions.

The Assist deliberately performs **no Perturb-and-Observe or other MPPT search** on the 24 V bus.

This prevents the Assist from interfering with the solar MPPT performed by the BatterySourceCharger.

> **Development status:** The protocol support for `grantedAssistPower_W` is implemented. The final battery-capacity- and battery-state-dependent allocation algorithm in the BatterySourceCharger is still under development.


## Battery Capacity

The BatteryAssistCharger owns the configuration of its own 12 V battery capacity.

The current configured value is:

`100 Ah`

This value is marked:

`Fix!Me`

and must be adapted to the battery actually installed.

The Assist reports the configured capacity to the BatterySourceCharger through:

`batteryCapacity_Ah`

The Source therefore does not require a separately duplicated configuration of the 12 V battery capacity.

If the 12 V battery is replaced by a battery with a different capacity, the configuration is changed in the Assist firmware.


## 12 V Battery Support

Battery Support is deliberately separate from normal SolarShare operation.

During long periods without sufficient solar energy or another charging source, permanent small loads may slowly discharge the 12 V battery.

The Assist can then use a small amount of energy from the 24 V battery to prevent damaging 12 V undervoltage.

```text
24 V Battery
     │
     ▼
 MID_POWER
     │
     ▼
BatteryAssistCharger
     │
     ▼
12 V Battery
```

Current provisional parameters are:

| Parameter | Value |
|---|---:|
| 12 V support start | 11.8 V |
| 12 V support stop | 12.4 V |
| 24 V support enable | 24.5 V |
| 24 V support stop | 24.0 V |
| Support power | 8 W |
| Maximum support current | 800 mA |

These values are marked `Fix!Me`.

The objective is not to normally charge the 12 V battery from stored 24 V battery energy.

The objective is:

> **Prevent damaging deep discharge of the 12 V battery without unnecessarily discharging the 24 V battery.**


## Battery Observation

The Assist observes the behaviour of the 12 V battery independently of the normal charging state.

The firmware includes observation of conditions such as:

- low battery voltage,
- active load,
- recovery,
- unusually fast discharge,
- support request,
- battery requiring attention,
- external charging.

Battery observation information is communicated to the Source as:

`batteryInfoCode`

Current information codes are:

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

This information describes the current battery condition and is separate from warning and error flags.


## External 12 V Charging

The 12 V battery may also be charged by an external charger or vehicle charging system.

The Assist observes this condition instead of assuming that all measured battery current originates from its own DC/DC converter.

External charging is therefore an operating condition rather than an error.

If another charger raises the battery voltage above the Assist DC/DC output, reverse current may occur.

Persistent reverse current causes the Assist to disconnect its own DC/DC output and retry later.

Current reverse-current parameters include:

```text
Detection threshold       -50 mA
Persistence time            4 s
Retry time                 60 s    Fix!Me
```


## DC/DC Pre-Charge

Before connecting the DC/DC converter directly to the battery, the Assist verifies that the converter output can be controlled.

The sequence is approximately:

```text
Battery connection open
        │
        ▼
Measure battery voltage
        │
        ▼
Match DC/DC output
        │
        ▼
Raise output by ~80 mV
        │
        ▼
Verify ≥40 mV response
        │
        ▼
Set output ~30 mV
above battery voltage
        │
        ▼
Connect battery
```

This test reduces the risk of connecting an incorrectly controlled DC/DC output directly to the battery.


## Current and Power Limits

Current configured charge-current values are:

| Input | Setpoint | Maximum |
|---|---:|---:|
| HIGH_POWER | 5 A | 9 A |
| MID_POWER | 5 A | 9 A |
| LOW_POWER | 2 A | 2 A |

Current input-current limits are:

| Input | Maximum |
|---|---:|
| HIGH_POWER | 9 A |
| MID_POWER | 9 A |
| LOW_POWER | 2 A |

The configured continuous DC/DC power limit is:

`350 W`

This value is marked `Fix!Me`.

These are software operating limits and must not be interpreted as verified absolute hardware ratings.


## Thermal Management

The Assist monitors two temperature locations:

- MOSFET / power-switch area,
- DC/DC inductor.

The hotter sensor determines the thermal power limit.

Current provisional thermal strategy:

| Temperature | Maximum DC/DC power |
|---|---:|
| ≤ 65 °C | 350 W |
| 70 °C | 260 W |
| 75 °C | 175 W |
| 80 °C | 100 W |
| ≥ 85 °C | Charging OFF |

Restart is permitted when both monitored temperatures have fallen to approximately:

`55 °C`

These values are marked `Fix!Me`.

The cooling fan runs while charging and remains active after charging until both monitored temperatures have fallen to approximately:

`35 °C`


## Communication

The BatteryAssistCharger communicates with the BatterySourceCharger through the system UART protocol.

The Source acts as communication master.

```text
BatterySourceCharger                  BatteryAssistCharger

       MASTER_REQUEST
             ───────────────────────►

                                      Validate
                                      Process

       CLIENT_RESPONSE
             ◄───────────────────────
             same sequence number
```

The Assist only responds to a valid Source request.

Important information received from the Source includes:

- 24 V battery voltage and current,
- source power,
- safe source power,
- SolarShare grant,
- Source operating states,
- Source warnings and errors.

Important information returned by the Assist includes:

- 12 V battery voltage and current,
- actual input power,
- battery capacity,
- charge state,
- operating state,
- battery information,
- warnings and errors.

Normal SolarShare requires recent valid Source communication.

Stale Source information must not continue to authorize SolarShare indefinitely.


## Warnings and Errors

Current Assist warning flags include:

| Mask | Meaning |
|---:|---|
| `0x0001` | Low battery |
| `0x0002` | Fast discharge |
| `0x0004` | Thermal derating |

Current defined error flags include:

| Mask | Meaning |
|---:|---|
| `0x0001` | DC/DC pre-charge failure |
| `0x0002` | Battery overvoltage |
| `0x0004` | Overtemperature |

> **Development note:** The battery-overvoltage error bit is defined, but the final 12 V overvoltage threshold/protection behaviour has not yet been fully defined in the current firmware.


## Firmware

The current firmware is located in:

```text
BatteryAssistCharger/
└── Firmware/
    └── BatteryAssistCharger.ino
```

The firmware is intended for an **ATmega328P / Arduino-compatible environment** and was developed using VisualMicro / Microchip Studio.


## Hardware

Hardware documentation belongs in:

```text
BatteryAssistCharger/
└── Hardware/
```

This directory is intended for the current verified hardware documentation, including where available:

- schematic,
- PCB documentation,
- component placement,
- BOM,
- hardware revision information,
- photographs,
- measurement notes.

Only the current applicable hardware revision should be treated as authoritative.


## Related Documentation

System-level documentation is available in:

- [System Architecture](../Documentation/SystemArchitecture.md)
- [Charging Strategy](../Documentation/ChargingStrategy.md)
- [Communication Protocol](../Documentation/CommunicationProtocol.md)
- [Parameters](../Documentation/Parameters.md)


## Development Status

The BatteryAssistCharger is an operational part of the DualBatterySolarCharger system, but several parameters still require practical verification in the final installation.

Such parameters are marked directly in the firmware with:

`Fix!Me`

The most important remaining system-level development item is the final SolarShare allocation algorithm in the BatterySourceCharger.

Until that algorithm is active, the protocol and Assist-side handling of `grantedAssistPower_W` are prepared, but the Source does not yet provide the intended dynamic SolarShare power allocation.

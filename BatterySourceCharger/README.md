# BatterySourceCharger

## Overview

The **BatterySourceCharger** is the 24 V battery charger and primary solar-energy controller of the **DualBatterySolarCharger** system.

It is responsible for:

- charging and protecting the 24 V battery,
- acquiring solar energy using MPPT,
- charging from external DC sources,
- controlling the DC/DC converter,
- monitoring voltage, current and temperature,
- recovering a deeply discharged 24 V battery after BMS shutdown,
- communicating with the BatteryAssistCharger,
- determining how much solar power may be shared with the 12 V system.

The BatterySourceCharger is the **only controller in the system that performs solar MPPT**.

The BatteryAssistCharger receives solar-derived energy through the 24 V battery bus and does not perform its own MPPT.


## Role in the System

```text
        Solar Panels                    External DC
             │                         Main / Max InputDC
             │                                │
             ▼                                ▼
        ┌──────────────────────────────────────────┐
        │          BatterySourceCharger            │
        │                                          │
        │  Solar MPPT                              │
        │  External DC charging                    │
        │  Input source management                 │
        │  24 V battery charging                   │
        │  Power / SolarShare management           │
        └────────────────────┬─────────────────────┘
                             │
                             ▼
                       ┌─────────────┐
                       │24 V Battery │
                       └──────┬──────┘
                              │
                              │  24 V battery bus
                              │
                              ▼
                    BatteryAssistCharger
                              │
                              ▼
                        12 V Battery
```

The Source has two principal tasks that must remain conceptually separate:

```text
Battery charging
    → safely charge and protect the 24 V battery

Solar energy management
    → acquire available solar power
    → determine how much may be shared
```


## Power Inputs

The BatterySourceCharger distinguishes three input sources:

```text
MAIN
SOLAR
HELP
```

Conceptually:

| Input | Function |
|---|---|
| `MAIN` | Main / Max external DC source |
| `SOLAR` | Solar input with MPPT |
| `HELP` | Auxiliary low-power source |

Input-source selection is performed locally by the Source.

The active source determines not only how the DC/DC converter is controlled, but also whether SolarShare is permitted.


## Solar Input

The `SOLAR` input is the energy source used for MPPT operation.

The solar panel system used during development has approximately:

| Parameter | Typical value |
|---|---:|
| Open-circuit voltage | ~45 V |
| Typical MPP voltage | ~36 V |
| Minimum useful voltage | ~24 V |

Current important thresholds include:

```text
Solar valid threshold        24.0 V     Fix!Me
Fast-drop threshold          28.0 V     Fix!Me
```

The solar input is considered usable above the configured minimum voltage.

If the solar voltage falls into the fast-drop region, the Source rapidly reduces load to prevent collapse of a weak solar source.


## External DC Charging

The Source can also charge the 24 V battery from its external Main / Max InputDC.

```text
External DC
Main / Max InputDC
       │
       ▼
BatterySourceCharger
       │
       ▼
24 V Battery
```

This input does not require MPPT.

Charging remains subject to:

- input-current limits,
- battery-current limits,
- DC/DC power limits,
- battery-voltage protection,
- thermal limits,
- DC/DC protection.

External DC power connected to the Source is **not automatically allocated to SolarShare**.

SolarShare is associated with energy acquired from the solar input.


## Solar MPPT

The BatterySourceCharger performs the solar MPPT function for the complete DualBatterySolarCharger system.

The MPPT controller observes solar voltage and current and calculates:

```text
Psolar = Usolar × Isolar
```

The current implementation uses an adaptive **Perturb-and-Observe** strategy.

Instead of directly commanding a solar power value, the controller changes the requested charging load and observes the resulting change in solar power.

Conceptually:

```text
Measure Usolar and Isolar
          │
          ▼
    Calculate Psolar
          │
          ▼
 Compare with previous Psolar
          │
          ▼
 Increase or decrease load
          │
          ▼
 Measure new operating point
```

The Source therefore determines the useful operating point of the solar panels while the battery charger and protection logic determine how much of that power may actually be used.


## Adaptive MPPT

The MPPT does not use a single fixed perturbation speed.

Current timing values are:

| MPPT mode | Interval |
|---|---:|
| Fast | 200 ms |
| Normal | 500 ms |
| Slow | 1000 ms |

These timing values are marked `Fix!Me`.

Current perturbation steps include:

| Parameter | Value |
|---|---:|
| Fine step | 50 mA |
| Normal step | 100 mA |
| Large step | 250 mA |
| Fast down-step | 1000 mA |

Current power thresholds include:

```text
Power hysteresis             0.2 W
Fine-power difference        0.5 W
Large-power difference       2.0 W
```

The controller observes the behaviour over a 10-cycle window.

Current classification uses:

```text
Dynamic cycles for FAST       3
Stable cycles for SLOW        7
```

The purpose of the adaptive timing is to react quickly to significant solar changes while avoiding unnecessary continuous large perturbations around a stable operating point.


## MPPT Startup

The MPPT does not immediately apply a large load after the DC/DC converter becomes ready.

The startup sequence begins with no MPPT charging current during pre-charge.

After the converter reaches the ready state, the initial probing sequence is approximately:

```text
0 mA
  │
  ▼
50 mA
  │
  ▼
100 mA
  │
  ▼
Normal adaptive P&O
```

This allows the solar source to be loaded progressively rather than applying an abrupt initial demand.


## Fast Solar Drop

The Source also contains a fast response for sudden solar-voltage collapse.

This check operates independently of the slower adaptive MPPT interval.

Current values:

```text
Fast-drop voltage          28.0 V     Fix!Me
Fast current reduction     1000 mA    Fix!Me
```

This allows the charger to unload the solar source rapidly when changing irradiation or other conditions cause the panel voltage to collapse.


## 24 V Battery Charging

The Source remains responsible for the 24 V battery regardless of which input source is active.

Charging power is limited by the most restrictive applicable condition:

```text
Requested charge current
          │
          ▼
Battery current limit
          │
          ▼
Input-source limit
          │
          ▼
DC/DC power limit
          │
          ▼
Thermal limit
          │
          ▼
Battery protection
          │
          ▼
Actual permitted charge current
```

Solar MPPT determines what the solar source can provide.

It does **not** override battery or converter protection.


## 24 V Battery Voltage Limits

Important current 24 V battery values include:

```text
Absorption voltage          28.4 V
Battery hard stop           28.2 V
BMS low-voltage region      21.0 V
```

The Source includes special recovery behaviour below the normal battery operating range because the external 24 V battery BMS may disconnect the battery after deep undervoltage.


## 24 V Battery Recovery

A deeply discharged 24 V battery may present a special condition:

```text
Battery deeply discharged
          │
          ▼
External BMS disconnects
          │
          ▼
Normal charger connection
may no longer be possible
```

The Source therefore provides a controlled recovery procedure.

Current recovery parameters include:

| Parameter | Value | Status |
|---|---:|---|
| BMS low-voltage region | 21.0 V | current setting |
| Recovery target | 21.5 V | tested |
| Recovery wake threshold | 22.0 V | `Fix!Me` |
| Recovery ramp threshold | 23.0 V | `Fix!Me` |
| Initial recovery current | 100 mA | `Fix!Me` |
| Ramp interval | 30 s | `Fix!Me` |
| Long-recovery warning | 2 h | `Fix!Me` |

The **21.5 V recovery target has been practically tested**.

The staged current concept is:

```text
100 mA
  │
  ▼
250 mA
  │
  ▼
500 mA
  │
  ▼
1 A
  │
  ▼
2 A
  │
  ▼
3 A
  │
  ▼
4 A
  │
  ▼
5 A
```

The current stages remain provisional.

After severe undervoltage, only a very small current may initially flow even though the charger is providing recovery voltage.

This is not automatically considered a charger failure.

The external battery BMS may require time before the battery becomes fully available again.

A recovery lasting an unusually long time therefore produces a warning while recovery is allowed to continue.


## SolarShare

The Source is responsible for determining how much of the available solar power may be used by the BatteryAssistCharger.

The intended energy path is:

```text
Solar Panels
     │
     ▼
Source MPPT
     │
     ▼
Available Solar Power
     │
     ▼
Safe Solar Power
     │
     ├────────► 24 V battery
     │
     └────────► SolarShare
                    │
                    ▼
           grantedAssistPower_W
                    │
                    ▼
           BatteryAssistCharger
                    │
                    ▼
              12 V Battery
```

The Source communicates the permitted Assist power as:

`grantedAssistPower_W`

The Assist must remain within this power budget.

The purpose is to allow both battery systems to use available solar energy without allowing two independent controllers to compete for the same solar operating point.


## SolarShare Allocation

The intended SolarShare allocation can use information from both battery systems.

Relevant information includes:

- available safe solar power,
- 24 V battery condition,
- 24 V battery capacity,
- 12 V battery condition,
- 12 V battery capacity,
- current charging requirements,
- charger power limits.

The Assist reports its own configured battery capacity as:

`batteryCapacity_Ah`

The Source can therefore compare the requirements of battery banks with different capacities without duplicating the Assist battery configuration locally.

The intended strategy is not necessarily a fixed percentage split.

Conceptually:

```text
                    Safe Solar Power
                           │
                           ▼
                 ┌──────────────────┐
                 │ Power Allocation │
                 └────────┬─────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
       24 V battery              12 V battery
                                      │
                                      ▼
                           grantedAssistPower_W
```

> **Development status:** The protocol fields required for SolarShare are implemented, but the final battery-capacity- and battery-state-dependent allocation algorithm is still under development. In the current Source firmware, `grantedAssistPower_W` does not yet provide the intended dynamic allocation.


## Battery Capacity

The Source owns the configuration of its own 24 V battery capacity.

The Assist independently owns the configuration of the 12 V battery capacity and transmits it to the Source.

```text
BatterySourceCharger
        │
        └── 24 V battery capacity


BatteryAssistCharger
        │
        └── 12 V battery capacity
                 │
                 ▼
        batteryCapacity_Ah
                 │
                 ▼
        BatterySourceCharger
```

This avoids maintaining the same battery parameter in two separate firmware projects.

The capacity information is intended to become one input to the SolarShare allocation algorithm.


## DC/DC Pre-Charge

Before connecting the DC/DC converter directly to the 24 V battery, the Source verifies that its output can be controlled.

The general sequence is:

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
Raise output slightly
        │
        ▼
Verify converter response
        │
        ▼
Set connection voltage
slightly above battery
        │
        ▼
Connect battery
```

Current basic test values include:

```text
Test raise offset        +80 mV
Minimum expected rise     40 mV
Connection offset        +30 mV
```

The test reduces the risk of connecting an incorrectly controlled converter output directly to the battery.


## Reverse Current

Reverse current is not automatically treated as a fault.

It can occur when another charging source raises the battery voltage above the local DC/DC converter output.

Current parameters include:

```text
Reverse-current threshold     -50 mA
Persistence time                4 s
Retry interval                 60 s     Fix!Me
```

Persistent reverse current causes the Source to disconnect its own DC/DC output and retry later.

This allows another charger or energy source to dominate without the Source unnecessarily working against it.


## DC/DC Power Limit

The current configured continuous DC/DC power limit is:

`350 W`

Status:

`Fix!Me`

This is a software operating limit.

It must not be interpreted as an absolute verified hardware rating for every input/output voltage ratio.

Actual converter losses depend strongly on operating conditions.

In particular, step-up and step-down operation can result in significantly different losses and thermal behaviour.


## Thermal Management

The Source monitors two important temperature locations:

- MOSFET / power-switch area,
- DC/DC inductor.

The hotter measurement determines the thermal power limit.

Current provisional strategy:

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

The cooling fan operates whenever the DC/DC converter is actively charging.

After charging stops, the fan remains active until both monitored temperatures have fallen to approximately:

`35 °C`


## Communication

The BatterySourceCharger is the **communication master**.

It periodically sends:

`MASTER_REQUEST`

to the BatteryAssistCharger.

```text
BatterySourceCharger                  BatteryAssistCharger

       MASTER_REQUEST
             ───────────────────────►
             Sequence = N

                                      Validate
                                      Process

       CLIENT_RESPONSE
             ◄───────────────────────
             Sequence = N
```

The Source increments the sequence number for each new request.

The Assist responds using the same sequence number.

Current communication timing:

```text
Communication interval       1000 ms
RX partial-frame timeout       20 ms     Fix!Me
Communication link timeout   3000 ms     Fix!Me
```


## Information Sent to the Assist

The Source currently transmits:

- 24 V battery voltage,
- 24 V battery current,
- source power,
- safe source power,
- granted Assist power,
- battery SOC field,
- source mode,
- MPPT state,
- DC/DC state,
- warning flags,
- error flags.

The battery SOC field exists in the protocol but the final SOC calculation is not yet implemented.


## Information Received from the Assist

The Source receives:

- 12 V battery voltage,
- 12 V battery current,
- requested input power,
- actual input power,
- battery SOC field,
- charge state,
- operating state,
- 12 V battery capacity,
- battery information code,
- warning flags,
- error flags.

The Assist battery capacity is particularly important for the planned SolarShare allocation.

The `requestedInputPower_W` and SOC fields are available in the protocol but are not yet fully used by the present power-allocation logic.


## Communication Failure

The Source remains responsible for its own battery and converter even when communication with the Assist is unavailable.

A communication failure therefore does not disable:

- 24 V battery protection,
- solar MPPT,
- source selection,
- DC/DC protection,
- thermal protection,
- 24 V battery recovery.

However, communication-dependent system functions must not rely on stale Assist information.


## Warnings and Errors

Current Source warning flags include:

| Mask | Meaning |
|---:|---|
| `0x0001` | Thermal derating |
| `0x0002` | Recovery unusually long |

Current Source error flags include:

| Mask | Meaning |
|---:|---|
| `0x0001` | DC/DC pre-charge failure |
| `0x0002` | Battery overvoltage |
| `0x0004` | Overtemperature |

The firmware distinguishes between:

```text
STATUS
    Normal or noteworthy operating condition

WARNING
    Suspicious or abnormal condition
    Operation may continue

ERROR
    Protection condition requiring intervention
```


## Firmware

The current firmware is located in:

```text
BatterySourceCharger/
└── Firmware/
    └── BatterySourceCharger.ino
```

The firmware is intended for an **ATmega328P / Arduino-compatible environment** and was developed using VisualMicro / Microchip Studio.


## Hardware

Hardware documentation belongs in:

```text
BatterySourceCharger/
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

The 24 V hardware has undergone further development from the earlier charger designs.

Only the latest verified hardware revision should therefore be treated as authoritative.


## Related Documentation

System-level documentation is available in:

- [System Architecture](../Documentation/SystemArchitecture.md)
- [Charging Strategy](../Documentation/ChargingStrategy.md)
- [Communication Protocol](../Documentation/CommunicationProtocol.md)
- [Parameters](../Documentation/Parameters.md)


## Development Status

The BatterySourceCharger already contains the principal functions required for:

- 24 V battery charging,
- solar MPPT,
- external DC charging,
- protected DC/DC operation,
- thermal management,
- 24 V BMS recovery,
- communication with the BatteryAssistCharger.

Parameters requiring practical verification are marked directly in the firmware with:

`Fix!Me`

The principal remaining system-level development task is the final **SolarShare allocation algorithm**.

The communication protocol already provides the required information path:

```text
Assist battery condition
Assist battery capacity
          │
          ▼
BatterySourceCharger
          │
          ▼
SolarShare allocation
          │
          ▼
grantedAssistPower_W
```

The final allocation algorithm will determine how the safely available solar power is distributed between the 24 V and 12 V battery systems.

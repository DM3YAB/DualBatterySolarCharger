# BatterySourceCharger Firmware

## Overview

This directory contains the firmware for the **BatterySourceCharger**, the 24 V battery controller and solar MPPT controller of the DualBatterySolarCharger system.

Firmware file:

```text
BatterySourceCharger.ino
```

The firmware is responsible for the local control of the 24 V charger, including:

- 24 V battery charging
- input-source selection
- solar MPPT
- external DC charging
- DC/DC converter control
- pre-charge verification
- current and voltage monitoring
- thermal management
- reverse-current handling
- 24 V battery BMS recovery
- communication with the BatteryAssistCharger
- preparation and transmission of SolarShare information


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

- 24 V battery parameters
- input sources
- solar MPPT
- DC/DC converter
- pre-charge
- reverse current
- BMS recovery
- thermal management
- communication
- SolarShare

Parameters that still require practical verification are marked directly in the source with:

`Fix!Me`

These markers are intentional and should not be removed simply because the firmware compiles or appears to operate correctly.


## Power Inputs

The firmware distinguishes three input sources:

```text
MAIN
    Main / Max external DC source

SOLAR
    Solar input with MPPT

HELP
    Auxiliary low-power DC source
```

Input selection and charging control are performed locally by the BatterySourceCharger.

Only the `SOLAR` input participates in solar MPPT and SolarShare calculations.

External power connected to `MAIN` is used for charging the 24 V battery but is not automatically treated as solar energy available for SolarShare.


## Solar MPPT

The BatterySourceCharger is the only controller in the DualBatterySolarCharger system that performs solar MPPT.

The firmware measures:

```text
Usolar
Isolar
```

and calculates:

```text
Psolar = Usolar × Isolar
```

The MPPT algorithm uses an adaptive Perturb-and-Observe strategy.

The controlled variable is the requested charging load/current.

The algorithm changes the load and observes the resulting change in measured solar power.

Conceptually:

```text
Measure Usolar / Isolar
        │
        ▼
Calculate Psolar
        │
        ▼
Compare with previous power
        │
        ▼
Change requested load
        │
        ▼
Measure next operating point
```


## Solar Input Parameters

The solar system used during development has approximately:

```text
Open-circuit voltage       ~45 V
Typical MPP voltage        ~36 V
Minimum useful voltage     ~24 V
```

Current important thresholds include:

```text
Solar valid threshold       24.0 V    Fix!Me
Fast-drop threshold         28.0 V    Fix!Me
```

The 24 V threshold determines whether the solar source is considered usable.

The higher fast-drop threshold allows the charger to react before the solar source collapses completely.


## Adaptive MPPT Timing

The MPPT interval changes according to recent solar behaviour.

Current values:

| MPPT mode | Interval |
|---|---:|
| Fast | 200 ms |
| Normal | 500 ms |
| Slow | 1000 ms |

These values are marked `Fix!Me`.

Current perturbation steps include:

| Parameter | Value |
|---|---:|
| Fine step | 50 mA |
| Normal step | 100 mA |
| Large step | 250 mA |
| Fast down-step | 1000 mA |

Current power thresholds:

```text
Power hysteresis           0.2 W
Fine difference            0.5 W
Large difference           2.0 W
```

The current classification window contains:

```text
10 MPPT cycles
```

with:

```text
3 dynamic cycles → FAST
7 stable cycles  → SLOW
otherwise        → NORMAL
```

These tuning values are intended to be verified with the actual solar installation and are marked `Fix!Me` where applicable.


## MPPT Startup

During DC/DC pre-charge, the MPPT charging demand remains at zero.

After the converter reaches its ready state, the solar source is loaded progressively.

The initial sequence is approximately:

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

This avoids applying a large initial load before the available solar conditions are known.


## Fast Solar Drop

A sudden reduction in irradiation can cause the solar voltage to collapse faster than the normal MPPT interval can react.

The firmware therefore contains a separate fast-drop response.

Current values:

```text
Fast-drop voltage          28.0 V    Fix!Me
Fast current reduction     1000 mA   Fix!Me
```

The fast-drop check is independent of the normal adaptive MPPT interval.

Its purpose is to unload a weak solar source quickly and allow the panel voltage to recover.


## 24 V Battery Charging

The Source controls charging of the 24 V battery independently of whether the energy originates from solar or an external DC source.

The actual permitted charging current is determined by several simultaneous limits.

Conceptually:

```text
Requested current
       │
       ▼
Source-current limit
       │
       ▼
Battery-current limit
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
Permitted charging current
```

Solar MPPT determines how much power the solar source can provide.

It never overrides battery, converter or thermal protection.


## 24 V Battery Protection and BMS Recovery

The 24 V battery system uses an external BMS.

After severe undervoltage, the BMS may disconnect the battery.

The Source firmware therefore contains a special recovery mode rather than immediately treating the low-voltage condition as a permanent charger fault.

Current important values include:

```text
Battery hard stop             28.2 V
BMS low-voltage region        21.0 V

Recovery target               21.5 V
Recovery wake threshold       22.0 V    Fix!Me
Recovery ramp threshold       23.0 V    Fix!Me
```

The 21.5 V recovery target has been practically tested.


## BMS Recovery Current

Recovery begins with a deliberately small current.

Current provisional parameters include:

```text
Initial recovery current      100 mA    Fix!Me
Ramp interval                  30 s     Fix!Me
Long-recovery warning           2 h     Fix!Me
```

The staged current sequence is:

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

During recovery, only a few milliamperes may initially flow even though the charger is applying the recovery voltage.

This can be normal behaviour while the external battery BMS is still recovering.

A long recovery therefore generates a warning rather than immediately terminating the recovery procedure.


## DC/DC Control

The Source uses the 24 V DC/DC hardware calibration:

```text
DCDC_DAC_OFFSET_mV         30113
DCDC_DAC_SLOPE_uV_CODE      2404
```

The measured relationship is approximately:

```text
DCDC_mV = 30113 - 2.404 × DAC_Code
```

The hardware can physically reach a higher output voltage, but the software intentionally restricts the commanded operating range.

The configured software maximum is:

```text
DCDC_VOUT_MAX_mV           29000
```

This software limit must not be confused with the physical maximum capability of the converter.


## DC/DC Power Limit

The current configured continuous DC/DC power limit is:

```text
350 W    Fix!Me
```

This is a software operating limit and not an absolute hardware rating.

Converter losses depend strongly on the relationship between input and output voltage.

Step-up and step-down operation can therefore produce significantly different thermal conditions even at the same output power.


## Pre-Charge

Before connecting the DC/DC converter directly to the battery, the firmware verifies that its output can be controlled.

Current principal values include:

```text
Test raise offset          +80 mV
Minimum expected rise       40 mV
Connection offset          +30 mV
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
Raise output by ~80 mV
        │
        ▼
Verify ≥40 mV response
        │
        ▼
Set connection level
battery + ~30 mV
        │
        ▼
Connect battery
```

The pre-charge test reduces the risk of connecting an incorrectly controlled DC/DC output directly to the battery.


## Reverse Current

Reverse current is not automatically treated as an error.

It may occur when another charging source raises the battery voltage above the local DC/DC output.

Current parameters:

```text
Reverse-current threshold     -50 mA
Persistence time                4 s
Retry interval                 60 s    Fix!Me
```

If reverse current persists, the Source disconnects its own DC/DC output and retries later.

This allows another charging source to dominate without forcing the Source converter to work against it.


## Thermal Management

The firmware monitors two temperature locations:

```text
NTC1 → MOSFET / power-switch area
NTC2 → DC/DC inductor
```

The hotter sensor determines the thermal power limit.

Current provisional derating:

| Temperature | Maximum power |
|---|---:|
| ≤ 65 °C | 350 W |
| 70 °C | 260 W |
| 75 °C | 175 W |
| 80 °C | 100 W |
| ≥ 85 °C | Charging OFF |

Restart is permitted when both monitored temperatures have fallen to approximately:

```text
55 °C    Fix!Me
```

The fan operates while charging and continues running after charging until both monitored temperatures have fallen to approximately:

```text
35 °C
```

The thermal values require practical verification with the final hardware and installation.


## SolarShare

The Source is responsible for deciding how much solar-derived power may be used by the BatteryAssistCharger.

The protocol field is:

```text
grantedAssistPower_W
```

The intended power path is:

```text
Solar Panels
     │
     ▼
Source MPPT
     │
     ▼
Available / Safe Solar Power
     │
     ├────────► 24 V charging
     │
     └────────► SolarShare
                    │
                    ▼
          grantedAssistPower_W
                    │
                    ▼
           BatteryAssistCharger
```

SolarShare must remain separate from ordinary external DC charging.

Power entering the Source through `MAIN / Max InputDC` is not automatically made available as SolarShare.


## SolarShare Allocation Status

The communication infrastructure required for SolarShare is implemented.

The Source receives information from the Assist including:

```text
12 V battery voltage
12 V battery current
12 V battery capacity
charge state
operating state
battery information
warnings / errors
```

The intended future allocation can therefore consider:

```text
Available safe solar power
24 V battery condition
24 V battery capacity
12 V battery condition
12 V battery capacity
Current charging requirements
```

> **Current development status:** The final capacity- and battery-state-dependent SolarShare allocation algorithm is still under development.

The current firmware does not yet provide the intended dynamic SolarShare grant.

The `grantedAssistPower_W` field therefore remains prepared for this future function.


## Communication

The BatterySourceCharger is the communication master.

It periodically transmits:

```text
MASTER_REQUEST
```

The BatteryAssistCharger responds with:

```text
CLIENT_RESPONSE
```

using the same sequence number.

```text
BatterySourceCharger                  BatteryAssistCharger

MASTER_REQUEST
Sequence = N
       ─────────────────────────────►

                                      Validate
                                      Process

CLIENT_RESPONSE
Sequence = N
       ◄─────────────────────────────
```

The Source increments the sequence number from:

```text
0 ... 255
```

and then wraps back to zero.


## Communication Timing

Current parameters:

```text
Communication interval       1000 ms
RX partial-frame timeout       20 ms    Fix!Me
Communication link timeout   3000 ms    Fix!Me
```

The Source keeps diagnostic counters for communication problems in RAM.

These include conditions such as:

- CRC errors
- format errors
- implausible data
- timeouts
- sequence mismatches


## Protocol Information Sent by the Source

The current `MASTER_REQUEST` contains:

```text
batteryVoltage_10mV
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
```

The Source battery SOC field currently exists in the protocol but the final SOC calculation has not yet been implemented.


## Protocol Information Received from the Assist

The current `CLIENT_RESPONSE` contains:

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

The Assist owns the configuration of its 12 V battery capacity and sends it to the Source as:

```text
batteryCapacity_Ah
```

This avoids maintaining a duplicate 12 V battery-capacity parameter in the Source firmware.

The complete protocol definition is documented in:

[Communication Protocol](../../Documentation/CommunicationProtocol.md)


## Autonomous Operation

The BatterySourceCharger remains responsible for its own charger even if communication with the BatteryAssistCharger is unavailable.

Communication failure must not disable:

```text
24 V battery protection
Input-source selection
Solar MPPT
External DC charging
DC/DC protection
Thermal protection
BMS recovery
```

System-level functions that depend on Assist information must not continue using stale communication data.


## Warnings and Errors

Current Source warning flags:

```text
THERMAL_DERATING
RECOVERY_LONG
```

Current Source error flags:

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
    protection condition requiring intervention
```

BMS recovery itself is an operating condition and not automatically an error.


## Important Before Use

Before using the firmware on another installation, check at least:

```text
24 V battery parameters
Battery capacity
Input-current limits
Solar voltage thresholds
MPPT parameters
DC/DC calibration
DC/DC power limit
BMS recovery parameters
Temperature limits
Communication timing
```

All `Fix!Me` markers should be reviewed for the actual hardware, battery and solar installation.


## Related Documentation

Device overview:

[BatterySourceCharger](../README.md)

System documentation:

- [System Architecture](../../Documentation/SystemArchitecture.md)
- [Charging Strategy](../../Documentation/ChargingStrategy.md)
- [Communication Protocol](../../Documentation/CommunicationProtocol.md)
- [Parameters](../../Documentation/Parameters.md)


## Development Status

The firmware contains the current BatterySourceCharger implementation for the DualBatterySolarCharger system.

Implemented functions include:

- 24 V battery management
- solar MPPT
- external DC charging
- input-source selection
- protected DC/DC connection
- reverse-current handling
- thermal management
- 24 V BMS recovery
- communication with the BatteryAssistCharger
- transmission of solar and system operating information

The principal remaining system-level development task is the final dynamic **SolarShare allocation algorithm**.

The required communication path already exists:

```text
Solar power / Source state
          │
          │
          ├──────────────────────────┐
          │                          │
          ▼                          ▼
24 V battery information     Assist information
                             12 V battery capacity
                             12 V battery condition
          │                          │
          └────────────┬─────────────┘
                       ▼
              SolarShare allocation
                       │
                       ▼
             grantedAssistPower_W
```

Until this allocation algorithm is completed, the existing SolarShare protocol fields should be regarded as prepared infrastructure rather than a completed power-distribution strategy.

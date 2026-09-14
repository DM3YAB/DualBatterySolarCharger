# Parameters

## 1. Purpose

This document summarizes the most important configurable parameters of the **DualBatterySolarCharger**.

It is not intended to duplicate every firmware constant.

Instead, it documents the parameters that significantly influence:

- battery charging behaviour,
- source selection,
- SolarShare operation,
- battery support,
- DC/DC protection,
- thermal behaviour,
- communication,
- battery recovery.

The firmware remains the authoritative source for the exact current values.

Parameters marked with:

`Fix!Me`

have not yet been fully verified under all practical operating conditions and should be reviewed during commissioning or future development.


# 24 V BATTERY SOURCE CHARGER

## 2. 24 V Battery Parameters

The BatterySourceCharger is responsible for the 24 V battery.

Current important battery parameters:

| Parameter | Value | Status |
|---|---:|---|
| Absorption voltage | 28.4 V | current setting |
| Battery hard-stop voltage | 28.2 V | protection |
| BMS low-voltage region | 21.0 V | current setting |
| Recovery target | 21.5 V | tested |
| Recovery wake threshold | 22.0 V | `Fix!Me` |
| Recovery ramp threshold | 23.0 V | `Fix!Me` |

The recovery thresholds are used when the external battery BMS has disconnected the battery after deep undervoltage.


## 3. 24 V Battery Recovery

Current recovery parameters:

| Parameter | Value | Status |
|---|---:|---|
| Initial recovery current | 100 mA | `Fix!Me` |
| Recovery ramp interval | 30 s | `Fix!Me` |
| Long-recovery warning time | 2 h | `Fix!Me` |

Current staged recovery-current concept:

```text
100 mA
250 mA
500 mA
1 A
2 A
3 A
4 A
5 A
```

The staged values are provisional and should be verified with the actual battery BMS and wiring.

The recovery target of approximately **21.5 V** has already been tested successfully.


## 4. Source Input Selection

The Source distinguishes three input paths:

```text
MAIN
SOLAR
HELP
```

These correspond conceptually to:

```text
MAIN   = external Main / Max InputDC
SOLAR  = solar input with MPPT
HELP   = auxiliary input
```

The detailed source-current limits are defined in the firmware.

For the complete system architecture, the important distinction is:

```text
SOLAR
    → MPPT
    → SolarShare possible

MAIN / HELP
    → direct Source charging
    → not automatically SolarShare
```


## 5. Solar Source Parameters

The solar system used during development has approximately:

| Parameter | Typical value |
|---|---:|
| Open-circuit voltage | ~45 V |
| Typical MPP voltage | ~36 V |
| Minimum useful voltage | ~24 V |

Current important solar thresholds include:

| Parameter | Value | Status |
|---|---:|---|
| Solar valid threshold | 24.0 V | `Fix!Me` |
| Fast-drop threshold | 28.0 V | `Fix!Me` |

Below the fast-drop threshold, the MPPT controller rapidly reduces load to prevent collapse of a weak solar source.


## 6. Source MPPT Parameters

The Source MPPT uses adaptive Perturb-and-Observe control.

Current timing:

| Parameter | Value | Status |
|---|---:|---|
| Fast interval | 200 ms | `Fix!Me` |
| Normal interval | 500 ms | `Fix!Me` |
| Slow interval | 1000 ms | `Fix!Me` |

Current current-step sizes:

| Parameter | Value |
|---|---:|
| Fine step | 50 mA |
| Normal step | 100 mA |
| Large step | 250 mA |
| Fast down-step | 1000 mA |

Current power-change thresholds:

| Parameter | Value |
|---|---:|
| Power hysteresis | 0.2 W |
| Fine-power threshold | 0.5 W |
| Large-power threshold | 2.0 W |

Adaptive classification uses a 10-cycle observation window.

Current classification values:

```text
Dynamic count for FAST mode    3
Stable count for SLOW mode     7
```

These MPPT tuning values remain subject to practical optimization.


# 12 V BATTERY ASSIST CHARGER

## 7. 12 V Battery Charge Decision

The BatteryAssistCharger evaluates the 12 V battery before deciding whether a charging cycle is required.

Current thresholds:

| Battery voltage | Action | Status |
|---|---|---|
| ≥ 12.8 V | No charging required | `Fix!Me` |
| < 12.8 V | Float maintenance | `Fix!Me` |
| ≤ 12.6 V | Full charge cycle | `Fix!Me` |

This strategy is intended to avoid unnecessary repeated full charging cycles during long periods of vehicle storage.


## 8. 12 V Charge Voltages

Current charging values:

| Parameter | Value |
|---|---:|
| Absorption voltage | 14.2 V |
| Float voltage | 13.2 V |

The complete charging sequence is:

```text
Bulk
  │
  ▼
Absorption
  │
  ▼
Float
```


## 9. Absorption Parameters

Current absorption criteria:

| Parameter | Value | Status |
|---|---:|---|
| Minimum absorption time | 15 min | current setting |
| Maximum absorption time | 3 h | current setting |
| Full-current threshold | 100 mA | `Fix!Me` |

After the minimum time, charging may be considered complete when the battery voltage is in the absorption region and current has fallen below the configured full-current threshold.

The maximum absorption time prevents external loads from keeping the charger permanently in Absorption.


## 10. Assist Input Sources

The Assist uses:

```text
HIGH_POWER
MID_POWER
LOW_POWER
```

with the normal priority:

```text
HIGH_POWER → MID_POWER → LOW_POWER
```

The inputs are used as follows:

| Input | Function |
|---|---|
| `HIGH_POWER` | Main / external DC source |
| `MID_POWER` | 24 V battery bus / SolarShare / battery support |
| `LOW_POWER` | Auxiliary low-power source |


## 11. Assist Current Limits

Current configured values:

| Parameter | Value |
|---|---:|
| Main charge current setpoint | 5 A |
| Main charge current maximum | 9 A |
| SolarShare charge current setpoint | 5 A |
| SolarShare charge current maximum | 9 A |
| Auxiliary charge current setpoint | 2 A |
| Auxiliary charge current maximum | 2 A |

Current input limits:

| Input | Maximum |
|---|---:|
| Main input | 9 A |
| SolarShare input | 9 A |
| Auxiliary input | 2 A |

These are configured operating limits and do not replace hardware-current or thermal verification.


# SOLARSHARE

## 12. SolarShare Parameters

The Source communicates the maximum power currently permitted for the Assist as:

`grantedAssistPower_W`

The Assist must not exceed this power budget during normal SolarShare operation.

The protocol currently allows:

```text
0 ... 500 W
```

as the accepted plausibility range.

This is a protocol plausibility limit, not necessarily the final operational SolarShare limit.

> **Development status:** The final battery-capacity- and battery-state-dependent SolarShare allocation algorithm is still under development.


## 13. Battery Capacity

Each controller owns the capacity value of its own battery.

```text
BatterySourceCharger
    → 24 V battery capacity

BatteryAssistCharger
    → 12 V battery capacity
```

The Assist transmits:

`batteryCapacity_Ah`

Current Assist configuration:

`100 Ah`

Status:

`Fix!Me`

The field uses:

```text
uint16_t
1 Ah / bit
```

Special value:

```text
0 Ah = unknown / not configured
```

Current protocol plausibility limit:

`2000 Ah`

Status:

`Fix!Me`


# 12 V BATTERY SUPPORT

## 14. Support Parameters

Battery Support is separate from normal SolarShare operation.

It is used only to prevent damaging deep discharge of the 12 V battery when normal charging energy is unavailable.

Current values:

| Parameter | Value | Status |
|---|---:|---|
| 12 V support start | 11.8 V | `Fix!Me` |
| 12 V support stop | 12.4 V | `Fix!Me` |
| 24 V support enable | 24.5 V | `Fix!Me` |
| 24 V support stop | 24.0 V | `Fix!Me` |
| Support power | 8 W | `Fix!Me` |
| Maximum support current | 800 mA | `Fix!Me` |

The intention is to provide only limited maintenance energy.

The support function must not unnecessarily discharge the 24 V battery.


# DC/DC PARAMETERS

## 15. DC/DC Power Limit

Current continuous configured DC/DC power limit:

`350 W`

Status:

`Fix!Me`

This limit is used as a software operating limit.

It must not be interpreted as a guarantee that all hardware components have been verified for continuous 350 W operation under every voltage ratio and temperature condition.


## 16. Pre-Charge Parameters

Before connecting the DC/DC output directly to a battery, the firmware tests whether the converter output can be controlled correctly.

Current important values:

| Parameter | Value |
|---|---:|
| Test raise offset | +80 mV |
| Minimum expected rise | 40 mV |
| Battery connection offset | +30 mV |

Assist-specific current values also include:

```text
DCDC minimum test output     10.0 V
Connection settle time       300 ms
Stable measurement count     3
Test-phase timeout           5 s
```

The equivalent Source values are defined in its firmware and use the same basic pre-charge principle.


## 17. Reverse Current

Reverse current may occur when another charger raises the battery voltage above the local DC/DC output.

Current parameters:

| Parameter | Value |
|---|---:|
| Reverse-current threshold | -50 mA |
| Persistence time | 4 s |
| Retry interval | 60 s |

The retry interval is marked:

`Fix!Me`

Persistent reverse current causes the local charger to disconnect and observe the battery before retrying.

Reverse current itself is not automatically considered a fault.


# THERMAL MANAGEMENT

## 18. Thermal Power Derating

Both chargers use the hottest measured temperature from:

- MOSFET area,
- power inductor.

Current provisional derating:

| Temperature | Permitted power | Status |
|---|---:|---|
| ≤ 65 °C | 350 W | `Fix!Me` |
| 70 °C | 260 W | `Fix!Me` |
| 75 °C | 175 W | `Fix!Me` |
| 80 °C | 100 W | `Fix!Me` |
| ≥ 85 °C | Charging OFF | `Fix!Me` |

Restart threshold:

`55 °C`

Status:

`Fix!Me`

All thermal values require practical verification because converter losses depend strongly on the operating voltage ratio and actual installation.


## 19. Fan Parameters

The cooling fan is switched on whenever the DC/DC converter is actively charging.

After charging stops, the fan remains active until both monitored temperatures are approximately:

`35 °C`

Current fan check interval:

`1 s`

Status:

`Fix!Me`


# COMMUNICATION PARAMETERS

## 20. Communication Timing

Current values:

| Parameter | Value | Status |
|---|---:|---|
| Communication interval | 1000 ms | current setting |
| Partial-frame RX timeout | 20 ms | `Fix!Me` |
| Communication link timeout | 3000 ms | `Fix!Me` |

The Source is the communication master.


## 21. Protocol Parameters

Current fixed protocol values:

| Parameter | Value |
|---|---:|
| Start byte 1 | `0xA5` |
| Start byte 2 | `0x5A` |
| Protocol version | `0x01` |
| Payload size | 21 bytes |
| Complete frame size | 30 bytes |
| CRC polynomial | `0x1021` |
| CRC initial value | `0xFFFF` |

These values define the current wire protocol and should not be changed independently in only one controller.


## 22. Protocol Plausibility Limits

Current Source-data plausibility limits include:

| Field | Maximum / Range |
|---|---:|
| 24 V battery voltage | 36.00 V |
| Battery current | -50 ... +50 A |
| Source power | 1500 W |
| Safe source power | 1500 W |
| Assist grant | 500 W |
| SOC | 0 ... 100 % |
| Source mode | 0 ... 3 |
| MPPT state | 0 ... 7 |
| DC/DC state | 0 ... 4 |

Assist-data plausibility includes:

```text
SOC                 0 ... 100 %
Charge state        0 ... 4
Battery capacity    0 ... 2000 Ah     Fix!Me
Battery info code   0 ... 7
```

These values are only protocol plausibility limits.

They do not represent electrical operating or protection limits.


# FIX!ME CONVENTION

## 23. Meaning of Fix!Me

The firmware intentionally uses the marker:

`Fix!Me`

for parameters that still require practical verification or may depend on the final installation.

Typical reasons include:

- battery-specific behaviour,
- BMS behaviour,
- solar-panel characteristics,
- converter thermal behaviour,
- actual wiring losses,
- final vehicle installation,
- long-term operating experience.

The marker should not be removed merely because the firmware is working.

It should only be removed when the corresponding parameter has been deliberately reviewed and accepted.


## 24. Parameter Ownership

To avoid duplicated configuration, parameters should remain with the controller that owns the corresponding hardware or battery.

```text
BatterySourceCharger

    24 V battery parameters
    Source input parameters
    Solar / MPPT parameters
    24 V recovery parameters
    Source thermal parameters


BatteryAssistCharger

    12 V battery parameters
    12 V charge strategy
    Assist input parameters
    Battery support parameters
    Assist thermal parameters


Communication Protocol

    Shared frame structure
    Shared field scaling
    Shared protocol version
    Shared plausibility rules
```

A parameter should only be duplicated between firmware projects when this is technically necessary.


## 25. Final Note

The values documented here represent the current development state of the DualBatterySolarCharger.

The general rule is:

> **Firmware defines the exact value.  
> Documentation explains why the value exists and how it influences system behaviour.**

For detailed behaviour refer to:

- `SystemArchitecture.md`
- `ChargingStrategy.md`
- `CommunicationProtocol.md`

For the exact implementation refer to the corresponding firmware source.

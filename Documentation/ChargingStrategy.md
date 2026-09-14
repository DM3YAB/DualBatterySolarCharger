# Charging Strategy

## 1. Purpose

This document describes the charging decisions used by the **DualBatterySolarCharger**.

It concentrates on the behaviour of the two battery chargers:

- when charging starts,
- which energy source is used,
- how the charge current is limited,
- how charging progresses,
- when charging is considered complete,
- how available solar energy is shared,
- how abnormal battery conditions are handled.

The electrical architecture and controller responsibilities are described separately in:

`Documentation/SystemArchitecture.md`


## 2. General Charging Principle

Each charger is responsible for the battery directly connected to it.

```text
BatterySourceCharger
        │
        └──► 24 V battery

BatteryAssistCharger
        │
        └──► 12 V battery
```

Both chargers determine locally whether their battery requires charging.

The charging current is never determined by a single parameter alone.

It is limited by the most restrictive applicable condition, including:

- battery charge-current limit,
- available input power,
- input-source current limit,
- DC/DC power limit,
- thermal derating,
- battery voltage,
- charging state,
- SolarShare grant where applicable.

Conceptually:

```text
Requested charge current
          │
          ▼
 Battery current limit
          │
          ▼
 Source current/power limit
          │
          ▼
 DC/DC power limit
          │
          ▼
 Thermal limit
          │
          ▼
 SolarShare limit, if applicable
          │
          ▼
 Actual permitted charge current
```


## 3. 24 V Battery Charging

The BatterySourceCharger is responsible for charging the 24 V battery.

The present charging parameters are based on:

```text
Absorption voltage       28.4 V
Battery hard stop        28.2 V
```

The exact protection and charging parameters are defined in the firmware.

Some parameters are marked `Fix!Me` because they still require practical verification.


## 4. 24 V Charging from Solar

When the solar input is selected, the Source performs MPPT.

The solar panel used during development has approximately:

```text
Open-circuit voltage      ~45 V
Typical MPP voltage       ~36 V
Minimum useful voltage    ~24 V
```

The MPPT controller continuously observes:

```text
Psolar = Usolar × Isolar
```

and adjusts the requested charging load to find the useful solar operating point.

The MPPT algorithm therefore determines how much power can currently be obtained from the solar panels.

The battery charger then determines how much of this available power can safely be accepted by the 24 V battery and DC/DC converter.


## 5. 24 V Charging from External DC

The BatterySourceCharger can also charge from its external Main / Max InputDC.

This source does not require MPPT.

```text
External DC
     │
     ▼
BatterySourceCharger
     │
     ▼
24 V Battery
```

The charge current remains subject to the normal:

- input-current limit,
- battery-current limit,
- DC/DC power limit,
- thermal limits,
- battery-voltage protection.

External DC power connected to the Source is not automatically allocated to SolarShare.


## 6. 24 V Battery Recovery

The 24 V battery system uses a battery BMS that may disconnect the battery after deep undervoltage.

This creates a special startup condition.

A disconnected or deeply discharged battery must not immediately be treated like a normally operating battery.

The Source therefore provides a recovery procedure.

Current recovery parameters include:

```text
BMS low-voltage region       < 21.0 V
Recovery target              21.5 V
Recovery wake threshold      22.0 V     Fix!Me
Normal ramp region           > 23.0 V   Fix!Me
Initial recovery current     100 mA     Fix!Me
```

The **21.5 V recovery target has been practically tested**.

At the beginning of recovery, only a very small current may flow. This is expected behaviour because the external battery BMS may require time and voltage recovery before reconnecting the battery completely.

The recovery current is therefore increased gradually rather than immediately applying the normal charge-current limit.

Conceptually:

```text
Battery deeply discharged / BMS disconnected
                    │
                    ▼
             Recovery start
                    │
                    ▼
              ~100 mA limit
                    │
                    ▼
             Battery/BMS wakes
                    │
                    ▼
          Voltage rises sufficiently
                    │
                    ▼
         Gradually increase current
                    │
                    ▼
           Normal charging mode
```

Recovery may take a considerable amount of time after a severe winter undervoltage.

A long recovery is therefore treated as a warning condition rather than immediately as an error.


## 7. 12 V Battery Charging

The BatteryAssistCharger is responsible for the 12 V battery.

The 12 V charging strategy deliberately avoids performing unnecessary complete charge cycles during long periods of vehicle storage.

The battery is first evaluated before selecting the required charging strategy.


## 8. 12 V Battery Charge Decision

The present strategy distinguishes three battery conditions:

| Resting battery voltage | Charging action |
|---|---|
| **≥ 12.8 V** | No charging required |
| **< 12.8 V** | Float maintenance charging |
| **≤ 12.6 V** | Complete charging cycle |

The thresholds are configurable and marked `Fix!Me` where practical verification is still required.

The resulting decision is:

```text
                  Measure 12 V battery
                          │
             ┌────────────┼────────────┐
             │            │            │
          ≥12.8 V      <12.8 V      ≤12.6 V
             │            │            │
             ▼            ▼            ▼
          No charge      Float      Full charge
                          │            │
                          │            ▼
                          │          Bulk
                          │            │
                          │            ▼
                          │       Absorption
                          │            │
                          └────────────┤
                                       ▼
                                     Float
```

This means that a battery that is already sufficiently charged is left alone.

A moderately discharged battery only needs maintenance charging.

A more significantly discharged battery receives a complete charging cycle.


## 9. 12 V Full Charge Cycle

A complete charging cycle consists of:

```text
Bulk
  │
  ▼
Absorption
  │
  ▼
Float
```

The present absorption voltage is:

`14.2 V`

During Bulk charging, the charger attempts to supply the permitted charge current until the absorption-voltage region is reached.

During Absorption, the voltage is held near the configured absorption voltage while the battery current decreases.

The present completion criteria include:

```text
Minimum absorption time      15 minutes
Maximum absorption time       3 hours
Full-current threshold       100 mA     Fix!Me
```

After the minimum absorption time, the battery can be considered full when the voltage is within the required range and the charging current has fallen below the configured full-current threshold.

The maximum absorption time prevents external loads from keeping the charger indefinitely in Absorption.

Reaching the maximum time is therefore treated as a warning condition rather than automatically indicating a defective battery.


## 10. 12 V Float Charging

The Float voltage is:

`13.2 V`

Float serves two purposes in the Assist strategy:

1. maintaining a battery that does not require a complete charging cycle,
2. maintaining the battery after a completed Bulk/Absorption cycle.

This is particularly useful during long vehicle storage, where repeated unnecessary full charging cycles are undesirable.


## 11. Assist Input-Source Selection

The BatteryAssistCharger has three possible input sources:

```text
HIGH_POWER
    Main / external DC

MID_POWER
    24 V battery bus / SolarShare

LOW_POWER
    Auxiliary DC
```

The normal priority is:

```text
HIGH_POWER → MID_POWER → LOW_POWER
```

The source selection and the battery charging decision are separate decisions.

For example:

```text
HIGH_POWER available
        │
        ▼
Select HIGH_POWER
        │
        ▼
Does 12 V battery require charging?
        │
        ├── No  → Do not charge
        │
        └── Yes → Charge according to battery state
```

The presence of a power source alone therefore does not require a charging cycle.


## 12. SolarShare Charging

SolarShare allows part of the solar energy acquired by the BatterySourceCharger to charge the 12 V battery.

The energy path is:

```text
Solar Panels
     │
     ▼
Source MPPT
     │
     ▼
24 V Bus
     │
     ▼
Assist MID_POWER
     │
     ▼
12 V Battery
```

The Source determines the permitted power and transmits:

`grantedAssistPower_W`

The Assist must remain within this power budget.

The Assist does not perform MPPT and must not deliberately perturb the 24 V bus to search for additional solar power.

The Assist converts the granted power into an appropriate battery charge-current limit.

Conceptually:

```text
grantedAssistPower_W
          │
          ▼
  Available Assist power
          │
          ▼
Battery voltage / converter limits
          │
          ▼
Permitted battery charge current
```


## 13. Solar Power Distribution

Solar energy is first acquired by the Source MPPT.

The available power must then be divided between the two battery systems.

```text
                 Available Solar Power
                          │
                          ▼
                 Safe Solar Power
                          │
                          ▼
                  Power Allocation
                    ┌─────┴─────┐
                    ▼           ▼
                 24 V          12 V
                Battery       Battery
                               │
                               ▼
                    grantedAssistPower_W
```

The intended allocation considers the charging requirements and capacities of both batteries.

A fixed 50/50 division is not required.

For example, a nearly full battery should not reserve the same proportion of solar power as a substantially discharged battery merely because both batteries are connected.

> **Development status:** The final capacity- and battery-state-dependent allocation algorithm is not yet active in the current Source firmware. The current protocol already provides the required `batteryCapacity_Ah` and `grantedAssistPower_W` fields.


## 14. 12 V Battery Support

Battery Support is a separate operating strategy from SolarShare.

Its purpose is to prevent damaging deep discharge of the 12 V battery when no normal charging source is available.

The energy then comes from the 24 V battery itself:

```text
24 V Battery
     │
     ▼
Assist MID_POWER
     │
     ▼
12 V Battery
```

Current provisional support parameters are:

| Parameter | Value |
|---|---:|
| 12 V support start | 11.8 V |
| 12 V support stop | 12.4 V |
| 24 V support enable | 24.5 V |
| 24 V support stop | 24.0 V |
| Support power | 8 W |
| Maximum support current | 800 mA |

These values are marked `Fix!Me`.

Support is deliberately weak.

The purpose is not to normally charge the 12 V battery from the 24 V battery.

The purpose is:

> **Keep the 12 V battery out of damaging undervoltage without unnecessarily discharging the 24 V battery.**


## 15. SolarShare vs. Battery Support

Although both functions use the same electrical path through `MID_POWER`, their purpose is fundamentally different.

| | SolarShare | Battery Support |
|---|---|---|
| Energy origin | Current solar production | Stored 24 V battery energy |
| Normal operation | Yes | Exceptional/maintenance |
| Power determined by | Source grant | Fixed limited support strategy |
| Purpose | Use available solar energy | Prevent 12 V deep discharge |
| MPPT | Source only | Not applicable |
| May discharge 24 V battery intentionally | No normal objective | Yes, but strictly limited |

This distinction is important for the overall energy strategy.


## 16. External 12 V Charging

An external charger or vehicle charging system may charge the 12 V battery independently of the Assist.

The Assist observes this condition.

If the external charger raises the battery voltage above the DC/DC output, the Assist may measure reverse current.

Reverse current is not automatically considered an error.

Persistent reverse current causes the Assist to disconnect its own charger and observe the battery.

```text
External charger becomes dominant
             │
             ▼
     Reverse current detected
             │
             ▼
       Wait for persistence
             │
             ▼
     Disconnect Assist DCDC
             │
             ▼
       Observe / retry later
```

This prevents the chargers from unnecessarily working against each other.


## 17. DC/DC Pre-Charge

Before connecting a DC/DC converter directly to a battery, the charger verifies that its output can be controlled correctly.

The general procedure is:

```text
Battery disconnected from DCDC
             │
             ▼
Measure battery voltage
             │
             ▼
Match DCDC output to battery
             │
             ▼
Raise DCDC output slightly
             │
             ▼
Verify that output responds
             │
             ▼
Set connection voltage
slightly above battery voltage
             │
             ▼
Close battery connection
```

Typical present test values include:

```text
Test raise offset        +80 mV
Required measured rise    40 mV
Connection offset        +30 mV
```

This test helps detect an uncontrollable or incorrectly behaving DC/DC output before it is connected to the battery.


## 18. Power Limiting

The current continuous DC/DC power limit used by the chargers is:

`350 W`

This value is marked `Fix!Me`.

It is a configured operating limit and must not be interpreted as a statement that every component, connector, PCB trace and thermal condition has been validated for exactly this continuous power under all input/output voltage ratios.

The effective charge current is reduced when necessary to remain within the configured power limit.

For example:

```text
Imax ≈ Pmax / Vbattery
```

The actual firmware additionally considers current limits, source limits and other protection conditions.


## 19. Thermal Derating

Charging power is reduced as converter temperature increases.

Both MOSFET and inductor temperatures are monitored.

The hotter sensor determines the thermal limit.

The present provisional strategy is:

| Temperature | Permitted DC/DC power |
|---|---:|
| ≤ 65 °C | 350 W |
| 70 °C | ~260 W |
| 75 °C | ~175 W |
| 80 °C | ~100 W |
| ≥ 85 °C | Charging OFF |

Restart is permitted when both monitored temperatures have fallen to approximately:

`55 °C`

These values are marked `Fix!Me`.

Actual converter losses depend strongly on operating conditions, particularly the input/output voltage ratio. Step-up operation may create substantially different losses from step-down operation.

The thermal limits therefore require practical verification on the actual hardware.


## 20. Fan Strategy

The cooling fan runs whenever the DC/DC converter is actively charging.

After charging stops, the fan continues running until both monitored temperatures have fallen to approximately:

`35 °C`

This provides active after-cooling of the converter and inductor instead of stopping airflow immediately when charging current reaches zero.


## 21. Protection Priority

Normal charging decisions are always subordinate to protection.

Conceptually:

```text
Battery requests charging
          │
          ▼
Source available?
          │
          ▼
Pre-charge successful?
          │
          ▼
Battery voltage valid?
          │
          ▼
Temperature valid?
          │
          ▼
Power/current limits valid?
          │
          ▼
SolarShare grant valid, if required?
          │
          ▼
        CHARGE
```

Any protection condition can reduce or stop charging even when the battery itself requests more energy.


## 22. Strategy Summary

The complete charging strategy can be summarized as:

```text
24 V SYSTEM

Solar available
    → MPPT
    → charge 24 V battery
    → determine surplus/shareable solar power
    → grant part to Assist when appropriate

External DC available
    → charge 24 V battery directly

Deeply discharged / BMS disconnected
    → controlled low-current recovery
    → gradual return to normal charging


12 V SYSTEM

Battery sufficiently charged
    → leave battery alone

Battery moderately discharged
    → Float maintenance

Battery requires full charge
    → Bulk
    → Absorption
    → Float

SolarShare available
    → use only Source-granted power

External DC available
    → charge directly

12 V battery dangerously low
and no normal charging source available
    → limited 24 V battery support


ALWAYS

    Battery protection
        >
    Thermal protection
        >
    DC/DC protection
        >
    Source/power limits
        >
    Requested charging power
```

The objective is not to extract the maximum possible power at every moment.

The objective is to use the available energy efficiently while keeping both battery systems, the DC/DC converters and the available energy sources within controlled operating conditions.

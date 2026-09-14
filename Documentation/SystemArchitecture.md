# System Architecture

## 1. Purpose

The **DualBatterySolarCharger** consists of two independently operating battery chargers and one passive protocol monitor.

The system manages two electrically separate battery systems:

- **24 V battery system**
- **12 V battery system**

The 24 V system is the primary solar-energy system.  
The 12 V system can receive part of the available solar energy through the 24 V battery bus.

Both battery chargers can also use additional external DC power sources.

A fundamental design rule is:

> **Only the BatterySourceCharger performs solar MPPT.**

The BatteryAssistCharger does not attempt to find its own operating point on the 24 V supply. It receives a permitted SolarShare power budget from the BatterySourceCharger.


## 2. System Overview

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
                     ┌─────────────────────┐
 External DC ───────►│ BatteryAssistCharger│
 Auxiliary DC ──────►│                     │
                     │ 12 V charging       │
                     │ SolarShare          │
                     │ Battery support     │
                     └──────────┬──────────┘
                                │
                                ▼
                          ┌─────────────┐
                          │12 V Battery │
                          └─────────────┘


                   UART communication
       BatterySourceCharger ◄────────► BatteryAssistCharger
                           │
                           ▼
                   BatteryProtocolMonitor
                      passive monitor
```

The BatterySourceCharger therefore has two fundamentally different energy sources:

```text
SOLAR input
    → MPPT
    → 24 V charging
    → SolarShare calculation
    → possible power grant to Assist

MAIN / MAX InputDC
    → external DC charging
    → 24 V battery
    → no SolarShare allocation
```

The BatteryAssistCharger also has several possible energy sources:

```text
HIGH_POWER
    → external DC charging

MID_POWER
    → 24 V battery bus
    → SolarShare when granted
    → limited battery support when required

LOW_POWER
    → auxiliary DC charging
```

This distinction is important because **SolarShare describes the distribution of available solar energy**, not an arbitrary transfer of power from every source connected to the 24 V system.


## 3. Architectural Separation

The system deliberately separates three different tasks:

1. **Solar energy acquisition**
2. **Battery charging**
3. **Solar power distribution between the battery systems**

These tasks must not be confused with each other.


### Solar energy acquisition

The BatterySourceCharger controls the solar operating point using MPPT.

It determines how much power can safely be taken from the solar source.

The external Main / Max InputDC is a separate power source and does not participate in MPPT.


### Battery charging

Each charger remains responsible for its own battery.

The BatterySourceCharger controls the charging requirements and protection of the 24 V battery.

The BatteryAssistCharger independently controls the charging requirements and protection of the 12 V battery.

External DC power can therefore charge the corresponding battery without requiring SolarShare operation.


### Solar power distribution

When solar power is available, the BatterySourceCharger determines how much of this power may be used by the BatteryAssistCharger.

The BatteryAssistCharger may use this granted power, but it must not independently increase the load on the 24 V system beyond that grant.

Power supplied through the Source Main / Max InputDC is not automatically made available as SolarShare.


## 4. BatterySourceCharger

The **BatterySourceCharger** is the primary controller of the 24 V battery and solar-energy system.

Its responsibilities include:

- solar MPPT,
- external DC charging,
- 24 V battery charging,
- input-source selection,
- DC/DC converter control,
- current and voltage monitoring,
- thermal management,
- DC/DC pre-charge and connection checking,
- reverse-current handling,
- determining safely available solar power,
- communication with the BatteryAssistCharger,
- calculation of the SolarShare power grant.

The Source distinguishes between power required by its own 24 V system and solar power that may be shared with the 12 V system.

Conceptually:

```text
Solar Panels
     │
     ▼
Solar MPPT
     │
     ▼
Available Solar Power
     │
     ▼
Safe Solar Power
     │
     ├────────► 24 V battery charging
     │
     └────────► SolarShare
                    │
                    ▼
           grantedAssistPower_W
```

The Source always remains responsible for protecting its own input source, DC/DC converter and 24 V battery.

The external Main / Max InputDC follows a different path:

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

This power source does not require MPPT and is not automatically included in the SolarShare calculation.


## 5. BatteryAssistCharger

The **BatteryAssistCharger** is responsible for the 12 V battery.

It has three possible power inputs:

```text
HIGH_POWER  = Main / external DC source
MID_POWER   = 24 V battery bus / SolarShare
LOW_POWER   = Auxiliary low-power source
```

The normal priority is:

```text
HIGH_POWER → MID_POWER → LOW_POWER
```

The Assist performs:

- 12 V battery charging,
- input-source selection,
- DC/DC control,
- battery observation,
- current and voltage monitoring,
- thermal management,
- pre-charge checking,
- reverse-current handling,
- detection of external 12 V charging,
- low-battery observation,
- fast-discharge detection,
- autonomous low-power battery support,
- communication with the Source.

The Assist does **not** perform solar MPPT.

When `HIGH_POWER` is available, the Assist can charge the 12 V battery directly from its external DC source without requiring a SolarShare grant.


## 6. SolarShare

SolarShare is the controlled transfer of **solar-derived energy** from the 24 V system to the 12 V system.

The electrical path is:

```text
Solar
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

The 24 V battery bus acts as the electrical connection between both chargers.

However, the available voltage on this bus alone does not give the Assist permission to draw arbitrary charging power.

The Source communicates a power budget:

`grantedAssistPower_W`

The Assist converts this granted power budget into a suitable charging current while respecting its own current, power, battery and thermal limits.

Therefore:

```text
Source decides:

    How much solar power is safely available?
                     │
                     ▼
    How much of that power may be shared?


Assist decides:

    Does the 12 V battery require charging?
                     │
                     ▼
    How can the granted power be used safely?
```

The Assist does not perturb the 24 V bus to search for a better operating point. Doing so would interfere with the MPPT controller in the Source.


## 7. Separation of SolarShare and External DC Power

SolarShare is deliberately associated with the **solar source**.

The Source knows which input source is active and can therefore distinguish solar operation from external DC operation.

```text
SOLAR
  │
  ├── MPPT
  │
  ├── 24 V charging
  │
  └── SolarShare possible


MAIN / MAX InputDC
  │
  ├── External DC charging
  │
  └── No automatic SolarShare
```

This prevents an external DC supply connected to the BatterySourceCharger from unintentionally becoming a general power source for the 12 V system.

If an external DC supply is intended to charge the 12 V battery directly, the BatteryAssistCharger provides its own `HIGH_POWER` input for this purpose.


## 8. Battery Capacity Ownership

Each controller owns the configuration data belonging to its own battery.

```text
BatterySourceCharger
        │
        └── 24 V battery capacity

BatteryAssistCharger
        │
        └── 12 V battery capacity
```

The Assist reports its configured battery capacity to the Source as:

`batteryCapacity_Ah`

The capacity is transmitted as a `uint16_t` value in Ah.

`0 Ah` means that the capacity is unknown or not configured.

This architecture avoids maintaining the same configuration value in two different firmware projects.

For example, when the 12 V battery is replaced by a battery with a different capacity, only the BatteryAssistCharger configuration needs to be changed.

The Source receives the new capacity automatically through the communication protocol.


## 9. Solar Power Allocation

The Source receives information about the 12 V system and combines it with information about its own 24 V battery and the available solar power.

The intended allocation uses information such as:

- available safe solar power,
- 24 V battery condition,
- 24 V battery capacity,
- 12 V battery condition,
- 12 V battery capacity,
- current charging requirements,
- charger and system limits.

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
       power share               power share
                                      │
                                      ▼
                           grantedAssistPower_W
```

The objective is not necessarily a fixed percentage split.

A battery that requires more energy may temporarily receive a larger proportion of the available solar power, while a battery that is already well charged requires less.

Battery capacity must also be considered so that the charge requirements of differently sized battery banks can be compared meaningfully.

> **Development status:** The final capacity- and battery-state-dependent SolarShare allocation algorithm is still under development.


## 10. 12 V Battery Support Mode

Battery Support is intentionally separate from SolarShare.

SolarShare distributes **currently available solar energy**.

Support mode may take a small amount of energy from the **24 V battery itself** to prevent damaging undervoltage of the 12 V battery.

```text
Normal SolarShare:

Solar
  │
  ▼
24 V system
  │
  ▼
Assist
  │
  ▼
12 V battery


Battery Support:

24 V battery
     │
     ▼
   Assist
     │
     ▼
12 V battery
```

The support function is deliberately power limited.

Current provisional parameters include:

| Parameter | Current value |
|---|---:|
| 12 V support start | 11.8 V |
| 12 V support stop | 12.4 V |
| 24 V source start requirement | 24.5 V |
| 24 V source stop limit | 24.0 V |
| Support power | 8 W |
| Maximum support current | 800 mA |

These parameters are marked `Fix!Me` in the firmware and require practical verification.

The objective is not to transfer the complete remaining energy of the 24 V battery into the 12 V battery.

The objective is:

> **Prevent damaging 12 V battery undervoltage while preserving a useful energy reserve in the 24 V system.**


## 11. External Charging of the 12 V Battery

The 12 V battery may also be charged by another source, for example an external charger or vehicle charging system.

The BatteryAssistCharger observes battery voltage and current and can recognize an external charging condition.

This condition is reported as operating information rather than treated as an error.

If the external charger raises the battery voltage above the Assist converter output, reverse current may be detected.

Persistent reverse current causes the Assist charger to disconnect its own DC/DC output and observe the battery before retrying.

Reverse current is therefore an operating condition, not automatically a DC/DC fault.


## 12. Autonomous Operation

Communication improves cooperation between the chargers, but the basic protection of each battery must remain local.


### Source autonomy

The Source remains responsible for:

- its input sources,
- solar MPPT,
- 24 V battery protection,
- DC/DC protection,
- thermal protection.


### Assist autonomy

The Assist remains responsible for:

- 12 V battery protection,
- its DC/DC converter,
- thermal protection,
- external charging detection,
- low-battery observation,
- support operation.

A communication failure must therefore not disable fundamental local protection.

SolarShare, however, requires valid Source information because the Assist must know how much power it is permitted to draw from the 24 V system.

If valid Source communication is unavailable, normal SolarShare charging is not authorized.


## 13. Communication Architecture

The Source is the communication master.

The normal exchange is:

```text
BatterySourceCharger                  BatteryAssistCharger

       MASTER_REQUEST
             ───────────────────────►

                                      validate frame
                                      process Source data

       CLIENT_RESPONSE
             ◄───────────────────────
             same sequence number
```

The Source periodically sends a `MASTER_REQUEST`.

The Assist only responds to a valid request and echoes the received sequence number.

This allows the Source and ProtocolMonitor to identify missing, delayed or mismatched responses.

The exchanged information includes data such as:

```text
SOURCE → ASSIST

24 V battery voltage/current
available source power
safe source power
granted Assist power
Source operating states
warnings
errors


ASSIST → SOURCE

12 V battery voltage/current
actual input power
12 V battery capacity
charge state
operating state
battery information
warnings
errors
```

The detailed frame format is documented separately in:

`Documentation/CommunicationProtocol.md`


## 14. BatteryProtocolMonitor

The **BatteryProtocolMonitor** is not part of the charging control loop.

It is a passive diagnostic device.

```text
SOURCE TX ──────────────┐
                       ├──► BatteryProtocolMonitor
ASSIST TX ──────────────┘
```

It observes both communication directions and can evaluate:

- frame structure,
- protocol version,
- message type,
- payload length,
- CRC,
- sequence numbers,
- data plausibility,
- communication timing.

Rejected frames can be recorded separately for later diagnosis.

A failure or removal of the BatteryProtocolMonitor therefore has no influence on the charging system.


## 15. Failure Philosophy

The architecture distinguishes between three classes of information:


### STATUS

A normal or noteworthy operating condition.

Examples:

- external charging,
- support active,
- recovery operation,
- reverse current handling.


### WARNING

An abnormal or suspicious condition where operation may continue.

Examples:

- low battery,
- fast battery discharge,
- thermal derating,
- unusually long recovery.


### ERROR

A condition requiring protection or intervention.

Examples:

- DC/DC pre-charge failure,
- battery overvoltage,
- overtemperature shutdown.

This distinction is used both for local diagnostics and communication between the controllers.


## 16. Parameter Verification

Several system parameters depend on the real installation, battery type, thermal construction, wiring and practical operating behaviour.

Parameters that have not yet been sufficiently verified are marked directly in the firmware with:

`Fix!Me`

This marker is intentional.

It identifies values that should be checked during commissioning or future development instead of allowing provisional engineering assumptions to become undocumented permanent settings.


## 17. Design Principle

The overall architecture can be summarized as:

```text
BatterySourceCharger

    Acquire solar energy using MPPT.
    Charge and protect the 24 V battery.
    Charge from external DC when available.
    Determine safely available solar power.
    Decide how much solar power may be shared.


BatteryAssistCharger

    Charge and protect the 12 V battery.
    Charge directly from external DC when available.
    Use SolarShare only within the grant from the Source.
    Provide limited battery support when necessary.
    Never perform its own solar MPPT.


BatteryProtocolMonitor

    Observe both controllers.
    Diagnose communication.
    Record protocol problems.
    Never influence charging.
```

The central architectural rule is therefore:

> **The Source controls the origin and availability of shared solar energy.  
> The Assist controls the safe use of that energy for the 12 V battery.**

This separation keeps the individual controllers understandable and allows each charger to remain responsible for the hardware and battery directly connected to it.

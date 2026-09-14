DualBatterySolarCharger
Overview

The DualBatterySolarCharger is a distributed charging and energy-management system for two independent battery systems:

a 24 V battery system, primarily supplied by solar power,
a 12 V battery system, used as a separate auxiliary battery supply.

The project evolved from two initially independent DC/DC battery chargers. During development it became useful to allow both battery systems to share the available solar energy instead of treating them as completely separate systems.

The resulting system consists of three controllers:

BatterySourceCharger
Controls the 24 V battery system and the solar input. It performs the solar MPPT regulation and determines how much solar power is safely available.

BatteryAssistCharger
Controls the 12 V battery system. It can charge from an external DC source or receive energy from the 24 V battery system. It does not perform its own solar MPPT.

BatteryProtocolMonitor
Passively monitors the communication between Source and Assist. It is intended for development, diagnostics and long-term observation of the complete system.

Background

The original chargers were developed as independent 12 V and 24 V DC/DC battery chargers. Both use the same basic hardware concept: multiple selectable input sources, a controlled DC/DC converter, current and voltage measurement, temperature monitoring and a protected connection to the battery.

A separate solar MPPT charger was subsequently developed for the 24 V battery system.

The next development step was the realization that the 12 V system does not need its own solar MPPT controller. In the actual installation, the solar panels charge the 24 V system, while the 12 V charger is connected to the 24 V battery bus.

This changes the task fundamentally:

Solar panels
     │
     ▼
BatterySourceCharger
  MPPT + 24 V charging
     │
     ▼
24 V Battery
     │
     │  SolarShare
     ▼
BatteryAssistCharger
     │
     ▼
12 V Battery

There is therefore only one solar MPPT controller in the complete system: the BatterySourceCharger.

The BatteryAssistCharger does not search for a solar operating point. Instead, the Source determines how much power is currently available and grants part of this power to the Assist.

Power Sharing

The purpose of the communication between both chargers is not simply to transfer measurement data. It allows the two battery systems to cooperate.

The Source knows the condition of the 24 V system and the available solar power. The Assist reports the condition and configured capacity of its 12 V battery.

From this information, the Source can determine how the available solar power should be distributed between both battery systems.

For example, if the 24 V battery is already relatively well charged while the 12 V battery requires considerably more energy, a larger part of the available solar power may be granted to the 12 V system.

The battery capacities are parameters of the individual controllers. The Assist therefore transmits its configured battery capacity to the Source. Replacing the 12 V battery with a battery of another capacity does not require the same configuration to be manually duplicated in the Source firmware.

The calculated power available to the Assist is transmitted as:

grantedAssistPower_W

The Assist treats this value as a power budget and must not draw more SolarShare power than the Source has granted.

Development status: The protocol support for battery capacity and granted power is part of the system architecture. The final capacity/state-dependent SolarShare calculation in the Source is still under development.

12 V Battery Support

The 12 V system has an additional function that is deliberately separate from normal SolarShare charging.

During long periods without sufficient solar power or an external DC supply, small permanent loads can slowly discharge the 12 V battery.

The purpose of the Assist is not to keep both battery systems permanently charged from each other. Doing so could eventually discharge both batteries.

Instead, the 24 V battery can provide a small amount of emergency support when the 12 V battery reaches a low-voltage condition.

The present configuration uses approximately 8 W of support power. Support is stopped before the 24 V battery itself is unnecessarily discharged.

These voltage and power thresholds directly influence system behaviour and are therefore marked Fix!Me in the firmware until they have been sufficiently verified under real operating conditions.

The objective is:

Protect the 12 V battery from damaging deep discharge without sacrificing the 24 V battery to do so.

Normal charging resumes when solar energy becomes available again or an external DC supply is connected.

12 V Charging During Storage

Another design objective is to avoid unnecessary charging cycles while the vehicle is standing unused.

A 12 V battery that is already sufficiently charged does not need to be driven through a complete absorption cycle every time solar energy becomes available.

The current strategy therefore distinguishes between three conditions:

Battery >= 12.8 V
    No charging required

Battery < 12.8 V
    Recharge to Float (13.2 V)

Battery <= 12.6 V
    Perform one complete charge cycle
    Bulk → Absorption (14.2 V) → Float

The thresholds are configurable and marked Fix!Me where practical verification may lead to further adjustment.

This replaces the earlier concept of performing a full charging cycle once per solar day.

Power Source Priority

The BatteryAssistCharger distinguishes three input sources:

HIGH_POWER   = Main / external DC supply
MID_POWER    = SolarShare / 24 V battery system
LOW_POWER    = Auxiliary low-power input

The basic priority is:

HIGH_POWER → MID_POWER → LOW_POWER

An external supply connected to HIGH_POWER therefore has priority over energy transferred from the 24 V battery system.

Communication and Diagnostics

BatterySourceCharger and BatteryAssistCharger communicate through a compact binary UART protocol.

The communication includes battery voltage, battery current, available source power, granted Assist power, battery capacity, charger states, operating information, warnings and errors.

The BatteryProtocolMonitor can listen passively to both directions without participating in charger control.

Its purpose is to make the complete system observable:

BatterySourceCharger
        │
        ├──────────────► BatteryAssistCharger
        │                       │
        ◄───────────────────────┘
                │
                ▼
       BatteryProtocolMonitor

The monitor checks framing, CRC, sequence numbers and payload plausibility and can record communication and rejected frames for later analysis.

DualBatterySolarCharger/
│
├── README.md
│
├── BatterySourceCharger/
│   ├── README.md
│   ├── Firmware/
│   │   └── BatterySourceCharger.ino
│   └── Hardware/
│
├── BatteryAssistCharger/
│   ├── README.md
│   ├── Firmware/
│   │   └── BatteryAssistCharger.ino
│   └── Hardware/
│
├── BatteryProtocolMonitor/
│   ├── README.md
│   ├── Firmware/
│   │   └── BatteryProtocolMonitor.ino
│   └── Hardware/
│
└── Documentation/
    ├── SystemArchitecture.md
    ├── CommunicationProtocol.md
    ├── ChargingStrategy.md
    └── Parameters.md
    
Each firmware remains an independent program. The repository documents them together because their communication and power-management functions form one complete system.

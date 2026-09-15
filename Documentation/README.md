# Documentation

This directory contains the system-level documentation for the **DualBatterySolarCharger**.

The individual BatterySourceCharger, BatteryAssistCharger and BatteryProtocolMonitor directories contain the hardware- and firmware-specific documentation for each controller.

The documents in this directory describe the functions that involve the **complete system** and the interaction between the controllers.


## Documents

### [System Architecture](SystemArchitecture.md)

Describes the overall system structure and the responsibilities of each controller.

Topics include:

- 24 V and 12 V battery systems
- BatterySourceCharger and BatteryAssistCharger responsibilities
- solar MPPT architecture
- external DC sources
- SolarShare
- battery support
- BatteryProtocolMonitor
- separation between local and system-level control

**Recommended starting point for understanding the system.**


### [Charging Strategy](ChargingStrategy.md)

Describes how the system decides when and how the batteries are charged.

Topics include:

- 24 V battery charging
- solar MPPT operation
- external DC charging
- 24 V BMS recovery
- 12 V Bulk / Absorption / Float charging
- storage charging strategy
- SolarShare
- 12 V battery support
- pre-charge and reverse-current handling
- power and thermal limiting


### [Communication Protocol](CommunicationProtocol.md)

Technical reference for the UART communication between the BatterySourceCharger and BatteryAssistCharger.

Topics include:

- 30-byte frame structure
- message types
- MASTER_REQUEST
- CLIENT_RESPONSE
- 21-byte payloads
- byte order and scaling
- CRC16-CCITT
- sequence handling
- plausibility checking
- BatteryProtocolMonitor diagnostics


### [Parameters](Parameters.md)

Summary of the important configurable and system-relevant parameters.

Topics include:

- battery voltage thresholds
- charging parameters
- MPPT parameters
- SolarShare parameters
- battery support thresholds
- DC/DC limits
- thermal derating
- communication timing
- protocol plausibility limits
- `Fix!Me` convention


## Recommended Reading Order

For a first overview of the complete system:

```text
1. SystemArchitecture.md
          │
          ▼
2. ChargingStrategy.md
          │
          ▼
3. CommunicationProtocol.md
          │
          ▼
4. Parameters.md
```

The architecture explains **who is responsible for what**.

The charging strategy explains **how charging decisions are made**.

The communication protocol explains **how the controllers exchange the required information**.

The parameter documentation summarizes **the values that determine this behaviour**.


## Development Status

The project is under active development.

Parameters that still require practical verification are marked in the firmware and documentation with:

`Fix!Me`

Some protocol fields are already implemented for functions that are still being developed. In particular, the communication infrastructure for battery capacity and SolarShare power allocation exists, while the final capacity- and battery-state-dependent SolarShare allocation algorithm is still under development.

The firmware source code remains the authoritative reference for the exact implementation and current parameter values.

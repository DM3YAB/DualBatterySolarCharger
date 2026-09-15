# Changelog

All notable changes to the DualBatterySolarCharger project will be documented in this file.

The project existed as several experimental charger, MPPT and battery-management
developments before this repository structure was created.

Rather than reconstructing every intermediate development version, this changelog
starts with the first consolidated and documented development state.


## [0.1.0] - 2026-09-15

### Consolidated Development Snapshot

This version represents the first consolidated state of approximately six months
of charger, MPPT, battery-management and communication development.

The individual experimental developments have been reorganized into a common
project structure.


### BatterySourceCharger

- 24 V charger hardware based on BatteryCharger_24V
- three power-source inputs
- solar MPPT operation
- 24 V battery monitoring
- BMS recovery mode
- DC/DC pre-charge verification
- reverse-current detection and recovery
- thermal power derating
- fan control and after-run
- Source–Assist communication
- transmission of Source, battery and solar operating information
- preparation for future SolarShare power allocation


### BatteryAssistCharger

- 12 V charger hardware based on BatteryCharger_12V
- independent 12 V battery charging
- external charger detection
- battery observation and warning functions
- low-battery support from the 24 V system
- DC/DC pre-charge verification
- reverse-current detection and recovery
- thermal power derating
- fan control and after-run
- Source–Assist communication
- battery capacity transmitted to BatterySourceCharger
- SolarShare power-budget handling prepared


### BatteryProtocolMonitor

- passive monitoring of both Source–Assist communication directions
- independent UART frame decoding
- CRC16-CCITT verification
- frame plausibility checking
- communication sequence monitoring
- SD-card logging
- rejected-frame logging
- RTC timestamp support
- NUCLEO-F103RB free-wired hardware documented
- optional stand-alone display concept prepared


### Communication Protocol

Current protocol basis:

```text
Start bytes      A5 5A
Protocol version 0x01
Frame length     30 bytes
Payload length   21 bytes
CRC              CRC16-CCITT
```

Current primary message types:

```text
0x0001  MASTER_REQUEST
0x0002  CLIENT_RESPONSE
```

The protocol includes battery, charger, Source, operating-state, warning,
error and battery-capacity information.


### Documentation

Initial consolidated documentation added for:

- system architecture
- charging strategy
- communication protocol
- configurable parameters
- BatterySourceCharger
- BatteryAssistCharger
- BatteryProtocolMonitor
- firmware
- hardware
- monitor wiring
- UART level adaptation
- SD-card connection
- RTC / VBAT backup


### Licensing

The project is published using separate licenses for its different parts:

- Firmware: MIT License
- Hardware: CERN-OHL-P-2.0
- Documentation and original project images: CC BY 4.0


### Known Development Items

This is a development snapshot, not a final validated production release.

Parameters requiring further practical verification are marked in the firmware with:

```text
Fix!Me
```

These include selected:

- thermal limits
- timing values
- current limits
- MPPT parameters
- recovery parameters
- battery thresholds

The `Fix!Me` marker is intentionally retained as part of the development workflow.


### Not Yet Implemented

SolarShare communication infrastructure is prepared, but automatic solar-power
allocation between the 24 V and 12 V battery systems is not yet implemented.

The BatterySourceCharger currently does not calculate and issue the final
dynamic Assist power grant.

This functionality is planned for a later development version.


---

## Development Direction

Future versions will build on this consolidated state rather than on the earlier
experimental firmware branches.

Major changes from this point forward will be recorded in this changelog.

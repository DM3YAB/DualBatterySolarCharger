# BatterySourceCharger Hardware

## Hardware Basis

The BatterySourceCharger does not require a separate charger PCB.

It uses the same hardware and component population as:

**BatteryCharger_24V**

There are no special power-stage component changes required for the BatterySourceCharger version.

The functional differences between BatteryCharger_24V and BatterySourceCharger are primarily implemented in firmware.

The only system-specific hardware requirement is that the UART communication signals are made externally accessible.


## Source–Assist Communication

For operation within the DualBatterySolarCharger system, the following UART signals are brought out from the controller:

```text
TX
RX
GND
```

BatterySourceCharger and BatteryAssistCharger are connected using a crossed UART connection:

```text
BatterySourceCharger             BatteryAssistCharger

TX  ───────────────────────────► RX
RX  ◄─────────────────────────── TX
GND ──────────────────────────── GND
```

In other words:

> **TX connects to RX, and RX connects to TX.**

Do not connect:

```text
TX → TX
RX → RX
```

This sounds obvious, but is worth documenting directly at the hardware connection. :-)


## Communication Function

The BatterySourceCharger acts as the communication master.

It sends a:

```text
MASTER_REQUEST
```

to the BatteryAssistCharger.

The Assist responds with:

```text
CLIENT_RESPONSE
```

The hardware connection itself remains a normal UART connection.

Protocol handling, sequence numbers, CRC checking and SolarShare information are implemented in firmware.


## BatteryProtocolMonitor Connection

The BatteryProtocolMonitor observes the communication passively.

For monitoring the Source transmission, the existing Source TX signal is additionally connected to one of the ProtocolMonitor RX inputs.

Conceptually:

```text
                         ┌────► BatteryAssistCharger RX
                         │
BatterySourceCharger TX ─┤
                         │
                         └────► BatteryProtocolMonitor RX
```

The return direction is monitored in the same way:

```text
                         ┌────► BatterySourceCharger RX
                         │
BatteryAssistCharger TX ─┤
                         │
                         └────► BatteryProtocolMonitor RX
```

The ProtocolMonitor therefore only listens to both TX signals.

It does not transmit anything into the Source–Assist communication.


## Complete UART Wiring

The complete communication wiring is:

```text
          BatterySourceCharger
                  │
             TX   │   RX
              │   │    ▲
              │   │    │
              │   │    │
              ▼   │    │
             RX   │   TX
          BatteryAssistCharger


Source TX ───────────────► Assist RX
Assist TX ───────────────► Source RX
GND       ──────────────── GND
```

With the optional ProtocolMonitor connected:

```text
Source TX ──────┬────────► Assist RX
                │
                └────────► ProtocolMonitor RX1

Assist TX ──────┬────────► Source RX
                │
                └────────► ProtocolMonitor RX2

GND ─────────────────────► Common GND
```

The two ProtocolMonitor receive inputs are logically interchangeable.

The monitor firmware identifies Source and Assist from the protocol message type.


## Hardware Differences

Compared with the standard BatteryCharger_24V hardware:

```text
Power stage              unchanged
DC/DC converter          unchanged
Current measurement      unchanged
Voltage measurement      unchanged
Temperature measurement  unchanged
Fan control              unchanged
Local display            unchanged
Local controls           unchanged

UART TX/RX               externally accessible
```

No different component population of the charger PCB is required specifically for the BatterySourceCharger function.


## 24 V Charger Hardware

The BatterySourceCharger therefore inherits the electrical characteristics of the BatteryCharger_24V hardware.

This includes:

- DC/DC power stage
- source inputs
- INA238 current measurement
- battery-current measurement
- battery-voltage measurement
- DAC-controlled DC/DC output
- temperature monitoring
- fan control
- switching and protection hardware

The Source-specific behaviour is implemented by the BatterySourceCharger firmware.


## Solar Operation

The same hardware is also used for the solar input.

Solar MPPT is a firmware function of the BatterySourceCharger.

No separate MPPT hardware or differently populated charger PCB is required.

Conceptually:

```text
BatteryCharger_24V hardware
          │
          ├── External DC charging
          │
          └── Solar input
                  │
                  ▼
          Source firmware
                  │
                  ▼
                MPPT
```

The distinction between normal external DC power and solar power is therefore primarily made by the firmware and the selected input.


## SolarShare

SolarShare also does not require a separate power connection between Source and Assist.

The BatteryAssistCharger obtains the available energy from the existing 24 V system / battery bus.

The UART connection only communicates how much solar-derived power the Assist is permitted to use.

```text
Solar
  │
  ▼
BatterySourceCharger
  │
  ├────────► 24 V Battery / Bus
  │                    │
  │                    ▼
  │           BatteryAssistCharger
  │
  └── UART ─────────► SolarShare information
```

Therefore:

> **SolarShare is a control and allocation function, not an additional power cable between Source and Assist.**


## Hardware Documentation

For the complete:

```text
Schematic
PCB
Component placement
Power stage
Measurement circuits
DC/DC converter
```

refer to the hardware documentation of:

**BatteryCharger_24V**

This directory only documents hardware details that are specific to using that charger hardware as a BatterySourceCharger.


## Hardware Relationship

The relationship between the projects can therefore be summarized as:

```text
BatteryCharger_24V
        │
        │ same hardware
        ▼
BatterySourceCharger
        │
        ├── Source firmware
        ├── Solar MPPT
        ├── 24 V BMS recovery
        ├── SolarShare management
        └── UART communication
```

There is no separately populated BatterySourceCharger power PCB.


## Important Before Connection

Before connecting Source and Assist, verify:

```text
Source TX  → Assist RX
Source RX  ← Assist TX
GND        ↔ GND
```

If the BatteryProtocolMonitor is also connected:

```text
Source TX → Assist RX + ProtocolMonitor RX
Assist TX → Source RX + ProtocolMonitor RX
```

The monitor connections are receive-only.


## Hardware Status

Current BatterySourceCharger hardware:

```text
BatteryCharger_24V hardware
            │
            ├── unchanged power stage
            ├── unchanged measurement hardware
            ├── unchanged DC/DC hardware
            ├── unchanged thermal hardware
            │
            └── UART TX/RX brought outside
                         │
                         ▼
                BatteryAssistCharger
                         +
                BatteryProtocolMonitor
```

The BatterySourceCharger should therefore be regarded as:

> **BatteryCharger_24V hardware with Source-specific firmware and externally accessible UART communication.**

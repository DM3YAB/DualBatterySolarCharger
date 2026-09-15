# BatteryAssistCharger Hardware

## Hardware Basis

The BatteryAssistCharger does not require a separate charger PCB.

It uses the same hardware and component population as:

**BatteryCharger_12V**

There are no special power-stage component changes required for the BatteryAssistCharger version.

The functional differences between BatteryCharger_12V and BatteryAssistCharger are primarily implemented in firmware.


## Source–Assist Communication

For operation within the DualBatterySolarCharger system, the UART communication signals are made externally accessible.

Required signals:

```text
TX
RX
GND
```

The UART connection between BatteryAssistCharger and BatterySourceCharger is crossed:

```text
BatteryAssistCharger             BatterySourceCharger

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


## BatteryProtocolMonitor

The BatteryProtocolMonitor observes the communication passively.

For monitoring, only the respective charger **TX** signal is required:

```text
BatteryAssistCharger TX
          │
          ├────────► BatterySourceCharger RX
          │
          └────────► BatteryProtocolMonitor RX
```

The ProtocolMonitor does not need to be inserted into the communication path.

Source and Assist therefore continue communicating normally when the monitor is disconnected.


## Hardware Differences

Compared with the standard BatteryCharger_12V hardware:

```text
Power stage              unchanged
Current measurement      unchanged
Voltage measurement      unchanged
DC/DC control            unchanged
Temperature measurement  unchanged
Fan control              unchanged
Display / controls       unchanged

UART TX/RX               externally accessible
```

The BatteryAssistCharger should therefore be regarded as a **BatteryCharger_12V hardware variant with system-specific firmware and external UART communication**.


## Hardware Documentation

For schematic, PCB, component placement and power-stage information, refer to the hardware documentation of:

**BatteryCharger_12V**

Only Source–Assist-specific wiring and later verified hardware modifications should be documented in this directory.


## Important

When assembling or servicing the system, verify the UART wiring before connecting the controllers:

```text
Assist TX  → Source RX
Assist RX  ← Source TX
GND        ↔ GND
```

The hardware itself remains based on the BatteryCharger_12V design.

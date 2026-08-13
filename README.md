# System-Bus-Architecture

A scalable, parameterized, multi-master and multi-slave system bus architecture designed to connect heterogeneous digital hardware blocks operating at different clock frequencies.

The project focuses on the design and verification of a complex SoC-style interconnect using **Verilog, SystemVerilog, and Chisel**. The complete RTL design will be simulated and verified using **ModelSim/Questa**, with support for simulation and FPGA implementation workflows using **AMD Vivado** and **Intel Quartus Prime**.

---

## 1. Project Overview

Modern System-on-Chip (SoC) designs contain multiple processing elements, memory controllers, peripherals, DMA engines, accelerators, and other hardware IP blocks.

These components may:

- Operate at different clock frequencies
- Use different clock domains
- Generate simultaneous bus requests
- Have different data widths
- Have different transaction requirements
- Require independent buffering
- Generate responses at different times

Connecting these components directly becomes complex as the system grows.

This project aims to design a **general-purpose system bus/interconnect architecture** that manages communication between multiple masters and multiple slaves while providing safe communication across different clock domains.

The architecture will be designed to be:

- Modular
- Parameterized
- Scalable
- Synthesizable
- Reusable
- Simulation-friendly
- FPGA-friendly
- Suitable for future ASIC-oriented development

---

# 2. Main Objectives

The main objective is to design a system bus capable of handling:

### Multiple Masters

Examples:

- CPU
- DMA controller
- Hardware accelerator
- Debug controller
- Custom processing unit

### Multiple Slaves

Examples:

- SRAM
- ROM
- UART
- SPI
- I2C
- GPIO
- Timer
- Custom peripherals

### Multiple Clock Domains

Different components may operate at different frequencies.

Example:

```text
Master 0     : 100 MHz
Master 1     : 200 MHz
Bus Fabric   : 150 MHz
Slave 0      : 50 MHz
Slave 1      : 75 MHz
Slave 2      : 250 MHz
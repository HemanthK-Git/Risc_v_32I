# RISCV 5-Stage Pipeline Processor with Hazard Detection

A hardware implementation of a 5-stage pipelined processor based on the RISC-V 32I ISA. This project implements a complete 5-stage pipeline with comprehensive hazard detection and has been successfully synthesized for low-power medical applications, serving as a baseline microarchitecture for future pacemaker integration.

## 📑 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Microarchitecture](#microarchitecture)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Testing](#testing)
- [Results](#results)
- [Future Work](#future-work)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Overview

This repository contains the source code and documentation for a fully-functional, 5-stage pipelined RISC-V CPU. The core implements the RV32I base integer instruction set and is built around the classic stages of Instruction **Fetch (IF)**, **Instruction Decode (ID)**, **Execute (EX)**, **Memory Access (MEM)**, and **Write Back (WB)** to maximize throughput. The design includes a comprehensive **hazard detection** and **forwarding unit** to ensure correct execution and maintain pipeline efficiency.

## Features

* **ISA Support:** RV32I Base Integer Instruction Set
* **Pipeline:** Standard 5-Stage (IF, ID, EX, MEM, WB)
* **Hazard Handling:**
  * **Data Hazards:** Resolved via **Forwarding (Bypassing)** from EX/MEM and MEM/WB stages
  * **Control Hazards:** Branches resolved in ID stage with 1-cycle penalty
  * **Load-Use Hazards:** Detected and handled with pipeline stalls
* **Memory Architecture:** Harvard Architecture with separate Instruction and Data Memory
* **Synthesizable:** Written in synthesizable Verilog/VHDL
* **Tested:** Comprehensive test suite including ISA compliance tests

## Microarchitecture

### Pipeline Stages

The processor implements a classic 5-stage RISC pipeline with comprehensive hazard handling.

| Stage | Acronym | Description |
|-------|----------|-------------|
| **Instruction Fetch** | IF | Fetches instruction from IMEM using PC address<br>Increments PC by 4 for next instruction<br>Handles branch and jump redirection from EX stage |
| **Instruction Decode** | ID | Decodes instruction opcode and fields<br>Reads source registers from register file<br>Generates pipeline control signals<br>Sign-extends immediate values<br>Hazard detection for RAW dependencies |
| **Execute** | EX | Performs ALU operations and calculations<br>Computes memory addresses for load/store<br>Evaluates branch conditions and targets<br>Data forwarding from MEM/WB stages |
| **Memory Access** | MEM | Accesses data memory for load/store operations<br>Reads data for load instructions<br>Writes data for store instructions<br>Bypasses memory for non-memory instructions |
| **Write Back** | WB | Writes results back to register file<br>Selects data from ALU result or memory load<br>Completes instruction execution cycle |

### Hazard Handling

The processor implements a comprehensive hazard resolution system to maintain pipeline efficiency and ensure correct execution:

#### Data Hazards
- **Forwarding (Bypassing)**: Results from EX, MEM, and WB stages are forwarded directly to the EX stage as ALU inputs
- **Load-Use Stalls**: Pipeline stalls for one cycle when an instruction immediately depends on a preceding load operation
- **Register File Bypass**: Write-back and register read happen in same cycle for back-to-back dependencies

#### Control Hazards
- **Branch Prediction**: Static prediction assumes branches are not taken
- **Branch Resolution**: Branches resolved in EX stage with 1-cycle penalty on misprediction
- **Pipeline Flush**: Incorrectly fetched instructions after branches are flushed from IF and ID stages

#### Structural Hazards
- **Memory Arbitration**: Separate instruction and data memories eliminate memory access conflicts
- **Register File**: Multi-port design supports simultaneous read and write operations
- **Write-back Priority**: Ensures correct register update ordering when multiple writes occur

#### Hazard Detection Unit
- Monitors pipeline registers for dependencies between instructions
- Generates forwarding control signals for operand selection
- Issues pipeline stall signals when necessary
- Manages pipeline flushes for control flow changes

### Block Diagram
<img width="1115" height="695" alt="image" src="https://github.com/user-attachments/assets/c8cb4813-0064-425a-aae9-580cac19b348" />


### Hazard Unit

The Hazard Unit handles:
- **Data Hazards:** Forwarding from EX/MEM and MEM/WB stages
- **Load-Use Hazards:** Pipeline stalls when needed
- **Control Hazards:** Branch flushing and PC updates

## Repository Structure
riscv-5-stage-pipeline/
├── rtl/ # Source HDL files (Verilog/SystemVerilog)
│ ├── core/ # Main processor core modules
│ │ ├── riscv_core.v # Top-level processor module
│ │ ├── instruction_fetch.v # IF stage - PC, instruction memory
│ │ ├── instruction_decode.v # ID stage - Register file, immediate gen
│ │ ├── execute.v # EX stage - ALU, branch calculation
│ │ ├── mem_access.v # MEM stage - Data memory access
│ │ ├── write_back.v # WB stage - Register write back
│ │ ├── control_unit.v # Main control unit
│ │ ├── alu.v # Arithmetic Logic Unit
│ │ ├── reg_file.v # Register file (32 x 32-bit)
│ │ └── hazard_unit.v # Hazard detection & forwarding
│ └── memory/ # Memory models and interfaces
├── sim/ # Simulation environment
│ ├── testbenches/ # Verification testbenches
│ ├── scripts/ # Simulation scripts (Makefile, TCL)
│ └── waveforms/ # Sample waveform outputs
├── docs/ # Documentation
│ ├── microarchitecture.md # Detailed architecture explanation
│ ├── implementation.md # Implementation notes
│ └── verification.md # Testing methodology
└── README.md # Project overview (this file)

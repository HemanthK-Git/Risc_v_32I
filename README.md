# RISC-V 5-Stage Pipeline Processor

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/[YourUsername]/[YourRepo]/blob/main/LICENSE)
[![ISA](https://img.shields.io/badge/ISA-RV32I-orange.svg)](https://riscv.org/technical/specifications/)

A hardware implementation of a 5-stage pipelined processor based on the RISC-V ISA. This project is designed for educational purposes, FPGA prototyping, and to serve as a baseline for microarchitectural exploration.

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

This repository contains the source code (HDL), testbenches, and documentation for a classic 5-stage pipelined CPU. The processor implements the **RV32I** base integer instruction set, providing a complete, usable core. The five stages—Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), and Write Back (WB)—are designed to maximize instruction throughput.

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

| Stage | Acronym | Description |
|-------|---------|-------------|
| **Instruction Fetch** | IF | Fetches instruction from IMEM using PC, increments PC by 4 |
| **Instruction Decode** | ID | Decodes instruction, reads registers, generates control signals |
| **Execute** | EX | ALU operations, address calculation, forwarding resolution |
| **Memory Access** | MEM | Data memory access for load/store instructions |
| **Write Back** | WB | Writes result back to register file |

### Block Diagram

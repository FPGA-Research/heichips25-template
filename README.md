# HeiChips 2025 — Snitch Core Macro

This repository is a **HeiChips 2025 Hackathon** project built from the official template. It integrates a compact **Snitch RISC‑V (RV32) integer core** and wraps it in the required HeiChips 2-tile-macro interface for simulation  and ASIC implementation with **LibreLane** on the **IHP SG13G2 (130 nm)** PDK.

A central design choice of our macro is a **narrow, time‑multiplexed I/O datapath**: because the chip‑level I/O budget is not much and our macro will later be connected through an on‑die **eFPGA fabulous fabric**, we **serialize 32‑bit instruction/data transfers over a 4‑bit bus**. Each 32‑bit word is delivered as **8 bits in 8 cycles**.

> **Summary:**
> • Small RV32 Snitch core.
> • **4‑bit fetch/store** path (32 bits in 8 cycles).
> • Targets the **double‑tile** macro (500 µm × 415 µm).
> • Flows for **RTL/GL sim**, **LibreLane** P\&R.

---

## Table of contents

* [What this project is](#what-this-project-is)

* [Core background & licensing](#core-background--licensing)

* [Architecture overview](#architecture-overview)

* [4‑bit transfer protocol](#4bit-transfer-protocol)

* [Repository layout](#repository-layout)

* [Build & run](#build--run)

  * [Nix dev‑shell](#nix-devshell)
  * [Simulation (RTL & gate‑level)](#simulation-rtl--gatelevel)
  * [ASIC implementation with LibreLane](#asic-implementation-with-librelane)

* [Top‑level interface](#toplevel-interface)

* [Configuration notes](#configuration-notes)

* [Integration notes (eFPGA + chip I/O)](#integration-notes-efpga--chip-io)

* [Status & known limitations](#Status & known limitations)

* [References & attributions](#references--attributions)

* [License](#license)

---

## What this project is

This project implements a small  RV32 compute/control macro based on the Snitch core. The macro is intended to plug into the HeiChips SoC fabric, with external connectivity and additional logic routed via the chip’s eFPGA. The design emphasizes:

* **Low‑pin I/O** using a **4‑bit, time‑multiplexed** transfer for both reads and writes.
* Clean separation between the **Snitch core**, the **4 bit** **serializer/deserializer (SerDes)**, and the **macro I/O shell**.

## Core background & licensing

**Snitch** is a single‑stage, single‑issue core developed in the **PULP** ecosystem (ETH Zürich/UniBo). It is small and efficient, with clean interfaces. 

Licensing (upstream):

* Hardware sources: **Solderpad Hardware License v0.51 (SHL‑0.51)**.
* Associated software/utilities: **Apache‑2.0**.

We integrate Snitch here as an **RV32I‑class** integer core (no FPU) to minimize area and routing pressure. See the upstream Snitch documentation for parameters and port details.

---

## Architecture overview

```
                +-------------------------------+
                |         heichips25_*          |
                |   (macro top-level wrapper)   |
                |                               |
  ui_in[7:0] -->|                               |
 uo_out[7:0] <--|      I/O shell & control      |<-- uio_oe[7:0]
 uio_in[7:0] -->|         (handshake)           |--> uio_out[7:0]
                |           |    ^              |
                |           v    |              |
                |     +-----------------+       |
                |     |  4b SerDes +    |       |
                |     |  fetch/store IF |<----> eFPGA fabric / SRAM / peers
                |     +-----------------+       |
                |           |                   |
                |           v                   |
                |     +-----------+             |
                |     |  Snitch   |             |
                |     |   RV32    |             |
                |     +-----------+             |
                +---------------|---------------+
                                |
                             clk,rst_n
```

**Key points**

* **4‑bit SerDes**: A small shifter/accumulator converts between 4‑bit nibbles on the macro boundary and 32‑bit words inside.
* **Handshake**: Valid/ready‑style flow‑control prevents overrun/underrun.
* **Bus‑agnostic edge**: The external face is narrow by design so the eFPGA can adapt to multiple roles (bridge to SRAM, memory‑mapped I/O).

---

## 4‑bit transfer protocol

Each 32‑bit word is split into 8 nibbles (N0…N7). On every cycle where `nib_valid && nib_ready` is true, exactly one nibble transfers.

* **Receive (fetch/load)**

  1. `N0` arrives first, then `N1`, …, `N7` (nibble order is configurable; default is LSB‑first, i.e., `N0`→bits `[3:0]`, `N7`→bits `[31:28]`).
  2. The SerDes shifts/accumulates nibbles; on the 8th accepted nibble it asserts `word_valid` with the assembled 32‑bit word.
  3. If the core isn’t ready, the SerDes keeps the completed word latched until it is consumed.

* **Transmit (store/write)**

  1. When the core presents `word_valid`, the SerDes de‑serializes it into 8 nibbles.
  2. It drives one nibble per cycle while the external interface holds `nib_ready` high; it stalls otherwise.

### Example

```systemverilog
// Receive path (nibbles -> word)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    cnt <= 3'd0; word_valid <= 1'b0; shreg <= '0;
  end else begin
    if (nib_valid && nib_ready) begin
      shreg <= {nibble, shreg[31:4]}; // LSB-first (N0 enters lowest 4b)
      cnt   <= cnt + 3'd1;
      if (cnt == 3'd7) begin
        word       <= {nibble, shreg[31:4]};
        word_valid <= 1'b1;
        cnt        <= 3'd0;
      end
    end
    if (word_valid && word_ready) word_valid <= 1'b0;
  end
end
```

*(The transmit path mirrors this with a down‑counter and 4‑bit mux.)*

## Repository layout

```
.
├─ fpga/                 # Board wrappers & make targets for emulation
├─ librelane/            # OpenROAD/KLayout flow configuration
├─ macro/                # Final DEF/GDS (after `make macro`)
├─ nix-openxc7/          # 
├─ src/                  # RTL: top-level, SerDes, Snitch glue
├─ tb/                   # Testbench (cocotb) & wave dumps
├─ Bender.yml            # HDL dependencies (incl. Snitch)
├─ Makefile              # High-level sim/FPGA/macro targets
├─ heichips.f            # Unified file list for tools
├─ flake.nix, shell.nix  # Nix environment (LibreLane, simulators)
└─ LICENSE               # Apache‑2.0 for this repo
```

---

## Build & run

### Nix dev‑shell

```bash
nix-shell
```

&#x20;On the HeiChips VM, Nix is pre‑installed.

### Simulation (RTL & gate‑level)

```bash
# RTL simulation
make sim

# Gate‑level simulation
ciel enable --pdk-family ihp-sg13g2 cb7daaa8901016cf7c5d272dfa322c41f024931f
make sim-gl

# Use Verilator 
export SIM=verilator && make sim

# Waves
gtkwave tb/sim_build/heichips25_template.fst
```

### ASIC implementation with LibreLane

```bash
# Full macro flow
make macro
```

We target the **double tile**  (**500 µm × 415 µm**).&#x20;

## Top‑level interface

The macro complies with the template’s top‑level:

```verilog
module heichips25_template (
  input  wire [7:0] ui_in,
  output wire [7:0] uo_out,
  input  wire [7:0] uio_in,
  output wire [7:0] uio_out,
  output wire [7:0] uio_oe,
  input  wire       ena,
  input  wire       clk,
  input  wire       rst_n
);
```

Bit assignments are kept in `src/` so the same RTL serves sim, FPGA, and macro builds. Note: we used the in-out pins as additional output pins.

---

## Configuration notes

* **Core**: Snitch (RV32 integer variant), single‑stage pipeline.
* **SerDes**: nibble order configurable (default LSB‑first). Our macro uses 32‑bit words @ 4‑bit lane.
* **Clock/reset**: single `clk`, active‑low `rst_n`.
* **Power**: default VPWR/VGND rails per template.

---

## Integration notes (eFPGA + chip I/O)

* **Role of the eFPGA**: terminate the 4‑bit link and bridge to SRAM, MMIO.
* **Clocking**: the link runs on `clk` with 100MHz.
* **Reset**: assert `rst_n=0` long enough for SerDes counters to clear.

---

## Status & known limitations

**Done**

* Snitch RV32 core integrated behind a 4‑bit SerDes.
* Unified build flow (sim/FPGA/macro) via Make + Nix.

**Known limitations**

* Peak bandwidth is limited by the 4‑bit lane (see [Throughput & latency](#throughput--latency)).
* Current branch uses the integer core only (no FPU) to meet area/timing in the double‑tile.

---

## References & attributions

* **Snitch** core and docs — PULP platform community (ETH Zürich / UniBo).
* **HeiChips 2025** template and tapeout infrastructure.
* **LibreLane** Nix‑packaged digital ASIC flow.

---

## License

* This repository: **Apache‑2.0** (see `LICENSE`).
* Snitch hardware sources (upstream): **SHL‑0.51**; utilities: **Apache‑2.0**.

---


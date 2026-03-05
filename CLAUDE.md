# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MiSTer FPGA core for the Sega MegaDrive/Genesis, based on the [Nuked-MD](https://github.com/nukeykt/Nuked-MD-FPGA) cycle-accurate chipset emulation derived from decapped silicon analysis. Also supports Sega Master System cartridges. Created by Alexey Melnikov.

## Build System

This is a Quartus FPGA project targeting the MiSTer DE10-Nano (Intel Cyclone V).

- **Quartus version:** 17.0.2 Standard Edition
- **Top-level entity:** `sys_top` (defined in `MegaDrive.qsf`)
- **Project file:** `MegaDrive.qpf`
- **Output:** `output_files/MegaDrive.rbf`

**Build command (Quartus shell):**
```
quartus_sh --flow compile MegaDrive
```

**Important QSF note:** Do NOT add files via the Quartus IDE — it will corrupt `MegaDrive.qsf`. Add files manually to `files.qip` instead.

**Verilog macros defined in QSF:** `M68K_CHEAT=1`, `Z80_CHEAT=1`, `EXT_CLOCKS=1`

There are no testbenches or CI pipelines in this project.

## Verification with Verilator

Verilator is available for lint-checking Verilog files. **You MUST run a Verilator lint check before and after any signal renaming, port changes, or structural edits to Nuked-MD files.** The lint must pass cleanly (no new warnings) before committing.

**Lint command for a single module (e.g., ym7101.v):**
```
verilator --lint-only -Wno-PINMISSING -Wno-DECLFILENAME -Wno-MULTITOP \
  -Wno-GENUNNAMED -Wno-UNOPTFLAT -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  -Wno-SELRANGE -Wno-PINCONNECTEMPTY \
  rtl/nuked-md/ym7101.v rtl/nuked-md/ym_lib.v
```

**Lint command for cross-file changes (port renames touching fc1004.v/md_board.v):**
```
verilator --lint-only -Wno-PINMISSING -Wno-DECLFILENAME -Wno-MULTITOP \
  -Wno-GENUNNAMED -Wno-UNOPTFLAT -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  -Wno-SELRANGE -Wno-PINCONNECTEMPTY \
  rtl/nuked-md/md_board.v rtl/nuked-md/ym_lib.v rtl/nuked-md/*.v
```

**Suppressed warnings explained:**
- `PINMISSING` — ym_lib primitives have optional `val`/`nval`/`q`/`nq` outputs; unused ones are intentionally unconnected
- `WIDTHEXPAND`/`WIDTHTRUNC`/`SELRANGE` — ym_lib shift register parameterization edge cases
- `DECLFILENAME`, `MULTITOP`, `GENUNNAMED`, `UNOPTFLAT`, `PINCONNECTEMPTY` — benign structural warnings

**What Verilator catches:** undeclared signals (missed renames), port mismatches, width errors — all common mechanical renaming mistakes.

**What it does NOT catch:** swapped signals (e.g., renaming `w100` to `hsync` when it should be `vsync`). Verify signal semantics manually.

## Architecture

### Module Hierarchy

```
sys_top (sys/sys_top.v)           — MiSTer hardware abstraction (pins, clocks, HPS)
  └── emu (MegaDrive.sv)          — Core HAL: config, clocks, I/O wiring, OSD menu
        ├── md_board (md_board.v)  — Complete MegaDrive PCB as HDL
        │     ├── m68kcpu (68k.v)        — Motorola 68000 CPU (cycle-accurate)
        │     ├── z80cpu (z80.v)         — Zilog Z80 CPU (cycle-accurate)
        │     ├── ym7101 (ym7101.v)      — VDP graphics processor
        │     ├── ym3438 (ym3438.v)      — FM synthesis (YM2612/YM3438)
        │     ├── fc1004 (fc1004.v)      — Integrated Yamaha chip
        │     ├── ym6045 (ym6045.v)      — Bus arbiter
        │     ├── ym6046 (ym6046.v)      — I/O controller
        │     └── tmss (tmss.v)          — TMSS security ROM
        ├── cartridge (cartridge.sv)     — ROM loading, mappers, SDRAM, saves
        ├── video_cond (video_cond.sv)   — Video signal conditioning
        ├── audio_cond (audio_cond.sv)   — Audio filtering (model 1/2 filters)
        ├── pad_io / md_io               — Controller/joystick interface
        ├── SVP (SVP.vhd)               — Virtua Racing coprocessor
        └── VM2413 (opll.vhd)           — Master System FM (OPLL)
```

### Key Directories

- **`rtl/`** — Core RTL source files (integration, cartridge, audio/video conditioning, I/O peripherals)
- **`rtl/nuked-md/`** — Cycle-accurate chip emulations from Nuked-MD (68K, Z80, VDP, FM, arbiter, I/O)
- **`rtl/SVP/`** — SSP1601 DSP for Virtua Racing (VHDL)
- **`rtl/VM2413/`** — Yamaha OPLL FM chip for SMS compatibility (VHDL)
- **`sys/`** — MiSTer framework (shared across all MiSTer cores, not project-specific)

### File Organization

- **`files.qip`** — Master file list for Quartus compilation (add new RTL files here)
- **`rtl/nuked-md.qip`** — Nuked-MD chip file list
- **`rtl/SVP/SVP.qip`** — SVP chip file list
- **`rtl/VM2413/opll.qip`** — OPLL chip file list
- **`MegaDrive.sdc`** — Timing constraints

### Language Mix

The project uses SystemVerilog (`.sv`) for integration/peripheral modules, Verilog (`.v`) for Nuked-MD chip emulations, and VHDL (`.vhd`) for SVP and VM2413 subsystems.

### Signal Flow

```
Cartridge ROM (SDRAM) → 68K + Z80 CPUs → md_board chipset → Video/Audio
                              ↕                                  ↓
                        RAM (68K/Z80/VRAM)              MiSTer framework output
                              ↕
                     I/O (controllers, keyboard, lightgun)
```

### Clocking

- Master clock: ~53.7 MHz (NTSC) / ~54.2 MHz (PAL), generated by PLL from 50 MHz input
- A doubled 107 MHz clock is also generated for timing margin
- Clock enables gate the individual chip emulations at their correct rates

### Key Integration Points

- **`MegaDrive.sv`** — The main integration file. Contains the OSD configuration string (`CONF_STR`), all peripheral instantiation, region handling, save states, and cheat code wiring. Most feature changes start here.
- **`cartridge.sv`** — Mapper logic, SDRAM interface, EEPROM save handling. Add new mapper support here.
- **`md_board.v`** — Wires the Nuked-MD chips together like a real MegaDrive PCB. Rarely needs modification.

### Nuked-MD Signal Naming Conventions

The `rtl/nuked-md/` files have been annotated with descriptive signal names and section comments. When working in these files:

- **`ym_lib.v`** — Shared primitive library. Master-slave flip-flop registers use `master`/`slave` naming. Each module has a doc comment describing its function.
- **`fc1004.v`** — Chip integration wrapper. Internal wires are prefixed by source chip (`vdp_`, `fm_`, `arb_`, `ioc_`, `tmss_`). Organized with section comments for wire declarations, chip instantiations, bus muxes, and test routing.
- **`ym6045.v`** — Bus arbiter. Outputs use descriptive names (e.g., `VD8_OE_n`, `VA_MID_OE_n`). Section comments mark major functional blocks.
- **`ym6046.v`** — I/O controller. Bus direction outputs use descriptive names (e.g., `ZA_OE`, `VD_LO_OE_n`). Section comments throughout.
- **`ym3438_io.v`** — FM synth I/O. Section comments mark the sync pipeline, busy counter, status readback, and debug mux.
- **`ym3438.v`** — FM synth top-level. Section comments mark each pipeline stage (prescaler through DAC output).
- **`ym3438_prescaler.v`**, **`ym3438_fsm.v`**, **`ym3438_detune.v`**, **`ym3438_ch.v`**, **`ym3438_lfo.v`**, **`ym3438_pg.v`**, **`ym3438_op.v`**, **`ym3438_eg.v`**, **`ym3438_regs.v`** — FM synth sub-modules. Each has a module doc comment and section comments marking functional blocks. Generate labels use descriptive names (e.g., `clk_stages`, `slot_rows`, `multi_decode`, `timer_bits`).
- **`tmss.v`**, **`vram.v`** — Annotated in Phase 1.
- **`z80.v`** — Z80 CPU. Section comments mark major functional blocks (control logic, PLA, sequencer, ALU, register file, incrementer, bus bridge). Key wire declarations have inline comments identifying signal roles. Helper primitives (`z80_dlatch`, `z80_rs_trig_nor`, `z80_rs_trig_nand`) have doc comments. 45 annotated signals have been renamed to descriptive names (e.g., `iff1`, `iff2`, `reset_sync`, `nmi_edge`, `prefix_active`, `exec_early`, `step_s0`–`step_s4`, `unprefixed`, `ed_prefix`, `dd_fd_prefix`); remaining opaque `w###`/`l###` names are unannotated signals.
- **`68k.v`** — 68000 CPU. Module doc comment describes architecture (microcode ROMs, PLAs, internal buses, register file). Section comments mark 11 functional blocks (wire declarations, initialization, clock phases, microcode sequencer, microcode ROM decode, instruction decode PLA, secondary decode & IRD PLAs, execution control, ALU/flags/bus control, internal bus bridge, register file & data I/O). Key wire declarations have inline comments identifying signal roles (clocks, microcode control word, instruction register, CCR flags, bus control strobes, register file, internal buses). PLA entries for a0_pla[0-19] have instruction mnemonic comments. 62 annotated signals have been renamed to descriptive names (e.g., `dtack_sync1`, `reset_sync1`, `seq_ctl_0`–`seq_ctl_5`, `fc_bit0`–`fc_bit2`, `bus_cycle_active`, `ccr_n`/`ccr_z`/`ccr_v`/`ccr_x`/`ccr_c`, `rom_col_0`–`rom_col_2`, `rom_bank_0`/`rom_bank_1`); remaining opaque `w###` names are unannotated signals.
- **`ym7101.v`** — VDP (Video Display Processor). Module doc comment describes architecture (scroll planes, sprites, DMA, VRAM, color RAM, DAC, PSG). Section comments with `// ---...` banners mark 14 functional blocks (prescaler, I/O & DMA, timing FSM, H/V PLAs, scroll planes, VSRAM, sprite processing, SAT/sprdata/linebuffer, VRAM interface, video MUX, DAC, color RAM, PSG, bus drive, misc outputs). Wire/reg declarations have grouped sub-section comments identifying signal roles (CPU interface, prescaler, counters, PLAs, registers, sprite pipeline, pixel data, memory arrays, video output). Helper submodule `ym7101_rs_trig` has a doc comment. ~610 annotated signals have been renamed to descriptive names across the sprite engine (SAT traversal, attribute fetch, tile fetch, X-sort, line buffer, pixel readout), VRAM interface (RAS/CAS/WE timing, address mux, serial data capture), and earlier functional blocks (e.g., `reg_wr_80`–`reg_wr_8D`, `hcnt_load_en`, `vcnt_inc_en`, `dma_active_trig`, `sat_read_mux`, `spr_y_visible`, `tile_offset_cnt`, `pixel_serial`, `lb_write_en`, `vram_oe1_comb`, `vram_addr_out`); remaining opaque `w###`/`l###` names are unannotated signals in other sections.

**When renaming signals across files:** Always rename the port in the module definition AND update all instantiation sites (especially in `fc1004.v` and `md_board.v`). Run the Verilator lint check (see "Verification with Verilator" section) before and after every rename batch to catch missed references.

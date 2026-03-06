/*
 * Copyright (C) 2023 nukeykt
 *
 * This file is part of Nuked-MD.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  YM7101 emulator
 *  Thanks:
 *      Fritzchens Fritz:
 *          YM7101 decap and die shot.
 *      andkorzh:
 *          YM7101 deroute.
 *      org (ogamespec):
 *          early YM7101 decap and die shot.
 *      HardWareMan:
 *          help & support.
 *
 */

// YM7101 — Sega MegaDrive/Genesis Video Display Processor (VDP)
//
// Custom Yamaha gate-array responsible for all video output. Implements:
//   - Two scroll planes (A & B) with per-tile and per-line scrolling
//   - 80-entry sprite engine with per-line rendering and priority
//   - 64-entry color RAM (CRAM) with 9-bit RGB (512-color palette)
//   - 40-entry vertical scroll RAM (VSRAM)
//   - 64 KB VRAM interface with RAS/CAS/WE timing for two DRAM chips
//   - DMA engine (68K→VRAM, VRAM fill, VRAM copy) with 4-word FIFO
//   - H/V counter with programmable interrupts (HINT, VINT, EINT)
//   - Programmable display modes: H32/H40, V28/V30, interlace
//   - Shadow/highlight priority processing
//   - Integrated SN76489-compatible PSG (3 tone + 1 noise channel)
//   - Non-linear DAC with 17-level RGB output
//
// Architecture:
//   The VDP is clocked from MCLK (~53.7 MHz NTSC / ~54.2 MHz PAL).
//   A prescaler generates dot clocks (clk1/clk2) and half-rate pixel
//   clocks (hclk1/hclk2). Most rendering logic runs on hclk1/hclk2.
//   The CPU interface accepts both 68K and Z80 bus transactions.
//
// Major functional blocks (in source order):
//   1. Prescaler & clock generation — MCLK dividers, CPU/PSG clock
//   2. I/O, DMA & FIFO — CPU bus decode, register writes, DMA FSM
//   3. Timing FSM — H/V counters, sync generation, display enable
//   4. H/V counter PLAs — timing event comparators for mode variants
//   5. Scroll plane rendering — planes A/B tile fetch & pixel serialize
//   6. VSRAM — 40-entry vertical scroll RAM
//   7. Sprite processing — SAT traversal, Y-test, tile fetch, X-sort
//   8. SAT cache / sprite render data / line buffer — sprite storage
//   9. VRAM interface — RAS/CAS/WE sequencer, serial data, refresh
//  10. Video priority MUX — sprite/plane/BG priority, shadow/highlight
//  11. DAC & color output — non-linear RGB DAC, color RAM
//  12. PSG (SN76489) — 3 tone + noise channels, audio output
//  13. VRAM/IO/color bus drive — wired-AND val/pull bus patterns
//  14. Miscellaneous outputs — VDP status to fc1004, CRAM display pipe
//
// Signal naming:
//   w### = combinational wire, l### = latch/register, t### = trigger/RS-FF,
//   dff###_l2 = DFF slave output, reg_* = named config register bits,
//   pla_vcnt/pla_hcnt1/pla_hcnt2 = timing event PLAs.
//   Original opaque names are preserved (not renamed) due to the ~1800
//   signal count; grouped comments and inline annotations identify roles.

module ym7101
	(
	input MCLK,
	input [7:0] SD,
	output SE1,
	output SE0,
	output SC,
	output RAS1,
	output CAS1,
	output WE1,
	output WE0,
	output OE1,
	input [7:0] RD_i,
	output [7:0] RD_o,
	output RD_d,
	output [7:0] DAC_R,
	output [7:0] DAC_G,
	output [7:0] DAC_B,
	input [7:0] AD_i,
	output [7:0] AD_o,
	output AD_d,
	output YS,
	input SPA_B_i,
	output SPA_B_pull,
	output VSYNC,
	input CSYNC_i,
	output CSYNC_pull,
	input HSYNC_i,
	output HSYNC_pull,
	input HL,
	input SEL0,
	input PAL,
	input RESET,
	//input SEL1,
	input CLK1_i,
	output CLK1_o,
	//output CLK1_d,
	output SBCR,
	output CLK0,
	input MCLK_e,
	input EDCLK_i,
	output EDCLK_o,
	output EDCLK_d,
	input [15:0] CD_i,
	output [15:0] CD_o,
	output CD_d,
	input [22:0] CA_i,
	output [22:0] CA_o,
	output CA_d,
	output reg [15:0] SOUND,
	output INT_pull,
	output BR_pull,
	input BGACK_i,
	output BGACK_pull,
	input BG,
	input MREQ,
	input INTAK,
	output IPL1_pull,
	output IPL2_pull,
	input IORQ,
	input RD,
	input WR,
	input M1,
	input AS,
	input UDS,
	input LDS,
	input RW,
	input DTACK_i,
	output DTACK_pull,
	output UWR,
	output LWR,
	output OE0,
	output CAS0,
	output RAS0,
	output [7:0] RA,
	input ext_test_2,
	output vdp_hclk1,
	output vdp_intfield,
	output vdp_de_h,
	output vdp_de_v,
	output vdp_m5, // md mode
	output vdp_rs1, // h32/h40
	output vdp_m2, // v28/v30
	output vdp_lcb,
	output vdp_psg_clk1,
	output vdp_hsync2,
	input  vdp_cramdot_dis,
	output vdp_dma_oe_early,
	output vdp_dma
	);

	// --- Wire/register declarations ---------------------------------------------------
	// Grouped by functional area. Opaque w###/l###/t### names preserved from netlist.

	// CPU interface signals (active-high internal equivalents of active-low bus pins)
	wire cpu_sel;
	wire cpu_as;
	wire cpu_uds;
	wire cpu_lds;
	wire cpu_m1;
	wire cpu_rd;
	wire cpu_wr;
	wire cpu_mreq;
	wire cpu_iorq;
	wire cpu_rw;
	wire cpu_bg;
	wire cpu_intak;
	wire cpu_bgack;
	wire cpu_pal;
	wire cpu_pen;

	// CPU clock outputs
	wire cpu_clk0;
	wire cpu_clk1;

	// Active-high sync/sprite input inversions
	wire i_csync = ~CSYNC_i;
	wire i_hsync = ~HSYNC_i;

	wire i_spa = ~SPA_B_i;

	wire reset_ext = ~RESET;

	// Master dot clocks and half-rate pixel clocks
	wire clk1, clk2;
	wire hclk1, hclk2;
	
	// Reset
	wire reset_comb;

	// Prescaler DFF chain (MCLK divider for clk1/clk2/hclk1/hclk2)
	wire mclk_and1;
	//reg prescaler_dff1 = 1'h0;
	//reg prescaler_dff2 = 1'h0;
	//reg prescaler_dff3 = 1'h0;
	//reg prescaler_dff4 = 1'h0;
	//reg prescaler_dff5 = 1'h0;
	//reg prescaler_dff6 = 1'h0;
	//reg prescaler_dff7 = 1'h0;
	//reg prescaler_dff8 = 1'h0;
	//reg prescaler_dff9 = 1'h0;
	//reg prescaler_dff10 = 1'h0;
	//reg prescaler_dff11 = 1'h0;

	wire prescaler_dff1_l2;
	wire prescaler_dff2_l2;
	wire prescaler_dff3_l2;
	wire prescaler_dff4_l2;
	wire prescaler_dff5_l2;
	wire prescaler_dff6_l2;
	wire prescaler_dff7_l2;
	wire prescaler_dff8_l2;
	wire prescaler_dff9_l2;
	wire prescaler_dff10_l2;
	wire prescaler_dff11_l2;

	wire prescaler_dff12_l2;
	wire prescaler_dff13_l2;
	wire prescaler_dff14_l2;
	wire prescaler_dff15_l2;
	wire prescaler_dff16_l2;
	wire prescaler_dff17_l2;

	// Prescaler-derived clock selects
	wire mclk_clk1;  // dot clock phase 1 select
	wire mclk_clk2;  // dot clock phase 2 select
	wire mclk_clk3;  // inverted prescaler stage
	wire mclk_clk4;  // PAL SBCR clock candidate
	wire mclk_clk5;  // NTSC SBCR / CPU clock candidate
	wire mclk_sbcr;  // selected sub-carrier clock
	wire mclk_cpu_clk0; // CPU clock 0 source
	wire mclk_cpu_clk1; // CPU clock 1 source
	wire mclk_dclk;  // dot clock (pixel rate)
	
	// Z80 M1 cycle synchronizer chain
	wire io_m1_dff1_l2;
	wire io_m1_dff2_l2;
	wire io_m1_dff3_l2;
	wire io_m1_dff4_l2;
	wire io_m1_s1;
	wire io_m1_s2;
	wire io_m1_s3;
	wire io_m1_s4;
	wire io_m1_s5;

	// I/O bus registers and control
	reg [22:0] io_address;
	wire io_address_22o;
	wire io_oe0;   // RAM output enable
	wire cpu_wr_cas_gate;
	wire io_cas0;  // RAM CAS
	wire io_ras0;  // RAM RAS
	wire io_lwr;   // lower byte write
	wire io_uwr;   // upper byte write
	wire io_wr;    // write strobe
	wire io_ipl1;  // interrupt priority level bit 1
	wire io_ipl2;  // interrupt priority level bit 2
	reg [15:0] io_data;

	// I/O block: CPU bus decode, DTACK, register access, DMA/FIFO (cpu_wr_strobe-fifo_slot_en)
	wire cpu_wr_strobe;
	wire dff1_l2;
	wire dff2_l2;
	wire bus_br_hold;
	wire addr_hi_sel;
	wire dma_req_any;
	wire dma_rst;
	wire dff3_l2;
	wire dff4_l2;
	wire dma_copy_req;
	wire bus_granted;
	wire dma_68k_req;
	wire arb_grant_cond;
	wire edclk_pipe1;
	wire edclk_pipe2;
	wire edclk_pipe3;
	wire edclk_13_gap;
	wire edclk_gap;
	wire bus_access_gate_n;
	wire dma_addr_dly;
	wire dma_addr_hold;
	wire bus_phase_a;
	wire bus_phase_b;
	wire bus_phase_c;
	wire any_irq_active;
	wire bus_idle;
	wire ipl1_src;
	wire ipl2_src;
	wire dma_ext_gate;
	wire ras_z80_gate;
	wire cas_z80_gate;
	wire ras_dma_gate;
	wire cas_readback; // nc
	wire dma_addr_bus;
	wire oe_late_phase;
	wire ras_early;
	wire ras_gated;
	wire ras_sel;
	wire dtack_gate_n;
	wire cas_68k;
	wire oe_cpu_rd;
	wire dff5_l2;
	wire dff6_l2;
	wire dff_timing_gate;
	wire dff7_l2;
	wire z80_ras_pulse;
	wire z80_cas_pulse;
	wire dff8_l2;
	wire dff9_l2;
	wire bus_late;
	wire bus_phase2;
	wire dtack_rst_cond;
	wire dff10_l2;
	wire dff11_l2;
	wire bus_wr_phase;
	wire bus_invalid;
	wire bus_active;
	wire intak_clear;
	wire bus_valid;
	wire as_inactive;
	wire dff12_l2;
	wire bus_cycle_rst;
	wire dff13_l2;
	wire dff14_l2;
	wire dff15_l2;
	wire dtack_done;
	wire bus_allow;
	wire arb_cnt_rst;
	wire dff16_l2;
	wire dff17_l2;
	wire dff18_l2;
	wire dff19_l2;
	wire dff20_l2;
	wire dff21_l2;
	wire dff22_l2;
	wire br_pull_ctl; // BR pull control (bus request)
	wire dff23_l2;
	wire dff24_l2;
	wire dff25_l2;
	wire dff26_l2;
	wire dff27_l2;
	wire dff28_l2;
	wire dff29_l2;
	wire arb_cnt_hi;
	wire arb_cnt_top;
	wire m68k_int_ack;
	wire int_ack_any;
	wire z80_int_ack;
	wire int_ack_latch;
	wire int_ack_m5;
	wire int_ack_sync;
	wire int_ack_dly1;
	wire int_ack_dly2;
	wire int_latch_rst;
	wire status_rd_set;
	wire status_rd_pend;
	wire status_rd_dly;
	wire status_rd_gate;
	wire status_rd_dly2;
	wire status_clr_n;
	wire status_clr_latch;
	wire m4_int_en;
	wire vint_clr;
	wire dff30_l2;
	wire dff31_l2;
	wire hint_clr;
	wire dff32_l2;
	wire eint_clr;
	wire hint_irq;
	wire vint_irq;
	wire hint_pend;
	wire vint_irq_pend;
	wire vint_set_cond;
	wire eint_irq;
	wire dma_enable;
	wire dma_68k_start;
	wire dma_copy_start;
	wire bgack_pull_ctl; // BGACK pull control (bus grant acknowledge)
	wire z80_hv_sel;
	wire spr_of_latch;
	wire eint_pend;
	wire spr_overflow;
	wire spr_collision;
	wire hint_cnt_tick;
	wire hint_cnt_reload;
	wire hint_cnt_load;
	wire hint_fired;
	wire vdp_addr_68k;
	wire z80_vdp_rd;
	wire hv_byte_sel;
	wire hv_data_sel;
	wire tst18_sel_f;
	wire tst18_sel_8;
	wire tst18_sel_7;
	wire tst18_sel_6;
	wire tst18_sel_5;
	wire tst18_sel_4;
	wire tst18_sel_3;
	wire tst18_sel_2;
	wire tst18_sel_1;
	wire tst18_sel_0;
	wire tst_fn0_wr;
	wire tst_fn1_wr;
	wire tst_fn2_wr;
	wire tst_fn2_rd;
	wire tst_fn3_wr;
	wire tst_fn3_rd;
	wire tst_fn4_wr;
	wire tst_fn4_rd;
	wire tst_fn5_wr;
	wire tst_fn5_rd;
	wire tst_fn6_wr;
	wire tst_fn6_rd;
	wire tst_fn7_wr;
	wire tst_fn7_rd;
	wire tst_fn8_wr;
	wire tst_fn8_rd;
	wire int_reset_state; // internal reset state
	wire z80_addr_gate;
	wire z80_cas_cond;
	wire [7:0] ra_addr_mux; // RA[7:0] output (VRAM row address)
	wire ra_addr_valid;
	wire [7:0] color_readback;
	wire color_rd_sel;
	wire interlace_dblres; // interlace double-res mode (reg_lsm0_latch & reg_lsm1_latch)
	wire v28_mode; // V28 mode detect (~reg_m2 & reg_m5)
	wire v30_mode; // V30 mode detect (reg_m2 & reg_m5)
	wire vram_128k;
	wire z80_psg_wr;
	wire psg_wr_any;
	wire z80_data_rd;
	wire vdp_data_rd;
	wire vdp_data_rd_odd;
	wire z80_bus_ext;
	wire cas_ext_68k;
	wire dtack_pull_ctl; // DTACK pull control
	wire oe_comb;
	wire cpu_rd_latch;
	wire int_timing_sel;
	wire eint_set_cond;
	wire z80_int_rst;
	wire z80_int_trig;
	wire int_pull_ctl; // INT pull control
	wire vcnt_bit_sel;
	wire vdp_access_valid;
	wire dtack_68k;
	wire dtack_rd_gate;
	wire dtack_wr_gate;
	wire tst_reg_wr;
	wire tst_fn_wr;
	wire byte_sel;
	wire vdp_data_wr;
	wire vdp_data_port_rd;
	wire vdp_ctrl_wr;
	wire hv_cnt_rd;
	wire tst_fn_rd;
	wire access_strobe;
	wire dma_done_rst;
	wire dma_bus_cond;
	wire fifo_ctrl_pend;
	wire fifo_ctrl_set;
	wire vdp_write_any;
	wire z80_vdp_wr;
	wire z80_hv_rd;
	wire hv_rd_any;
	wire wr_byte_latch;
	wire rd_byte_latch;
	wire data_rd_first;
	wire ctrl_wr_phase, ctrl_wr_phase_n;
	wire data_wr_phase, data_wr_phase_n;
	wire fifo_wr_pend;
	wire fifo_wr_rst;
	wire dma_wr_cond;
	wire dma_inc_mode, dma_inc_mode_n;
	wire dtack_any;
	wire uds_latch;
	wire lds_latch;
	wire any_byte_sel;
	wire wr_fifo_ready;
	wire fifo_rd_cond;
	wire ctrl_wr_done;
	wire fifo_busy;
	wire cpu_data_oe; // CPU data bus drive enable
	wire cpu_rd_any;
	wire fifo_phase_a;
	wire fifo_ready;
	wire fifo_phase_b;
	wire dma_cycle_rst;
	wire dma_start_cond;
	wire access_pending;
	wire access_rst;
	wire data_rd_even;
	wire fifo_idle_rst;
	wire rd_phase_pend;
	wire rd_gate;
	wire vdp_io_any;
	wire ctrl_wr_dtack;
	wire data_wr_first;
	wire z80_data_wr;
	wire z80_ctrl_repeat;
	wire dma_auto_inc;
	wire ctrl_port_wr;
	wire fifo_advance;
	wire dma_inc_latch;
	wire data_rd_latch;
	wire fifo_write_any;
	wire ctrl_phase_rst;
	wire ctrl_phase_set;
	wire fifo_adv_rst;
	wire fifo_wr_set;
	wire data_wr_latch;
	wire fifo_flush_rst;
	wire vram_wr_sel;
	wire vsram_wr_normal;
	wire vsram_wr_test;
	wire dma_or_vram_wr;
	wire dma_addr_latch;
	wire reg_data_wr_en;
	wire dma_start_bit;
	wire fifo_pipe_rst;
	wire fifo_idle;
	wire fifo_stg4;
	wire fifo_stg3;
	wire fifo_stg2;
	wire fifo_stg1;
	wire fifo_stg0;
	wire dma_data_latch;
	wire reg_addr_load;
	wire dma_fill_trig, dma_fill_trig_n;
	wire dma_fill_start;
	wire dma_fill_dly;
	wire dma_copy_cycle;
	wire dma_fill_cycle;
	wire dma_wr_path;
	wire dma_ext_cycle;
	wire dma_data_active;
	wire code_not_vram;
	wire cram_wr_gate;
	wire auto_rd_cond;
	wire dma_ext_copy;
	wire dma_ext_dly;
	wire dma_ext_cond;
	wire dma_start_pend;
	wire dma_pend_clr;
	wire dma_pend_set;
	wire dma_copy_trig;
	wire dma_norm_trig;
	wire cram_wr_sel;
	wire vsram_wr_sel;
	wire vram_wr_pipe1;
	wire vram_wr_pipe2;
	wire sms_rd_trig;
	wire code_hi_rst;
	wire reg_wr_pipe;
	wire cram_gate_pipe;
	wire cram_rd_pipe2;
	wire cram_rd_active;
	wire reg_wr_strobe;
	wire reg_sel_grp0;
	wire reg_sel_grp1;
	wire reg_sel_grp2;
	wire reg_sel_grp1m5;
	wire wr_en_8F;
	wire wr_en_93;
	wire wr_en_94;
	wire wr_en_8B_scr;
	wire wr_en_96;
	wire wr_en_8C;
	// Register write strobes (active for one cycle when CPU writes reg 0x80-0x97)
	wire reg_wr_80; // reg 0x80 write strobe
	wire reg_wr_81; // reg 0x81 write strobe
	wire reg_wr_82; // reg 0x82 write strobe
	wire reg_wr_83; // reg 0x83 write strobe
	wire reg_wr_84; // reg 0x84 write strobe
	wire reg_wr_85; // reg 0x85 write strobe
	wire reg_wr_86; // reg 0x86 write strobe
	wire reg_wr_87; // reg 0x87 write strobe
	wire reg_wr_88; // reg 0x88 write strobe
	wire reg_wr_89; // reg 0x89 write strobe
	wire reg_wr_8A; // reg 0x8A write strobe
	wire reg_wr_8B; // reg 0x8B write strobe
	wire reg_wr_8C; // reg 0x8C write strobe
	wire reg_wr_8D; // reg 0x8D write strobe
	wire reg_wr_hi; // reg 0x8E-0x97 write range
	wire wr_en_88;
	wire wr_en_8E;
	wire wr_en_8D_m5;
	wire dma_cnt_tst;
	wire dma_cnt_norm;

	// VRAM address pipeline latches (DMA/FIFO address stages, 17-bit)
	wire [16:0] fifo_addr_pipe;
	wire [16:0] fifo_data_0;
	wire [16:0] fifo_data_1;
	wire [16:0] fifo_data_2;
	wire [16:0] fifo_data_3;
	wire dma_len_last;
	wire dma_copy_mode;
	wire dma_68k_mode;
	wire dma_fill_mode;
	wire dma_ext_mode;
	wire dma_addr_pipe;
	wire dma_addr_edge;
	wire dma_addr_latch2;
	wire dma_wr_active;
	wire dma_len_hi_cry;
	wire fifo_wr_pipe1;
	wire fifo_wr_gate;
	wire fifo_wr_pipe2;
	wire dma_slot_pipe1;
	wire dma_slot_pipe2;
	wire dma_slot_edge;
	wire dma_state_set;
	wire dma_src_hi_cry;
	wire dma_state;
	wire fifo_empty_slot;
	wire fifo_rd_s2;
	wire fifo_rd_s0;
	wire fifo_rd_s3;
	wire fifo_rd_s1;
	wire fifo_rd_trigger;
	wire dma_ext_busy; // nc
	wire dma_ext_state;
	wire fifo_wr_trig;
	wire dma_copy_active;
	wire vram_addr_b0_lat;
	wire addr_latch_en;
	wire dma_addr_oe; // DMA address bus drive enable
	wire dma_len_done_dly;
	wire non_dma_slot;
	wire slot_no_dma;
	wire idle_slot_dly;
	wire dma_active_rst;
	wire dma_active_set;
	wire dma_active_trig; // DMA active trigger
	wire dma_fill_slot_dly;
	wire dma_fill_cond;
	wire fifo_vram_sel;
	wire vram_sel;
	wire dma_fill_busy;
	wire uds_or_dff3;
	wire byte_sel_hi;
	wire lds_or_dff3;
	wire byte_sel_lo;
	wire addr_b0_m5;
	wire dma_cpy_hi_sel;
	wire dma_cpy_lo_sel;
	wire vram_wr_hi_en;
	wire vram_wr_lo_en;
	wire lo_byte_valid;
	wire vram_wr_lo_gate;
	wire vram128k_hi;
	wire vram_wr_hi_gate;
	wire vram_wr_active;
	wire fifo_slot_en;
	wire [1:0] fifo_wr_ptr;
	wire fifo_en_s2;
	wire fifo_en_s3;
	wire fifo_en_s0;
	wire fifo_en_s1;
	wire fifo_ptr_match;
	wire fifo_cnt_inc;
	wire fifo_cnt_dec;
	wire fifo_cnt_b0;
	wire fifo_rd_b0;
	wire fifo_rd_b1;
	wire fifo_not_busy;
	wire fifo_addr_wr_en;
	wire fifo_read_slot;
	wire fifo_uds_s3;
	wire fifo_uds_s2;
	wire fifo_uds_s1;
	wire fifo_uds_s0;
	wire fifo_lds_s3;
	wire fifo_lds_s2;
	wire fifo_lds_s1;
	wire fifo_lds_s0;
	wire fifo_cd0_s3;
	wire fifo_cd0_s2;
	wire fifo_cd0_s1;
	wire fifo_cd0_s0;
	wire fifo_cd1_s3;
	wire fifo_cd1_s2;
	wire fifo_cd1_s1;
	wire fifo_cd1_s0;
	wire fifo_cd2_s3;
	wire fifo_cd2_s2;
	wire fifo_cd2_s1;
	wire fifo_cd2_s0;
	wire fifo_cd3_s3;
	wire fifo_cd3_s2;
	wire fifo_cd3_s1;
	wire fifo_cd3_s0;
	wire fifo_out_active;
	wire cram_wr_hi_cond;
	wire cram_wr_lo_cond;
	wire fifo_data_wr_en;
	wire fifo_rd_valid;
	wire rd_ptr_3;
	wire rd_ptr_2;
	wire rd_ptr_1;
	wire rd_ptr_0;
	wire fifo_out_cd1;
	wire fifo_out_uds;
	wire fifo_out_cd2;
	wire fifo_out_lds;
	wire fifo_out_cd3;
	wire fifo_out_cd0;
	wire out_cd1_sel;
	wire out_uds_sel;
	wire out_cd2_sel;
	wire out_lds_sel;
	wire out_cd3_sel;
	wire out_cd0_sel;
	wire out_cd0_raw;
	wire vsram_wr_hi;
	wire vsram_wr_lo;
	wire vram_code_sel;
	wire data_rd_s0;
	wire data_rd_s2;
	wire data_rd_s1;
	wire data_rd_s3;
	wire byte_swap_gate;
	wire fifo_byte_sel;
	wire uds_lds_diff;
	wire hv_cnt_sel;
	wire hv_latch_ext;
	wire pen_dly1;
	wire pen_dly2;
	wire pen_fall_edge;
	wire pen_edge_dly;
	wire vcnt_latch_en;
	wire hcnt_latch_en;
	wire vram_direct;
	wire vram_wr_hi_pipe;
	wire addr_b0_dly1;
	wire addr_b0_dly2;
	wire vram_wr_lo_cond;
	wire vram_wr_hi_cond;
	wire addr_b0_dly3;
	wire dma_wr_dly1;
	wire dma_wr_dly2;
	wire vram_wr_hi_strb;
	wire vram_wr_lo_strb;
	wire vram_wr_hi_sel;
	wire vram_wr_lo_sel;
	wire vram_wr_hi_clk;
	wire vram_wr_lo_clk;
	wire vram128k_rd_sel;
	wire vram_byte_swap;
	// VRAM data pipeline latches (4 pairs of 8-bit read stages: vcnt_latch-data_rd_pipe)
	wire [7:0] vcnt_latch;
	wire [7:0] hcnt_latch;
	wire [7:0] hv_cnt_data;
	wire [7:0] vram_rd_lo_data;
	wire [7:0] vram_rd_lo_lat;
	wire [7:0] vram_rd_hi_data;
	wire [7:0] vram_rd_hi_lat;
	wire [7:0] cpu_data_hi;
	wire [7:0] fifo_in_hi;
	wire [7:0] fifo_in_lo;
	wire [7:0] fifo_hi_s1;
	wire [7:0] fifo_lo_s1;
	wire [7:0] fifo_out_s1;
	wire [7:0] unk_data;
	wire [7:0] fifo_hi_s2;
	wire [7:0] fifo_lo_s2;
	wire [7:0] fifo_out_s2;
	wire [7:0] fifo_hi_s3;
	wire [7:0] fifo_lo_s3;
	wire [7:0] fifo_out_s3;
	wire [7:0] fifo_hi_s0;
	wire [7:0] fifo_lo_s0;
	wire [7:0] fifo_out_s0;
	wire [7:0] data_rd_pipe;
	
	// H/V counters and timing FSM (vcnt-vsync_pipe_prev, vcnt_ext-disp_active_flag)
	wire [8:0] vcnt; // vertical counter (9-bit)
	wire [9:0] vcnt_ext; // extended vcnt (interlace: {vcnt, field}, normal: {0, vcnt})
	wire [8:0] hcnt; // horizontal counter (9-bit)
	wire field_trig_dly8;
	wire edclk_dly;
	wire slot_idle_dly;
	wire line_zero_dly;
	wire slot3_active;
	wire tst_mode_1;
	wire no_slot_123e;
	wire slot_idle;
	wire disp_start;
	wire hint_time_dly;
	wire spr_end_dly;
	wire hcnt_load_en; // hcnt load enable
	wire hcnt_inc_gate;
	wire hcnt_inc_en; // hcnt increment enable
	wire [8:0] hcnt_load_val; // hcnt load value
	wire hcnt_rld_b0;
	wire hcnt_rld_b1;
	wire hcnt_rld_b2;
	wire hcnt_rld_b3;
	wire hcnt_rld_b3_aux;
	wire hcnt_m5_reload;
	wire line_run_prev;
	wire spr_mid_dly;
	wire active_end;
	wire odd_slot;
	wire tst_mode_2;
	wire slot2_active;
	wire not_disp_start_dly;
	wire slot3_dly;
	wire slot2_dly;
	wire field_inv;
	wire csync_src;
	wire csync_dly;
	wire csync_combined;
	wire line_run_next;
	wire line_not_reset;
	wire line_running;
	wire adj_point_dly;
	wire render_start_dly;
	wire vscr_or_disp;
	wire vscr_active;
	wire tst_normal;
	wire tst_mode_5;
	wire tst_mode_3;
	wire tst_mode_4;
	wire odd_slot_gate;
	wire slot1_active;
	wire no_slot0_odd_ext;
	wire px8_bound_dly;
	wire slot1_dly;
	wire slot0_dly;
	wire csync_m5_mux;
	wire csync_xor_dly8;
	wire hv_disp_active;
	wire csync_out_dly;
	wire full_disp_en;
	wire vint_latch_dly;
	wire m5_vblank_or_vsync;
	wire m5_field_hblank;
	wire sync_area_dly;
	wire hdisp_rst;
	wire hdisp_en_trig; // horizontal display enable trigger
	wire hblank_end_dly;
	wire cell_m4_active;
	wire reload_pulse_dly;
	wire slot0_active;
	wire no_disp_main;
	wire cell_m4_dly;
	wire access_ext_dly;
	wire csync_ext_mux;
	wire csync_sel_dly7;
	wire hsync_out_dly;
	wire csync_pull_gate;
	wire no_m5_vert_event;
	wire vert_fetch_gate;
	wire vblank_vsync_fetch;
	wire vint_pending;
	wire vint_set;
	wire scroll_end_dly;
	wire access_win_dly;
	wire cell_bound_active;
	wire reload_edge_dly;
	wire hcnt_reload_pulse;
	wire cell_bound_dly;
	wire access_main_dly;
	wire m4_border_dly;
	wire hsync_pull_gate;
	wire hsync_no_vert_a;
	wire hsync_no_vert_b;
	wire hblank_state;
	wire hblank_set;
	wire hblank_rst;
	wire hblank_end_comb;
	wire scroll_start_dly;
	wire fetch_point_dly;
	wire hcnt_reload_edge;
	wire vram_slot_dly;
	wire m4_border2_dly;
	wire fetch_all_dly;
	wire blank_start_dly;
	wire hsync_ext_in;
	wire hsync_in_dly;
	wire csync_comb_a_dly;
	wire csync_comb_a;
	wire csync_comb_b;
	wire csync_comb_b_dly;
	wire between_fetches;
	wire fetch_phase_rst;
	wire line_first_dly;
	wire hint_pos_dly;
	wire pre_wrap_active;
	wire h40_mode;
	wire m4_or_vram_slot;
	wire vram_or_ext_slot;
	wire blank_slot_active;
	wire pre_wrap_dly;
	wire ext_access_dly;
	wire m4_window_dly;
	wire csync_xor_field;
	wire csync_div_sel;
	wire csync_comb_a_dly2;
	wire vint_delayed;
	wire vint_pend_dly8;
	wire hsync_trig; // horizontal sync trigger
	wire hsync_trig_set;
	wire sub_slot_active;
	wire h40_no_ext_latch;
	wire sub_slot_dly;
	wire csync_div_bit;
	wire csync_div_inc;
	wire csync_xor_prev;
	wire csync_prog_mode;
	wire [8:0] vcnt_load_val; // vcnt load value (mode/region-dependent)
	wire vcnt_rld_b5;
	wire ntsc_v28;
	wire pal_m4;
	wire pal_v28;
	wire vcnt_rld_b3;
	wire pal_even_field;
	wire field_xor_region;
	wire vcnt_inc_en; // vcnt increment enable
	wire vcnt_load_en; // vcnt load enable
	wire vcnt_wrap_cond;
	wire vcnt_at_max; // display active region flag
	wire active_disp_gate; // active display gating (~(reg_disp & (vcnt_at_max | vdisp_en_trig)))
	wire frame_end_dly;
	wire vblank_hi_rst;
	wire vblank_hi_latch;
	wire m5_vblank_hi;
	wire vblank_hi_dly;
	wire frame_end_gated;
	wire field_bit_trig; // field bit trigger
	wire m5_field_active;
	wire vblank_lo_dly;
	wire vblank_hi_gated;
	wire field_trig_rst;
	wire vsync_latch;
	wire field_bit; // interlace field bit (even/odd frame)
	wire vsync_area_dly;
	wire vblank_lo_gated;
	wire vsync_latch_rst;
	wire hint_or_even;
	wire field_set_dly;
	wire m5_vsync;
	wire field_bit_rst;
	wire field_toggle_dly;
	wire field_toggle_gated;
	wire vsync_gate_set;
	wire vsync_gate;
	wire field_adj_h40;
	wire field_adj_h32;
	wire field_set_cond;
	wire vdisp_active_dly;
	wire vdisp_ended;
	wire vdisp_en_rst;
	wire vdisp_end_at_zero;
	wire vdisp_en_trig; // vertical display enable trigger
	wire spr_mid_dly2;
	wire vcnt_at_zero;
	wire field_adj_en;
	wire vcnt_adj_reload;
	wire field_adj_rst;
	wire adj_point_dly2;
	wire vcnt_eq_dly;
	wire not_vsync_rst;
	wire csync_adj_comb;
	wire vsync_pipe_next;
	wire vsync_pipe;
	wire vsync_pipe_prev;
	wire field_adj_set;
	wire disp_active_flag;
	// H/V counter PLAs — timing event comparators for mode variants
	wire [47:0] pla_vcnt;  // 48 vertical timing events
	wire [62:0] pla_hcnt1; // 63 horizontal timing events (group 1)
	wire [45:0] pla_hcnt2; // 46 horizontal timing events (group 2)
	wire vcnt_eq_pulse;
	wire vcnt_not_zero;
	wire vcnt_vdisp_active;
	wire vcnt_field_toggle;
	wire vcnt_vsync_area;
	wire vcnt_vblank_lo;
	wire vcnt_vblank_hi;
	wire vcnt_frame_end;
	wire vcnt_not_max;
	wire hcnt_edclk;
	wire hcnt_slot3;
	wire hcnt_slot2;
	wire hcnt_slot1;
	wire hcnt_slot0;
	wire hcnt_access_ext;
	wire hcnt_access_main;
	wire hcnt_odd;
	wire hcnt_m4_window;
	wire hcnt_ext_access;
	wire hcnt_disp_active;
	wire hcnt_fetch_all;
	wire hcnt_blank_start;
	wire hcnt_m4_border;
	wire hcnt2_spr_end;
	wire hcnt2_spr_mid;
	wire hcnt2_adj_point;
	wire hcnt2_sync_area;
	wire hcnt2_scroll_end;
	wire hcnt2_scroll_start;
	wire hcnt2_line_first;
	wire hcnt2_hint_pos;
	wire hcnt2_fetch_point;
	wire hcnt2_access_win;
	wire hcnt2_hblank_end;
	wire hcnt2_render_start;
	wire hcnt2_not_active_end;
	wire hcnt2_hint_time;
	wire hcnt2_line_zero;
	wire hcnt2_not_disp_start;
	wire hcnt2_8px_bound;
	wire hcnt2_cell_m4;
	wire hcnt2_cell_bound;
	wire hcnt2_m4_border;
	wire hcnt2_vram_slot;
	wire hcnt2_pre_wrap;
	wire hcnt2_sub_slot;
	wire field_trig_pipe;

	// Scroll plane rendering: tilemap address, tile fetch, pixel serialization
	wire w513;
	wire l178;
	wire w514;
	wire [7:0] l179; // H-scroll data latch (from reg write)
	wire [10:0] w515; // 11 bits
	wire [10:0] l180; // VSRAM read latch (11 bits)
	wire [10:0] l181;
	wire [2:0] l182;
	wire l183; // VSRAM data bus drive
	wire w516;
	wire w517;
	wire [10:0] l184;
	wire [10:0] l185;
	wire l186;
	wire [10:0] w518;
	wire w519;
	wire w520;
	wire [10:0] w521;
	wire [10:0] w522;
	wire [1:0] reg_hsz; // horizontal scroll size (00=32, 01=64, 11=128)
	wire [1:0] reg_vsz; // vertical scroll size (00=32, 01=64, 11=128)
	wire w523;
	wire w524;
	wire w525;
	wire [6:0] w526;
	wire [6:0] w527;
	wire [6:0] w528;
	wire w529;
	wire w530;
	wire w531;
	// Nametable base address registers
	wire [3:0] reg_sa; // plane A nametable base (reg 0x82)
	wire [1:0] reg_nt; // m4 nametable (reg 0x82, SMS mode)
	wire [3:0] reg_sb; // plane B nametable base (reg 0x84)
	wire [3:0] w532;
	wire [1:0] w533;
	wire reg_8e_b0;
	wire reg_8e_b4;
	wire w534;
	wire [7:0] w535; // 8 bits
	wire [5:0] w536; // 6 bits
	// Window plane registers
	wire [5:0] reg_wd; // window nametable base (reg 0x83)
	wire [6:0] reg_hs; // H-scroll table base (reg 0x8D)
	wire [7:0] w537;
	wire w538;
	wire w539;
	wire w540;
	wire w541;
	wire [4:0] reg_whp; // window H position (reg 0x91)
	wire reg_rigt;       // window right flag (reg 0x91 bit 7)
	wire [4:0] reg_wvp; // window V position (reg 0x92)
	wire reg_down;       // window down flag (reg 0x92 bit 7)
	wire w542;
	wire w543;
	wire w544;
	wire l187;
	wire [4:0] l188;
	wire l189;
	wire [4:0] l190;
	wire w545;
	wire w546;
	wire w547;
	wire [7:0] reg_88; // m4 scroll
	wire [7:0] l191;
	wire [7:0] l192;
	wire w548;
	wire w549;
	wire w550;
	wire w551;
	wire w552;
	wire w553;
	wire [7:0] l193;
	wire [9:0] w554;
	wire [1:0] l194;
	wire [1:0] l195;
	wire [6:0] w555;
	wire l196;
	wire l197;
	wire l198;
	wire l199;
	wire w556;
	wire l200;
	wire w557;
	wire l201;
	wire w558;
	wire w559;
	wire l202;
	wire w560;
	wire l203;
	wire l204;
	wire l205;
	wire w561;
	wire w562;
	wire w563;
	wire w564;
	wire [3:0] w565;
	wire w566;
	wire [1:0] w567;
	wire w568;
	wire l206;
	wire l207;
	wire l208;
	wire l209;
	wire l210;
	wire l211;
	wire w569;
	wire [5:0] l212; // 6 bits
	wire l213;
	wire w570;
	wire l214;
	wire w571;
	wire l215;
	wire w572;
	wire l216;
	wire w573;
	wire w574;
	wire w575;
	wire [3:0] w576;
	wire [3:0] w577;
	wire [8:0] w578;
	wire [2:0] w579;
	wire [11:0] w580;
	wire w581;
	wire l217;
	wire w582;
	wire l218;
	wire w583;
	// Plane A tile pixel shift registers (4 × 8-bit bitplanes)
	wire [7:0] l219;
	wire [7:0] l220;
	wire [7:0] l221;
	wire [7:0] l222;
	wire w584;
	wire w585;
	wire [1:0] w586;
	wire w587;
	// Plane A tile attribute pipeline (4 × 8-bit)
	wire [7:0] l223;
	wire [7:0] l224;
	wire [7:0] l225;
	wire [7:0] l226;
	wire l227;
	wire l228;
	wire l229;
	wire l230;
	wire l231;
	wire w588;
	wire w589;
	wire w590;
	wire w591;
	wire [7:0] l232;
	wire [7:0] l233;
	wire [7:0] l234;
	wire [7:0] l235;
	wire w592;
	wire [3:0] l236;
	wire w593;
	wire [7:0] l237;
	wire [7:0] l238;
	wire [7:0] l239;
	wire [7:0] l240;
	wire [3:0] l241;
	wire l242;
	wire [7:0] l243;
	wire [7:0] l244;
	wire [7:0] l245;
	wire [7:0] l246;
	wire l247;
	wire l248;
	wire l249;
	wire l250;
	wire w594;
	wire w595;
	wire w596;
	wire w597;
	wire l251;
	wire l252;
	wire l253;
	wire l254;
	wire [7:0] l255;
	wire [7:0] l256;
	wire [7:0] l257;
	wire [7:0] l258;
	wire [7:0] l259;
	wire [7:0] l260;
	wire [7:0] l261;
	wire [7:0] l262;
	wire [7:0] l263;
	wire [7:0] l264;
	wire [7:0] l265;
	wire w598;
	wire w599;
	wire w600;
	wire w601;
	wire w602;
	wire w603;
	wire w604;
	wire w605;
	wire [2:0] w606;
	wire [3:0] w607;
	wire w608;
	wire w609;
	wire w610;
	wire l266;
	wire w611;
	wire w612;
	wire l267;
	wire l268;
	wire w613;
	wire [3:0] l269;
	wire [3:0] l270;
	wire [1:0] l271;
	wire [1:0] l272;
	wire l273;
	wire l274;
	wire w614;
	// Plane B tile pixel shift registers + attribute pipeline (8 × 8-bit)
	wire [7:0] l275;
	wire [7:0] l276;
	wire [7:0] l277;
	wire [7:0] l278;
	wire [7:0] l279;
	wire [7:0] l280;
	wire [7:0] l281;
	wire [7:0] l282;
	wire w615;
	wire w616;
	wire w617;
	wire w618;
	wire l283;
	wire l284;
	wire l285;
	wire l286;
	wire w619;
	wire w620;
	wire w621;
	wire w622;
	wire [7:0] l287;
	wire [7:0] l288;
	wire [7:0] l289;
	wire [7:0] l290;
	wire [7:0] l291;
	wire [7:0] l292;
	wire w623;
	wire [5:0] l293;
	wire w624;
	wire [4:0] w625;
	wire [5:0] w626;
	wire [7:0] l294;
	wire [7:0] l295;
	wire [7:0] l296;
	wire [7:0] l297;
	wire l298;
	wire l299;
	wire l300;
	wire l301;
	wire l302;
	wire w627;
	wire w628;
	wire w629;
	wire w630;
	wire w631;
	wire [7:0] l303;
	wire [7:0] l304;
	wire [7:0] l305;
	wire [7:0] l306;
	wire [7:0] l307;
	wire [7:0] l308;
	wire [7:0] l309;
	wire [7:0] l310;
	wire [3:0] l311;
	wire [2:0] w632;
	wire w633;
	wire w634;
	wire w635;
	wire w636;
	wire w637;
	wire w638;
	wire l312;
	wire l313;
	wire w639;
	wire w640;
	wire [2:0] w641;
	wire w642;
	wire l314;
	wire l315;
	wire l316;
	wire w643;
	wire w644;
	wire l317;
	wire w645;
	wire w646;
	wire [3:0] w647;
	wire [3:0] l318;
	wire [3:0] l319;
	wire w648;
	wire l320;
	wire l321;
	wire [1:0] l322;
	wire [1:0] l323;
	wire w649;

	wire [10:0] sat_read_mux;
	wire [10:0] sat_field_latch;
	wire sat_read_phase;
	wire sat_rd_pipe_0;
	wire [10:0] sat_attr_pipe;
	wire [10:0] sat_attr_hold;
	wire sat_field_latch_en;
	wire sat_rd_pipe_1;
	wire sat_rd_pipe_2;
	wire sat_rd_pipe_3;
	wire [10:0] sat_attr_latch;
	wire sat_active_n;
	wire sat_attr_latch_en;
	wire [9:0] spr_y_adjusted;
	wire m5_single_res;
	wire sat_link_is_7f;
	wire sat_ybit8_valid;
	wire sat_y_sign;
	wire sat_stop;
	wire sat_clk_gate;
	wire sat_clk_latch;
	wire sat_ypos_strobe_a;
	wire sat_clk_pipe;
	wire sat_ypos_strobe_b;
	wire [9:0] y_delta;
	wire sat_vs1_pipe;
	wire sat_vs0_pipe;
	wire sat_ybit8_pipe;
	wire sat_ysign_pipe;
	wire y_size_test_0;
	wire y_size_test_1;
	wire spr_y_visible;
	wire [9:0] sat_y_mux;
	wire [9:0] y_cmp_val;
	wire [9:0] y_delta_result;
	wire y_d0_terminate;
	wire y_range_bit_0;
	wire y_range_bit_1;
	wire y_range_bit_2;
	wire y_range_bit_3;
	wire y_range_bit_4;
	wire [7:0] sat_vram_serial;
	wire [9:0] sat_ypos_latch;
	wire [9:0] sat_ypos_hold;
	wire y_size_adj_lo;
	wire y_size_adj_hi;
	wire y_size_adj_ext;
	wire sat_rd_window;
	wire m4_ypos_sel;
	wire sat_busy;
	wire sat_active;
	wire [9:0] sat_ypos_pipe_0;
	wire [9:0] sat_ypos_pipe_1;
	wire [9:0] sat_ypos_src;
	wire sat_start_pipe_0;
	wire sat_start_pipe_1;
	wire sat_start_pipe_2;
	wire sat_start_pipe_3;
	wire sat_rd_win_m5;
	wire sat_rd_win_m4;
	wire [6:0] sat_link_cnt;
	wire link_cnt_load;
	wire link_ld_pipe;
	wire link_cnt_inc;
	wire link_timing;
	wire link_cnt_rst;
	wire link_rst_comb;
	wire link_rst_edge_0;
	wire link_rst_edge_1;
	wire sat_start_trig;
	wire sat_start_latch;
	wire sat_rd_pipe_4;
	wire sat_rd_pipe_5;
	wire sat_wr_link;
	wire sat_wr_size;
	wire sat_wr_yhi;
	wire sat_wr_ylo;
	wire sat_h40_bit;
	wire sat_addr_bit1;
	wire [5:0] sat_addr_mid;
	wire sat_addr_pipe_0;
	wire sat_addr_pipe_1;
	wire sat_wr_active;
	wire sat_wr_trig_a;
	wire sat_wr_trig_b;
	wire sat_wr_phase_a;
	wire sat_wr_phase_b;
	wire [6:0] sat_index_mux;
	wire [6:0] sat_link_idx;
	wire [4:0] sat_link_delay;
	// SAT field decodes
	wire [6:0] sat_link; // sprite link (next sprite index)
	wire [3:0] sat_size; // sprite size {HS1, HS0, VS1, VS0}
	wire [9:0] sat_ypos; // sprite Y position
	wire [3:0] sat_size_serial;
	wire [4:0] sprdata_wr_idx;
	wire tile_done_pipe;
	wire spr_cnt_full_det;
	wire spr_cnt_full_pipe;
	wire spr_render_end_pipe;
	wire spr_idle_pipe;
	wire spr_render_end;
	wire spr_idle;
	wire [4:0] tile_offset_cnt;
	wire hblank_pipe;
	wire tile_cnt_rst_comb;
	wire tile_cnt_rst;
	wire tile_cnt_en;
	wire tile_cnt_inc;
	wire tile_cnt_dec;
	wire test_sel_ne2;
	wire test_sel_ne1;
	wire test_sel_ne0;
	wire test_wr_attr;
	wire sprdata_wr_attr;
	wire test_wr_hpos;
	wire test_wr_pat;
	wire sprdata_wr_pat;
	wire sprdata_wr_norm;
	wire render_wr_pipe_0;
	wire render_wr_pipe_1;
	wire spr_limit_m4_pipe;
	wire spr_limit_m4_n;
	wire sprdata_wr_hpos;
	wire render_trig_comb;
	wire render_trig_pipe;
	wire spr_found_any;
	wire spr_found_pipe;
	wire spr_attr_latch_en;
	wire [3:0] spr_size_latch;
	wire [5:0] spr_yoff_latch;
	wire spr_found_full;
	wire spr_found_avail;
	wire spr_stop_or_ovfl;
	wire line_start_pipe;
	wire spr_found_trig;
	wire spr_overflow_trig;
	wire yoff_carry_0;
	wire yoff_carry_1;
	wire yoff_carry_2;
	wire [1:0] yoff_size_hi;
	wire [1:0] yoff_add_hi;
	wire render_active_pipe;
	wire sat_avail_pipe;
	wire sat_avail_n;
	wire sat_stop_pipe;
	wire spr_overflow_pipe;
	wire [3:0] yoff_flip_lo;
	wire [2:0] yoff_final_hi;
	wire [1:0] yoff_xor_mid;
	wire [5:0] yoff; // sprite Y offset into current tile row
	wire [7:0] vram_attr_lo;
	wire [7:0] vram_attr_hi;
	wire [7:0] vram_attr_sel;
	wire spr_cnt_shift_en;
	wire [9:0] spr_cnt_sr_0;
	wire [9:0] spr_cnt_sr_1;
	wire [9:0] spr_cnt_sr_2;
	wire [9:0] spr_cnt_sr_3;
	wire spr_cnt_bit_0;
	wire spr_cnt_bit_1;
	wire spr_cnt_bit_2;
	wire spr_cnt_bit_3;
	wire spr_cnt_m4_bit;
	wire spr_cnt_wr_en;
	wire [3:0] spr_cnt_data;
	wire reg_86_b2;
	wire reg_86_b5;
	wire [7:0] reg_at;
	wire sat_addr_match;
	wire sat_wr_match;
	wire m4_attr_phase;
	wire spr_count_limit;
	wire vram_strobe_pipe_0;
	wire vram_strobe_0;
	wire vram_strobe_1;
	wire vram_strobe_2;
	wire vram_strobe_pipe_1;
	wire vram_strobe_lat_0;
	wire vram_gate_0;
	wire vram_gate_1;
	wire vram_strobe_lat_1;
	wire vram_strobe_3;
	wire fetch_active_pipe;
	wire fetch_delay_pipe;
	wire fetch_active_sel;
	wire [6:0] sat_idx_pipe_0;
	wire [6:0] sat_idx_pipe_1;
	wire [6:0] sat_idx_pipe_2;
	wire attr_phase_any;
	wire attr_phase_0;
	wire attr_phase_1;
	wire [6:0] spridx_data_mux;
	wire spridx_ovfl_mux;
	wire [19:0] spridx_sr_0;
	wire [19:0] spridx_sr_1;
	wire [19:0] spridx_sr_2;
	wire [19:0] spridx_sr_3;
	wire [19:0] spridx_sr_4;
	wire [19:0] spridx_sr_5;
	wire [19:0] spridx_sr_6;
	wire [19:0] spridx_sr_active;
	wire fetch_timing_0;
	wire fetch_timing_1;
	wire m4_fetch_active;
	wire m4_fetch_phase;
	wire m5_fetch_phase;
	wire [6:0] spridx_readback;
	wire sprdata_in_hflip;
	wire sprdata_test_sel;
	wire sprdata_norm_sel;
	wire [1:0] sprdata_in_pal;
	wire sprdata_in_pri;
	wire [1:0] sprdata_in_xs;
	wire [1:0] sprdata_in_ys;
	wire [5:0] sprdata_in_yoff;
	wire sprdata_rd_strobe;
	wire [10:0] sprdata_test_mux;
	wire spr_rd_hflip;
	wire [1:0] spr_rd_pal;
	wire spr_rd_pri;
	wire [1:0] spr_rd_xs;
	wire [1:0] spr_rd_ys;
	wire [5:0] spr_rd_yoff;
	// Sprite render data outputs (read from sprdata[] buffer)
	wire [10:0] sprdata_pattern_o; // tile pattern index
	wire [8:0] sprdata_hpos_o;     // X position on screen
	wire sprdata_hflip_o;          // horizontal flip
	wire [1:0] sprdata_pal_o;      // palette select
	wire sprdata_priority_o;       // priority over planes
	wire [1:0] sprdata_xs_o;       // horizontal size (tiles)
	wire [1:0] sprdata_ys_o;       // vertical size (tiles)
	wire [5:0] sprdata_yoffset_o;  // Y offset within sprite
	wire tile_fetch_pipe;
	wire [1:0] xtile_cnt;
	wire xtile_cnt_zero;
	wire xtile_done;
	wire spr_line_cnt;
	wire spr_line_cnt_pipe;
	wire spr_render_active;
	wire spr_render_pipe;
	wire tile_data_valid;
	wire [10:0] sprdata_in_pat;
	wire [10:0] spr_rd_pattern;
	wire [8:0] sprdata_in_hpos;
	wire [8:0] spr_rd_hpos;
	wire sprdata_wr_ready;
	wire sprdata_ready_comb;
	wire tile_start_comb;
	wire tile_start_pipe; // nc
	wire [3:0] yoff_size_sel;
	wire [3:0] yoff_size_add;
	wire [10:0] pattern_addr;
	wire m5_fetch_pipe;
	wire m5_fetch_comb;
	wire spr_hpos_nonzero;
	wire spr_hpos_valid;
	wire fetch_done_pipe;
	wire yoff_rst_comb;
	wire fetch_pipe_0;
	wire tile_row_done;
	wire yoff_rst_latch;
	wire yoff_active;
	wire [1:0] yoff_ys_mux;
	wire [3:0] yoff_accum;
	wire [3:0] yoff_add_val;
	wire [3:0] yoff_accum_pipe;
	wire [7:0] vram_serial_a;
	wire [7:0] vram_serial_b;
	wire [7:0] m4_pattern_data;
	wire [7:0] pattern_serial_sel;
	wire tile_data_strobe;
	wire hflip_delay, hflip_delay_1;
	wire [1:0] pal_delay, pal_delay_1;
	wire pri_delay, pri_delay_1;
	wire [1:0] xs_delay, xs_delay_1;
	wire [8:0] hpos_delay, hpos_delay_1;
	wire xaddr_load_pipe;
	wire xaddr_idle;
	wire xaddr_dec;
	wire xaddr_done_pipe;
	wire xaddr_adj_0;
	wire xaddr_xs0_m5;
	wire xaddr_xs1_m5;
	wire xaddr_adj_1;
	wire hflip_m5;
	wire m4_xflip;
	wire xaddr_sign;
	wire xaddr_inc;
	wire [8:0] xpos_mux;
	wire [5:0] xaddr_init;
	wire [5:0] xaddr_pipe_a;
	wire [5:0] xaddr_pipe_b;
	wire [5:0] xaddr_sel;
	wire [5:0] xaddr_accum;
	wire pixel_stage_strobe;
	wire xpos_hflip;
	wire xpos_hflip_pipe;
	wire xpos_pri_latch;
	wire xpos_pri;
	wire [1:0] xpos_pal_latch;
	wire [1:0] xpos_pal;
	wire pixel_done_latch;
	wire tile_strobe_pipe_0;
	wire tile_strobe_pipe_1;
	wire tile_group_done;
	wire tile_group_strobe;
	wire xsize_init_lo;
	wire [1:0] tile_group_cnt;
	wire xsize_init_hi;
	wire tile_group_cnt_zero;
	wire [8:0] disp_x_offset;
	wire xbounds_pipe;
	wire xaddr_in_bounds;
	wire lb_pri_out;
	wire [1:0] lb_pal_out;
	wire pixel_active_pipe_0;
	wire pixel_active_pipe_1;
	wire pixel_dir_xor;
	wire pixel_direction;
	wire lb_write_valid;
	wire lb_write_valid_n;
	wire lb_write_en;
	wire lb_write_pipe;
	wire lb_write_window;
	wire lb_clear_pipe;
	wire lb_write_any;
	wire lb_read_strobe_n;
	wire lb_read_latch;
	wire lb_read_gate;
	wire lb_read_active_n;
	wire lb_read_active_comb;
	wire disp_x_bit_2;
	wire disp_x_bit_1;
	wire disp_x_bit_0;
	wire disp_active;
	wire disp_active_pipe;
	wire disp_active_latch;
	wire [2:0] pixel_col_pipe;
	wire [2:0] pixel_col_latch;
	wire lb_sel_spr;
	wire lb_sel_disp;
	wire lb_sel_test;
	wire [5:0] xaddr_disp_pipe;
	wire [5:0] disp_addr_pipe;
	wire [5:0] lb_addr_mux;
	wire slot_ge_5;
	wire col_eq_4;
	wire [7:0] tile_plane3_a;
	wire [7:0] tile_plane2_a;
	wire [7:0] tile_plane0_a;
	wire [7:0] tile_plane1_a;
	wire tile_strobe_p0;
	wire tile_strobe_p0_pipe;
	wire tile_strobe_lat_0;
	wire tile_strobe_p1;
	wire tile_gate_0;
	wire tile_gate_1;
	wire tile_strobe_lat_1;
	wire tile_strobe_p2;
	wire tile_strobe_p2_pipe;
	wire tile_strobe_p3;
	wire [7:0] tile_plane2_b;
	wire [7:0] tile_plane0_b;
	wire [7:0] tile_plane1_b;
	wire tile_pipe_0;
	wire tile_pipe_1;
	wire tile_stage_strobe;
	wire [7:0] tile_plane3;
	wire [7:0] tile_plane2;
	wire [7:0] tile_plane0;
	wire [7:0] tile_plane1;
	wire pixel_bit_0;
	wire pixel_bit_1;
	wire pixel_bit_2;
	wire pixel_bit_3;
	wire pixel_done;
	wire tile_strobe_hold;
	wire pixel_start_n;
	wire hflip_serial_pipe;
	wire pixel_sel_0;
	wire pixel_sel_1;
	wire pixel_sel_2;
	wire pixel_sel_3;
	wire pixel_mode_m5;
	wire pixel_mode_m4;
	wire [7:0] pixel_serial;
	wire [7:0] pixel_hflip;
	wire [7:0] pixel_data_latch;
	wire [7:0] pixel_data_mux;
	wire [1:0] tile_col_cnt;
	wire tile_col_load;
	wire [2:0] pixel_col;
	wire pixel_col_bit0;
	wire pixel_col_bit0_pipe;
	wire pixel_nibble_swap;
	wire fetch_start_comb;
	wire fetch_pipe_1;
	wire fetch_pipe_2;
	wire fetch_pipe_3;
	wire fetch_pipe_4;
	wire fetch_phase_sel;
	wire fetch_phase;
	wire disp_gate_latch;
	wire disp_gate_pipe;
	wire disp_gate_edge;
	wire disp_gate_delay;
	wire slot_ge_1;
	wire test_bank_0_n;
	wire test_bank_1_n;
	wire test_bank_2_n;
	wire test_bank_3_n;
	wire lb_wr_bank_0;
	wire lb_wr_bank_1;
	wire lb_wr_bank_2;
	wire lb_wr_bank_3;
	wire col_eq_0;
	wire col_eq_1;
	wire col_eq_2;
	wire col_eq_3;
	wire col_eq_5;
	wire col_eq_6;
	wire col_eq_7;
	wire slot_ge_2;
	wire slot_ge_3;
	wire slot_ge_4;
	wire slot_ge_6;
	wire slot_ge_7;
	wire stage_strobe_0;
	wire col_grp_1;
	wire stage_strobe_1;
	wire stage_strobe_2;
	wire col_grp_3;
	wire lb_rd_mask_n;
	wire stage_strobe_3;
	wire stage_strobe_4;
	wire col_grp_5;
	wire stage_strobe_5;
	wire stage_strobe_6;
	wire col_grp_7;
	wire stage_strobe_7;
	// Sprite pixel data pipeline (8 pixels × 4-bit: stage_px_0-lb_px_7)
	wire [3:0] stage_px_0;
	wire [3:0] stage_px_1;
	wire [3:0] stage_px_2;
	wire [3:0] stage_px_3;
	wire [3:0] stage_px_4;
	wire [3:0] stage_px_5;
	wire [3:0] stage_px_6;
	wire [3:0] stage_px_7;
	wire lb_stage_strobe;
	wire [3:0] lb_px_0;
	wire [3:0] lb_px_1;
	wire [3:0] lb_px_2;
	wire [3:0] lb_px_3;
	wire [3:0] lb_px_4;
	wire [3:0] lb_px_5;
	wire [3:0] lb_px_6;
	wire [3:0] lb_px_7;
	wire slot_valid_0;
	wire slot_valid_1;
	wire slot_valid_2;
	wire slot_valid_3;
	wire slot_valid_4;
	wire slot_valid_5;
	wire slot_valid_6;
	wire slot_sel_0;
	wire slot_sel_1;
	wire slot_sel_2;
	wire slot_sel_3;
	wire slot_sel_4;
	wire slot_sel_5;
	wire slot_sel_6;
	wire slot_sel_7;
	wire slot_wr_0;
	wire slot_wr_1;
	wire slot_wr_2;
	wire slot_wr_3;
	wire slot_wr_4;
	wire slot_wr_5;
	wire slot_wr_6;
	wire slot_wr_7;
	wire lb_wr_en_0;
	wire lb_wr_en_1;
	wire lb_wr_en_2;
	wire lb_wr_en_3;
	wire lb_wr_en_4;
	wire lb_wr_en_5;
	wire lb_wr_en_6;
	wire lb_wr_en_7;
	wire lb_wr_latch_0;
	wire lb_wr_latch_1;
	wire lb_wr_latch_2;
	wire lb_wr_latch_3;
	wire lb_wr_latch_4;
	wire lb_wr_latch_5;
	wire lb_wr_latch_6;
	wire lb_wr_latch_7;
	wire lb_wr_clk_0;
	wire lb_wr_clk_1;
	wire lb_wr_clk_2;
	wire lb_wr_clk_3;
	wire lb_wr_clk_4;
	wire lb_wr_clk_5;
	wire lb_wr_clk_6;
	wire lb_wr_clk_7;
	wire lb_idx_nz_0;
	wire lb_idx_nz_1;
	wire lb_idx_nz_2;
	wire lb_idx_nz_3;
	wire lb_idx_nz_4;
	wire lb_idx_nz_5;
	wire lb_idx_nz_6;
	wire lb_idx_nz_7;
	wire lb_collide_0;
	wire lb_collide_1;
	wire lb_collide_2;
	wire lb_collide_3;
	wire lb_collide_4;
	wire lb_collide_5;
	wire lb_collide_6;
	wire lb_collide_7;
	wire lb_px_nz_0;
	wire lb_px_nz_1;
	wire lb_px_nz_2;
	wire lb_px_nz_3;
	wire lb_px_nz_4;
	wire lb_px_nz_5;
	wire lb_px_nz_6;
	wire lb_px_nz_7;
	wire lb_wr_cond_0;
	wire lb_wr_cond_1;
	wire lb_wr_cond_2;
	wire lb_wr_cond_3;
	wire lb_wr_cond_4;
	wire lb_wr_cond_5;
	wire lb_wr_cond_6;
	wire lb_wr_cond_7;
	wire lb_wr_mode;
	wire lb_clear;
	wire test_idx_0;
	wire test_idx_1;
	wire test_idx_2;
	wire test_idx_3;
	wire test_rd_pal0_even;
	wire test_rd_pal1_even;
	wire test_rd_pri_even;
	wire test_rd_idx0_even;
	wire test_rd_idx1_even;
	wire test_rd_idx2_even;
	wire test_rd_idx3_even;
	wire test_rd_pal0_odd;
	wire test_rd_pal1_odd;
	wire test_rd_pri_odd;
	wire test_rd_idx0_odd;
	wire test_rd_idx1_odd;
	wire test_rd_idx2_odd;
	wire test_rd_idx3_odd;
	wire [1:0] spr_pal_pipe_0;
	wire spr_pri_pipe_0;
	wire [3:0] spr_idx_pipe_0;
	// Final sprite pixel attributes (fed to priority MUX)
	wire [1:0]spr_pal;      // sprite palette select
	wire spr_priority;      // sprite priority flag
	wire [3:0] spr_index;   // sprite color index
	wire [1:0] spr_pal_m5_sel;
	wire spr_pri_m5_sel;
	wire [3:0] spr_idx_m5_sel;
	wire [1:0] spr_pal_pipe_1;
	wire spr_pri_pipe_1;
	wire [3:0] spr_idx_pipe_1;
	wire [1:0] spr_pal_pipe_2;
	wire spr_pri_pipe_2;
	wire [3:0] spr_idx_pipe_2;
	wire spr_out_pri;
	wire [1:0] spr_out_pal;
	wire spr_pal_is_3;
	wire spr_idx_nonzero;
	wire spr_idx_is_14;
	wire spr_idx_is_15;
	wire lb_rd_mask_pipe;
	wire disp_gate_pipe_2;
	wire lb_data_pri_hi;
	wire [1:0] lb_data_pal_hi;
	wire lb_data_pri_lo;
	wire [1:0] lb_data_pal_lo;
	wire spr_collision_pipe;
	wire spr_collision_any;
	wire spr_overflow_flag;
	
	// Line buffer output: 8 pixels unpacked from 56-bit entry
	wire [1:0] linebuffer_out_pal[0:7];      // palette per pixel
	wire linebuffer_out_priority[0:7];        // priority per pixel
	wire [3:0] linebuffer_out_index[0:7];     // color index per pixel

	// VRAM interface: RAS/CAS/WE sequencer, serial data, refresh (vram_oe1_comb-spr_collision_any)
	wire vram_clk_latch;
	wire vram_clk_pipe;
	wire vram_clk_pipe_2;
	wire vram_clk_pipe_3;
	wire vram_oe1_comb;
	wire vram_we0_comb;
	wire vram_we1_comb;
	wire vram_cas_comb;
	wire vram_ras_comb;
	wire vram_data_dir;
	wire vram_access_pipe;
	wire vram_access_comb;
	wire vram_refresh_comb;
	wire vram_refresh_latch;
	wire vram_refresh_pipe;
	wire vram_access_delay;
	wire vram_oe_gate;
	wire vram_cycle_latch;
	wire vram_cycle_pipe;
	wire vram_req_pipe;
	wire vram_active_latch;
	wire vram_phase_pipe;
	wire vram_phase_lat_1;
	wire vram_phase_lat_2;
	wire vram_phase_lat_3;
	wire vram_addr_gate;
	wire vram_addr_latch;
	wire vram_wr_pipe;
	wire vram_sd_sel;
	wire vram_addr_hi_sel;
	wire vram_addr_lo_sel;
	wire vram_wr_timing;
	wire vram_capture_strobe;
	wire vram_dma_pipe;
	wire vram_we_pipe;
	wire vram_we_comb;
	wire vram_we_delay;
	wire vram_refresh_trig;
	wire vram_serial_sel;
	wire vram_cas_sel;
	wire vram_timing_0;
	wire vram_timing_1;
	wire vram_req_pipe_1;
	wire vram_active_comb;
	wire vram_addr_strobe;
	wire vram_data_strobe;
	wire vram_dma_comb;
	wire vram_128k_pipe;
	wire vram_wr_strobe;
	wire vram_sc_comb;
	wire vram_se_comb;
	wire vram_timing_any;
	wire vram_ys_mux;
	wire vram_m5_bit1;
	wire vram_addr_bit;
	wire [7:0] vram_row_addr;
	wire vram_128k_bit;
	wire [7:0] vram_row_latch;
	wire [7:0] vram_col_addr;
	wire [7:0] vram_col_latch;
	wire [7:0] vram_addr_out;
	wire [7:0] vram_data_addr;
	wire [7:0] vram_wdata_lo_sel;
	wire [7:0] vram_wdata_lo;
	wire [7:0] vram_wdata_hi;
	wire [7:0] vram_wdata_hi_sel;
	wire [7:0] vram_wdata_hi_out;
	wire [7:0] vram_rdata_hi;
	wire [7:0] vram_rdata_lo;
	wire [7:0] vram_ad_out;
	wire [7:0] vram_rd_out;
	
	// Video MUX: sprite/plane/background priority, shadow/highlight (cram_wr_any-cram_wr_m4)
	wire cram_wr_any;
	wire cram_wr_hi; // CRAM upper write enable (color bus → CRAM[8:6])
	wire cram_wr_lo; // CRAM lower write enable (color bus → CRAM[5:0])
	wire pri_spr_or_a;
	wire pri_spr_hi_b_hi;
	wire pri_a_hi_only;
	wire pri_b_hi_only;
	wire pri_a_hi_b_hi;
	wire sh_no_priority;
	wire sh_spr_only;
	wire sh_mode_active;
	wire sh_spr_special;
	wire spr_transparent;
	wire spr_opaque;
	wire planeb_opaque;
	wire test_layer_spr;
	wire test_layer_a;
	wire test_layer_b;
	wire test_layer_bg;
	wire sel_spr_case_1;
	wire sel_spr_case_2;
	wire sel_spr_case_3;
	wire sel_spr_cond;
	wire sel_spr_valid;
	wire sel_spr_normal;
	wire sel_spr_final;
	wire sel_spr_sh;
	wire sel_a_case_1;
	wire sel_a_case_2;
	wire sel_a_case_3;
	wire sel_a_cond;
	wire sel_a_valid;
	wire sel_a_final;
	wire sel_b_case_1;
	wire sel_b_case_2;
	wire sel_b_case_3;
	wire sel_b_case_4;
	wire sel_b_cond;
	wire sel_b_valid;
	wire sel_b_final;
	wire all_layers_trans;
	wire sel_bg_cond;
	wire sel_bg_final;
	wire disp_not_test;
	wire sel_spr_pipe;
	wire sel_a_pipe;
	wire sel_b_pipe;
	wire sel_bg_pipe;
	wire not_spr_not_idx14;
	wire no_sh_special;
	wire spr_idx_14_or_15;
	wire sh_highlight_cond;
	wire sh_highlight_pipe;
	wire sh_shadow_cond;
	wire sh_shadow_gated;
	wire sh_shadow_pipe_1;
	wire sh_shadow_mux;
	wire sh_shadow_pipe_2;
	wire sh_shadow_pipe_3;
	wire sh_highlight_mux;
	wire sh_highlight_pipe_2;
	wire sh_highlight_pipe_3;
	wire spa_b_gate_n;
	wire spa_b_pipe;
	wire cram_wr_m5;
	wire cram_wr_m4;
	// Background color registers (reg 0x87)
	wire [3:0] reg_col_index; // background color index
	wire [1:0] reg_col_pal;   // background palette
	wire reg_col_b6;
	wire reg_col_b7;
	wire blank_out_pipe;
	wire spa_in_pipe;
	wire blank_out_comb;
	wire active_delay_8;
	wire active_mux_m5;
	wire [5:0] color_bus_mux;
	wire [5:0] color_bus_pipe;
	wire active_delay_3;
	wire bg_color_zero;
	wire bg_zero_pipe_1;
	wire [2:0] cram_data_hi;
	wire [2:0] cram_red_bits;
	wire [2:0] cram_grn_bits;
	wire [8:0] cram_rd_latch;
	wire [8:0] cram_rd_pipe;
	wire cram_wr_dly_1, cram_wr_dly_2, cram_wr_dly_3;
	wire cram_latch_en;
	wire disp_unblanked;
	wire disp_unblank_pipe;
	wire bg_zero_pipe_2;
	wire force_bg_zero;
	wire cram_r_bit0;
	wire cram_r_bit1;
	wire cram_g_bit0;
	wire cram_g_bit1;
	wire cram_b_bit0;
	wire cram_b_bit1;
	wire dac_r_bit0;
	wire dac_r_bit1;
	wire dac_g_bit0;
	wire dac_g_bit1;
	wire dac_b_bit0;
	wire dac_b_bit1;
	wire w1095;
	wire w1096;
	wire w1097;
	wire dac_b_bit2;
	wire dac_g_bit2;
	wire dac_r_bit2;
	// DAC & color output
	wire [2:0] dac_red_pipe; // R post-processing (3-bit CRAM red)
	wire [2:0] dac_grn_pipe; // G post-processing (3-bit CRAM green)
	wire [2:0] dac_blu_pipe; // B post-processing (3-bit CRAM blue)
	wire shadow_flag; // shadow mode flag
	wire highlight_flag; // highlight mode flag
	wire normal_intensity; // normal intensity select
	wire shadow_level;
	wire [16:0] w1103[0:2]; // 17-level non-linear DAC thermometer code (R/G/B)
	
	// PSG (SN76489) signals
	wire psg_clk1;
	wire psg_clk2;
	wire psg_rst_pipe_1;
	wire psg_rst_pipe_2;
	wire psg_rst_edge;
	wire psg_div_fb;
	wire psg_div_out;
	wire psg_div_latch;
	wire psg_hclk1;
	wire psg_hclk2;
	wire psg_wr_trig;
	wire psg_wr_comb;
	wire psg_wr_pipe_1;
	wire psg_wr_pipe_2;
	wire psg_rst_hclk;
	wire psg_noise_trig;
	wire psg_noise_pipe;
	wire psg_tone_en;
	wire psg_cnt_down;
	wire psg_cnt_up;
	wire psg_cnt_zero;
	wire psg_cnt_zero_pipe;
	wire psg_noise_fb;
	wire psg_test_mute_0;
	wire psg_test_mute_1;
	wire psg_test_mute_2;
	wire psg_test_mute_3;
	wire [9:0] psg_freq_mux;
	wire [9:0] psg_freq_inc;
	wire psg_lfsr_fb;
	wire [15:0] psg_lfsr;
	wire psg_lfsr_nz;
	wire psg_lfsr_gate;
	wire [9:0] psg_freq_pipe_0;
	wire [9:0] psg_freq_pipe_1;
	wire [9:0] psg_freq_pipe_2;
	wire [9:0] psg_freq_pipe_3;
	wire psg_tone_active;
	wire psg_ch0_out;
	wire psg_ch1_out;
	wire psg_ch2_out;
	wire psg_ch3_out;
	wire psg_sq_0;
	wire psg_sq_1;
	wire psg_sq_2;
	wire psg_sq_3;
	wire psg_div_cnt;
	wire [9:0] psg_freq_sel;
	wire psg_freq_match;
	wire psg_ch3_sel;
	wire psg_ch2_sel;
	wire psg_ch1_sel;
	wire psg_ch0_sel;
	wire [3:0] psg_ch_ring;
	wire psg_ring_wrap;
	wire psg_rst_sync;
	wire psg_not_rst;
	wire [3:0] psg_tone_ring;
	wire [7:0] psg_data_latch;
	wire [7:0] psg_data_mux;
	wire [2:0] psg_reg_addr;
	wire psg_wr_vol0;
	wire psg_wr_vol1;
	wire psg_wr_freq2;
	wire psg_wr_freq1;
	wire psg_wr_vol2;
	wire psg_wr_freq0;
	wire psg_wr_vol3;
	wire psg_wr_noise;
	wire [3:0] psg_vol_data;
	wire [3:0] psg_vol_0;
	wire [3:0] psg_vol_1;
	wire [3:0] psg_vol_2;
	wire [3:0] psg_vol_3;
	wire psg_latch_mode;
	wire [9:0] psg_freq_0;
	wire [9:0] psg_freq_1;
	wire [9:0] psg_freq_2;
	wire [2:0] psg_noise_ctrl;
	wire psg_sq0_mute;
	wire psg_sq1_mute;
	wire psg_sq2_mute;
	wire psg_noise_mute;
	wire [3:0] psg_atten_0;
	wire [3:0] psg_atten_1;
	wire [3:0] psg_atten_2;
	wire [3:0] psg_atten_3;

	// VDP configuration registers (directly named from register map)
	wire [14:0] reg_test0;   // test register 0
	wire [11:0] reg_test_18; // test register 0x18
	wire [7:0] reg_hit;      // H-interrupt counter (reg 0x8A)
	wire [10:0] reg_test1;   // test register 1
	// Mode register 0x80
	wire reg_80_b7;
	wire reg_80_b6;
	wire reg_lcb;   // left column blank (reg 0x80 bit 5)
	wire reg_ie1;   // H-interrupt enable (reg 0x80 bit 4)
	wire reg_80_b3;
	wire reg_80_b2;
	wire reg_m3;    // H/V counter latch (reg 0x80 bit 1)
	wire reg_80_b0;
	// Mode register 0x8C
	wire reg_lsm0;  // interlace mode bit 0 (reg 0x8C bit 1)
	wire reg_lsm1;  // interlace mode bit 1 (reg 0x8C bit 2)
	wire reg_ste;   // shadow/highlight enable (reg 0x8C bit 3)
	wire reg_8c_b4;
	wire reg_8c_b5;
	wire reg_8c_b6;
	wire reg_rs0;   // external pixel clock select (reg 0x8C bit 7)
	wire reg_rs1;   // H40 mode (reg 0x8C bit 0)
	// Mode register 0x81
	wire reg_81_b0;
	wire reg_81_b1;
	wire reg_m5;    // MegaDrive (MD) mode (reg 0x81 bit 2)
	wire reg_m2;    // V30 mode (reg 0x81 bit 3)
	wire reg_m1;    // DMA enable (reg 0x81 bit 4)
	wire reg_ie0;   // V-interrupt enable (reg 0x81 bit 5)
	wire reg_disp;  // display enable (reg 0x81 bit 6)
	wire reg_81_b7;
	// Scroll mode register 0x8B
	wire reg_lscr;  // full-screen/per-line scroll select
	wire reg_hscr;  // H-scroll mode (reg 0x8B bit 0)
	wire reg_vscr;  // V-scroll per-2-cell mode (reg 0x8B bit 2)
	wire reg_ie2;   // external interrupt enable (reg 0x8B bit 3)
	wire reg_8b_b4;
	wire reg_8b_b5;
	wire reg_8b_b6;
	wire reg_8b_b7;
	// Interlace latch (latched at vsync)
	wire reg_lsm0_latch;
	wire reg_lsm1_latch;
	// VRAM access control registers
	wire [4:0] reg_code;     // VRAM/CRAM/VSRAM access mode code
	wire [16:0] reg_addr;    // VRAM address register (17-bit)
	wire [7:0] reg_inc;      // auto-increment value (reg 0x8F)
	wire [16:0] reg_data_l2; // data port latch
	// DMA registers
	wire [5:0] reg_sa_high;  // DMA source address high (reg 0x97)
	wire [1:0] reg_dmd;      // DMA mode (reg 0x97 bits 7:6)
	wire [15:0] reg_lg;      // DMA length counter (reg 0x93-0x94)
	wire [15:0] reg_sa_low;  // DMA source address low (reg 0x95-0x96)
	
	assign reset_comb = ~(RESET & int_reset_state);

	// VRAM bus registers
	reg [16:0] vram_address;  // current VRAM address (active on bus)
	reg [15:0] vram_data;     // VRAM data register (active on bus)
	wire [7:0] vram_serial;   // serial VRAM data captured from SD pins

	//reg [16:0] vram_address_mem;
	//reg [15:0] vram_data_mem;

	// Color bus outputs (from priority MUX to CRAM)
	wire [3:0] color_index;   // pixel color index
	wire color_priority;      // pixel priority flag
	wire [1:0] color_pal;     // pixel palette select

	// On-chip memory arrays
	reg [10:0] vsram[0:39];       // Vertical Scroll RAM (40 entries × 11 bits)
	reg [10:0] vsram_out;
	reg [10:0] vsram_out_0;
	reg [10:0] vsram_out_1;

	reg [20:0] sat[0:79];         // Sprite Attribute Table cache (80 entries × 21 bits)
	reg [20:0] sat_out;
	reg [20:0] sat_out_0;
	reg [20:0] sat_out_1;
	reg [20:0] sat_out_2;
	reg [20:0] sat_out_3;

	reg [33:0] sprdata[0:19];     // Sprite render data buffer (20 entries × 34 bits)
	reg [33:0] sprdata_out;
	reg [33:0] sprdata_out_0;
	reg [33:0] sprdata_out_1;

	reg [55:0] linebuffer[0:39];  // Line buffer (40 entries × 56 bits, 8 pixels each)
	reg [55:0] linebuffer_out;
	reg [55:0] linebuffer_out_0;
	reg [55:0] linebuffer_out_1;

	reg [8:0] color_ram[0:63];    // Color RAM (64 entries × 9 bits, 3×3-bit RGB)
	reg [8:0] color_ram_out;

	// Display pipeline CRAM readout (active display, bypasses normal bus)
	wire [5:0] color_bus_dp;
	wire [5:0] color_bus_pipe_dp;
	reg [8:0] color_ram_out_dp;
	
	
	// -------------------------------------------------------------------------
	// Prescaler & clock generation
	// -------------------------------------------------------------------------
	// Divides MCLK (~53.7 MHz NTSC / ~54.2 MHz PAL) into dot clocks
	// (clk1/clk2) and CPU clock (cpu_clk0/cpu_clk1). Also generates
	// sub-carrier reference (SBCR) and the per-pixel dot clock (dclk).

	assign mclk_and1 = prescaler_dff2_l2 & ~prescaler_dff1_l2;
	
	assign mclk_clk1 = prescaler_dff4_l2;
	
	assign mclk_clk2 = prescaler_dff7_l2;
	
	assign mclk_clk3 = ~prescaler_dff11_l2;
	
	assign mclk_clk4 = prescaler_dff13_l2 | prescaler_dff14_l2;
	
	assign mclk_clk5 = prescaler_dff16_l2 | prescaler_dff17_l2;
	
	assign mclk_sbcr = PAL ? mclk_clk4 : mclk_clk5;
	
	assign mclk_cpu_clk0 = reg_test1[0] ? CLK1_i : mclk_clk5;
	
	assign mclk_dclk = (reg_rs0 | reg_test1[0]) ? EDCLK_i : (reg_rs1 ? mclk_clk1 : mclk_clk2);
	//assign mclk_dclk = reg_rs1 ? mclk_clk1 : mclk_clk2;
	
	/*
	
	assign mclk_cpu_clk1 = ~mclk_clk3;
	
	always @(posedge MCLK)
	begin
		prescaler_dff1 <= reset_comb;
		prescaler_dff2 <= prescaler_dff1;
		
		if (mclk_and1)
		begin
			prescaler_dff3 <= 1'h0;
			prescaler_dff4 <= 1'h0;
			prescaler_dff5 <= 1'h0;
			prescaler_dff6 <= 1'h0;
			prescaler_dff7 <= 1'h0;
			prescaler_dff8 <= 1'h0;
			prescaler_dff9 <= 1'h0;
			prescaler_dff10 <= 1'h0;
			prescaler_dff11 <= 1'h0;
		end
		else
		begin
			prescaler_dff3 <= prescaler_dff4;
			prescaler_dff4 <= ~prescaler_dff3;
			
			prescaler_dff5 <= prescaler_dff7;
			prescaler_dff6 <= prescaler_dff5;
			prescaler_dff7 <= ~(prescaler_dff5 & prescaler_dff6);
			
			prescaler_dff8 <= prescaler_dff11;
			prescaler_dff9 <= prescaler_dff8;
			prescaler_dff10 <= ~(prescaler_dff8 & prescaler_dff9);
			prescaler_dff11 <= prescaler_dff10;
		end
	end*/
	
	reg mclk_clk3_l;
	
	assign mclk_cpu_clk1 = ~(mclk_clk3 | mclk_clk3_l);
	
	always @(posedge MCLK)
	begin
		mclk_clk3_l <= mclk_clk3;
	end
	
	ym7101_dff prescaler_dff1(.MCLK(MCLK), .clk(MCLK_e), .inp(reset_comb), .rst(1'h0), .outp(prescaler_dff1_l2));
	ym7101_dff prescaler_dff2(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff1_l2), .rst(1'h0), .outp(prescaler_dff2_l2));
	ym7101_dff prescaler_dff3(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff4_l2), .rst(mclk_and1), .outp(prescaler_dff3_l2));
	ym7101_dff prescaler_dff4(.MCLK(MCLK), .clk(MCLK_e), .inp(~prescaler_dff3_l2), .rst(mclk_and1), .outp(prescaler_dff4_l2));
	ym7101_dff prescaler_dff5(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff7_l2), .rst(mclk_and1), .outp(prescaler_dff5_l2));
	ym7101_dff prescaler_dff6(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff5_l2), .rst(mclk_and1), .outp(prescaler_dff6_l2));
	ym7101_dff prescaler_dff7(.MCLK(MCLK), .clk(MCLK_e), .inp(~(prescaler_dff5_l2 & prescaler_dff6_l2)), .rst(mclk_and1), .outp(prescaler_dff7_l2));
	ym7101_dff prescaler_dff8(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff11_l2), .rst(mclk_and1), .outp(prescaler_dff8_l2));
	ym7101_dff prescaler_dff9(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff8_l2), .rst(mclk_and1), .outp(prescaler_dff9_l2));
	ym7101_dff prescaler_dff10(.MCLK(MCLK), .clk(MCLK_e), .inp(~(prescaler_dff8_l2 & prescaler_dff9_l2)), .rst(mclk_and1), .outp(prescaler_dff10_l2));
	ym7101_dff prescaler_dff11(.MCLK(MCLK), .clk(MCLK_e), .inp(prescaler_dff10_l2), .rst(mclk_and1), .outp(prescaler_dff11_l2));
	
	ym7101_dff prescaler_dff12(.MCLK(MCLK), .clk(mclk_clk1), .inp(~(prescaler_dff12_l2 | prescaler_dff13_l2)), .rst(mclk_and1), .outp(prescaler_dff12_l2));
	ym7101_dff prescaler_dff13(.MCLK(MCLK), .clk(mclk_clk1), .inp(prescaler_dff12_l2), .rst(mclk_and1), .outp(prescaler_dff13_l2));
	ym7101_dff prescaler_dff14(.MCLK(MCLK), .clk(~mclk_clk1), .inp(prescaler_dff13_l2), .rst(mclk_and1), .outp(prescaler_dff14_l2));
	ym7101_dff prescaler_dff15(.MCLK(MCLK), .clk(mclk_clk2), .inp(~(prescaler_dff15_l2 | prescaler_dff16_l2)), .rst(mclk_and1), .outp(prescaler_dff15_l2));
	ym7101_dff prescaler_dff16(.MCLK(MCLK), .clk(mclk_clk2), .inp(prescaler_dff15_l2), .rst(mclk_and1), .outp(prescaler_dff16_l2));
	ym7101_dff prescaler_dff17(.MCLK(MCLK), .clk(~mclk_clk2), .inp(prescaler_dff16_l2), .rst(mclk_and1), .outp(prescaler_dff17_l2));
	
	assign SBCR = mclk_sbcr;
	assign CLK0 = mclk_cpu_clk0;
	assign CLK1_o = mclk_cpu_clk1;
	
	assign EDCLK_o = mclk_dclk;
	
	assign EDCLK_d = reg_test1[1];
	
	assign cpu_clk0 = mclk_cpu_clk0;
	assign cpu_clk1 = CLK1_i;
	
	// --- clk1, clk2 (master dot clock phases) ---
	
	
	reg dclk_l;
	reg dclk_l2;
	reg dclk_l3;
	reg dclk_l4;
	
	always @(posedge MCLK)
	begin
		dclk_l <= dclk_l2;
		dclk_l2 <= dclk_l3;
		dclk_l3 <= dclk_l4;
		dclk_l4 <= mclk_dclk;
	end
	
	assign clk1 = ~mclk_dclk & dclk_l;
	assign clk2 = mclk_dclk & ~dclk_l;
	
	/*reg dclk_l;
	reg tclk1_l;
	reg tclk2_l;
	
	wire tclk1 = ~mclk_dclk & dclk_l;
	wire tclk2 = mclk_dclk & ~dclk_l;
	
	always @(posedge MCLK)
	begin
		dclk_l <= mclk_dclk;
		tclk1_l <= tclk1;
		tclk2_l <= tclk2;
	end
	
	assign clk1 = tclk1 | tclk1_l;
	assign clk2 = tclk2 | tclk2_l;*/
	
	
	// --- hclk1, hclk2 (half-rate pixel clocks, main rendering clock) ---
	
	wire reset_l1_o;
	wire reset_l2_o;
	wire reset_pulse = reset_l1_o & ~reset_l2_o;
	ym_sr_bit reset_l1(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(~reset_comb), .sr_out(reset_l1_o)); // static latch
	ym_sr_bit reset_l2(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(reset_l1_o), .sr_out(reset_l2_o));
	
	wire dclk_prescaler_l1_o;
	wire dclk_prescaler_l2_o;
	wire dclk_prescaler_l3_o;
	wire dclk_prescaler_dff1_l2;
	wire dclk_prescaler_dff2_l2;
	assign hclk1 = ~dclk_prescaler_dff1_l2;
	assign hclk2 = ~dclk_prescaler_dff2_l2;
	ym_sr_bit dclk_prescaler_l1(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(~(dclk_prescaler_l1_o | reset_pulse)), .sr_out(dclk_prescaler_l1_o));
	ym_dlatch_1 dclk_prescaler_l2(.MCLK(MCLK), .c1(clk1), .inp(dclk_prescaler_l1_o), .val(dclk_prescaler_l2_o));
	ym_dlatch_1 dclk_prescaler_l3(.MCLK(MCLK), .c1(clk1), .inp(~dclk_prescaler_l1_o), .val(dclk_prescaler_l3_o));
	ym7101_dff dclk_prescaler_dff1(.MCLK(MCLK), .clk(~clk1), .inp(1'h1), .rst(dclk_prescaler_l2_o & clk2), .outp(dclk_prescaler_dff1_l2));
	ym7101_dff dclk_prescaler_dff2(.MCLK(MCLK), .clk(~clk1), .inp(1'h1), .rst(dclk_prescaler_l3_o & clk2), .outp(dclk_prescaler_dff2_l2));
	
	// -------------------------------------------------------------------------
	// I/O, DMA & FIFO
	// -------------------------------------------------------------------------
	// CPU bus decode for both 68K (active-low AS/UDS/LDS/RW) and Z80
	// (active-low M1/RD/WR/MREQ/IORQ). Generates DTACK, handles VDP
	// register writes (0x80-0x97), VRAM/CRAM/VSRAM read/write via the
	// 4-word FIFO, DMA state machine (68K→VRAM, fill, copy), HV counter
	// read, interrupt generation (VINT, HINT, EINT), and bus arbitration.

	assign cpu_sel = SEL0;
	assign cpu_as = ~AS & cpu_sel;
	assign cpu_uds = ~UDS & cpu_sel;
	assign cpu_lds = ~LDS & cpu_sel;
	assign cpu_m1 = ~M1 & ~cpu_sel;
	assign cpu_rd = ~RD & ~cpu_sel;
	assign cpu_wr = ~WR & ~cpu_sel;
	assign cpu_mreq = ~MREQ & ~cpu_sel;
	assign cpu_iorq = ~IORQ & ~cpu_sel;
	assign cpu_rw = ~RW;
	assign cpu_bg = ~BG;
	assign cpu_intak = ~INTAK;
	assign cpu_bgack = BGACK_i;
	assign cpu_pal = PAL;
	assign cpu_pen = HL;
	
	ym7101_dff io_m1_dff1(.MCLK(MCLK), .clk(cpu_clk0), .inp(cpu_m1), .rst(1'h0), .outp(io_m1_dff1_l2));
	ym7101_dff io_m1_dff2(.MCLK(MCLK), .clk(cpu_clk0), .inp(io_m1_dff1_l2), .rst(1'h0), .outp(io_m1_dff2_l2));
	ym7101_dff io_m1_dff3(.MCLK(MCLK), .clk(~cpu_clk0), .inp(io_m1_dff2_l2), .rst(1'h0), .outp(io_m1_dff3_l2));
	
	assign io_m1_s1 = io_m1_dff3_l2 & io_m1_dff2_l2;
	assign io_m1_s2 = ~io_m1_s1 & io_m1_s4;
	ym7101_dff io_m1_dff4(.MCLK(MCLK), .clk(~cpu_clk0), .inp(io_m1_s2), .rst(1'h0), .outp(io_m1_dff4_l2));
	
	assign io_m1_s3 = io_m1_dff4_l2 & io_m1_s2;
	
	assign io_m1_s4 = cpu_mreq & (io_m1_s1 | ( &io_address[15:14]));
	
	assign io_m1_s5 = io_m1_s4 & io_m1_s1;
	
	assign io_oe0 = io_m1_s5 | oe_cpu_rd | z80_ras_pulse | oe_comb | bus_phase_c;
	
	assign cpu_wr_cas_gate  = cpu_wr_strobe & ~addr_hi_sel;
	
	assign io_cas0 = reg_8b_b6 ?
		(io_m1_dff2_l2 | cas_z80_gate | z80_cas_pulse | bus_phase2 | z80_cas_cond) :
		(bus_phase_c | oe_cpu_rd | cpu_wr_cas_gate);
	
	assign io_ras0 = reg_8b_b6 ?
		(io_m1_s4 | ras_gated | ras_z80_gate | z80_ras_pulse) :
		(io_m1_s2 | ras_sel | ras_dma_gate);
	
	assign io_wr = cpu_rw & dff1_l2;
	
	assign io_lwr = cpu_wr | (cpu_lds & io_wr);
	assign io_uwr = cpu_uds & io_wr;
	
	assign cpu_wr_strobe = ~cpu_rw & (cpu_uds | cpu_lds);
	ym7101_dff dff1(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dtack_gate_n), .rst(1'h0), .outp(dff1_l2));
	
	ym7101_dff dff2(.MCLK(MCLK), .clk(cpu_clk1), .inp(cpu_bg), .rst(1'h0), .outp(dff2_l2));
	
	ym7101_rs_trig rs1(.MCLK(MCLK), .set(cpu_bg | reset_comb), .rst(~reg_data_l2[7] & reg_wr_8B & reg_m5), .q(bus_br_hold));
	
	assign addr_hi_sel = intak_clear & (&io_address[22:20]);
	
	assign io_address_22o = ~(edclk_gap & dma_fill_mode & (bus_phase_a | ~bus_phase_b));
	
	ym7101_dff dff4(.MCLK(MCLK), .clk(hclk2), .inp(dma_req_any), .rst(dma_rst), .outp(dff4_l2));
	ym7101_dff dff3(.MCLK(MCLK), .clk(hclk2), .inp(dff4_l2), .rst(dma_rst), .outp(dff3_l2));
	
	assign dma_req_any = dma_copy_req | bus_granted;
	
	assign dma_rst = reset_comb | dma_len_done_dly;
	
	ym7101_rs_trig rs2(.MCLK(MCLK), .set(dma_copy_start), .rst(dma_rst), .q(dma_copy_req));
	ym7101_rs_trig rs3(.MCLK(MCLK), .set(arb_grant_cond), .rst(dma_rst), .q(bus_granted));
	ym7101_rs_trig rs4(.MCLK(MCLK), .set(dma_68k_start), .rst(dma_rst | arb_grant_cond), .q(dma_68k_req));
	
	assign arb_grant_cond = dff22_l2 & cpu_bgack & DTACK_i & dff2_l2 & cpu_sel & as_inactive;
	
	assign io_ipl1 = ~(ipl1_src & cpu_sel);
	assign io_ipl2 = ~(ipl2_src & cpu_sel);
	
	ym_sr_bit sr1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(edclk_dly), .sr_out(edclk_pipe1));
	ym_sr_bit sr2(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(edclk_pipe1), .sr_out(edclk_pipe2));
	ym_sr_bit sr3(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(edclk_pipe2), .sr_out(edclk_pipe3));
	ym_sr_bit sr4(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~(edclk_dly | edclk_pipe1 | edclk_pipe2 | edclk_pipe3)), .sr_out(edclk_gap));
	ym_sr_bit sr5(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_addr_oe), .sr_out(dma_addr_dly));
	ym_dlatch_1 dl6(.MCLK(MCLK), .c1(hclk1), .inp(~(bus_access_gate_n & dma_addr_hold & odd_slot)), .nval(bus_phase_a));
	ym_dlatch_1 dl7(.MCLK(MCLK), .c1(clk1), .inp(bus_phase_a), .nval(bus_phase_b));
	ym_dlatch_2 dl8(.MCLK(MCLK), .c2(clk2), .inp(bus_phase_b), .nval(bus_phase_c));
	
	assign edclk_13_gap = ~(edclk_pipe1 | edclk_pipe3);
	assign bus_access_gate_n = ~(edclk_13_gap & fifo_wr_gate);
	
	assign dma_addr_hold = dma_addr_dly & dma_addr_oe;
	
	assign any_irq_active = vint_irq | hint_irq | eint_irq;
	assign bus_idle = ~dff13_l2;
	
	assign ipl1_src = vint_irq | eint_irq;
	assign ipl2_src = hint_irq | eint_irq;
	
	assign dma_ext_gate = edclk_gap & dma_68k_mode;
	
	assign ras_z80_gate = ~bus_phase_b | (bus_phase_a & dma_ext_gate);
	
	assign cas_z80_gate = (~edclk_gap & bus_phase_a) | (bus_phase_c & dma_ext_gate);
	
	assign ras_dma_gate = (~bus_phase_b & dma_ext_gate) | (bus_phase_c & dma_ext_gate);
	
	assign cas_readback = edclk_pipe2 & bus_phase_c;
	
	assign dma_addr_bus = dma_addr_oe & edclk_gap & bus_phase_c;
	
	assign oe_late_phase = bus_late & bus_active;
	
	assign ras_early = bus_active & dff7_l2;
	
	assign ras_gated = bus_active & dff_timing_gate;
	
	assign ras_sel = bus_idle ? ras_gated : ras_early;
	
	assign dtack_gate_n = ~((bus_idle & bus_phase2) | (dff5_l2 & bus_active));
	
	assign cas_68k = bus_active & (dff6_l2 | (dff11_l2 & bus_idle));
	
	assign oe_cpu_rd = cpu_rd & cpu_rd_latch;
	
	ym7101_dff dff5(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dff6_l2), .rst(bus_idle), .outp(dff5_l2));
	ym7101_dff dff6(.MCLK(MCLK), .clk(cpu_clk1), .inp(dff7_l2), .rst(bus_idle), .outp(dff6_l2));
	
	assign dff_timing_gate = ~(~dff6_l2 & dff8_l2);
	
	ym7101_dff dff7(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dff8_l2), .rst(bus_idle), .outp(dff7_l2));
	
	assign z80_ras_pulse = dff17_l2 & ~dff19_l2;
	
	assign z80_cas_pulse = dff16_l2 & ~dff19_l2;
	
	ym7101_dff dff8(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dff9_l2), .rst(bus_idle), .outp(dff8_l2));
	
	ym7101_dff dff9(.MCLK(MCLK), .clk(cpu_clk1), .inp(bus_phase2), .rst(bus_idle), .outp(dff9_l2));
	
	assign bus_late = ~(bus_idle | dff9_l2);
	
	assign bus_phase2 = bus_active & dff10_l2;
	
	assign dtack_rst_cond = reset_comb | dff21_l2 | dff13_l2;
	
	ym7101_dff dff10(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dff11_l2), .rst(bus_invalid), .outp(dff10_l2));
	ym7101_dff dff11(.MCLK(MCLK), .clk(cpu_clk1), .inp(bus_valid), .rst(bus_invalid), .outp(dff11_l2));
	
	assign bus_wr_phase = dff11_l2 & cpu_wr_strobe;
	
	assign bus_invalid = ~bus_valid;
	
	assign bus_active = addr_hi_sel & bus_valid;
	
	assign intak_clear = ~cpu_intak;
	
	assign bus_valid = cpu_as & bus_allow;
	
	assign as_inactive = ~cpu_as;
	
	ym7101_dff dff12(.MCLK(MCLK), .clk(as_inactive), .inp(1'h1), .rst(bus_idle), .outp(dff12_l2));
	
	assign bus_cycle_rst = dff12_l2 | reset_comb;
	
	ym7101_dff dff13(.MCLK(MCLK), .clk(bus_active), .inp(arb_cnt_top), .rst(bus_cycle_rst), .outp(dff13_l2));
	
	ym7101_dff dff14(.MCLK(MCLK), .clk(cpu_clk1), .inp(arb_cnt_hi), .rst(1'h0), .outp(dff14_l2));
	
	ym7101_dff dff15(.MCLK(MCLK), .clk(dff14_l2), .inp(arb_cnt_top), .rst(dtack_rst_cond), .outp(dff15_l2));
	
	assign dtack_done = ~dff15_l2;
	
	assign bus_allow = dtack_done | dff21_l2;
	
	assign arb_cnt_rst = ~(~dff21_l2 & cpu_sel & dff_timing_gate);
	
	ym7101_dff dff16(.MCLK(MCLK), .clk(cpu_clk1), .inp(1'h1), .rst(dtack_done), .outp(dff16_l2));
	ym7101_dff dff17(.MCLK(MCLK), .clk(cpu_clk1), .inp(dff16_l2), .rst(dtack_done), .outp(dff17_l2));
	ym7101_dff dff18(.MCLK(MCLK), .clk(cpu_clk1), .inp(dff17_l2), .rst(dtack_done), .outp(dff18_l2));
	ym7101_dff dff19(.MCLK(MCLK), .clk(~cpu_clk1), .inp(dff18_l2), .rst(dtack_done), .outp(dff19_l2));
	ym7101_dff dff20(.MCLK(MCLK), .clk(cpu_clk1), .inp(dff19_l2), .rst(dtack_done), .outp(dff20_l2));
	
	ym7101_dff dff21(.MCLK(MCLK), .clk(cpu_clk1), .inp(dff20_l2), .rst(1'h0), .outp(dff21_l2));
	
	ym7101_dff dff22(.MCLK(MCLK), .clk(cpu_clk1), .inp(dma_68k_req), .rst(1'h0), .outp(dff22_l2));
	
	assign br_pull_ctl = ~(dff22_l2 & cpu_sel);
	
	wire [6:0] i_sum = {6'h0, bgack_pull_ctl} + { dff29_l2, dff28_l2, dff27_l2, dff26_l2, dff25_l2, dff24_l2, dff23_l2 };
	
	ym7101_dff dff23(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[0]), .rst(arb_cnt_rst), .outp(dff23_l2));
	ym7101_dff dff24(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[1]), .rst(arb_cnt_rst), .outp(dff24_l2));
	ym7101_dff dff25(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[2]), .rst(arb_cnt_rst), .outp(dff25_l2));
	ym7101_dff dff26(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[3]), .rst(arb_cnt_rst), .outp(dff26_l2));
	ym7101_dff dff27(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[4]), .rst(arb_cnt_rst), .outp(dff27_l2));
	ym7101_dff dff28(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[5]), .rst(arb_cnt_rst), .outp(dff28_l2));
	ym7101_dff dff29(.MCLK(MCLK), .clk(cpu_clk1), .inp(i_sum[6]), .rst(arb_cnt_rst), .outp(dff29_l2));
	
	assign arb_cnt_hi = dff25_l2 & dff24_l2 & dff26_l2 & arb_cnt_top;
	
	assign arb_cnt_top = dff28_l2 & dff27_l2 & dff29_l2;
	
	assign m68k_int_ack = cpu_as & cpu_intak;
	
	assign int_ack_any = m68k_int_ack | z80_int_ack;
	
	assign z80_int_ack = cpu_m1 & cpu_iorq;
	
	ym7101_rs_trig rs5(.MCLK(MCLK), .set(int_ack_any), .rst(int_ack_sync), .q(int_ack_latch));
	
	assign int_ack_m5 = int_ack_latch & reg_m5;
	
	ym_sr_bit sr9(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(int_ack_m5), .sr_out(int_ack_sync));
	ym_dlatch_1 dl10(.MCLK(MCLK), .c1(clk1), .inp(int_ack_sync), .nval(int_ack_dly1));
	ym_dlatch_2 dl11(.MCLK(MCLK), .c2(clk2), .inp(int_ack_dly1), .nval(int_ack_dly2));
	
	assign int_latch_rst = reset_comb | (int_ack_dly2 & int_ack_dly1);
	
	assign status_rd_set = reset_comb | vdp_data_rd_odd;
	
	ym7101_rs_trig rs6(.MCLK(MCLK), .set(status_rd_set), .rst(status_rd_dly2), .q(status_rd_pend));
	
	ym_sr_bit sr12(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(status_rd_pend), .sr_out(status_rd_dly));
	
	assign status_rd_gate = ~(status_rd_dly | reset_comb);
	
	ym_sr_bit sr13(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(status_rd_dly), .sr_out(status_rd_dly2));
	
	assign status_clr_n = ~(status_rd_dly2 & status_rd_gate);
	
	ym_dlatch_1 dl14(.MCLK(MCLK), .c1(hclk1), .inp(status_clr_n), .nval(status_clr_latch));
	
	assign m4_int_en = status_clr_latch & ~reg_m5;
	
	assign vint_clr = m4_int_en | dff30_l2;
	
	ym7101_dff dff30(.MCLK(MCLK), .clk(~int_ack_m5), .inp(vint_irq), .rst(int_latch_rst), .outp(dff30_l2));
	
	ym7101_dff dff31(.MCLK(MCLK), .clk(~int_ack_m5), .inp(hint_irq), .rst(int_latch_rst), .outp(dff31_l2));
	
	assign hint_clr = m4_int_en | dff31_l2;
	
	ym7101_dff dff32(.MCLK(MCLK), .clk(~int_ack_m5), .inp(eint_irq), .rst(int_latch_rst), .outp(dff32_l2));
	
	assign eint_clr = m4_int_en | dff32_l2;
	
	assign hint_irq = hint_pend & ~eint_irq & reg_ie1;
	
	assign vint_irq = ~hint_irq & ~eint_irq & vint_irq_pend & reg_ie2;
	
	ym7101_rs_trig rs7(.MCLK(MCLK), .set(hint_fired), .rst(hint_clr), .q(hint_pend));
	
	ym7101_rs_trig rs8(.MCLK(MCLK), .set(vint_set_cond), .rst(vint_clr), .q(vint_irq_pend));
	
	assign vint_set_cond = reg_m5 & pen_edge_dly;
	
	assign eint_irq = eint_pend & reg_ie0;
	
	assign dma_enable = reg_m1 & reg_m5;
	
	assign dma_68k_start = dma_enable & ~reg_dmd[1] & dma_start_bit;
	
	assign dma_copy_start = dma_enable & dma_start_bit & reg_dmd[1];
	
	assign bgack_pull_ctl = ~bus_granted;
	
	assign z80_hv_sel = ~(~reg_m5 | io_address[1] | cpu_sel);
	
	assign spr_of_latch = ~eint_pend & spr_overflow_flag;
	
	ym7101_rs_trig rs9(.MCLK(MCLK), .set(eint_set_cond), .rst(eint_clr), .q(eint_pend));
	
	ym7101_rs_trig rs10(.MCLK(MCLK), .set(spr_of_latch), .rst(status_clr_latch), .q(spr_overflow));
	
	ym7101_rs_trig rs11(.MCLK(MCLK), .set(spr_collision_pipe), .rst(status_clr_latch), .q(spr_collision));
	
	assign hint_cnt_tick = active_end | reg_test0[3];
	
	assign hint_cnt_reload = ~(vdisp_en_trig | vcnt_at_max | reg_test0[3]);
	
	wire cnt1_of;
	
	ym_cnt_bit_load #(.DATA_WIDTH(8)) cnt1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(hint_cnt_tick), .reset(1'h0), .load(hint_cnt_load), .load_val(reg_hit), .c_out(cnt1_of));
		
	assign hint_cnt_load = hint_cnt_reload | hint_fired;
	
	ym_sr_bit sr15(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cnt1_of & ~hint_cnt_reload), .sr_out(hint_fired));
	
	assign vdp_addr_68k = cpu_sel & (io_address & 23'h738070) == 23'h600000;
	
	assign z80_vdp_rd = ~cpu_sel & reg_test0[2] & hv_rd_any;

	assign hv_byte_sel = z80_hv_sel ? fifo_wr_gate : cpu_pal;
	
	assign hv_data_sel = z80_hv_sel ? dma_state : dff3_l2;
	
	assign tst18_sel_f = reg_test_18[11:8] == 4'hf;
	assign tst18_sel_8 = reg_test_18[11:8] == 4'h8;
	assign tst18_sel_7 = reg_test_18[11:8] == 4'h7;
	assign tst18_sel_6 = reg_test_18[11:8] == 4'h6;
	assign tst18_sel_5 = reg_test_18[11:8] == 4'h5;
	assign tst18_sel_4 = reg_test_18[11:8] == 4'h4;
	assign tst18_sel_3 = reg_test_18[11:8] == 4'h3;
	assign tst18_sel_2 = reg_test_18[11:8] == 4'h2;
	assign tst18_sel_1 = reg_test_18[11:8] == 4'h1;
	assign tst18_sel_0 = reg_test_18[11:8] == 4'h0;
	
	assign tst_fn0_wr = tst18_sel_0 & tst_fn_wr;
	assign tst_fn1_wr = tst18_sel_1 & tst_fn_wr;
	assign tst_fn2_wr = tst18_sel_2 & tst_fn_wr;
	assign tst_fn2_rd = tst18_sel_2 & tst_fn_rd;
	assign tst_fn3_wr = tst18_sel_3 & tst_fn_wr;
	assign tst_fn3_rd = tst18_sel_3 & tst_fn_rd;
	assign tst_fn4_wr = tst18_sel_4 & tst_fn_wr;
	assign tst_fn4_rd = tst18_sel_4 & tst_fn_rd;
	assign tst_fn5_wr = tst18_sel_5 & tst_fn_wr;
	assign tst_fn5_rd = tst18_sel_5 & tst_fn_rd;
	assign tst_fn6_wr = tst18_sel_6 & tst_fn_wr;
	assign tst_fn6_rd = tst18_sel_6 & tst_fn_rd;
	assign tst_fn7_wr = tst18_sel_7 & tst_fn_wr;
	assign tst_fn7_rd = tst18_sel_7 & tst_fn_rd;
	assign tst_fn8_wr = tst18_sel_8 & tst_fn_wr;
	assign tst_fn8_rd = tst18_sel_8 & tst_fn_rd;
	assign int_reset_state = ~(tst18_sel_f & tst_fn_wr);
	
	assign z80_addr_gate = ras_dma_gate | dff11_l2 | io_m1_s3;
	
	assign z80_cas_cond = io_m1_s3 & ra_addr_valid;
	
	assign ra_addr_mux = reg_8b_b7 ? color_readback : (z80_addr_gate ? 
		{ io_address[15], io_address[13], io_address[12], io_address[11],
			io_address[10], io_address[9], io_address[8], io_address[14] } :
			io_address[7:0]);
	
	assign ra_addr_valid = reg_8b_b7 ? 1'h1 : (z80_addr_gate ? 1'h1 : 1'h0);
	
	ym_dlatch_1 #(.DATA_WIDTH(8)) dl16(.MCLK(MCLK), .c1(hclk1), .inp({ sh_highlight_mux, color_rd_sel, color_pal, color_index}), .val(color_readback));
	
	assign color_rd_sel = reg_test0[0] ? color_priority : sh_shadow_mux;
	
	assign interlace_dblres = reg_lsm0_latch & reg_lsm1_latch;
	
	assign v28_mode = ~reg_m2 & reg_m5;
	assign v30_mode = reg_m2 & reg_m5;
	assign vram_128k = reg_m5 & reg_81_b7;
	
	assign z80_psg_wr = io_address[7:6] == 2'h1 & cpu_iorq & cpu_wr; // z80 psg
	assign psg_wr_any = z80_psg_wr | (vdp_ctrl_wr & cpu_lds);
	
	assign z80_data_rd = io_address[7:6] == 2'h2 & cpu_iorq & cpu_rd;
	assign vdp_data_rd = z80_data_rd | vdp_data_port_rd;
	assign vdp_data_rd_odd = vdp_data_rd & byte_sel;
	
	assign z80_bus_ext = reg_8b_b6 & bus_phase2;
	assign cas_ext_68k = z80_bus_ext | cas_68k;
	
	assign dtack_pull_ctl = ~(cas_68k | dtack_68k | tst_reg_wr | tst_fn_wr | vdp_ctrl_wr); // dtack
	
	assign oe_comb = (cpu_wr_strobe & 1'h0) | (bus_wr_phase & cas_ext_68k) | oe_late_phase;
	
	ym_slatch sl17(.MCLK(MCLK), .en(cpu_clk0), .inp(cpu_rd), .val(cpu_rd_latch));
	
	assign int_timing_sel = cpu_sel ? line_zero_dly : active_end;
	
	assign eint_set_cond = int_timing_sel & vdisp_ended;
	
	assign z80_int_rst = reset_comb | disp_start;
	
	ym7101_rs_trig rs12(.MCLK(MCLK), .set(eint_set_cond), .rst(z80_int_rst), .q(z80_int_trig));
	
	assign int_pull_ctl = ~(cpu_sel ? z80_int_trig : any_irq_active); // z80 int
	
	assign vcnt_bit_sel = reg_lsm0_latch ? vcnt_ext[8] : vcnt_ext[0];
	
	assign vdp_access_valid = vdp_addr_68k & cpu_as & access_pending;
	
	assign dtack_68k = cpu_sel & dtack_any;
	
	assign dtack_rd_gate = cpu_rd_any & rd_gate;
	
	assign dtack_wr_gate = wr_fifo_ready & ctrl_port_wr;
	
	assign tst_reg_wr = vdp_access_valid & cpu_rw & io_address[3:2] == 2'h3 & ~byte_sel; // test address
	
	assign tst_fn_wr = vdp_access_valid & cpu_rw & io_address[3:2] == 2'h3 & byte_sel;
	
	assign byte_sel = cpu_sel ? io_address[1] : io_address[0];
	
	assign vdp_data_wr = vdp_access_valid & cpu_rw & io_address[3:2] == 2'h0 & any_byte_sel;
	
	assign vdp_data_port_rd = vdp_access_valid & ~cpu_rw & io_address[3:2] == 2'h0 & any_byte_sel;
	
	assign vdp_ctrl_wr = vdp_access_valid & cpu_rw & io_address[3:2] == 2'h2;
	
	assign hv_cnt_rd = vdp_access_valid & ~cpu_rw & io_address[3:2] == 2'h1;
	
	assign tst_fn_rd = vdp_access_valid & ~cpu_rw & io_address[3:2] == 2'h3;
	
	ym7101_rs_trig rs13(.MCLK(MCLK), .set(vdp_io_any), .rst(access_rst), .q(access_strobe));
	
	assign dma_done_rst = dma_len_done_dly | reset_comb;
	
	assign dma_bus_cond = bus_br_hold & fifo_ctrl_pend & dma_auto_inc;
	
	ym7101_rs_trig rs14(.MCLK(MCLK), .set(fifo_ctrl_set), .rst(data_rd_first), .q(fifo_ctrl_pend));
	
	assign fifo_ctrl_set = reset_comb | vram_wr_hi_pipe;
	
	assign vdp_write_any = vdp_data_wr | z80_vdp_wr;
	
	assign z80_vdp_wr = cpu_iorq & cpu_wr & io_address[7:6] == 2'h2;
	
	assign z80_hv_rd = cpu_iorq & cpu_rd & io_address[7:6] == 2'h1;
	
	assign hv_rd_any = z80_hv_rd | hv_cnt_rd; // HV cnt read
	
	ym_slatch sl18(.MCLK(MCLK), .en(~vdp_write_any), .inp(byte_sel), .val(wr_byte_latch));
	
	ym_slatch sl19(.MCLK(MCLK), .en(~vdp_data_rd), .inp(byte_sel), .val(rd_byte_latch));
	
	assign data_rd_first = ~rd_byte_latch & vdp_data_rd;
	
	ym7101_rs_trig rs15(.MCLK(MCLK), .set(ctrl_phase_set), .rst(ctrl_phase_rst), .q(ctrl_wr_phase), .nq(ctrl_wr_phase_n));
	
	ym7101_rs_trig rs16(.MCLK(MCLK), .set(z80_data_wr), .rst(fifo_adv_rst), .q(data_wr_phase), .nq(data_wr_phase_n));
	
	ym7101_rs_trig rs17(.MCLK(MCLK), .set(fifo_wr_set), .rst(fifo_phase_b), .q(fifo_wr_pend));
	
	assign fifo_wr_rst = (fifo_wr_pend & fifo_ready) | reset_comb;
	
	assign dma_wr_cond = fifo_ready & data_wr_latch & code_not_vram & reg_m5;
	
	ym7101_rs_trig rs18(.MCLK(MCLK), .set(dma_wr_cond), .rst(fifo_wr_rst), .q(dma_inc_mode), .nq(dma_inc_mode_n));
	
	assign dtack_any = dtack_rd_gate | dtack_wr_gate | dma_bus_cond | ctrl_wr_dtack | data_wr_first;
	
	ym7101_rs_trig rs19(.MCLK(MCLK), .set(cpu_uds), .rst(fifo_pipe_rst), .q(uds_latch));
	
	ym7101_rs_trig rs20(.MCLK(MCLK), .set(cpu_lds), .rst(fifo_pipe_rst), .q(lds_latch));
	
	assign any_byte_sel = cpu_uds | cpu_lds;
	
	assign wr_fifo_ready = ~(fifo_busy | fifo_wr_gate);
	
	assign fifo_rd_cond = ctrl_wr_done & fifo_ready;
	
	ym7101_rs_trig rs21(.MCLK(MCLK), .set(ctrl_port_wr), .rst(fifo_pipe_rst), .q(ctrl_wr_done));
	
	assign fifo_busy = ctrl_wr_done & fifo_phase_a;
	
	assign cpu_data_oe = cpu_rd_any | z80_int_ack;
	
	assign cpu_rd_any = vdp_data_rd | tst_fn_rd | hv_rd_any;
	
	assign fifo_phase_a = fifo_stg3 & ~fifo_stg1;
	
	assign fifo_ready = ~fifo_stg1 & ~fifo_stg2;
	
	assign fifo_phase_b = fifo_stg3 & fifo_stg2;
	
	assign dma_cycle_rst = dma_fill_dly | dma_done_rst;
	
	assign dma_start_cond = dma_or_vram_wr & dma_ext_dly;
	
	assign access_pending = access_strobe | fifo_stg2;
	
	assign access_rst = fifo_stg1 | reset_comb;
	
	assign data_rd_even = vdp_data_rd & ~byte_sel;
	
	assign fifo_idle_rst = fifo_ready | reset_comb;
	
	ym7101_rs_trig rs22(.MCLK(MCLK), .set(fifo_idle_rst), .rst(vram_wr_hi_pipe), .q(rd_phase_pend));
	
	assign rd_gate = ~(rd_phase_pend & data_rd_even);
	
	assign vdp_io_any = vdp_data_rd | vdp_write_any;
	
	assign ctrl_wr_dtack = (cpu_sel & data_wr_first) | z80_ctrl_repeat;
	
	assign data_wr_first = vdp_write_any & dma_inc_mode_n & ctrl_wr_phase_n & wr_byte_latch;
	
	assign z80_data_wr = data_wr_first & ~cpu_sel;
	
	assign z80_ctrl_repeat = wr_byte_latch & z80_vdp_wr & ctrl_wr_phase;
	
	assign dma_auto_inc = vdp_write_any & ctrl_wr_phase_n & wr_byte_latch & dma_inc_mode & reg_m5;
	
	assign ctrl_port_wr = ~wr_byte_latch & vdp_write_any;
	
	assign fifo_advance = ctrl_wr_dtack | ctrl_port_wr | data_rd_even | dma_auto_inc | vdp_data_rd_odd;
	
	ym7101_rs_trig rs23(.MCLK(MCLK), .set(dma_auto_inc), .rst(fifo_flush_rst), .q(dma_inc_latch));
	
	ym7101_rs_trig rs24(.MCLK(MCLK), .set(data_rd_first), .rst(fifo_flush_rst), .q(data_rd_latch));
	
	assign fifo_write_any = data_rd_even | ctrl_port_wr | dma_auto_inc | vdp_data_rd_odd;
	
	assign ctrl_phase_rst = (data_wr_phase_n & fifo_ready) | reset_comb;
	
	assign ctrl_phase_set = fifo_ready & data_wr_phase;
	
	assign fifo_adv_rst = reset_comb | fifo_advance;
	
	assign fifo_wr_set = reset_comb | fifo_write_any;
	
	ym7101_rs_trig rs25(.MCLK(MCLK), .set(ctrl_wr_dtack), .rst(fifo_flush_rst), .q(data_wr_latch));
	
	assign fifo_flush_rst = reset_comb | fifo_phase_b;
	
	assign vram_wr_sel = ~(cram_wr_sel | vsram_wr_sel);
	
	assign vsram_wr_normal = vsram_wr_sel & ~reg_test0[4];
	
	assign vsram_wr_test = vsram_wr_sel & reg_test0[4];
	
	assign dma_or_vram_wr = dma_copy_mode | vram_byte_swap;
	
	ym_dlatch_1 dl20(.MCLK(MCLK), .c1(clk1), .inp(dma_addr_bus), .val(dma_addr_latch));
	
	assign reg_data_wr_en = dma_addr_latch | ctrl_wr_dtack | dma_auto_inc | dma_data_active | fifo_rd_trigger;
	
	assign dma_start_bit = dma_auto_inc & io_data[7];
	
	assign fifo_pipe_rst = (fifo_stg4 & ~fifo_stg3) | reset_comb;
	
	assign fifo_idle = ~(fifo_stg2 | fifo_stg0);
	
	ym_dlatch_1 dl21(.MCLK(MCLK), .c1(clk1), .inp(~fifo_stg3), .nval(fifo_stg4));
	ym_dlatch_2 dl22(.MCLK(MCLK), .c2(clk2), .inp(fifo_stg2), .nval(fifo_stg3));
	ym_dlatch_1 dl23(.MCLK(MCLK), .c1(clk1), .inp(fifo_stg1), .nval(fifo_stg2));
	
	ym_slatch dl24(.MCLK(MCLK), .en(clk2), .inp(fifo_stg0), .val(fifo_stg1));
	ym_slatch dl25(.MCLK(MCLK), .en(clk1), .inp(access_strobe), .val(fifo_stg0));
	
	ym_dlatch_1 dl26(.MCLK(MCLK), .c1(hclk1), .inp(dma_data_active), .nval(dma_data_latch));
	
	assign reg_addr_load = dma_data_latch & fifo_stg1;
	
	ym7101_rs_trig rs26(.MCLK(MCLK), .set(dma_start_cond), .rst(dma_cycle_rst), .q(dma_fill_trig), .nq(dma_fill_trig_n));
	
	assign dma_fill_start = dma_fill_trig & dma_state & slot_idle_dly;
	
	ym_sr_bit sr27(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_fill_start), .sr_out(dma_fill_dly));
	
	assign dma_copy_cycle = dma_fill_dly & dma_copy_mode;
	
	assign dma_fill_cycle = ~dma_copy_mode & dma_fill_dly;
	
	assign dma_wr_path = dma_ext_dly | dma_fill_cycle;
	
	assign dma_ext_cycle = dma_ext_dly & ~dma_copy_mode;
	
	assign dma_data_active = dma_ext_cycle | dma_copy_cycle | dma_fill_slot_dly;
	
	assign code_not_vram = reg_code[1:0] != 2'h2;
	
	assign cram_wr_gate = ~(~code_not_vram & fifo_idle & data_wr_latch);
	
	assign auto_rd_cond = ~(data_rd_latch | dma_inc_latch | (data_wr_latch & ~reg_m5));
	
	assign dma_ext_copy = dma_ext_dly & dma_copy_mode;
	
	ym_sr_bit sr28(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_ext_cond), .sr_out(dma_ext_dly));
	
	assign dma_ext_cond = dma_fill_trig_n & dma_start_pend & dma_state & slot_idle_dly;
	
	ym7101_rs_trig rs27(.MCLK(MCLK), .set(dma_pend_set), .rst(dma_pend_clr), .q(dma_start_pend));
	
	assign dma_pend_clr = dma_ext_dly | dma_done_rst;
	
	assign dma_pend_set = dma_copy_trig | dma_norm_trig | sms_rd_trig;
	
	assign dma_copy_trig = dma_copy_mode & reg_code[4];
	
	assign dma_norm_trig = ~((reg_code[4] | reg_code[1] | reg_code[0]) | auto_rd_cond | ~fifo_ready);
	
	assign cram_wr_sel = dma_wr_path & reg_code[3:2] == 2'h1;
	
	assign vsram_wr_sel = dma_wr_path & reg_code[3:2] == 2'h2;
	
	ym_sr_bit sr29(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_sel), .sr_out(vram_wr_pipe1));
	
	ym_sr_bit sr30(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_pipe1), .sr_out(vram_wr_pipe2));
	
	assign sms_rd_trig = fifo_ready & data_rd_latch & ~reg_m5;
	
	assign code_hi_rst = reset_comb | ~reg_m5;
	
	ym_dlatch_2 dl31(.MCLK(MCLK), .c2(hclk2), .inp(cram_rd_active), .nval(reg_wr_pipe));
	
	ym_dlatch_2 dl32(.MCLK(MCLK), .c2(clk2), .inp(cram_wr_gate), .nval(cram_gate_pipe));
	
	ym_sr_bit sr33(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(cram_gate_pipe), .sr_out(cram_rd_pipe2));
	
	ym_dlatch_2 dl34(.MCLK(MCLK), .c2(clk1), .inp(cram_gate_pipe | cram_rd_pipe2), .nval(cram_rd_active));
	
	assign reg_wr_strobe = reg_wr_pipe & hclk1;
	
	assign reg_sel_grp0 = reg_wr_strobe & reg_data_l2[12:11] == 2'h0;
	assign reg_sel_grp1 = reg_wr_strobe & reg_data_l2[12:11] == 2'h1;
	assign reg_sel_grp2 = reg_wr_strobe & reg_data_l2[12:11] == 2'h2 & reg_m5;
	assign reg_sel_grp1m5 = reg_wr_strobe & reg_data_l2[12:11] == 2'h1 & reg_m5;
	
	assign wr_en_8F = (reg_sel_grp1m5 & reg_data_l2[10:8] == 3'h7) | reset_comb; // 8f
	assign wr_en_93 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h3) | reset_comb; // 93
	assign wr_en_94 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h4) | reset_comb; // 94
	assign wr_en_8B_scr = (reg_sel_grp1m5 & reg_data_l2[10:8] == 3'h3) | reset_comb; // 8b
	assign wr_en_96 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h6) | reset_comb; // 96
	assign wr_en_8C = (reg_sel_grp1m5 & reg_data_l2[10:8] == 3'h4) | reset_comb; // 8c
	assign reg_wr_80 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h0) | reset_comb; // 80
	assign reg_wr_81 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h1) | reset_comb; // 81
	assign reg_wr_82 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h2) | reset_comb; // 82
	assign reg_wr_83 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h3) | reset_comb; // 83
	assign reg_wr_84 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h4) | reset_comb; // 84
	assign reg_wr_85 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h7) | reset_comb; // 87
	assign reg_wr_86 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h2) | reset_comb; // 92
	assign reg_wr_87 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h1) | reset_comb; // 91
	assign reg_wr_88 = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h0) | reset_comb; // 90
	assign reg_wr_89 = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h6) | reset_comb; // 86
	assign reg_wr_8A = (reg_sel_grp0 & reg_data_l2[10:8] == 3'h5) | reset_comb; // 85
	assign reg_wr_8B = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h7) | reset_comb; // 97
	assign reg_wr_8C = (reg_sel_grp2 & reg_data_l2[10:8] == 3'h5) | reset_comb; // 95
	assign reg_wr_8D = (reg_sel_grp1 & reg_data_l2[10:8] == 3'h2) | reset_comb; // 8a
	assign reg_wr_hi = (reg_sel_grp1 & reg_data_l2[10:8] == 3'h1) | reset_comb; // 89
	assign wr_en_88 = (reg_sel_grp1 & reg_data_l2[10:8] == 3'h0) | reset_comb; // 88
	assign wr_en_8E = (reg_sel_grp1m5 & reg_data_l2[10:8] == 3'h6) | reset_comb; // 8e
	assign wr_en_8D_m5 = (reg_sel_grp1m5 & reg_data_l2[10:8] == 3'h5) | reset_comb; // 8d
	
	assign dma_cnt_tst = dma_wr_active & reg_test0[1];
	
	assign dma_cnt_norm = dma_wr_active & ~reg_test0[1];
	
	ym_slatch #(.DATA_WIDTH(17)) sl35(.MCLK(MCLK), .en(fifo_addr_wr_en), .inp(vram_address), .val(fifo_addr_pipe));
	ym_slatch #(.DATA_WIDTH(17)) sl36(.MCLK(MCLK), .en(fifo_en_s1), .inp(reg_data_l2), .val(fifo_data_0));
	ym_slatch #(.DATA_WIDTH(17)) sl37(.MCLK(MCLK), .en(fifo_en_s0), .inp(reg_data_l2), .val(fifo_data_1));
	ym_slatch #(.DATA_WIDTH(17)) sl38(.MCLK(MCLK), .en(fifo_en_s3), .inp(reg_data_l2), .val(fifo_data_2));
	ym_slatch #(.DATA_WIDTH(17)) sl39(.MCLK(MCLK), .en(fifo_en_s2), .inp(reg_data_l2), .val(fifo_data_3));
	
	assign dma_len_last = reg_lg == 16'hfffe & dma_wr_active;
	
	assign dma_copy_mode = dff3_l2 & reg_dmd == 2'h3;
	assign dma_68k_mode = dff3_l2 & reg_dmd == 2'h1;
	assign dma_fill_mode = dff3_l2 & reg_dmd == 2'h0;
	assign dma_ext_mode = dff3_l2 & reg_dmd == 2'h2;
	
	ym_sr_bit sr40(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(dma_addr_bus), .sr_out(dma_addr_pipe));
	
	assign dma_addr_edge = dma_addr_pipe & dma_addr_bus;
	
	ym_dlatch_2 dl41(.MCLK(MCLK), .c2(hclk2), .inp(dma_addr_bus), .val(dma_addr_latch2));
	
	assign dma_wr_active = dma_copy_cycle | dma_fill_slot_dly | dma_addr_latch2;
	
	wire reg_lg_of;
	
	assign dma_len_hi_cry = reg_lg_of | dma_cnt_tst;
	
	ym_sr_bit sr42(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(fifo_wr_trig), .sr_out(fifo_wr_pipe1));
	
	assign fifo_wr_gate = (~reset_comb & fifo_ptr_match & fifo_wr_pipe2) | (~reset_comb & fifo_ptr_match & fifo_wr_pipe1);
	
	ym_sr_bit sr43(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(fifo_wr_gate), .sr_out(fifo_wr_pipe2));
	
	ym_sr_bit sr44(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(idle_slot_dly), .sr_out(dma_slot_pipe1));
	
	ym_sr_bit sr45(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(dma_slot_pipe1), .sr_out(dma_slot_pipe2));
	
	assign dma_slot_edge = dma_slot_pipe1 & ~dma_slot_pipe2 & ~fifo_cnt_b0;
	
	assign dma_state_set = reset_comb | (dma_state & fifo_ptr_match) | (fifo_ptr_match & dma_slot_edge);
	
	wire reg_sa_of;
	
	assign dma_src_hi_cry = reg_sa_of | dma_cnt_tst;
	
	ym_sr_bit sr46(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(dma_state_set), .sr_out(dma_state));
	
	assign fifo_empty_slot = idle_slot_dly & ~fifo_cnt_b0;
	
	assign fifo_rd_s2 = fifo_empty_slot & fifo_rd_b0 & ~fifo_rd_b1;
	assign fifo_rd_s0 = fifo_empty_slot & ~fifo_rd_b0 & ~fifo_rd_b1;
	assign fifo_rd_s3 = fifo_empty_slot & fifo_rd_b0 & fifo_rd_b1;
	assign fifo_rd_s1 = fifo_empty_slot & ~fifo_rd_b0 & fifo_rd_b1;
	
	assign fifo_rd_trigger = fifo_rd_cond | 1'h0;
	
	assign dma_ext_busy = dma_ext_mode & fifo_read_slot;
	
	assign dma_ext_state = dma_ext_mode & dma_state;
	
	assign fifo_wr_trig = dma_addr_edge | fifo_busy;
	
	assign dma_copy_active = dma_copy_mode & dma_state;
	
	ym_slatch sl47(.MCLK(MCLK), .en(addr_latch_en), .inp(vram_address[0]), .val(vram_addr_b0_lat));
	
	assign addr_latch_en = hclk1 & odd_slot;
	
	assign dma_addr_oe = dma_fill_mode | dma_68k_mode;
	
	ym_sr_bit sr48(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_len_last), .sr_out(dma_len_done_dly));
	
	assign non_dma_slot = ~dma_copy_active & odd_slot;
	
	assign slot_no_dma = slot_idle_dly & ~dma_state;
	
	ym_sr_bit sr49(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(slot_no_dma), .sr_out(idle_slot_dly));
	
	assign dma_active_rst = reset_comb | dma_len_done_dly;
	
	assign dma_active_set = dff3_l2 & idle_slot_dly;
	
	ym7101_rs_trig rs28(.MCLK(MCLK), .set(dma_active_set), .rst(dma_active_rst), .q(dma_active_trig));
	
	ym_sr_bit sr50(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_fill_cond), .sr_out(dma_fill_slot_dly));
	
	assign dma_fill_cond = dma_active_trig & dma_ext_state & slot_idle_dly;
	
	assign fifo_vram_sel = vram_sel & vram_code_sel;
	
	assign vram_sel = ~vram_128k & cpu_sel;
	
	assign dma_fill_busy = dma_fill_cycle | fifo_read_slot;
	
	assign uds_or_dff3 = dff3_l2 | uds_latch;
	
	assign byte_sel_hi = cpu_sel ? uds_or_dff3 : addr_b0_m5;
	
	assign lds_or_dff3 = dff3_l2 | lds_latch;
	
	assign byte_sel_lo = cpu_sel ? lds_or_dff3 : ~addr_b0_m5;
	
	assign addr_b0_m5 = reg_data_l2[0] & reg_m5;
	
	assign dma_cpy_hi_sel = dma_copy_cycle & (vram_128k | vram_address[0]);
	
	assign dma_cpy_lo_sel = dma_copy_cycle & (vram_128k | ~vram_address[0]);
	
	assign vram_wr_hi_en = dma_cpy_hi_sel | (vram_wr_active & out_uds_sel);
	
	assign vram_wr_lo_en = dma_cpy_lo_sel | (vram_wr_active & out_lds_sel);
	
	assign lo_byte_valid = out_lds_sel | ~vram_128k;
	
	assign vram_wr_lo_gate = dma_copy_cycle | (vram_wr_active & lo_byte_valid);
	
	assign vram128k_hi = vram_128k & out_uds_sel;
	
	assign vram_wr_hi_gate = dma_copy_cycle | (vram128k_hi & vram_wr_active);
	
	assign vram_wr_active = fifo_out_active & ~out_cd1_sel & ~out_cd2_sel & ~out_cd3_sel;
	
	assign fifo_slot_en = fifo_rd_cond | (dma_addr_edge & clk1);
	
	ym_cnt_bit #(.DATA_WIDTH(2)) cnt2(.MCLK(MCLK), .c1(clk1), .c2(clk2),
		.c_in(fifo_wr_trig), .reset(reset_comb), .val(fifo_wr_ptr));
	
	assign fifo_en_s2 = fifo_slot_en & fifo_wr_ptr == 2'h2;
	assign fifo_en_s3 = fifo_slot_en & fifo_wr_ptr == 2'h3;
	assign fifo_en_s0 = fifo_slot_en & fifo_wr_ptr == 2'h0;
	assign fifo_en_s1 = fifo_slot_en & fifo_wr_ptr == 2'h1;
	
	assign fifo_ptr_match = fifo_rd_b1 == fifo_wr_ptr[1] & fifo_rd_b0 == fifo_wr_ptr[0]; //fifo_wr_ptr == { fifo_rd_b1, fifo_rd_b0 };
	
	assign fifo_cnt_inc = idle_slot_dly & fifo_rd_valid;
	assign fifo_cnt_dec = idle_slot_dly & ~fifo_rd_valid;
	
	wire [2:0] l52_sum = reset_comb ? 3'h0 : ({fifo_rd_b1, fifo_rd_b0, fifo_cnt_b0} + { 1'h0, fifo_cnt_dec, fifo_cnt_inc });
	
	ym_sr_bit sr52(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l52_sum[0]), .sr_out(fifo_cnt_b0));
	ym_sr_bit sr53(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l52_sum[1]), .sr_out(fifo_rd_b0));
	ym_sr_bit sr54(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l52_sum[2]), .sr_out(fifo_rd_b1));
	
	assign fifo_not_busy = ~(fifo_cnt_b0 | dma_fill_cycle);
	
	assign fifo_addr_wr_en = hclk1 & (dma_data_active | (idle_slot_dly & fifo_not_busy));
	
	assign fifo_read_slot = fifo_vram_sel & idle_slot_dly & fifo_cnt_b0;
	
	ym_slatch sl55(.MCLK(MCLK), .en(fifo_en_s3), .inp(byte_sel_hi), .val(fifo_uds_s3));
	ym_slatch sl56(.MCLK(MCLK), .en(fifo_en_s2), .inp(byte_sel_hi), .val(fifo_uds_s2));
	ym_slatch sl57(.MCLK(MCLK), .en(fifo_en_s1), .inp(byte_sel_hi), .val(fifo_uds_s1));
	ym_slatch sl58(.MCLK(MCLK), .en(fifo_en_s0), .inp(byte_sel_hi), .val(fifo_uds_s0));
	ym_slatch sl59(.MCLK(MCLK), .en(fifo_en_s3), .inp(byte_sel_lo), .val(fifo_lds_s3));
	ym_slatch sl60(.MCLK(MCLK), .en(fifo_en_s2), .inp(byte_sel_lo), .val(fifo_lds_s2));
	ym_slatch sl61(.MCLK(MCLK), .en(fifo_en_s1), .inp(byte_sel_lo), .val(fifo_lds_s1));
	ym_slatch sl62(.MCLK(MCLK), .en(fifo_en_s0), .inp(byte_sel_lo), .val(fifo_lds_s0));
	ym_slatch sl63(.MCLK(MCLK), .en(fifo_en_s3), .inp(reg_code[0]), .val(fifo_cd0_s3));
	ym_slatch sl64(.MCLK(MCLK), .en(fifo_en_s2), .inp(reg_code[0]), .val(fifo_cd0_s2));
	ym_slatch sl65(.MCLK(MCLK), .en(fifo_en_s1), .inp(reg_code[0]), .val(fifo_cd0_s1));
	ym_slatch sl66(.MCLK(MCLK), .en(fifo_en_s0), .inp(reg_code[0]), .val(fifo_cd0_s0));
	ym_slatch sl67(.MCLK(MCLK), .en(fifo_en_s3), .inp(reg_code[1]), .val(fifo_cd1_s3));
	ym_slatch sl68(.MCLK(MCLK), .en(fifo_en_s2), .inp(reg_code[1]), .val(fifo_cd1_s2));
	ym_slatch sl69(.MCLK(MCLK), .en(fifo_en_s1), .inp(reg_code[1]), .val(fifo_cd1_s1));
	ym_slatch sl70(.MCLK(MCLK), .en(fifo_en_s0), .inp(reg_code[1]), .val(fifo_cd1_s0));
	ym_slatch sl71(.MCLK(MCLK), .en(fifo_en_s3), .inp(reg_code[2]), .val(fifo_cd2_s3));
	ym_slatch sl72(.MCLK(MCLK), .en(fifo_en_s2), .inp(reg_code[2]), .val(fifo_cd2_s2));
	ym_slatch sl73(.MCLK(MCLK), .en(fifo_en_s1), .inp(reg_code[2]), .val(fifo_cd2_s1));
	ym_slatch sl74(.MCLK(MCLK), .en(fifo_en_s0), .inp(reg_code[2]), .val(fifo_cd2_s0));
	ym_slatch sl75(.MCLK(MCLK), .en(fifo_en_s3), .inp(reg_code[3]), .val(fifo_cd3_s3));
	ym_slatch sl76(.MCLK(MCLK), .en(fifo_en_s2), .inp(reg_code[3]), .val(fifo_cd3_s2));
	ym_slatch sl77(.MCLK(MCLK), .en(fifo_en_s1), .inp(reg_code[3]), .val(fifo_cd3_s1));
	ym_slatch sl78(.MCLK(MCLK), .en(fifo_en_s0), .inp(reg_code[3]), .val(fifo_cd3_s0));
	
	assign fifo_out_active = out_cd0_sel & (dma_fill_slot_dly | idle_slot_dly);
	
	assign cram_wr_hi_cond = fifo_out_active & out_cd1_sel & ~out_cd3_sel & out_uds_sel & ~out_cd2_sel;
	
	assign cram_wr_lo_cond = fifo_out_active & out_cd1_sel & ~out_cd3_sel & out_lds_sel & ~out_cd2_sel;
	
	assign fifo_data_wr_en = dma_addr_edge | ctrl_port_wr;
	
	assign fifo_rd_valid = ~(~fifo_vram_sel | uds_lds_diff);
	
	assign rd_ptr_3 = fifo_rd_b0 & fifo_rd_b1;
	assign rd_ptr_2 = ~fifo_rd_b0 & fifo_rd_b1;
	assign rd_ptr_1 = fifo_rd_b0 & ~fifo_rd_b1;
	assign rd_ptr_0 = ~fifo_rd_b0 & ~fifo_rd_b1;
	
	assign fifo_out_cd1 = (rd_ptr_3 & fifo_cd1_s3) | (rd_ptr_2 & fifo_cd1_s2) | (rd_ptr_1 & fifo_cd1_s1) | (rd_ptr_0 & fifo_cd1_s0);
	
	assign fifo_out_uds = (rd_ptr_3 & fifo_uds_s3) | (rd_ptr_2 & fifo_uds_s2) | (rd_ptr_1 & fifo_uds_s1) | (rd_ptr_0 & fifo_uds_s0);
	
	assign fifo_out_cd2 = (rd_ptr_3 & fifo_cd2_s3) | (rd_ptr_2 & fifo_cd2_s2) | (rd_ptr_1 & fifo_cd2_s1) | (rd_ptr_0 & fifo_cd2_s0);
	
	assign fifo_out_lds = (rd_ptr_3 & fifo_lds_s3) | (rd_ptr_2 & fifo_lds_s2) | (rd_ptr_1 & fifo_lds_s1) | (rd_ptr_0 & fifo_lds_s0);
	
	assign fifo_out_cd3 = (rd_ptr_3 & fifo_cd3_s3) | (rd_ptr_2 & fifo_cd3_s2) | (rd_ptr_1 & fifo_cd3_s1) | (rd_ptr_0 & fifo_cd3_s0);
	
	assign fifo_out_cd0 = (rd_ptr_3 & fifo_cd0_s3) | (rd_ptr_2 & fifo_cd0_s2) | (rd_ptr_1 & fifo_cd0_s1) | (rd_ptr_0 & fifo_cd0_s0);
	
	assign out_cd1_sel = dma_fill_slot_dly ? reg_code[1] : fifo_out_cd1;
	
	assign out_uds_sel = dma_fill_slot_dly ? byte_sel_hi : fifo_out_uds;
	
	assign out_cd2_sel = dma_fill_slot_dly ? reg_code[2] : fifo_out_cd2;
	
	assign out_lds_sel = dma_fill_slot_dly ? byte_sel_lo : fifo_out_lds;
	
	assign out_cd3_sel = dma_fill_slot_dly ? reg_code[3] : fifo_out_cd3;
	
	assign out_cd0_sel = dma_fill_slot_dly ? reg_code[0] : out_cd0_raw;
	
	assign out_cd0_raw = fifo_out_cd0 | ~reg_m5;
	
	assign vsram_wr_hi = fifo_out_active & out_uds_sel & ~out_cd1_sel & out_cd2_sel & ~out_cd3_sel;
	
	assign vsram_wr_lo = fifo_out_active & out_lds_sel & ~out_cd1_sel & out_cd2_sel & ~out_cd3_sel;
	
	assign vram_code_sel = ~out_cd2_sel & ~out_cd1_sel;
	
	assign data_rd_s0 = non_dma_slot & ~fifo_rd_b0 & ~fifo_rd_b1;
	assign data_rd_s2 = non_dma_slot & ~fifo_rd_b0 & fifo_rd_b1;
	assign data_rd_s1 = non_dma_slot & fifo_rd_b0 & ~fifo_rd_b1;
	assign data_rd_s3 = non_dma_slot & fifo_rd_b0 & fifo_rd_b1;
	
	assign byte_swap_gate = ~(fifo_vram_sel & uds_lds_diff);
	
	assign fifo_byte_sel = byte_swap_gate ? fifo_cnt_b0 : fifo_out_uds;
	
	assign uds_lds_diff = fifo_out_uds ^ fifo_out_lds;
	
	assign hv_cnt_sel = cpu_sel | io_address[0];
	
	assign hv_latch_ext = ~(~reg_m5 | reg_m3);
	
	ym_sr_bit sr79(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cpu_pen), .sr_out(pen_dly1));
	ym_sr_bit sr80(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(pen_dly1), .sr_out(pen_dly2));
	
	assign pen_fall_edge = ~pen_dly1 & pen_dly2;
	
	ym_sr_bit sr81(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(pen_fall_edge), .sr_out(pen_edge_dly));
	
	assign vcnt_latch_en = ~reg_m5 | hcnt_latch_en;
	
	assign hcnt_latch_en = hv_latch_ext | (hclk1 & pen_edge_dly);
	
	assign vram_direct = ~(reg_code[3] | reg_code[2]);
	
	ym_sr_bit sr82(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_hi_sel), .sr_out(vram_wr_hi_pipe));
	ym_sr_bit sr83(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_addr_b0_lat), .sr_out(addr_b0_dly1));
	ym_sr_bit sr84(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(addr_b0_dly1), .sr_out(addr_b0_dly2));
	
	assign vram_wr_lo_cond = ~(addr_b0_dly2 & vram_byte_swap);
	
	assign vram_wr_hi_cond = ~(~addr_b0_dly2 & vram_byte_swap);
	
	ym_sr_bit sr85(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(addr_b0_dly2), .sr_out(addr_b0_dly3));
	ym_sr_bit sr86(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_wr_path), .sr_out(dma_wr_dly1));
	ym_sr_bit sr87(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_wr_dly1), .sr_out(dma_wr_dly2));
	
	assign vram_wr_hi_strb = dma_wr_dly2 & vram_wr_hi_cond;
	
	assign vram_wr_lo_strb = dma_wr_dly2 & vram_wr_lo_cond;
	
	ym_sr_bit sr88(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_hi_strb), .sr_out(vram_wr_hi_sel));
	ym_sr_bit sr89(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_lo_strb), .sr_out(vram_wr_lo_sel));
	
	assign vram_wr_hi_clk = hclk1 & vram_wr_hi_sel;
	
	assign vram_wr_lo_clk = hclk1 & vram_wr_lo_sel;
	
	assign vram128k_rd_sel = vram_128k & ~cpu_sel & addr_b0_dly3;
	
	assign vram_byte_swap = ~vram_128k & cpu_sel & vram_direct;
	
	ym_slatch #(.DATA_WIDTH(8)) sl90(.MCLK(MCLK), .en(vcnt_latch_en), .inp({ vcnt_ext[7:1], vcnt_bit_sel}), .val(vcnt_latch)); // v counter
	
	ym_slatch #(.DATA_WIDTH(8)) sl91(.MCLK(MCLK), .en(hcnt_latch_en), .inp(hcnt[8:1]), .val(hcnt_latch)); // h counter
	
	assign hv_cnt_data = hv_cnt_sel ? hcnt_latch : vcnt_latch;
	
	assign vram_rd_lo_data = vram128k_rd_sel ? vram_data[15:8] : vram_data[7:0];
	
	ym_slatch #(.DATA_WIDTH(8)) sl92(.MCLK(MCLK), .en(vram_wr_lo_clk), .inp(vram_rd_lo_data), .val(vram_rd_lo_lat));
	
	assign vram_rd_hi_data = vram_byte_swap ? vram_data[7:0] : vram_data[15:8];
	
	ym_slatch #(.DATA_WIDTH(8)) sl93(.MCLK(MCLK), .en(vram_wr_hi_clk), .inp(vram_rd_hi_data), .val(vram_rd_hi_lat));
	
	assign cpu_data_hi = cpu_sel ? io_data[15:8] : io_data[7:0];
	
	ym_slatch #(.DATA_WIDTH(8)) sl94(.MCLK(MCLK), .en(fifo_data_wr_en), .inp(cpu_data_hi), .val(fifo_in_hi));
	ym_slatch #(.DATA_WIDTH(8)) sl95(.MCLK(MCLK), .en(fifo_data_wr_en), .inp(io_data[7:0]), .val(fifo_in_lo));
	
	ym_slatch #(.DATA_WIDTH(8)) sl96(.MCLK(MCLK), .en(fifo_en_s1), .inp(fifo_in_hi), .val(fifo_hi_s1));
	ym_slatch #(.DATA_WIDTH(8)) sl97(.MCLK(MCLK), .en(fifo_en_s1), .inp(fifo_in_lo), .val(fifo_lo_s1));
	
	assign fifo_out_s1 = fifo_byte_sel ? fifo_hi_s1 : fifo_lo_s1;
	
	assign unk_data =
		(data_rd_s1 ? fifo_lo_s1 : 8'h0) |
		(data_rd_s2 ? fifo_lo_s2 : 8'h0) |
		(data_rd_s3 ? fifo_lo_s3 : 8'h0) |
		(data_rd_s0 ? fifo_lo_s0 : 8'h0);
	
	ym_slatch #(.DATA_WIDTH(8)) sl98(.MCLK(MCLK), .en(fifo_en_s2), .inp(fifo_in_hi), .val(fifo_hi_s2));
	ym_slatch #(.DATA_WIDTH(8)) sl99(.MCLK(MCLK), .en(fifo_en_s2), .inp(fifo_in_lo), .val(fifo_lo_s2));
	
	assign fifo_out_s2 = fifo_byte_sel ? fifo_hi_s2 : fifo_lo_s2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl100(.MCLK(MCLK), .en(fifo_en_s3), .inp(fifo_in_hi), .val(fifo_hi_s3));
	ym_slatch #(.DATA_WIDTH(8)) sl101(.MCLK(MCLK), .en(fifo_en_s3), .inp(fifo_in_lo), .val(fifo_lo_s3));
	
	assign fifo_out_s3 = fifo_byte_sel ? fifo_hi_s3 : fifo_lo_s3;
	
	ym_slatch #(.DATA_WIDTH(8)) sl102(.MCLK(MCLK), .en(fifo_en_s0), .inp(fifo_in_hi), .val(fifo_hi_s0));
	ym_slatch #(.DATA_WIDTH(8)) sl103(.MCLK(MCLK), .en(fifo_en_s0), .inp(fifo_in_lo), .val(fifo_lo_s0));
	
	assign fifo_out_s0 = fifo_byte_sel ? fifo_hi_s0 : fifo_lo_s0;
	
	ym_sr_bit_array #(.DATA_WIDTH(8)) sr104(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(unk_data), .data_out(data_rd_pipe));
	
	ym_slatch_r #(.DATA_WIDTH(8)) sl_hit(.MCLK(MCLK), .en(reg_wr_8D), .rst(reset_comb), .inp(~reg_data_l2[7:0]), .val(reg_hit));
	
	ym_slatch sl_lsm0_latch(.MCLK(MCLK), .en(vdisp_ended), .inp(reg_lsm0), .val(reg_lsm0_latch));
	ym_slatch sl_lsm1_latch(.MCLK(MCLK), .en(vdisp_ended), .inp(reg_lsm1), .val(reg_lsm1_latch));
	
	ym_slatch_r #(.DATA_WIDTH(12)) sl_test_18(.MCLK(MCLK), .en(tst_reg_wr), .rst(reset_ext), .inp(io_data[11:0]), .val(reg_test_18));
	
	ym_slatch_r #(.DATA_WIDTH(15)) sl_test0(.MCLK(MCLK), .en(tst_fn0_wr), .rst(reset_ext), .inp(io_data[14:0]), .val(reg_test0));
	
	ym_slatch_r #(.DATA_WIDTH(11)) sl_test1(.MCLK(MCLK), .en(tst_fn1_wr), .rst(reset_ext), .inp(io_data[10:0]), .val(reg_test1));
	
	ym_slatch #(.DATA_WIDTH(2)) sl_code_01(.MCLK(MCLK), .en(ctrl_wr_dtack), .inp(cpu_data_hi[7:6]), .val(reg_code[1:0]));
	ym_slatch_r #(.DATA_WIDTH(3)) sl_code_234(.MCLK(MCLK), .en(dma_auto_inc), .rst(code_hi_rst), .inp(io_data[6:4]), .val(reg_code[4:2]));
	
	ym_slatch #(.DATA_WIDTH(8)) sl_addr_1(.MCLK(MCLK), .en(data_wr_first), .inp(io_data[7:0]), .val(reg_addr[7:0]));
	ym_slatch #(.DATA_WIDTH(6)) sl_addr_2(.MCLK(MCLK), .en(ctrl_wr_dtack), .inp(cpu_data_hi[5:0]), .val(reg_addr[13:8]));
	ym_slatch_r #(.DATA_WIDTH(3)) sl_addr_3(.MCLK(MCLK), .en(dma_auto_inc), .rst(code_hi_rst), .inp(io_data[2:0]), .val(reg_addr[16:14]));
	
	wire [16:0] reg_data_sum = reg_data_l2 + { 9'h0, reg_inc } + { 16'h0, ~reg_m5 };
	wire [16:0] reg_data_mux = reg_addr_load ? reg_addr : reg_data_sum;
	
	ym7101_dff #(.DATA_WIDTH(14)) reg_data_1(.MCLK(MCLK), .clk(~reg_data_wr_en), .inp(reg_data_mux[13:0]),
		.rst(reset_comb), .outp(reg_data_l2[13:0]));
	
	ym7101_dff #(.DATA_WIDTH(3)) reg_data_2(.MCLK(MCLK), .clk(~reg_data_wr_en), .inp(reg_data_mux[16:14]),
		.rst(code_hi_rst), .outp(reg_data_l2[16:14]));
	
	ym_slatch sl_80_b0(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[0]), .val(reg_80_b0));
	ym_slatch sl_m3(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[1]), .val(reg_m3));
	ym_slatch sl_80_b2(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[2]), .val(reg_80_b2));
	ym_slatch sl_80_b3(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[3]), .val(reg_80_b3));
	ym_slatch sl_ie1(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[4]), .val(reg_ie1));
	ym_slatch sl_lcb(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[5]), .val(reg_lcb));
	ym_slatch sl_80_b6(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[6]), .val(reg_80_b6));
	ym_slatch sl_80_b7(.MCLK(MCLK), .en(reg_wr_80), .inp(reg_data_l2[7]), .val(reg_80_b7));
	
	
	ym_slatch sl_rs1(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[0]), .val(reg_rs1));
	ym_slatch sl_lsm0(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[1]), .val(reg_lsm0));
	ym_slatch sl_lsm1(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[2]), .val(reg_lsm1));
	ym_slatch sl_ste(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[3]), .val(reg_ste));
	ym_slatch sl_8c_b4(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[4]), .val(reg_8c_b4));
	ym_slatch sl_8c_b5(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[5]), .val(reg_8c_b5));
	ym_slatch sl_8c_b6(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[6]), .val(reg_8c_b6));
	ym_slatch sl_rs0(.MCLK(MCLK), .en(wr_en_8C), .inp(reg_data_l2[7]), .val(reg_rs0));
	
	ym_slatch sl_81_b0(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[0]), .val(reg_81_b0));
	ym_slatch sl_81_b1(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[1]), .val(reg_81_b1));
	ym_slatch sl_m5(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[2]), .val(reg_m5));
	ym_slatch sl_m2(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[3]), .val(reg_m2));
	ym_slatch sl_m1(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[4]), .val(reg_m1));
	ym_slatch sl_ie0(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[5]), .val(reg_ie0));
	ym_slatch sl_disp(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[6]), .val(reg_disp));
	ym_slatch sl_81_b7(.MCLK(MCLK), .en(reg_wr_81), .inp(reg_data_l2[7]), .val(reg_81_b7));
	
	
	ym_slatch sl_lscr(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[0]), .val(reg_lscr));
	ym_slatch sl_hscr(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[1]), .val(reg_hscr));
	ym_slatch sl_vscr(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[2]), .val(reg_vscr));
	ym_slatch sl_ie2(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[3]), .val(reg_ie2));
	ym_slatch sl_8b_b4(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[4]), .val(reg_8b_b4));
	ym_slatch sl_8b_b5(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[5]), .val(reg_8b_b5));
	ym_slatch sl_8b_b6(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[6]), .val(reg_8b_b6));
	ym_slatch sl_8b_b7(.MCLK(MCLK), .en(wr_en_8B_scr), .inp(reg_data_l2[7]), .val(reg_8b_b7));
	
	ym_slatch #(.DATA_WIDTH(8)) sl_inc(.MCLK(MCLK), .en(wr_en_8F), .inp(reg_data_l2[7:0]), .val(reg_inc));
	
	ym_slatch #(.DATA_WIDTH(6)) sl_sa_high(.MCLK(MCLK), .en(reg_wr_8B), .inp(reg_data_l2[5:0]), .val(reg_sa_high));
	
	ym_slatch #(.DATA_WIDTH(2)) sl_dmd(.MCLK(MCLK), .en(reg_wr_8B), .inp(reg_data_l2[7:6]), .val(reg_dmd));
	
	ym_cnt_bit_load #(.DATA_WIDTH(8)) cnt_lg_1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(dma_cnt_norm), .reset(1'h0), .load(wr_en_93), .load_val(~reg_data_l2[7:0]), .c_out(reg_lg_of), .val(reg_lg[7:0]));
		
	ym_cnt_bit_load #(.DATA_WIDTH(8)) cnt_lg_2(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(dma_len_hi_cry), .reset(1'h0), .load(wr_en_94), .load_val(~reg_data_l2[7:0]), .val(reg_lg[15:8]));
	
	ym_cnt_bit_load #(.DATA_WIDTH(8)) cnt_sa_low_1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(dma_cnt_norm), .reset(1'h0), .load(reg_wr_8C), .load_val(reg_data_l2[7:0]), .c_out(reg_sa_of), .val(reg_sa_low[7:0]));
		
	ym_cnt_bit_load #(.DATA_WIDTH(8)) cnt_sa_low_2(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(dma_src_hi_cry), .reset(1'h0), .load(wr_en_96), .load_val(reg_data_l2[7:0]), .val(reg_sa_low[15:8]));
	
	assign IPL1_pull = ~io_ipl1;
	assign IPL2_pull = ~io_ipl2;
	assign UWR = ~io_uwr;
	assign LWR = ~io_lwr;
	assign OE0 = ~io_oe0;
	assign CAS0 = ~io_cas0;
	assign RAS0 = ~io_ras0;
	assign BR_pull = ~br_pull_ctl;
	assign BGACK_pull = ~bgack_pull_ctl;
	assign DTACK_pull = ~dtack_pull_ctl;
	assign RA = ra_addr_mux[7:0];
	assign INT_pull = ~int_pull_ctl;
	
	// -------------------------------------------------------------------------
	// Timing FSM
	// -------------------------------------------------------------------------
	// H/V counter state machine. hcnt is the 9-bit horizontal counter,
	// vcnt is the 9-bit vertical counter. Together with the PLA comparators
	// (pla_vcnt, pla_hcnt1, pla_hcnt2), they generate sync pulses, blanking
	// intervals, display enable, and all per-line/per-frame timing events.

	ym_cnt_bit_load #(.DATA_WIDTH(9)) cnt105(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(vcnt_inc_en), .reset(1'h0), .load(vcnt_load_en), .load_val(vcnt_load_val), .val(vcnt));
	
	assign vcnt_ext = interlace_dblres ? { vcnt, field_bit } : { 1'h0, vcnt };
	
	ym_cnt_bit_load #(.DATA_WIDTH(9)) cnt106(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(hcnt_inc_en), .reset(1'h0), .load(hcnt_load_en), .load_val(hcnt_load_val), .val(hcnt));
	
	ym_sr_bit #(.SR_LENGTH(8)) sr107(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(field_trig_pipe), .sr_out(field_trig_dly8));
	
	ym_sr_bit sr108(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_edclk), .sr_out(edclk_dly));
	
	ym_sr_bit sr109(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(slot_idle), .sr_out(slot_idle_dly));
	
	ym_sr_bit sr110(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_line_zero), .sr_out(line_zero_dly));
	
	assign slot3_active = tst_mode_1 | (slot3_dly & tst_normal);
	
	assign tst_mode_1 = reg_test1[6:4] == 3'h1;
	
	assign no_slot_123e = ~(hcnt_edclk | hcnt_slot3 | hcnt_slot2 | hcnt_slot1);
	
	assign slot_idle = no_slot_123e & no_slot0_odd_ext & no_disp_main;
	
	assign disp_start = ~not_disp_start_dly;
	
	ym_sr_bit sr111(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_hint_time), .sr_out(hint_time_dly));
	
	ym_sr_bit sr112(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_spr_end), .sr_out(spr_end_dly));
	
	assign hcnt_load_en = spr_end_dly | tst_fn3_wr | reset_comb | hcnt_m5_reload;
	
	assign hcnt_inc_gate = ~(hcnt_load_en | reg_test1[3]);
	
	assign hcnt_inc_en = hcnt_inc_gate | (reg_test1[3] & ~cpu_intak);
	
	assign hcnt_load_val = tst_fn3_wr ? io_data[8:0] : { 4'he, ~hcnt_rld_b0, hcnt_rld_b3, hcnt_rld_b2, hcnt_rld_b1, hcnt_rld_b0 };
	
	assign hcnt_rld_b0 = ~reg_80_b0 & reg_rs1 & reg_m5;

	assign hcnt_rld_b1 = ~reg_80_b0 & ~reg_rs1;
	
	assign hcnt_rld_b2 = reg_80_b0 & ~reg_rs1;
	
	assign hcnt_rld_b3 = hcnt_rld_b0 | hcnt_rld_b3_aux;
	
	assign hcnt_rld_b3_aux = ~reg_rs1 & reg_80_b0 & reg_m5;
	// h40: 457
	// h32: 466
	// m4: 466
	
	assign hcnt_m5_reload = ~line_run_prev & line_running & reg_80_b0;
	
	ym_sr_bit sr113(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(line_running), .sr_out(line_run_prev));
	
	ym_sr_bit sr114(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_spr_mid), .sr_out(spr_mid_dly));
	
	wire active_end_t;
	assign active_end = ~active_end_t;
	ym_sr_bit sr115(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_not_active_end), .sr_out(active_end_t));
	
	wire odd_slot_t;
	assign odd_slot = ~odd_slot_t;
	ym_sr_bit sr116(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(odd_slot_gate), .sr_out(odd_slot_t));
	
	assign tst_mode_2 = reg_test1[6:4] == 3'h2;
	
	assign slot2_active = tst_mode_2 | (slot2_dly & tst_normal);
	
	ym_sr_bit sr117(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_not_disp_start), .sr_out(not_disp_start_dly));
	
	ym_sr_bit sr118(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_slot3), .sr_out(slot3_dly));
	
	ym_sr_bit sr119(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_slot2), .sr_out(slot2_dly));
	
	assign field_inv = ~(reg_m5 ? field_trig_dly8 : field_trig_pipe);
	
	assign csync_src = reg_8c_b6 ? hclk2 : field_inv;
	
	ym_sr_bit sr120(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(i_csync), .sr_out(csync_dly));
	
	assign csync_combined = csync_dly | hsync_ext_in;
	
	assign line_run_next = line_not_reset & (line_running | csync_combined);
	
	assign line_not_reset = ~(spr_mid_dly | reset_comb);
	
	ym_sr_bit sr121(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(line_run_next), .sr_out(line_running));
	
	ym_sr_bit sr122(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_adj_point), .sr_out(adj_point_dly));
	
	ym_sr_bit sr123(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_render_start), .sr_out(render_start_dly));
	
	assign vscr_or_disp = reg_vscr ? px8_bound_dly : ~not_disp_start_dly;
	
	assign vscr_active = vscr_or_disp & tst_normal;
	
	assign tst_normal = reg_test1[6:4] == 3'h0;
	
	assign tst_mode_5 = reg_test1[6:4] == 3'h5;
	
	assign tst_mode_3 = reg_test1[6:4] == 3'h3;
	
	assign tst_mode_4 = reg_test1[6:4] == 3'h4;
	
	assign odd_slot_gate = ~(hcnt_odd & ~reload_pulse_dly);
	
	assign slot1_active = tst_mode_3 | (tst_normal & slot1_dly);
	
	assign no_slot0_odd_ext = ~(hcnt_slot0 | hcnt_odd | hcnt_access_ext);
	
	ym_sr_bit sr124(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_8px_bound), .sr_out(px8_bound_dly));
	
	ym_sr_bit sr125(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_slot1), .sr_out(slot1_dly));
	
	ym_sr_bit sr126(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_slot0), .sr_out(slot0_dly));
	
	assign csync_m5_mux = reg_m5 ? csync_xor_dly8 : csync_xor_field;
	
	ym_sr_bit #(.SR_LENGTH(8)) sr127(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_xor_field), .sr_out(csync_xor_dly8));
	
	assign hv_disp_active = hdisp_en_trig & ~active_disp_gate;
	
	ym_sr_bit sr128(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_pull_gate), .sr_out(csync_out_dly));
	
	assign full_disp_en = reg_disp & hdisp_en_trig & vdisp_en_trig;
	
	ym_sr_bit sr129(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vint_pending), .sr_out(vint_latch_dly));
	
	assign m5_vblank_or_vsync = m5_vblank_hi | m5_vsync;
	
	assign m5_field_hblank = hblank_state & m5_field_active;
	
	ym_sr_bit sr130(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_sync_area), .sr_out(sync_area_dly));
	
	assign hdisp_rst = reset_comb | scroll_end_dly;
	
	ym7101_rs_trig rs29(.MCLK(MCLK), .set(sync_area_dly), .rst(hdisp_rst), .q(hdisp_en_trig));
	
	ym_sr_bit sr131(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_hblank_end), .sr_out(hblank_end_dly));
	
	assign cell_m4_active = cell_m4_dly & tst_normal;
	
	ym_sr_bit sr132(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_reload_pulse), .sr_out(reload_pulse_dly));
	
	assign slot0_active = tst_mode_4 | (tst_normal & slot0_dly);
	
	assign no_disp_main = ~(hcnt_disp_active | hcnt_m4_border | hcnt_access_main | hcnt_blank_start);
	
	ym_sr_bit sr133(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_cell_m4), .sr_out(cell_m4_dly));
	
	ym_sr_bit sr134(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_access_ext), .sr_out(access_ext_dly));
	
	assign csync_ext_mux = reg_m5 ? csync_sel_dly7 : csync_div_sel;
	
	ym_sr_bit #(.SR_LENGTH(7)) sr135(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_div_sel), .sr_out(csync_sel_dly7));
	
	ym_sr_bit sr136(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hsync_pull_gate), .sr_out(hsync_out_dly));
	
	assign csync_pull_gate = ~(~reg_80_b0 & csync_m5_mux);
	
	assign no_m5_vert_event = ~(m5_vblank_hi | m5_field_active | m5_vsync);
	
	assign vert_fetch_gate = ~no_m5_vert_event & between_fetches;
	
	assign vblank_vsync_fetch = m5_vblank_or_vsync & between_fetches;
	
	ym7101_rs_trig rs30(.MCLK(MCLK), .set(vint_set), .rst(line_first_dly), .q(vint_pending));
	
	assign vint_set = reset_comb | scroll_start_dly;
	
	ym_sr_bit sr137(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_scroll_end), .sr_out(scroll_end_dly));
	
	ym_sr_bit sr138(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_access_win), .sr_out(access_win_dly));
	
	assign cell_bound_active = tst_normal & cell_bound_dly;
	
	ym_sr_bit sr139(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_reload_edge), .sr_out(reload_edge_dly));
	
	assign hcnt_reload_pulse = reload_edge_dly | hcnt_reload_edge;
	
	ym_sr_bit sr140(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_cell_bound), .sr_out(cell_bound_dly));
	
	ym_sr_bit sr141(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_access_main), .sr_out(access_main_dly));
	
	ym_sr_bit sr142(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_m4_border), .sr_out(m4_border_dly));
	
	assign hsync_pull_gate = ~(csync_ext_mux & ~reg_8c_b5);
	
	assign hsync_no_vert_a = no_m5_vert_event & hsync_trig;
	
	assign hsync_no_vert_b = no_m5_vert_event & hsync_trig;
	
	ym7101_rs_trig rs31(.MCLK(MCLK), .set(hblank_set), .rst(hblank_rst), .q(hblank_state));
	
	assign hblank_set = scroll_end_dly | hint_pos_dly;
	
	assign hblank_rst = reset_comb | hblank_end_comb;
	
	assign hblank_end_comb = hblank_end_dly | fetch_point_dly;
	
	ym_sr_bit sr143(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_scroll_start), .sr_out(scroll_start_dly));
	
	ym_sr_bit sr144(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_fetch_point), .sr_out(fetch_point_dly));
	
	assign hcnt_reload_edge = hcnt_load_en & h40_mode;
	
	ym_sr_bit sr145(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_vram_slot), .sr_out(vram_slot_dly));
	
	ym_sr_bit sr146(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_m4_border), .sr_out(m4_border2_dly));
	
	ym_sr_bit sr147(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_fetch_all), .sr_out(fetch_all_dly));
	
	ym_sr_bit sr148(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_blank_start), .sr_out(blank_start_dly));
	
	assign hsync_ext_in = reg_8c_b5 & hsync_in_dly;
	
	ym_sr_bit sr149(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(i_hsync), .sr_out(hsync_in_dly));
	
	ym_sr_bit sr150(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_comb_a), .sr_out(csync_comb_a_dly));
	
	assign csync_comb_a = hsync_no_vert_a | vert_fetch_gate;
	
	assign csync_comb_b = hsync_no_vert_b | vblank_vsync_fetch | m5_field_hblank;
	
	ym_sr_bit sr151(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_comb_b), .sr_out(csync_comb_b_dly));
	
	ym7101_rs_trig rs32(.MCLK(MCLK), .set(fetch_phase_rst), .rst(access_win_dly), .q(between_fetches));
	
	assign fetch_phase_rst = hblank_end_comb | reset_comb;
	
	ym_sr_bit sr152(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_line_first), .sr_out(line_first_dly));
	
	ym_sr_bit sr153(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_hint_pos), .sr_out(hint_pos_dly));
	
	assign pre_wrap_active = tst_normal & pre_wrap_dly;
	
	assign h40_mode = ~reg_8c_b4 & reg_80_b0;
	
	assign m4_or_vram_slot = m4_border2_dly | vram_slot_dly;
	
	assign vram_or_ext_slot = vram_slot_dly | ext_access_dly;
	
	assign blank_slot_active = tst_mode_5 | (tst_normal & blank_start_dly);
	
	ym_sr_bit sr154(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_pre_wrap), .sr_out(pre_wrap_dly));
	
	ym_sr_bit sr155(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_ext_access), .sr_out(ext_access_dly));
	
	ym_sr_bit sr156(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt_m4_window), .sr_out(m4_window_dly));
	
	assign csync_xor_field = csync_comb_b_dly ^ field_trig_pipe;
	
	assign csync_div_sel = csync_prog_mode ? csync_div_bit : csync_comb_a_dly2;
	
	ym_sr_bit sr157(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_comb_a_dly), .sr_out(csync_comb_a_dly2));
	
	assign vint_delayed = reg_m5 ? vint_pend_dly8 : vint_latch_dly;
	
	ym_sr_bit #(.SR_LENGTH(8)) sr158(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vint_latch_dly), .sr_out(vint_pend_dly8));
	
	ym7101_rs_trig rs33(.MCLK(MCLK), .set(hsync_trig_set), .rst(render_start_dly), .q(hsync_trig));
	
	assign hsync_trig_set = reset_comb | hblank_end_dly;
	
	assign sub_slot_active = sub_slot_dly & tst_normal;
	
	assign h40_no_ext_latch = h40_mode & ~reg_81_b0;
	
	ym_sr_bit sr159(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt2_sub_slot), .sr_out(sub_slot_dly));
	
	ym_cnt_bit cnt160(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(csync_div_inc), .reset(reset_comb), .val(csync_div_bit));
	
	assign csync_div_inc = csync_xor_field & ~csync_xor_prev;
	
	ym_sr_bit sr161(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(csync_xor_field), .sr_out(csync_xor_prev));
	
	assign csync_prog_mode = reg_m5 & reg_80_b3;
	
	assign vcnt_load_val = tst_fn2_wr ? io_data[8:0] :
		{ 2'h3, ~pal_m4, vcnt_rld_b5, ~v28_mode, vcnt_rld_b3, ~cpu_pal, pal_even_field, field_xor_region };
	
	assign vcnt_rld_b5 = ntsc_v28 | pal_m4;
	
	assign ntsc_v28 = ~cpu_pal & v28_mode;
	
	assign pal_m4 = cpu_pal & ~reg_m5;
	
	assign pal_v28 = cpu_pal & v28_mode;
	
	assign vcnt_rld_b3 = pal_v28 | pal_m4;
	
	assign pal_even_field = cpu_pal & ~field_bit;
	
	assign field_xor_region = (~cpu_pal) ^ field_bit;
	
	assign vcnt_inc_en = (~reg_test1[2] & active_end & ~vcnt_load_en) | (reg_test1[2] & ~cpu_bg);
	
	assign vcnt_load_en = vcnt_wrap_cond | reset_comb | tst_fn2_wr | vcnt_adj_reload;
	
	assign vcnt_wrap_cond = active_end & vcnt_eq_dly;
	
	ym_sr_bit sr162(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~vcnt_not_max), .sr_out(vcnt_at_max));
	
	assign active_disp_gate = ~(reg_disp & (vcnt_at_max | vdisp_en_trig));
	
	ym_sr_bit sr163(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_frame_end), .sr_out(frame_end_dly));
	
	assign vblank_hi_rst = reset_comb | frame_end_gated;
	
	ym7101_rs_trig rs34(.MCLK(MCLK), .set(vblank_hi_gated), .rst(vblank_hi_rst), .q(vblank_hi_latch));
	
	assign m5_vblank_hi = vblank_hi_latch & reg_m5;
	
	ym_sr_bit sr164(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_vblank_hi), .sr_out(vblank_hi_dly));
	
	assign frame_end_gated = frame_end_dly & hint_or_even;
	
	ym7101_rs_trig rs35(.MCLK(MCLK), .set(field_trig_rst), .rst(vblank_hi_gated), .q(field_bit_trig));
	
	assign m5_field_active = field_bit_trig & reg_m5;
	
	ym_sr_bit sr165(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_vblank_lo), .sr_out(vblank_lo_dly));
	
	assign vblank_hi_gated = vblank_hi_dly & hint_or_even;
	
	assign field_trig_rst = reset_comb | vblank_lo_gated;
	
	ym7101_rs_trig rs36(.MCLK(MCLK), .set(field_toggle_gated), .rst(vsync_latch_rst), .q(vsync_latch));
	
	ym_cnt_bit_rs cnt166(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .c_in(field_adj_h32), .reset(field_bit_rst), .set(field_set_dly), .val(field_bit));
	
	ym_sr_bit sr167(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_vsync_area), .sr_out(vsync_area_dly));
	
	assign vblank_lo_gated = vblank_lo_dly & hint_or_even;
	
	assign vsync_latch_rst = reset_comb | vblank_lo_gated;
	
	assign hint_or_even = hint_time_dly | ~field_bit;
	
	ym_sr_bit sr168(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(field_set_cond), .sr_out(field_set_dly));
	
	assign m5_vsync = reg_m5 & vsync_latch;
	
	assign field_bit_rst = reset_comb | ~reg_lsm0 | (field_adj_h40 & ~field_adj_en);
	
	ym_sr_bit sr169(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_field_toggle), .sr_out(field_toggle_dly));
	
	assign field_toggle_gated = field_toggle_dly & hint_or_even;
	
	assign vsync_gate_set = field_toggle_gated | reset_comb;
	
	ym7101_rs_trig rs37(.MCLK(MCLK), .set(vsync_gate_set), .rst(vsync_area_dly), .q(vsync_gate));
	
	assign field_adj_h40 = reg_80_b0 & vdisp_end_at_zero;
	
	assign field_adj_h32 = ~reg_80_b0 & vdisp_end_at_zero;
	
	assign field_set_cond = field_adj_h40 & field_adj_en;
	
	ym_sr_bit sr170(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_vdisp_active), .sr_out(vdisp_active_dly));
	
	assign vdisp_ended = ~vdisp_active_dly;
	
	assign vdisp_en_rst = reset_comb | vdisp_ended;
	
	assign vdisp_end_at_zero = ~vdisp_active_dly & line_zero_dly;
	
	ym7101_rs_trig rs38(.MCLK(MCLK), .set(vcnt_at_zero), .rst(vdisp_en_rst), .q(vdisp_en_trig));
	
	ym_sr_bit sr171(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_mid_dly), .sr_out(spr_mid_dly2));
	
	ym_sr_bit sr172(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~vcnt_not_zero), .sr_out(vcnt_at_zero));
	
	ym7101_rs_trig rs39(.MCLK(MCLK), .set(field_adj_set), .rst(field_adj_rst), .q(field_adj_en));
	
	assign vcnt_adj_reload = reg_80_b0 & ~vsync_pipe_prev & vsync_pipe;
	
	assign field_adj_rst = reset_comb | (vcnt_adj_reload & spr_mid_dly2);
	
	ym_sr_bit sr173(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(adj_point_dly), .sr_out(adj_point_dly2));
	
	ym_sr_bit sr174(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vcnt_eq_pulse), .sr_out(vcnt_eq_dly));
	
	assign not_vsync_rst = ~(vsync_area_dly | reset_comb);
	
	assign csync_adj_comb = csync_dly & (adj_point_dly | spr_mid_dly);
	
	assign vsync_pipe_next = not_vsync_rst & (vsync_pipe | csync_adj_comb);
	
	ym_sr_bit sr175(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsync_pipe_next), .sr_out(vsync_pipe));
	
	ym_sr_bit sr176(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsync_pipe), .sr_out(vsync_pipe_prev));
	
	assign field_adj_set = vcnt_adj_reload & adj_point_dly2;
	
	assign disp_active_flag = ~active_disp_gate;

	// -------------------------------------------------------------------------
	// H/V counter PLAs
	// -------------------------------------------------------------------------
	// Combinational comparators against H/V counter values. Each PLA entry
	// fires for a specific counter value in a specific display mode (PAL/NTSC,
	// H32/H40, V28/V30, interlace). These drive all per-scanline and
	// per-frame timing events (sync, blanking, VRAM access slots, etc.).
	//   pla_vcnt[47:0]  — 48 vertical counter events
	//   pla_hcnt1[62:0] — 63 horizontal counter events (group 1)
	//   pla_hcnt2[45:0] — 46 horizontal counter events (group 2)

	assign pla_vcnt[0] = vcnt == 9'd511;
	assign pla_vcnt[1] = field_bit & cpu_pal & v30_mode & vcnt == 9'd471;
	assign pla_vcnt[2] = field_bit & cpu_pal & v28_mode & vcnt == 9'd463;
	assign pla_vcnt[3] = field_bit & ~cpu_pal & v28_mode & vcnt == 9'd490;
	assign pla_vcnt[4] = ~field_bit & cpu_pal & v30_mode & vcnt == 9'd472;
	assign pla_vcnt[5] = ~field_bit & cpu_pal & v28_mode & vcnt == 9'd464;
	assign pla_vcnt[6] = ~field_bit & cpu_pal & ~reg_m5 & vcnt == 9'd448;
	assign pla_vcnt[7] = ~field_bit & ~cpu_pal & v28_mode & vcnt == 9'd491;
	assign pla_vcnt[8] = ~field_bit & ~cpu_pal & ~reg_m5 & vcnt == 9'd475;
	assign pla_vcnt[9] = field_bit & cpu_pal & v30_mode & vcnt == 9'd468;
	assign pla_vcnt[10] = field_bit & cpu_pal & v28_mode & vcnt == 9'd460;
	assign pla_vcnt[11] = field_bit & ~cpu_pal & v28_mode & vcnt == 9'd487;
	assign pla_vcnt[12] = ~field_bit & cpu_pal & v30_mode & vcnt == 9'd469;
	assign pla_vcnt[13] = ~field_bit & cpu_pal & v28_mode & vcnt == 9'd461;
	assign pla_vcnt[14] = ~field_bit & cpu_pal & ~reg_m5 & vcnt == 9'd445;
	assign pla_vcnt[15] = ~field_bit & ~cpu_pal & v28_mode & vcnt == 9'd488;
	assign pla_vcnt[16] = ~field_bit & ~cpu_pal & ~reg_m5 & vcnt == 9'd472;
	assign pla_vcnt[17] = field_bit & cpu_pal & v30_mode & vcnt == 9'd465;
	assign pla_vcnt[18] = field_bit & cpu_pal & v28_mode & vcnt == 9'd457;
	assign pla_vcnt[19] = field_bit & ~cpu_pal & v28_mode & vcnt == 9'd484;
	assign pla_vcnt[20] = ~field_bit & cpu_pal & v30_mode & vcnt == 9'd466;
	assign pla_vcnt[21] = ~field_bit & cpu_pal & v28_mode & vcnt == 9'd458;
	assign pla_vcnt[22] = ~field_bit & cpu_pal & ~reg_m5 & vcnt == 9'd442;
	assign pla_vcnt[23] = ~field_bit & ~cpu_pal & v28_mode & vcnt == 9'd485;
	assign pla_vcnt[24] = ~field_bit & ~cpu_pal & ~reg_m5 & vcnt == 9'd469;
	assign pla_vcnt[25] = cpu_pal & v30_mode & vcnt == 9'd482;
	assign pla_vcnt[26] = cpu_pal & v28_mode & vcnt == 9'd474;
	assign pla_vcnt[27] = cpu_pal & ~reg_m5 & vcnt == 9'd458;
	assign pla_vcnt[28] = ~cpu_pal & v28_mode & vcnt == 9'd501;
	assign pla_vcnt[29] = ~cpu_pal & ~reg_m5 & vcnt == 9'd485;
	assign pla_vcnt[30] = reg_lsm0 & cpu_pal & v30_mode & vcnt == 9'd263;
	assign pla_vcnt[31] = reg_lsm0 & cpu_pal & v28_mode & vcnt == 9'd255;
	assign pla_vcnt[32] = ~reg_lsm0 & cpu_pal & v30_mode & vcnt == 9'd264;
	assign pla_vcnt[33] = ~reg_lsm0 & cpu_pal & v28_mode & vcnt == 9'd256;
	assign pla_vcnt[34] = ~reg_lsm0 & cpu_pal & ~reg_m5 & vcnt == 9'd240;
	assign pla_vcnt[35] = ~cpu_pal & v28_mode & vcnt == 9'd232;
	assign pla_vcnt[36] = ~cpu_pal & ~reg_m5 & vcnt == 9'd216;
	assign pla_vcnt[37] = v30_mode & vcnt == 9'd240;
	assign pla_vcnt[38] = v28_mode & vcnt == 9'd224;
	assign pla_vcnt[39] = ~reg_m5 & vcnt == 9'd192;
	assign pla_vcnt[40] = vcnt == 9'd0;
	assign pla_vcnt[41] = reg_lsm0 & cpu_pal & v30_mode & vcnt == 9'd265;
	assign pla_vcnt[42] = reg_lsm0 & cpu_pal & v28_mode & vcnt == 9'd257;
	assign pla_vcnt[43] = ~reg_lsm0 & cpu_pal & v30_mode & vcnt == 9'd266;
	assign pla_vcnt[44] = ~reg_lsm0 & cpu_pal & v28_mode & vcnt == 9'd258;
	assign pla_vcnt[45] = ~reg_lsm0 & cpu_pal & ~reg_m5 & vcnt == 9'd242;
	assign pla_vcnt[46] = ~cpu_pal & v28_mode & vcnt == 9'd234;
	assign pla_vcnt[47] = ~cpu_pal & ~reg_m5 & vcnt == 9'd218;
	
	assign pla_hcnt1[0] = disp_active_flag & ~reg_m5 & hcnt == 9'd488;
	assign pla_hcnt1[1] = disp_active_flag & ~reg_m5 & hcnt == 9'd484;
	assign pla_hcnt1[2] = disp_active_flag & ~reg_m5 & (hcnt & 9'd507) == 9'd472;
	assign pla_hcnt1[3] = disp_active_flag & ~reg_m5 & (hcnt & 9'd503) == 9'd272;
	assign pla_hcnt1[4] = disp_active_flag & ~reg_m5 & (hcnt & 9'd495) == 9'd268;
	assign pla_hcnt1[5] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd463) == 9'd266;
	assign pla_hcnt1[6] = disp_active_flag & reg_m5 & (hcnt & 9'd271) == 9'd10;
	assign pla_hcnt1[7] = disp_active_flag & reg_m5 & reg_rs1 & hcnt == 9'd484;
	assign pla_hcnt1[8] = disp_active_flag & reg_m5 & reg_rs1 & hcnt == 9'd460;
	assign pla_hcnt1[9] = disp_active_flag & reg_m5 & reg_rs1 & hcnt == 9'd458;
	assign pla_hcnt1[10] = disp_active_flag & ~h40_no_ext_latch & reg_m5 & reg_rs1 & (hcnt & 9'd505) == 9'd344;
	assign pla_hcnt1[11] = disp_active_flag & ~h40_mode & reg_m5 & reg_rs1 & hcnt == 9'd364;
	assign pla_hcnt1[12] = disp_active_flag & ~h40_mode & reg_m5 & reg_rs1 & (hcnt & 9'd509) == 9'd360;
	assign pla_hcnt1[13] = disp_active_flag & ~h40_mode & reg_m5 & reg_rs1 & (hcnt & 9'd505) == 9'd352;
	assign pla_hcnt1[14] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd505) == 9'd336;
	assign pla_hcnt1[15] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd505) == 9'd328;
	assign pla_hcnt1[16] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd509) == 9'd324;
	assign pla_hcnt1[17] = disp_active_flag & ~h40_mode & reg_m5 & ~reg_rs1 & hcnt == 9'd290;
	assign pla_hcnt1[18] = disp_active_flag & ~h40_mode & reg_m5 & ~reg_rs1 & (hcnt & 9'd509) == 9'd292;
	assign pla_hcnt1[19] = disp_active_flag & ~h40_no_ext_latch & reg_m5 & ~reg_rs1 & (hcnt & 9'd505) == 9'd280;
	assign pla_hcnt1[20] = disp_active_flag & reg_m5 & ~reg_rs1 & (hcnt & 9'd505) == 9'd264;
	assign pla_hcnt1[21] = disp_active_flag & reg_m5 & ~reg_rs1 & (hcnt & 9'd509) == 9'd260;
	assign pla_hcnt1[22] = disp_active_flag & reg_m5 & ~reg_rs1 & (hcnt & 9'd505) == 9'd272;
	assign pla_hcnt1[23] = disp_active_flag & reg_m5 & hcnt == 9'd486;
	assign pla_hcnt1[24] = disp_active_flag & reg_m5 & (hcnt & 9'd503) == 9'd498;
	assign pla_hcnt1[25] = disp_active_flag & reg_m5 & (hcnt & 9'd505) == 9'd488;
	assign pla_hcnt1[26] = disp_active_flag & reg_m5 & (hcnt & 9'd509) == 9'd480;
	assign pla_hcnt1[27] = disp_active_flag & reg_m5 & (hcnt & 9'd497) == 9'd464;
	assign pla_hcnt1[28] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd488;
	assign pla_hcnt1[29] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd476;
	assign pla_hcnt1[30] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd284;
	assign pla_hcnt1[31] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd272;
	assign pla_hcnt1[32] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd484;
	assign pla_hcnt1[33] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd472;
	assign pla_hcnt1[34] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd280;
	assign pla_hcnt1[35] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd268;
	assign pla_hcnt1[36] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd480;
	assign pla_hcnt1[37] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd468;
	assign pla_hcnt1[38] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd276;
	assign pla_hcnt1[39] = disp_active_flag & ~reg_m5 & (hcnt & 9'd509) == 9'd264;
	assign pla_hcnt1[40] = disp_active_flag & ~reg_m5 & (hcnt & 9'd279) == 9'd18;
	assign pla_hcnt1[41] = disp_active_flag & ~reg_m5 & (hcnt & 9'd287) == 9'd10;
	assign pla_hcnt1[42] = disp_active_flag & ~reg_m5 & (hcnt & 9'd497) == 9'd496;
	assign pla_hcnt1[43] = ~disp_active_flag & (hcnt & 9'd259) == 9'd0;
	assign pla_hcnt1[44] = (hcnt & 9'd1) == 9'd1;
	assign pla_hcnt1[45] = disp_active_flag & reg_m5 & hcnt == 9'd510;
	assign pla_hcnt1[46] = disp_active_flag & reg_m5 & hcnt == 9'd502;
	assign pla_hcnt1[47] = disp_active_flag & reg_m5 & hcnt == 9'd508;
	assign pla_hcnt1[48] = disp_active_flag & reg_m5 & hcnt == 9'd500;
	assign pla_hcnt1[49] = disp_active_flag & reg_m5 & hcnt == 9'd504;
	assign pla_hcnt1[50] = disp_active_flag & reg_m5 & hcnt == 9'd496;
	assign pla_hcnt1[51] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd455) == 9'd262;
	assign pla_hcnt1[52] = disp_active_flag & (hcnt & 9'd263) == 9'd6;
	assign pla_hcnt1[53] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd455) == 9'd260;
	assign pla_hcnt1[54] = disp_active_flag & (hcnt & 9'd263) == 9'd4;
	assign pla_hcnt1[55] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd463) == 9'd264;
	assign pla_hcnt1[56] = disp_active_flag & (hcnt & 9'd271) == 9'd8;
	assign pla_hcnt1[57] = disp_active_flag & ~reg_m5 & (hcnt & 9'd271) == 9'd8;
	assign pla_hcnt1[58] = disp_active_flag & reg_m5 & reg_rs1 & (hcnt & 9'd463) == 9'd256;
	assign pla_hcnt1[59] = disp_active_flag & (hcnt & 9'd271) == 9'd0;
	assign pla_hcnt1[60] = ~disp_active_flag & (hcnt & 9'd63) == 9'd50;
	assign pla_hcnt1[61] = disp_active_flag & reg_m5 & reg_rs1 & hcnt == 9'd306;
	assign pla_hcnt1[62] = disp_active_flag & reg_m5 & (hcnt & 9'd319) == 9'd50;
	
	assign pla_hcnt2[0] = (hcnt & 9'd15) == 9'd3;
	assign pla_hcnt2[1] = hcnt == 9'd507;
	assign pla_hcnt2[2] = reg_m5 & reg_rs1 & (hcnt & 9'd463) == 9'd269;
	assign pla_hcnt2[3] = reg_m5 & (hcnt & 9'd271) == 9'd13;
	assign pla_hcnt2[4] = ~reg_m5 & hcnt == 9'd483;
	assign pla_hcnt2[5] = ~reg_m5 & hcnt == 9'd471;
	assign pla_hcnt2[6] = ~reg_m5 & hcnt == 9'd279;
	assign pla_hcnt2[7] = ~reg_m5 & hcnt == 9'd267;
	assign pla_hcnt2[8] = (hcnt & 9'd15) == 9'd15;
	assign pla_hcnt2[9] = ~reg_m5 & (hcnt & 9'd15) == 9'd15;
	assign pla_hcnt2[10] = (hcnt & 9'd15) == 9'd7;
	assign pla_hcnt2[11] = (hcnt & 9'd7) == 9'd0;
	assign pla_hcnt2[12] = reg_rs1 & hcnt == 9'd322;
	assign pla_hcnt2[13] = ~reg_rs1 & hcnt == 9'd258;
	assign pla_hcnt2[14] = hcnt == 9'd0;
	assign pla_hcnt2[15] = reg_rs1 & hcnt == 9'd120;
	assign pla_hcnt2[16] = ~reg_rs1 & hcnt == 9'd95;
	assign pla_hcnt2[17] = reg_m5 & reg_rs1 & hcnt == 9'd328;
	assign pla_hcnt2[18] = reg_m5 & ~reg_rs1 & hcnt == 9'd264;
	assign pla_hcnt2[19] = ~reg_m5 & hcnt == 9'd488;
	assign pla_hcnt2[20] = reg_rs1 & hcnt == 9'd482;
	assign pla_hcnt2[21] = ~reg_rs1 & hcnt == 9'd488;
	assign pla_hcnt2[22] = reg_rs1 & hcnt == 9'd358;
	assign pla_hcnt2[23] = ~reg_rs1 & hcnt == 9'd292;
	assign pla_hcnt2[24] = reg_rs1 & hcnt == 9'd164;
	assign pla_hcnt2[25] = reg_rs1 & hcnt == 9'd466;
	assign pla_hcnt2[26] = ~reg_rs1 & hcnt == 9'd134;
	assign pla_hcnt2[27] = ~reg_rs1 & hcnt == 9'd475;
	assign pla_hcnt2[28] = reg_rs1 & hcnt == 9'd148;
	assign pla_hcnt2[29] = ~reg_rs1 & hcnt == 9'd121;
	assign pla_hcnt2[30] = reg_rs1 & hcnt == 9'd120;
	assign pla_hcnt2[31] = ~reg_rs1 & hcnt == 9'd95;
	assign pla_hcnt2[32] = reg_rs1 & hcnt == 9'd1;
	assign pla_hcnt2[33] = ~reg_rs1 & hcnt == 9'd0;
	assign pla_hcnt2[34] = reg_rs1 & hcnt == 9'd348;
	assign pla_hcnt2[35] = ~reg_rs1 & hcnt == 9'd284;
	assign pla_hcnt2[36] = reg_rs1 & hcnt == 9'd330;
	assign pla_hcnt2[37] = ~reg_rs1 & hcnt == 9'd266;
	assign pla_hcnt2[38] = hcnt == 9'd18;
	assign pla_hcnt2[39] = ~reg_lcb & hcnt == 9'd10;
	assign pla_hcnt2[40] = reg_rs1 & hcnt == 9'd43;
	assign pla_hcnt2[41] = ~reg_rs1 & hcnt == 9'd36;
	assign pla_hcnt2[42] = reg_rs1 & hcnt == 9'd253;
	assign pla_hcnt2[43] = ~reg_rs1 & hcnt == 9'd206;
	assign pla_hcnt2[44] = reg_rs1 & hcnt == 9'd363;
	assign pla_hcnt2[45] = ~reg_rs1 & hcnt == 9'd294;

	assign vcnt_eq_pulse = pla_vcnt[41] | pla_vcnt[42] | pla_vcnt[43]
		| pla_vcnt[44] | pla_vcnt[45] | pla_vcnt[46] | pla_vcnt[47];
	assign vcnt_not_zero = ~pla_vcnt[40];
	assign vcnt_vdisp_active = ~(pla_vcnt[37] | pla_vcnt[38] | pla_vcnt[39]);
	assign vcnt_field_toggle = pla_vcnt[30] | pla_vcnt[31] | pla_vcnt[32]
		| pla_vcnt[33] | pla_vcnt[34] | pla_vcnt[35] | pla_vcnt[36];
	assign vcnt_vsync_area = pla_vcnt[25] | pla_vcnt[26] | pla_vcnt[27]
		| pla_vcnt[28] | pla_vcnt[29];
	assign vcnt_vblank_lo = pla_vcnt[17] | pla_vcnt[18] | pla_vcnt[19] | pla_vcnt[20]
		| pla_vcnt[21] | pla_vcnt[22] | pla_vcnt[23] | pla_vcnt[24];
	assign vcnt_vblank_hi = pla_vcnt[9] | pla_vcnt[10] | pla_vcnt[11] | pla_vcnt[12]
		| pla_vcnt[13] | pla_vcnt[14] | pla_vcnt[15] | pla_vcnt[16];
	assign vcnt_frame_end = pla_vcnt[1] | pla_vcnt[2] | pla_vcnt[3] | pla_vcnt[4]
		| pla_vcnt[5] | pla_vcnt[6] | pla_vcnt[7] | pla_vcnt[8];
	assign vcnt_not_max = ~pla_vcnt[0];

	assign hcnt_edclk = pla_hcnt1[60] | pla_hcnt1[61] | pla_hcnt1[62];
	assign hcnt_slot3 = pla_hcnt1[50] | pla_hcnt1[57] | pla_hcnt1[58] | pla_hcnt1[59];
	assign hcnt_slot2 = pla_hcnt1[49] | pla_hcnt1[55] | pla_hcnt1[56];
	assign hcnt_slot1 = pla_hcnt1[47] | pla_hcnt1[48] | pla_hcnt1[53] | pla_hcnt1[54];
	assign hcnt_slot0 = pla_hcnt1[45] | pla_hcnt1[46] | pla_hcnt1[51] | pla_hcnt1[52];
	assign hcnt_access_ext = pla_hcnt1[5] | pla_hcnt1[6] | pla_hcnt1[36] |
		pla_hcnt1[37] | pla_hcnt1[38] | pla_hcnt1[39];
	assign hcnt_access_main = pla_hcnt1[7] | pla_hcnt1[8] | pla_hcnt1[9] | pla_hcnt1[10]
		| pla_hcnt1[11] | pla_hcnt1[12] | pla_hcnt1[13] | pla_hcnt1[14]
		| pla_hcnt1[15] | pla_hcnt1[16] | pla_hcnt1[17] | pla_hcnt1[18]
		| pla_hcnt1[19] | pla_hcnt1[20] | pla_hcnt1[21] | pla_hcnt1[22]
		| pla_hcnt1[24] | pla_hcnt1[25] | pla_hcnt1[26] | pla_hcnt1[27]
		| pla_hcnt1[32] | pla_hcnt1[33] | pla_hcnt1[34] | pla_hcnt1[35];
	assign hcnt_odd = pla_hcnt1[44];
	assign hcnt_m4_window = pla_hcnt1[0] | pla_hcnt1[1] | pla_hcnt1[2] | pla_hcnt1[3] | pla_hcnt1[4];
	assign hcnt_ext_access = pla_hcnt1[36] | pla_hcnt1[37] | pla_hcnt1[38] | pla_hcnt1[39]
		| pla_hcnt1[43];
	assign hcnt_disp_active = hcnt_reload_pulse | pla_hcnt1[40] | pla_hcnt1[41] | pla_hcnt1[42];
	assign hcnt_fetch_all = pla_hcnt1[7] | pla_hcnt1[8] | pla_hcnt1[9] | pla_hcnt1[10]
		| pla_hcnt1[11] | pla_hcnt1[12] | pla_hcnt1[13] | pla_hcnt1[14]
		| pla_hcnt1[17] | pla_hcnt1[18] | pla_hcnt1[19] | pla_hcnt1[22] | pla_hcnt1[23]
		| pla_hcnt1[24] | pla_hcnt1[25] | pla_hcnt1[26] | pla_hcnt1[27]
		| pla_hcnt1[40] | pla_hcnt1[41] | pla_hcnt1[42]
		| pla_hcnt1[46] | pla_hcnt1[47] | pla_hcnt1[48]
		| pla_hcnt1[49] | pla_hcnt1[50];
	assign hcnt_blank_start = pla_hcnt1[23];
	assign hcnt_m4_border = pla_hcnt1[31] | pla_hcnt1[28] | pla_hcnt1[29] | pla_hcnt1[30];

	assign hcnt2_spr_end = pla_hcnt2[44] | pla_hcnt2[45];
	assign hcnt2_spr_mid = pla_hcnt2[42] | pla_hcnt2[43];
	assign hcnt2_adj_point = pla_hcnt2[40] | pla_hcnt2[41];
	assign hcnt2_sync_area = pla_hcnt2[38] | pla_hcnt2[39];
	assign hcnt2_scroll_end = pla_hcnt2[36] | pla_hcnt2[37];
	assign hcnt2_scroll_start = pla_hcnt2[34] | pla_hcnt2[35];
	assign hcnt2_line_first = pla_hcnt2[32] | pla_hcnt2[33];
	assign hcnt2_hint_pos = pla_hcnt2[30] | pla_hcnt2[31];
	assign hcnt2_fetch_point = pla_hcnt2[28] | pla_hcnt2[29];
	assign hcnt2_access_win = pla_hcnt2[24] | pla_hcnt2[25] | pla_hcnt2[26] | pla_hcnt2[27];
	assign hcnt2_hblank_end = pla_hcnt2[22] | pla_hcnt2[23];
	assign hcnt2_render_start = pla_hcnt2[20] | pla_hcnt2[21];
	assign hcnt2_not_active_end = ~(pla_hcnt2[17] | pla_hcnt2[18] | pla_hcnt2[19]);
	assign hcnt2_hint_time = pla_hcnt2[15] | pla_hcnt2[16];
	assign hcnt2_line_zero = pla_hcnt2[14];
	assign hcnt2_not_disp_start = ~(pla_hcnt2[12] | pla_hcnt2[13]);
	assign hcnt2_8px_bound = pla_hcnt2[11];
	assign hcnt2_cell_m4 = pla_hcnt2[9] | pla_hcnt2[10];
	assign hcnt2_cell_bound = pla_hcnt2[8];
	assign hcnt2_m4_border = pla_hcnt2[4] | pla_hcnt2[5] | pla_hcnt2[6] | pla_hcnt2[7];
	assign hcnt2_vram_slot = pla_hcnt2[2] | pla_hcnt2[3];
	assign hcnt2_pre_wrap = pla_hcnt2[1];
	assign hcnt2_sub_slot = pla_hcnt2[0];
	
	ym_sr_bit sr663(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(field_bit_trig), .sr_out(field_trig_pipe));
	
	assign VSYNC = csync_src;
	assign CSYNC_pull = ~csync_out_dly;
	assign HSYNC_pull = ~hsync_out_dly;

	// -------------------------------------------------------------------------
	// Scroll plane rendering
	// -------------------------------------------------------------------------
	// Planes A and B tile fetch and pixel serialization pipeline:
	//   1. VSRAM read → per-column vertical scroll offset
	//   2. H-scroll table read → per-line/per-tile horizontal scroll offset
	//   3. Nametable address calculation (reg_sa/reg_sb base + scroll)
	//   4. Tile pattern fetch from VRAM (4 bitplanes × 8 pixels)
	//   5. Pixel shift registers → 4-bit color index output per pixel
	// Also handles the window plane (overrides plane A in a rectangular region).

	assign w513 = (hclk1 & vscr_active) | reg_test1[7];
	
	ym_sr_bit sr178(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt[3]), .sr_out(l178));
	
	assign w514 = hclk1 & active_end & (reg_m5 | vcnt_at_max);
	
	ym_slatch #(.DATA_WIDTH(8)) sl179(.MCLK(MCLK), .en(reg_wr_hi), .inp(reg_data_l2[7:0]), .val(l179));
	
	assign w515 = reg_m5 ? vsram_out : { 3'h0, l179 };
	
	ym_slatch #(.DATA_WIDTH(11)) sl180(.MCLK(MCLK), .en(w516), .inp(vsram_out), .val(l180));
	
	ym_sr_bit_array #(.DATA_WIDTH(11)) sr181(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({ l182, data_rd_pipe }), .data_out(l181));
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr182(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vram_data[10:8]), .data_out(l182));
	
	ym_sr_bit sr183(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l209), .sr_out(l183));
	
	assign w516 = l209 & hclk1;
	
	assign w517 = w514 | reg_test1[7];
	
	ym_slatch #(.DATA_WIDTH(11)) sl184(.MCLK(MCLK), .en(w517), .inp(w515), .val(l184));
	
	ym_slatch #(.DATA_WIDTH(11)) sl185(.MCLK(MCLK), .en(w513), .inp(vsram_out), .val(l185));
	
	ym_sr_bit sr186(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w519), .sr_out(l186));
	
	assign w518 = { l184[10:8], l186 ? l184[7:0] : 8'h0 };
	
	assign w519 = ~(hcnt[7:6] == 2'h3 & reg_80_b7);
	
	assign w520 = reg_m5 & (l178 | reg_vscr);
	
	assign w521 = w520 ? l185 : w518;
	
	assign w522 = w521 + { 2'h0, vcnt_ext[8:0] };
	
	ym_slatch #(.DATA_WIDTH(2)) sl_hsz(.MCLK(MCLK), .en(reg_wr_88), .inp(reg_data_l2[1:0]), .val(reg_hsz));
	
	ym_slatch #(.DATA_WIDTH(2)) sl_vsz(.MCLK(MCLK), .en(reg_wr_88), .inp(reg_data_l2[5:4]), .val(reg_vsz));
	
	assign w523 = reg_hsz == 2'h0;
	
	assign w524 = reg_hsz == 2'h1;
	
	assign w525 = reg_hsz == 2'h3;
	
	assign w526 = interlace_dblres ? w522[10:4] : w522[9:3];
	
	assign w527 =
		( w523 ? w528 : 7'h0 ) |
		( w524 ? { w528[5:0], w555[5] }: 7'h0 ) |
		( w525 ? { w528[4:0], w555[6:5] } : 7'h0 );
	
	wire [2:0] w528_sum = w526[4:2] + { 2'h0, w529 };
	
	assign w528 = { reg_vsz[1] & w526[6], reg_vsz[0] & w526[5], w528_sum, w526[1:0] };
	
	assign w529 = ~reg_m5 & w530;
	
	assign w530 = w526[4:2] == 3'h7 | w526[5];
	
	assign w531 = reg_m5 & w558;
	
	ym_slatch #(.DATA_WIDTH(4)) sl_sa(.MCLK(MCLK), .en(reg_wr_82), .inp(reg_data_l2[6:3]), .val(reg_sa));
	
	ym_slatch #(.DATA_WIDTH(2)) sl_nt(.MCLK(MCLK), .en(reg_wr_82), .inp(reg_data_l2[2:1]), .val(reg_nt));
	
	ym_slatch #(.DATA_WIDTH(4)) sl_sb(.MCLK(MCLK), .en(reg_wr_84), .inp(reg_data_l2[3:0]), .val(reg_sb));
	
	assign w532 = l200 ? reg_sb : reg_sa;
	
	assign w533 = reg_m5 ? w527[6:5] : reg_nt;
	
	ym_slatch sl_8e_b0(.MCLK(MCLK), .en(wr_en_8E), .inp(reg_data_l2[0]), .val(reg_8e_b0));
	
	ym_slatch sl_8e_b4(.MCLK(MCLK), .en(wr_en_8E), .inp(reg_data_l2[4]), .val(reg_8e_b4));
	
	assign w534 = hcnt[8:7] != 2'h3;
	
	assign w535 = { reg_hscr ? w537[7:3] : 5'h0, reg_lscr ? w537[2:0] : 3'h0 };
	
	assign w536 = reg_rs1 ?
		{ w537[7:3], hcnt[8] } :
		{ reg_wd[0], w537[7:3] };
	
	ym_slatch #(.DATA_WIDTH(6)) sl_wd(.MCLK(MCLK), .en(reg_wr_83), .inp(reg_data_l2[6:1]), .val(reg_wd));
	
	ym_slatch #(.DATA_WIDTH(7)) sl_hs(.MCLK(MCLK), .en(wr_en_8D_m5), .inp(reg_data_l2[6:0]), .val(reg_hs));
	
	assign w537 = interlace_dblres ? vcnt_ext[8:1] : vcnt_ext[7:0];
	
	assign w538 = w546 ^ ~l187;
	
	assign w539 = w538 & w534;
	
	assign w540 = w545 ^ l189;
	
	assign w541 = (w540 | w539) & ~hcnt[3] & reg_m5;
	
	ym_slatch #(.DATA_WIDTH(5)) sl_whp(.MCLK(MCLK), .en(reg_wr_87), .inp(reg_data_l2[4:0]), .val(reg_whp));
	
	ym_slatch sl_rigt(.MCLK(MCLK), .en(reg_wr_87), .inp(reg_data_l2[7]), .val(reg_rigt));
	
	ym_slatch #(.DATA_WIDTH(5)) sl_wvp(.MCLK(MCLK), .en(reg_wr_86), .inp(reg_data_l2[4:0]), .val(reg_wvp));
	
	ym_slatch sl_down(.MCLK(MCLK), .en(reg_wr_86), .inp(reg_data_l2[7]), .val(reg_down));
	
	assign w542 = reg_test1[7] | active_end;
	
	assign w543 = active_end & (reg_m5 | vcnt_at_max);
	
	assign w544 = w543 | reg_test1[7];
	
	ym_slatch sl187(.MCLK(MCLK), .en(w542), .inp(reg_rigt), .val(l187));
	
	ym_slatch #(.DATA_WIDTH(5)) sl188(.MCLK(MCLK), .en(w542), .inp(reg_whp), .val(l188));
	
	ym_slatch sl189(.MCLK(MCLK), .en(w544), .inp(reg_down), .val(l189));
	
	ym_slatch #(.DATA_WIDTH(5)) sl190(.MCLK(MCLK), .en(w544), .inp(reg_wvp), .val(l190));
	
	assign w545 = w537[7:3] < l190;
	
	assign w546 = l188 <= hcnt[8:4];
	
	assign w547 = reg_test1[7] | active_end;
	
	ym_slatch #(.DATA_WIDTH(8)) sl_88(.MCLK(MCLK), .en(wr_en_88), .inp(reg_data_l2[7:0]), .val(reg_88));
	
	ym_slatch #(.DATA_WIDTH(8)) sl191(.MCLK(MCLK), .en(w570), .inp(vram_serial), .val(l191));
	
	ym_slatch #(.DATA_WIDTH(8)) sl192(.MCLK(MCLK), .en(w572), .inp(vram_serial), .val(l192));
	
	assign w548 = slot0_active | slot1_active;
	
	assign w549 = slot2_active | (reg_test1[8] & cpu_pen);
	
	assign w550 = slot0_active & ~reg_m5;
	
	assign w551 = w550 | slot1_active;
	
	assign w552 = ~(~reg_80_b6 | vcnt_ext[5] | vcnt_ext[6] | vcnt_ext[4] | vcnt_ext[7]);
	
	assign w553 = w552 | reg_m5;
	
	ym_slatch #(.DATA_WIDTH(8)) sl193(.MCLK(MCLK), .en(w547), .inp(reg_88), .val(l193));
	
	assign w554 = ~(
		(~w553 ? { 2'h0, l193 } : 10'h0) |
		(w574 ? { l194, l191 } : 10'h0) |
		(w575 ? { l195, l192 } : 10'h0)
		);
	
	ym_slatch #(.DATA_WIDTH(2)) sl194(.MCLK(MCLK), .en(w571), .inp(vram_serial[1:0]), .val(l194));
	
	ym_slatch #(.DATA_WIDTH(2)) sl195(.MCLK(MCLK), .en(w573), .inp(vram_serial[1:0]), .val(l195));
	
	assign w555 = { w554[9:4], w564 } + { w567, w565, w563 } + 7'h1;
	
	ym_sr_bit sr196(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w548), .sr_out(l196));
	
	ym_sr_bit sr197(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hcnt[3]), .sr_out(l197));
	
	ym_sr_bit sr198(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~w550), .sr_out(l198));
	
	ym_sr_bit sr199(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w551), .sr_out(l199));
	
	assign w556 = reg_m5 & ~reg_test1[8] & w549;
	
	ym_sr_bit sr200(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w556), .sr_out(l200));
	
	assign w557 = ~w541 & slot3_active;
	
	ym_sr_bit sr201(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w557), .sr_out(l201));
	
	assign w558 = l200 | l201;
	
	assign w559 = slot3_active & w541;
	
	ym_sr_bit sr202(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w559), .sr_out(l202));
	
	assign w560 = slot3_active | w549;
	
	ym_sr_bit sr203(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w560), .sr_out(l203));
	
	ym_sr_bit sr204(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l203), .sr_out(l204));
	
	ym_sr_bit sr205(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l204), .sr_out(l205));
	
	assign w561 = reg_rs1 ? w568 : hcnt[8];
	
	assign w562 = ~(w561 | reg_m5);
	
	assign w563 = w562 & hcnt[3];
	
	assign w564 = w554[3] | reg_m5;
	
	assign w565 = w561 ? 4'hf : hcnt[7:4];
	
	assign w566 = reg_m5 & l199;
	
	assign w567 = w561 ? reg_hsz : { w568, hcnt[8] };
	
	assign w568 = hcnt[8] & (hcnt[7] | hcnt[6]);
	
	ym_sr_bit sr206(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cram_wr_sel), .sr_out(l206));
	
	ym_sr_bit sr207(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsram_wr_hi), .sr_out(l207));
	
	ym_sr_bit sr208(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsram_wr_lo), .sr_out(l208));
	
	ym_sr_bit sr209(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l206), .sr_out(l209));
	
	ym_sr_bit sr210(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l207), .sr_out(l210));
	
	ym_sr_bit sr211(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l208), .sr_out(l211));
	
	assign w569 = l206 | l207 | l208;
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr212(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(w626), .data_out(l212));
	
	ym_dlatch_1 dl213(.MCLK(MCLK), .c1(clk1), .inp(~(hclk1 & l316)), .nval(l213));
	
	assign w570 = l213 & clk2;
	
	ym_sr_bit sr214(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l213), .sr_out(l214));
	
	assign w571 = l214 & clk2;
	
	ym_sr_bit sr215(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l214), .sr_out(l215));
	
	assign w572 = l215 & clk2;
	
	ym_sr_bit sr216(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l215), .sr_out(l216));
	
	assign w573 = l216 & clk2;
	
	assign w574 = ~w541 & reg_m5 & ~hcnt[3];
	
	assign w575 = ~w541 & reg_m5 & hcnt[3];
	
	assign w576 = l217 ? vcnt_ext[3:0] : w522[3:0];
	
	assign w577 = w583 ? ~w576 : w576;
	
	assign w578 = interlace_dblres ? { l219, w577[3] } : { l220[0], l219 };
	
	assign w579 = interlace_dblres ? l220[2:0] : { w581, l220[2:1] };
	
	assign w580 = interlace_dblres ? { l222[2:0], l221, w577[3] } : { w581, l222[2:0], l221 };
	
	assign w581 = l197 ? reg_8e_b4 : reg_8e_b0;
	
	ym_sr_bit sr217(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w541), .sr_out(l217));
	
	assign w582 = slot0_active & reg_m5;
	
	ym_sr_bit sr218(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w582), .sr_out(l218));
	
	assign w583 = l218 ? l222[4] : w585;
	
	ym_slatch #(.DATA_WIDTH(8)) sl219(.MCLK(MCLK), .en(w615), .inp(vram_serial), .val(l219));
	
	ym_slatch #(.DATA_WIDTH(8)) sl220(.MCLK(MCLK), .en(w616), .inp(vram_serial), .val(l220));
	
	ym_slatch #(.DATA_WIDTH(8)) sl221(.MCLK(MCLK), .en(w617), .inp(vram_serial), .val(l221));
	
	ym_slatch #(.DATA_WIDTH(8)) sl222(.MCLK(MCLK), .en(w618), .inp(vram_serial), .val(l222));
	
	assign w584 = reg_m5 ? l220[3] : l220[1];
	assign w585 = reg_m5 ? l220[4] : l220[2];
	assign w586 = reg_m5 ? l220[6:5] : { 1'h0, l220[3] };
	assign w587 = reg_m5 ? l220[7] : l220[4];
	
	ym_slatch #(.DATA_WIDTH(8)) sl223(.MCLK(MCLK), .en(w591), .inp(vram_serial), .val(l223));
	
	ym_slatch #(.DATA_WIDTH(8)) sl224(.MCLK(MCLK), .en(w590), .inp(vram_serial), .val(l224));
	
	ym_slatch #(.DATA_WIDTH(8)) sl225(.MCLK(MCLK), .en(w589), .inp(vram_serial), .val(l225));
	
	ym_slatch #(.DATA_WIDTH(8)) sl226(.MCLK(MCLK), .en(w588), .inp(vram_serial), .val(l226));
	
	ym_dlatch_1 dl227(.MCLK(MCLK), .c1(clk1), .inp(w613), .nval(l227));
	
	ym_sr_bit sr228(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l227), .sr_out(l228));
	
	ym_sr_bit sr229(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l228), .sr_out(l229));
	
	ym_sr_bit sr230(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l229), .sr_out(l230));
	
	ym_sr_bit sr231(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l230), .sr_out(l231));
	
	assign w588 = l227 & clk2;
	
	assign w589 = l228 & clk2;
	
	assign w590 = l229 & clk2;
	
	assign w591 = l230 & clk2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl232(.MCLK(MCLK), .en(w592), .inp(l223), .val(l232));
	
	ym_slatch #(.DATA_WIDTH(8)) sl233(.MCLK(MCLK), .en(w592), .inp(l224), .val(l233));
	
	ym_slatch #(.DATA_WIDTH(8)) sl234(.MCLK(MCLK), .en(w592), .inp(l225), .val(l234));
	
	ym_slatch #(.DATA_WIDTH(8)) sl235(.MCLK(MCLK), .en(w592), .inp(l226), .val(l235));
	
	assign w592 = w598 & clk2;
	
	ym_slatch #(.DATA_WIDTH(4)) sl236(.MCLK(MCLK), .en(l242), .inp(w554[3:0]), .val(l236));
	
	assign w593 = w614 & l236 == 4'hf;
	
	ym_slatch #(.DATA_WIDTH(8)) sl237(.MCLK(MCLK), .en(w611), .inp(l232), .val(l237));
	
	ym_slatch #(.DATA_WIDTH(8)) sl238(.MCLK(MCLK), .en(w611), .inp(l233), .val(l238));
	
	ym_slatch #(.DATA_WIDTH(8)) sl239(.MCLK(MCLK), .en(w611), .inp(l234), .val(l239));
	
	ym_slatch #(.DATA_WIDTH(8)) sl240(.MCLK(MCLK), .en(w611), .inp(l235), .val(l240));
	
	ym_cnt_bit_load #(.DATA_WIDTH(4)) cnt241(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(1'h1), .reset(1'h0), .load(w614), .load_val(l236), .val(l241));
	
	ym_sr_bit sr242(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w614), .sr_out(l242));
	
	ym_slatch #(.DATA_WIDTH(8)) sl243(.MCLK(MCLK), .en(w597), .inp(vram_serial), .val(l243));
	
	ym_slatch #(.DATA_WIDTH(8)) sl244(.MCLK(MCLK), .en(w596), .inp(vram_serial), .val(l244));
	
	ym_slatch #(.DATA_WIDTH(8)) sl245(.MCLK(MCLK), .en(w595), .inp(vram_serial), .val(l245));
	
	ym_slatch #(.DATA_WIDTH(8)) sl246(.MCLK(MCLK), .en(w594), .inp(vram_serial), .val(l246));
	
	ym_dlatch_1 dl247(.MCLK(MCLK), .c1(clk1), .inp(w600), .nval(l247));
	
	ym_sr_bit sr248(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l247), .sr_out(l248));
	
	ym_sr_bit sr249(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l248), .sr_out(l249));
	
	ym_sr_bit sr250(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l249), .sr_out(l250));
	
	assign w594 = l247 & clk2;
	
	assign w595 = l248 & clk2;
	
	assign w596 = l249 & clk2;
	
	assign w597 = l250 & clk2;
	
	ym_dlatch_1 dl251(.MCLK(MCLK), .c1(clk1), .inp(w633), .nval(l251));
	
	ym_sr_bit sr252(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l251), .sr_out(l252));
	
	ym_sr_bit sr253(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l252), .sr_out(l253));
	
	ym_sr_bit sr254(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l253), .sr_out(l254));
	
	ym_slatch #(.DATA_WIDTH(8)) sl255(.MCLK(MCLK), .en(w592), .inp(l243), .val(l255));
	
	ym_slatch #(.DATA_WIDTH(8)) sl256(.MCLK(MCLK), .en(w592), .inp(l244), .val(l256));
	
	ym_slatch #(.DATA_WIDTH(8)) sl257(.MCLK(MCLK), .en(w592), .inp(l245), .val(l257));
	
	ym_slatch #(.DATA_WIDTH(8)) sl258(.MCLK(MCLK), .en(w592), .inp(l246), .val(l258));
	
	ym_slatch #(.DATA_WIDTH(8)) sl259(.MCLK(MCLK), .en(w591),
		.inp({ w587, l222[7], w586[1], l222[6], w586[0], l222[5], w584, l222[3] }), .val(l259));
	
	ym_slatch #(.DATA_WIDTH(8)) sl260(.MCLK(MCLK), .en(w592), .inp(l259), .val(l260));
	
	ym_slatch #(.DATA_WIDTH(8)) sl261(.MCLK(MCLK), .en(w611), .inp(l255), .val(l261));
	
	ym_slatch #(.DATA_WIDTH(8)) sl262(.MCLK(MCLK), .en(w611), .inp(l256), .val(l262));
	
	ym_slatch #(.DATA_WIDTH(8)) sl263(.MCLK(MCLK), .en(w611), .inp(l257), .val(l263));
	
	ym_slatch #(.DATA_WIDTH(8)) sl264(.MCLK(MCLK), .en(w611), .inp(l258), .val(l264));
	
	ym_slatch #(.DATA_WIDTH(8)) sl265(.MCLK(MCLK), .en(w611), .inp(l260), .val(l265));
	
	assign w598 = reg_m5 ? l302 : l231;
	
	assign w599 = reg_m5 & l241[3];
	
	assign w600 = ~(hclk1 & w612);
	
	assign w601 = reg_m5 ^ l241[1];
	
	assign w602 = w599 ? l265[0] : l265[1];
	
	assign w603 = w599 ? l265[2] : l265[3];
	
	assign w604 = w599 ? l265[4] : l265[5];
	
	assign w605 = w599 ? l265[6] : l265[7];
	
	wire[2:0] w606_t = { l241[2], w601, l241[0] };
	
	assign w606 = w602 ? ~w606_t : w606_t;
	
	wire [7:0] w606_sel;
	
	assign w606_sel[0] = w606 == 3'h0;
	assign w606_sel[1] = w606 == 3'h1;
	assign w606_sel[2] = w606 == 3'h2;
	assign w606_sel[3] = w606 == 3'h3;
	assign w606_sel[4] = w606 == 3'h4;
	assign w606_sel[5] = w606 == 3'h5;
	assign w606_sel[6] = w606 == 3'h6;
	assign w606_sel[7] = w606 == 3'h7;
	
	wire [3:0] w607_m4 =
		(w606_sel[0] ? { l239[7], l240[7], l263[7], l264[7] } : 4'h0 ) |
		(w606_sel[1] ? { l239[6], l240[6], l263[6], l264[6] } : 4'h0 ) |
		(w606_sel[2] ? { l239[5], l240[5], l263[5], l264[5] } : 4'h0 ) |
		(w606_sel[3] ? { l239[4], l240[4], l263[4], l264[4] } : 4'h0 ) |
		(w606_sel[4] ? { l239[3], l240[3], l263[3], l264[3] } : 4'h0 ) |
		(w606_sel[5] ? { l239[2], l240[2], l263[2], l264[2] } : 4'h0 ) |
		(w606_sel[6] ? { l239[1], l240[1], l263[1], l264[1] } : 4'h0 ) |
		(w606_sel[7] ? { l239[0], l240[0], l263[0], l264[0] } : 4'h0 );
	
	wire [3:0] w607_m5_1 =
		(w606_sel[7] ? l261[3:0] : 4'h0) |
		(w606_sel[6] ? l261[7:4] : 4'h0) |
		(w606_sel[5] ? l262[3:0] : 4'h0) |
		(w606_sel[4] ? l262[7:4] : 4'h0) |
		(w606_sel[3] ? l263[3:0] : 4'h0) |
		(w606_sel[2] ? l263[7:4] : 4'h0) |
		(w606_sel[1] ? l264[3:0] : 4'h0) |
		(w606_sel[0] ? l264[7:4] : 4'h0);
	
	wire [3:0] w607_m5_2 =
		(w606_sel[7] ? l237[3:0] : 4'h0) |
		(w606_sel[6] ? l237[7:4] : 4'h0) |
		(w606_sel[5] ? l238[3:0] : 4'h0) |
		(w606_sel[4] ? l238[7:4] : 4'h0) |
		(w606_sel[3] ? l239[3:0] : 4'h0) |
		(w606_sel[2] ? l239[7:4] : 4'h0) |
		(w606_sel[1] ? l240[3:0] : 4'h0) |
		(w606_sel[0] ? l240[7:4] : 4'h0);
	
	assign w607 =
		(~reg_m5 ? w607_m4 : 4'h0) |
		((reg_m5 & ~l241[3]) ? w607_m5_1 : 4'h0) |
		((reg_m5 & l241[3]) ? w607_m5_2 : 4'h0);
	
	assign w608 = l241[3] | ~reg_m5;
	
	assign w609 = w608 & l241[2:0] == 3'h7;
	
	assign w610 = ~(w609 | w593);
	
	ym_dlatch_1 dl266(.MCLK(MCLK), .c1(hclk1), .inp(w610), .nval(l266));
	
	assign w611 = hclk2 & l266;
	
	assign w612 = cell_m4_active | (reg_test1[9] & cpu_pen);
	
	ym_sr_bit sr267(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w612), .sr_out(l267));
	
	ym_sr_bit sr268(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l267), .sr_out(l268));
	
	assign w613 = ~(l268 & hclk1);
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr269(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(w607), .data_out(l269));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr270(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(l269), .data_out(l270));
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr271(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({ w604, w603 }), .data_out(l271));
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr272(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(l271), .data_out(l272));
	
	ym_sr_bit sr273(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w605), .sr_out(l273));
	
	ym_sr_bit sr274(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l273), .sr_out(l274));
	
	assign w614 = sub_slot_active | tst_fn4_wr;
	
	ym_slatch #(.DATA_WIDTH(8)) sl275(.MCLK(MCLK), .en(w622), .inp(vram_serial), .val(l275));
	
	ym_slatch #(.DATA_WIDTH(8)) sl276(.MCLK(MCLK), .en(w621), .inp(vram_serial), .val(l276));
	
	ym_slatch #(.DATA_WIDTH(8)) sl277(.MCLK(MCLK), .en(w620), .inp(vram_serial), .val(l277));
	
	ym_slatch #(.DATA_WIDTH(8)) sl278(.MCLK(MCLK), .en(w619), .inp(vram_serial), .val(l278));
	
	ym_slatch #(.DATA_WIDTH(8)) sl279(.MCLK(MCLK), .en(w631), .inp(l275), .val(l279));
	
	ym_slatch #(.DATA_WIDTH(8)) sl280(.MCLK(MCLK), .en(w631), .inp(l276), .val(l280));
	
	ym_slatch #(.DATA_WIDTH(8)) sl281(.MCLK(MCLK), .en(w631), .inp(l277), .val(l281));
	
	ym_slatch #(.DATA_WIDTH(8)) sl282(.MCLK(MCLK), .en(w631), .inp(l278), .val(l282));
	
	assign w615 = l251 & clk2;
	assign w616 = l252 & clk2;
	assign w617 = l253 & clk2;
	assign w618 = l254 & clk2;
	
	ym_dlatch_1 dl283(.MCLK(MCLK), .c1(clk1), .inp(w640), .nval(l283));
	
	ym_sr_bit sr284(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l283), .sr_out(l284));
	
	ym_sr_bit sr285(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l284), .sr_out(l285));
	
	ym_sr_bit sr286(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l285), .sr_out(l286));
	
	assign w619 = l283 & clk2;
	assign w620 = l284 & clk2;
	assign w621 = l285 & clk2;
	assign w622 = l286 & clk2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl287(.MCLK(MCLK), .en(w645), .inp(l279), .val(l287));
	
	ym_slatch #(.DATA_WIDTH(8)) sl288(.MCLK(MCLK), .en(w645), .inp(l280), .val(l288));
	
	ym_slatch #(.DATA_WIDTH(8)) sl289(.MCLK(MCLK), .en(w645), .inp(l281), .val(l289));
	
	ym_slatch #(.DATA_WIDTH(8)) sl290(.MCLK(MCLK), .en(w645), .inp(l282), .val(l290));
	
	ym_slatch #(.DATA_WIDTH(8)) sl291(.MCLK(MCLK), .en(w631),
		.inp({ w587, l222[7], w586[1], l222[6], w586[0], l222[5], w584, l222[3] }), .val(l291));
	
	ym_slatch #(.DATA_WIDTH(8)) sl292(.MCLK(MCLK), .en(w645), .inp(l291), .val(l292));
	
	assign w623 = ~(reg_m5 & reg_vscr);
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr293(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vram_address[6:1]), .data_out(l293));
	
	assign w624 = w623 ^ hcnt[3];
	
	assign w625 = reg_vscr ? hcnt[8:4] : 5'h0;
	
	assign w626 = w569 ? l293 : { w625, w624 };
	
	ym_slatch #(.DATA_WIDTH(8)) sl294(.MCLK(MCLK), .en(w630), .inp(vram_serial), .val(l294));
	
	ym_slatch #(.DATA_WIDTH(8)) sl295(.MCLK(MCLK), .en(w629), .inp(vram_serial), .val(l295));
	
	ym_slatch #(.DATA_WIDTH(8)) sl296(.MCLK(MCLK), .en(w628), .inp(vram_serial), .val(l296));
	
	ym_slatch #(.DATA_WIDTH(8)) sl297(.MCLK(MCLK), .en(w627), .inp(vram_serial), .val(l297));
	
	ym_dlatch_1 dl298(.MCLK(MCLK), .c1(clk1), .inp(w639), .nval(l298));
	
	ym_sr_bit sr299(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l298), .sr_out(l299));
	
	ym_sr_bit sr300(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l299), .sr_out(l300));
	
	ym_sr_bit sr301(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l300), .sr_out(l301));
	
	ym_sr_bit sr302(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(l301), .sr_out(l302));
	
	assign w627 = l298 & clk2;
	
	assign w628 = l299 & clk2;
	
	assign w629 = l300 & clk2;
	
	assign w630 = l301 & clk2;
	
	assign w631 = l302 & clk2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl303(.MCLK(MCLK), .en(w631), .inp(l294), .val(l303));
	
	ym_slatch #(.DATA_WIDTH(8)) sl304(.MCLK(MCLK), .en(w631), .inp(l295), .val(l304));
	
	ym_slatch #(.DATA_WIDTH(8)) sl305(.MCLK(MCLK), .en(w631), .inp(l296), .val(l305));
	
	ym_slatch #(.DATA_WIDTH(8)) sl306(.MCLK(MCLK), .en(w631), .inp(l297), .val(l306));
	
	ym_slatch #(.DATA_WIDTH(8)) sl307(.MCLK(MCLK), .en(w645), .inp(l303), .val(l307));
	
	ym_slatch #(.DATA_WIDTH(8)) sl308(.MCLK(MCLK), .en(w645), .inp(l304), .val(l308));
	
	ym_slatch #(.DATA_WIDTH(8)) sl309(.MCLK(MCLK), .en(w645), .inp(l305), .val(l309));
	
	ym_slatch #(.DATA_WIDTH(8)) sl310(.MCLK(MCLK), .en(w645), .inp(l306), .val(l310));
	
	ym_cnt_bit_load #(.DATA_WIDTH(4)) cnt311(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(1'h1), .reset(1'h0), .load(w649), .load_val({~w554[3], w554[2:0]}), .val(l311));
	
	wire [2:0] w632_t = l311[2:0];
	
	assign w632 = w634 ? ~w632_t : w632_t;
	
	assign w633 = ~(hclk1 & l205);
	
	assign w634 = l311[3] ? l292[0] : l292[1];
	assign w635 = l311[3] ? l292[2] : l292[3];
	assign w636 = l311[3] ? l292[4] : l292[5];
	assign w637 = l311[3] ? l292[6] : l292[7];
	
	assign w638 = cell_bound_active | (reg_test1[10] & cpu_pen);
	
	ym_sr_bit sr312(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w638), .sr_out(l312));
	
	ym_sr_bit sr313(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l312), .sr_out(l313));
	
	assign w639 = ~(hclk1 & l313);
	
	assign w640 = ~(hclk1 & w638);
	
	assign w641 = { w632[2], ~w632[1], w632[0] };
	
	assign w642 = blank_slot_active | (reg_test1[7] & cpu_pen);
	
	ym_sr_bit sr314(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w642), .sr_out(l314));
	
	ym_sr_bit sr315(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l314), .sr_out(l315));
	
	ym_sr_bit sr316(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l315), .sr_out(l316));
	
	assign w643 = l314 & ~reg_test1[7];
	
	assign w644 = ~(l311 == 4'hf);
	
	ym_dlatch_1 dl317(.MCLK(MCLK), .c1(hclk1), .inp(w644), .nval(l317));
	
	assign w645 = l317 & hclk2;
	
	assign w646 = l269 != 4'h0;
	
	wire [7:0] w641_sel;
	
	assign w641_sel[0] = w641 == 3'h0;
	assign w641_sel[1] = w641 == 3'h1;
	assign w641_sel[2] = w641 == 3'h2;
	assign w641_sel[3] = w641 == 3'h3;
	assign w641_sel[4] = w641 == 3'h4;
	assign w641_sel[5] = w641 == 3'h5;
	assign w641_sel[6] = w641 == 3'h6;
	assign w641_sel[7] = w641 == 3'h7;
	
	wire [3:0] w647_1 =
		(w641_sel[7] ? l307[3:0] : 4'h0) |
		(w641_sel[6] ? l307[7:4] : 4'h0) |
		(w641_sel[5] ? l308[3:0] : 4'h0) |
		(w641_sel[4] ? l308[7:4] : 4'h0) |
		(w641_sel[3] ? l309[3:0] : 4'h0) |
		(w641_sel[2] ? l309[7:4] : 4'h0) |
		(w641_sel[1] ? l310[3:0] : 4'h0) |
		(w641_sel[0] ? l310[7:4] : 4'h0);
	
	wire [3:0] w647_2 =
		(w641_sel[7] ? l287[3:0] : 4'h0) |
		(w641_sel[6] ? l287[7:4] : 4'h0) |
		(w641_sel[5] ? l288[3:0] : 4'h0) |
		(w641_sel[4] ? l288[7:4] : 4'h0) |
		(w641_sel[3] ? l289[3:0] : 4'h0) |
		(w641_sel[2] ? l289[7:4] : 4'h0) |
		(w641_sel[1] ? l290[3:0] : 4'h0) |
		(w641_sel[0] ? l290[7:4] : 4'h0);
	
	assign w647 =
		(l311[3] ? w647_1 : 4'h0) |
		(~l311[3] ? w647_2 : 4'h0);
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr318(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(w647), .data_out(l318));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr319(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(l318), .data_out(l319));
	
	assign w648 = l318 != 4'h0;
	
	ym_sr_bit sr320(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(w637), .sr_out(l320));
	
	ym_sr_bit sr321(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(l320), .sr_out(l321));
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr322(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({w636, w635}), .data_out(l322));
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr323(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(l322), .data_out(l323));
	
	assign w649 = tst_fn5_wr | pre_wrap_active;
	
	// -------------------------------------------------------------------------
	// VSRAM (Vertical Scroll RAM)
	// -------------------------------------------------------------------------
	// 40 entries × 11 bits. Each entry provides the vertical scroll offset
	// for a 2-cell (16-pixel) column. Directly indexed by the scroll plane
	// rendering logic. Dual-ported: write from FIFO, read during tile fetch.

	wire [5:0] vsram_index = l212;
	
	always @(posedge MCLK)
	begin
		if (vsram_index < 6'd40)
		begin
			if (hclk1) // write cycle
			begin
				if (l211)
					vsram[vsram_index][7:0] <= l181[7:0];
				if (l210)
					vsram[vsram_index][10:8] <= l181[10:8];
			end
			vsram_out <= vsram[vsram_index];
			if (vsram_index[0])
				vsram_out_1 <= vsram[vsram_index];
			else
				vsram_out_0 <= vsram[vsram_index];
		end
		else
		begin
			if (vsram_index[0])
				vsram_out <= vsram_out & vsram_out_1;
			else
				vsram_out <= vsram_out & vsram_out_0;
		end
	end
	
	
	// -------------------------------------------------------------------------
	// Sprite processing
	// -------------------------------------------------------------------------
	// Multi-stage sprite engine:
	//   1. SAT cache traversal — follow link chain, Y-range test vs scanline
	//   2. Attribute fetch — read size, link, pattern, palette, priority, flip
	//   3. Sprite line render — fetch tile patterns for visible sprites
	//   4. X-position sort — place sprite pixels into the line buffer
	//   5. Pixel data pipeline — read line buffer during active display
	// Supports up to 80 sprites total, 20 per scanline (H40) or 16 (H32).

	assign sat_read_mux = sat_read_phase ? { sat_size, sat_link } : sat_ypos;
	
	ym_slatch #(.DATA_WIDTH(11)) sl324(.MCLK(MCLK), .en(sat_field_latch_en), .inp(sat_read_mux), .val(sat_field_latch));
	
	ym_sr_bit sr325(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_address[1]), .sr_out(sat_read_phase));
	
	ym_sr_bit sr326(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsram_wr_test), .sr_out(sat_rd_pipe_0));
	
	ym_sr_bit_array #(.DATA_WIDTH(11)) sr327(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in( { sat_size, sat_link } ), .data_out(sat_attr_pipe));
	
	ym_dlatch_1 #(.DATA_WIDTH(11)) dl328(.MCLK(MCLK), .c1(hclk1), .inp(sat_attr_pipe), .nval(sat_attr_hold));
	
	assign sat_field_latch_en = hclk1 & sat_rd_pipe_0;
	
	ym_sr_bit sr329(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_rd_pipe_0), .sr_out(sat_rd_pipe_1));
	
	ym_sr_bit sr330(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_rd_pipe_1), .sr_out(sat_rd_pipe_2));
	
	ym_sr_bit sr331(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(access_ext_dly), .sr_out(sat_rd_pipe_3));
	
	ym_slatch #(.DATA_WIDTH(11)) sl332(.MCLK(MCLK), .en(sat_attr_latch_en), .inp(sat_attr_hold), .val(sat_attr_latch));
	
	ym_dlatch_1 dl333(.MCLK(MCLK), .c1(hclk1), .inp(sat_active), .nval(sat_active_n));
	
	assign sat_attr_latch_en = hclk2 & clk2 & sat_active_n;
	
	assign spr_y_adjusted = vcnt_ext + { 1'h0, interlace_dblres, m5_single_res, 5'h0, interlace_dblres, m5_single_res };
	
	assign m5_single_res = ~interlace_dblres & reg_m5;
	
	assign sat_link_is_7f = reg_m5 & sat_attr_latch[6:0] == 7'h7f;
	
	assign sat_ybit8_valid = reg_m5 & ~sat_attr_latch[8];
	
	assign sat_y_sign = reg_m5 ? ~sat_attr_latch[7] : reg_81_b1;
	
	assign sat_stop = sat_rd_window & (sat_link_is_7f | y_d0_terminate);
	
	assign sat_clk_gate = ~(hclk1 & ~sat_active);
	
	ym_dlatch_1 dl334(.MCLK(MCLK), .c1(clk1), .inp(sat_clk_gate), .nval(sat_clk_latch));
	
	assign sat_ypos_strobe_a = sat_clk_latch & clk2;
	
	ym_sr_bit sr335(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(sat_clk_latch), .sr_out(sat_clk_pipe));
	
	assign sat_ypos_strobe_b = sat_clk_pipe & clk2;
	
	assign y_delta = 10'h1 + spr_y_adjusted + y_cmp_val;
	
	ym_sr_bit sr336(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~sat_attr_latch[10]), .sr_out(sat_vs1_pipe));
	
	ym_sr_bit sr337(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(~sat_attr_latch[9]), .sr_out(sat_vs0_pipe));
	
	ym_sr_bit sr338(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_ybit8_valid), .sr_out(sat_ybit8_pipe));
	
	ym_sr_bit sr339(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_y_sign), .sr_out(sat_ysign_pipe));
	
	assign y_size_test_0 = ~(sat_ybit8_pipe | sat_ysign_pipe);
	
	assign y_size_test_1 = sat_ybit8_pipe & ~sat_ysign_pipe;
	
	assign spr_y_visible = y_range_bit_0 & y_range_bit_1 & y_range_bit_2 & y_range_bit_3
		& y_range_bit_4 & y_delta_result[7] & y_delta_result[6] & ~y_size_adj_ext & sat_avail_pipe;
	
	assign sat_y_mux = m4_ypos_sel ? { 2'h0, sat_vram_serial[7:0] } : sat_ypos_hold;
	
	ym_dlatch_1 #(.DATA_WIDTH(10)) dl340(.MCLK(MCLK), .c1(hclk1), .inp(sat_y_mux), .nval(y_cmp_val));
	
	ym_dlatch_2 #(.DATA_WIDTH(6)) dl341_1(.MCLK(MCLK), .c2(hclk2), .inp(y_delta[5:0]), .val(y_delta_result[5:0]));
	ym_dlatch_2 #(.DATA_WIDTH(4)) dl341_2(.MCLK(MCLK), .c2(hclk2), .inp(y_delta[9:6]), .nval(y_delta_result[9:6]));
	
	assign y_d0_terminate = sat_y_mux == 10'd208 & ~reg_m5;
	
	assign y_range_bit_0 = ~(y_size_test_0 & y_size_adj_lo);
	
	assign y_range_bit_1 = ~(~sat_ybit8_pipe & y_size_adj_hi);
	
	assign y_range_bit_2 = ~(y_size_test_1 & y_size_adj_lo & y_size_adj_hi);
	
	assign y_range_bit_3 = y_delta_result[9] | ~interlace_dblres;
	
	assign y_range_bit_4 = y_delta_result[8] | ~reg_m5;
	
	ym_slatch #(.DATA_WIDTH(8)) sl342(.MCLK(MCLK), .en(sat_ypos_strobe_b), .inp(vram_serial), .val(sat_vram_serial));
	
	ym_slatch #(.DATA_WIDTH(10)) sl343(.MCLK(MCLK), .en(sat_ypos_strobe_a), .inp(sat_ypos_src), .val(sat_ypos_latch));
	
	ym_dlatch_2 #(.DATA_WIDTH(10)) dl344(.MCLK(MCLK), .c2(hclk2), .inp(sat_ypos_latch), .nval(sat_ypos_hold));
	
	assign y_size_adj_lo = interlace_dblres ? y_delta_result[4] : y_delta_result[3];
	
	assign y_size_adj_hi = interlace_dblres ? y_delta_result[5] : y_delta_result[4];
	
	assign y_size_adj_ext = interlace_dblres ? 1'h0 : y_delta_result[5];
	
	assign sat_rd_window = reg_m5 ? sat_rd_win_m5 : sat_rd_win_m4;
	
	assign m4_ypos_sel = ~reg_m5 & sat_start_pipe_3;
	
	assign sat_busy = sat_start_pipe_0 | sat_start_pipe_1 | sat_rd_pipe_5;
	
	assign sat_active = ~(reg_m5 ? sat_busy : sat_start_pipe_1);
	
	ym_sr_bit_array #(.DATA_WIDTH(10)) sr345(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(sat_ypos), .data_out(sat_ypos_pipe_0));
	
	ym_dlatch_1 #(.DATA_WIDTH(10)) dl346(.MCLK(MCLK), .c1(hclk1), .inp(sat_ypos_pipe_0), .nval(sat_ypos_pipe_1));
	
	assign sat_ypos_src = reg_m5 ? sat_ypos_pipe_1 : { 2'h3, ~vram_serial };
	
	ym_sr_bit sr347(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_start_latch), .sr_out(sat_start_pipe_0));
	
	ym_sr_bit sr348(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_start_pipe_0), .sr_out(sat_start_pipe_1));
	
	ym_sr_bit sr349(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_start_pipe_1), .sr_out(sat_start_pipe_2));
	
	ym_sr_bit sr350(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_start_pipe_2), .sr_out(sat_start_pipe_3));
	
	assign sat_rd_win_m5 = sat_start_pipe_1 | sat_start_pipe_2;
	
	assign sat_rd_win_m4 = sat_start_pipe_2 | sat_start_pipe_3;
	
	ym_cnt_bit_load #(.DATA_WIDTH(7)) cnfield_bit_trig1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(link_cnt_inc), .reset(link_cnt_rst), .load(link_cnt_load), .load_val(sat_link), .val(sat_link_cnt));
	
	assign link_cnt_load = reg_m5 & (link_ld_pipe | link_timing);
	
	ym_sr_bit sr352(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(link_timing), .sr_out(link_ld_pipe));
	
	assign link_cnt_inc = ~reg_m5 & link_timing;
	
	ym_sr_bit sr353(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_all_dly), .sr_out(link_timing));
	
	ym_sr_bit sr354(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(link_rst_comb), .sr_out(link_cnt_rst));
	
	assign link_rst_comb = reset_comb | active_end | (~link_rst_edge_0 & link_rst_edge_1);
	
	ym_sr_bit sr355(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(link_rst_edge_1), .sr_out(link_rst_edge_0));
	
	ym_sr_bit sr356(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(rd_gate), .sr_out(link_rst_edge_1));
	
	assign sat_start_trig = fetch_all_dly & (vdisp_en_trig | vcnt_at_max);
	
	ym_sr_bit sr357(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_start_trig), .sr_out(sat_start_latch));
	
	ym_sr_bit sr358(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_rd_pipe_3), .sr_out(sat_rd_pipe_4));
	
	ym_sr_bit sr359(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_rd_pipe_4), .sr_out(sat_rd_pipe_5));
	
	assign sat_wr_link = sat_wr_phase_b & sat_addr_bit1;
	assign sat_wr_size = sat_wr_phase_a & sat_addr_bit1;
	assign sat_wr_yhi = sat_wr_phase_a & ~sat_addr_bit1;
	assign sat_wr_ylo = sat_wr_phase_b & ~sat_addr_bit1;
	
	assign sat_h40_bit = reg_rs1 & vram_address[9];
	
	ym_sr_bit sr360_1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_address[1]), .sr_out(sat_addr_bit1));
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr360_83(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vram_address[8:3]), .data_out(sat_addr_mid));
	
	ym_sr_bit sr361(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsram_wr_test), .sr_out(sat_addr_pipe_0));
	
	ym_sr_bit sr362(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_h40_bit), .sr_out(sat_addr_pipe_1));
	
	assign sat_wr_active = sat_rd_pipe_4 | sat_wr_phase_a | sat_wr_phase_b | sat_addr_pipe_0;
	
	assign sat_wr_trig_a = sat_wr_match & vram_wr_hi_en;
	assign sat_wr_trig_b = sat_wr_match & vram_wr_lo_en;
	
	ym_sr_bit sr363(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_wr_trig_a), .sr_out(sat_wr_phase_a));
	
	ym_sr_bit sr364(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_wr_trig_b), .sr_out(sat_wr_phase_b));
	
	assign sat_index_mux = sat_wr_active ? { sat_addr_pipe_1, sat_addr_mid } : sat_link_cnt;
	
	assign sat_link_idx = reg_m5 ? sat_link_cnt : { 1'h1, sat_link_delay, odd_slot };
	
	ym_sr_bit_array #(.DATA_WIDTH(5), .SR_LENGTH(2)) sr365(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(sat_link_cnt[4:0]), .data_out(sat_link_delay));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr366(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vram_data[11:8]), .data_out(sat_size_serial));
	
	assign sprdata_wr_idx = reg_test0[12] ? reg_test_18[4:0] : tile_offset_cnt;
	
	ym_sr_bit sr367(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_row_done), .sr_out(tile_done_pipe));
	
	assign spr_cnt_full_det = tile_done_pipe & tile_offset_cnt[4] & tile_offset_cnt[2];
	
	ym_sr_bit sr368(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_cnt_full_det), .sr_out(spr_cnt_full_pipe));
	
	ym_sr_bit sr369(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_render_end), .sr_out(spr_render_end_pipe));
	
	ym_sr_bit sr370(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_idle), .sr_out(spr_idle_pipe));
	
	assign spr_render_end = reg_m5 & spr_idle_pipe & ~spr_idle;
	
	assign spr_idle = ~(spr_cnt_full_pipe | spr_hpos_valid);
	
	ym_cnt_bit_rev #(.DATA_WIDTH(5)) cnt371(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .c_in(tile_cnt_inc), .dec(tile_cnt_dec), .reset(tile_cnt_rst), .val(tile_offset_cnt));
	
	ym_sr_bit sr372(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_start), .sr_out(hblank_pipe));
	
	assign tile_cnt_rst_comb = disp_start | line_zero_dly;
	
	ym_sr_bit sr373(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_cnt_rst_comb), .sr_out(tile_cnt_rst));
	
	assign tile_cnt_en = ~reg_m5 | render_wr_pipe_1 | tile_row_done;
	
	assign tile_cnt_inc = tile_cnt_en & ~1'h0;
	
	assign tile_cnt_dec = tile_cnt_en & 1'h0;
	
	assign test_sel_ne2 = reg_test_18[6:5] != 2'h2;
	assign test_sel_ne1 = reg_test_18[6:5] != 2'h1;
	assign test_sel_ne0 = reg_test_18[6:5] != 2'h0;
	
	assign test_wr_attr = tst_fn7_wr & reg_test_18[6] & ~reg_test_18[5];
	
	assign sprdata_wr_attr = test_wr_attr | sprdata_wr_norm;
	
	assign test_wr_hpos = tst_fn7_wr & ~reg_test_18[6] & reg_test_18[5];
	
	assign test_wr_pat = tst_fn7_wr & ~reg_test_18[6] & ~reg_test_18[5];
	
	assign sprdata_wr_pat = test_wr_pat | sprdata_wr_norm;
	
	assign sprdata_wr_norm = render_wr_pipe_0 & ~reg_test0[12];
	
	ym_sr_bit sr374(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(render_trig_pipe), .sr_out(render_wr_pipe_0));
	
	ym_sr_bit sr375(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(render_wr_pipe_0), .sr_out(render_wr_pipe_1));
	
	ym_sr_bit #(.SR_LENGTH(10)) sr376(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_count_limit), .sr_out(spr_limit_m4_pipe));
	
	assign spr_limit_m4_n = ~(spr_limit_m4_pipe & ~reg_m5);
	
	assign sprdata_wr_hpos = test_wr_hpos | sprdata_wr_norm | ~sprdata_wr_ready;
	
	assign render_trig_comb = ~spr_count_limit & fetch_active_pipe;
	
	ym_sr_bit sr377(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(render_trig_comb), .sr_out(render_trig_pipe));
	
	assign spr_found_any = spr_found_full | render_active_pipe;
	
	ym_sr_bit sr378(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(render_active_pipe), .sr_out(spr_found_pipe));
	
	assign spr_attr_latch_en = hclk1 & spr_found_pipe & clk1;
	
	ym_slatch #(.DATA_WIDTH(4)) sl379(.MCLK(MCLK), .en(spr_attr_latch_en), .inp({sat_vs1_pipe, sat_vs0_pipe, sat_ybit8_pipe, sat_ysign_pipe}), .val(spr_size_latch));
	
	ym_slatch #(.DATA_WIDTH(6)) sl380(.MCLK(MCLK), .en(spr_attr_latch_en), .inp(y_delta_result[5:0]), .val(spr_yoff_latch));
	
	assign spr_found_full = spr_y_visible & spr_count_limit;
	
	assign spr_found_avail = spr_y_visible & ~spr_count_limit;
	
	assign spr_stop_or_ovfl = sat_stop_pipe | spr_overflow_pipe;
	
	ym_sr_bit sr381(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(active_end), .sr_out(line_start_pipe));
	
	ym7101_rs_trig rs40(.MCLK(MCLK), .set(spr_stop_or_ovfl), .rst(line_start_pipe), .q(spr_found_trig));
	
	ym7101_rs_trig rs41(.MCLK(MCLK), .set(spr_overflow_pipe), .rst(line_start_pipe), .q(spr_overflow_trig));
	
	assign yoff_carry_0 = vram_attr_hi[4] & (spr_size_latch[1] | spr_size_latch[0]);
	
	assign yoff_carry_1 = vram_attr_hi[4] & spr_size_latch[1] & ~spr_size_latch[0];
	
	assign yoff_carry_2 = vram_attr_hi[4] & spr_size_latch[1];
	
	assign yoff_size_hi = interlace_dblres ? spr_yoff_latch[5:4] : spr_yoff_latch[4:3];
	
	assign yoff_add_hi = yoff_size_hi + {1'h0, yoff_carry_1};
	
	ym_sr_bit sr382(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_or_ext_slot), .sr_out(render_active_pipe));
	
	ym_sr_bit sr383(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_avail_n), .sr_out(sat_avail_pipe));
	
	assign sat_avail_n = ~(spr_found_trig | ~sat_rd_window);
	
	ym_sr_bit sr384(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sat_stop), .sr_out(sat_stop_pipe));
	
	ym_sr_bit sr385(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_found_avail), .sr_out(spr_overflow_pipe));
	
	assign yoff_flip_lo = vram_attr_hi[4] ? ~spr_yoff_latch[3:0] : spr_yoff_latch[3:0];
	
	assign yoff_final_hi = interlace_dblres ? { yoff_xor_mid, yoff_flip_lo[3] } : { spr_yoff_latch[5], yoff_xor_mid };
	
	assign yoff_xor_mid = yoff_add_hi ^ { yoff_carry_2, yoff_carry_0 };
	
	assign yoff = { yoff_final_hi, yoff_flip_lo[2:0] };
	
	ym_slatch #(.DATA_WIDTH(8)) sl386(.MCLK(MCLK), .en(vram_strobe_1), .inp(vram_serial), .val(vram_attr_lo));
	
	ym_slatch #(.DATA_WIDTH(8)) sl387(.MCLK(MCLK), .en(vram_strobe_2), .inp(vram_serial), .val(vram_attr_hi));
	
	assign vram_attr_sel = attr_phase_0 ? vram_attr_hi : vram_attr_lo;
	
	assign spr_cnt_shift_en = spr_cnt_wr_en | tst_fn6_wr | tst_fn6_rd;
	
	ym_sr_bit_en #(.SR_LENGTH(10)) sr388(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spr_cnt_data[0]), .data_out(spr_cnt_sr_0));
	
	ym_sr_bit_en #(.SR_LENGTH(10)) sr389(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spr_cnt_data[1]), .data_out(spr_cnt_sr_1));
	
	ym_sr_bit_en #(.SR_LENGTH(10)) sr390(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spr_cnt_data[2]), .data_out(spr_cnt_sr_2));
	
	ym_sr_bit_en #(.SR_LENGTH(10)) sr391(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spr_cnt_data[3]), .data_out(spr_cnt_sr_3));
	
	assign spr_cnt_bit_0 = attr_phase_0 ? spr_cnt_sr_0[9] : spr_cnt_sr_0[8];
	
	assign spr_cnt_bit_1 = attr_phase_0 ? spr_cnt_sr_1[9] : spr_cnt_sr_1[8];
	
	assign spr_cnt_bit_2 = attr_phase_0 ? spr_cnt_sr_2[9] : spr_cnt_sr_2[8];
	
	assign spr_cnt_bit_3 = attr_phase_0 ? spr_cnt_sr_3[9] : spr_cnt_sr_3[8];
	
	assign spr_cnt_m4_bit = reg_81_b1 ? spr_cnt_bit_3 : vram_attr_sel[0];
	
	assign spr_cnt_wr_en = ~reg_test0[14] & spr_found_any;
	
	assign spr_cnt_data = tst_fn6_wr ? io_data[3:0] : y_delta_result[3:0];
	
	ym_slatch sl_86_b2(.MCLK(MCLK), .en(reg_wr_89), .inp(reg_data_l2[2]), .val(reg_86_b2));
	
	ym_slatch sl_86_b5(.MCLK(MCLK), .en(reg_wr_89), .inp(reg_data_l2[5]), .val(reg_86_b5));
	
	ym_slatch #(.DATA_WIDTH(8)) sl_at(.MCLK(MCLK), .en(reg_wr_8A), .inp(reg_data_l2[7:0]), .val(reg_at));
	
	wire [7:0] spr_at_1 = reg_at | { 7'h0, reg_rs1 };
	wire [7:0] spr_at_2 = vram_address[16:9] | { 7'h0, reg_rs1 };
	
	assign sat_addr_match = spr_at_1 == spr_at_2;
	
	assign sat_wr_match = sat_addr_match & ~vram_address[2] & reg_m5;
	
	assign m4_attr_phase = ~reg_m5 & attr_phase_any;
	
	assign spr_count_limit = (reg_m5 & reg_rs1 & spridx_sr_active[19])
		| (reg_m5 & ~reg_rs1 & spridx_sr_active[15])
		| (~reg_m5 & spridx_sr_active[7]);
	
	ym_sr_bit sr392(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(vram_strobe_lat_0), .sr_out(vram_strobe_pipe_0));
	
	assign vram_strobe_0 = vram_strobe_lat_0 & clk2;
	
	assign vram_strobe_1 = vram_strobe_pipe_0 & clk2;
	
	assign vram_strobe_2 = vram_strobe_pipe_1 & clk2;
	
	ym_sr_bit sr393(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(vram_strobe_lat_1), .sr_out(vram_strobe_pipe_1));
	
	ym_dlatch_1 dl394(.MCLK(MCLK), .c1(clk1), .inp(vram_gate_0), .nval(vram_strobe_lat_0));
	
	assign vram_gate_0 = ~(fetch_active_pipe & hclk1);
	
	assign vram_gate_1 = ~(m4_or_vram_slot & hclk1);
	
	ym_dlatch_1 dl395(.MCLK(MCLK), .c1(clk1), .inp(vram_gate_1), .nval(vram_strobe_lat_1));
	
	assign vram_strobe_3 = vram_strobe_lat_1 & clk2;
	
	ym_sr_bit sr396(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_active_sel), .sr_out(fetch_active_pipe));
	
	ym_sr_bit sr397(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(m4_or_vram_slot), .sr_out(fetch_delay_pipe));
	
	assign fetch_active_sel = reg_m5 ? m4_or_vram_slot : fetch_delay_pipe;
	
	ym_sr_bit_array #(.DATA_WIDTH(7)) sr398(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(sat_link_idx), .data_out(sat_idx_pipe_0));
	
	ym_sr_bit_array #(.DATA_WIDTH(7)) sr399(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(sat_idx_pipe_0), .data_out(sat_idx_pipe_1));
	
	ym_sr_bit_array #(.DATA_WIDTH(7)) sr400(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(sat_idx_pipe_1), .data_out(sat_idx_pipe_2));
	
	assign attr_phase_any = attr_phase_1 | attr_phase_0;
	
	ym_sr_bit sr401(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(access_main_dly), .sr_out(attr_phase_0));
	
	ym_sr_bit sr402(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(m4_border_dly), .sr_out(attr_phase_1));
	
	assign spridx_data_mux = tst_fn6_wr ? io_data[10:4] : sat_idx_pipe_2;
	
	assign spridx_ovfl_mux = tst_fn6_wr ? io_data[11] : render_active_pipe;
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr403(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[0]), .data_out(spridx_sr_0));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr404(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[1]), .data_out(spridx_sr_1));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr405(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[2]), .data_out(spridx_sr_2));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr406(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[3]), .data_out(spridx_sr_3));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr407(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[4]), .data_out(spridx_sr_4));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr408(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[5]), .data_out(spridx_sr_5));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr409(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_data_mux[6]), .data_out(spridx_sr_6));
	
	ym_sr_bit_en #(.SR_LENGTH(20)) sr410(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .en1(spr_cnt_shift_en), .en2(~spr_cnt_shift_en), .data_in(spridx_ovfl_mux), .data_out(spridx_sr_active));
	
	ym_sr_bit sr411(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_all_dly), .sr_out(fetch_timing_0));
	
	ym_sr_bit sr412(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(access_ext_dly), .sr_out(fetch_timing_1));
	
	assign m4_fetch_active = ~reg_m5 & (fetch_timing_0 | fetch_timing_1);
	
	assign m4_fetch_phase = ~reg_m5 & fetch_timing_1;
	
	assign m5_fetch_phase = reg_m5 & fetch_timing_1;
	
	assign spridx_readback = reg_rs1 ?
		{ spridx_sr_6[19], spridx_sr_5[19], spridx_sr_4[19], spridx_sr_3[19], spridx_sr_2[19], spridx_sr_1[19], spridx_sr_0[19] } :
		{ reg_at[0], spridx_sr_5[15], spridx_sr_4[15], spridx_sr_3[15], spridx_sr_2[15], spridx_sr_1[15], spridx_sr_0[15] };
	
	assign sprdata_in_hflip = (sprdata_test_sel ? io_data[0] : 1'h0) | (sprdata_norm_sel ? vram_attr_hi[3] : 1'h0);
	
	assign sprdata_test_sel = sprdata_wr_ready & reg_test0[12];
	
	assign sprdata_norm_sel = sprdata_wr_ready & ~reg_test0[12];
	
	assign sprdata_in_pal = (sprdata_test_sel ? io_data[2:1] : 2'h0) | (sprdata_norm_sel ? vram_attr_hi[6:5] : 2'h0);
	
	assign sprdata_in_pri = (sprdata_test_sel ? io_data[3] : 1'h0) | (sprdata_norm_sel ? vram_attr_hi[7] : 1'h0);
	
	assign sprdata_in_xs = (sprdata_test_sel ? io_data[5:4] : 2'h0) | (sprdata_norm_sel ? spr_size_latch[3:2] : 2'h0);
	
	assign sprdata_in_ys = (sprdata_test_sel ? io_data[7:6] : 2'h0) | (sprdata_norm_sel ? spr_size_latch[1:0] : 2'h0);
	
	assign sprdata_in_yoff = (sprdata_test_sel ? io_data[13:8] : 6'h0) | (sprdata_norm_sel ? yoff : 6'h0);
	
	assign sprdata_rd_strobe = hclk1 & (tst_fn7_rd | xtile_done);
	
	assign sprdata_test_mux = (~test_sel_ne2 ? {spr_rd_yoff[2:0], spr_rd_ys, spr_rd_xs, spr_rd_pri, spr_rd_pal, spr_rd_hflip } : 11'h0) |
		(~test_sel_ne1 ? spr_rd_hpos : 11'h0) |
		(~test_sel_ne0 ? spr_rd_pattern : 11'h0);
	
	ym_slatch sl413(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_hflip_o), .val(spr_rd_hflip));
	ym_slatch #(.DATA_WIDTH(2)) sl414(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_pal_o), .val(spr_rd_pal));
	ym_slatch sl415(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_priority_o), .val(spr_rd_pri));
	ym_slatch #(.DATA_WIDTH(2)) sl416(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_xs_o), .val(spr_rd_xs));
	ym_slatch #(.DATA_WIDTH(2)) sl417(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_ys_o), .val(spr_rd_ys));
	ym_slatch #(.DATA_WIDTH(6)) sl418(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_yoffset_o), .val(spr_rd_yoff));
	
	ym_sr_bit sr419(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_phase), .sr_out(tile_fetch_pipe));
	
	ym_cnt_bit_load #(.DATA_WIDTH(2)) cnt420(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(access_main_dly), .reset(hblank_pipe), .load(xtile_done), .load_val(~sprdata_xs_o), .val(xtile_cnt));
	
	assign xtile_cnt_zero = xtile_cnt == 2'h0;
	
	assign xtile_done = xtile_cnt_zero & access_main_dly;
	
	ym_cnt_bit cnt421(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(tile_fetch_pipe), .reset(disp_start), .val(spr_line_cnt));
	
	ym_sr_bit sr422(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_line_cnt), .sr_out(spr_line_cnt_pipe));
	
	ym7101_rs_trig rs42(.MCLK(MCLK), .set(hblank_pipe), .rst(spr_render_end_pipe), .q(spr_render_active));
	
	ym_sr_bit sr423(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_render_active), .sr_out(spr_render_pipe));
	
	assign tile_data_valid = spr_render_pipe & fetch_phase & spr_limit_m4_n;
	
	assign sprdata_in_pat =
		(sprdata_test_sel ? io_data[10:0] : 11'h0) |
		(sprdata_norm_sel ? { vram_attr_hi[2:0], vram_serial_a } : 11'h0);
	
	ym_slatch #(.DATA_WIDTH(11)) sl424(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_pattern_o), .val(spr_rd_pattern));
	
	assign sprdata_in_hpos =
		(sprdata_test_sel ? io_data[8:0] : 9'h0) |
		(sprdata_norm_sel ? { vram_attr_lo[0], vram_serial_b } : 9'h0);
	
	ym_slatch #(.DATA_WIDTH(9)) sl425(.MCLK(MCLK), .en(sprdata_rd_strobe), .inp(sprdata_hpos_o), .val(spr_rd_hpos));
	
	ym_sr_bit sr426(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sprdata_ready_comb), .sr_out(sprdata_wr_ready));
	
	assign sprdata_ready_comb = ~(~reg_m5 | xtile_done);
	
	assign tile_start_comb = hblank_pipe | tile_row_done;
	
	ym_sr_bit sr427(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_start_comb), .sr_out(tile_start_pipe));
	
	assign yoff_size_sel = { 2'h0,interlace_dblres ? spr_rd_yoff[5:4] : spr_rd_yoff[4:3] };
	
	assign yoff_size_add = yoff_size_sel + yoff_accum;
	
	assign pattern_addr = spr_rd_pattern + { 7'h0, yoff_size_add };
	
	ym_sr_bit sr428(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(m5_fetch_comb), .sr_out(m5_fetch_pipe));
	
	assign m5_fetch_comb = access_main_dly & reg_m5;
	
	assign spr_hpos_nonzero = spr_rd_hpos != 9'h0;
	
	ym_dlatch_2 dl429(.MCLK(MCLK), .c2(hclk2), .inp(spr_hpos_nonzero), .nval(spr_hpos_valid));
	
	ym_sr_bit sr430(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_pipe_0), .sr_out(fetch_done_pipe));
	
	assign yoff_rst_comb = hblank_pipe | xtile_done;
	
	ym_sr_bit sr431(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(access_main_dly), .sr_out(fetch_pipe_0));
	
	assign tile_row_done = fetch_pipe_0 & xtile_cnt_zero;
	
	ym_dlatch_1 dl432(.MCLK(MCLK), .c1(hclk1), .inp(yoff_rst_comb), .nval(yoff_rst_latch));
	
	ym_dlatch_1 dl433(.MCLK(MCLK), .c1(hclk1), .inp(fetch_done_pipe), .val(yoff_active));
	
	assign yoff_ys_mux = yoff_active ? spr_rd_ys : 2'h0;
	
	assign yoff_accum = yoff_rst_latch ? yoff_add_val : 4'h0;
	
	assign yoff_add_val = {3'h0, yoff_active} + yoff_accum_pipe + { 2'h0, yoff_ys_mux };
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr434(.MCLK(MCLK), .c1(hclk2), .c2(hclk1), .data_in(yoff_accum), .data_out(yoff_accum_pipe));
	
	ym_slatch #(.DATA_WIDTH(8)) sl435(.MCLK(MCLK), .en(vram_strobe_3), .inp(vram_serial), .val(vram_serial_a));
	
	ym_slatch #(.DATA_WIDTH(8)) sl436(.MCLK(MCLK), .en(vram_strobe_0), .inp(vram_serial), .val(vram_serial_b));
	
	ym_slatch #(.DATA_WIDTH(8)) sl437(.MCLK(MCLK), .en(tile_data_strobe), .inp(pattern_serial_sel), .val(m4_pattern_data));
	
	assign pattern_serial_sel = spr_line_cnt_pipe ? vram_serial_b : vram_serial_a;
	
	ym_sr_bit sr438(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_data_valid), .sr_out(tile_data_strobe));
	
	ym_dlatch_2 dl439_1(.MCLK(MCLK), .c2(hclk2), .inp(spr_rd_hflip), .val(hflip_delay_1));
	ym_sr_bit #(.SR_LENGTH(5)) sr439(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hflip_delay_1), .sr_out(hflip_delay));
	
	ym_dlatch_2 #(.DATA_WIDTH(2)) dl440_1(.MCLK(MCLK), .c2(hclk2), .inp(spr_rd_pal), .val(pal_delay_1));
	ym_sr_bit_array #(.SR_LENGTH(5), .DATA_WIDTH(2)) sr440(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(pal_delay_1), .data_out(pal_delay));
	
	ym_dlatch_2 dl441_1(.MCLK(MCLK), .c2(hclk2), .inp(spr_rd_pri), .val(pri_delay_1));
	ym_sr_bit #(.SR_LENGTH(5)) sr441(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(pri_delay_1), .sr_out(pri_delay));
	
	ym_dlatch_2 #(.DATA_WIDTH(2)) dl442_1(.MCLK(MCLK), .c2(hclk2), .inp(spr_rd_xs), .val(xs_delay_1));
	ym_sr_bit_array #(.SR_LENGTH(5), .DATA_WIDTH(2)) sr442(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(xs_delay_1), .data_out(xs_delay));
	
	ym_dlatch_2 #(.DATA_WIDTH(9)) dl443_1(.MCLK(MCLK), .c2(hclk2), .inp(spr_rd_hpos), .val(hpos_delay_1));
	ym_sr_bit_array #(.SR_LENGTH(5), .DATA_WIDTH(9)) sr443(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(hpos_delay_1), .data_out(hpos_delay));
	
	ym_sr_bit sr444(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(tile_group_strobe), .sr_out(xaddr_load_pipe));
	
	assign xaddr_idle = ~(xaddr_load_pipe | xaddr_done_pipe);
	
	assign xaddr_dec = xaddr_idle & xpos_hflip_pipe;
	
	ym_sr_bit sr445(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_done_latch), .sr_out(xaddr_done_pipe));
	
	assign xaddr_adj_0 = xaddr_xs0_m5 | m4_xflip;
	
	assign xaddr_xs0_m5 = xs_delay[0] & hflip_m5;
	
	assign xaddr_xs1_m5 = xs_delay[1] & hflip_m5;
	
	assign xaddr_adj_1 = xaddr_xs1_m5 | m4_xflip;
	
	assign hflip_m5 = hflip_delay & reg_m5;
	
	assign m4_xflip = ~reg_m5 & reg_80_b3;
	
	assign xaddr_sign = reg_m5 | m4_xflip;
	
	assign xaddr_inc = xaddr_idle & ~xpos_hflip_pipe;
	
	assign xpos_mux = reg_m5 ? hpos_delay : { 1'h0, m4_pattern_data };
	
	assign xaddr_init = xpos_mux[8:3] + {5'h0, hflip_m5} + { xaddr_sign, xaddr_sign, m4_xflip, m4_xflip, xaddr_adj_1, xaddr_adj_0 };
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr446(.MCLK(MCLK), .c1(clk1), .c2(clk2), .data_in(xaddr_init), .data_out(xaddr_pipe_a));
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr447(.MCLK(MCLK), .c1(clk1), .c2(clk2), .data_in(xaddr_accum), .data_out(xaddr_pipe_b));
	
	assign xaddr_sel = xaddr_load_pipe ? xaddr_pipe_a : xaddr_pipe_b;
	
	assign xaddr_accum = xaddr_sel + {5'h0, xaddr_inc} + {6{xaddr_dec}};
	
	assign pixel_stage_strobe = clk2 & pixel_done;
	
	ym_slatch sl448(.MCLK(MCLK), .en(tile_group_strobe), .inp(hflip_m5), .val(xpos_hflip));
	
	ym_sr_bit sr449(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(xpos_hflip), .sr_out(xpos_hflip_pipe));
	
	ym_slatch sl450(.MCLK(MCLK), .en(tile_group_strobe), .inp(pri_delay), .val(xpos_pri_latch));
	
	ym_slatch sl451(.MCLK(MCLK), .en(pixel_stage_strobe), .inp(xpos_pri_latch), .val(xpos_pri));
	
	ym_slatch #(.DATA_WIDTH(2)) sl452(.MCLK(MCLK), .en(tile_group_strobe), .inp(pal_delay), .val(xpos_pal_latch));
	
	ym_slatch #(.DATA_WIDTH(2)) sl453(.MCLK(MCLK), .en(pixel_stage_strobe), .inp(xpos_pal_latch), .val(xpos_pal));
	
	ym_dlatch_2 dl454(.MCLK(MCLK), .c2(clk2), .inp(pixel_done), .nval(pixel_done_latch));
	
	ym_sr_bit sr455(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_data_strobe), .sr_out(tile_strobe_pipe_0));
	
	ym_sr_bit sr456(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(tile_strobe_pipe_0), .sr_out(tile_strobe_pipe_1));
	
	assign tile_group_done = tile_strobe_pipe_1 & tile_group_cnt_zero;
	
	assign tile_group_strobe = tile_strobe_pipe_1 & tile_group_cnt_zero & hclk1;
	
	assign xsize_init_lo = ~(reg_m5 & xs_delay[0]);
	
	ym_cnt_bit_load #(.DATA_WIDTH(2)) cnt457(.MCLK(MCLK), .c1(hclk1), .c2(hclk2),
		.c_in(tile_strobe_pipe_1), .reset(disp_start), .load(tile_group_done), .load_val( { xsize_init_hi, xsize_init_lo } ), .val(tile_group_cnt));
	
	assign xsize_init_hi = ~(reg_m5 & xs_delay[1]);
	
	assign tile_group_cnt_zero = tile_group_cnt == 2'h0;
	
	assign disp_x_offset = hcnt + 9'h1 + { 4'hf, ~reg_m5, reg_m5, 2'h2, reg_m5 };
	
	ym_sr_bit sr458(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(xaddr_in_bounds), .sr_out(xbounds_pipe));
	
	assign xaddr_in_bounds = ~((xaddr_accum[5] & reg_rs1 & (xaddr_accum[4] | xaddr_accum[3]))
		| (xaddr_accum[5] & ~reg_rs1));
	
	assign lb_pri_out = xpos_pri & disp_active;
	
	assign lb_pal_out = disp_active ? xpos_pal : 2'h0;
	
	ym_sr_bit sr459(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(~pixel_done_latch), .sr_out(pixel_active_pipe_0));
	
	ym_sr_bit sr460(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_active_pipe_0), .sr_out(pixel_active_pipe_1));
	
	assign pixel_dir_xor = pixel_active_pipe_1 ^ xpos_hflip_pipe;
	
	ym_dlatch_1 dl461(.MCLK(MCLK), .c1(clk1), .inp(pixel_dir_xor), .val(pixel_direction));
	
	assign lb_write_valid = xbounds_pipe & (~pixel_done_latch | pixel_active_pipe_1);
	
	ym_dlatch_1 dl462(.MCLK(MCLK), .c1(clk1), .inp(lb_write_valid), .nval(lb_write_valid_n));
	
	assign lb_write_en = ~(lb_write_valid_n | reg_test0[13]);
	
	ym_sr_bit sr463(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(lb_write_valid), .sr_out(lb_write_pipe));
	
	assign lb_write_window = lb_write_valid | lb_write_pipe;
	
	ym_sr_bit sr464(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(lb_clear), .sr_out(lb_clear_pipe));
	
	assign lb_write_any = lb_clear_pipe | lb_write_window | lb_clear | reg_test0[13];
	
	assign lb_read_strobe_n = ~(lb_read_latch & ~reg_test0[13]);
	
	ym_dlatch_1 dl465(.MCLK(MCLK), .c1(clk1), .inp(lb_read_gate), .nval(lb_read_latch));
	
	assign lb_read_gate = ~(hclk2 & lb_read_active_n);
	
	ym_dlatch_1 dl466(.MCLK(MCLK), .c1(hclk1), .inp(lb_read_active_comb), .nval(lb_read_active_n));
	
	assign lb_read_active_comb = disp_active | disp_x_bit_2 | disp_x_bit_1 | disp_x_bit_0;
	
	ym_sr_bit sr467(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_x_offset[2]), .sr_out(disp_x_bit_2));
	
	ym_sr_bit sr468(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_x_offset[1]), .sr_out(disp_x_bit_1));
	
	ym_sr_bit sr469(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_x_offset[0]), .sr_out(disp_x_bit_0));
	
	assign disp_active = ~(reg_m5 ? disp_active_pipe : hv_disp_active);
	
	ym_sr_bit sr470(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_active_latch), .sr_out(disp_active_pipe));
	
	ym_sr_bit sr471(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hv_disp_active), .sr_out(disp_active_latch));
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr472(.MCLK(MCLK), .c1(clk1), .c2(clk2), .data_in(pixel_col_latch), .data_out(pixel_col_pipe));
	
	ym_slatch #(.DATA_WIDTH(3)) sl473(.MCLK(MCLK), .en(tile_group_strobe), .inp(xpos_mux[2:0]), .val(pixel_col_latch));
	
	assign lb_sel_spr = disp_active & ~reg_test0[13];
	
	assign lb_sel_disp = ~disp_active & ~reg_test0[13];
	
	assign lb_sel_test = reg_test0[13];
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr474(.MCLK(MCLK), .c1(clk1), .c2(clk2), .data_in(xaddr_accum), .data_out(xaddr_disp_pipe));
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr475(.MCLK(MCLK), .c1(clk1), .c2(clk2), .data_in(disp_x_offset[8:3]), .data_out(disp_addr_pipe));
	
	assign lb_addr_mux =
		(lb_sel_spr ? xaddr_disp_pipe : 6'h0) |
		(lb_sel_disp ? disp_addr_pipe : 6'h0) |
		(lb_sel_test ? reg_test_18[5:0] : 6'h0);
	
	ym_slatch #(.DATA_WIDTH(8)) sl478(.MCLK(MCLK), .en(tile_strobe_p3), .inp(vram_serial), .val(tile_plane3_a));
	
	ym_slatch #(.DATA_WIDTH(8)) sl479(.MCLK(MCLK), .en(tile_strobe_p2), .inp(vram_serial), .val(tile_plane2_a));
	
	ym_slatch #(.DATA_WIDTH(8)) sl480(.MCLK(MCLK), .en(tile_strobe_p0), .inp(vram_serial), .val(tile_plane0_a));
	
	ym_slatch #(.DATA_WIDTH(8)) sl481(.MCLK(MCLK), .en(tile_strobe_p1), .inp(vram_serial), .val(tile_plane1_a));
	
	assign tile_strobe_p0 = tile_strobe_p0_pipe & clk2;
	
	ym_sr_bit sr482(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(tile_strobe_lat_0), .sr_out(tile_strobe_p0_pipe));
	
	ym_dlatch_1 dl483(.MCLK(MCLK), .c1(clk1), .inp(tile_gate_0), .nval(tile_strobe_lat_0));
	
	assign tile_strobe_p1 = tile_strobe_lat_0 & clk2;
	
	assign tile_gate_0 = ~(fetch_pipe_3 & hclk1);
	
	assign tile_gate_1 = ~(fetch_phase & hclk1);
	
	ym_dlatch_1 dl484(.MCLK(MCLK), .c1(clk1), .inp(tile_gate_1), .nval(tile_strobe_lat_1));
	
	assign tile_strobe_p2 = tile_strobe_lat_1 & clk2;
	
	ym_sr_bit sr485(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(tile_strobe_lat_1), .sr_out(tile_strobe_p2_pipe));
	
	assign tile_strobe_p3 = tile_strobe_p2_pipe & clk2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl486(.MCLK(MCLK), .en(tile_strobe_p3), .inp(tile_plane2_a), .val(tile_plane2_b));
	
	ym_slatch #(.DATA_WIDTH(8)) sl487(.MCLK(MCLK), .en(tile_strobe_p3), .inp(tile_plane0_a), .val(tile_plane0_b));
	
	ym_slatch #(.DATA_WIDTH(8)) sl488(.MCLK(MCLK), .en(tile_strobe_p3), .inp(tile_plane1_a), .val(tile_plane1_b));
	
	ym_sr_bit sr489(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(tile_strobe_p2_pipe), .sr_out(tile_pipe_0));
	
	ym_sr_bit sr490(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(tile_pipe_0), .sr_out(tile_pipe_1));
	
	assign tile_stage_strobe = tile_pipe_1 & clk2;
	
	ym_slatch #(.DATA_WIDTH(8)) sl491(.MCLK(MCLK), .en(tile_stage_strobe), .inp(tile_plane3_a), .val(tile_plane3));
	
	ym_slatch #(.DATA_WIDTH(8)) sl492(.MCLK(MCLK), .en(tile_stage_strobe), .inp(tile_plane2_b), .val(tile_plane2));
	
	ym_slatch #(.DATA_WIDTH(8)) sl493(.MCLK(MCLK), .en(tile_stage_strobe), .inp(tile_plane0_b), .val(tile_plane0));
	
	ym_slatch #(.DATA_WIDTH(8)) sl494(.MCLK(MCLK), .en(tile_stage_strobe), .inp(tile_plane1_b), .val(tile_plane1));
	
	ym_sr_bit sr495(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_start_n), .sr_out(pixel_bit_0));
	
	ym_sr_bit sr496(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_bit_0), .sr_out(pixel_bit_1));

	ym_sr_bit sr497(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_bit_1), .sr_out(pixel_bit_2));
	
	ym_sr_bit sr498(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_bit_2), .sr_out(pixel_bit_3));
	
	ym_dlatch_1 dl499(.MCLK(MCLK), .c1(clk1), .inp(pixel_bit_3), .nval(pixel_done));
	
	ym_dlatch_1 dl500(.MCLK(MCLK), .c1(hclk1), .inp(tile_data_strobe), .val(tile_strobe_hold));
	
	assign pixel_start_n = ~(tile_strobe_hold & hclk2);
	
	ym_sr_bit sr501(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(hflip_m5), .sr_out(hflip_serial_pipe));
	
	assign pixel_sel_0 = hflip_serial_pipe ? pixel_bit_0 : pixel_bit_3;
	
	assign pixel_sel_1 = hflip_serial_pipe ? pixel_bit_1 : pixel_bit_2;
	
	assign pixel_sel_2 = hflip_serial_pipe ? pixel_bit_2 : pixel_bit_1;
	
	assign pixel_sel_3 = hflip_serial_pipe ? pixel_bit_3 : pixel_bit_0;
	
	assign pixel_mode_m5 = ~(~reg_m5 | disp_gate_delay);
	
	assign pixel_mode_m4 = ~(reg_m5 | disp_gate_delay);
	
	wire [7:0] pixel_serial_m5 =
		(~pixel_sel_0 ? tile_plane2 : 8'h0) |
		(~pixel_sel_1 ? tile_plane3 : 8'h0) |
		(~pixel_sel_2 ? tile_plane1 : 8'h0) |
		(~pixel_sel_3 ? tile_plane0 : 8'h0);
	
	wire [7:0] pixel_serial_m4 =
		(~pixel_bit_0 ? { tile_plane3[7], tile_plane2[7], tile_plane0[7], tile_plane1[7], tile_plane3[6], tile_plane2[6], tile_plane0[6], tile_plane1[6] } : 8'h0) |
		(~pixel_bit_1 ? { tile_plane3[5], tile_plane2[5], tile_plane0[5], tile_plane1[5], tile_plane3[4], tile_plane2[4], tile_plane0[4], tile_plane1[4] } : 8'h0) |
		(~pixel_bit_2 ? { tile_plane3[3], tile_plane2[3], tile_plane0[3], tile_plane1[3], tile_plane3[2], tile_plane2[2], tile_plane0[2], tile_plane1[2] } : 8'h0) |
		(~pixel_bit_3 ? { tile_plane3[1], tile_plane2[1], tile_plane0[1], tile_plane1[1], tile_plane3[0], tile_plane2[0], tile_plane0[0], tile_plane1[0] } : 8'h0);
	
	assign pixel_serial =
		(pixel_mode_m5 ? pixel_serial_m5 : 8'h0) |
		(pixel_mode_m4 ? pixel_serial_m4 : 8'h0);
	
	assign pixel_hflip = pixel_nibble_swap ? { pixel_serial[3:0], pixel_serial[7:4] } : pixel_serial;
	
	ym_dlatch_1 #(.DATA_WIDTH(8)) dl502(.MCLK(MCLK), .c1(clk1), .inp(pixel_hflip), .val(pixel_data_latch));
	
	assign pixel_data_mux = reg_test0[13] ?
		{ io_data[14], io_data[13], io_data[12], io_data[11], io_data[6], io_data[5], io_data[4], io_data[3] } :
		pixel_data_latch;
	
	ym_cnt_bit_load #(.DATA_WIDTH(2)) cnt503(.MCLK(MCLK), .c1(clk1), .c2(clk2),
		.c_in(~tile_col_load), .reset(1'h0), .load(tile_col_load), .load_val(xpos_mux[2:1]), .val(tile_col_cnt));
	
	assign tile_col_load = ~pixel_start_n;
	
	ym_dlatch_1 #(.DATA_WIDTH(3)) dl504(.MCLK(MCLK), .c1(clk1), .inp({ tile_col_cnt, pixel_col_bit0_pipe }), .val(pixel_col));
	
	ym_slatch sl505(.MCLK(MCLK), .en(tile_col_load), .inp(xpos_mux[0]), .val(pixel_col_bit0));
	
	ym_sr_bit sr506(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(pixel_col_bit0), .sr_out(pixel_col_bit0_pipe));
	
	assign pixel_nibble_swap = pixel_col_bit0_pipe ^ hflip_serial_pipe;
	
	assign fetch_start_comb = m4_window_dly | (reg_m5 & access_main_dly);
	
	ym_sr_bit sr507(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_start_comb), .sr_out(fetch_pipe_1));
	
	ym_sr_bit sr508(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_pipe_1), .sr_out(fetch_pipe_2));
	
	ym_sr_bit sr509(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_pipe_2), .sr_out(fetch_pipe_3));
	
	ym_sr_bit sr510(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_pipe_3), .sr_out(fetch_pipe_4));
	
	assign fetch_phase_sel = reg_m5 ? fetch_pipe_3 : fetch_pipe_4;
	
	ym_sr_bit sr511(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(fetch_phase_sel), .sr_out(fetch_phase));
	
	ym_dlatch_1 dl512(.MCLK(MCLK), .c1(hclk1), .inp(hv_disp_active), .nval(disp_gate_latch));
	
	ym_sr_bit sr513(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(disp_gate_latch), .sr_out(disp_gate_pipe));
	
	assign disp_gate_edge = disp_gate_pipe & ~disp_gate_latch;
	
	ym_sr_bit sr514(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(disp_gate_edge), .sr_out(disp_gate_delay));
	
	assign test_bank_0_n = ~(~reg_test_18[7] & ~reg_test_18[6] & tst_fn8_wr);
	assign test_bank_1_n = ~(~reg_test_18[7] & reg_test_18[6] & tst_fn8_wr);
	assign test_bank_2_n = ~(reg_test_18[7] & ~reg_test_18[6] & tst_fn8_wr);
	assign test_bank_3_n = ~(reg_test_18[7] & reg_test_18[6] & tst_fn8_wr);
	
	assign lb_wr_bank_0 = ~(test_bank_0_n & lb_read_strobe_n);
	assign lb_wr_bank_1 = ~(test_bank_1_n & lb_read_strobe_n);
	assign lb_wr_bank_2 = ~(test_bank_2_n & lb_read_strobe_n);
	assign lb_wr_bank_3 = ~(test_bank_3_n & lb_read_strobe_n);
	
	assign lb_rd_mask_n = ~(~reg_test0[13] & disp_gate_pipe_2);
	
	assign col_eq_0 = pixel_col == 3'h0;
	assign col_eq_1 = pixel_col == 3'h1;
	assign col_eq_2 = pixel_col == 3'h2;
	assign col_eq_3 = pixel_col == 3'h3;
	assign col_eq_4 = pixel_col == 3'h4;
	assign col_eq_5 = pixel_col == 3'h5;
	assign col_eq_6 = pixel_col == 3'h6;
	assign col_eq_7 = pixel_col == 3'h7;
	
	assign col_grp_1 = col_eq_1 | lb_rd_mask_n;
	assign col_grp_3 = col_eq_3 | lb_rd_mask_n;
	assign col_grp_5 = col_eq_5 | lb_rd_mask_n;
	assign col_grp_7 = col_eq_7 | lb_rd_mask_n;
	
	assign stage_strobe_0 = clk2 & (col_eq_0 | col_grp_7);
	assign stage_strobe_1 = clk2 & (col_grp_1 | col_eq_0);
	assign stage_strobe_2 = clk2 & (col_eq_2 | col_grp_1);
	assign stage_strobe_3 = clk2 & (col_grp_3 | col_eq_2);
	assign stage_strobe_4 = clk2 & (col_eq_4 | col_grp_3);
	assign stage_strobe_5 = clk2 & (col_grp_5 | col_eq_4);
	assign stage_strobe_6 = clk2 & (col_eq_6 | col_grp_5);
	assign stage_strobe_7 = clk2 & (col_grp_7 | col_eq_6);
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl515(.MCLK(MCLK), .en(stage_strobe_0), .inp(pixel_data_mux[7:4]), .val(stage_px_0));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl516(.MCLK(MCLK), .en(stage_strobe_1), .inp(pixel_data_mux[3:0]), .val(stage_px_1));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl517(.MCLK(MCLK), .en(stage_strobe_2), .inp(pixel_data_mux[7:4]), .val(stage_px_2));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl518(.MCLK(MCLK), .en(stage_strobe_3), .inp(pixel_data_mux[3:0]), .val(stage_px_3));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl519(.MCLK(MCLK), .en(stage_strobe_4), .inp(pixel_data_mux[7:4]), .val(stage_px_4));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl520(.MCLK(MCLK), .en(stage_strobe_5), .inp(pixel_data_mux[3:0]), .val(stage_px_5));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl521(.MCLK(MCLK), .en(stage_strobe_6), .inp(pixel_data_mux[7:4]), .val(stage_px_6));
	
	ym_slatch_t #(.DATA_WIDTH(4)) sl522(.MCLK(MCLK), .en(stage_strobe_7), .inp(pixel_data_mux[3:0]), .val(stage_px_7));
	
	assign lb_stage_strobe = clk2 & (lb_rd_mask_pipe | pixel_done);
	
	ym_slatch #(.DATA_WIDTH(4)) sl523(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_0), .val(lb_px_0));
	
	ym_slatch #(.DATA_WIDTH(4)) sl524(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_1), .val(lb_px_1));
	
	ym_slatch #(.DATA_WIDTH(4)) sl525(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_2), .val(lb_px_2));
	
	ym_slatch #(.DATA_WIDTH(4)) sl526(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_3), .val(lb_px_3));
	
	ym_slatch #(.DATA_WIDTH(4)) sl527(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_4), .val(lb_px_4));
	
	ym_slatch #(.DATA_WIDTH(4)) sl528(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_5), .val(lb_px_5));
	
	ym_slatch #(.DATA_WIDTH(4)) sl529(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_6), .val(lb_px_6));
	
	ym_slatch #(.DATA_WIDTH(4)) sl530(.MCLK(MCLK), .en(lb_stage_strobe), .inp(stage_px_7), .val(lb_px_7));
	
	assign slot_ge_1 = pixel_col_pipe >= 3'h1;
	assign slot_ge_2 = pixel_col_pipe >= 3'h2;
	assign slot_ge_3 = pixel_col_pipe >= 3'h3;
	assign slot_ge_4 = pixel_col_pipe >= 3'h4;
	assign slot_ge_5 = pixel_col_pipe >= 3'h5;
	assign slot_ge_6 = pixel_col_pipe >= 3'h6;
	assign slot_ge_7 = pixel_col_pipe >= 3'h7;
	
	ym_dlatch_1 dl531(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_1), .nval(slot_valid_0));
	
	ym_dlatch_1 dl532(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_2), .nval(slot_valid_1));
	
	ym_dlatch_1 dl533(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_3), .nval(slot_valid_2));
	
	ym_dlatch_1 dl534(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_4), .nval(slot_valid_3));
	
	ym_dlatch_1 dl535(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_5), .nval(slot_valid_4));
	
	ym_dlatch_1 dl536(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_6), .nval(slot_valid_5));
	
	ym_dlatch_1 dl537(.MCLK(MCLK), .c1(clk1), .inp(slot_ge_7), .nval(slot_valid_6));
	
	assign slot_sel_0 = slot_valid_0 ^ pixel_direction;
	assign slot_sel_1 = slot_valid_1 ^ pixel_direction;
	assign slot_sel_2 = slot_valid_2 ^ pixel_direction;
	assign slot_sel_3 = slot_valid_3 ^ pixel_direction;
	assign slot_sel_4 = slot_valid_4 ^ pixel_direction;
	assign slot_sel_5 = slot_valid_5 ^ pixel_direction;
	assign slot_sel_6 = slot_valid_6 ^ pixel_direction;
	assign slot_sel_7 = ~pixel_direction;
	
	assign slot_wr_0 = slot_sel_0 & lb_write_en;
	assign slot_wr_1 = slot_sel_1 & lb_write_en;
	assign slot_wr_2 = slot_sel_2 & lb_write_en;
	assign slot_wr_3 = slot_sel_3 & lb_write_en;
	assign slot_wr_4 = slot_sel_4 & lb_write_en;
	assign slot_wr_5 = slot_sel_5 & lb_write_en;
	assign slot_wr_6 = slot_sel_6 & lb_write_en;
	assign slot_wr_7 = slot_sel_7 & lb_write_en;
	
	assign lb_wr_en_0 = ~(lb_wr_bank_0 | (slot_wr_0 & lb_wr_cond_0));
	assign lb_wr_en_1 = ~(lb_wr_bank_0 | (slot_wr_1 & lb_wr_cond_1));
	assign lb_wr_en_2 = ~(lb_wr_bank_1 | (slot_wr_2 & lb_wr_cond_2));
	assign lb_wr_en_3 = ~(lb_wr_bank_1 | (slot_wr_3 & lb_wr_cond_3));
	assign lb_wr_en_4 = ~(lb_wr_bank_2 | (slot_wr_4 & lb_wr_cond_4));
	assign lb_wr_en_5 = ~(lb_wr_bank_2 | (slot_wr_5 & lb_wr_cond_5));
	assign lb_wr_en_6 = ~(lb_wr_bank_3 | (slot_wr_6 & lb_wr_cond_6));
	assign lb_wr_en_7 = ~(lb_wr_bank_3 | (slot_wr_7 & lb_wr_cond_7));
	
	ym_dlatch_2 dl538(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_0), .nval(lb_wr_latch_0));
	
	ym_dlatch_2 dl539(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_1), .nval(lb_wr_latch_1));
	
	ym_dlatch_2 dl540(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_2), .nval(lb_wr_latch_2));
	
	ym_dlatch_2 dl541(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_3), .nval(lb_wr_latch_3));
	
	ym_dlatch_2 dl542(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_4), .nval(lb_wr_latch_4));
	
	ym_dlatch_2 dl543(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_5), .nval(lb_wr_latch_5));
	
	ym_dlatch_2 dl544(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_6), .nval(lb_wr_latch_6));
	
	ym_dlatch_2 dl545(.MCLK(MCLK), .c2(clk2), .inp(lb_wr_en_7), .nval(lb_wr_latch_7));
	
	assign lb_wr_clk_0 = lb_wr_latch_0 & clk1;
	
	assign lb_wr_clk_1 = lb_wr_latch_1 & clk1;
	
	assign lb_wr_clk_2 = lb_wr_latch_2 & clk1;
	
	assign lb_wr_clk_3 = lb_wr_latch_3 & clk1;
	
	assign lb_wr_clk_4 = lb_wr_latch_4 & clk1;
	
	assign lb_wr_clk_5 = lb_wr_latch_5 & clk1;
	
	assign lb_wr_clk_6 = lb_wr_latch_6 & clk1;
	
	assign lb_wr_clk_7 = lb_wr_latch_7 & clk1;
	
	assign lb_idx_nz_0 = linebuffer_out_index[0] != 4'h0;
	assign lb_idx_nz_1 = linebuffer_out_index[1] != 4'h0;
	assign lb_idx_nz_2 = linebuffer_out_index[2] != 4'h0;
	assign lb_idx_nz_3 = linebuffer_out_index[3] != 4'h0;
	assign lb_idx_nz_4 = linebuffer_out_index[4] != 4'h0;
	assign lb_idx_nz_5 = linebuffer_out_index[5] != 4'h0;
	assign lb_idx_nz_6 = linebuffer_out_index[6] != 4'h0;
	assign lb_idx_nz_7 = linebuffer_out_index[7] != 4'h0;
	
	assign lb_collide_0 = lb_idx_nz_0 & slot_wr_0 & lb_px_nz_0;
	assign lb_collide_1 = lb_idx_nz_1 & slot_wr_1 & lb_px_nz_1;
	assign lb_collide_2 = lb_idx_nz_2 & slot_wr_2 & lb_px_nz_2;
	assign lb_collide_3 = lb_idx_nz_3 & slot_wr_3 & lb_px_nz_3;
	assign lb_collide_4 = lb_idx_nz_4 & slot_wr_4 & lb_px_nz_4;
	assign lb_collide_5 = lb_idx_nz_5 & slot_wr_5 & lb_px_nz_5;
	assign lb_collide_6 = lb_idx_nz_6 & slot_wr_6 & lb_px_nz_6;
	assign lb_collide_7 = lb_idx_nz_7 & slot_wr_7 & lb_px_nz_7;
	
	assign lb_px_nz_0 = lb_px_0 != 4'h0;
	assign lb_px_nz_1 = lb_px_1 != 4'h0;
	assign lb_px_nz_2 = lb_px_2 != 4'h0;
	assign lb_px_nz_3 = lb_px_3 != 4'h0;
	assign lb_px_nz_4 = lb_px_4 != 4'h0;
	assign lb_px_nz_5 = lb_px_5 != 4'h0;
	assign lb_px_nz_6 = lb_px_6 != 4'h0;
	assign lb_px_nz_7 = lb_px_7 != 4'h0;
	
	assign lb_wr_cond_0 = lb_wr_mode ? lb_px_nz_0 : ~lb_idx_nz_0;
	assign lb_wr_cond_1 = lb_wr_mode ? lb_px_nz_1 : ~lb_idx_nz_1;
	assign lb_wr_cond_2 = lb_wr_mode ? lb_px_nz_2 : ~lb_idx_nz_2;
	assign lb_wr_cond_3 = lb_wr_mode ? lb_px_nz_3 : ~lb_idx_nz_3;
	assign lb_wr_cond_4 = lb_wr_mode ? lb_px_nz_4 : ~lb_idx_nz_4;
	assign lb_wr_cond_5 = lb_wr_mode ? lb_px_nz_5 : ~lb_idx_nz_5;
	assign lb_wr_cond_6 = lb_wr_mode ? lb_px_nz_6 : ~lb_idx_nz_6;
	assign lb_wr_cond_7 = lb_wr_mode ? lb_px_nz_7 : ~lb_idx_nz_7;
	
	assign lb_wr_mode = 1'h0;
	
	assign lb_clear = ~lb_read_active_comb;
	
	assign test_idx_0 = reg_test_18[7:6] == 2'h0;
	assign test_idx_1 = reg_test_18[7:6] == 2'h1;
	assign test_idx_2 = reg_test_18[7:6] == 2'h2;
	assign test_idx_3 = reg_test_18[7:6] == 2'h3;
	
	assign test_rd_pal0_even =
		(test_idx_0 & linebuffer_out_pal[0][0]) |
		(test_idx_1 & linebuffer_out_pal[2][0]) |
		(test_idx_2 & linebuffer_out_pal[4][0]) |
		(test_idx_3 & linebuffer_out_pal[6][0]);
	
	assign test_rd_pal1_even =
		(test_idx_0 & linebuffer_out_pal[0][1]) |
		(test_idx_1 & linebuffer_out_pal[2][1]) |
		(test_idx_2 & linebuffer_out_pal[4][1]) |
		(test_idx_3 & linebuffer_out_pal[6][1]);
	
	assign test_rd_pri_even =
		(test_idx_0 & linebuffer_out_priority[0]) |
		(test_idx_1 & linebuffer_out_priority[2]) |
		(test_idx_2 & linebuffer_out_priority[4]) |
		(test_idx_3 & linebuffer_out_priority[6]);
	
	assign test_rd_idx0_even =
		(test_idx_0 & linebuffer_out_index[0][0]) |
		(test_idx_1 & linebuffer_out_index[2][0]) |
		(test_idx_2 & linebuffer_out_index[4][0]) |
		(test_idx_3 & linebuffer_out_index[6][0]);
	
	assign test_rd_idx1_even =
		(test_idx_0 & linebuffer_out_index[0][1]) |
		(test_idx_1 & linebuffer_out_index[2][1]) |
		(test_idx_2 & linebuffer_out_index[4][1]) |
		(test_idx_3 & linebuffer_out_index[6][1]);
	
	assign test_rd_idx2_even =
		(test_idx_0 & linebuffer_out_index[0][2]) |
		(test_idx_1 & linebuffer_out_index[2][2]) |
		(test_idx_2 & linebuffer_out_index[4][2]) |
		(test_idx_3 & linebuffer_out_index[6][2]);
	
	assign test_rd_idx3_even =
		(test_idx_0 & linebuffer_out_index[0][3]) |
		(test_idx_1 & linebuffer_out_index[2][3]) |
		(test_idx_2 & linebuffer_out_index[4][3]) |
		(test_idx_3 & linebuffer_out_index[6][3]);
	
	assign test_rd_pal0_odd =
		(test_idx_0 & linebuffer_out_pal[1][0]) |
		(test_idx_1 & linebuffer_out_pal[3][0]) |
		(test_idx_2 & linebuffer_out_pal[5][0]) |
		(test_idx_3 & linebuffer_out_pal[7][0]);
	
	assign test_rd_pal1_odd =
		(test_idx_0 & linebuffer_out_pal[1][1]) |
		(test_idx_1 & linebuffer_out_pal[3][1]) |
		(test_idx_2 & linebuffer_out_pal[5][1]) |
		(test_idx_3 & linebuffer_out_pal[7][1]);
	
	assign test_rd_pri_odd =
		(test_idx_0 & linebuffer_out_priority[1]) |
		(test_idx_1 & linebuffer_out_priority[3]) |
		(test_idx_2 & linebuffer_out_priority[5]) |
		(test_idx_3 & linebuffer_out_priority[7]);
	
	assign test_rd_idx0_odd =
		(test_idx_0 & linebuffer_out_index[1][0]) |
		(test_idx_1 & linebuffer_out_index[3][0]) |
		(test_idx_2 & linebuffer_out_index[5][0]) |
		(test_idx_3 & linebuffer_out_index[7][0]);
	
	assign test_rd_idx1_odd =
		(test_idx_0 & linebuffer_out_index[1][1]) |
		(test_idx_1 & linebuffer_out_index[3][1]) |
		(test_idx_2 & linebuffer_out_index[5][1]) |
		(test_idx_3 & linebuffer_out_index[7][1]);
	
	assign test_rd_idx2_odd =
		(test_idx_0 & linebuffer_out_index[1][2]) |
		(test_idx_1 & linebuffer_out_index[3][2]) |
		(test_idx_2 & linebuffer_out_index[5][2]) |
		(test_idx_3 & linebuffer_out_index[7][2]);
	
	assign test_rd_idx3_odd =
		(test_idx_0 & linebuffer_out_index[1][3]) |
		(test_idx_1 & linebuffer_out_index[3][3]) |
		(test_idx_2 & linebuffer_out_index[5][3]) |
		(test_idx_3 & linebuffer_out_index[7][3]);
	
	wire [7:0] load_val_pal0;
	wire [7:0] load_val_pal1;
	wire [7:0] load_val_priority;
	wire [7:0] load_val_index0;
	wire [7:0] load_val_index1;
	wire [7:0] load_val_index2;
	wire [7:0] load_val_index3;
	
	genvar gi;
	generate
		for (gi = 0; gi < 8; gi = gi + 1)
		begin : gl1
			assign load_val_pal0[gi] = linebuffer_out_pal[gi][0];
			assign load_val_pal1[gi] = linebuffer_out_pal[gi][1];
			assign load_val_priority[gi] = linebuffer_out_priority[gi];
			assign load_val_index0[gi] = linebuffer_out_index[gi][0];
			assign load_val_index1[gi] = linebuffer_out_index[gi][1];
			assign load_val_index2[gi] = linebuffer_out_index[gi][2];
			assign load_val_index3[gi] = linebuffer_out_index[gi][3];
		end
	endgenerate
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr546(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_pal0), .next(spr_pal[0]));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr547(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_pal1), .next(spr_pal[1]));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr548(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_priority), .next(spr_priority));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr549(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_index0), .next(spr_index[0]));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr550(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_index1), .next(spr_index[1]));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr551(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_index2), .next(spr_index[2]));
	
	ym_dbg_read #(.DATA_WIDTH(8)) sr552(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .prev(1'h0), .load(lb_clear),
		.load_val(load_val_index3), .next(spr_index[3]));
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr553(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_pal), .data_out(spr_pal_pipe_0));
	
	ym_sr_bit sr554(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_priority), .sr_out(spr_pri_pipe_0));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr555(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_index), .data_out(spr_idx_pipe_0));
	
	assign spr_pal_m5_sel = reg_m5 ? spr_pal_pipe_0 : spr_pal;
	
	assign spr_pri_m5_sel = reg_m5 ? spr_pri_pipe_0 : spr_priority;
	
	assign spr_idx_m5_sel = reg_m5 ? spr_idx_pipe_0 : spr_index;
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr556(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_pal_m5_sel), .data_out(spr_pal_pipe_1));
	
	ym_sr_bit sr557(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_pri_m5_sel), .sr_out(spr_pri_pipe_1));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr558(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_idx_m5_sel), .data_out(spr_idx_pipe_1));
	
	assign spr_out_pri = spr_pri_pipe_1 & reg_m5;
	
	assign spr_out_pal = reg_m5 ? spr_pal_pipe_1 : 2'h1;
	
	assign spr_pal_is_3 = spr_pal_pipe_1 == 2'h3;
	
	assign spr_idx_nonzero = spr_idx_pipe_1 != 4'h0;
	
	assign spr_idx_is_14 = spr_idx_pipe_1 == 4'he;

	assign spr_idx_is_15 = spr_idx_pipe_1 == 4'hf;
	
	ym_sr_bit_array #(.DATA_WIDTH(2)) sr559(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_out_pal), .data_out(spr_pal_pipe_2));
	
	ym_sr_bit sr560(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spr_out_pri), .sr_out(spr_pri_pipe_2));
	
	ym_sr_bit_array #(.DATA_WIDTH(4)) sr561(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(spr_idx_pipe_1), .data_out(spr_idx_pipe_2));
	
	ym_sr_bit sr562(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(lb_rd_mask_n), .sr_out(lb_rd_mask_pipe));
	
	ym_dlatch_1 dl563(.MCLK(MCLK), .c1(clk1), .inp(disp_gate_delay), .nval(disp_gate_pipe_2));
	
	assign lb_data_pri_hi = reg_test0[13] ? io_data[10] : lb_pri_out;
	
	assign lb_data_pal_hi = reg_test0[13] ? io_data[9:8] : lb_pal_out;
	
	assign lb_data_pri_lo = reg_test0[13] ? io_data[2] : lb_pri_out;
	
	assign lb_data_pal_lo = reg_test0[13] ? io_data[1:0] : lb_pal_out;
	
	ym_sr_bit sr600(.MCLK(MCLK), .c1(clk2), .c2(clk1), .bit_in(spr_collision_any), .sr_out(spr_collision_pipe));
	
	assign spr_collision_any = lb_collide_0 | lb_collide_1 | lb_collide_2 | lb_collide_3 | lb_collide_4 | lb_collide_5 | lb_collide_6 | lb_collide_7;
	
	assign spr_overflow_flag = spr_overflow_trig & active_end;
	
	// -------------------------------------------------------------------------
	// SAT cache (Sprite Attribute Table)
	// -------------------------------------------------------------------------
	// 80-entry × 21-bit on-chip cache of sprite attributes. Populated from
	// VRAM during vertical blanking. Each entry stores Y-position (10 bits),
	// size (4 bits), and link (7 bits).

	wire [6:0] sat_index = sat_index_mux;
	
	wire [20:0] sat_data_in;
	
	assign sat_data_in[6:0] = data_rd_pipe[6:0];
	assign sat_data_in[10:7] = sat_size_serial;
	assign sat_data_in[20:11] = { sat_size_serial[1:0], data_rd_pipe };
	
	assign sat_link = sat_out[6:0];
	assign sat_size = sat_out[10:7];
	assign sat_ypos = sat_out[20:11];
	
	always @(posedge MCLK)
	begin
		if (sat_index < 7'd80)
		begin
			if (hclk1) // write cycle
			begin
				if (sat_wr_link)
					sat[sat_index][6:0] <= sat_data_in[6:0];
				if (sat_wr_size)
					sat[sat_index][10:7] <= sat_data_in[10:7];
				if (sat_wr_yhi)
					sat[sat_index][20:19] <= sat_data_in[20:19];
				if (sat_wr_ylo)
					sat[sat_index][18:11] <= sat_data_in[18:11];
			end
			sat_out <= sat[sat_index];
			case (sat_index[1:0])
				2'h0: sat_out_0 <= sat[sat_index];
				2'h1: sat_out_1 <= sat[sat_index];
				2'h2: sat_out_2 <= sat[sat_index];
				2'h3: sat_out_3 <= sat[sat_index];
			endcase
		end
		else
		begin
			case (sat_index[1:0])
				2'h0: sat_out <= sat_out & sat_out_0;
				2'h1: sat_out <= sat_out & sat_out_1;
				2'h2: sat_out <= sat_out & sat_out_2;
				2'h3: sat_out <= sat_out & sat_out_3;
			endcase
		end
	end
	
	// -------------------------------------------------------------------------
	// Sprite render data buffer
	// -------------------------------------------------------------------------
	// 20-entry × 34-bit buffer holding the attributes of sprites visible on
	// the current scanline. Fields: pattern index (11), H-position (9),
	// H-flip (1), palette (2), priority (1), X-size (2), Y-size (2),
	// Y-offset (6).

	wire [4:0] sprdata_index = sprdata_wr_idx;
	
	wire [33:0] sprdata_in;
	
	assign sprdata_in[10:0] = sprdata_in_pat;
	assign sprdata_in[19:11] = sprdata_in_hpos;
	assign sprdata_in[33:20] = { sprdata_in_yoff, sprdata_in_ys, sprdata_in_xs, sprdata_in_pri, sprdata_in_pal, sprdata_in_hflip };
	
	assign sprdata_pattern_o = sprdata_out[10:0];
	assign sprdata_hpos_o = sprdata_out[19:11];
	assign sprdata_hflip_o = sprdata_out[20];
	assign sprdata_pal_o = sprdata_out[22:21];
	assign sprdata_priority_o = sprdata_out[23];
	assign sprdata_xs_o = sprdata_out[25:24];
	assign sprdata_ys_o = sprdata_out[27:26];
	assign sprdata_yoffset_o = sprdata_out[33:28];
	
	always @(posedge MCLK)
	begin
		if (sprdata_index < 5'd20)
		begin
			if (hclk1) // write cycle
			begin
				if (sprdata_wr_pat)
					sprdata[sprdata_index][10:0] <= sprdata_in[10:0];
				if (sprdata_wr_hpos)
					sprdata[sprdata_index][19:11] <= sprdata_in[19:11];
				if (sprdata_wr_attr)
					sprdata[sprdata_index][33:20] <= sprdata_in[33:20];
			end
			sprdata_out <= sprdata[sprdata_index];
			if (sprdata_index[0])
				sprdata_out_1 <= sprdata[sprdata_index];
			else
				sprdata_out_0 <= sprdata[sprdata_index];
		end
		else
		begin
			if (sprdata_index[0])
				sprdata_out <= sprdata_out & sprdata_out_1;
			else
				sprdata_out <= sprdata_out & sprdata_out_0;
		end
	end
	
	// -------------------------------------------------------------------------
	// Line buffer
	// -------------------------------------------------------------------------
	// 40-entry × 56-bit buffer (8 pixels packed per entry). Each pixel has
	// 4-bit index + 2-bit palette + 1-bit priority = 7 bits × 8 = 56 bits.
	// Written during sprite render, read during active display.

	wire [5:0] linebuffer_index = lb_addr_mux;
	
	wire [55:0] linebuffer_data_in;
	
	assign linebuffer_data_in[0] = lb_data_pri_lo;
	assign linebuffer_data_in[2:1] = lb_data_pal_lo;
	assign linebuffer_data_in[6:3] = lb_px_0;
	
	assign linebuffer_data_in[7] = lb_data_pri_hi;
	assign linebuffer_data_in[9:8] = lb_data_pal_hi;
	assign linebuffer_data_in[13:10] = lb_px_1;
	
	assign linebuffer_data_in[14] = lb_data_pri_lo;
	assign linebuffer_data_in[16:15] = lb_data_pal_lo;
	assign linebuffer_data_in[20:17] = lb_px_2;
	
	assign linebuffer_data_in[21] = lb_data_pri_hi;
	assign linebuffer_data_in[23:22] = lb_data_pal_hi;
	assign linebuffer_data_in[27:24] = lb_px_3;
	
	assign linebuffer_data_in[28] = lb_data_pri_lo;
	assign linebuffer_data_in[30:29] = lb_data_pal_lo;
	assign linebuffer_data_in[34:31] = lb_px_4;
	
	assign linebuffer_data_in[35] = lb_data_pri_hi;
	assign linebuffer_data_in[37:36] = lb_data_pal_hi;
	assign linebuffer_data_in[41:38] = lb_px_5;
	
	assign linebuffer_data_in[42] = lb_data_pri_lo;
	assign linebuffer_data_in[44:43] = lb_data_pal_lo;
	assign linebuffer_data_in[48:45] = lb_px_6;
	
	assign linebuffer_data_in[49] = lb_data_pri_hi;
	assign linebuffer_data_in[51:50] = lb_data_pal_hi;
	assign linebuffer_data_in[55:52] = lb_px_7;
	
	wire [55:0] linebuffer_out2 = lb_write_any ? linebuffer_out : ~56'h0;
	
	generate
		for (gi = 0; gi < 8; gi = gi + 1)
		begin : gl2
			assign linebuffer_out_priority[gi] = linebuffer_out2[gi*7];
			assign linebuffer_out_pal[gi] = linebuffer_out2[gi*7+2:gi*7+1];
			assign linebuffer_out_index[gi] = linebuffer_out2[gi*7+6:gi*7+3];
		end
	endgenerate
	
	always @(posedge MCLK)
	begin
		if (linebuffer_index < 6'd40)
		begin
			if (lb_write_any) // write cycle
			begin
				if (lb_wr_clk_0)
					linebuffer[linebuffer_index][6:0] <= linebuffer_data_in[6:0];
				if (lb_wr_clk_1)
					linebuffer[linebuffer_index][13:7] <= linebuffer_data_in[13:7];
				if (lb_wr_clk_2)
					linebuffer[linebuffer_index][20:14] <= linebuffer_data_in[20:14];
				if (lb_wr_clk_3)
					linebuffer[linebuffer_index][27:21] <= linebuffer_data_in[27:21];
				if (lb_wr_clk_4)
					linebuffer[linebuffer_index][34:28] <= linebuffer_data_in[34:28];
				if (lb_wr_clk_5)
					linebuffer[linebuffer_index][41:35] <= linebuffer_data_in[41:35];
				if (lb_wr_clk_6)
					linebuffer[linebuffer_index][48:42] <= linebuffer_data_in[48:42];
				if (lb_wr_clk_7)
					linebuffer[linebuffer_index][55:49] <= linebuffer_data_in[55:49];
			end
			linebuffer_out <= linebuffer[linebuffer_index];
			if (linebuffer_index[0])
				linebuffer_out_1 <= linebuffer[linebuffer_index];
			else
				linebuffer_out_0 <= linebuffer[linebuffer_index];
		end
		else
		begin
			if (linebuffer_index[0])
				linebuffer_out <= linebuffer_out & linebuffer_out_1;
			else
				linebuffer_out <= linebuffer_out & linebuffer_out_0;
		end
	end
	
	// -------------------------------------------------------------------------
	// VRAM interface
	// -------------------------------------------------------------------------
	// Read/write sequencer for the two 32K×8 DRAM chips. Generates RAS/CAS/WE
	// timing, captures serial data from the SD bus, and manages DRAM refresh
	// cycles. The VRAM address and data buses use a wired-AND val/pull pattern.

	ym_dlatch_1 dl564(.MCLK(MCLK), .c1(hclk1), .inp(odd_slot), .nval(vram_clk_latch));
	
	ym_sr_bit sr565(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(vram_clk_latch), .sr_out(vram_clk_pipe));
	
	ym_dlatch_1 dl566(.MCLK(MCLK), .c1(clk1), .inp(vram_clk_pipe), .nval(vram_clk_pipe_2));
	
	ym_dlatch_2 dl567(.MCLK(MCLK), .c2(clk2), .inp(vram_clk_pipe_2), .nval(vram_clk_pipe_3));
	
	wire vram_phase_delay = vram_phase_pipe; // FIXME
	
	assign vram_oe1_comb = (vram_clk_pipe & vram_oe_gate & vram_phase_lat_3)
		| (vram_active_latch & vram_phase_pipe)
		| (vram_active_latch & vram_phase_delay);
	
	assign vram_we0_comb = vram_access_pipe & vram_phase_lat_1 & vram_phase_lat_3;
	assign vram_we1_comb = vram_phase_lat_1 & vram_we_pipe & vram_phase_lat_3;
	
	assign vram_cas_comb = (vram_clk_pipe_2 & vram_cas_sel) | vram_phase_lat_3;
	assign vram_ras_comb = (vram_clk_pipe_2 & ~vram_cas_sel) | vram_phase_lat_2;
	
	assign vram_data_dir = (vram_refresh_comb & vram_phase_lat_3) | vram_phase_pipe | reg_test0[5];
	
	ym_sr_bit sr568(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_access_comb), .sr_out(vram_access_pipe));
	
	assign vram_access_comb = vram_wr_lo_gate | vram_access_delay;
	
	assign vram_refresh_comb = vram_refresh_latch | ~vram_refresh_pipe;
	
	ym_dlatch_2 dl569(.MCLK(MCLK), .c2(hclk2), .inp(vram_refresh_pipe), .nval(vram_refresh_latch));
	
	ym_dlatch_1 dl570(.MCLK(MCLK), .c1(hclk1), .inp(vram_refresh_trig), .nval(vram_refresh_pipe));
	
	ym_sr_bit sr571(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_lo_gate), .sr_out(vram_access_delay));
	
	assign vram_oe_gate = ~vram_refresh_comb & (vram_cycle_latch | ~vram_cycle_pipe);
	
	ym_dlatch_2 dl572(.MCLK(MCLK), .c2(hclk2), .inp(vram_cycle_pipe), .nval(vram_cycle_latch));
	
	ym_dlatch_1 dl573(.MCLK(MCLK), .c1(hclk1), .inp(vram_req_pipe_1), .nval(vram_cycle_pipe));
	
	ym_sr_bit sr574(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(slot_idle_dly), .sr_out(vram_req_pipe));
	
	ym_dlatch_1 dl575(.MCLK(MCLK), .c1(hclk1), .inp(vram_active_comb), .nval(vram_active_latch));
	
	ym_sr_bit sr576(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(~vram_clk_latch), .sr_out(vram_phase_pipe));
	
	ym_dlatch_1 dl577(.MCLK(MCLK), .c1(clk1), .inp(vram_phase_pipe), .nval(vram_phase_lat_1));
	
	ym_dlatch_2 dl578(.MCLK(MCLK), .c2(clk2), .inp(vram_phase_lat_1), .nval(vram_phase_lat_2));
	
	ym_dlatch_1 dl579(.MCLK(MCLK), .c1(clk1), .inp(~vram_phase_lat_2), .nval(vram_phase_lat_3));
	
	assign vram_addr_gate = ~((odd_slot & ~dma_copy_active & ~dma_ext_state) | (dma_copy_active & vram_wr_pipe));
	
	ym_dlatch_1 dl580(.MCLK(MCLK), .c1(hclk1), .inp(vram_addr_gate), .nval(vram_addr_latch));
	
	ym_sr_bit sr581(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_timing), .sr_out(vram_wr_pipe));
	
	assign vram_sd_sel = vram_clk_pipe;
	assign vram_addr_hi_sel = vram_phase_pipe & vram_clk_pipe_3; // addr high
	assign vram_addr_lo_sel = vram_phase_lat_2 & vram_phase_pipe; // addr low
	
	ym_sr_bit sr582(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_ext_copy), .sr_out(vram_wr_timing));
	
	assign vram_capture_strobe = vram_phase_lat_1 & vram_phase_lat_3;
	
	ym_sr_bit sr583(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_dma_comb), .sr_out(vram_dma_pipe));
	
	ym_sr_bit sr584(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_we_comb), .sr_out(vram_we_pipe));
	
	assign vram_we_comb = vram_wr_hi_gate | vram_we_delay;
	
	ym_sr_bit sr585(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_wr_hi_gate), .sr_out(vram_we_delay));
	
	assign vram_refresh_trig = vram_we_delay | vram_access_delay;
	
	ym_sr_bit sr586(.MCLK(MCLK), .c1(clk1), .c2(clk2), .bit_in(vram_se_comb), .sr_out(vram_serial_sel));
	
	ym_sr_bit sr587(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_timing_any), .sr_out(vram_cas_sel));
	
	ym_sr_bit sr588(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(edclk_dly), .sr_out(vram_timing_0));
	
	ym_sr_bit sr589(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_timing_0), .sr_out(vram_timing_1));
	
	ym_sr_bit sr590(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vram_req_pipe), .sr_out(vram_req_pipe_1));
	
	assign vram_active_comb = vram_req_pipe | vram_req_pipe_1 | vram_timing_any;
	
	assign vram_addr_strobe = odd_slot & hclk1;
	
	assign vram_data_strobe = hclk2 & vram_addr_latch;
	
	assign vram_dma_comb = odd_slot & vram_wr_pipe2;
	
	ym_sr_bit sr591(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(dma_copy_active), .sr_out(vram_128k_pipe));
	
	assign vram_wr_strobe = hclk1 & odd_slot;
	
	assign vram_sc_comb = clk1 & (~vram_128k | hclk2);
	
	assign vram_se_comb = vram_128k & hclk1;
	
	assign vram_timing_any = vram_timing_0 | vram_timing_1;
	
	assign vram_ys_mux = reg_test0[5] ? vram_address[16] : blank_out_pipe;
	
	assign vram_m5_bit1 = reg_m5 & vram_address[1];
	
	assign vram_addr_bit = vram_m5_bit1 | (~reg_m5 & vram_address[9]);
	
	assign vram_128k_bit = reg_8b_b4 ? reg_8b_b5 : vram_address[16];
	
	assign vram_row_addr = vram_128k ? // 128k
		{ vram_128k_bit, vram_address[15:10], vram_m5_bit1 } :
		{ vram_address[15:10], vram_addr_bit, vram_address[0] };
	
	assign vram_col_addr = reg_m5 ? vram_address[9:2] : vram_address[8:1];
	
	ym_slatch #(.DATA_WIDTH(8)) sl592(.MCLK(MCLK), .en(vram_addr_strobe), .inp(vram_row_addr), .val(vram_row_latch));
	
	ym_slatch #(.DATA_WIDTH(8)) sl593(.MCLK(MCLK), .en(vram_addr_strobe), .inp(vram_col_addr), .val(vram_col_latch));
	
	assign vram_addr_out =
		(vram_addr_lo_sel ? vram_row_latch : 8'h0) |
		(vram_addr_hi_sel ? vram_col_latch : 8'h0) |
		(vram_sd_sel ? vram_data_addr : 8'h0);
	
	ym_slatch #(.DATA_WIDTH(8)) sl594(.MCLK(MCLK), .en(vram_data_strobe), .inp(vram_wdata_lo_sel), .val(vram_data_addr));
	
	assign vram_wdata_lo_sel = vram_128k_pipe ? vram_rdata_lo : vram_wdata_lo;
	
	ym_slatch #(.DATA_WIDTH(8)) sl595(.MCLK(MCLK), .en(vram_wr_strobe), .inp(vram_data[7:0]), .val(vram_wdata_lo));
	
	ym_slatch #(.DATA_WIDTH(8)) sl596(.MCLK(MCLK), .en(vram_wr_strobe), .inp(vram_data[15:8]), .val(vram_wdata_hi));
	
	assign vram_wdata_hi_sel = vram_128k_pipe ? vram_rdata_hi : vram_wdata_hi;
	
	ym_slatch #(.DATA_WIDTH(8)) sl597(.MCLK(MCLK), .en(vram_data_strobe), .inp(vram_wdata_hi_sel), .val(vram_wdata_hi_out));
	
	ym_slatch #(.DATA_WIDTH(8)) sl598(.MCLK(MCLK), .en(vram_capture_strobe), .inp(RD_i), .val(vram_rdata_hi));
	
	ym_slatch #(.DATA_WIDTH(8)) sl599(.MCLK(MCLK), .en(vram_capture_strobe), .inp(AD_i), .val(vram_rdata_lo));
	
	assign vram_ad_out = reg_test0[5] ? vram_address[7:0] : vram_addr_out;
	
	assign vram_rd_out = reg_test0[5] ? vram_address[15:8] : vram_wdata_hi_out;
	
	assign SE0 = vram_serial_sel;
	assign SE1 = ~vram_serial_sel;
	assign SC = ~vram_sc_comb;
	assign RAS1 = ~vram_ras_comb;
	assign CAS1 = ~vram_cas_comb;
	assign WE1 = ~vram_we1_comb;
	assign WE0 = ~vram_we0_comb;
	assign OE1 = ~vram_oe1_comb;
	
	assign RD_d = ~vram_data_dir;
	assign AD_d = ~vram_data_dir;
	
	assign RD_o = vram_rd_out;
	assign AD_o = vram_ad_out;
	
	assign YS = vram_ys_mux;
	
	assign SPA_B_pull = ~spa_b_pipe;
	
	// -------------------------------------------------------------------------
	// Video priority MUX
	// -------------------------------------------------------------------------
	// Resolves pixel priority between sprite, plane A, plane B, and background
	// color. Implements shadow/highlight mode (reg_ste): priority bit and
	// palette index 14/15 control normal/shadow/highlight intensity levels.
	// Output is a 7-bit color bus: {priority, pal[1:0], index[3:0]}.

	assign cram_wr_any = cram_wr_hi | vsram_wr_normal | cram_wr_lo;

	ym_sr_bit sr601(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cram_wr_hi_cond), .sr_out(cram_wr_hi));

	ym_sr_bit sr602(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cram_wr_lo_cond), .sr_out(cram_wr_lo));
	
	assign pri_spr_or_a = l273 ? spr_out_pri : ~l320;
	
	assign pri_spr_hi_b_hi = ~l273 & spr_out_pri & l320;
	
	assign pri_a_hi_only = l273 & ~spr_out_pri & ~l320;
	
	assign pri_b_hi_only = ~l273 & ~spr_out_pri & l320;
	
	assign pri_a_hi_b_hi = l273 & ~spr_out_pri & l320;
	
	assign sh_no_priority = sh_mode_active & ~l273 & ~spr_out_pri & ~l320;
	
	assign sh_spr_only = sh_mode_active & ~l273 & spr_out_pri & ~l320;
	
	assign sh_mode_active = reg_ste & reg_m5;
	
	assign sh_spr_special = sh_mode_active & spr_pal_is_3 & spr_idx_14_or_15;
	
	assign spr_transparent = sh_spr_special | ~spr_idx_nonzero;
	
	assign spr_opaque = ~w646 & (reg_m5 | spr_idx_nonzero);
	
	assign planeb_opaque = ~reg_m5 | ~w648;
	
	assign test_layer_spr = reg_test0[8:7] == 2'h1;
	assign test_layer_a = reg_test0[8:7] == 2'h2;
	assign test_layer_b = reg_test0[8:7] == 2'h3;
	assign test_layer_bg = reg_test0[8:7] == 2'h0;
	
	assign sel_spr_case_1 = spr_opaque & pri_a_hi_only;
	
	assign sel_spr_case_2 = pri_b_hi_only & planeb_opaque;
	
	assign sel_spr_case_3 = spr_opaque & planeb_opaque & pri_a_hi_b_hi;
	
	assign sel_spr_cond = sel_spr_case_1 | sel_spr_case_2 | sel_spr_case_3 | pri_spr_or_a | pri_spr_hi_b_hi;
	
	assign sel_spr_valid = sel_spr_cond & spr_idx_nonzero & disp_not_test;
	
	assign sel_spr_normal = sel_spr_valid & ~sh_spr_special;
	
	assign sel_spr_final = sel_spr_normal | test_layer_spr;
	
	assign sel_spr_sh = sel_spr_valid & sh_spr_special;
	
	assign sel_a_case_1 = spr_transparent & pri_spr_or_a;
	
	assign sel_a_case_2 = spr_transparent & planeb_opaque & pri_spr_hi_b_hi;
	
	assign sel_a_case_3 = spr_transparent & planeb_opaque & pri_b_hi_only;
	
	assign sel_a_cond = sel_a_case_3 | sel_a_case_2 | sel_a_case_1 | pri_a_hi_b_hi | pri_a_hi_only;
	
	assign sel_a_valid = sel_a_cond & ~spr_opaque & disp_not_test;
	
	assign sel_a_final = sel_a_valid | test_layer_a;
	
	assign sel_b_case_1 = spr_transparent & pri_spr_hi_b_hi;
	
	assign sel_b_case_2 = spr_opaque & pri_a_hi_b_hi;
	
	assign sel_b_case_3 = spr_opaque & spr_transparent & pri_spr_or_a;
	
	assign sel_b_case_4 = spr_opaque & spr_transparent & pri_a_hi_only;
	
	assign sel_b_cond = sel_b_case_4 | sel_b_case_2 | sel_b_case_1 | sel_b_case_3 | pri_b_hi_only;
	
	assign sel_b_valid = sel_b_cond & ~planeb_opaque & disp_not_test;
	
	assign sel_b_final = sel_b_valid | test_layer_b;
	
	assign all_layers_trans = spr_opaque & spr_transparent & planeb_opaque;
	
	assign sel_bg_cond = all_layers_trans | ~disp_not_test;
	
	assign sel_bg_final = sel_bg_cond & test_layer_bg;
	
	assign disp_not_test = ~reg_test0[6] & active_delay_3;
	
	ym_sr_bit sr603(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sel_spr_final), .sr_out(sel_spr_pipe));
	
	ym_sr_bit sr604(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sel_a_final), .sr_out(sel_a_pipe));
	
	ym_sr_bit sr605(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sel_b_final), .sr_out(sel_b_pipe));
	
	ym_sr_bit sr606(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sel_bg_final), .sr_out(sel_bg_pipe));
	
	assign not_spr_not_idx14 = ~sel_spr_final & ~spr_idx_is_14;
	
	assign no_sh_special = ~sh_no_priority & ~sh_spr_only;
	
	assign spr_idx_14_or_15 = spr_idx_is_14 | spr_idx_is_15;
	
	assign sh_highlight_cond = no_sh_special & spr_idx_is_14 & sel_spr_sh;
	
	ym_sr_bit sr607(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_highlight_cond), .sr_out(sh_highlight_pipe));
	
	assign sh_shadow_cond = (sel_spr_sh & spr_idx_is_15) | (~spr_idx_is_14 & sh_no_priority) | (~no_sh_special & not_spr_not_idx14);
	
	assign sh_shadow_gated = sh_shadow_cond & active_delay_3;
	
	ym_sr_bit sr608(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_shadow_gated), .sr_out(sh_shadow_pipe_1));
	
	assign sh_shadow_mux = reg_test0[6] ? reg_col_b6 : sh_shadow_pipe_1;
	
	ym_sr_bit sr609(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_shadow_mux), .sr_out(sh_shadow_pipe_2));
	
	ym_sr_bit sr610(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_shadow_pipe_2), .sr_out(sh_shadow_pipe_3));
	
	assign sh_highlight_mux = reg_test0[6] ? reg_col_b7 : sh_highlight_pipe;
	
	ym_sr_bit sr611(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_highlight_mux), .sr_out(sh_highlight_pipe_2));
	
	ym_sr_bit sr612(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_highlight_pipe_2), .sr_out(sh_highlight_pipe_3));
	
	assign spa_b_gate_n = ~(sel_spr_pipe & reg_8c_b4);
	
	ym_sr_bit sr613(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(spa_b_gate_n), .sr_out(spa_b_pipe));
	
	assign cram_wr_m5 = reg_m5 & cram_wr_any;
	
	assign cram_wr_m4 = ~reg_m5 & cram_wr_any;
	
	ym_slatch #(.DATA_WIDTH(4)) sl_col_index(.MCLK(MCLK), .en(reg_wr_85), .inp(reg_data_l2[3:0]), .val(reg_col_index));
	
	ym_slatch #(.DATA_WIDTH(2)) sl_col_pal(.MCLK(MCLK), .en(reg_wr_85), .inp(reg_data_l2[5:4]), .val(reg_col_pal));
	
	ym_slatch sl_col_b6(.MCLK(MCLK), .en(reg_wr_85), .inp(reg_data_l2[6]), .val(reg_col_b6));
	
	ym_slatch sl_col_b7(.MCLK(MCLK), .en(reg_wr_85), .inp(reg_data_l2[7]), .val(reg_col_b7));
	
	ym_sr_bit sr614(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(blank_out_comb), .sr_out(blank_out_pipe));
	
	ym_sr_bit sr615(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(i_spa), .sr_out(spa_in_pipe));
	
	assign blank_out_comb = ~(force_bg_zero | (spa_in_pipe & ~reg_8c_b4));
	
	ym_sr_bit #(.SR_LENGTH(8)) sr616(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(full_disp_en), .sr_out(active_delay_8));
	
	assign active_mux_m5 = reg_m5 ? active_delay_8 : full_disp_en;
	
	assign color_bus_mux =
		(~cram_wr_any ? { color_pal, color_index } : 6'h0) |
		(cram_wr_m5 ? vram_address[6:1] : 6'h0) |
		(cram_wr_m4 ? { 1'h0, vram_address[4:0] } : 6'h0);
	
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr617(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(color_bus_mux), .data_out(color_bus_pipe));
	
	ym_sr_bit #(.SR_LENGTH(3)) sr618(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(active_mux_m5), .sr_out(active_delay_3));
	
	assign bg_color_zero = color_index == 4'h0;
	
	ym_sr_bit sr619(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(bg_color_zero), .sr_out(bg_zero_pipe_1));
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr620(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vram_data[11:9]), .data_out(cram_data_hi));
	
	assign cram_red_bits = reg_m5 ? data_rd_pipe[3:1] : data_rd_pipe[2:0];
	
	assign cram_grn_bits = reg_m5 ? data_rd_pipe[7:5] : data_rd_pipe[5:3];
	
	ym_slatch #(.DATA_WIDTH(9)) sl621(.MCLK(MCLK), .en(cram_latch_en), .inp(color_ram_out), .val(cram_rd_latch));
	
	ym_sr_bit_array #(.DATA_WIDTH(9)) sr622(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vdp_cramdot_dis ? color_ram_out_dp : color_ram_out), .data_out(cram_rd_pipe));
	
	ym_sr_bit sr623_1(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vsram_wr_normal), .sr_out(cram_wr_dly_1));
	
	ym_sr_bit sr623_2(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cram_wr_dly_1), .sr_out(cram_wr_dly_2));
	
	ym_sr_bit sr623_3(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(cram_wr_dly_2), .sr_out(cram_wr_dly_3));
	
	assign cram_latch_en = cram_wr_dly_1 & hclk1;
	
	assign disp_unblanked = ~(vint_delayed | vsync_gate);
	
	ym_sr_bit sr624(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(disp_unblanked), .sr_out(disp_unblank_pipe));
	
	ym_sr_bit sr625(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(bg_zero_pipe_1), .sr_out(bg_zero_pipe_2));
	
	assign force_bg_zero = bg_zero_pipe_2 & disp_unblank_pipe;
	
	assign cram_r_bit0 = reg_m5 ? cram_rd_pipe[1] : cram_rd_pipe[0];
	assign cram_r_bit1 = reg_m5 ? cram_rd_pipe[2] : cram_rd_pipe[1];
	assign cram_g_bit0 = reg_m5 ? cram_rd_pipe[4] : cram_rd_pipe[2];
	assign cram_g_bit1 = reg_m5 ? cram_rd_pipe[5] : cram_rd_pipe[3];
	assign cram_b_bit0 = reg_m5 ? cram_rd_pipe[7] : cram_rd_pipe[4];
	assign cram_b_bit1 = reg_m5 ? cram_rd_pipe[8] : cram_rd_pipe[5];
	
	assign dac_r_bit0 = cram_r_bit0 & disp_unblank_pipe & reg_80_b2;
	assign dac_r_bit1 = cram_r_bit1 & disp_unblank_pipe & reg_80_b2;
	assign dac_g_bit0 = cram_g_bit0 & disp_unblank_pipe & reg_80_b2;
	assign dac_g_bit1 = cram_g_bit1 & disp_unblank_pipe & reg_80_b2;
	assign dac_b_bit0 = cram_b_bit0 & disp_unblank_pipe & reg_80_b2;
	assign dac_b_bit1 = cram_b_bit1 & disp_unblank_pipe & reg_80_b2;
	
	assign dac_b_bit2 = cram_rd_pipe[6] & disp_unblank_pipe & reg_m5;
	assign dac_g_bit2 = cram_rd_pipe[3] & disp_unblank_pipe & reg_m5;
	assign dac_r_bit2 = cram_rd_pipe[0] & disp_unblank_pipe & reg_m5;
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr626(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({ dac_r_bit1, dac_r_bit0, dac_r_bit2 }), .data_out(dac_red_pipe));
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr627(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({ dac_g_bit1, dac_g_bit0, dac_g_bit2 }), .data_out(dac_grn_pipe));
	
	ym_sr_bit_array #(.DATA_WIDTH(3)) sr628(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in({ dac_b_bit1, dac_b_bit0, dac_b_bit2 }), .data_out(dac_blu_pipe));
	
	ym_sr_bit sr629(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_shadow_pipe_3), .sr_out(shadow_flag));
	
	ym_sr_bit sr630(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(sh_highlight_pipe_3), .sr_out(highlight_flag));
	
	assign normal_intensity = ~(shadow_flag | highlight_flag | ~reg_m5);
	
	assign shadow_level = ~highlight_flag & shadow_flag;
	
	wire [7:0] r_col;
	wire [7:0] g_col;
	wire [7:0] b_col;
	
	assign r_col[0] = dac_red_pipe == 3'h0;
	assign r_col[1] = dac_red_pipe == 3'h1;
	assign r_col[2] = dac_red_pipe == 3'h2;
	assign r_col[3] = dac_red_pipe == 3'h3;
	assign r_col[4] = dac_red_pipe == 3'h4;
	assign r_col[5] = dac_red_pipe == 3'h5;
	assign r_col[6] = dac_red_pipe == 3'h6;
	assign r_col[7] = dac_red_pipe == 3'h7;
	
	assign g_col[0] = dac_grn_pipe == 3'h0;
	assign g_col[1] = dac_grn_pipe == 3'h1;
	assign g_col[2] = dac_grn_pipe == 3'h2;
	assign g_col[3] = dac_grn_pipe == 3'h3;
	assign g_col[4] = dac_grn_pipe == 3'h4;
	assign g_col[5] = dac_grn_pipe == 3'h5;
	assign g_col[6] = dac_grn_pipe == 3'h6;
	assign g_col[7] = dac_grn_pipe == 3'h7;
	
	assign b_col[0] = dac_blu_pipe == 3'h0;
	assign b_col[1] = dac_blu_pipe == 3'h1;
	assign b_col[2] = dac_blu_pipe == 3'h2;
	assign b_col[3] = dac_blu_pipe == 3'h3;
	assign b_col[4] = dac_blu_pipe == 3'h4;
	assign b_col[5] = dac_blu_pipe == 3'h5;
	assign b_col[6] = dac_blu_pipe == 3'h6;
	assign b_col[7] = dac_blu_pipe == 3'h7;
	
	assign w1103[0][0] = (normal_intensity & r_col[0]) | (~reg_m5 & r_col[0]) | (~reg_m5 & r_col[1]) | (shadow_level & r_col[0]);
	assign w1103[0][1] = (shadow_level & r_col[1]);
	assign w1103[0][2] = (normal_intensity & r_col[1]) | (shadow_level & r_col[2]);
	assign w1103[0][3] = (shadow_level & r_col[3]);
	assign w1103[0][4] = (normal_intensity & r_col[2]) | (shadow_level & r_col[4]);
	assign w1103[0][5] = (~reg_m5 & r_col[2]) | (~reg_m5 & r_col[3]);
	assign w1103[0][6] = (shadow_level & r_col[5]);
	assign w1103[0][7] = (normal_intensity & r_col[3]) | (shadow_level & r_col[6]);
	assign w1103[0][8] = (shadow_level & r_col[7]) | (highlight_flag & r_col[0]);
	assign w1103[0][9] = (normal_intensity & r_col[4]) | (highlight_flag & r_col[1]);
	assign w1103[0][10] = (highlight_flag & r_col[2]);
	assign w1103[0][11] = (~reg_m5 & r_col[4]) | (~reg_m5 & r_col[5]);
	assign w1103[0][12] = (normal_intensity & r_col[5]) | (highlight_flag & r_col[3]);
	assign w1103[0][13] = (highlight_flag & r_col[4]);
	assign w1103[0][14] = (normal_intensity & r_col[6]) | (highlight_flag & r_col[5]);
	assign w1103[0][15] = (highlight_flag & r_col[6]);
	assign w1103[0][16] = (normal_intensity & r_col[7]) | (~reg_m5 & r_col[6]) | (~reg_m5 & r_col[7]) | (highlight_flag & r_col[7]);
	
	assign w1103[1][0] = (normal_intensity & g_col[0]) | (~reg_m5 & g_col[0]) | (~reg_m5 & g_col[1]) | (shadow_level & g_col[0]);
	assign w1103[1][1] = (shadow_level & g_col[1]);
	assign w1103[1][2] = (normal_intensity & g_col[1]) | (shadow_level & g_col[2]);
	assign w1103[1][3] = (shadow_level & g_col[3]);
	assign w1103[1][4] = (normal_intensity & g_col[2]) | (shadow_level & g_col[4]);
	assign w1103[1][5] = (~reg_m5 & g_col[2]) | (~reg_m5 & g_col[3]);
	assign w1103[1][6] = (shadow_level & g_col[5]);
	assign w1103[1][7] = (normal_intensity & g_col[3]) | (shadow_level & g_col[6]);
	assign w1103[1][8] = (shadow_level & g_col[7]) | (highlight_flag & g_col[0]);
	assign w1103[1][9] = (normal_intensity & g_col[4]) | (highlight_flag & g_col[1]);
	assign w1103[1][10] = (highlight_flag & g_col[2]);
	assign w1103[1][11] = (~reg_m5 & g_col[4]) | (~reg_m5 & g_col[5]);
	assign w1103[1][12] = (normal_intensity & g_col[5]) | (highlight_flag & g_col[3]);
	assign w1103[1][13] = (highlight_flag & g_col[4]);
	assign w1103[1][14] = (normal_intensity & g_col[6]) | (highlight_flag & g_col[5]);
	assign w1103[1][15] = (highlight_flag & g_col[6]);
	assign w1103[1][16] = (normal_intensity & g_col[7]) | (~reg_m5 & g_col[6]) | (~reg_m5 & g_col[7]) | (highlight_flag & g_col[7]);
	
	assign w1103[2][0] = (normal_intensity & b_col[0]) | (~reg_m5 & b_col[0]) | (~reg_m5 & b_col[1]) | (shadow_level & b_col[0]);
	assign w1103[2][1] = (shadow_level & b_col[1]);
	assign w1103[2][2] = (normal_intensity & b_col[1]) | (shadow_level & b_col[2]);
	assign w1103[2][3] = (shadow_level & b_col[3]);
	assign w1103[2][4] = (normal_intensity & b_col[2]) | (shadow_level & b_col[4]);
	assign w1103[2][5] = (~reg_m5 & b_col[2]) | (~reg_m5 & b_col[3]);
	assign w1103[2][6] = (shadow_level & b_col[5]);
	assign w1103[2][7] = (normal_intensity & b_col[3]) | (shadow_level & b_col[6]);
	assign w1103[2][8] = (shadow_level & b_col[7]) | (highlight_flag & b_col[0]);
	assign w1103[2][9] = (normal_intensity & b_col[4]) | (highlight_flag & b_col[1]);
	assign w1103[2][10] = (highlight_flag & b_col[2]);
	assign w1103[2][11] = (~reg_m5 & b_col[4]) | (~reg_m5 & b_col[5]);
	assign w1103[2][12] = (normal_intensity & b_col[5]) | (highlight_flag & b_col[3]);
	assign w1103[2][13] = (highlight_flag & b_col[4]);
	assign w1103[2][14] = (normal_intensity & b_col[6]) | (highlight_flag & b_col[5]);
	assign w1103[2][15] = (highlight_flag & b_col[6]);
	assign w1103[2][16] = (normal_intensity & b_col[7]) | (~reg_m5 & b_col[6]) | (~reg_m5 & b_col[7]) | (highlight_flag & b_col[7]);
	
/*
	// linear DAC
	assign DAC_R =
		(w1103[0][0] ? 8'd0 : 8'd0) |
		(w1103[0][1] ? 8'd18 : 8'd0) |
		(w1103[0][2] ? 8'd36 : 8'd0) |
		(w1103[0][3] ? 8'd54 : 8'd0) |
		(w1103[0][4] ? 8'd72 : 8'd0) |
		(w1103[0][5] ? 8'd85 : 8'd0) |
		(w1103[0][6] ? 8'd91 : 8'd0) |
		(w1103[0][7] ? 8'd109 : 8'd0) |
		(w1103[0][8] ? 8'd127 : 8'd0) |
		(w1103[0][9] ? 8'd145 : 8'd0) |
		(w1103[0][10] ? 8'd163 : 8'd0) |
		(w1103[0][11] ? 8'd170 : 8'd0) |
		(w1103[0][12] ? 8'd182 : 8'd0) |
		(w1103[0][13] ? 8'd200 : 8'd0) |
		(w1103[0][14] ? 8'd218 : 8'd0) |
		(w1103[0][15] ? 8'd236 : 8'd0) |
		(w1103[0][16] ? 8'd255 : 8'd0);
	
	assign DAC_G =
		(w1103[1][0] ? 8'd0 : 8'd0) |
		(w1103[1][1] ? 8'd18 : 8'd0) |
		(w1103[1][2] ? 8'd36 : 8'd0) |
		(w1103[1][3] ? 8'd54 : 8'd0) |
		(w1103[1][4] ? 8'd72 : 8'd0) |
		(w1103[1][5] ? 8'd85 : 8'd0) |
		(w1103[1][6] ? 8'd91 : 8'd0) |
		(w1103[1][7] ? 8'd109 : 8'd0) |
		(w1103[1][8] ? 8'd127 : 8'd0) |
		(w1103[1][9] ? 8'd145 : 8'd0) |
		(w1103[1][10] ? 8'd163 : 8'd0) |
		(w1103[1][11] ? 8'd170 : 8'd0) |
		(w1103[1][12] ? 8'd182 : 8'd0) |
		(w1103[1][13] ? 8'd200 : 8'd0) |
		(w1103[1][14] ? 8'd218 : 8'd0) |
		(w1103[1][15] ? 8'd236 : 8'd0) |
		(w1103[1][16] ? 8'd255 : 8'd0);
	
	assign DAC_B =
		(w1103[2][0] ? 8'd0 : 8'd0) |
		(w1103[2][1] ? 8'd18 : 8'd0) |
		(w1103[2][2] ? 8'd36 : 8'd0) |
		(w1103[2][3] ? 8'd54 : 8'd0) |
		(w1103[2][4] ? 8'd72 : 8'd0) |
		(w1103[2][6] ? 8'd91 : 8'd0) |
		(w1103[2][5] ? 8'd102 : 8'd0) |
		(w1103[2][7] ? 8'd109 : 8'd0) |
		(w1103[2][8] ? 8'd127 : 8'd0) |
		(w1103[2][9] ? 8'd145 : 8'd0) |
		(w1103[2][10] ? 8'd163 : 8'd0) |
		(w1103[2][11] ? 8'd170 : 8'd0) |
		(w1103[2][12] ? 8'd182 : 8'd0) |
		(w1103[2][13] ? 8'd200 : 8'd0) |
		(w1103[2][14] ? 8'd218 : 8'd0) |
		(w1103[2][15] ? 8'd236 : 8'd0) |
		(w1103[2][16] ? 8'd255 : 8'd0);
*/
	// -------------------------------------------------------------------------
	// DAC & color output (non-linear)
	// -------------------------------------------------------------------------
	// 17-level non-linear RGB DAC. The thermometer-coded w1103[0..2] signals
	// (one per color channel) are weighted with non-linear values that model
	// the voltage divider on the original MegaDrive board. Shadow/highlight
	// modes shift the intensity range via shadow_flag/highlight_flag flags.
	// (The commented-out linear DAC above was the original uniform version.)
	assign DAC_R =
		(w1103[0][0] ? 8'd0 : 8'd0) |
		(w1103[0][1] ? 8'd27 : 8'd0) |
		(w1103[0][2] ? 8'd49 : 8'd0) |
		(w1103[0][3] ? 8'd67 : 8'd0) |
		(w1103[0][4] ? 8'd84 : 8'd0) |
		(w1103[0][5] ? 8'd95 : 8'd0) |
		(w1103[0][6] ? 8'd100 : 8'd0) |
		(w1103[0][7] ? 8'd114 : 8'd0) |
		(w1103[0][8] ? 8'd128 : 8'd0) |
		(w1103[0][9] ? 8'd142 : 8'd0) |
		(w1103[0][10] ? 8'd156 : 8'd0) |
		(w1103[0][11] ? 8'd161 : 8'd0) |
		(w1103[0][12] ? 8'd172 : 8'd0) |
		(w1103[0][13] ? 8'd188 : 8'd0) |
		(w1103[0][14] ? 8'd206 : 8'd0) |
		(w1103[0][15] ? 8'd228 : 8'd0) |
		(w1103[0][16] ? 8'd255 : 8'd0);
	
	assign DAC_G =
		(w1103[1][0] ? 8'd0 : 8'd0) |
		(w1103[1][1] ? 8'd27 : 8'd0) |
		(w1103[1][2] ? 8'd49 : 8'd0) |
		(w1103[1][3] ? 8'd67 : 8'd0) |
		(w1103[1][4] ? 8'd84 : 8'd0) |
		(w1103[1][5] ? 8'd95 : 8'd0) |
		(w1103[1][6] ? 8'd100 : 8'd0) |
		(w1103[1][7] ? 8'd114 : 8'd0) |
		(w1103[1][8] ? 8'd128 : 8'd0) |
		(w1103[1][9] ? 8'd142 : 8'd0) |
		(w1103[1][10] ? 8'd156 : 8'd0) |
		(w1103[1][11] ? 8'd161 : 8'd0) |
		(w1103[1][12] ? 8'd172 : 8'd0) |
		(w1103[1][13] ? 8'd188 : 8'd0) |
		(w1103[1][14] ? 8'd206 : 8'd0) |
		(w1103[1][15] ? 8'd228 : 8'd0) |
		(w1103[1][16] ? 8'd255 : 8'd0);
	
	assign DAC_B =
		(w1103[2][0] ? 8'd0 : 8'd0) |
		(w1103[2][1] ? 8'd27 : 8'd0) |
		(w1103[2][2] ? 8'd49 : 8'd0) |
		(w1103[2][3] ? 8'd67 : 8'd0) |
		(w1103[2][4] ? 8'd84 : 8'd0) |
		(w1103[2][6] ? 8'd100 : 8'd0) |
		(w1103[2][5] ? 8'd109 : 8'd0) |
		(w1103[2][7] ? 8'd114 : 8'd0) |
		(w1103[2][8] ? 8'd128 : 8'd0) |
		(w1103[2][9] ? 8'd142 : 8'd0) |
		(w1103[2][10] ? 8'd156 : 8'd0) |
		(w1103[2][11] ? 8'd161 : 8'd0) |
		(w1103[2][12] ? 8'd172 : 8'd0) |
		(w1103[2][13] ? 8'd188 : 8'd0) |
		(w1103[2][14] ? 8'd206 : 8'd0) |
		(w1103[2][15] ? 8'd228 : 8'd0) |
		(w1103[2][16] ? 8'd255 : 8'd0);
	
	// -------------------------------------------------------------------------
	// Color RAM (CRAM)
	// -------------------------------------------------------------------------
	// 64 entries × 9 bits (3×3-bit RGB). Indexed by the 6-bit color bus
	// {pal[1:0], index[3:0]}. Written via the video MUX during active display.

	wire [5:0] color_ram_index = color_bus_pipe;
	
	wire [8:0] color_ram_data_in = { cram_data_hi, cram_grn_bits, cram_red_bits };
	
	always @(posedge MCLK)
	begin
		if (hclk1) // write cycle
		begin
			if (cram_wr_lo)
				color_ram[color_ram_index][5:0] <= color_ram_data_in[5:0];
			if (cram_wr_hi)
				color_ram[color_ram_index][8:6] <= color_ram_data_in[8:6];
		end
		color_ram_out <= color_ram[color_ram_index];
	end
	
	// -------------------------------------------------------------------------
	// PSG (SN76489)
	// -------------------------------------------------------------------------
	// Integrated SN76489-compatible Programmable Sound Generator.
	// 3 square-wave tone channels + 1 noise channel, each with 4-bit
	// volume attenuation. Clocked from the CPU clock. Output is a 16-bit
	// sum of all four channel values.

	assign psg_clk1 = cpu_clk0;
	assign psg_clk2 = ~cpu_clk0;
	
	ym_sr_bit sr631(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .bit_in(reset_comb), .sr_out(psg_rst_pipe_1));
	ym_sr_bit sr632(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .bit_in(psg_rst_pipe_1), .sr_out(psg_rst_pipe_2));
	
	assign psg_rst_edge = psg_rst_pipe_1 & ~psg_rst_pipe_2;
	
	assign psg_div_fb = ~psg_rst_edge & ~psg_div_out;
	
	ym_cnt_bit cnt649(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .c_in(psg_div_out), .reset(psg_rst_edge), .val(psg_div_cnt));
	
	ym_sr_bit sr633(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .bit_in(psg_div_fb), .sr_out(psg_div_out));
	
	ym_dlatch_1 dl634(.MCLK(MCLK), .c1(psg_clk1), .inp(psg_div_cnt), .nval(psg_div_latch));
	
	assign psg_hclk1 = psg_div_latch & psg_div_out;
	
	assign psg_hclk2 = ~psg_div_latch & psg_div_out;
	
	ym7101_rs_trig rs43(.MCLK(MCLK), .set(psg_wr_pipe_1), .rst(psg_wr_any), .q(psg_wr_trig));
	
	assign psg_wr_comb = ~psg_wr_trig & ~psg_wr_any;
	
	ym_sr_bit sr635(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .bit_in(psg_wr_comb), .sr_out(psg_wr_pipe_1));
	
	ym_sr_bit sr636(.MCLK(MCLK), .c1(psg_clk1), .c2(psg_clk2), .bit_in(psg_wr_pipe_1), .sr_out(psg_wr_pipe_2));
	
	ym_sr_bit sr637(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(reset_comb), .sr_out(psg_rst_hclk));
	
	ym7101_rs_trig rs44(.MCLK(MCLK), .set(psg_wr_noise), .rst(psg_noise_pipe), .q(psg_noise_trig));
	
	ym_sr_bit sr638(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_noise_trig), .sr_out(psg_noise_pipe));
	
	assign psg_tone_en = ~psg_noise_pipe & ~psg_rst_hclk;
	
	assign psg_cnt_down = psg_tone_en & ~psg_cnt_zero;
	
	assign psg_cnt_up = psg_tone_en & psg_cnt_zero;
	
	assign psg_cnt_zero = ~psg_cnt_zero_pipe & psg_noise_fb;
	
	ym_sr_bit sr639(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_noise_fb), .sr_out(psg_cnt_zero_pipe));
	
	assign psg_noise_fb = psg_noise_ctrl[1:0] == 2'h3 ? psg_sq_2 : psg_sq_3;
	
	assign psg_test_mute_0 = reg_test0[9] & reg_test0[11:10] != 2'h0;
	assign psg_test_mute_1 = reg_test0[9] & reg_test0[11:10] != 2'h1;
	assign psg_test_mute_2 = reg_test0[9] & reg_test0[11:10] != 2'h2;
	assign psg_test_mute_3 = reg_test0[9] & reg_test0[11:10] != 2'h3;
	
	assign psg_freq_mux = psg_tone_active ? psg_freq_pipe_3 : 10'h0;
	
	assign psg_freq_inc = psg_freq_mux + 10'h1;
	
	assign psg_lfsr_fb = psg_lfsr[15] ^ psg_lfsr[12];
	
	ym_sr_bit_en #(.SR_LENGTH(16)) sr640(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .en1(psg_cnt_up), .en2(psg_cnt_down),
		.data_in(~psg_lfsr_nz | ~psg_lfsr_gate), .data_out(psg_lfsr));
	
	assign psg_lfsr_nz = psg_lfsr[14:0] != 15'h0;
	
	assign psg_lfsr_gate = ~(psg_lfsr_fb & psg_noise_ctrl[2]);
	
	ym_sr_bit_array #(.DATA_WIDTH(10)) sr641(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .data_in(psg_freq_inc), .data_out(psg_freq_pipe_0));
	
	ym_sr_bit_array #(.DATA_WIDTH(10)) sr642(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .data_in(psg_freq_pipe_0), .data_out(psg_freq_pipe_1));
	
	ym_sr_bit_array #(.DATA_WIDTH(10)) sr643(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .data_in(psg_freq_pipe_1), .data_out(psg_freq_pipe_2));
	
	ym_sr_bit_array #(.DATA_WIDTH(10)) sr644(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .data_in(psg_freq_pipe_2), .data_out(psg_freq_pipe_3));
	
	assign psg_tone_active = ~psg_rst_sync & ~psg_freq_match;
	
	assign psg_ch0_out = psg_ch_ring[0] & psg_tone_ring[3];
	assign psg_ch1_out = psg_ch_ring[0] & psg_tone_ring[2];
	assign psg_ch2_out = psg_ch_ring[0] & psg_tone_ring[1];
	assign psg_ch3_out = psg_ch_ring[0] & psg_tone_ring[0];
	
	ym_cnt_bit cnt645(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .c_in(psg_ch0_out), .reset(psg_rst_sync), .val(psg_sq_0));
	
	ym_cnt_bit cnt646(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .c_in(psg_ch1_out), .reset(psg_rst_sync), .val(psg_sq_1));
	
	ym_cnt_bit cnt647(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .c_in(psg_ch2_out), .reset(psg_rst_sync), .val(psg_sq_2));
	
	ym_cnt_bit cnt648(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .c_in(psg_ch3_out), .reset(psg_rst_sync), .val(psg_sq_3));
	
	assign psg_freq_sel =
		(psg_ch0_sel ? psg_freq_0 : 10'h0) |
		(psg_ch1_sel ? psg_freq_1 : 10'h0) |
		(psg_ch2_sel ? psg_freq_2 : 10'h0) |
		(psg_ch3_sel ? { 3'h0, psg_noise_ctrl[1:0] == 2'h2, psg_noise_ctrl[1:0] == 2'h1, psg_noise_ctrl[1:0] == 2'h0, 4'h0 } : 10'h0);
	
	assign psg_freq_match = psg_freq_sel <= psg_freq_pipe_3;
	
	assign psg_ch3_sel = psg_ch_ring[3] & ~psg_rst_sync;
	assign psg_ch2_sel = psg_ch_ring[2] & ~psg_rst_sync;
	assign psg_ch1_sel = psg_ch_ring[1] & ~psg_rst_sync;
	assign psg_ch0_sel = psg_ch_ring[0] & ~psg_rst_sync;
	
	ym_sr_bit sr650_0(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_ring_wrap), .sr_out(psg_ch_ring[0]));
	
	ym_sr_bit sr650_1(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_ch_ring[0]), .sr_out(psg_ch_ring[1]));
	
	ym_sr_bit sr650_2(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_ch_ring[1]), .sr_out(psg_ch_ring[2]));
	
	ym_sr_bit sr650_3(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_ch_ring[2]), .sr_out(psg_ch_ring[3]));

	assign psg_ring_wrap = psg_ch_ring[2:0] == 3'h0 & ~psg_rst_hclk;
	
	ym_sr_bit sr651(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_rst_hclk), .sr_out(psg_rst_sync));
	
	assign psg_not_rst = ~psg_rst_sync;
	
	ym_sr_bit sr652_0(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_freq_match), .sr_out(psg_tone_ring[0]));
	
	ym_sr_bit sr652_1(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_tone_ring[0]), .sr_out(psg_tone_ring[1]));
	
	ym_sr_bit sr652_2(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_tone_ring[1]), .sr_out(psg_tone_ring[2]));
	
	ym_sr_bit sr652_3(.MCLK(MCLK), .c1(psg_hclk1), .c2(psg_hclk2), .bit_in(psg_tone_ring[2]), .sr_out(psg_tone_ring[3]));
	
	ym_slatch #(.DATA_WIDTH(8)) sl653(.MCLK(MCLK), .en(psg_wr_any), .inp(io_data[7:0]), .val(psg_data_latch));
	
	assign psg_data_mux = psg_not_rst ? psg_data_latch : 8'h0;
	
	ym_slatch #(.DATA_WIDTH(3)) sl654(.MCLK(MCLK), .en(psg_wr_pipe_1 & psg_data_mux[7]), .inp(psg_data_mux[6:4]), .val(psg_reg_addr));
	
	assign psg_wr_vol0 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h1);
	assign psg_wr_vol1 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h3);
	assign psg_wr_freq2 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h4);
	assign psg_wr_freq1 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h2);
	assign psg_wr_vol2 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h5);
	assign psg_wr_freq0 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h0);
	assign psg_wr_vol3 = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h7);
	assign psg_wr_noise = psg_rst_hclk | (psg_wr_pipe_2 & psg_reg_addr == 3'h6);
	
	assign psg_vol_data = psg_not_rst ? psg_data_mux[3:0] : 4'hf;
	
	ym_slatch #(.DATA_WIDTH(4)) sl655(.MCLK(MCLK), .en(psg_wr_vol0), .inp(psg_vol_data), .val(psg_vol_0));
	
	ym_slatch #(.DATA_WIDTH(4)) sl656(.MCLK(MCLK), .en(psg_wr_vol1), .inp(psg_vol_data), .val(psg_vol_1));
	
	ym_slatch #(.DATA_WIDTH(4)) sl657(.MCLK(MCLK), .en(psg_wr_vol2), .inp(psg_vol_data), .val(psg_vol_2));
	
	ym_slatch #(.DATA_WIDTH(4)) sl658(.MCLK(MCLK), .en(psg_wr_vol3), .inp(psg_vol_data), .val(psg_vol_3));
	
	assign psg_latch_mode = psg_data_mux[7] | psg_rst_hclk;
	
	ym_slatch #(.DATA_WIDTH(6)) sl661_1(.MCLK(MCLK), .en(psg_wr_freq2 & ~psg_data_mux[7]), .inp(psg_data_mux[5:0]), .val(psg_freq_2[9:4]));
	ym_slatch #(.DATA_WIDTH(4)) sl661_2(.MCLK(MCLK), .en(psg_wr_freq2 & psg_latch_mode), .inp(psg_data_mux[3:0]), .val(psg_freq_2[3:0]));
	
	ym_slatch #(.DATA_WIDTH(6)) sl660_1(.MCLK(MCLK), .en(psg_wr_freq1 & ~psg_data_mux[7]), .inp(psg_data_mux[5:0]), .val(psg_freq_1[9:4]));
	ym_slatch #(.DATA_WIDTH(4)) sl660_2(.MCLK(MCLK), .en(psg_wr_freq1 & psg_latch_mode), .inp(psg_data_mux[3:0]), .val(psg_freq_1[3:0]));
	
	ym_slatch #(.DATA_WIDTH(6)) sl659_1(.MCLK(MCLK), .en(psg_wr_freq0 & ~psg_data_mux[7]), .inp(psg_data_mux[5:0]), .val(psg_freq_0[9:4]));
	ym_slatch #(.DATA_WIDTH(4)) sl659_2(.MCLK(MCLK), .en(psg_wr_freq0 & psg_latch_mode), .inp(psg_data_mux[3:0]), .val(psg_freq_0[3:0]));
	
	ym_slatch #(.DATA_WIDTH(3)) sl662(.MCLK(MCLK), .en(psg_wr_noise), .inp(psg_data_mux[2:0]), .val(psg_noise_ctrl));
	
	assign psg_sq0_mute = ~reg_test0[9] & ~psg_sq_0;
	assign psg_sq1_mute = ~reg_test0[9] & ~psg_sq_1;
	assign psg_sq2_mute = ~reg_test0[9] & ~psg_sq_2;
	assign psg_noise_mute = ~reg_test0[9] & ~psg_lfsr[14];
	
	assign psg_atten_0 = psg_sq0_mute ? 4'hf : psg_vol_0;
	assign psg_atten_1 = psg_sq1_mute ? 4'hf : psg_vol_1;
	assign psg_atten_2 = psg_sq2_mute ? 4'hf : psg_vol_2;
	assign psg_atten_3 = psg_noise_mute ? 4'hf : psg_vol_3;
	
	function [15:0] psg_vol;
		input [3:0] value;
		begin
			case (value)
				4'h0   : psg_vol = 16'd1200;
				4'h1   : psg_vol = 16'd0926;
				4'h2   : psg_vol = 16'd0746;
				4'h3   : psg_vol = 16'd0582;
				4'h4   : psg_vol = 16'd0458;
				4'h5   : psg_vol = 16'd0348;
				4'h6   : psg_vol = 16'd0274;
				4'h7   : psg_vol = 16'd0208;
				4'h8   : psg_vol = 16'd0158;
				4'h9   : psg_vol = 16'd0115;
				4'ha   : psg_vol = 16'd0086;
				4'hb   : psg_vol = 16'd0061;
				4'hc   : psg_vol = 16'd0040;
				4'hd   : psg_vol = 16'd0022;
				4'he   : psg_vol = 16'd0010;
				4'hf   : psg_vol = 16'd0000;
				default: psg_vol = 16'd0000;
			endcase
		end
	endfunction

	wire [15:0] psg_val[0:3]; // PSG Volume

	assign psg_val[0] = psg_test_mute_0 ? -16'd1270 : psg_vol(psg_atten_0);
	assign psg_val[1] = psg_test_mute_1 ? -16'd1270 : psg_vol(psg_atten_1);
	assign psg_val[2] = psg_test_mute_2 ? -16'd1270 : psg_vol(psg_atten_2);
	assign psg_val[3] = psg_test_mute_3 ? -16'd1270 : psg_vol(psg_atten_3);
	
	//assign SOUND = psg_val[0] + psg_val[1] + psg_val[2] + psg_val[3];
	
	always @(posedge MCLK)
	begin
		SOUND <= psg_val[0] + psg_val[1] + psg_val[2] + psg_val[3];
	end
	
	// -------------------------------------------------------------------------
	// VRAM bus drive
	// -------------------------------------------------------------------------
	// Wired-AND bus pattern for VRAM address and data. Multiple drivers
	// assert val/pull pairs; the final bus value is the AND of all active
	// drivers. The serial data latch captures SD[7:0] on clk1.

	ym_dlatch_1 #(.DATA_WIDTH(8)) dl_vs(.MCLK(MCLK), .c1(clk1), .inp(SD), .val(vram_serial));
	
	wire [15:0] vram_data_val =
		(data_rd_s1 ? { fifo_hi_s1, fifo_out_s1 } : 16'hffff) &
		(data_rd_s2 ? { fifo_hi_s2, fifo_out_s2 } : 16'hffff) &
		(data_rd_s3 ? { fifo_hi_s3, fifo_out_s3 } : 16'hffff) &
		(data_rd_s0 ? { fifo_hi_s0, fifo_out_s0 } : 16'hffff) &
		(l183 ? { 5'h1f, l180 } : 16'hffff) &
		(sat_rd_pipe_2 ? { 5'h1f, sat_field_latch } : 16'hffff) &
		(vram_dma_pipe ? { vram_rdata_hi, vram_rdata_lo } : 16'hffff) &
		(cram_wr_dly_3 ? { 4'hf, cram_rd_latch[8:6], 1'h1, cram_rd_latch[5:3], 1'h1, cram_rd_latch[2:0], 1'h1 } : 16'hffff);
		
	wire [15:0] vram_data_pull =
		(data_rd_s1 ? 16'hffff : 16'h0) |
		(data_rd_s2 ? 16'hffff : 16'h0) |
		(data_rd_s3 ? 16'hffff : 16'h0) |
		(data_rd_s0 ? 16'hffff : 16'h0) |
		(l183 ? 16'h07ff : 16'h0) |
		(sat_rd_pipe_2 ? 16'h07ff : 16'h0) |
		(vram_dma_pipe ? 16'hffff : 16'h0) |
		(cram_wr_dly_3 ? 16'heee : 16'h0);
	
	wire [16:0] vram_address_val =
		(dma_ext_copy ? { reg_sa_high[0], reg_sa_low } : 17'h1ffff) &
		(dma_data_active ? reg_data_l2[16:0] : 17'h1ffff) &
		(dma_fill_busy ? { fifo_addr_pipe[16:1], ~fifo_addr_pipe[0] } : 17'h1ffff) &
		(fifo_rd_s2 ? fifo_data_0 : 17'h1ffff) &
		(fifo_rd_s0 ? fifo_data_1 : 17'h1ffff) &
		(fifo_rd_s3 ? fifo_data_2 : 17'h1ffff) &
		(fifo_rd_s1 ? fifo_data_3 : 17'h1ffff) &
		(w531 ? { w532[3:1], 14'h3fff } : 17'h1ffff) &
		(w558 ? { 3'h7, w532[0], w533, w527[4:0], w555[4:0], 1'h0 } : 17'h1ffff) &
		(w643 ? { reg_hs, w535, 2'h0 } : 17'h1ffff) & // hscroll
		(l202 ? { reg_wd[5:1], w536, hcnt[7:4], 2'h0 } : 17'h1ffff) & // window
		(l196 ? { 12'hfff, w577[2:0], ~l198, 1'h0 } : 17'h1ffff) &
		(l199 ? { 3'h7, w578, 5'h1f } : 17'h1ffff) &
		(w566 ? { w579, 14'h3fff } : 17'h1ffff) &
		(l218 ? { w580, 5'h1f } : 17'h1ffff) &
		(link_cnt_inc ? { 9'h1ff, 2'h0, sat_link_cnt[4:0], 1'h0 } : 17'h1ffff) &
		(m4_attr_phase ? { 3'h7, reg_86_b2, vram_attr_sel[7:1], spr_cnt_m4_bit, spr_cnt_bit_2, spr_cnt_bit_1, spr_cnt_bit_0, ~hcnt[1], 1'h0 } : 17'h1ffff) &
		(m4_fetch_active ? { 3'h7, reg_at[6:1], 8'hff} : 17'h1ffff) &
		(m5_fetch_phase ? { reg_at[7:1], spridx_readback[6:0], 3'h4 } : 17'h1ffff) &
		(m4_fetch_phase ? { 9'h1ff, spridx_sr_6[7], spridx_sr_5[7], spridx_sr_4[7], spridx_sr_3[7], spridx_sr_2[7], spridx_sr_1[7], spridx_sr_0[7], 1'h0 } : 17'h1ffff) &
		(m5_fetch_pipe ? (interlace_dblres ?
			{ pattern_addr, spr_rd_yoff[3], spr_rd_yoff[2:0], 2'h0 } : { reg_86_b5, pattern_addr, spr_rd_yoff[2:0], 2'h0 }) : 17'h1ffff);
	
	wire [16:0] vram_address_pull =
		(dma_ext_copy ? 17'h1ffff : 17'h0) |
		(dma_data_active ? 17'h1ffff : 17'h0) |
		(dma_fill_busy ? 17'h1ffff : 17'h0) |
		(fifo_rd_s2 ? 17'h1ffff : 17'h0) |
		(fifo_rd_s0 ? 17'h1ffff : 17'h0) |
		(fifo_rd_s3 ? 17'h1ffff : 17'h0) |
		(fifo_rd_s1 ? 17'h1ffff : 17'h0) |
		(w531 ? 17'h1c000 : 17'h0) |
		(w558 ? 17'h03fff : 17'h0) |
		(w643 ? 17'h1ffff : 17'h0) |
		(l202 ? 17'h1ffff : 17'h0) | 
		(l196 ? 17'h0001f : 17'h0) |
		(l199 ? 17'h03fe0 : 17'h0) |
		(w566 ? 17'h1c000 : 17'h0) |
		(l218 ? 17'h1ffe0 : 17'h0) |
		(link_cnt_inc ? 17'h000ff : 17'h0) |
		(m4_attr_phase ? 17'h03fff : 17'h0) |
		(m4_fetch_active ? 17'h03f00 : 17'h0) |
		(m5_fetch_phase ? 17'h1ffff : 17'h0) |
		(m4_fetch_phase ? 17'h000ff : 17'h0) |
		(m5_fetch_pipe ? 17'h1ffff : 17'h0);
	
	/*assign vram_data =
		(data_rd_s1 ? { fifo_hi_s1, fifo_out_s1 } : 16'h0) |
		(data_rd_s2 ? { fifo_hi_s2, fifo_out_s2 } : 16'h0) |
		(data_rd_s3 ? { fifo_hi_s3, fifo_out_s3 } : 16'h0) |
		(data_rd_s0 ? { fifo_hi_s0, fifo_out_s0 } : 16'h0) |
		(l183 ? { 5'h0, l180 } : 16'h0) |
		(sat_rd_pipe_2 ? { 5'h0, sat_field_latch } : 16'h0) |
		(vram_dma_pipe ? { vram_rdata_hi, vram_rdata_lo } : 16'h0) |
		(cram_wr_dly_3 ? { 4'h0, cram_rd_latch[8:6], 1'h0, cram_rd_latch[5:3], 1'h0, cram_rd_latch[2:0], 1'h0 } : 16'h0);*/
		
	/*assign vram_address =
		(dma_ext_copy ? { reg_sa_high[0], reg_sa_low } : 17'h0) |
		(dma_data_active ? reg_data_l2[16:0] : 17'h0) |
		(dma_fill_busy ? { fifo_addr_pipe[16:1], ~fifo_addr_pipe[0] } : 17'h0) |
		(fifo_rd_s2 ? fifo_data_0 : 17'h0) |
		(fifo_rd_s0 ? fifo_data_1 : 17'h0) |
		(fifo_rd_s3 ? fifo_data_2 : 17'h0) |
		(fifo_rd_s1 ? fifo_data_3 : 17'h0) |
		(w531 ? { w532[3:1], 14'h0 } : 17'h0) |
		(w558 ? { 3'h0, w532[0], w533, w527[4:0], w555[4:0], 1'h0 } : 17'h0) |
		(w643 ? { reg_hs, w535, 2'h0 } : 17'h0) | // hscroll
		(l202 ? { reg_wd[5:1], w536, hcnt[7:4], 2'h0 } : 17'h0) | // window
		(l196 ? { 12'h0, w577[2:0], ~l198, 1'h0 } : 17'h0) |
		(l199 ? { 3'h0, w578, 5'h0 } : 17'h0) |
		(w566 ? { w579, 14'h0 } : 17'h0) |
		(l218 ? { w580, 5'h0 } : 17'h0) |
		(link_cnt_inc ? { 9'h0, 2'h0, sat_link_cnt[4:0], 1'h0 } : 17'h0) |
		(m4_attr_phase ? { 3'h0, reg_86_b2, vram_attr_sel[7:1], spr_cnt_m4_bit, spr_cnt_bit_2, spr_cnt_bit_1, spr_cnt_bit_0, ~hcnt[1], 1'h0 } : 17'h0) |
		(m4_fetch_active ? { 3'h0, reg_at[6:1], 8'h0} : 17'h0) |
		(m5_fetch_phase ? { reg_at[7:1], spridx_readback[6:0], 3'h4 } : 17'h0) |
		(m4_fetch_phase ? { 9'h0, spridx_sr_6[7], spridx_sr_5[7], spridx_sr_4[7], spridx_sr_3[7], spridx_sr_2[7], spridx_sr_1[7], spridx_sr_0[7], 1'h0 } : 17'h0) |
		(m5_fetch_pipe ? (interlace_dblres ?
			{ pattern_addr, spr_rd_yoff[3], spr_rd_yoff[2:0], 2'h0 } : { reg_86_b5, pattern_addr, spr_rd_yoff[2:0], 2'h0 }) : 17'h0);*/
	
	always @(posedge MCLK)
	begin
		vram_data <= (vram_data_pull & vram_data_val) | (~vram_data_pull & vram_data);	
		vram_address <= (vram_address_pull & vram_address_val) | (~vram_address_pull & vram_address);
	end
	
	// -------------------------------------------------------------------------
	// I/O bus drive
	// -------------------------------------------------------------------------
	// CPU address and data bus drivers. Same wired-AND val/pull pattern as
	// the VRAM bus. During DMA, the VDP drives the address bus with the DMA
	// source address (reg_sa_high/reg_sa_low).

	wire vdp_data_dir = ~cpu_data_oe | ext_test_2;
	wire vdp_address_dir = ~dma_addr_oe | ext_test_2;
	
	wire [22:0] io_address_val =
		(vdp_address_dir ? (CA_i & 23'h73ffff) : 23'h73ffff) &
		(dma_addr_oe ? ({ reg_sa_high, reg_sa_low } & 23'h33ffff) : 23'h73ffff);
	
	wire [22:0] io_address_pull =
		(vdp_address_dir ? 23'h73ffff : 23'h0) |
		(dma_addr_oe ? 23'h33ffff : 23'h0);
	
	//reg [22:0] io_address_mem = 23'h0;
	
	wire [22:0] io_address_t = (io_address_pull & io_address_val) | (~io_address_pull & io_address);
	
	assign CA_o[22] = io_address_22o;
	assign CA_o[21:0] = io_address[21:0];
	
	assign CA_d = vdp_address_dir;
	
	wire [15:0] io_data_val =
		(vdp_data_dir ? CD_i : 16'hffff) &
		(tst_fn7_rd ? { 2'h3, spr_rd_yoff[5:3], sprdata_test_mux } : 16'hffff) &
		(z80_vdp_rd ? { 5'h1f, ~vcnt_ext[9], ~vcnt_ext[8], ~hcnt[0], 8'hff} : 16'hffff) &
		(vdp_data_rd_odd ? { 6'h3f, dma_state, fifo_wr_gate, eint_pend, spr_overflow, spr_collision, field_bit, active_disp_gate, vint_delayed, hv_data_sel, hv_byte_sel } : 16'hffff) &
		(hv_cnt_rd ? { vcnt_latch[7:0], 8'hff } : 16'hffff) &
		(hv_rd_any ? { 8'hff, hv_cnt_data[7:0] } : 16'hffff) &
		(data_rd_even ? { vram_rd_hi_lat[7:0], vram_rd_lo_lat[7:0] } : 16'hffff) &
		(z80_int_ack ? { 8'hff, 5'h0, ipl2_src, ipl1_src, 1'h0 } : 16'hffff) &
		(tst_fn2_rd ? { 8'hff, ~line_zero_dly, ~disp_start, ~vscr_active, ~cell_m4_active, ~cell_bound_active, ~m4_or_vram_slot, ~pre_wrap_active, ~sub_slot_active } : 16'hffff) &
		(tst_fn3_rd ? { 2'h3, ~m4_window_dly, ~vram_or_ext_slot, ~blank_slot_active, ~fetch_all_dly, ~m4_border_dly, ~access_main_dly, ~access_ext_dly, ~odd_slot, ~slot0_active, ~slot1_active, ~slot2_active, ~slot3_active, ~edclk_dly, ~slot_idle_dly } : 16'hffff) &
		(tst_fn6_rd ? { 4'hf, spridx_sr_active[19], spridx_sr_6[19], spridx_sr_5[19], spridx_sr_4[19], spridx_sr_3[19], spridx_sr_2[19], spridx_sr_1[19], spridx_sr_0[19], spr_cnt_sr_3[9], spr_cnt_sr_2[9], spr_cnt_sr_1[9], spr_cnt_sr_0[9] } : 16'hffff) &
		(tst_fn8_rd ? { 1'h1, ~test_rd_idx3_odd, ~test_rd_idx2_odd, ~test_rd_idx1_odd, ~test_rd_idx0_odd, ~test_rd_pri_odd, ~test_rd_pal1_odd, ~test_rd_pal0_odd, 1'h1, ~test_rd_idx3_even, ~test_rd_idx2_even, ~test_rd_idx1_even, ~test_rd_idx0_even, ~test_rd_pri_even, ~test_rd_pal1_even, ~test_rd_pal0_even } : 16'hffff) &
		(tst_fn4_rd ? { 5'h1f, ~dac_b_bit1, ~dac_b_bit0, ~dac_b_bit2, ~dac_g_bit1, ~dac_g_bit0, ~dac_g_bit2, ~dac_r_bit1, ~dac_r_bit0, ~dac_r_bit2, ~sh_shadow_pipe_3, ~sh_highlight_pipe_3 } : 16'hffff) &
		(tst_fn5_rd ? { ~psg_atten_0, ~psg_atten_1, ~psg_atten_2, ~psg_atten_3 } : 16'hffff);
	
	wire [15:0] io_data_pull =
		(vdp_data_dir ? 16'hffff : 16'h0) |
		(tst_fn7_rd ? 16'h3fff : 16'h0) |
		(z80_vdp_rd ? 16'h0700 : 16'h0) |
		(vdp_data_rd_odd ? 16'h03ff : 16'h0) |
		(hv_cnt_rd ? 16'hff00 : 16'h0) |
		(hv_rd_any ? 16'h00ff : 16'h0) |
		(data_rd_even ? 16'hffff : 16'h0) |
		(z80_int_ack ? 16'h00ff : 16'h0) |
		(tst_fn2_rd ? 16'h00ff : 16'h0) |
		(tst_fn3_rd ? 16'h3fff : 16'h0) |
		(tst_fn6_rd ? 16'h0fff : 16'h0) |
		(tst_fn8_rd ? 16'h7f7f : 16'h0) |
		(tst_fn4_rd ? 16'h07ff : 16'h0) |
		(tst_fn5_rd ? 16'hffff : 16'h0);
	
	//reg [15:0] io_data_mem = 16'h0;
	
	/*assign io_data =
		(vdp_data_dir ? CD_i : 16'h0) |
		(tst_fn7_rd ? { 2'h0, spr_rd_yoff[5:3], sprdata_test_mux } : 16'h0) |
		(z80_vdp_rd ? { 5'h0, ~vcnt_ext[9], ~vcnt_ext[8], ~hcnt[0], 8'h0} : 16'h0) |
		(vdp_data_rd_odd ? { 6'h0, dma_state, fifo_wr_gate, eint_pend, spr_overflow, spr_collision, field_bit, active_disp_gate, vint_delayed, hv_data_sel, hv_byte_sel } : 16'h0) |
		(hv_cnt_rd ? { vcnt_latch[7:0], 8'h0 } : 16'h0) |
		(hv_rd_any ? { 8'h0, hv_cnt_data[7:0] } : 16'h0) |
		(data_rd_even ? { vram_rd_hi_lat[7:0], vram_rd_lo_lat[7:0] } : 16'h0) |
		(z80_int_ack ? { 8'h0, 5'h0, ipl2_src, ipl1_src, 1'h0 } : 16'h0) |
		(tst_fn2_rd ? { 8'h0, ~line_zero_dly, ~disp_start, ~vscr_active, ~cell_m4_active, ~cell_bound_active, ~m4_or_vram_slot, ~pre_wrap_active, ~sub_slot_active } : 16'h0) |
		(tst_fn3_rd ? { 2'h0, ~m4_window_dly, ~vram_or_ext_slot, ~blank_slot_active, ~fetch_all_dly, ~m4_border_dly, ~access_main_dly, ~access_ext_dly, ~odd_slot, ~slot0_active, ~slot1_active, ~slot2_active, ~slot3_active, ~edclk_dly, ~slot_idle_dly } : 16'h0) |
		(tst_fn6_rd ? { 4'h0, spridx_sr_active[19], spridx_sr_6[19], spridx_sr_5[19], spridx_sr_4[19], spridx_sr_3[19], spridx_sr_2[19], spridx_sr_1[19], spridx_sr_0[19], spr_cnt_sr_3[9], spr_cnt_sr_2[9], spr_cnt_sr_1[9], spr_cnt_sr_0[9] } : 16'h0) |
		(tst_fn8_rd ? { 1'h0, ~test_rd_idx3_odd, ~test_rd_idx2_odd, ~test_rd_idx1_odd, ~test_rd_idx0_odd, ~test_rd_pri_odd, ~test_rd_pal1_odd, ~test_rd_pal0_odd, 1'h0, ~test_rd_idx3_even, ~test_rd_idx2_even, ~test_rd_idx1_even, ~test_rd_idx0_even, ~test_rd_pri_even, ~test_rd_pal1_even, ~test_rd_pal0_even } : 16'h0) |
		(tst_fn4_rd ? { 5'h0, ~dac_b_bit1, ~dac_b_bit0, ~dac_b_bit2, ~dac_g_bit1, ~dac_g_bit0, ~dac_g_bit2, ~dac_r_bit1, ~dac_r_bit0, ~dac_r_bit2, ~sh_shadow_pipe_3, ~sh_highlight_pipe_3 } : 16'h0) |
		(tst_fn5_rd ? { ~psg_atten_0, ~psg_atten_1, ~psg_atten_2, ~psg_atten_3 } : 16'h0);*/
	
	assign CD_o = io_data;
	
	assign CD_d = vdp_data_dir;
	
	always @(posedge MCLK)
	begin
		io_data <= (io_data_pull & io_data_val) | (~io_data_pull & io_data);
		io_address[22:20] <= io_address_t[22:20];
		io_address[19:18] <= reg_sa_high[3:2];
		io_address[17:0] <= io_address_t[17:0];
	end
	
	// -------------------------------------------------------------------------
	// Color bus
	// -------------------------------------------------------------------------
	// 7-bit color bus: {priority, pal[1:0], index[3:0]}. Arbitrated between
	// background color, plane A/B, and sprite sources via the priority MUX.

	wire [6:0] color_bus;
	
	assign color_index = color_bus[3:0];
	assign color_pal = color_bus[5:4];
	assign color_priority = color_bus[6];
	
	wire [6:0] color_bus_val =
		(sel_bg_pipe ? { 1'h0, reg_m5 ? reg_col_pal : 2'h1, reg_col_index } : 7'h7f) &
		(sel_spr_pipe ? { spr_pri_pipe_2, spr_pal_pipe_2, spr_idx_pipe_2 } : 7'h7f) &
		(sel_b_pipe ? { l321, l323, l319 } : 7'h7f) &
		(sel_a_pipe ? { l274, l272, l270 } : 7'h7f);
	
	reg [6:0] color_bus_mem;
	
	assign color_bus = (sel_bg_pipe | sel_spr_pipe | sel_b_pipe | sel_a_pipe) ? color_bus_val : color_bus_mem;
	
	always @(posedge MCLK)
	begin
		color_bus_mem <= color_bus;
	end
	
	// -------------------------------------------------------------------------
	// Miscellaneous outputs
	// -------------------------------------------------------------------------
	// VDP status signals exported to fc1004 for system integration:
	// pixel clock, interlace field, display enable, mode flags, hsync,
	// DMA status, and the color RAM display pipeline (bypasses priority MUX
	// for direct CRAM dot output used by the MiSTer framework).

	assign vdp_hclk1 = hclk1;
	
	assign vdp_intfield = field_bit;
	
	wire [1:0] vdp_de_1 = { vdisp_en_trig, hdisp_en_trig };
	wire [1:0] vdp_de_delay_m5;
	
	ym_sr_bit_array #(.SR_LENGTH(8), .DATA_WIDTH(2)) vdp_de_delay_m5_sr(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vdp_de_1), .data_out(vdp_de_delay_m5));
	
	wire [1:0] vdp_de_2 = reg_m5 ? vdp_de_delay_m5 : vdp_de_1;
	wire [1:0] vdp_de_3;
	
	ym_sr_bit_array #(.SR_LENGTH(7), .DATA_WIDTH(2)) vdp_de_delay_sr(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(vdp_de_2), .data_out(vdp_de_3));
	
	assign vdp_de_h = vdp_de_3[0];
	assign vdp_de_v = vdp_de_3[1];
	
	assign vdp_m5 = reg_m5;
	assign vdp_rs1 = reg_rs1;
	assign vdp_m2 = v30_mode;
	assign vdp_lcb = reg_lcb;
	
	assign vdp_psg_clk1 = psg_hclk1;
	
	wire vdp_hsync2_delay1;
	ym_sr_bit #(.SR_LENGTH(2)) vdp_hsync2_delay1_sr(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(hsync_trig), .sr_out(vdp_hsync2_delay1));
	wire vdp_hsync2_delay2;
	ym_sr_bit #(.SR_LENGTH(7)) vdp_hsync2_delay2_sr(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vdp_hsync2_delay1), .sr_out(vdp_hsync2_delay2));
	wire vdp_hsync2_1 = reg_m5 ? vdp_hsync2_delay2 : vdp_hsync2_delay1;
	wire vdp_hsync2_delay3;
	ym_sr_bit vdp_hsync2_delay3_sr(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .bit_in(vdp_hsync2_1), .sr_out(vdp_hsync2_delay3));
	
	assign vdp_hsync2 = vdp_hsync2_delay3;
	
	assign color_bus_dp = { color_pal, color_index };
	ym_sr_bit_array #(.DATA_WIDTH(6)) sr617_dp(.MCLK(MCLK), .c1(hclk1), .c2(hclk2), .data_in(color_bus_dp), .data_out(color_bus_pipe_dp));
	
	always @(posedge MCLK) color_ram_out_dp <= color_ram[color_bus_pipe_dp];
	
	assign vdp_dma_oe_early = reg_8b_b6 ?
		(io_m1_dff2_l2 | cas_z80_gate | z80_cas_pulse | bus_phase2 | z80_cas_cond) :
		(bus_phase_a | bus_phase_c | oe_cpu_rd | cpu_wr_cas_gate);
	
	assign vdp_dma = bus_phase_a | bus_phase_c;

endmodule

// ym7101_rs_trig — RS (set/reset) flip-flop
//
// Priority: set > rst > hold. On each MCLK posedge:
//   set=1 → q=1, nq=0
//   rst=1 → q=0, nq=1
//   else  → q holds, nq = ~q
// Used throughout ym7101 for timing triggers (bus_br_hold-psg_noise_trig).
module ym7101_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output reg q = 1'h0,
	output reg nq = 1'h1
	);
	
	//reg mem = 1'h0;
	
	//assign q = set ? 1'h1 : (rst ? 1'h0 : mem);
	//assign nq = rst ? 1'h1 : (set ? 1'h0 : ~mem); 
	
	always @(posedge MCLK)
	begin
		q <= set ? 1'h1 : (rst ? 1'h0 : q);
		nq <= rst ? 1'h1 : (set ? 1'h0 : ~q);
	end
	
endmodule

/*module ym7101_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output q,
	output nq
	);
	
	reg mem = 1'h0;
	
	assign q = set | ~nq;
	assign nq = rst | ~q; 
	
endmodule*/


module ym7101_dff #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] inp,
	input rst,
	output [DATA_WIDTH-1:0] outp
	);
	
	reg [DATA_WIDTH-1:0] edclk_pipe1 = {DATA_WIDTH{1'h0}}, edclk_pipe2 = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] l2_assign = rst ? {DATA_WIDTH{1'h0}} : (clk ? edclk_pipe1 : edclk_pipe2);
	
	assign outp = l2_assign;
	//assign outp = edclk_pipe2;
	
	always @(posedge MCLK)
	begin
		if (rst)
		begin
			edclk_pipe1 <= {DATA_WIDTH{1'h0}};
		end
		else
		begin
			if (~clk)
				edclk_pipe1 <= inp;
		end
		edclk_pipe2 <= l2_assign;
	end
	
endmodule

/*module ym7101_dff #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] inp,
	input rst,
	output [DATA_WIDTH-1:0] outp
	);
	
	reg [DATA_WIDTH-1:0] edclk_pipe1 = {DATA_WIDTH{1'h0}}, edclk_pipe2 = {DATA_WIDTH{1'h0}};
	
	//assign outp = l2_assign;
	assign outp = edclk_pipe2;
	
	always @(*)
	begin
		if (rst)
		begin
			edclk_pipe1 <= {DATA_WIDTH{1'h0}};
			edclk_pipe2 <= {DATA_WIDTH{1'h0}};
		end
		else
		begin
			if (~clk)
				edclk_pipe1 <= inp;
			else
				edclk_pipe2 <= edclk_pipe1;
		end
	end
	
endmodule*/

/*module ym7101_dff #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] inp,
	input rst,
	output [DATA_WIDTH-1:0] outp
	);
	
	reg [DATA_WIDTH-1:0] edclk_pipe2 = {DATA_WIDTH{1'h0}};

	assign outp = edclk_pipe2;
	
	always @(posedge clk or posedge rst)
	begin
		if (rst)
			edclk_pipe2 <= {DATA_WIDTH{1'h0}};
		else
			edclk_pipe2 <= inp;
	end
	
endmodule*/

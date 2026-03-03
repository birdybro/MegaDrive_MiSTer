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
 *  YM6045C(FC1004) emulator — Bus Arbiter
 *  Thanks:
 *      org (ogamespec):
 *          FC1004 decap and die shot.
 *      andkorzh, HardWareMan (emu-russia):
 *          help & support.
 *
 *  Handles 68K/Z80 bus arbitration, address decode (chip selects for ROM,
 *  RAM, VDP, I/O, sound, timers), DTACK generation, DRAM refresh, Z80 bank
 *  register, reset/NMI sequencing, and VDP access timing pipeline.
 */
module ym6045
	(
	input MCLK,
	input MCLK_e,
	input VCLK,
	input ZCLK,
	input VD8_i,
	input [15:7] ZA_i,
	input ZA0_i,
	input [22:7] VA_i,
	input ZRD_i,
	input M1,
	input ZWR_i,
	input BGACK_i,
	input BG,
	input IORQ,
	input RW_i,
	input UDS_i,
	input AS_i,
	input DTACK_i,
	input LDS_i,
	input CAS0,
	input M3,
	input WRES,
	input CART,
	input OE0,
	input WAIT_i,
	input ZBAK,
	input MREQ_i,
	input FC0,
	input FC1,
	input SRES,
	input test_mode_0,
	input ZD0_i,
	input HSYNC,
	output VD8_o,
	output ZA0_o,
	output [15:8] ZA_o,
	output [22:7] VA_o,
	output ZRD_o,
	output UDS_o,
	output ZWR_o,
	output BGACK_o,
	output AS_o,
	output RW_d,
	output RW_o,
	output LDS_o,
	output strobe_dir,
	output DTACK_o,
	output BR,
	output IA14,
	output TIME,
	output CE0,
	output FDWR,
	output FDC,
	output ROM,
	output ASEL,
	output EOE,
	output NOE,
	output RAS2,
	output CAS2,
	output REF,
	output ZRAM,
	output WAIT_o,
	output ZBR,
	output NMI,
	output ZRES,
	output SOUND,
	output VZ,
	output MREQ_o,
	output VRES,
	output VPA,
	output VDPM,
	output IO,
	output ZV,
	output INTAK,
	output EDCLK,
	output vtoz,
	output VD8_OE_n,
	output VA_MID_OE_n,
	output VA_HI_OE_n,
	output VSYNC_TEST,
	output YS_TEST
	);

	wire pal_trap = ~1'h1; // PAL trap — disabled (active-low, always 0)

	// =======================================================================
	// EDCLK generation — divide-by-6 counter + 4-bit prescaler
	//   Divides MCLK to produce EDCLK for external devices.
	//   Three counter bits (div1-div3) form a divide-by-6 from MCLK_e.
	//   Four prescaler bits (pre1-pre4) further divide, gated by HSYNC.
	// =======================================================================
	wire edclk_div1_nq;
	wire edclk_div2_q, edclk_div2_nq;
	wire edclk_div3_q, edclk_div3_nq;

	wire edclk_pre1_nq, edclk_pre1_cout;
	wire edclk_pre2_nq, edclk_pre2_cout;
	wire edclk_pre3_nq, edclk_pre3_cout;
	wire edclk_pre4_nq;
	wire edclk_ovf_nq;          // prescaler overflow (set when all pre bits high)
	wire edclk_hsync_q;         // HSYNC-gated latch for prescaler reload
	wire edclk_cnt_nz;          // counter is not all zeros (load condition)
	wire edclk_phase;           // EDCLK output phase (= div3)
	wire edclk_carry;           // carry from div2 & div3
	wire edclk_pre_clk;         // prescaler clock (= edclk_phase)
	wire edclk_pre_full;        // all prescaler bits are high
	wire edclk_ovf_rst;         // overflow reset (= edclk_ovf_nq)
	wire edclk_hsync_val;       // HSYNC sample value for prescaler
	wire edclk_hsync_in;        // HSYNC input gating

	// =======================================================================
	// RAM OE / BGACK pipeline
	//   Controls external RAM output enables (NOE, EOE) based on BGACK
	//   handshake state. Multi-stage pipeline synchronizes BGACK to VCLK.
	// =======================================================================
	wire sres_int;              // internal SRES alias
	wire bgack_n;               // inverted BGACK_i
	wire ram_oe_dis;            // RAM OE disable condition
	wire ram_oe_en;             // RAM OE enable condition
	wire bgack_sync1_q, bgack_sync1_nq;  // BGACK sync stage 1
	wire bgack_sync2_q, bgack_sync2_nq;  // BGACK sync stage 2
	wire bgack_wait_q;          // BGACK wait state
	wire cas0_bgack_pipe;       // CAS0 gated by BGACK pipeline
	wire oe0_gate;              // OE0 gating for NOE
	wire noe_combine;           // combined NOE terms
	wire noe_raw;               // raw NOE value
	wire noe_active;            // active NOE output
	wire bgack_pipe_val;        // BGACK pipeline value (= bgack_pipe2_q)
	wire bgack_pipe_in;         // BGACK pipeline input (= bgack_sync1_q)
	wire bgack_pipe1_q, bgack_pipe1_nq;  // BGACK pipeline stage 1
	wire bgack_pipe2_q;         // BGACK pipeline stage 2

	// =======================================================================
	// Bus arbitration — 68K ↔ Z80 ownership
	//   Manages bus ownership between 68K and Z80 processors.
	//   z80_has_bus indicates Z80 currently owns the main bus.
	//   Multi-stage pipeline generates properly timed control signals.
	// =======================================================================
	wire br_latch_q;            // bus request output latch
	wire z80_upper_gate;        // Z80 upper address qualification
	wire z80_bank_gate;         // Z80 bank window qualification
	wire z80_bus_req_raw;       // raw Z80 bus request
	wire z80_bus_req_sync;      // Z80 bus request synced to ZCLK
	wire z80_has_bus;           // Z80 currently owns bus
	wire dtack_wait_raw;        // DTACK wait condition
	wire wait_active;           // WAIT output active
	wire br_val;                // bus request value
	wire bus_arb_state;         // bus arbitration state
	wire arb_pipe1_q;           // arbitration pipeline stage 1
	wire arb_pipe1_or_z80;      // stage 1 OR z80_has_bus
	wire arb_pipe2_q;           // arbitration pipeline stage 2
	wire arb_pipe2_or_z80;      // stage 2 OR z80_has_bus
	wire arb_pipe3_q;           // arbitration pipeline stage 3
	wire z80_rd_gate;           // Z80 read strobe gate
	wire z80_wr_gate;           // Z80 write strobe gate
	wire z80_strobe_gate;       // Z80 combined strobe gate
	wire bgack_out_en;          // BGACK output enable
	wire bus_released;          // bus released (reset active OR arb granted)
	wire bus_released_2;        // alias for bus_released
	wire as_active;             // address strobe active (= ~AS_i)
	wire bgack_active;          // BGACK active (= ~BGACK_i)
	wire arb_request;           // arbitration request conditions met
	wire arb_next;              // next arbitration state
	wire bus_arb_q;             // bus arbitration state FF
	wire bus_released_3;        // alias for bus_released
	wire strobe_en;             // strobe direction enable

	// =======================================================================
	// BGACK feedback loop
	// =======================================================================
	reg bgack_fb_mem;           // BGACK feedback memory
	wire bgack_fb;              // BGACK feedback
	wire bgack_or_fb;           // BGACK_i OR feedback
	wire bgack_fb_n;            // inverted feedback

	// =======================================================================
	// VDP match
	// =======================================================================
	wire vdp_match_term;        // VDP address match intermediate
	wire vdp_match;             // VDP address matched

	// =======================================================================
	// Reset/NMI delay chain
	//   Long delay chain (9 toggle FFs) for reset sequencing.
	//   Generates properly timed VRES, ZRES, and NMI signals.
	// =======================================================================
	wire rst_chain_gate;        // reset chain clock gate
	wire wres_latch_q;          // WRES (warm reset) latch
	wire wres_active;           // warm reset active
	wire wres_n;                // inverted WRES input
	wire rst_dly1_q, rst_dly1_nq;    // reset delay stage 1
	wire rst_dly2_q, rst_dly2_nq;    // reset delay stage 2
	wire rst_dly3_q, rst_dly3_nq;    // reset delay stage 3
	wire rst_dly4_q, rst_dly4_nq;    // reset delay stage 4
	wire rst_chain_clk2;        // secondary chain clock
	wire rst_dly5_q, rst_dly5_nq;    // reset delay stage 5
	wire rst_dly6_q, rst_dly6_nq;    // reset delay stage 6
	wire rst_dly7_q, rst_dly7_nq;    // reset delay stage 7
	wire rst_dly8_q, rst_dly8_nq;    // reset delay stage 8
	wire rst_dly9_q, rst_dly9_nq;    // reset delay stage 9
	wire nmi_q;                 // NMI output latch
	wire vres_dly1_q;           // VRES delay stage 1
	wire vres_dly2_nq;          // VRES delay stage 2 (inverted)
	wire wres_sample_q, wres_sample_nq; // WRES sample FF
	wire vres_combined;         // combined VRES condition

	// =======================================================================
	// Z80 IORQ / M1 pipeline
	// =======================================================================
	wire z80_iorq_raw;          // Z80 IORQ qualification (IORQ | M3 | ~M1)
	wire z80_iorq_sync_q;       // IORQ synced to ZCLK
	wire z80_iorq_active;       // IORQ access active
	wire m1_sync1_q;            // M1 sync stage 1
	wire m1_sync2_q;            // M1 sync stage 2
	wire m1_delayed;            // M1 delayed (alias)
	wire m1_delayed_2;          // M1 delayed (alias 2)
	wire m1_pipe_q, m1_pipe_nq; // M1 pipeline output
	wire z80_inactive;          // Z80 bus inactive (MREQ | ~M1 delayed)

	// =======================================================================
	// Miscellaneous control signals
	// =======================================================================
	wire sres;                  // SRES alias
	wire vd8;                   // VD8 alias
	wire mreq_in;               // MREQ alias
	wire test;                  // test mode alias

	wire uds_prev_q;            // UDS sampled on falling VCLK
	wire uds_falling;           // UDS falling edge detect (NAND)
	wire za0_out_val;           // ZA0 output value

	wire zbr_nq;                // Z80 bus request latch (inverted)
	wire zbak_in;               // ZBAK input alias
	wire zbr_or_zbak;           // ZBR OR ZBAK

	wire strobe_active;         // data strobe active (LDS or UDS low)
	wire strobe_sync_q;         // strobe synced to VCLK
	wire mreq_raw;              // raw MREQ output value

	wire zbr_write_gate;        // Z80 bus request write gate
	wire zbr_clk;               // Z80 bus request register clock

	wire uds_rw_active;         // UDS and RW both active (both low)
	wire zbr_wr_clk;            // ZBR write clock (VA bank 2)
	wire z80_wr_clk;            // Z80 write clock (VA bank 0)

	// =======================================================================
	// VDP/AS timing pipeline
	//   Multi-stage pipeline for VDP access timing and DTACK generation.
	// =======================================================================
	wire z80_region_as;         // Z80 region + AS qualification
	wire as_timing1;            // AS timing intermediate 1
	wire as_timing2_nq;         // AS timing intermediate 2
	wire as_timing3;            // AS timing intermediate 3
	wire as_pipe1_q, as_pipe1_nq;  // AS pipeline stage 1
	wire as_sync_q;             // AS synced to VCLK
	wire as_combined;           // combined AS condition
	wire as_pipe2_nq;           // AS pipeline stage 2 (inverted)
	wire as_dtack_gate;         // AS-based DTACK gate
	wire dtack_region_as;       // DTACK region + AS combined

	// =======================================================================
	// VDP bus arbitration
	// =======================================================================
	wire ref_fc_term;           // refresh/function-code term
	wire ref_sres_term;         // refresh/SRES term
	wire as_arb_gate;           // AS arbitration gate
	wire arb_next_val;          // next arbitration value
	wire zres_gate;             // ZRES gate condition
	wire vdp_arb1_q, vdp_arb1_nq;  // VDP arbitration FF
	wire as_pending;            // AS pending (access in progress)
	wire as_pending_sync_q;     // AS pending synced to falling VCLK

	// =======================================================================
	// Interrupt acknowledge
	// =======================================================================
	wire intak_n;               // interrupt acknowledge (active-low)

	// =======================================================================
	// CE0 generation — cartridge ROM chip enable
	//   Complex logic combining address decode, bank mode, VDP pipeline
	//   state, and cart presence to generate the CE0 chip select.
	// =======================================================================
	wire m3_sres_rst;           // M3 AND synced SRES (reset for VDP pipeline)
	wire bank_mode_nq;          // bank mode register (inverted)
	wire cas0_n;                // inverted CAS0
	wire ce0_addr_latch_q;      // CE0 address latch
	wire ce0_addr_term;         // CE0 address term
	wire ce0_combine;           // CE0 combined condition
	wire ce0_bank_mux;          // CE0 bank/address mux
	wire ce0_cart_gate;         // CE0 cart gate
	wire ce0_timing_gate;       // CE0 timing gate
	wire ce0_timing_cart;       // CE0 timing + cart combined
	wire not_68k_access;        // not a 68K access (~M3 | AS_i | va23_in)
	wire vdp_pipe_gate;         // VDP pipeline gate for CE0
	wire ce0_vdp_term;          // CE0 VDP term
	wire vdp_pipe_valid;        // VDP pipeline access valid
	wire ce0_vdp_bank;          // CE0 VDP bank term
	wire ce0_vdp_cart;          // CE0 VDP cart term
	wire vdp_pipe_sync_q;       // VDP pipe synced to falling VCLK
	wire vdp_pipe_en;           // VDP pipeline enable
	wire ce0_bank_term;         // CE0 bank term
	wire z80_ce0_gate;          // Z80 CE0 qualification
	wire ce0_active;            // CE0 final active signal

	// =======================================================================
	// ROM / RAS2 / CAS2 generation
	// =======================================================================
	wire rom_timing_gate;       // ROM timing gate
	wire rom_addr_term;         // ROM address term
	wire rom_timing_q, rom_timing_nq;  // ROM timing latch
	wire rom_timing_comb;       // ROM timing combined
	wire z80_region_gate;       // Z80 region gate (z80_access_gate | va23_in)
	wire ras2_addr_term;        // RAS2 address term
	wire cas0_or_z80gate;       // CAS0 OR Z80 access gate
	wire ras2_vdp_term;         // RAS2 VDP term
	wire ras2_timing_term;      // RAS2 timing term
	wire ras2_active;           // RAS2 active
	wire rom_addr_term1;        // ROM address term 1
	wire va22_cart;              // VA22 XOR CART
	wire rom_addr_term2;        // ROM address term 2
	wire rom_sel;               // ROM select
	wire z80_ram_sel;           // Z80 RAM region select
	wire cas2_active;           // CAS2 active
	wire asel_active;           // ASEL active

	// =======================================================================
	// VDP timing pipeline — multi-stage address/timing latch
	//   Controls the timing of VDP accesses to ensure proper setup/hold.
	// =======================================================================
	wire not_68k_as;            // ~M3 | AS_i
	wire vdp_addr_chk1;        // VDP address check 1
	wire vdp_addr_chk2;        // VDP address check 2
	wire vdp_access_valid;     // VDP access valid
	wire vdp_pipe_s1_q;        // VDP pipe stage 1
	wire vdp_pipe_s1_val;      // stage 1 value (alias)
	wire vdp_pipe_s1_val2;     // stage 1 value (alias 2)
	wire vdp_pipe_s2_q, vdp_pipe_s2_nq;  // VDP pipe stage 2
	wire vdp_pipe_s2_gate;     // stage 2 gated
	wire vdp_pipe_s3_q;        // VDP pipe stage 3
	wire vdp_pipe_s3_gate;     // stage 3 gate
	wire vdp_pipe_comb;        // VDP pipe combined
	wire vdp_pipe_sync1_q;     // VDP pipe sync latch
	wire vdp_pipe_ext;         // VDP pipe extended
	wire vdp_pipe_dly1_q;      // VDP pipe delay
	wire vdp_pipe_or;          // VDP pipe OR
	wire vdp_pipe_with_s4;     // VDP pipe with stage 4
	wire vdp_pipe_latch_q, vdp_pipe_latch_nq;  // VDP pipe latch
	wire vdp_pipe_nand;        // VDP pipe NAND gate
	wire vdp_addr_gate;        // VDP address gate
	wire vdp_pipe_val;         // VDP pipe value
	wire vdp_pipe_s4_q, vdp_pipe_s4_nq;  // VDP pipe stage 4
	wire vdp_pipe_rw;          // VDP pipe read/write active

	// =======================================================================
	// DRAM refresh counters
	//   Two 4-bit counter chains (low + high) generate periodic REF cycles.
	//   ref_match triggers when the full 8-bit count reaches its target.
	// =======================================================================
	wire ref_load_raw;          // refresh counter load (raw)
	wire ref_load;              // refresh counter load
	wire ref_cnt_cin;           // refresh counter carry-in
	wire ref_lo1_nq, ref_lo1_cout;  // refresh counter low bit 1
	wire ref_lo2_nq, ref_lo2_cout;  // refresh counter low bit 2
	wire ref_lo3_nq, ref_lo3_cout;  // refresh counter low bit 3
	wire ref_lo4_nq, ref_lo4_cout;  // refresh counter low bit 4
	wire ref_lo_full;           // all low counter bits high
	wire ref_lo_match;          // low counter match
	wire ref_hi_cin;            // high counter carry-in
	wire ref_hi1_nq, ref_hi1_cout;  // refresh counter high bit 1
	wire ref_hi2_nq, ref_hi2_cout;  // refresh counter high bit 2
	wire ref_hi3_nq, ref_hi3_cout;  // refresh counter high bit 3
	wire ref_hi4_nq, ref_hi4_cout;  // refresh counter high bit 4
	wire ref_hi_full;           // all high counter bits high
	wire ref_match;             // full refresh counter match

	// =======================================================================
	// VDP FIFO / arbitration state machine
	// =======================================================================
	wire vdp_fifo_ready;        // VDP FIFO ready for access
	wire vdp_arb_val;           // VDP arbitration next value
	wire vdp_arb_q, vdp_arb_nq;      // VDP arbitration state
	wire vdp_access_q, vdp_access_nq; // VDP access state

	// =======================================================================
	// Chip select / FDC / DTACK signals
	// =======================================================================
	wire fdc_sel;               // FDC chip select (AS | fdc_region_n)
	wire fdwr_val;              // FDWR next value
	wire fdwr_latch_q, fdwr_latch_nq; // FDWR latch
	wire io_sel;                // I/O chip select (io_port_n | AS)
	wire io_active;             // I/O access active
	wire time_sel;              // TIME chip select
	wire strobe_dir_raw;        // strobe direction raw
	wire zres_latch_q;          // ZRES latch
	wire zres_val;              // ZRES value
	wire sres_syncv_q, sres_syncv_nq;  // SRES synced to VCLK
	wire vdp_as_sel;            // VDP address + AS select
	wire dtack_any_sel;         // any chip select active (for DTACK)
	wire dtack_with_intak;      // DTACK combined with INTAK
	wire dtack_active;          // DTACK active
	wire va21_mreq;             // VA21 OR MREQ
	wire z80_access_gate;       // Z80 access gate (bgack_fb_n | BGACK | ~M3)
	wire wait_in;               // WAIT input alias
	wire vres_raw;              // raw VRES value

	// =======================================================================
	// Function code decode and gating
	// =======================================================================
	wire fc00_gate;             // FC=00 gated by M3 delay (data/user)
	wire fc01_gate;             // FC=01 gated by M3 delay (data/supervisor)
	wire fc10_gate;             // FC=10 gated by M3 delay (program/user)
	wire nmi_gate;              // NMI gating signal
	wire nmi_sres_term;         // NMI SRES term
	wire fc00, fc01, fc10, fc11; // decoded function codes

	// =======================================================================
	// Address aliases
	// =======================================================================
	wire va14_in;               // VA[13] alias (directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low note: VA_i[13] = address bit 14)
	wire va21_in;               // VA[20] alias (address bit 21)
	wire va22_in;               // VA[21] alias (address bit 22)
	wire va23_in;               // VA[22] alias (address bit 23)
	wire za15_in;               // ZA[15] alias

	// =======================================================================
	// Address decode — VA[8:7] bank select
	// =======================================================================
	wire va_bank_0;             // VA[8:7] == 0
	wire va_bank_1;             // VA[8:7] == 1
	wire va_bank_2;             // VA[8:7] == 2
	wire va_bank_3;             // VA[8:7] == 3 (unused)

	// =======================================================================
	// Address decode — chip selects (active-low unless noted)
	// =======================================================================
	wire tmss_io_access;        // TMSS I/O region ($A14000, active when no match)
	wire z80_region_n;          // Z80 bus region ($A00000-$A0FFFF)
	wire io_port_n;             // I/O port region ($A10000-$A1007F)
	wire vdp_region_n;          // VDP FIFO region ($A10880-$A108FF)
	wire fdc_region_n;          // FDC region ($A12000-$A1207F)
	wire time_region_n;         // TIME region ($A13000-$A1307F)
	wire zram_n;                // Z80 RAM (ZA[15:14] == 0, $0000-$3FFF)
	wire sound_n;               // Sound/YM2612 (ZA[15:13] == 2, $4000-$5FFF)
	wire z80bank_wr_n;          // Z80 bank register write (ZA = $60xx)
	wire z80_window_n;          // Z80 window into 68K space (ZA = $7Fxx)

	// DRAM refresh
	wire ref_m3_gate;           // REF gate for M3 mode
	wire ref_z80_gate;          // REF gate for Z80 mode
	wire ref_active;            // REF cycle active

	// =======================================================================
	// Z80 bank register and VA output
	// =======================================================================
	wire [8:0] z80bank_q;      // 9-bit Z80 bank register (shift register)
	wire [15:0] va_out;        // VA output bus
	reg edclk_buf;              // EDCLK output buffer

	// =======================================================================
	// EDCLK divide-by-6 counter
	//   Counts MCLK_e edges. When counter reaches 0 (all bits low), it
	//   reloads. edclk_phase (div3) is the output, toggling every 3 MCLK_e.
	// =======================================================================
	/*always @(posedge MCLK)
	begin
		if (!sres)
		begin
			dff1 <= 1'h0;
			dff2 <= 1'h0;
			dff3 <= 1'h0;
		end
		else
		begin
			if (~edclk_cnt_nz)
			begin
				dff1 <= 1'h1;
				dff2 <= ~edclk_hsync_q;
				dff3 <= 1'h0;
			end
			else
			begin
				dff1 <= dff1 ^ edclk_carry;
				dff2 <= ~dff2;
				dff3 <= dff3 ^ dff2;
			end
		end

		edclk_buf <= edclk_phase;
	end*/

	ym_scnt_bit dff1(.MCLK(MCLK), .clk(MCLK_e), .load(edclk_cnt_nz), .val(1'h1), .cin(edclk_carry), .rst(sres), .nq(edclk_div1_nq));
	ym_scnt_bit dff2(.MCLK(MCLK), .clk(MCLK_e), .load(edclk_cnt_nz), .val(~edclk_hsync_q), .cin(1'h1), .rst(sres), .q(edclk_div2_q), .nq(edclk_div2_nq));
	ym_scnt_bit dff3(.MCLK(MCLK), .clk(MCLK_e), .load(edclk_cnt_nz), .val(1'h0), .cin(edclk_div2_q), .rst(sres), .q(edclk_div3_q), .nq(edclk_div3_nq));

	always @(posedge MCLK)
	begin
		edclk_buf <= edclk_phase;
	end

	assign edclk_cnt_nz = ~(edclk_div1_nq & edclk_div2_nq & edclk_div3_nq);
	assign edclk_phase = edclk_div3_q;
	assign edclk_carry = edclk_div2_q & edclk_div3_q;
	assign edclk_pre_clk = edclk_phase;
	assign edclk_pre_full = ~(edclk_pre1_nq | edclk_pre2_nq | edclk_pre3_nq | edclk_pre4_nq);

	// 4-bit prescaler — counts edclk_phase edges, reloads on HSYNC gate
	ym_scnt_bit dff4(.MCLK(MCLK), .clk(edclk_pre_clk), .load(edclk_hsync_q), .val(1'h1), .cin(edclk_hsync_q), .rst(sres), .nq(edclk_pre1_nq), .cout(edclk_pre1_cout));
	ym_scnt_bit dff5(.MCLK(MCLK), .clk(edclk_pre_clk), .load(edclk_hsync_q), .val(1'h0), .cin(edclk_pre1_cout), .rst(sres), .nq(edclk_pre2_nq), .cout(edclk_pre2_cout));
	ym_scnt_bit dff6(.MCLK(MCLK), .clk(edclk_pre_clk), .load(edclk_hsync_q), .val(1'h0), .cin(edclk_pre2_cout), .rst(sres), .nq(edclk_pre3_nq), .cout(edclk_pre3_cout));
	ym_scnt_bit dff7(.MCLK(MCLK), .clk(edclk_pre_clk), .load(edclk_hsync_q), .val(1'h0), .cin(edclk_pre3_cout), .rst(sres), .nq(edclk_pre4_nq));

	assign EDCLK = edclk_buf;

	// HSYNC gating — prescaler reloads when HSYNC detected and overflow reached
	assign edclk_ovf_rst = edclk_ovf_nq;
	assign edclk_hsync_in = ~(~HSYNC | edclk_hsync_q);
	assign edclk_hsync_val = ~(edclk_hsync_in | (1'h0 & edclk_hsync_q));

	ym_sdffr dff9(.MCLK(MCLK), .clk(edclk_phase), .val(edclk_hsync_val), .reset(edclk_ovf_rst), .q(edclk_hsync_q));
	ym_sdffs dff8(.MCLK(MCLK), .clk(edclk_phase), .val(edclk_pre_full), .set(sres), .nq(edclk_ovf_nq));

	// =======================================================================
	// RAM OE control
	//   Generates NOE (normal OE) and EOE (even OE) based on BGACK pipeline.
	// =======================================================================
	assign sres_int = sres;

	assign bgack_n = ~BGACK_i;
	assign ram_oe_dis = ~(sres_int & (bgack_sync2_nq | bgack_pipe2_q));
	assign ram_oe_en = ~ram_oe_dis;

	ym_sdffr dff49(.MCLK(MCLK), .clk(VCLK), .val(bgack_n), .reset(bgack_wait_q), .q(bgack_sync1_q), .nq(bgack_sync1_nq));
	ym_sdffr dff50(.MCLK(MCLK), .clk(~VCLK), .val(bgack_pipe_val), .reset(sres_int), .q(bgack_sync2_q), .nq(bgack_sync2_nq));
	ym_sdffr dff51(.MCLK(MCLK), .clk(bgack_n), .val(wait_in), .reset(ram_oe_en), .q(bgack_wait_q));

	assign cas0_bgack_pipe = CAS0 & bgack_pipe2_q;
	assign oe0_gate = bgack_pipe1_nq & OE0;
	assign noe_combine = oe0_gate | cas0_bgack_pipe;
	assign noe_raw = ~(noe_combine & (bgack_sync1_nq | bgack_sync2_q));
	assign noe_active = ~noe_raw;
	assign NOE = noe_active;
	assign EOE = ~(~noe_active & M3);

	assign bgack_pipe_val = bgack_pipe2_q;

	assign bgack_pipe_in = bgack_sync1_q;

	ym_sdffr dff61(.MCLK(MCLK), .clk(~VCLK), .val(bgack_pipe_in), .reset(bgack_wait_q), .q(bgack_pipe1_q), .nq(bgack_pipe1_nq));

	ym_sdffr dff62(.MCLK(MCLK), .clk(VCLK), .val(bgack_pipe1_q), .reset(bgack_wait_q), .q(bgack_pipe2_q));

	// =======================================================================
	// Delay chains — propagation delay modeling
	// =======================================================================

	wire d1_out;  // M1 delayed 1 cycle
	wire d2_out;  // z80_bus_req_raw delayed 1 cycle
	wire d3_out;  // z80_region_gate delayed 7 cycles (CE0/ASEL timing)
	wire d4_out;  // z80_iorq_raw delayed 1 cycle
	wire d5_out;  // rom_timing_comb delayed 2 cycles
	wire d6_out;  // cas0_or_z80gate delayed 6 cycles
	wire d7_out;  // z80_access_gate delayed 6 cycles
	wire d8_out;  // M3 delayed 1 cycle (function code gating)

	ym_delaychain #(.DELAY_CNT(1)) d1(.MCLK(MCLK), .inp(M1), .outp(d1_out));
	ym_delaychain #(.DELAY_CNT(1)) d2(.MCLK(MCLK), .inp(z80_bus_req_raw), .outp(d2_out));
	ym_delaychain #(.DELAY_CNT(7)) d3(.MCLK(MCLK), .inp(z80_region_gate), .outp(d3_out));
	ym_delaychain #(.DELAY_CNT(1)) d4(.MCLK(MCLK), .inp(z80_iorq_raw), .outp(d4_out));
	ym_delaychain #(.DELAY_CNT(2)) d5(.MCLK(MCLK), .inp(rom_timing_comb), .outp(d5_out));
	ym_delaychain #(.DELAY_CNT(6)) d6(.MCLK(MCLK), .inp(cas0_or_z80gate), .outp(d6_out));
	ym_delaychain #(.DELAY_CNT(6)) d7(.MCLK(MCLK), .inp(z80_access_gate), .outp(d7_out));
	ym_delaychain #(.DELAY_CNT(1)) d8(.MCLK(MCLK), .inp(M3), .outp(d8_out));

	// =======================================================================
	// Bus arbitration — Z80 bus request and ownership
	//   z80_has_bus is the key signal: when high, Z80 owns the main bus and
	//   drives VA, AS, UDS, LDS, RW through the arbiter.
	// =======================================================================

	assign z80_upper_gate = ~M3 | z80_inactive | ~ZA_i[15];
	assign z80_bank_gate = z80_window_n | z80_inactive;
	assign z80_bus_req_raw = z80_bank_gate & z80_upper_gate;
	ym_sdff dff34(.MCLK(MCLK), .clk(ZCLK), .val(d2_out), .q(z80_bus_req_sync));
	assign z80_has_bus = z80_bus_req_raw & z80_bus_req_sync;
	assign dtack_wait_raw = ~(DTACK_i | bus_arb_state);
	assign wait_active = ~(dtack_wait_raw | pal_trap | z80_has_bus);
	assign WAIT_o = ~wait_active;
	assign br_val = ~bus_arb_state | z80_has_bus | ~sres;
	assign bus_arb_state = bus_arb_q | z80_has_bus | ~sres;
	ym_sdff dff10(.MCLK(MCLK), .clk(VCLK), .val(br_val), .q(br_latch_q));
	assign BR = br_latch_q;
	ym_sdff dff28(.MCLK(MCLK), .clk(VCLK), .val(bus_arb_state), .q(arb_pipe1_q));
	assign arb_pipe1_or_z80 = arb_pipe1_q | z80_has_bus;
	ym_sdff dff22(.MCLK(MCLK), .clk(VCLK), .val(arb_pipe1_or_z80), .q(arb_pipe2_q));
	assign arb_pipe2_or_z80 = arb_pipe2_q | z80_has_bus;
	ym_sdff dff18(.MCLK(MCLK), .clk(VCLK), .val(arb_pipe2_or_z80), .q(arb_pipe3_q));

	// Z80 bus strobes — directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low gated by arbitration pipeline
	assign z80_rd_gate = arb_pipe2_or_z80 | ZRD_i;
	assign z80_wr_gate = arb_pipe3_q | ZWR_i;
	assign z80_strobe_gate = z80_rd_gate & z80_wr_gate;
	assign UDS_o = z80_strobe_gate | ZA0_i;      // UDS driven when A0=0 (even byte)
	assign LDS_o = z80_strobe_gate | ~ZA0_i;     // LDS driven when A0=1 (odd byte)
	assign AS_o = arb_pipe2_or_z80;
	assign bgack_out_en = ~(test | pal_trap | bus_released_3);
	assign bus_released = ~sres | bus_arb_q;
	assign ztov = bus_released & M3;
	assign bus_released_2 = bus_released;
	assign as_active = ~AS_i;
	assign bgack_active = ~BGACK_i;
	assign arb_request = as_active | bgack_active | z80_has_bus | BG;
	assign arb_next = arb_request & bus_arb_state;
	ym_sdff dff21(.MCLK(MCLK), .clk(~VCLK), .val(arb_next), .q(bus_arb_q));
	assign bus_released_3 = bus_released;
	assign strobe_en = ~(test | pal_trap | bus_released_3);
	assign RW_d = bus_released_3 | test;
	assign strobe_dir = ~strobe_en;
	assign BGACK_o = ~bgack_out_en;

	// BGACK feedback loop
	assign bgack_fb = bgack_or_fb & ztov;
	assign bgack_or_fb = bgack_fb_mem | BGACK_i;

	always @(posedge MCLK)
	begin
		bgack_fb_mem <= bgack_fb;
	end

	assign bgack_fb_n = ~bgack_fb;

	// VDP match — detects VDP access pattern on data bus
	assign vdp_match_term = vd8 | mreq_in | va22_in | M3;
	assign vdp_match = ~(vd8 | vdp_match_term);
	assign VDPM = ~vdp_match;

	// =======================================================================
	// Reset/NMI delay chain
	//   9-stage toggle flip-flop chain creates a long delay from SRES/WRES
	//   to the actual reset/NMI outputs. This ensures proper sequencing of
	//   the reset process across the 68K and Z80 processors.
	// =======================================================================
	assign rst_chain_gate = ~(vdp_arb_nq | fc10_gate);
	ym_sdffr dff60(.MCLK(MCLK), .clk(~rst_chain_gate), .val(wres_sample_q), .reset(sres_syncv_q), .q(wres_latch_q));
	assign wres_active = ~(wres_latch_q | wres_sample_nq);

	assign wres_n = ~WRES;

	// Toggle flip-flop chain (each stage toggles on falling edge of previous)
	ym_sdffr dff68(.MCLK(MCLK), .clk(~rst_chain_gate), .val(rst_dly1_nq), .reset(~sres_syncv_nq), .q(rst_dly1_q), .nq(rst_dly1_nq));
	ym_sdffr dff71(.MCLK(MCLK), .clk(~rst_dly1_q), .val(rst_dly2_nq), .reset(~sres_syncv_nq), .q(rst_dly2_q), .nq(rst_dly2_nq));
	ym_sdffr dff72(.MCLK(MCLK), .clk(~rst_dly2_q), .val(rst_dly3_nq), .reset(~sres_syncv_nq), .q(rst_dly3_q), .nq(rst_dly3_nq));
	ym_sdffr dff76(.MCLK(MCLK), .clk(~rst_dly3_q), .val(rst_dly4_nq), .reset(~sres_syncv_nq), .q(rst_dly4_q), .nq(rst_dly4_nq));
	assign rst_chain_clk2 = fc01_gate | rst_dly4_q;
	ym_sdffr dff63(.MCLK(MCLK), .clk(~rst_chain_clk2), .val(rst_dly5_nq), .reset(~sres_syncv_nq), .q(rst_dly5_q), .nq(rst_dly5_nq));
	ym_sdffr dff52(.MCLK(MCLK), .clk(~rst_dly5_q), .val(rst_dly6_nq), .reset(~sres_syncv_nq), .q(rst_dly6_q), .nq(rst_dly6_nq));
	ym_sdffr dff65(.MCLK(MCLK), .clk(~rst_dly6_q), .val(rst_dly7_nq), .reset(~sres_syncv_nq), .q(rst_dly7_q), .nq(rst_dly7_nq));
	ym_sdffr dff67(.MCLK(MCLK), .clk(~rst_dly7_q), .val(rst_dly8_nq), .reset(~sres_syncv_nq), .q(rst_dly8_q), .nq(rst_dly8_nq));
	ym_sdffr dff74(.MCLK(MCLK), .clk(~rst_dly8_q), .val(rst_dly9_nq), .reset(sres_syncv_q), .q(rst_dly9_q), .nq(rst_dly9_nq));

	// NMI and VRES generation (clocked by final delay stage)
	ym_sdffs nmi(.MCLK(MCLK), .clk(rst_dly9_q), .val(va23_in), .set(nmi_gate), .q(nmi_q));
	ym_sdffr dff57(.MCLK(MCLK), .clk(rst_dly9_q), .val(sres_syncv_q), .reset(sres_syncv_q), .q(vres_dly1_q));
	ym_sdffr dff58(.MCLK(MCLK), .clk(rst_dly9_q), .val(vres_dly1_q), .reset(sres_syncv_q), .nq(vres_dly2_nq));
	ym_sdffr dff69(.MCLK(MCLK), .clk(rst_dly9_q), .val(wres_n), .reset(sres_syncv_q), .q(wres_sample_q), .nq(wres_sample_nq));
	assign vres_combined = ~(vres_dly2_nq | wres_active);

	// =======================================================================
	// Z80 IORQ handling
	// =======================================================================
	assign z80_iorq_raw = IORQ | M3 | ~M1;   // qualified IORQ (only valid during M1)
	ym_sdff dff29(.MCLK(MCLK), .clk(ZCLK), .val(d4_out), .q(z80_iorq_sync_q));
	assign z80_iorq_active = z80_iorq_raw & z80_iorq_sync_q;

	// Z80 M1 pipeline — synchronizes M1 signal across clock domains
	ym_sdff dff27(.MCLK(MCLK), .clk(ZCLK), .val(d1_out), .q(m1_sync1_q));
	ym_sdff dff30(.MCLK(MCLK), .clk(ZCLK), .val(m1_sync1_q), .q(m1_sync2_q));
	assign m1_delayed = m1_sync2_q;
	assign m1_delayed_2 = m1_delayed;
	ym_sdff dff44(.MCLK(MCLK), .clk(~ZCLK), .val(m1_delayed_2), .q(m1_pipe_q), .nq(m1_pipe_nq));

	assign z80_inactive = mreq_in | m1_pipe_nq;

	assign NMI = nmi_q;

	// =======================================================================
	// DRAM refresh
	//   REF is asserted periodically to refresh dynamic RAM.
	//   In M3 mode, gated by AS pending state.
	//   In Z80 mode, gated by M1 pipeline and MREQ.
	// =======================================================================
	assign ref_m3_gate = ~(~M3 | as_pending_sync_q);

	assign ref_z80_gate = ~(~M3 | m1_pipe_q | mreq_in);
	assign ref_active = ref_m3_gate | ref_z80_gate;
	assign REF = ~ref_active;

	// =======================================================================
	// Z80 bus request register
	//   Written by 68K to request Z80 bus. Directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low VD8 is latched
	//   on write to $A11100 (va_bank_1 region, vdp_region_n decode).
	// =======================================================================
	assign zbr_write_gate = ~(va_bank_1 & ~vdp_region_n & ~UDS_i);
	assign VD8_OE_n = test | ~RW_i | zbr_write_gate;
	assign zbr_clk = zbr_write_gate | RW_i;
	ym_sdffr zbr(.MCLK(MCLK), .clk(zbr_clk), .val(vd8), .reset(sres_syncv_q), .nq(zbr_nq));
	assign zbak_in = ZBAK;
	assign zbr_or_zbak = zbak_in | zbr_nq;
	assign ZBR = zbr_nq;

	// =======================================================================
	// MREQ output — directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low generates Z80 MREQ from 68K strobes
	// =======================================================================
	assign strobe_active = ~(LDS_i & UDS_i);
	ym_sdff dff59(.MCLK(MCLK), .clk(VCLK), .val(strobe_active), .q(strobe_sync_q));
	assign mreq_raw = strobe_active & as_pipe1_nq & strobe_sync_q;
	assign MREQ_o = ~mreq_raw;

	// =======================================================================
	// VDP/AS timing pipeline
	//   Multi-stage pipeline ensures proper timing of VDP accesses.
	//   as_pending tracks whether an address strobe cycle is in progress.
	// =======================================================================
	assign z80_region_as = z80_region_n | AS_i;

	assign as_timing1 = ~(as_pending & as_pending_sync_q);
	ym_sdff dff75(.MCLK(MCLK), .clk(~VCLK), .val(as_timing1), .nq(as_timing2_nq));
	assign as_timing3 = ~(as_pending & as_pending_sync_q & as_timing2_nq);
	ym_sdff dff66(.MCLK(MCLK), .clk(~VCLK), .val(as_timing3), .q(as_pipe1_q), .nq(as_pipe1_nq));
	ym_sdff dff73(.MCLK(MCLK), .clk(VCLK), .val(AS_i), .q(as_sync_q));
	assign as_combined = as_pipe1_q | AS_i | as_sync_q;
	ym_sdff dff64(.MCLK(MCLK), .clk(VCLK), .val(as_combined), .nq(as_pipe2_nq));
	assign as_dtack_gate = ~(as_pipe2_nq & wait_in);
	assign dtack_region_as = z80_region_as | as_dtack_gate;

	assign vtoz = z80_region_n | test | zbak_in;

	// VDP bus arbitration
	assign ref_fc_term = ~(ref_match | fc00_gate);
	assign ref_sres_term = ~(ref_fc_term | sres_syncv_nq);
	assign as_arb_gate = ~(vdp_arb1_q | AS_i);
	assign arb_next_val = ~(as_arb_gate | ref_sres_term);
	assign zres_gate = ~(zbr_or_zbak & zres_val);
	ym_sdffs dff47(.MCLK(MCLK), .clk(~VCLK), .val(arb_next_val), .set(zres_gate), .q(vdp_arb1_q), .nq(vdp_arb1_nq));
	assign as_pending = ~(AS_i & vdp_arb1_nq);
	ym_sdff dff70(.MCLK(MCLK), .clk(~VCLK), .val(as_pending), .q(as_pending_sync_q));

	// =======================================================================
	// Interrupt acknowledge
	//   INTAK asserted when Z80 has bus AND function code = 11 (interrupt ack)
	// =======================================================================
	assign intak_n = ~(ztov & fc11);
	assign INTAK = intak_n;

	assign VPA = AS_i | intak_n;

	// =======================================================================
	// CE0 generation — cartridge ROM chip enable
	//   Four independent terms are ANDed together to generate CE0:
	//   1. ce0_vdp_cart — VDP pipeline + cart presence
	//   2. ce0_timing_cart — timing + cart gate
	//   3. z80_ce0_gate — Z80 upper address qualification
	//   4. ce0_bank_term — bank mode qualification
	// =======================================================================
	assign m3_sres_rst = sres_syncv_q & M3;

	ym_sdffr dff26(.MCLK(MCLK), .clk(z80_wr_clk), .val(vd8), .reset(sres_syncv_q), .nq(bank_mode_nq));
	assign cas0_n = ~CAS0;
	ym_sdffs dff45(.MCLK(MCLK), .clk(cas0_n), .val(va23_in), .set(~z80_access_gate), .q(ce0_addr_latch_q));
	assign ce0_addr_term = ce0_addr_latch_q & va23_in & cas0_n;
	assign ce0_combine = z80_access_gate | ce0_addr_term;
	assign ce0_bank_mux = bank_mode_nq ? ce0_combine : z80_region_gate;
	assign ce0_cart_gate = ce0_bank_mux | ~va22_cart;
	assign ce0_timing_gate = bank_mode_nq | d5_out;
	assign ce0_timing_cart = ce0_timing_gate & ce0_cart_gate;
	assign not_68k_access = ~M3 | AS_i | va23_in;
	assign vdp_pipe_gate = vdp_pipe_s4_nq | vdp_pipe_ext;
	assign ce0_vdp_term = vdp_pipe_gate | not_68k_access;
	assign vdp_pipe_valid = ~(vdp_pipe_comb | vdp_pipe_latch_nq);
	assign ce0_vdp_bank = ~(vdp_pipe_valid | bank_mode_nq);
	assign ce0_vdp_cart = ce0_vdp_bank | not_68k_access | ~va22_cart;

	ym_sdffr dff25(.MCLK(MCLK), .clk(~VCLK), .val(vdp_pipe_latch_nq), .reset(m3_sres_rst), .q(vdp_pipe_sync_q));
	assign vdp_pipe_en = ~(vdp_pipe_s2_nq | vdp_pipe_sync_q);
	assign ce0_bank_term = vdp_pipe_en | bank_mode_nq;
	assign z80_ce0_gate = za15_in | z80_inactive | M3;
	assign ce0_active = ~(ce0_vdp_cart & ce0_timing_cart & z80_ce0_gate & ce0_bank_term);
	assign CE0 = ~ce0_active;

	// =======================================================================
	// ROM timing and RAS2/CAS2 generation
	// =======================================================================
	assign rom_timing_gate = ~(z80_access_gate & d7_out);
	assign rom_addr_term = rom_timing_gate & va23_in;
	ym_sdffs dff46(.MCLK(MCLK), .clk(cas0_n), .val(rom_timing_nq), .set(rom_addr_term), .q(rom_timing_q), .nq(rom_timing_nq));
	assign rom_timing_comb = rom_timing_q | d6_out;

	assign z80_region_gate = z80_access_gate | va23_in;
	assign ras2_addr_term = ~va21_in | va22_cart | z80_region_gate;
	assign cas0_or_z80gate = CAS0 | z80_access_gate;
	assign ras2_vdp_term = ~(~not_68k_access & vdp_pipe_valid & ~va22_cart & va21_in);
	assign ras2_timing_term = ras2_addr_term & d5_out;
	assign ras2_active = ~(ras2_timing_term & vdp_pipe_en & ras2_vdp_term);
	assign RAS2 = ~ras2_active;

	// ROM chip select — active when address is in cartridge ROM range
	assign rom_addr_term1 = va22_cart | va21_in | not_68k_access;
	assign va22_cart = ~(va22_in ^ CART);
	assign rom_addr_term2 = va22_cart | ce0_combine | va21_in;
	assign rom_sel = ~(rom_addr_term1 & rom_addr_term2);
	assign ROM = ~rom_sel;

	// Z80 RAM select and CAS2
	assign z80_ram_sel = ~(ZA_i[15:14] == 2'h2 & ~z80_inactive & ~M3); // Z80 $8000-$BFFF
	assign cas2_active = ~(vdp_pipe_rw & z80_ram_sel & cas0_or_z80gate);
	assign CAS2 = ~cas2_active;

	// ASEL — directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low address strobe for external devices
	assign asel_active = ~(vdp_pipe_ext & d3_out);
	assign ASEL = ~asel_active;

	// =======================================================================
	// VDP timing pipeline — multi-stage access timing
	//   Ensures VDP accesses are properly timed with respect to VCLK.
	//   Pipeline stages track the progress of a VDP access cycle.
	// =======================================================================
	assign not_68k_as = ~M3 | AS_i;
	assign vdp_addr_chk1 = ~(vdp_access_q & va23_in);
	assign vdp_addr_chk2 = ~(vdp_access_nq & vdp_arb_nq & ref_lo_match);
	assign vdp_access_valid = vdp_addr_chk1 & vdp_addr_chk2;
	ym_sdffs dff17(.MCLK(MCLK), .clk(VCLK), .val(vdp_access_valid), .set(m3_sres_rst), .q(vdp_pipe_s1_q));
	assign vdp_pipe_s1_val = vdp_pipe_s1_q;
	assign vdp_pipe_s1_val2 = vdp_pipe_s1_val;
	ym_sdffs dff20(.MCLK(MCLK), .clk(~VCLK), .val(vdp_pipe_s1_val2), .set(m3_sres_rst), .q(vdp_pipe_s2_q), .nq(vdp_pipe_s2_nq));
	assign vdp_pipe_s2_gate = vdp_pipe_s2_q & vdp_pipe_s1_val2;

	ym_sdffs dff19(.MCLK(MCLK), .clk(~VCLK), .val(vdp_pipe_s2_gate), .set(m3_sres_rst), .q(vdp_pipe_s3_q));
	assign vdp_pipe_s3_gate = ~(vdp_pipe_s3_q & vdp_pipe_s2_gate);
	assign vdp_pipe_comb = vdp_pipe_s3_gate | va23_in | not_68k_as;
	ym_sdff dff16(.MCLK(MCLK), .clk(VCLK), .val(vdp_pipe_comb), .q(vdp_pipe_sync1_q));
	assign vdp_pipe_ext = vdp_pipe_comb | vdp_pipe_sync1_q;
	ym_sdff dff11(.MCLK(MCLK), .clk(~VCLK), .val(vdp_pipe_ext), .q(vdp_pipe_dly1_q));
	assign vdp_pipe_or = vdp_pipe_dly1_q | vdp_pipe_comb;

	assign vdp_pipe_with_s4 = vdp_pipe_or | vdp_pipe_s4_q;
	ym_sdff dff12(.MCLK(MCLK), .clk(~VCLK), .val(vdp_pipe_with_s4), .q(vdp_pipe_latch_q), .nq(vdp_pipe_latch_nq));
	assign vdp_pipe_nand = ~(vdp_pipe_s4_nq & vdp_pipe_latch_q);
	assign vdp_addr_gate = vdp_access_nq | va23_in;
	assign vdp_pipe_val = vdp_pipe_nand & vdp_addr_gate;
	ym_sdffs dff15(.MCLK(MCLK), .clk(VCLK), .val(vdp_pipe_val), .set(m3_sres_rst), .q(vdp_pipe_s4_q), .nq(vdp_pipe_s4_nq));

	assign vdp_pipe_rw = vdp_pipe_or & vdp_pipe_s2_gate;


	// =======================================================================
	// DRAM refresh counters
	//   Two 4-bit counters (low + high) generate periodic refresh cycles.
	//   Low counter counts VCLK edges. When low counter is full (ref_lo_match),
	//   high counter increments. ref_match triggers the refresh cycle.
	// =======================================================================
	assign ref_load_raw = ~(ref_match | sres_syncv_nq);
	assign ref_load = ref_load_raw;
	assign ref_cnt_cin = ref_load & 1'h1 & 1'h1;

	// Low nibble counter
	ym_scnt_bit dff78(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_cnt_cin), .rst(1'h1), .nq(ref_lo1_nq), .cout(ref_lo1_cout));
	ym_scnt_bit dff80(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_lo1_cout), .rst(1'h1), .nq(ref_lo2_nq), .cout(ref_lo2_cout));
	ym_scnt_bit dff79(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_lo2_cout), .rst(1'h1), .nq(ref_lo3_nq), .cout(ref_lo3_cout));
	ym_scnt_bit dff77(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_lo3_cout), .rst(1'h1), .nq(ref_lo4_nq), .cout(ref_lo4_cout));
	assign ref_lo_full = ~(ref_lo4_nq | ref_lo1_nq | ref_lo3_nq | ref_lo2_nq);
	assign ref_lo_match = ref_lo_full & 1'h1;

	// High nibble counter — increments when low counter is full
	assign ref_hi_cin = ref_load & ref_lo_match & ref_lo_match;
	ym_scnt_bit dff48(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_hi_cin), .rst(1'h1), .nq(ref_hi1_nq), .cout(ref_hi1_cout));
	ym_scnt_bit dff54(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_hi1_cout), .rst(1'h1), .nq(ref_hi2_nq), .cout(ref_hi2_cout));
	ym_scnt_bit dff53(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(1'h0), .cin(ref_hi2_cout), .rst(1'h1), .nq(ref_hi3_nq), .cout(ref_hi3_cout));
	ym_scnt_bit dff55(.MCLK(MCLK), .clk(VCLK), .load(ref_load), .val(M3), .cin(ref_hi3_cout), .rst(1'h1), .nq(ref_hi4_nq), .cout(ref_hi4_cout));
	assign ref_hi_full = ~(ref_hi1_nq | ref_hi3_nq | ref_hi2_nq | ref_hi4_nq);
	assign ref_match = ref_hi_full & ref_lo_match;

	// =======================================================================
	// VDP FIFO / arbitration state machine
	//   Controls when the VDP can access the bus for FIFO operations.
	// =======================================================================
	assign vdp_fifo_ready = ~(vdp_arb_q | vdp_access_q | ref_lo_match | ~z80_access_gate);
	assign vdp_arb_val = ~(vdp_fifo_ready | ref_match | fc00_gate);
	ym_sdffs dff33(.MCLK(MCLK), .clk(VCLK), .val(vdp_arb_val), .set(sres_syncv_q), .q(vdp_arb_q), .nq(vdp_arb_nq));
	ym_sdffr dff23(.MCLK(MCLK), .clk(~not_68k_as), .val(vdp_arb_nq), .reset(vdp_arb_nq), .q(vdp_access_q), .nq(vdp_access_nq));


	// =======================================================================
	// Z80 bus interface outputs
	// =======================================================================
	assign ZRD_o = AS_i | ~RW_i;

	assign ZV = ztov;

	// UDS edge detect for ZA0 output
	ym_sdff dff13(.MCLK(MCLK), .clk(~VCLK), .val(UDS_i), .q(uds_prev_q));
	assign uds_falling = ~(uds_prev_q & UDS_i);
	assign za0_out_val = ~uds_falling;

	assign sres = SRES;

	assign ZWR_o = RW_i | AS_i;

	assign vd8 = VD8_i;

	assign mreq_in = MREQ_i;

	// =======================================================================
	// Chip select generation and DTACK
	// =======================================================================

	// UDS + RW active (both active-low, so both must be low)
	assign uds_rw_active = ~(UDS_i | RW_i);

	// Write clocks for different VA[8:7] bank regions
	assign zbr_wr_clk = ~(uds_rw_active & va_bank_2 & ~vdp_region_n);  // VA bank 2 write
	assign z80_wr_clk = ~(uds_rw_active & va_bank_0 & ~vdp_region_n);  // VA bank 0 write

	// FDC chip select ($A12000-$A1207F)
	assign fdc_sel = AS_i | fdc_region_n;
	assign fdwr_val = ~(fdwr_latch_q | RW_i | fdc_sel);
	ym_sdff dff24(.MCLK(MCLK), .clk(VCLK), .val(fdwr_val), .q(fdwr_latch_q), .nq(fdwr_latch_nq));
	assign FDC = fdc_sel;
	assign FDWR = fdwr_latch_nq;

	// I/O chip select ($A10000-$A1007F)
	assign io_sel = io_port_n | AS_i;
	assign io_active = ~(z80_iorq_active & io_sel);
	assign IO = ~io_active;

	// Sound chip select (directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low Z80 $4000-$5FFF region)
	assign SOUND = sound_n;

	// TIME chip select ($A13000-$A1307F)
	assign time_sel = time_region_n | AS_i;
	assign TIME = time_sel;

	assign VA_MID_OE_n = ztov | test | pal_trap;

	assign strobe_dir_raw = ~(bus_released_2 | test | pal_trap);

	assign VA_HI_OE_n = ~strobe_dir_raw;

	assign test = test_mode_0;

	// ZRES generation — Z80 reset output
	ym_sdffr dff31(.MCLK(MCLK), .clk(zbr_wr_clk), .val(vd8), .reset(vres_combined), .q(zres_latch_q));
	assign zres_val = M3 ? zres_latch_q : vres_combined;

	// SRES synchronized to VCLK
	ym_sdff sres_syncv(.MCLK(MCLK), .clk(VCLK), .val(SRES), .q(sres_syncv_q), .nq(sres_syncv_nq));

	assign RW_o = ZWR_i;

	// =======================================================================
	// DTACK generation
	//   DTACK is asserted when any chip select matches AND no INTAK override.
	// =======================================================================
	assign vdp_as_sel = vdp_region_n | AS_i;
	assign dtack_any_sel = ~(ce0_vdp_term & vdp_as_sel & fdc_sel & time_sel & io_sel & dtack_region_as);
	assign dtack_with_intak = ~(dtack_any_sel & intak_n);
	assign dtack_active = ~(dtack_with_intak | test);
	assign DTACK_o = ~dtack_active;

	assign va21_mreq = va21_in | mreq_in;

	// Z80 access gate — Z80 can access bus when BGACK feedback is inactive
	assign z80_access_gate = bgack_fb_n | BGACK_i | ~M3;

	assign ZRES = zres_val;

	assign wait_in = WAIT_i;

	// VRES generation
	assign vres_raw = ~(M3 & vres_combined);
	assign VRES = ~vres_raw;

	// =======================================================================
	// Function code decode and gating
	//   68K function codes (FC1:FC0) identify the type of bus cycle.
	//   Each is gated by delayed M3 to ensure proper timing.
	//     FC=00: data/user       FC=01: data/supervisor
	//     FC=10: program/user    FC=11: interrupt acknowledge
	// =======================================================================
	assign fc00_gate = ~(~fc00 | d8_out);
	assign fc01_gate = ~(~fc01 | d8_out);
	assign fc10_gate = ~(~fc10 | d8_out);
	assign nmi_gate = ~(nmi_sres_term | d8_out);
	assign nmi_sres_term = ~(d8_out | sres_syncv_q);

	assign YS_TEST = tmss_io_access;

	assign fc00 = ~FC1 & ~FC0;
	assign fc01 = ~FC1 & FC0;
	assign fc10 = FC1 & ~FC0;
	assign fc11 = FC1 & FC0;

	// =======================================================================
	// Z80 memory map chip selects
	// =======================================================================
	assign ZRAM = zram_n;

	assign va14_in = VA_i[13];

	assign va21_in = VA_i[20];

	assign va22_in = VA_i[21];

	assign va23_in = VA_i[22];

	assign ZA0_o = za0_out_val;

	assign ZA_o[15:8] = { 1'h0, VA_i[13:7] };

	assign VZ = vtoz;

	assign za15_in = ZA_i[15];

	// =======================================================================
	// Z80 bank register — 9-bit shift register
	//   Written by Z80 at $6000. Each write shifts in ZD0 from the MSB side.
	//   Provides upper address bits when Z80 accesses the $8000-$FFFF window.
	// =======================================================================
	ym_sdffr #(.DATA_WIDTH(9)) z80bank(.MCLK(MCLK), .clk(z80bank_wr_n), .val({ ZD0_i, z80bank_q[8:1] }),
		.reset(sres_syncv_q), .q(z80bank_q));

	// VA output mux — selects between Z80 bank register, fixed vectors, and
	// Z80 bus signals depending on M3 mode and window decode.
	wire [15:0] va_out_t = M3 ? { z80_window_n ? z80bank_q : 9'h180, ZA_i[14:8] } : { 3'h0, zres_val, IORQ, mreq_in, va21_mreq, ZA_i[15:7] };

	assign va_out = z80_window_n ? va_out_t : {va_out_t[15:8], 8'h0};

	// =======================================================================
	// Address decode — VA[8:7] register bank select
	//   Selects which register bank within the I/O chip ($A100xx) is accessed.
	// =======================================================================
	assign va_bank_0 = VA_i[8:7] == 2'h0;  // $A10000-$A1007F
	assign va_bank_1 = VA_i[8:7] == 2'h1;  // $A10080-$A100FF
	assign va_bank_2 = VA_i[8:7] == 2'h2;  // $A10100-$A1017F
	assign va_bank_3 = VA_i[8:7] == 2'h3;  // $A10180-$A101FF (unused)

	// =======================================================================
	// Address decode — 68K address space chip selects
	//   Each signal is active-low: low when the address matches AND M3=1.
	// =======================================================================
	assign tmss_io_access = AS_i | LDS_i | UDS_i | VA_i[22:7] != 16'ha140; // $A14000 region
	assign VSYNC_TEST = ~(VA_i[22:7] == 16'hc000 & ~AS_i);                  // $C00000 (VDP)

	assign z80_region_n = ~(M3 & VA_i[22:15] == 8'ha0);    // $A00000-$A0FFFF (Z80 bus window)

	assign io_port_n = ~(M3 & VA_i[22:7] == 16'ha100);     // $A10000-$A1007F (I/O ports)

	assign vdp_region_n = ~(M3 & VA_i[22:9] == 14'h2844);  // $A10880-$A108FF (VDP FIFO)

	assign fdc_region_n = ~(M3 & VA_i[22:7] == 16'ha120);  // $A12000-$A1207F (FDC)

	assign time_region_n = ~(M3 & VA_i[22:7] == 16'ha130); // $A13000-$A1307F (TIME)

	// Z80 address space chip selects
	assign zram_n = ~(ZA_i[15:14] == 2'h0 & ~z80_inactive & M3);   // $0000-$3FFF (Z80 RAM)

	assign sound_n = ~(ZA_i[15:13] == 3'h2 & ~z80_inactive & M3);  // $4000-$5FFF (YM2612)

	assign z80bank_wr_n = ~(M3 & ZA_i[15:8] == 8'h60 & ~ZWR_i & ~z80_inactive); // $6000 bank write

	assign z80_window_n = ~(ZA_i[15:8] == 8'h7f & M3);     // $7F00-$7FFF (68K bus window)

	assign IA14 = ~(M3 & va14_in);

	assign VA_o[22:7] = va_out;

	assign VD8_o = zbak_in;

endmodule

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
 *  TMSS(FC1004) emulator
 *  Thanks:
 *      org (ogamespec):
 *          FC1004 decap and die shot.
 *      andkorzh, HardWareMan (emu-russia):
 *          help & support.
 *
 *  Trademark Security System (TMSS):
 *    Software must write "SEGA" (0x5345 then 0x4741) to $A14000/$A14002
 *    before the console will allow cartridge ROM access via CE0. A register
 *    at $A14101 controls whether the cartridge ROM or internal TMSS ROM is
 *    mapped. The TMSS ROM displays the "Produced by or under license..."
 *    splash screen.
 */

module tmss
	(
	input MCLK,
	input [15:0] VD_i,
	input [2:0] test,
	input JAP,
	input AS,
	input LDS,
	input UDS,
	input RW,
	input [22:0] VA,
	input SRES,
	input CE0_i,
	input M3,
	input CART,
	input INTAK,
	output [15:0] VD_o,
	output DTACK,
	output RESET,
	output CE0_o,
	output test_0,
	output test_1,
	output test_2,
	output test_3,
	output test_4,
	output data_out_en,
	input tmss_enable,
	input [15:0] tmss_data,
	output [9:0] tmss_address
	);

	// -----------------------------------------------------------------------
	// SEGA signature detection
	//   Two-stage latch: first write captures low word, second write captures
	//   high word. When sig_word_low == "SE" (0x5345) and sig_word_high == "GA"
	//   (0x4741), sega_signature_match goes high. This arms signature_stage1,
	//   which is then sampled by signature_locked_n to unlock ROM access.
	// -----------------------------------------------------------------------
	wire signature_stage1;       // latched on signature write clock when match detected
	wire signature_locked_n;     // cleared (active-low) once signature propagates through bank access
	wire sega_signature_match;   // combinational: both halves of "SEGA" match
	wire bank_access_clk;        // clock for second stage — fires on any access to VA[22:20]==6
	wire tmss_reg_access;        // decoded access to TMSS signature register at $A14000
	wire rom_flag_access;        // decoded access to ROM enable flag at $A14101
	wire signature_write_clk;    // gated write strobe for signature register
	wire tmss_read_strobe;       // gated read strobe for signature register
	wire rom_enabled;            // bit 0 written to $A14101 — enables cartridge ROM
	wire cart_or_no_m3;          // CART pin asserted OR not in M3 mode (bypass TMSS)
	wire ce0_gate;               // combined gate: rom_enabled | cart_or_no_m3 | CE0_i
	wire latch_low_en;           // write enable for low word latch (even address)
	wire [15:0] sig_word_low;    // latched low signature word ("SE" = 0x5345)
	wire latch_high_en;          // write enable for high word latch (odd address)
	wire [15:0] sig_word_high;   // latched high signature word ("GA" = 0x4741)
	wire [15:0] sig_readback;    // muxed readback of signature words

	// -----------------------------------------------------------------------
	// Test mode decode outputs
	// -----------------------------------------------------------------------
	wire test_not_0;             // test[2:0] != 0
	wire test_not_1;             // test[2:0] != 1
	wire test_not_2;             // test[2:0] != 2
	wire test_not_3;             // test[2:0] != 3
	wire test_not_4;             // test[2:0] != 4
	wire test_not_7;             // test[2:0] != 7
	wire test_xor_0;             // XOR pattern for test output 0
	wire test_xor_1;             // XOR pattern for test output 1
	wire test_xor_2;             // XOR pattern for test output 2
	wire test_xor_3;             // XOR pattern for test output 3
	wire test_xor_4;             // XOR pattern for test output 4 (also used as override)

	// -----------------------------------------------------------------------
	// Signature match and lock pipeline
	//   Stage 1: sample sega_signature_match on rising edge of signature_write_clk.
	//   Stage 2: sample stage 1 on rising edge of bank_access_clk; signature_locked_n
	//            is active-low, set on SRES.
	// -----------------------------------------------------------------------
	ym_sdffr dff1(.MCLK(MCLK), .clk(signature_write_clk), .val(sega_signature_match), .reset(SRES), .q(signature_stage1));
	ym_sdffs dff2(.MCLK(MCLK), .clk(bank_access_clk), .val(signature_stage1), .set(SRES), .nq(signature_locked_n));

	// "SEGA" = 0x5345_4741: first word "SE", second word "GA"
	assign sega_signature_match = sig_word_low == 16'h5345 & sig_word_high == 16'h4741;

	// -----------------------------------------------------------------------
	// Reset output
	//   On Japanese consoles (JAP=1), RESET is held low if signature is not
	//   yet validated (signature_locked_n=1). test_4 can override.
	// -----------------------------------------------------------------------
	assign RESET = tmss_enable ? (~(JAP & signature_locked_n) | test_4) : 1'h1;

	// -----------------------------------------------------------------------
	// Address decode
	//   bank_access_clk:  any access where VA[22:20] == 6 (addresses $600000-$6FFFFF)
	//   tmss_reg_access:  full decode of $A14000 (VA[22:1] == 22'h285000, LDS+UDS active)
	//   rom_flag_access:  full decode of $A14101 (VA == 23'h50A080)
	// -----------------------------------------------------------------------
	assign bank_access_clk = ~(~AS & VA[22:20] == 3'h6);

	assign tmss_reg_access = ~AS & ~LDS & VA[22:1] == 22'h285000 & ~UDS; // $A14000
	assign rom_flag_access = ~AS & ~LDS & VA == 23'h50a080;              // $A14101

	// -----------------------------------------------------------------------
	// DTACK generation
	//   Directly acknowledge TMSS register or ROM flag accesses.
	// -----------------------------------------------------------------------
	assign DTACK = tmss_enable ? (~((tmss_reg_access | rom_flag_access) & INTAK) | test_4) : 1'h1;

	// -----------------------------------------------------------------------
	// Write/read strobes for signature register
	// -----------------------------------------------------------------------
	assign signature_write_clk = ~(~RW & tmss_reg_access); // write to $A14000
	assign tmss_read_strobe = ~(RW & tmss_reg_access);      // read from $A14000

	// -----------------------------------------------------------------------
	// ROM access flag register at $A14101
	//   Bit 0 of VD_i is latched on write. Controls whether cartridge CE0 passes
	//   through. Reset by SRES.
	// -----------------------------------------------------------------------
	ym_sdffr dff3(.MCLK(MCLK), .clk(~rom_flag_access | RW), .val(VD_i[0]), .reset(SRES), .q(rom_enabled));
	//ym_sdffs dff3(.MCLK(MCLK), .clk(~rom_flag_access | RW), .val(VD_i[0]), .set(SRES), .q(rom_enabled));

	// -----------------------------------------------------------------------
	// CE0 output gate (cartridge ROM chip-enable)
	//   Cartridge ROM is accessible if:
	//     - rom_enabled is set (software wrote 1 to $A14101), OR
	//     - CART pin is asserted (physical cartridge present), OR
	//     - M3 mode is inactive (Master System mode bypasses TMSS), OR
	//     - CE0_i is already asserted from upstream
	// -----------------------------------------------------------------------
	assign cart_or_no_m3 = CART | ~M3;
	assign ce0_gate = rom_enabled | cart_or_no_m3 | CE0_i;
	assign CE0_o = tmss_enable ? (~(rom_enabled | cart_or_no_m3) | CE0_i) : CE0_i;

	// -----------------------------------------------------------------------
	// Signature word latches
	//   Even address (VA[0]=0) writes to sig_word_low ("SE" half).
	//   Odd address  (VA[0]=1) writes to sig_word_high ("GA" half).
	// -----------------------------------------------------------------------
	assign latch_low_en = ~VA[0] & ~RW & tmss_reg_access;
	ym_slatch #(.DATA_WIDTH(16)) sl1(.MCLK(MCLK), .en(latch_low_en), .inp(VD_i), .val(sig_word_low));
	assign latch_high_en = VA[0] & ~RW & tmss_reg_access;
	ym_slatch #(.DATA_WIDTH(16)) sl2(.MCLK(MCLK), .en(latch_high_en), .inp(VD_i), .val(sig_word_high));

	// -----------------------------------------------------------------------
	// Output data mux
	//   When ce0_gate blocks ROM (TMSS ROM visible), output tmss_data (TMSS ROM).
	//   Otherwise, read back the latched signature words.
	// -----------------------------------------------------------------------
	assign sig_readback = VA[0] ? sig_word_high : sig_word_low;
	assign VD_o = tmss_enable ? (ce0_gate ? sig_readback : tmss_data) : 16'h0;

	assign tmss_address = VA[9:0];

	// Data bus output enable: active during TMSS read when ROM is gated off
	assign data_out_en = tmss_enable ? (tmss_read_strobe & ce0_gate) | test_4 : 1'h1;

	// -----------------------------------------------------------------------
	// Test mode decode
	//   Generates a 5-bit one-hot-like pattern from test[2:0] using XOR gates.
	//   Each test output is active-low, selecting a different test function on
	//   the die. test_4 (test[2:0]==4 vs 7 XOR) also serves as an override
	//   for RESET, DTACK, and data_out_en.
	// -----------------------------------------------------------------------
	assign test_not_0 = test[2:0] != 3'h0;
	assign test_not_1 = test[2:0] != 3'h1;
	assign test_not_2 = test[2:0] != 3'h2;
	assign test_not_3 = test[2:0] != 3'h3;
	assign test_not_4 = test[2:0] != 3'h4;
	assign test_not_7 = test[2:0] != 3'h7;

	assign test_xor_0 = test_not_0 ^ test_not_7;
	assign test_xor_1 = test_not_1 ^ test_not_7;
	assign test_xor_2 = test_not_2 ^ test_not_7;
	assign test_xor_3 = test_not_3 ^ test_not_7;
	assign test_xor_4 = test_not_7 ^ test_not_4;

	assign test_0 = ~test_xor_0;
	assign test_1 = ~test_xor_1;
	assign test_2 = ~test_xor_2;
	assign test_3 = ~test_xor_3;
	assign test_4 = ~test_xor_4;

endmodule

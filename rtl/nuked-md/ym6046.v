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
 *  YM6046(FC1004) emulator — I/O Controller
 *  Thanks:
 *      org (ogamespec):
 *          FC1004 decap and die shot.
 *      andkorzh, HardWareMan (emu-russia):
 *          help & support.
 *
 *  Handles 3 controller ports (A, B, C) with parallel I/O and UART.
 *  Maps to $A10001-$A1001F in 68K address space (M3 mode) or Z80 ports.
 *  Provides version register, controller data/direction, and serial I/O.
 */

module ym6046
	(
	input MCLK,
	input [6:0] PORT_A_i,
	input [6:0] PORT_B_i,
	input [6:0] PORT_C_i,
	input test,
	input M3,
	input IO,
	input CAS0,
	input SRES,
	input VCLK,
	input NTSC,
	input DISK,
	input JAP,
	input [7:0] ZA_i,
	input [7:0] ZD_i,
	input [6:0] VA_i,
	input [15:0] VD_i,
	input LWR,
	input t1,
	input ZV,
	input VZ,
	output [6:0] PORT_A_d,
	output [6:0] PORT_B_d,
	output [6:0] PORT_C_d,
	output [6:0] PORT_A_o,
	output [6:0] PORT_B_o,
	output [6:0] PORT_C_o,
	output HL,
	output FRES,
	output ZA_OE,
	output VD_LO_OE_n,
	output VD_HI_OE_n,
	output ZD_OE_n,
	output VA_LO_OE_n,
	output [7:0] vdata,
	output reg_3e_q,
	output [7:0] zdata,
	output [6:0] ztov_address,
	input tmss_enable
	);

	// -----------------------------------------------------------------------
	// Reset synchronization
	// -----------------------------------------------------------------------
	wire reset;                    // internal reset (synced to VCLK)
	wire res_dff_q, res_dff_nq;
	wire pal;                      // 1 = PAL region

	// -----------------------------------------------------------------------
	// UART clock generation
	//   VCLK is divided by a programmable counter to produce uart_base_clk.
	//   The counter reloads when it reaches 0xFF (both nibbles == 0xF).
	//   PAL reload value is 0x9D (157+1=158 VCLK ticks), NTSC is 0x9C (156+1).
	//   uart_base_clk is bit 2 of the high nibble counter.
	//   uart_clk_muxed selects VCLK (test mode) or uart_base_clk (normal).
	//   An 8-stage binary divider chain produces selectable baud rates.
	// -----------------------------------------------------------------------
	wire [3:0] uart_clk_sel1;      // mux inputs for uart baud clock (group 1)
	wire [3:0] uart_clk_sel2;      // mux inputs for uart baud clock (group 2)
	wire cnt_reload;               // counter reload signal
	wire [3:0] vclk_cnt_lo;        // low nibble of VCLK divider counter
	wire [3:0] vclk_cnt_hi;        // high nibble of VCLK divider counter
	wire uart_base_clk;            // base UART clock from counter bit
	wire uart_clk_muxed;           // final UART clock (test ? VCLK : uart_base_clk)
	wire uart_div0_q, uart_div0_nq;
	wire uart_div1_q, uart_div1_nq;
	wire uart_div2_q, uart_div2_nq;
	wire uart_div3_q, uart_div3_nq;
	wire uart_div4_q, uart_div4_nq;
	wire uart_div5_q, uart_div5_nq;
	wire uart_div6_q, uart_div6_nq;
	wire uart_div7_q, uart_div7_nq;

	// -----------------------------------------------------------------------
	// Bus interface signals
	// -----------------------------------------------------------------------
	wire [7:0] address;            // register address (from VA or ZA depending on M3)
	wire [3:0] read_address;       // 4-bit register read select
	wire [7:0] data_bus;           // write data bus (from VD or ZD depending on M3)
	wire [7:0] read_data;          // muxed register read output
	wire vdp_read_n;               // VDP-side read strobe (active-low, CAS0 | IO)
	wire vdp_write_n;              // VDP-side write strobe (active-low, LWR | IO)
	wire io_region_n;              // address is in I/O region ($00-$0F when M3=1)
	wire read_upper;               // read from upper register bank (address[3]=1)
	wire write_lower;              // write to lower register bank (address[3]=0)
	wire write_upper;              // write to upper register bank (address[3]=1)
	wire z80_write_n;              // Z80 write strobe (address 0x3E/0x3F, M3=0)
	wire z80_read_n;               // Z80 read strobe (SMS-mode port reads)
	wire z80_access;               // combined: not a Z80 special access
	wire z80_write_even;           // Z80 write to even address (0x3E)
	wire z80_write_odd;            // Z80 write to odd address (0x3F)
	wire [7:0] reg_3f_data;        // Z80 register $3F contents (SMS I/O port control)
	wire io_chip_active;           // chip is selected for I/O access
	wire upper_byte_sel;           // M3 mode upper byte select (M3 & ~ZA[0])
	wire arb_gate1;                // bus arbitration gate 1
	wire arb_gate2;                // bus arbitration gate 2

	// -----------------------------------------------------------------------
	// Per-port signals (Port A)
	// -----------------------------------------------------------------------
	wire read_rx_data_a;
	wire write_p_data_a;
	wire write_tx_data_a;
	wire write_s_control_a;
	wire write_p_control_a;
	wire uart_clk1_a;
	wire uart_clk2_a;
	wire [7:0] p_data_q_a;
	wire [7:0] p_control_q_a;
	wire [7:0] tx_data_a;
	wire [7:0] rx_data_q_a;
	wire [4:0] s_control_q_a;
	wire tx_pending_a;
	wire rx_data_ready_a;
	wire rx_frame_error_a;
	wire [6:0] port_a_d;
	wire [6:0] port_a_o;
	wire irq_b6_a;
	wire irq_uart_a;

	// -----------------------------------------------------------------------
	// Per-port signals (Port B)
	// -----------------------------------------------------------------------
	wire read_rx_data_b;
	wire write_p_data_b;
	wire write_tx_data_b;
	wire write_s_control_b;
	wire write_p_control_b;
	wire uart_clk1_b;
	wire uart_clk2_b;
	wire [7:0] p_data_q_b;
	wire [7:0] p_control_q_b;
	wire [7:0] tx_data_b;
	wire [7:0] rx_data_q_b;
	wire [4:0] s_control_q_b;
	wire tx_pending_b;
	wire rx_data_ready_b;
	wire rx_frame_error_b;
	wire [6:0] port_b_d;
	wire [6:0] port_b_o;
	wire irq_b6_b;
	wire irq_uart_b;

	// -----------------------------------------------------------------------
	// Per-port signals (Port C)
	// -----------------------------------------------------------------------
	wire read_rx_data_c;
	wire write_p_data_c;
	wire write_tx_data_c;
	wire write_s_control_c;
	wire write_p_control_c;
	wire uart_clk1_c;
	wire uart_clk2_c;
	wire [7:0] p_data_q_c;
	wire [7:0] p_control_q_c;
	wire [7:0] tx_data_c;
	wire [7:0] rx_data_q_c;
	wire [4:0] s_control_q_c;
	wire tx_pending_c;
	wire rx_data_ready_c;
	wire rx_frame_error_c;
	wire [6:0] port_c_d;
	wire [6:0] port_c_o;
	wire irq_b6_c;
	wire irq_uart_c;

	// -----------------------------------------------------------------------
	// Controller port submodule instances
	// -----------------------------------------------------------------------
	ym6046_controller_port port_a(.MCLK(MCLK), .port_i(PORT_A_i), .data_bus(data_bus), .reset(reset), .m3(M3), .uart_clk_i1(uart_clk_sel1),
		.uart_clk_i2(uart_clk_sel2), .read_rx_data(read_rx_data_a), .write_p_data(write_p_data_a), .write_tx_data(write_tx_data_a),
		.write_s_control(write_s_control_a), .write_p_control(write_p_control_a), .uart_clk1(uart_clk1_a), .uart_clk2(uart_clk2_a), .p_data_q(p_data_q_a),
		.p_control_q(p_control_q_a), .tx_data(tx_data_a), .rx_data_q(rx_data_q_a), .s_control_q(s_control_q_a),
		.tx_pending(tx_pending_a), .rx_data_ready(rx_data_ready_a), .rx_frame_error(rx_frame_error_a), .port_d(port_a_d), .port_o(port_a_o),
		.irq_b6(irq_b6_a), .irq_uart(irq_uart_a));

	ym6046_controller_port port_b(.MCLK(MCLK), .port_i(PORT_B_i), .data_bus(data_bus), .reset(reset), .m3(M3), .uart_clk_i1(uart_clk_sel1),
		.uart_clk_i2(uart_clk_sel2), .read_rx_data(read_rx_data_b), .write_p_data(write_p_data_b), .write_tx_data(write_tx_data_b),
		.write_s_control(write_s_control_b), .write_p_control(write_p_control_b), .uart_clk1(uart_clk1_b), .uart_clk2(uart_clk2_b), .p_data_q(p_data_q_b),
		.p_control_q(p_control_q_b), .tx_data(tx_data_b), .rx_data_q(rx_data_q_b), .s_control_q(s_control_q_b),
		.tx_pending(tx_pending_b), .rx_data_ready(rx_data_ready_b), .rx_frame_error(rx_frame_error_b), .port_d(port_b_d), .port_o(port_b_o),
		.irq_b6(irq_b6_b), .irq_uart(irq_uart_b));

	ym6046_controller_port port_c(.MCLK(MCLK), .port_i(PORT_C_i), .data_bus(data_bus), .reset(reset), .m3(M3), .uart_clk_i1(uart_clk_sel1),
		.uart_clk_i2(uart_clk_sel2), .read_rx_data(read_rx_data_c), .write_p_data(write_p_data_c), .write_tx_data(write_tx_data_c),
		.write_s_control(write_s_control_c), .write_p_control(write_p_control_c), .uart_clk1(uart_clk1_c), .uart_clk2(uart_clk2_c), .p_data_q(p_data_q_c),
		.p_control_q(p_control_q_c), .tx_data(tx_data_c), .rx_data_q(rx_data_q_c), .s_control_q(s_control_q_c),
		.tx_pending(tx_pending_c), .rx_data_ready(rx_data_ready_c), .rx_frame_error(rx_frame_error_c), .port_d(port_c_d), .port_o(port_c_o),
		.irq_b6(irq_b6_c), .irq_uart(irq_uart_c));

	// -----------------------------------------------------------------------
	// Reset synchronization
	//   SRES is sampled on VCLK rising edge. res_dff_nq is active-low reset
	//   output (directly becomes FRES). res_dff_q is active-high internal reset.
	// -----------------------------------------------------------------------
	ym_sdff res_dff(.MCLK(MCLK), .clk(VCLK), .val(SRES), .q(res_dff_q), .nq(res_dff_nq));

	assign FRES = res_dff_nq;
	assign reset = res_dff_q;

	assign pal = ~NTSC;

	// -----------------------------------------------------------------------
	// UART base clock generation
	//   Two 4-bit counters divide VCLK. The counter reloads to PAL (0x9D) or
	//   NTSC (0x9C) value when it reaches 0xFF or during reset. uart_base_clk
	//   is bit 2 of the high nibble, toggling at ~1/4 of the counter period.
	// -----------------------------------------------------------------------
	assign cnt_reload = ~(~reset | (vclk_cnt_lo == 4'hf & vclk_cnt_hi == 4'hf));
	ym_scnt_bit #(.DATA_WIDTH(4)) cnt1(.MCLK(MCLK), .clk(VCLK), .load(cnt_reload), .val(pal ? 4'hd : 4'hc), .cin(cnt_reload), .rst(1'h1),
		.q(vclk_cnt_lo));
	ym_scnt_bit #(.DATA_WIDTH(4)) cnt2(.MCLK(MCLK), .clk(VCLK), .load(cnt_reload), .val(4'h9), .cin(cnt_reload & vclk_cnt_lo == 4'hf), .rst(1'h1),
		.q(vclk_cnt_hi));

	assign uart_base_clk = vclk_cnt_hi[2];

	// In test mode, bypass counter and use VCLK directly
	assign uart_clk_muxed = test ? VCLK : uart_base_clk;

	// -----------------------------------------------------------------------
	// 8-stage binary divider chain
	//   Each stage divides by 2, providing selectable baud rates from the
	//   base UART clock. Stages are chained: each toggles on falling edge
	//   of the previous stage's output. All reset by internal reset.
	// -----------------------------------------------------------------------
	ym_sdffr uart_div0(.MCLK(MCLK), .clk(~uart_clk_muxed), .val(uart_div0_nq), .reset(reset), .q(uart_div0_q), .nq(uart_div0_nq));
	ym_sdffr uart_div1(.MCLK(MCLK), .clk(~uart_div0_q), .val(uart_div1_nq), .reset(reset), .q(uart_div1_q), .nq(uart_div1_nq));
	ym_sdffr uart_div2(.MCLK(MCLK), .clk(~uart_div1_q), .val(uart_div2_nq), .reset(reset), .q(uart_div2_q), .nq(uart_div2_nq));
	ym_sdffr uart_div3(.MCLK(MCLK), .clk(~uart_div2_q), .val(uart_div3_nq), .reset(reset), .q(uart_div3_q), .nq(uart_div3_nq));
	ym_sdffr uart_div4(.MCLK(MCLK), .clk(~uart_div3_q), .val(uart_div4_nq), .reset(reset), .q(uart_div4_q), .nq(uart_div4_nq));
	ym_sdffr uart_div5(.MCLK(MCLK), .clk(~uart_div4_q), .val(uart_div5_nq), .reset(reset), .q(uart_div5_q), .nq(uart_div5_nq));
	ym_sdffr uart_div6(.MCLK(MCLK), .clk(~uart_div5_q), .val(uart_div6_nq), .reset(reset), .q(uart_div6_q), .nq(uart_div6_nq));
	ym_sdffr uart_div7(.MCLK(MCLK), .clk(~uart_div6_q), .val(uart_div7_nq), .reset(reset), .q(uart_div7_q), .nq(uart_div7_nq));

	// Baud rate select mux inputs (selected per-port by s_control_q[4:3])
	assign uart_clk_sel1[0] = uart_clk_muxed;
	assign uart_clk_sel1[1] = uart_div0_q;
	assign uart_clk_sel1[2] = uart_div1_q;
	assign uart_clk_sel1[3] = uart_div3_q;

	assign uart_clk_sel2[0] = uart_div3_q;
	assign uart_clk_sel2[1] = uart_div4_q;
	assign uart_clk_sel2[2] = uart_div5_q;
	assign uart_clk_sel2[3] = uart_div7_q;

	// -----------------------------------------------------------------------
	// Address/data bus routing
	//   M3=1 (MegaDrive mode): address from VDP bus VA[6:0], data from VD[7:0]
	//   M3=0 (SMS mode):       address from Z80 bus ZA[7:0], data from ZD[7:0]
	// -----------------------------------------------------------------------
	assign address = M3 ? { 1'h0, VA_i[6:0] } : ZA_i[7:0];
	assign ztov_address = M3 ? ZA_i[7:1] : ZA_i[6:0];

	// In M3 mode, read_address is the full 4-bit register select.
	// In SMS mode, odd addresses read register 2, even read register 1.
	assign read_address = M3 ? address[3:0] : (address[0] ? 4'h2 : 4'h1);

	assign data_bus = M3 ? VD_i[7:0] : ZD_i[7:0];

	// -----------------------------------------------------------------------
	// Register read mux (16 registers)
	//   Reg 0: Version/status    Reg 8:  Port A RX data
	//   Reg 1: Port A data       Reg 9:  Port A serial status
	//   Reg 2: Port B data       Reg 10: Port B TX data
	//   Reg 3: Port C data       Reg 11: Port B RX data
	//   Reg 4: Port A control    Reg 12: Port B serial status
	//   Reg 5: Port B control    Reg 13: Port C TX data
	//   Reg 6: Port C control    Reg 14: Port C RX data
	//   Reg 7: Port A TX data    Reg 15: Port C serial status
	// -----------------------------------------------------------------------
	wire [15:0] ra_sel;

	assign ra_sel[0] = read_address == 4'h0;
	assign ra_sel[1] = read_address == 4'h1;
	assign ra_sel[2] = read_address == 4'h2;
	assign ra_sel[3] = read_address == 4'h3;
	assign ra_sel[4] = read_address == 4'h4;
	assign ra_sel[5] = read_address == 4'h5;
	assign ra_sel[6] = read_address == 4'h6;
	assign ra_sel[7] = read_address == 4'h7;
	assign ra_sel[8] = read_address == 4'h8;
	assign ra_sel[9] = read_address == 4'h9;
	assign ra_sel[10] = read_address == 4'ha;
	assign ra_sel[11] = read_address == 4'hb;
	assign ra_sel[12] = read_address == 4'hc;
	assign ra_sel[13] = read_address == 4'hd;
	assign ra_sel[14] = read_address == 4'he;
	assign ra_sel[15] = read_address == 4'hf;

	// Japanese console direction overrides for ports A and B
	wire [1:0] dir_a = JAP ? 2'h3 : PORT_A_d[6:5];
	wire [1:0] dir_b = JAP ? 2'h3 : PORT_B_d[6:5];

	// Reg 0: Version register
	//   bit 7: JAP (region), bits 6-1: test mode ? UART clocks : {PAL, DISK, 0000, tmss_enable}
	wire [7:0] reg_version = { JAP, test ? { uart_base_clk, uart_clk2_c, uart_clk1_c, uart_clk2_b, uart_clk1_b, uart_clk2_a, uart_clk1_a } :
		{ pal, DISK, 4'h0, tmss_enable } };

	// Reg 1: Port A data (M3 mode: full port; SMS mode: remapped)
	wire [7:0] reg_port_a_data = M3 ? { p_data_q_a[7], PORT_A_i } : { PORT_B_i[1:0], dir_a[0] & PORT_A_i[5], PORT_A_i[4], PORT_A_i[3:0] };

	// Reg 2: Port B data (M3 mode: full port; SMS mode: remapped)
	wire [7:0] reg_port_b_data = M3 ? { p_data_q_b[7], PORT_B_i } : { dir_b[1] & PORT_B_i[6], dir_a[1] & PORT_A_i[6], 2'h1, dir_b[0] & PORT_B_i[5], PORT_B_i[4:2] };

	// Reg 3: Port C data
	wire [7:0] reg_port_c_data = { p_data_q_c[7], PORT_C_i };

	// Regs 4-6: Port control registers
	wire [7:0] reg_port_a_ctrl = p_control_q_a;
	wire [7:0] reg_port_b_ctrl = p_control_q_b;
	wire [7:0] reg_port_c_ctrl = p_control_q_c;

	// Regs 7, 10, 13: TX data readback
	wire [7:0] reg_port_a_tx = tx_data_a;
	wire [7:0] reg_port_b_tx = tx_data_b;
	wire [7:0] reg_port_c_tx = tx_data_c;

	// Regs 8, 11, 14: RX data
	wire [7:0] reg_port_a_rx = rx_data_q_a;
	wire [7:0] reg_port_b_rx = rx_data_q_b;
	wire [7:0] reg_port_c_rx = rx_data_q_c;

	// Regs 9, 12, 15: Serial status {s_control[4:0], rx_error, rx_ready, tx_pending}
	wire [7:0] reg_port_a_serial_stat = { s_control_q_a, rx_frame_error_a, rx_data_ready_a, tx_pending_a };
	wire [7:0] reg_port_b_serial_stat = { s_control_q_b, rx_frame_error_b, rx_data_ready_b, tx_pending_b };
	wire [7:0] reg_port_c_serial_stat = { s_control_q_c, rx_frame_error_c, rx_data_ready_c, tx_pending_c };

	assign read_data =
		(ra_sel[0] ? reg_version : 8'h0) |
		(ra_sel[1] ? reg_port_a_data : 8'h0) |
		(ra_sel[2] ? reg_port_b_data : 8'h0) |
		(ra_sel[3] ? reg_port_c_data : 8'h0) |
		(ra_sel[4] ? reg_port_a_ctrl : 8'h0) |
		(ra_sel[5] ? reg_port_b_ctrl : 8'h0) |
		(ra_sel[6] ? reg_port_c_ctrl : 8'h0) |
		(ra_sel[7] ? reg_port_a_tx : 8'h0) |
		(ra_sel[8] ? reg_port_a_rx : 8'h0) |
		(ra_sel[9] ? reg_port_a_serial_stat : 8'h0) |
		(ra_sel[10] ? reg_port_b_tx : 8'h0) |
		(ra_sel[11] ? reg_port_b_rx : 8'h0) |
		(ra_sel[12] ? reg_port_b_serial_stat : 8'h0) |
		(ra_sel[13] ? reg_port_c_tx : 8'h0) |
		(ra_sel[14] ? reg_port_c_rx : 8'h0) |
		(ra_sel[15] ? reg_port_c_serial_stat : 8'h0);

	// -----------------------------------------------------------------------
	// Bus access control
	//   VDP read/write strobes and address decode for I/O register region.
	//   vdp_read_n/vdp_write_n: directly from CAS0/LWR gated by IO chip select.
	//   io_region_n: address $00-$0F in M3 mode (address[7:4] == 0).
	//   read_upper/write_lower/write_upper: split into two 8-register banks
	//   based on address[3].
	// -----------------------------------------------------------------------
	assign vdp_read_n = CAS0 | IO;
	assign vdp_write_n = LWR | IO;
	assign io_region_n = ~(M3 & address[7:4] == 4'h0);
	assign read_upper = ~io_region_n & address[3] & ~vdp_read_n;
	assign write_lower = ~io_region_n & ~address[3] & ~vdp_write_n;
	assign write_upper = ~io_region_n & address[3] & ~vdp_write_n;

	// -----------------------------------------------------------------------
	// Z80 (SMS mode) register access
	//   Z80 address $3E/$3F writes (M3=0), and specific read addresses for
	//   SMS-compatible port readback.
	// -----------------------------------------------------------------------
	assign z80_write_n = ~(address[7:1] == 7'h1f & ~M3 & ~vdp_write_n); // $3E/$3F write
	assign z80_read_n = ~((address & 8'he2) == 8'hc0
		& (address[4:2] == 3'h0 | address[4:2] == 3'h7) & ~M3);        // SMS port read
	assign z80_access = (z80_write_n & z80_read_n) & io_region_n;

	assign z80_write_even = z80_write_n | address[0];  // $3E (even)
	assign z80_write_odd = z80_write_n | ~address[0];   // $3F (odd)

	// -----------------------------------------------------------------------
	// Per-register write enables
	//   Each controller port register has its own write strobe, active-low.
	//   Lower bank (address[3]=0): data and control registers.
	//   Upper bank (address[3]=1): serial control and TX data.
	// -----------------------------------------------------------------------
	assign read_rx_data_a = ~(read_upper & address[2:0] == 3'h0);
	assign read_rx_data_b = ~(read_upper & address[2:0] == 3'h3);
	assign read_rx_data_c = ~(read_upper & address[2:0] == 3'h6);
	assign write_p_data_a = ~(write_lower & address[2:0] == 3'h1);
	assign write_p_data_b = ~(write_lower & address[2:0] == 3'h2);
	assign write_p_data_c = ~(write_lower & address[2:0] == 3'h3);
	assign write_p_control_a = ~(write_lower & address[2:0] == 3'h4);
	assign write_p_control_b = ~(write_lower & address[2:0] == 3'h5);
	assign write_p_control_c = ~(write_lower & address[2:0] == 3'h6);
	assign write_tx_data_a = ~(write_lower & address[2:0] == 3'h7);
	assign write_s_control_a = ~(write_upper & address[2:0] == 3'h1);
	assign write_tx_data_b = ~(write_upper & address[2:0] == 3'h2);
	assign write_s_control_b = ~(write_upper & address[2:0] == 3'h4);
	assign write_tx_data_c = ~(write_upper & address[2:0] == 3'h5);
	assign write_s_control_c = ~(write_upper & address[2:0] == 3'h7);

	// -----------------------------------------------------------------------
	// Z80 port direction/output registers ($3E, $3F)
	//   reg_3e: bit 4 controls nationalization (used externally)
	//   reg_3f: bits [7:0] control port A/B upper pin directions and outputs
	//           in SMS compatibility mode
	// -----------------------------------------------------------------------
	ym_sdffr reg_3e(.MCLK(MCLK), .clk(z80_write_even), .val(data_bus[4]), .reset(reset), .q(reg_3e_q));
	ym_sdffs #(.DATA_WIDTH(8)) reg_3f(.MCLK(MCLK), .clk(z80_write_odd), .val(data_bus), .set(reset), .q(reg_3f_data));

	// In SMS mode (M3=0), port direction bits [6:5] come from reg_3f
	assign PORT_A_d = M3 ? port_a_d : { reg_3f_data[1:0], port_a_d[4:0] };
	assign PORT_B_d = M3 ? port_b_d : { reg_3f_data[3:2], port_b_d[4:0] };
	assign PORT_C_d = port_c_d;

	// In SMS mode (M3=0), port output bits [6:5] come from reg_3f
	assign PORT_A_o = M3 ? port_a_o : { reg_3f_data[5:4], port_a_o[4:0] };
	assign PORT_B_o = M3 ? port_b_o : { reg_3f_data[7:6], port_b_o[4:0] };
	assign PORT_C_o = port_c_o;

	// -----------------------------------------------------------------------
	// I/O chip select and bus arbitration
	// -----------------------------------------------------------------------
	assign io_chip_active = ~(z80_access | IO | CAS0);

	assign upper_byte_sel = M3 & ~ZA_i[0];

	// Bus direction control: arbitrate between VDP and Z80 bus ownership
	assign arb_gate1 = ~(io_chip_active & M3) & (ZV | ~CAS0) & (VZ | CAS0);
	assign arb_gate2 = (ZV | CAS0) & (VZ | ~CAS0);

	assign ZA_OE = VZ | t1;
	assign VD_LO_OE_n = arb_gate1 | t1;
	assign VD_HI_OE_n = (arb_gate1 & M3) | t1;
	assign ZD_OE_n = arb_gate2 | t1;
	assign VA_LO_OE_n = ZV | t1;

	// -----------------------------------------------------------------------
	// Data output mux
	//   When io_chip_active, output register read data.
	//   Otherwise, pass through external bus data.
	// -----------------------------------------------------------------------
	assign vdata = io_chip_active ? read_data : ZD_i[7:0];
	assign zdata = io_chip_active ? read_data : (upper_byte_sel ? VD_i[15:8] : VD_i[7:0]);

	// -----------------------------------------------------------------------
	// Interrupt logic (HL output — directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low)
	//   M3 mode: HL low if any port has pin-6 IRQ or UART RX ready IRQ.
	//   SMS mode: HL low if pin-6 of port A or B is low with direction set.
	// -----------------------------------------------------------------------
	assign HL = M3 ?
		~(irq_b6_a | irq_uart_a | irq_b6_b | irq_uart_b | irq_b6_c | irq_uart_c) :
		~((PORT_A_d[6] & ~PORT_A_i[6]) | (PORT_B_d[6] & ~PORT_B_i[6]));

endmodule


// =======================================================================
// Controller port submodule
//   Each controller port (A, B, C) has:
//     - 8-bit parallel data register (directly accessible by CPU)
//     - 8-bit direction control register
//     - UART transmitter with 8-bit shift register and 4-bit FSM
//     - UART receiver with input synchronizer, two FSMs, and shift register
//     - Serial control register (baud rate select, enable)
//     - IRQ generation (pin-6 edge and UART RX ready)
// =======================================================================
module ym6046_controller_port
	(
	input MCLK,
	input [6:0] port_i,
	input [7:0] data_bus,
	input reset,
	input m3,
	input [3:0] uart_clk_i1,
	input [3:0] uart_clk_i2,
	input read_rx_data,
	input write_p_data,
	input write_tx_data,
	input write_s_control,
	input write_p_control,
	output uart_clk1,
	output uart_clk2,
	output [7:0] p_data_q,
	output [7:0] p_control_q,
	output [7:0] tx_data,
	output [7:0] rx_data_q,
	output [4:0] s_control_q,
	output tx_pending,
	output rx_data_ready,
	output rx_frame_error,
	output [6:0] port_d,
	output [6:0] port_o,
	output irq_b6,
	output irq_uart
	);

	// -----------------------------------------------------------------------
	// TX shift register and FSM
	// -----------------------------------------------------------------------
	wire [7:0] tx_shift_reg;       // 8-bit TX shift register (shifts right)
	wire tx_step;                  // shift enable (active when FSM is in data state)
	wire tx_serial_bit;            // current serial output bit (directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low directly active-low of shift[0])
	wire tx_fsm1_q, tx_fsm1_nq;   // TX FSM bit 1 (4-bit counter for 8 data + start + stop)
	wire tx_fsm2_q, tx_fsm2_nq;   // TX FSM bit 2
	wire tx_fsm3_q, tx_fsm3_nq;   // TX FSM bit 3
	wire tx_fsm4_q, tx_fsm4_nq;   // TX FSM bit 4
	wire tx_fsm5_q;                // TX FSM completion flag
	wire tx_pending_nq;
	wire tx_active;                // TX in progress (loaded from pending via FSM)
	wire tx_active_synced;         // tx_active sampled to uart_clk1 domain
	wire rx_input_synced, rx_input_synced_nq; // RX pin synchronized to uart_clk1

	// -----------------------------------------------------------------------
	// RX synchronizer FSM (stage 1 — bit-rate recovery)
	// -----------------------------------------------------------------------
	wire rx_sync_fsm1_nq;
	wire rx_sync_fsm2_q, rx_sync_fsm2_nq;
	wire rx_sync_fsm3_q, rx_sync_fsm3_nq;
	wire rx_sync_fsm4_q, rx_sync_fsm4_nq;
	wire rx_sync_fsm5_q, rx_sync_fsm5_nq;
	wire rx_bit_clk;               // recovered bit clock from sync FSM

	// -----------------------------------------------------------------------
	// RX capture FSM (stage 2 — byte framing)
	// -----------------------------------------------------------------------
	wire rx_cap_fsm1_q, rx_cap_fsm1_nq;
	wire rx_cap_fsm2_q, rx_cap_fsm2_nq;
	wire rx_cap_fsm3_q, rx_cap_fsm3_nq;
	wire rx_cap_fsm4_q, rx_cap_fsm4_nq;
	wire rx_cap_fsm5_q;
	wire rx_byte_clk;              // byte-complete clock from capture FSM
	wire [7:0] rx_shift_reg;       // 8-bit RX shift register
	reg [7:0] rx_shift_reg_delay;  // one-cycle delayed copy for data register

	// -----------------------------------------------------------------------
	// Parallel control register
	//   Bits [6:0] control pin direction (0=input, 1=output).
	//   Bit 7 is the TH (pin-6) interrupt enable.
	// -----------------------------------------------------------------------
	ym_sdffr #(.DATA_WIDTH(8)) p_control(.MCLK(MCLK), .clk(write_p_control), .val(data_bus), .reset(reset & m3), .q(p_control_q));

	// Port direction: inverted control bits masked by serial mode
	//   s_control[1]=1 (serial mode): pin 4 becomes TX output, pin 6 forced input
	//   s_control[2]=1: pin 5 forced as RX input direction
	assign port_d = ((~p_control_q[6:0]) & (s_control_q[1] ? 7'h6f : 7'h7f)) | (s_control_q[2] ? 7'h20 : 7'h0);

	// -----------------------------------------------------------------------
	// Parallel data register
	// -----------------------------------------------------------------------
	ym_sdffr #(.DATA_WIDTH(8)) p_data(.MCLK(MCLK), .clk(write_p_data), .val(data_bus), .reset(1'h1), .q(p_data_q));

	// Port output: pin 4 is serial TX bit when in serial mode
	assign port_o[6:5] = p_data_q[6:5];
	assign port_o[4] = s_control_q[1] ? tx_serial_bit : p_data_q[4];
	assign port_o[3:0] = p_data_q[3:0];

	// -----------------------------------------------------------------------
	// Serial control register
	//   [4:3]: baud rate select
	//   [2]:   RX enable
	//   [1]:   serial mode enable (TX on pin 4, RX on pin 5)
	//   [0]:   RX interrupt enable
	// -----------------------------------------------------------------------
	ym_sdffr #(.DATA_WIDTH(5)) s_control(.MCLK(MCLK), .clk(write_s_control), .val(data_bus[7:3]), .reset(reset & m3), .q(s_control_q));

	// TX data latch (active-low enable)
	ym_slatch #(.DATA_WIDTH(8)) tx_data_sl(.MCLK(MCLK), .en(~write_tx_data), .inp(data_bus), .val(tx_data));

	// Baud rate clock selection (2 clocks per port, selected by s_control[4:3])
	assign uart_clk1 = uart_clk_i1[s_control_q[4:3]];
	assign uart_clk2 = uart_clk_i2[s_control_q[4:3]];

	// -----------------------------------------------------------------------
	// TX shift register
	//   When tx_step is low (loading), inverted TX data is loaded.
	//   When tx_step is high (shifting), register shifts right with 0 fill.
	//   Clocked by uart_clk2 (slower baud clock). Reset by serial mode enable.
	// -----------------------------------------------------------------------
	ym_sdffr #(.DATA_WIDTH(8)) tx_shifter(.MCLK(MCLK), .clk(uart_clk2), .val(tx_step ? { 1'h0, tx_shift_reg[7:1] } : ~tx_data),
		.reset(s_control_q[1]), .q(tx_shift_reg));

	// TX serial output bit: inverted LSB of shift register during shift, high (stop/idle) otherwise
	ym_sdffs tx_bit(.MCLK(MCLK), .clk(uart_clk2), .val(~tx_shift_reg[0] & tx_step), .set(s_control_q[1]), .q(tx_serial_bit));

	// -----------------------------------------------------------------------
	// TX FSM — 4-bit counter (10 states: start + 8 data + stop)
	//   Combinational next-state logic using sum-of-products.
	//   tx_step goes high when the FSM is in data-shifting states.
	// -----------------------------------------------------------------------
	wire t_i1 = (tx_fsm4_q & tx_fsm1_nq)
		| (tx_fsm1_nq & tx_fsm4_nq & tx_active_synced)
		| (tx_fsm4_q & tx_fsm1_q & tx_fsm2_q);
	wire t_i2 = (tx_fsm1_nq & tx_fsm4_nq & tx_active_synced) | (tx_fsm1_q & tx_fsm2_q)
		| (tx_fsm2_nq & tx_fsm1_nq & tx_fsm3_q & tx_fsm4_nq);
	wire t_i3 = (tx_fsm1_nq & tx_fsm4_nq & tx_active_synced)
		| (tx_fsm4_q & tx_fsm1_q & tx_fsm2_q)
		| (tx_fsm2_q & tx_fsm4_q & tx_fsm3_q)
		| (tx_fsm4_q & tx_fsm3_q & tx_fsm1_q);
	wire t_i4 = (tx_fsm1_q & tx_fsm2_q)
		| (tx_fsm2_nq & tx_fsm1_nq & tx_fsm3_q & tx_fsm4_nq)
		| (tx_fsm4_q & tx_fsm1_q)
		| (tx_fsm4_q & tx_fsm2_q);
	wire t_i5 = ~(tx_fsm1_nq & tx_fsm2_nq & tx_fsm3_nq & tx_fsm4_q);

	ym_sdffr tx_fsm1(.MCLK(MCLK), .clk(uart_clk2), .val(t_i1), .reset(reset), .q(tx_fsm1_q), .nq(tx_fsm1_nq));
	ym_sdffr tx_fsm2(.MCLK(MCLK), .clk(uart_clk2), .val(t_i2), .reset(reset), .q(tx_fsm2_q), .nq(tx_fsm2_nq));
	ym_sdffr tx_fsm3(.MCLK(MCLK), .clk(uart_clk2), .val(t_i3), .reset(reset), .q(tx_fsm3_q), .nq(tx_fsm3_nq));
	ym_sdffr tx_fsm4(.MCLK(MCLK), .clk(uart_clk2), .val(t_i4), .reset(reset), .q(tx_fsm4_q), .nq(tx_fsm4_nq));
	ym_sdffs tx_fsm5(.MCLK(MCLK), .clk(uart_clk2), .val(t_i5), .set(reset), .q(tx_fsm5_q));

	// tx_step: high when FSM is NOT in idle state (all bits zero except tx_active_synced)
	assign tx_step = ~(tx_active_synced & tx_fsm1_nq & tx_fsm2_nq & tx_fsm3_nq & tx_fsm4_nq);

	// -----------------------------------------------------------------------
	// TX state machine — pending/active handshake
	//   tx_pending: set when CPU writes TX data, cleared when FSM begins shifting
	//   tx_active:  set by tx_pending, cleared by FSM completion (tx_fsm5_q)
	//   tx_active_synced: tx_active synchronized to uart_clk1
	// -----------------------------------------------------------------------
	ym_sdffsr tx_state1(.MCLK(MCLK), .clk(uart_clk2), .val(~tx_step & tx_pending), .set(write_tx_data), .reset(reset),
		.q(tx_pending), .nq(tx_pending_nq));
	ym_sdffsr tx_state2(.MCLK(MCLK), .clk(tx_fsm5_q), .val(1'h0), .set(tx_pending_nq), .reset(reset),
		.q(tx_active));

	ym_sdff tx_active_sync(.MCLK(MCLK), .clk(uart_clk1), .val(tx_active), .q(tx_active_synced));

	// -----------------------------------------------------------------------
	// RX input synchronizer
	//   Samples port pin 5 on uart_clk1 rising edge. Reset by RX enable
	//   (s_control[2]).
	// -----------------------------------------------------------------------
	ym_sdffs rx_input_bit(.MCLK(MCLK), .clk(uart_clk1), .val(port_i[5]), .set(s_control_q[2]), .q(rx_input_synced), .nq(rx_input_synced_nq));

	// -----------------------------------------------------------------------
	// RX synchronizer FSM (stage 1 — 5-bit, clocked by uart_clk1)
	//   Detects start bit and generates rx_bit_clk at the correct phase.
	//   r1_j triggers on start bit detection (input low, FSM idle, capture idle).
	// -----------------------------------------------------------------------
	wire r1_j = ~(rx_sync_fsm1_nq | ~(rx_cap_fsm1_nq & rx_cap_fsm4_nq) | rx_input_synced);
	wire r1_i1 = ~((rx_sync_fsm2_q | r1_j) & (rx_sync_fsm1_nq | r1_j));
	wire r1_i2 = (rx_sync_fsm4_nq & rx_sync_fsm5_q & rx_sync_fsm3_nq & ~rx_sync_fsm2_q)
		| (rx_sync_fsm2_q & rx_sync_fsm3_q)
		| (rx_sync_fsm2_q & rx_sync_fsm5_nq)
		| (rx_sync_fsm2_q & rx_sync_fsm4_q)
		| r1_j;
	wire r1_i3 = r1_j
		| (rx_sync_fsm3_q & rx_sync_fsm4_q)
		| (rx_sync_fsm4_nq & rx_sync_fsm5_q & rx_sync_fsm3_nq)
		| (rx_sync_fsm3_q & rx_sync_fsm5_nq);
	wire r1_i4 = r1_j
		| (rx_sync_fsm5_nq & rx_sync_fsm4_q)
		| (rx_sync_fsm5_q & rx_sync_fsm4_nq);
	wire r1_i5 = ~(r1_j | rx_sync_fsm5_q);

	ym_sdffr rx_sync_fsm1(.MCLK(MCLK), .clk(uart_clk1), .val(r1_i1), .reset(reset), .nq(rx_sync_fsm1_nq));
	ym_sdffs rx_sync_fsm2(.MCLK(MCLK), .clk(uart_clk1), .val(r1_i2), .set(reset), .q(rx_sync_fsm2_q), .nq(rx_sync_fsm2_nq));
	ym_sdffs rx_sync_fsm3(.MCLK(MCLK), .clk(uart_clk1), .val(r1_i3), .set(reset), .q(rx_sync_fsm3_q), .nq(rx_sync_fsm3_nq));
	ym_sdffs rx_sync_fsm4(.MCLK(MCLK), .clk(uart_clk1), .val(r1_i4), .set(reset), .q(rx_sync_fsm4_q), .nq(rx_sync_fsm4_nq));
	ym_sdffr rx_sync_fsm5(.MCLK(MCLK), .clk(uart_clk1), .val(r1_i5), .reset(reset), .q(rx_sync_fsm5_q), .nq(rx_sync_fsm5_nq));

	assign rx_bit_clk = rx_sync_fsm2_nq;

	// -----------------------------------------------------------------------
	// RX capture FSM (stage 2 — 5-bit, clocked by uart_clk1)
	//   Counts received bits (start + 8 data + stop). r2_j triggers on
	//   start bit detection. Generates rx_byte_clk when a full byte is received.
	// -----------------------------------------------------------------------
	wire r2_j = (rx_cap_fsm1_nq & rx_cap_fsm4_nq && ~rx_input_synced & rx_sync_fsm1_nq);
	wire r2_i1 = r2_j
		| (rx_cap_fsm1_q & rx_cap_fsm2_q & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm3_q & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm4_q);
	wire r2_i2 = r2_j
		| (rx_cap_fsm1_q & rx_cap_fsm2_nq & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm3_q & rx_cap_fsm4_q)
		| (rx_cap_fsm1_q & rx_cap_fsm2_q & rx_cap_fsm4_q);
	wire r2_i3 = r2_j
		| (rx_cap_fsm1_q & rx_cap_fsm2_nq & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm2_q & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm2_q & rx_cap_fsm3_q);
	wire r2_i4 = r2_j
		| (rx_cap_fsm1_q & rx_cap_fsm2_nq & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm2_q & rx_cap_fsm3_nq & rx_cap_fsm4_nq)
		| (rx_cap_fsm1_q & rx_cap_fsm3_q & rx_cap_fsm4_nq);
	wire r2_i5 = ~(rx_cap_fsm1_q & rx_cap_fsm2_nq & rx_cap_fsm3_nq & rx_cap_fsm4_nq);

	ym_sdffr rx_cap_fsm1(.MCLK(MCLK), .clk(uart_clk1), .val(r2_i1), .reset(reset), .q(rx_cap_fsm1_q), .nq(rx_cap_fsm1_nq));
	ym_sdffr rx_cap_fsm2(.MCLK(MCLK), .clk(uart_clk1), .val(r2_i2), .reset(reset), .q(rx_cap_fsm2_q), .nq(rx_cap_fsm2_nq));
	ym_sdffr rx_cap_fsm3(.MCLK(MCLK), .clk(uart_clk1), .val(r2_i3), .reset(reset), .q(rx_cap_fsm3_q), .nq(rx_cap_fsm3_nq));
	ym_sdffr rx_cap_fsm4(.MCLK(MCLK), .clk(uart_clk1), .val(r2_i4), .reset(reset), .q(rx_cap_fsm4_q), .nq(rx_cap_fsm4_nq));
	ym_sdffs rx_cap_fsm5(.MCLK(MCLK), .clk(uart_clk1), .val(r2_i5), .set(reset), .q(rx_cap_fsm5_q));
	assign rx_byte_clk = rx_bit_clk | rx_cap_fsm5_q;

	// -----------------------------------------------------------------------
	// RX shift register, data register, and status flags
	// -----------------------------------------------------------------------
	ym_sdffr #(.DATA_WIDTH(8)) rx_shifter(.MCLK(MCLK), .clk(rx_bit_clk), .val({ rx_shift_reg[6:0], rx_input_synced }),
		.reset(s_control_q[2]), .q(rx_shift_reg));

	// rx_data_ready: set when byte is complete, cleared on reset or CPU read
	ym_sdffr rx_ready(.MCLK(MCLK), .clk(rx_byte_clk), .val(1'h1), .reset(reset & read_rx_data), .q(rx_data_ready));
	// rx_frame_error: set if stop bit is low (framing error), cleared on reset or CPU read
	ym_sdffr rx_error(.MCLK(MCLK), .clk(rx_byte_clk), .val(rx_input_synced_nq), .reset(reset & read_rx_data), .q(rx_frame_error));
	// RX data register: captures shift register contents when byte is complete
	ym_sdffr #(.DATA_WIDTH(8)) rx_data(.MCLK(MCLK), .clk(rx_byte_clk), .val(rx_shift_reg_delay), .reset(1'h1), .q(rx_data_q));

	// -----------------------------------------------------------------------
	// Interrupt generation
	//   irq_b6: pin 6 is low AND interrupt enabled (p_control[7])
	//   irq_uart: RX data ready AND RX interrupt enabled (s_control[0])
	// -----------------------------------------------------------------------
	assign irq_b6 = ~port_i[6] & p_control_q[7];
	assign irq_uart = rx_data_ready & s_control_q[0];

	// One-cycle delay for RX shift register (ensures stable data at capture)
	always @(posedge MCLK)
	begin
		rx_shift_reg_delay <= rx_shift_reg;
	end
endmodule

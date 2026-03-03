/*
 * VRAM controller
 *
 * Emulates the MegaDrive's 64 KB Video RAM using DRAM-style RAS/CAS addressing.
 * Supports two access paths:
 *   - Parallel: random-access read/write at (row, column) for CPU/VDP operations.
 *   - Serial:   snapshot entire memory on OE rising edge (when in serial mode),
 *               then shift out bytes one-per-SC-cycle for background rendering.
 *
 * Memory is implemented as 8 x 256-byte RAM blocks (2 KB total per instance;
 * the VDP instantiates multiple vram modules to cover full VRAM).
 */

module vram
	(
	input MCLK,
	input RAS,
	input CAS,
	input WE,
	input OE,
	input SC,
	input SE,
	input [7:0] AD,
	input [7:0] RD_i,
	output reg [7:0] RD_o,
	output RD_d,
	output [7:0] SD_o,
	output SD_d
	);

	// -----------------------------------------------------------------------
	// Address and mode registers
	// -----------------------------------------------------------------------
	reg [15:0] addr;            // full 16-bit DRAM address: [15:8]=row, [7:0]=column
	reg serial_mode;            // set when OE is low at RAS falling edge (serial transfer)
	reg [7:0] serial_ptr;       // read pointer into serial snapshot
	reg [2047:0] serial_snapshot; // snapshot of all memory for serial readout

	// -----------------------------------------------------------------------
	// Edge-detect registers (previous cycle values)
	// -----------------------------------------------------------------------
	reg prev_oe;
	reg prev_ras;
	reg prev_cas;
	reg prev_sc;
	reg data_valid;

	// -----------------------------------------------------------------------
	// RAS/CAS decode
	//   DRAM-style strobes: active when RAS is asserted (active-low).
	//   col_strobe: RAS + CAS both active
	//   write_en:   RAS + CAS + WE all active
	//   read_en:    RAS + CAS + OE active, and NOT in serial mode
	// -----------------------------------------------------------------------
	wire col_strobe = ~RAS & ~CAS;
	wire write_en   = ~RAS & ~CAS & ~WE;
	wire read_en    = ~RAS & ~CAS & ~OE & ~serial_mode;

	// -----------------------------------------------------------------------
	// Memory array (8 banks x 256 bytes x 8 bits = 2 KB per instance)
	//   row_addr selects the 256-byte row; byte_sel decodes which of 32 byte
	//   positions within each bank to write.
	// -----------------------------------------------------------------------
	wire [7:0] row_addr = addr[15:8];
	wire [31:0] byte_sel;
	wire [2047:0] mem_out;

	wire [7:0] slice_serial[0:255];  // serial snapshot sliced into bytes
	wire [7:0] slice_parallel[0:255]; // parallel read data sliced into bytes

	genvar i;
	generate
		for (i = 0; i < 32; i = i + 1)
		begin : byte_decode
			assign byte_sel[i] = addr[4:0] == i;
		end
		for (i = 0; i < 8; i = i + 1)
		begin : mem_banks
			vram_ip mem
				(
				.clock(MCLK),
				.address(row_addr),
				.byteena(byte_sel),
				.data({32{RD_i}}),
				.wren(write_en & (addr[7:5] == i)),
				.q(mem_out[(256*(i+1)-1):(256*i)])
				);
		end
		for (i = 0; i < 256; i = i + 1)
		begin : byte_slicing
			assign slice_parallel[i] = mem_out[(8*(i+1)-1):(8*i)];
			assign slice_serial[i] = serial_snapshot[(8*(i+1)-1):(8*i)];
		end
	endgenerate

	// -----------------------------------------------------------------------
	// Output enables
	//   RD_d: active-low — drives parallel data bus when data_valid is set
	//   SD_d: serial data bus driven when SE (serial enable) is asserted
	// -----------------------------------------------------------------------
	assign RD_d = ~data_valid;
	assign SD_d = SE;

	// -----------------------------------------------------------------------
	// Serial byte output register
	// -----------------------------------------------------------------------
	reg [7:0] serial_byte_out;

	assign SD_o = serial_byte_out;

	// -----------------------------------------------------------------------
	// Main clocked logic
	// -----------------------------------------------------------------------
	always @(posedge MCLK)
	begin
		// --- Serial snapshot capture ---
		// On OE rising edge while in serial mode: capture all memory contents
		// and reset the serial read pointer to the current column address.
		if (serial_mode & !prev_oe & OE)
		begin
			serial_ptr <= addr[7:0];
			serial_snapshot <= mem_out;
		end
		// --- Serial shift ---
		// On SC rising edge: output current byte and advance pointer.
		else if (~prev_sc & SC)
		begin
			serial_ptr <= serial_ptr + 8'h1;
			serial_byte_out <= slice_serial[serial_ptr];
		end

		// --- Row address latch (RAS falling edge) ---
		// Also captures serial_mode from OE state at this moment.
		if (prev_ras & ~RAS)
		begin
			serial_mode <= ~OE;
			addr[15:8] <= AD;
		end

		// --- Column address latch (CAS rising edge) ---
		if (~prev_cas & col_strobe)
		begin
			addr[7:0] <= AD;
		end

		// --- Serial snapshot (duplicate block from original netlist) ---
		if (serial_mode & !prev_oe & OE)
		begin
			serial_ptr <= addr[7:0];
			serial_snapshot <= mem_out;
		end

		// --- Parallel read path ---
		if (read_en)
		begin
			RD_o <= slice_parallel[addr[7:0]];
			data_valid <= 1'h1;
		end
		else if (CAS | OE)
		begin
			data_valid <= 1'h0;
		end

		// --- Edge-detect flip-flops ---
		prev_oe  <= OE;
		prev_ras <= RAS;
		prev_cas <= col_strobe;
		prev_sc  <= SC;
	end

endmodule

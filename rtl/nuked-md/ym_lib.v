// ym3438, ym7101, fc1004 common cells

// Two-phase shift register bit: captures on c1, transfers on c2
module ym_sr_bit #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);

	reg [SR_LENGTH-1:0] master = 0;
	reg [SR_LENGTH-1:0] slave = 0;

	wire [SR_LENGTH-1:0] slave_next = c2 ? master : slave;

	//assign sr_out = slave_next[SR_LENGTH-1];
	assign sr_out = slave[SR_LENGTH-1];

	always @(posedge MCLK)
	begin
		if (c1)
		begin
			if (SR_LENGTH == 1)
				master <= bit_in;
			else
				master <= { slave[SR_LENGTH-2:0], bit_in };
		end
		slave <= slave_next;
	end
endmodule

/*module ym_sr_bit #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);
	
	reg [SR_LENGTH-1:0] v1 = 0;
	reg [SR_LENGTH-1:0] v2 = 0;
	
	assign sr_out = v2[SR_LENGTH-1];
	
	always @(*)
	begin
		if (c1)
		begin
			if (SR_LENGTH == 1)
				v1 <= bit_in;
			else
				v1 <= { v2[SR_LENGTH-2:0], bit_in };
		end
		if (c2)
		begin
			v2 <= v1;
		end
	end
endmodule*/

/*module ym_sr_bit2 #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);
	
	reg [SR_LENGTH-1:0] v2 = 0;
	
	assign sr_out = v2[SR_LENGTH-1];
	
	always @(posedge c2)
	begin
		if (SR_LENGTH == 1)
			v2 <= bit_in;
		else
			v2 <= { v2[SR_LENGTH-2:0], bit_in };
	end
endmodule*/

// Parallel array of ym_sr_bit instances
module ym_sr_bit_array #(parameter SR_LENGTH = 1, DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input [DATA_WIDTH-1:0] data_in,
	output [DATA_WIDTH-1:0] data_out
	);

	wire out[0:DATA_WIDTH-1];

	generate
		genvar i;
		for (i = 0; i < DATA_WIDTH; i = i + 1)
		begin : bits
			ym_sr_bit #(.SR_LENGTH(SR_LENGTH)) sr (
			.MCLK(MCLK),
			.c1(c1),
			.c2(c2),
			.bit_in(data_in[i]),
			.sr_out(out[i])
			);
			
			assign data_out[i] = out[i];
		end
	endgenerate

endmodule

// Binary counter built on shift register array
module ym_cnt_bit #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

/*module ym_cnt_bit2
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	output val,
	output c_out
	);
	
	wire data_in;
	wire data_out;
	wire [1:0] sum;
	
	ym_sr_bit2 mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.bit_in(data_in),
		.sr_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {1'h0, c_in};
	assign val = data_out;
	assign data_in = reset ? 1'h0 : sum[0];
	assign c_out = sum[1];
	
endmodule*/

// D-latch transparent during phase c1
module ym_dlatch_1 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);

	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH-1:0] mem_assign = c1 ? inp : mem;

	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end

	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;

endmodule

/*module ym_dlatch_1 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (c1)
			mem <= inp;
	end

	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

// D-latch transparent during phase c2
module ym_dlatch_2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c2,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);

	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH-1:0] mem_assign = c2 ? inp : mem;

	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end

	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;

endmodule

/*module ym_dlatch_2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c2,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (c2)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

// Rising-edge detector using c1-clocked latch
module ym_edge_detect
	(
	input MCLK,
	input c1,
	input inp,
	output outp
	);

	wire prev_out;
	
	ym_dlatch_1 prev
		(
		.MCLK(MCLK),
		.c1(c1),
		.inp(inp),
		.val(prev_out),
		.nval()
		);
	assign outp = ~(prev_out | ~inp);
endmodule

/*module ym_edge_detect
	(
	input MCLK,
	input c1,
	input inp,
	output outp
	);
	
	reg prev_out;
	
	always @(posedge c1)
	begin
		prev_out <= inp;
	end
	
	assign outp = ~(prev_out | ~inp);
endmodule*/

// Simple enable-gated latch (active-high en)
module ym_slatch #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);

	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;

	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end

	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;

endmodule

/*module ym_slatch #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (en)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

/*module ym_slatch2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

// Transparent variant of ym_slatch (combinational output)
module ym_slatch_t #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);

	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;

	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end

	assign val = mem_assign;
	assign nval = ~mem_assign;

endmodule

/*module ym_slatch_t #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (en)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

// RS flip-flop (set/reset trigger)
module ym_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output reg q = 1'h0,
	output reg nq = 1'h1
	);
	
	always @(posedge MCLK)
	begin
		q <= rst ? 1'h0 : (set ? 1'h1 : q);
		nq <= set ? 1'h0 : (rst ? 1'h1 : ~q); 
	end
	
endmodule

/*module ym_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output q,
	output nq
	);
	
	assign q = ~(rst | nq);
	assign nq = ~(set | q);
	
endmodule*/

// RS flip-flop gated by clock phase c1
module ym_rs_trig_sync
	(
	input MCLK,
	input set,
	input rst,
	input c1,
	output reg q = 1'h0,
	output reg nq = 1'h1
	);
	
	always @(posedge MCLK)
	begin
		q <= (c1 & rst) ? 1'h0 : ((c1 & set) ? 1'h1 : q);
		nq <= (c1 & set) ? 1'h0 : ((c1 & rst) ? 1'h1 : ~q); 
	end
	
endmodule

/*module ym_rs_trig_sync
	(
	input MCLK,
	input set,
	input rst,
	input c1,
	output q,
	output nq
	);
	
	assign q = ~((c1 & rst) | nq);
	assign nq = ~((c1 & set) | q);
	
endmodule*/

// Counter with parallel load
module ym_cnt_bit_load #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	wire [DATA_WIDTH-1:0] base_val = load ? load_val : data_out;
	
	assign sum = {1'h0, base_val} + {{DATA_WIDTH{1'h0}},c_in};
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign val = data_out;
	assign c_out = sum[DATA_WIDTH];
	
endmodule

// Debug scan chain (MSB-first readout)
module ym_dbg_read #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input prev,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output next
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
		
	wire [DATA_WIDTH-1:0] chain;
	
	assign data_in = chain | (load ? load_val : {DATA_WIDTH{1'h0}});
	
	generate
		if (DATA_WIDTH == 1)
			assign chain = prev;
		else
			assign chain = { prev, data_out[DATA_WIDTH-1:1] };
	endgenerate
	
	assign next = data_out[0];
	
endmodule

// Debug scan chain (LSB-first readout, EG variant)
module ym_dbg_read_eg #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input prev,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output next
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
		
	wire [DATA_WIDTH-1:0] chain;
	
	assign data_in = chain | (load ? load_val : {DATA_WIDTH{1'h0}});
	
	generate
		if (DATA_WIDTH == 1)
			assign chain = prev;
		else
			assign chain = { data_out[DATA_WIDTH-2:0], prev };
	endgenerate
	
	assign next = data_out[DATA_WIDTH-1];
	
endmodule

// Latch with synchronous reset
module ym_slatch_r #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input rst,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);

	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH-1:0] mem_assign = rst ? {DATA_WIDTH{1'h0}} : (en ? inp : mem);

	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end

	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;

endmodule

/*module ym_slatch_r #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input rst,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (rst)
			mem <= {DATA_WIDTH{1'h0}};
		else if (en)
			mem <= inp;
	end

	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

// Counter with set/reset
module ym_cnt_bit_rs #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	input set,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH-1:0] data_out_s = set ? {DATA_WIDTH{1'h1}} : data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = {1'h0,data_out_s} + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out_s;
	assign nval = ~data_out_s;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

// Reversible (up/down) counter
module ym_cnt_bit_rev #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input dec,
	input reset,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {1'h0, {DATA_WIDTH{dec}}} + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

// Shift register with shift/hold enable
module ym_sr_bit_en #(parameter SR_LENGTH = 2)
	(
	input MCLK,
	input c1,
	input c2,
	input en1,
	input en2,
	input data_in,
	output [SR_LENGTH-1:0] data_out
	);
	
	wire [SR_LENGTH-1:0] sr_out;
	wire [SR_LENGTH-1:0] sr_in =
		(en1 ? { sr_out[SR_LENGTH-2:0], data_in } : {SR_LENGTH{1'h0}}) |
		(en2 ? sr_out : {SR_LENGTH{1'h0}});
	
	assign data_out = sr_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(SR_LENGTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(sr_in),
		.data_out(sr_out)
		);

endmodule


// Synchronous counter with master-slave latches
module ym_scnt_bit #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input load,
	input [DATA_WIDTH-1:0] val,
	input cin,
	input rst,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq,
	output cout
	);

	reg [DATA_WIDTH-1:0] master = {DATA_WIDTH{1'h0}}, slave = {DATA_WIDTH{1'h0}};

	wire [DATA_WIDTH:0] sum = { 1'h0, slave } + {{DATA_WIDTH{1'h0}}, cin};

	assign cout = sum[DATA_WIDTH];

	assign q = slave;
	assign nq = ~slave;

	always @(posedge MCLK)
	begin
		if (~rst)
		begin
			master <= {DATA_WIDTH{1'h0}};
			slave <= {DATA_WIDTH{1'h0}};
		end
		else
		begin
			if (~clk)
				master <= ~load ? val : sum[DATA_WIDTH-1:0];
			else
				slave <= master;
		end
	end

endmodule


// Synchronous D flip-flop (master-slave)
module ym_sdff #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);

	reg [DATA_WIDTH-1:0] master = {DATA_WIDTH{1'h0}}, slave = {DATA_WIDTH{1'h0}};

	assign q = slave;
	assign nq = ~slave;

	always @(posedge MCLK)
	begin
		if (~clk)
			master <= val;
		else
			slave <= master;
	end

endmodule


// Synchronous D flip-flop with async set
module ym_sdffs #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input set,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);

	reg [DATA_WIDTH-1:0] master, slave;

	assign q = slave;
	assign nq = ~slave;

	always @(posedge MCLK)
	begin
		if (~clk)
			master <= val;
		else if (~set)
			master <= {DATA_WIDTH{1'h1}};
		if (~set)
			slave <= {DATA_WIDTH{1'h1}};
		else if (clk)
			slave <= master;
	end

endmodule


// Synchronous D flip-flop with async reset
module ym_sdffr #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input reset,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);

	reg [DATA_WIDTH-1:0] master = {DATA_WIDTH{1'h0}}, slave = {DATA_WIDTH{1'h0}};

	assign q = slave;
	assign nq = ~slave;

	always @(posedge MCLK)
	begin
		if (~reset)
			master <= {DATA_WIDTH{1'h0}};
		else if (~clk)
			master <= val;
		if (~reset)
			slave <= {DATA_WIDTH{1'h0}};
		else if (clk)
			slave <= master;
	end

endmodule


// Synchronous D flip-flop with async set and reset
module ym_sdffsr #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input set,
	input reset,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);

	reg [DATA_WIDTH-1:0] master = {DATA_WIDTH{1'h0}}, slave = {DATA_WIDTH{1'h0}};

	assign q = (~set & ~reset) ? {DATA_WIDTH{1'h0}} : slave;
	assign nq = (~set & ~reset) ? {DATA_WIDTH{1'h0}} : ~slave;

	always @(posedge MCLK)
	begin
		if (~reset)
			master <= {DATA_WIDTH{1'h0}};
		else if (~set)
			master <= {DATA_WIDTH{1'h1}};
		else if (~clk)
			master <= val;
		if (~set)
			slave <= {DATA_WIDTH{1'h1}};
		else if (~reset)
			slave <= {DATA_WIDTH{1'h0}};
		else if (clk)
			slave <= master;
	end

endmodule


// Multi-cycle delay line (shift register)
module ym_delaychain #(parameter DELAY_CNT = 1)
	(
	input MCLK,
	input inp,
	output outp
	);

	reg [DELAY_CNT-1:0] delay_sr = {DELAY_CNT{1'h0}};

	always @(posedge MCLK)
	begin
		if (DELAY_CNT == 1)
			delay_sr <= inp;
		else
			delay_sr <= { delay_sr[DELAY_CNT-2:0], inp };
	end

	assign outp = delay_sr[DELAY_CNT-1];

endmodule

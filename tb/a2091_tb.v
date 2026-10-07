// a2091.v unit testbench (iverilog -g2012)
`timescale 1ns/1ps

module a2091_tb;

reg clk = 0;
always #17.6 clk = ~clk;      // ~28.4 MHz

reg         reset = 1;
reg         sel = 0;
reg  [15:1] addr = 0;
reg  [15:0] din = 0;
wire [15:0] dout;
reg         rd = 0, hwr = 0, lwr = 0;
wire        int2;
wire        enabled;

reg   [1:0] hp_cmd = 0;
reg         hp_start = 0, hp_strobe = 0;
reg  [15:0] hp_wdata = 0;
wire [15:0] hp_rdata;
wire [15:0] hp_status;

wire        dma_req, dma_we;
wire [23:1] dma_addr;
wire [15:0] dma_wdata;
reg  [15:0] dma_rdata = 0;
reg         dma_ack = 0;

a2091 dut
(
	.clk(clk), .reset(reset), .configured(1'b1), .z2ram_ena(1'b1),
	.sel(sel), .addr(addr), .din(din), .dout(dout), .rd(rd), .hwr(hwr), .lwr(lwr),
	.int2(int2), .enabled(enabled),
	.hp_cmd(hp_cmd), .hp_start(hp_start), .hp_strobe(hp_strobe), .hp_wdata(hp_wdata),
	.hp_rdata(hp_rdata), .hp_status(hp_status),
	.dma_req(dma_req), .dma_we(dma_we), .dma_addr(dma_addr), .dma_wdata(dma_wdata),
	.dma_rdata(dma_rdata), .dma_ack(dma_ack)
);

// ---------------------------------------------------------------- memory model
reg [15:0] mem [0:65535];     // word-addressed, covers $000000-$01FFFF
integer lat;
always @(posedge clk) begin
	dma_ack <= 0;
	if (dma_req && !dma_ack) begin
		lat = lat + 1;
		if (lat == 5) begin
			lat = 0;
			if (dma_we) mem[dma_addr[16:1]] <= dma_wdata;
			else        dma_rdata <= mem[dma_addr[16:1]];
			dma_ack <= 1;
		end
	end
end
initial lat = 0;

integer errors = 0;
task check(input [255:0] what, input [15:0] got, input [15:0] exp);
	if (got !== exp) begin
		$display("FAIL %0s: got %04x expected %04x", what, got, exp);
		errors = errors + 1;
	end
endtask

// ---------------------------------------------------------------- CPU bus
task cpu_rd(input [15:0] off, output [15:0] v);
	begin
		@(posedge clk); sel <= 1; addr <= off[15:1]; rd <= 1;
		repeat (6) @(posedge clk);
		v = dout;
		sel <= 0; rd <= 0;
		repeat (4) @(posedge clk);
	end
endtask

task cpu_wr(input [15:0] off, input [15:0] v, input u, input l);
	begin
		@(posedge clk); sel <= 1; addr <= off[15:1]; din <= v; hwr <= u; lwr <= l;
		repeat (6) @(posedge clk);
		sel <= 0; hwr <= 0; lwr <= 0;
		repeat (4) @(posedge clk);
	end
endtask

task cpu_wb(input [15:0] off, input [7:0] v);   // byte write (68k lane rules)
	if (off[0]) cpu_wr(off, {8'h00, v}, 0, 1);
	else        cpu_wr(off, {v, 8'h00}, 1, 0);
endtask

// ---------------------------------------------------------------- HPS mailbox
reg [15:0] hps_last;
task hps_begin(input [1:0] c, output [15:0] st);
	begin
		@(posedge clk); hp_cmd <= c; hp_start <= 1;
		@(posedge clk); hp_start <= 0; st = hp_status;
		repeat (8) @(posedge clk);
	end
endtask
task hps_word(input [15:0] w, output [15:0] r);
	begin
		@(posedge clk); hp_wdata <= w; hp_strobe <= 1; r = hp_rdata;
		@(posedge clk); hp_strobe <= 0;
		repeat (8) @(posedge clk);
	end
endtask

reg [15:0] v, st;
integer k;

task wd_write(input [4:0] r, input [7:0] val);
	begin cpu_wb(16'h0091, {3'b0, r}); cpu_wb(16'h0093, val); end
endtask
task wd_read(input [4:0] r, output [15:0] val);
	begin cpu_wb(16'h0091, {3'b0, r}); cpu_rd(16'h0092, val); end
endtask

initial begin
	$dumpfile("a2091_tb.vcd");
	$dumpvars(0, a2091_tb);
	for (k = 0; k < 65536; k = k + 1) mem[k] = k[15:0] ^ 16'h5a5a;

	repeat (5) @(posedge clk);
	reset <= 0;
	repeat (5) @(posedge clk);

	// ---- status marker + enable
	hps_begin(2'd0, st);
	check("status marker", st & 16'hE000, 16'hA000);
	check("ev_reset after reset", st & 16'h0008, 16'h0008);
	hps_begin(2'd1, st); hps_word(16'h0700, v); hps_word(16'h0001, v);
	hps_begin(2'd1, st); hps_word(16'h0100, v); hps_word(16'h0008, v);   // clr ev_reset
	hps_begin(2'd0, st);
	check("enabled", st & 16'h1008, 16'h1000);

	// ---- ROM load: 8 words, read back at $2000 (= ROM byte 0x2000)
	hps_begin(2'd1, st); hps_word(16'h0600, v);
	for (k = 0; k < 8192; k = k + 1) hps_word(k[15:0] ^ 16'h1234, v);
	cpu_rd(16'h2000, v); check("rom $2000", v, 16'h1000 ^ 16'h1234);
	cpu_rd(16'h2006, v); check("rom $2006", v, 16'h1003 ^ 16'h1234);
	cpu_rd(16'h4000, v); check("rom $4000 mirror", v, 16'h0000 ^ 16'h1234);

	// ---- autoconfig image
	cpu_rd(16'h0000, v); check("ac 00", v & 16'hF000, 16'hD000);
	cpu_rd(16'h0006, v); check("ac 06", v & 16'hF000, 16'hC000);
	cpu_rd(16'h0028, v); check("ac 28", v & 16'hF000, 16'hD000);

	// ---- WD regs: write CDB area with auto-increment, read back
	cpu_wb(16'h0091, 8'h03);
	for (k = 0; k < 6; k = k + 1) cpu_wb(16'h0093, 8'h10 + k);
	cpu_wb(16'h0091, 8'h03);
	for (k = 0; k < 6; k = k + 1) begin cpu_rd(16'h0092, v); check("cdb readback", v, 16'h0010 + k); end
	// OWN_ID masks bit 5
	wd_write(5'h00, 8'hff); wd_read(5'h00, v); check("own id mask", v, 16'h00df);
	// HPS sees the registers
	hps_begin(2'd0, st);
	hps_word(0, v); check("hps reg0", v, 16'h00df);
	hps_word(0, v); hps_word(0, v); hps_word(0, v); check("hps reg3", v, 16'h0010);

	// ---- command write -> ev_cmd + BSY
	wd_write(5'h18, 8'h08);
	cpu_rd(16'h0090, v); check("aux busy", v, 16'h0020);
	hps_begin(2'd0, st); check("ev_cmd", st & 16'h0041, 16'h0041);

	// ---- ARM: status FIFO with two entries
	hps_begin(2'd1, st); hps_word(16'h0100, v); hps_word(16'h0001, v);   // clr ev_cmd
	hps_begin(2'd1, st); hps_word(16'h0200, v); hps_word(16'h0016, v); hps_word(16'h0185, v);
	repeat (40) @(posedge clk);
	cpu_rd(16'h0090, v); check("aux int", v, 16'h0080);
	cpu_rd(16'h0040, v); check("istr ints", v & 16'h00ff, 16'h00c0);
	check("no int2 without INTEN", int2, 0);
	cpu_wb(16'h0043, 8'h10);              // CNTR.INTEN
	@(posedge clk); check("int2", int2, 1);
	cpu_rd(16'h0040, v); check("istr int_p", v & 16'h00ff, 16'h00d0);
	wd_read(5'h17, v); check("status 1", v, 16'h0016);
	@(posedge clk); check("int2 cleared", int2, 0);
	repeat (4000) @(posedge clk);         // > 1 tick
	cpu_rd(16'h0090, v); check("aux int 2", v, 16'h0080);
	wd_read(5'h17, v); check("status 2", v, 16'h0085);
	cpu_rd(16'h0090, v); check("aux idle", v, 16'h0000);

	// ---- FLUSH / ISTR read clears FE
	cpu_wr(16'h00e8, 0, 1, 1);
	cpu_rd(16'h0040, v); check("istr fe", v & 16'h0001, 16'h0001);
	cpu_rd(16'h0040, v); check("istr fe clr", v & 16'h0001, 16'h0000);

	// ---- ACR + ST_DMA
	cpu_wr(16'h0084, 16'h0001, 1, 1);
	cpu_wr(16'h0086, 16'h2345, 1, 1);
	hps_begin(2'd0, st);
	for (k = 0; k < 34; k = k + 1) hps_word(0, v);
	check("acr hi", v, 16'h0001);
	hps_word(0, v); check("acr lo", v, 16'h2344);
	cpu_wr(16'h00e0, 0, 1, 1);
	hps_begin(2'd0, st); check("ev_stdma+dmac_dma", st & 16'h0082, 16'h0082);

	// ---- DMA buffer -> memory (16 words @ $1000)
	hps_begin(2'd1, st); hps_word(16'h0400, v);
	for (k = 0; k < 16; k = k + 1) hps_word(16'hA500 + k, v);
	hps_begin(2'd1, st); hps_word(16'h0500, v); hps_word(16'h0000, v); hps_word(16'h1000, v); hps_word(16'h0010, v);
	repeat (2) @(posedge clk);
	k = 0;
	while (hp_status[4] && k < 10000) begin @(posedge clk); k = k + 1; end
	check("dma wr done", hp_status[4], 0);
	for (k = 0; k < 16; k = k + 1) check("mem after dma", mem[16'h0800 + k], 16'hA500 + k);
	check("mem untouched", mem[16'h0810], 16'h0810 ^ 16'h5a5a);

	// ---- DMA memory -> buffer (8 words @ $2000), read back over 'h72
	hps_begin(2'd1, st); hps_word(16'h0500, v); hps_word(16'h0000, v); hps_word(16'h2000, v); hps_word(16'h8008, v);
	repeat (2) @(posedge clk);
	k = 0;
	while (hp_status[4] && k < 10000) begin @(posedge clk); k = k + 1; end
	hps_begin(2'd2, st);
	for (k = 0; k < 8; k = k + 1) begin hps_word(0, v); check("bufrd", v, (16'h1000 + k) ^ 16'h5a5a); end

	// ---- DMA to an unreachable address is dropped
	hps_begin(2'd1, st); hps_word(16'h0500, v); hps_word(16'h00F8, v); hps_word(16'h0000, v); hps_word(16'h0004, v);
	repeat (200) @(posedge clk);
	check("dma ROM skip done", hp_status[4], 0);

	// ---- PIO data register handshake
	hps_begin(2'd1, st); hps_word(16'h0019, v); hps_word(16'h0042, v);    // reg 0x19 = 0x42
	hps_begin(2'd1, st); hps_word(16'h0100, v); hps_word(16'h0040, v);    // set DBR
	cpu_rd(16'h0090, v); check("aux dbr", v, 16'h0001);
	wd_read(5'h19, v); check("pio data", v, 16'h0042);
	cpu_rd(16'h0090, v); check("aux dbr clr", v, 16'h0000);
	hps_begin(2'd0, st); check("ev_pio", st & 16'h0004, 16'h0004);

	// ---- reset clears WD state, sets ev_reset, keeps enable
	reset <= 1; repeat (3) @(posedge clk); reset <= 0;
	hps_begin(2'd0, st); check("reset status", st & 16'h1008, 16'h1008);

	if (errors == 0) $display("PASS: all a2091 tests");
	else             $display("FAILED: %0d errors", errors);
	$finish;
end

endmodule

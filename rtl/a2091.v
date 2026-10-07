// SPDX-License-Identifier: GPL-3.0-or-later
//
// Commodore A2091 (DMAC rev 2 + WD33C93A) Zorro II SCSI host adapter
// for the Minimig-AGA MiSTer core.
//
// Split between FPGA and HPS (ARM), modelled on amiberry/WinUAE a2091.cpp:
//
//   FPGA (this module)
//     - 64 KB Zorro II board window (autoconfig itself lives in cpu_wrapper)
//     - boot ROM (16 KB, loaded by the ARM at core start)
//     - DMAC registers: CNTR, ISTR, WTC, ACR, ST_DMA/SP_DMA/CINT/FLUSH, DIP
//     - WD33C93A register file (SASR/SCMD indirect access) with the bus-visible
//       side effects done locally so the 68k never waits on the ARM:
//         * SASR auto-increment
//         * AUX STATUS (INT/BSY/DBR)
//         * reading SCSI STATUS clears INT
//         * a status FIFO the ARM fills; entries are presented one at a time,
//           the next one only after the 68k has read SCSI STATUS
//     - a word DMA engine with a 4 KB buffer, driven by the ARM, mastering
//       Amiga memory through chipdma_arb (chip / slow / Zorro II RAM)
//     - INT2
//
//   ARM (support/minimig/minimig_a2091.cpp)
//     - WD33C93 command engine (select-and-transfer, transfer info, ...)
//     - SCSI-2 direct-access targets backed by .hdf files
//
// HPS mailbox (hps_ext commands 'h70..'h72). Word 0 of every transfer returns
// the status word. Following words:
//   'h70 READ  : item k = see hp_rdata mux below (WD regs, DMAC state, ...)
//   'h71 WRITE : word 1 = opcode, words 2.. = operands (see OP_*)
//   'h72 BUFRD : item k = dma buffer word k
//
// This module is clocked by clk_sys (28 MHz), the same clock as the chip bus,
// hps_ext and chipdma_arb.

module a2091
#(
	parameter HAS_A4091 = 0          // core also carries the A4091 bridge
)
(
	input             clk,
	input             reset,          // Amiga reset (CPU RESET / system)

	// board configuration from cpu_wrapper autoconfig
	input             configured,
	input             z2ram_ena,      // Zorro II fast RAM present at $200000

	// chip bus
	input             sel,            // board window selected (base matched)
	input      [15:1] addr,
	input      [15:0] din,
	output     [15:0] dout,
	input             rd,
	input             hwr,
	input             lwr,

	output            int2,

	// board enable: on by default so the board autoconfigs without any HPS
	// support; the ARM can switch it off. Survives Amiga resets.
	output reg        enabled,
	// a boot ROM has been loaded: autoconfig advertises the DiagArea
	output reg        rom_valid,
	// A4091 enable, also set through OP_CFG (bit 1) in a combined core
	output reg        a4091_en,

	// HPS mailbox (hps_ext)
	input       [1:0] hp_cmd,         // command - 'h70
	input             hp_start,       // pulse: command word received
	input             hp_strobe,      // pulse: data word received
	input      [15:0] hp_wdata,
	output reg [15:0] hp_rdata,     // combinational: valid 1 clk after a strobe
	output     [15:0] hp_status,

	// DMA master port (chipdma_arb)
	output reg        dma_req,
	output reg        dma_we,
	output reg [23:1] dma_addr,
	output reg [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	input             dma_ack
);

// ---------------------------------------------------------------------------
// DMAC register bits (DMAC rev 2, amiberry COMMODORE_DMAC)
localparam CNTR_TCEN  = 7;
localparam CNTR_PREST = 6;
localparam CNTR_PDMD  = 5;
localparam CNTR_INTEN = 4;
localparam CNTR_DDIR  = 3;

localparam ISTR_INT_F  = 7;
localparam ISTR_INTS   = 6;
localparam ISTR_E_INT  = 5;
localparam ISTR_INT_P  = 4;
localparam ISTR_FE_FLG = 0;

// WD33C93 registers
localparam WD_OWN_ID      = 5'h00;
localparam WD_SCSI_STATUS = 5'h17;
localparam WD_COMMAND     = 5'h18;
localparam WD_DATA        = 5'h19;
localparam WD_QUEUE_TAG   = 5'h1a;
localparam WD_AUX_STATUS  = 5'h1f;

// HPS write opcodes ('h71 word 1)
localparam OP_WDREGS = 8'h00;   // low byte = first register; words 2.. = values
localparam OP_CTRL   = 8'h01;   // word 2 = control bits (CTL_*)
localparam OP_STATUS = 8'h02;   // words 2.. = {delay, status} pushed to the FIFO
localparam OP_ACR    = 8'h03;   // word 2 = ACR[23:16], word 3 = ACR[15:0]
localparam OP_BUFWR  = 8'h04;   // words 2.. = dma buffer words from 0
localparam OP_DMA    = 8'h05;   // word 2 = addr[23:16], word 3 = addr[15:0],
                                // word 4 = {dir(1=read memory), 3'b0, count[11:0]} -> start
localparam OP_ROM    = 8'h06;   // words 2.. = boot ROM words from 0
localparam OP_CFG    = 8'h07;   // word 2 = bit0 A2091 enable, bit1 A4091 enable
localparam OP_PIO    = 8'h08;   // word 2 = {dir(1 = 68k writes), 3'b0, count[11:0]}:
                                // the 68k moves count bytes through the WD DATA
                                // register straight from/into the DMA buffer
                                // (byte k = buffer word k/2, even byte high);
                                // ev_pio when the last byte has moved

// control bits (OP_CTRL)
localparam CTL_CLR_EV_CMD   = 0;
localparam CTL_CLR_EV_STDMA = 1;
localparam CTL_CLR_EV_PIO   = 2;
localparam CTL_CLR_EV_RESET = 3;
localparam CTL_SET_BUSY     = 4;
localparam CTL_CLR_BUSY     = 5;
localparam CTL_SET_DBR      = 6;
localparam CTL_CLR_DBR      = 7;
localparam CTL_SET_BSYX     = 8;
localparam CTL_CLR_BSYX     = 9;
localparam CTL_WD_RESET     = 10;   // flush status FIFO, clear INT
localparam CTL_DMA_DONE     = 11;   // dmac_dma <= 0 (amiberry set_dma_done)

// ---------------------------------------------------------------------------
// bus cycle tracking

wire        acc     = sel & (rd | hwr | lwr);
reg         acc_d;
reg  [15:1] acc_addr;
reg         acc_rd, acc_hwr, acc_lwr;
reg  [15:0] acc_din;

wire        acc_end = acc_d & ~acc;

always @(posedge clk) begin
	acc_d <= acc;
	if (acc) begin
		if (~acc_d) begin
			acc_addr <= addr;
			acc_rd   <= rd;
			acc_hwr  <= hwr;
			acc_lwr  <= lwr;
		end
		else begin
			acc_rd   <= acc_rd  | rd;
			acc_hwr  <= acc_hwr | hwr;
			acc_lwr  <= acc_lwr | lwr;
		end
		if (hwr) acc_din[15:8] <= din[15:8];
		if (lwr) acc_din[7:0]  <= din[7:0];
	end
end

// byte value of a register write, emulating amiberry's word/byte lane rules:
// the register sees the odd (low) byte; an even-byte-only write lands as 0.
wire  [7:0] wr_byte = acc_lwr ? acc_din[7:0] : 8'h00;
wire [15:0] wr_word = {acc_hwr ? acc_din[15:8] : 8'h00, acc_lwr ? acc_din[7:0] : 8'h00};

// register offsets (address bits 7:1 of the 64 KB window)
wire [15:1] a = acc ? addr : acc_addr;
wire        in_regs = ~|a[15:8];               // 0x0000-0x00ff
wire        in_rom  = |a[15:13];               // 0x2000-0xffff

wire r_ac    = in_regs & (a[7:6] == 2'b00);     // 0x00-0x3f autoconfig image
wire r_istr  = in_regs & (a[7:1] == 7'h20);     // 0x40
wire r_cntr  = in_regs & (a[7:1] == 7'h21);     // 0x42
wire r_wtch  = in_regs & (a[7:1] == 7'h40);     // 0x80
wire r_wtcl  = in_regs & (a[7:1] == 7'h41);     // 0x82
wire r_acrh  = in_regs & (a[7:1] == 7'h42);     // 0x84
wire r_acrl  = in_regs & (a[7:1] == 7'h43);     // 0x86
wire r_sasr  = in_regs & (a[7:1] == 7'h48);     // 0x90/0x91
wire r_scmd  = in_regs & (a[7:1] == 7'h49);     // 0x92/0x93
wire r_xt    = in_regs & (a[7:3] == 5'h14);     // 0xa0-0xa7 (A590 XT, absent)
wire r_dip   = in_regs & (a[7:1] == 7'h60);     // 0xc0
wire r_ffff  = in_regs & (a[7:3] == 5'h18) & |a[2:1]; // 0xc2-0xc6
wire r_stdma = in_regs & (a[7:1] == 7'h70);     // 0xe0
wire r_spdma = in_regs & (a[7:1] == 7'h71);     // 0xe2
wire r_cint  = in_regs & (a[7:1] == 7'h72);     // 0xe4
wire r_flush = in_regs & (a[7:1] == 7'h74);     // 0xe8

// ---------------------------------------------------------------------------
// autoconfig image (same values cpu_wrapper presents at $E80000):
// er_Type $D1 (Zorro II, DiagArea valid, 64 KB), product 3 (A2091),
// manufacturer $0202 (Commodore West Chester), DiagArea vector $2000.

function [3:0] ac_nib(input [5:0] idx);
	case (idx)
		6'h00: ac_nib = rom_valid ? 4'hD : 4'hC;
		6'h01: ac_nib = 4'h1;
		6'h02: ac_nib = 4'hF;   // ~product[7:4]
		6'h03: ac_nib = 4'hC;   // ~product[3:0]  -> 3
		6'h08: ac_nib = 4'hF;   // ~$02 hi
		6'h09: ac_nib = 4'hD;
		6'h0a: ac_nib = 4'hF;   // ~$02 lo
		6'h0b: ac_nib = 4'hD;
		6'h14: ac_nib = 4'hD;   // ~$20  er_InitDiagVec hi
		6'h15: ac_nib = 4'hF;
		default: ac_nib = 4'hF;
	endcase
endfunction

// ---------------------------------------------------------------------------
// boot ROM: 8K x 16, big-endian byte pairs, mirrored over 0x2000-0xffff

reg  [15:0] rom [0:8191];
reg  [15:0] rom_q;
reg  [12:0] rom_waddr;
reg  [15:0] rom_wdata;
reg         rom_we;

always @(posedge clk) begin
	if (rom_we) rom[rom_waddr] <= rom_wdata;
	rom_q <= rom[a[13:1]];
end

// ---------------------------------------------------------------------------
// DMA buffer: 2K x 16. Port A = HPS (write / read), port B = DMA engine.

reg  [10:0] buf_haddr;
reg  [15:0] buf_hwdata;
reg         buf_hwe;
wire [15:0] buf_hq;

reg  [10:0] buf_daddr;
reg  [15:0] buf_dwdata;
reg         buf_dwe;
wire [15:0] buf_dq;

a2091_tdpram #(.AW(11), .DW(16)) dbuf
(
	.clk    (clk),
	.we_a   (buf_hwe),
	.addr_a (buf_haddr),
	.d_a    (buf_hwdata),
	.q_a    (buf_hq),
	.we_b   (buf_dwe),
	.addr_b (buf_daddr),
	.d_b    (buf_dwdata),
	.q_b    (buf_dq)
);

// ---------------------------------------------------------------------------
// state

reg   [7:0] cntr;
reg         istr_ints;
reg         istr_fe;
reg  [31:0] wtc;
reg  [23:1] acr;
reg  [15:0] dawr;
reg         dmac_dma;

reg   [7:0] wdregs [0:31];
reg   [4:0] sasr;
reg         wd_int;
reg         wd_busy;
reg         wd_dbr;
reg         wd_bsyx;

reg         ev_cmd, ev_stdma, ev_pio, ev_reset;

// PIO streaming through the DMA buffer
reg         pio_on;
reg         pio_wr;
reg  [11:0] pio_cnt;
reg  [11:0] pio_ptr;
reg   [7:0] pio_hi;
wire  [7:0] pio_rbyte = pio_ptr[0] ? buf_dq[7:0] : buf_dq[15:8];

// status FIFO
reg  [15:0] sfifo [0:3];
reg   [1:0] sf_rd, sf_wr;
reg   [2:0] sf_cnt;
reg  [10:0] tick_div;
reg   [7:0] sf_ticks;

wire        int_pending = istr_ints;
assign int2 = enabled & configured & cntr[CNTR_INTEN] & int_pending;

wire  [7:0] auxstatus = {wd_int, 1'b0, wd_busy | wd_bsyx, 4'b0000, wd_dbr};

// read value of the register selected by SASR (no side effects)
wire        wd_invalid = (sasr > WD_QUEUE_TAG && sasr < WD_AUX_STATUS) || (sasr == WD_QUEUE_TAG);
wire  [7:0] scmd_val = (sasr == WD_AUX_STATUS) ? auxstatus :
                       wd_invalid              ? 8'hff :
                       (sasr == WD_DATA && pio_on && ~pio_wr) ? pio_rbyte :
                       (sasr == WD_DATA && ~wd_dbr) ? 8'h00 :
                                                 wdregs[sasr];

wire  [7:0] istr_val = {wd_int, istr_ints, 1'b0, cntr[CNTR_INTEN] & istr_ints, 3'b000, istr_fe};

// ---------------------------------------------------------------------------
// CPU read mux

reg  [15:0] rdata;
always @(*) begin
	rdata = 16'h0000;
	if (in_rom)        rdata = rom_q;
	else if (r_ac)     rdata = {ac_nib(a[6:1]), (a[6:1] < 6'h02) ? 4'h0 : 4'hF, 8'hFF};
	else if (r_istr)   rdata = {8'h00, istr_val};
	else if (r_cntr)   rdata = {8'h00, cntr};
	else if (r_sasr)   rdata = {8'h00, auxstatus};
	else if (r_scmd)   rdata = {8'h00, scmd_val};
	else if (r_xt)     rdata = 16'h00FF;
	else if (r_dip)    rdata = 16'h00FF;
	else if (r_ffff)   rdata = 16'hFFFF;
end

assign dout = (sel & rd & enabled) ? rdata : 16'h0000;

// ---------------------------------------------------------------------------
// HPS mailbox

reg   [7:0] hp_op;
reg   [7:0] hp_idx;       // data words seen in this transfer
reg   [4:0] hp_reg;
reg  [23:0] hp_dma_addr;

// DMA engine
reg         dma_busy;
reg         dma_dir;      // 1 = memory -> buffer
reg  [11:0] dma_left;
reg   [2:0] dma_st;
// DMA targets: chip RAM $000000-$1FFFFF, Zorro II RAM $200000-$9FFFFF,
// slow RAM $C00000-$D7FFFF. Anything else (ROM, I/O) is never touched.
wire        dma_addr_ok = (dma_addr[23:21] == 3'b000) ||
                          (z2ram_ena && (dma_addr[23] ^ |dma_addr[22:21])) ||
                          (dma_addr[23:19] == 5'b11000) || (dma_addr[23:19] == 5'b11001) ||
                          (dma_addr[23:19] == 5'b11010);
localparam DS_IDLE = 3'd0, DS_FETCH = 3'd1, DS_FETCH2 = 3'd2, DS_REQ = 3'd3, DS_NEXT = 3'd4;

assign hp_status = {3'b101, enabled, wd_bsyx, wd_dbr, sf_cnt[1:0] | {2{sf_cnt[2]}},
                    dmac_dma, wd_busy, wd_int, dma_busy, ev_reset, ev_pio, ev_stdma, ev_cmd};

// read stream ('h70): item index = number of data strobes so far
reg [7:0] hp_ridx;
always @(*) begin
	case (hp_cmd)
		2'd0: begin
			if (hp_ridx < 8'd32)       hp_rdata = {8'h00, wdregs[hp_ridx[4:0]]};
			else case (hp_ridx)
				8'd32: hp_rdata = {3'b000, sasr, cntr};
				8'd33: hp_rdata = {8'h00, acr[23:16]};
				8'd34: hp_rdata = {acr[15:1], 1'b0};
				8'd35: hp_rdata = wtc[31:16];
				8'd36: hp_rdata = wtc[15:0];
				8'd37: hp_rdata = {8'h00, istr_val};
				8'd38: hp_rdata = {pio_on, 2'd0, pio_ptr, sf_cnt[0]};
				8'd39: hp_rdata = {15'd0, HAS_A4091 != 0};     // capabilities
				default: hp_rdata = 16'h0000;
			endcase
		end
		2'd2: hp_rdata = buf_hq;
		default: hp_rdata = 16'h0000;
	endcase
end

// ---------------------------------------------------------------------------
// main sequential block

integer i;

always @(posedge clk) begin
	rom_we  <= 1'b0;
	buf_hwe <= 1'b0;
	buf_dwe <= 1'b0;

	// --------------------------------------------------------------- HPS
	if (hp_start) begin
		hp_idx    <= 8'd0;
		hp_ridx   <= 8'd0;
		buf_haddr <= 11'd0;
	end

	if (hp_strobe) begin
		if (~&hp_idx)  hp_idx  <= hp_idx + 1'd1;
		if (~&hp_ridx) hp_ridx <= hp_ridx + 1'd1;

		if (hp_cmd == 2'd2) buf_haddr <= buf_haddr + 1'd1;

		if (hp_cmd == 2'd1) begin
			if (hp_idx == 8'd0) begin
				hp_op     <= hp_wdata[15:8];
				hp_reg    <= hp_wdata[4:0];
				buf_haddr <= 11'd0;
				rom_waddr <= 13'h1fff;     // pre-decrement: first word -> 0
			end
			else case (hp_op)
				OP_WDREGS: begin
					wdregs[hp_reg] <= hp_wdata[7:0];
					hp_reg <= hp_reg + 1'd1;
				end
				OP_CTRL: if (hp_idx == 8'd1) begin
					if (hp_wdata[CTL_CLR_EV_CMD])   ev_cmd   <= 1'b0;
					if (hp_wdata[CTL_CLR_EV_STDMA]) ev_stdma <= 1'b0;
					if (hp_wdata[CTL_CLR_EV_PIO])   ev_pio   <= 1'b0;
					if (hp_wdata[CTL_CLR_EV_RESET]) ev_reset <= 1'b0;
					if (hp_wdata[CTL_SET_BUSY])     wd_busy  <= 1'b1;
					if (hp_wdata[CTL_CLR_BUSY])     wd_busy  <= 1'b0;
					if (hp_wdata[CTL_SET_DBR])      wd_dbr   <= 1'b1;
					if (hp_wdata[CTL_CLR_DBR])      wd_dbr   <= 1'b0;
					if (hp_wdata[CTL_SET_BSYX])     wd_bsyx  <= 1'b1;
					if (hp_wdata[CTL_CLR_BSYX])     wd_bsyx  <= 1'b0;
					if (hp_wdata[CTL_CLR_DBR])      pio_on   <= 1'b0;
					if (hp_wdata[CTL_WD_RESET]) begin
						sf_rd <= 2'd0; sf_wr <= 2'd0; sf_cnt <= 3'd0;
						wd_int <= 1'b0;
					end
					if (hp_wdata[CTL_DMA_DONE])     dmac_dma <= 1'b0;
				end
				OP_STATUS: if (sf_cnt != 3'd4) begin
					sfifo[sf_wr] <= hp_wdata;
					sf_wr  <= sf_wr + 1'd1;
				end
				OP_ACR: begin
					if (hp_idx == 8'd1) acr[23:16] <= hp_wdata[7:0];
					if (hp_idx == 8'd2) acr[15:1]  <= hp_wdata[15:1];
				end
				OP_BUFWR: begin
					buf_hwdata <= hp_wdata;
					buf_hwe    <= 1'b1;
				end
				OP_DMA: begin
					if (hp_idx == 8'd1) hp_dma_addr[23:16] <= hp_wdata[7:0];
					if (hp_idx == 8'd2) hp_dma_addr[15:0]  <= hp_wdata;
				end
				OP_ROM: begin
					rom_waddr <= rom_waddr + 1'd1;
					rom_wdata <= hp_wdata;
					rom_we    <= 1'b1;
					rom_valid <= 1'b1;
				end
				OP_CFG: if (hp_idx == 8'd1) begin
					enabled  <= hp_wdata[0];
					a4091_en <= hp_wdata[1] & (HAS_A4091 != 0);
				end
				OP_PIO: if (hp_idx == 8'd1 && |hp_wdata[11:0]) begin
					pio_on    <= 1'b1;
					pio_wr    <= hp_wdata[15];
					pio_cnt   <= hp_wdata[11:0];
					pio_ptr   <= 12'd0;
					buf_daddr <= 11'd0;
					wd_dbr    <= 1'b1;
				end
				default: ;
			endcase
		end
	end

	// buffer write pointer advances after each written word
	if (buf_hwe) buf_haddr <= buf_haddr + 1'd1;

	// --------------------------------------------------------------- status FIFO
	if (OP_STATUS == hp_op && hp_cmd == 2'd1 && hp_strobe && hp_idx != 8'd0 && sf_cnt != 3'd4)
		sf_cnt <= sf_cnt + 1'd1;

	tick_div <= tick_div + 1'd1;
	if (wd_int || sf_cnt == 3'd0) sf_ticks <= 8'd0;
	else if (&tick_div && ~&sf_ticks) sf_ticks <= sf_ticks + 1'd1;

	// deliver the FIFO head when INT is clear, the bus is idle and its
	// delay (in ~64 us units, amiberry hsync ticks) has elapsed
	if (~wd_int && sf_cnt != 3'd0 && ~acc && ~acc_d && sf_ticks >= sfifo[sf_rd][15:8] &&
	    ~(hp_strobe && hp_cmd == 2'd1 && hp_op == OP_CTRL)) begin
		wdregs[WD_SCSI_STATUS] <= sfifo[sf_rd][7:0];
		wd_int   <= 1'b1;
		wd_busy  <= 1'b0;
		sf_rd    <= sf_rd + 1'd1;
		sf_cnt   <= sf_cnt - 1'd1 + ((OP_STATUS == hp_op && hp_cmd == 2'd1 && hp_strobe && hp_idx != 8'd0 && sf_cnt != 3'd4) ? 3'd1 : 3'd0);
		sf_ticks <= 8'd0;
	end

	// --------------------------------------------------------------- DMAC ISTR
	if (wd_int) istr_ints <= 1'b1;

	// --------------------------------------------------------------- CPU bus
	if (acc_end & enabled) begin
		// strobes: any access, read or write
		if (r_stdma && ~dmac_dma) begin
			dmac_dma <= 1'b1;
			ev_stdma <= 1'b1;
		end
		if (r_spdma) dmac_dma <= 1'b0;
		if (r_cint)  begin istr_ints <= wd_int; istr_fe <= 1'b0; end
		if (r_flush) istr_fe <= 1'b1;

		if (acc_rd) begin
			if (r_istr) istr_fe <= 1'b0;
			if (r_scmd) begin
				if (sasr == WD_SCSI_STATUS) begin
					wd_int    <= 1'b0;
					istr_ints <= 1'b0;
				end
				if (sasr == WD_DATA && pio_on && ~pio_wr) begin
					pio_ptr   <= pio_ptr + 1'd1;
					buf_daddr <= pio_ptr[11:1] + pio_ptr[0];
					if (pio_ptr + 1'd1 == pio_cnt) begin
						pio_on <= 1'b0;
						wd_dbr <= 1'b0;
						ev_pio <= 1'b1;
					end
				end
				else if (sasr == WD_DATA && wd_dbr) begin
					wd_dbr <= 1'b0;
					ev_pio <= 1'b1;
				end
				if (sasr != WD_AUX_STATUS && sasr != WD_DATA && sasr != WD_COMMAND)
					sasr <= sasr + 1'd1;
			end
		end

		if (acc_hwr | acc_lwr) begin
			if (r_cntr) cntr <= wr_byte;
			if (r_wtch) wtc[31:16] <= wr_word;
			if (r_wtcl) wtc[15:0]  <= wr_word;
			if (r_acrh) acr[23:16] <= wr_byte;
			if (r_acrl) acr[15:1]  <= {acc_hwr ? acc_din[15:8] : 8'h00, acc_lwr ? acc_din[7:1] : 7'h00};
			if (in_regs && a[7:1] == 7'h47) dawr <= wr_word;
			if (r_sasr) sasr <= wr_byte[4:0];
			if (r_scmd) begin
				if (sasr != WD_SCSI_STATUS && sasr != WD_AUX_STATUS && ~wd_invalid)
					wdregs[sasr] <= (sasr == WD_OWN_ID) ? (wr_byte & 8'hdf) : wr_byte;
				if (sasr == WD_COMMAND) begin
					wd_busy <= 1'b1;
					ev_cmd  <= 1'b1;
				end
				if (sasr == WD_DATA && pio_on && pio_wr) begin
					pio_ptr <= pio_ptr + 1'd1;
					if (~pio_ptr[0]) pio_hi <= wr_byte;
					if (pio_ptr[0] || pio_ptr + 1'd1 == pio_cnt) begin
						buf_daddr  <= pio_ptr[11:1];
						buf_dwdata <= pio_ptr[0] ? {pio_hi, wr_byte} : {wr_byte, 8'h00};
						buf_dwe    <= 1'b1;
					end
					if (pio_ptr + 1'd1 == pio_cnt) begin
						pio_on <= 1'b0;
						wd_dbr <= 1'b0;
						ev_pio <= 1'b1;
					end
				end
				else if (sasr == WD_DATA && wd_dbr) begin
					wd_dbr <= 1'b0;
					ev_pio <= 1'b1;
				end
				if (sasr != WD_AUX_STATUS && sasr != WD_DATA && sasr != WD_COMMAND && sasr != WD_SCSI_STATUS)
					sasr <= sasr + 1'd1;
			end
		end
	end

	// --------------------------------------------------------------- DMA engine
	case (dma_st)
		DS_IDLE: if (hp_strobe && hp_cmd == 2'd1 && hp_op == OP_DMA && hp_idx == 8'd3) begin
			dma_addr  <= hp_dma_addr[23:1];
			dma_dir   <= hp_wdata[15];
			dma_left  <= hp_wdata[11:0];
			buf_daddr <= 11'd0;
			dma_busy  <= |hp_wdata[11:0];
			dma_st    <= |hp_wdata[11:0] ? DS_FETCH : DS_IDLE;
		end
		DS_FETCH:  dma_st <= DS_FETCH2;           // buffer read latency
		DS_FETCH2: begin
			dma_wdata <= buf_dq;
			dma_we    <= ~dma_dir;
			if (dma_addr_ok) begin
				dma_req <= 1'b1;
				dma_st  <= DS_REQ;
			end
			else begin
				// not RAM the DMAC could reach: drop writes, reads float high
				if (dma_dir) begin
					buf_dwdata <= 16'hFFFF;
					buf_dwe    <= 1'b1;
				end
				dma_st <= DS_NEXT;
			end
		end
		DS_REQ: if (dma_ack) begin
			dma_req <= 1'b0;
			if (dma_dir) begin
				buf_dwdata <= dma_rdata;
				buf_dwe    <= 1'b1;
			end
			dma_st <= DS_NEXT;
		end
		DS_NEXT: begin
			dma_addr  <= dma_addr + 1'd1;
			buf_daddr <= buf_daddr + 1'd1;
			dma_left  <= dma_left - 1'd1;
			if (dma_left == 12'd1) begin
				dma_busy <= 1'b0;
				dma_st   <= DS_IDLE;
			end
			else dma_st <= DS_FETCH;
		end
		default: dma_st <= DS_IDLE;
	endcase

	// --------------------------------------------------------------- resets
	if (reset | (acc_end & (acc_hwr | acc_lwr) & r_cntr & wr_byte[CNTR_PREST])) begin
		// DMAC reset (bus reset or CNTR.PREST)
		cntr      <= 8'h00;
		istr_ints <= 1'b0;
		istr_fe   <= 1'b0;
		dmac_dma  <= 1'b0;
	end

	if (reset) begin
		// WD33C93 hardware reset (amiberry wd_cmd_reset without IRQ)
		for (i = 1; i <= 8'h16; i = i + 1) wdregs[i] <= 8'h00;
		wdregs[WD_COMMAND] <= 8'h00;
		sasr     <= 5'd0;
		wd_int   <= 1'b0;
		wd_busy  <= 1'b0;
		wd_dbr   <= 1'b0;
		wd_bsyx  <= 1'b0;
		sf_rd    <= 2'd0;
		sf_wr    <= 2'd0;
		sf_cnt   <= 3'd0;
		ev_cmd   <= 1'b0;
		ev_stdma <= 1'b0;
		ev_pio   <= 1'b0;
		ev_reset <= 1'b1;
		pio_on   <= 1'b0;
		acr      <= 23'd0;
		wtc      <= 32'd0;
	end
end

initial begin
	enabled   = 1'b1;
	rom_valid = 1'b0;
	a4091_en  = 1'b0;
	dma_st    = DS_IDLE;
	dma_req   = 1'b0;
	dma_we    = 1'b0;
	dma_busy  = 1'b0;
	hp_op     = 8'hff;
	hp_idx    = 8'd0;
	hp_ridx   = 8'd0;
	sf_rd     = 2'd0;
	sf_wr     = 2'd0;
	sf_cnt    = 3'd0;
	sf_ticks  = 8'd0;
	tick_div  = 11'd0;
	wd_int    = 1'b0;
	wd_busy   = 1'b0;
	wd_dbr    = 1'b0;
	wd_bsyx   = 1'b0;
	ev_cmd    = 1'b0;
	ev_stdma  = 1'b0;
	ev_pio    = 1'b0;
	ev_reset  = 1'b0;
	pio_on    = 1'b0;
	pio_wr    = 1'b0;
	pio_cnt   = 12'd0;
	pio_ptr   = 12'd0;
	cntr      = 8'h00;
	istr_ints = 1'b0;
	istr_fe   = 1'b0;
	dmac_dma  = 1'b0;
	acr       = 23'd0;
	wtc       = 32'd0;
	sasr      = 5'd0;
	acc_d     = 1'b0;
end

endmodule


// True dual-port RAM in the Quartus inference template (one clock).
module a2091_tdpram #(parameter AW = 11, parameter DW = 16)
(
	input               clk,
	input               we_a,
	input      [AW-1:0] addr_a,
	input      [DW-1:0] d_a,
	output reg [DW-1:0] q_a,
	input               we_b,
	input      [AW-1:0] addr_b,
	input      [DW-1:0] d_b,
	output reg [DW-1:0] q_b
);

reg [DW-1:0] ram [0:(1<<AW)-1];

always @(posedge clk) begin
	if (we_a) begin
		ram[addr_a] <= d_a;
		q_a <= d_a;
	end
	else q_a <= ram[addr_a];
end

always @(posedge clk) begin
	if (we_b) begin
		ram[addr_b] <= d_b;
		q_b <= d_b;
	end
	else q_b <= ram[addr_b];
end

endmodule

// ===========================================================================
// A4091 bridge - thin Zorro-III <-> HPS ARM bridge for the software SIOP.
//
// Replaces a4091_siop.v + a4091_target.v + a4091_sd.v. The 53C710 + SCRIPTS
// VM + SCSI-2 emulation now run on the ARM (support/minimig/a4091_lsi.cpp +
// a4091_scsi.c). This module is just:
//   * a 256-byte shadow register RAM  - CPU R/W at Zorro speed, zero SPI
//   * kick detect                     - CPU writes DSP MSB -> tell the ARM
//   * IRQ                             - ARM sets int_pending -> assert int2
//   * chip / Zorro-II DMA mailbox     - the rare non-Z3-fast DATA path
//
// The bulk DATA path is the ARM memcpy'ing straight into HPS DDR (Z3 fast RAM
// is physically DDR) - it never touches this module.
//
// See A4091/software-siop-exec-plan.md (Phase A).
// ===========================================================================

module a4091_bridge
(
	input             clk,            // clk_sys 28.6875 MHz
	input             reset,          // active high

	// ---- CPU register port (from a4091.v board-window decode) -------------
	// Dual-lane: hi -> D15:8 at reg_hi_addr, lo -> D7:0 at reg_lo_addr.
	// Strobes are LEVELS held for the whole (multi-cycle) Zorro access; the
	// "fire once per address phase" edge logic below is ported verbatim from
	// a4091_siop.v (self-clearing bits double-count on a held strobe, and a
	// 32-bit burst hits a different register pair each half).
	input      [7:0]  reg_hi_addr,
	input      [7:0]  reg_lo_addr,
	input             reg_rd_hi,
	input             reg_rd_lo,
	input             reg_wr_hi,
	input             reg_wr_lo,
	input      [7:0]  reg_wd_hi,
	input      [7:0]  reg_wd_lo,
	output reg [7:0]  reg_rq_hi,
	output reg [7:0]  reg_rq_lo,
	output            reg_ready,

	// ---- interrupt ------------------------------------------------------
	output            int2,

	// ---- HPS mailbox (hps_ext.v) --------------------------------------
	// 0x64 poll        : {kick_pending, soft_reset, dma_req, ...}   (FPGA->HPS)
	// 0x65 regs read   : burst-read shadow RAM                      (FPGA->HPS)
	// 0x66 regs write  : burst-write shadow RAM                     (HPS->FPGA)
	// 0x67 control     : set int_pending / clear kick / clear soft_reset
	// 0x68/0x69 chip-DMA : addr/len/we  +  data  (Phase A.2 / PR2)
	// mbx_addr drives a registered read: mbx_rdata = regs[mbx_addr] one clk
	// later, unconditionally. 0x66 burst-write pulses mbx_regs_wr.
	input      [7:0]  mbx_addr,
	input             mbx_regs_wr,      // 0x66: HPS writes mbx_wdata -> regs[mbx_addr]
	input      [7:0]  mbx_wdata,
	output reg [7:0]  mbx_rdata,
	input             mbx_set_int,      // 0x67 bit: raise int_pending
	input             mbx_clr_int,      // 0x67 bit: lower int_pending
	input             mbx_clr_kick,     // 0x67 bit: clear kick_pending
	input             mbx_clr_srst,     // 0x67 bit: clear soft_reset_pending
	input             mbx_cache_clr,    // 0x67 bit4: flush the RAM-ctrl read cache

	output     [15:0] mbx_status,       // 0x64 poll word

	// After a DATA-IN memcpy the ARM writes HPS DDR behind the FPGA's back, so
	// the Minimig RAM-controller read cache (cpu_cache_new) holds stale lines.
	// Pulse this into cpu_cacr[3] (the CPU's own cache-clear bit) for both RAM
	// controllers - the sanctioned runtime flush path - instead of the driver's
	// multi-KB scratch-arena eviction sweep.
	output            cache_clr,

	// ---- chip / Z2 DMA slot (drives sdram_ctrl's existing a4091 dma port) --
	// Kept as ports for wiring symmetry; the mailbox that fills them lands in
	// PR2 (exec-plan Phase A.2). Tie off until then.
	output            dma_req,
	output            dma_rw,
	output     [31:1] dma_addr,
	output     [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	output      [1:0] dma_bs,
	input             dma_ack,

	// ---- compact debug bus (hps_ext 0x68 heartbeat) ---------------------
	output     [63:0] dbg_bus,
	output            led
);

// ---------------------------------------------------------------------------
// shadow register RAM  (256 x 8, dual-port: CPU + HPS mailbox)
// ---------------------------------------------------------------------------
reg [7:0] regs [0:255];

integer i;
initial for (i = 0; i < 256; i = i + 1) regs[i] = 8'h00;

// ---------------------------------------------------------------------------
// CPU port: "fire exactly once per address phase" edge detect
// (verbatim from a4091_siop.v - trigger = strobe rising edge OR this lane's
//  decoded address changed)
// ---------------------------------------------------------------------------
reg  reg_rd_hi_d, reg_rd_lo_d, reg_wr_hi_d, reg_wr_lo_d;
reg  [7:0] reg_hi_addr_d, reg_lo_addr_d;
wire hi_new  = (reg_hi_addr != reg_hi_addr_d);
wire lo_new  = (reg_lo_addr != reg_lo_addr_d);
wire rd_hi_e = reg_rd_hi & (~reg_rd_hi_d | hi_new);
wire rd_lo_e = reg_rd_lo & (~reg_rd_lo_d | lo_new);
wire wr_hi_e = reg_wr_hi & (~reg_wr_hi_d | hi_new);
wire wr_lo_e = reg_wr_lo & (~reg_wr_lo_d | lo_new);

reg reg_ready_r;
assign reg_ready = reg_ready_r;

// ---------------------------------------------------------------------------
// kick detect  (ported from a4091_siop.v lines ~687-727)
// arm on a DSP-MSB (reg 0x2F) write; fire once the trailing DSP-LSB (0x2C)
// write lands, or after >=6 idle reg-bus cycles for a lone 0x2F write.
// ---------------------------------------------------------------------------
reg        kick_arm;
reg        kick_lo_seen;
reg  [2:0] kick_idle;
reg        kick_pending;         // sticky - ARM polls (0x64), clears (0x67)

// ISTAT (0x21) side effects the CPU expects synchronously.
reg        int_pending;
reg        soft_reset_pending;   // CPU wrote ISTAT bit6 - ARM re-inits the model
assign int2 = int_pending;

// Cache-flush stretch: 0x67 bit4 (one clk_sys) -> hold cache_clr high long
// enough for both RAM controllers (clk_114) to sample it into cpu_cacr[3]
// across a few !cpu_cs windows. ~64 clk_sys = ~256 clk_114.
reg  [6:0] cache_clr_cnt;
assign cache_clr = |cache_clr_cnt;

wire any_reg_bus = reg_rd_hi | reg_rd_lo | reg_wr_hi | reg_wr_lo;

always @(posedge clk) begin
	if (reset) begin
		reg_rd_hi_d <= 0; reg_rd_lo_d <= 0; reg_wr_hi_d <= 0; reg_wr_lo_d <= 0;
		reg_hi_addr_d <= 0; reg_lo_addr_d <= 0;
		reg_rq_hi <= 8'hff; reg_rq_lo <= 8'hff;
		reg_ready_r <= 0;
		kick_arm <= 0; kick_lo_seen <= 0; kick_idle <= 0; kick_pending <= 0;
		int_pending <= 0; soft_reset_pending <= 0;
		cache_clr_cnt <= 0;
	end
	else begin
		reg_rd_hi_d <= reg_rd_hi;  reg_rd_lo_d <= reg_rd_lo;
		reg_wr_hi_d <= reg_wr_hi;  reg_wr_lo_d <= reg_wr_lo;
		reg_hi_addr_d <= reg_hi_addr;  reg_lo_addr_d <= reg_lo_addr;

		reg_ready_r <= any_reg_bus;   // 1-cycle settle, same contract as a4091_siop

		// -- reads: latch the shadow byte, apply side effects once --
		if (rd_hi_e) reg_rq_hi <= regs[reg_hi_addr];
		if (rd_lo_e) reg_rq_lo <= regs[reg_lo_addr];
		// reading ISTAT (0x21) clears the pending interrupt (real 53C710)
		if ((rd_hi_e && reg_hi_addr == 8'h21) ||
		    (rd_lo_e && reg_lo_addr == 8'h21))
			int_pending <= 1'b0;

		// -- writes: store the byte(s) once per access --
		if (wr_hi_e) regs[reg_hi_addr] <= reg_wd_hi;
		if (wr_lo_e) regs[reg_lo_addr] <= reg_wd_lo;
		// ISTAT bit6 = software reset request
		if ((wr_hi_e && reg_hi_addr == 8'h21 && reg_wd_hi[6]) ||
		    (wr_lo_e && reg_lo_addr == 8'h21 && reg_wd_lo[6]))
			soft_reset_pending <= 1'b1;

		// ---- kick ----------------------------------------------------
		if ((wr_hi_e && reg_hi_addr == 8'h2f) ||
		    (wr_lo_e && reg_lo_addr == 8'h2f)) begin
			kick_arm     <= 1'b1;
			kick_idle    <= 0;
			kick_lo_seen <= 1'b0;
		end
		else if (kick_arm) begin
			if (any_reg_bus) kick_idle <= 0;
			else if (~&kick_idle) kick_idle <= kick_idle + 1'b1;
			if ((wr_hi_e && reg_hi_addr == 8'h2c) ||
			    (wr_lo_e && reg_lo_addr == 8'h2c)) kick_lo_seen <= 1'b1;
		end
		// DCNTL.STD (reg 0x3B bit 2): the 53C710 driver RESUMES SCRIPTS from
		// the current DSP by writing this, not by rewriting DSP. Treat it as
		// an immediate kick.
		if ((wr_hi_e && reg_hi_addr == 8'h3b && reg_wd_hi[2]) ||
		    (wr_lo_e && reg_lo_addr == 8'h3b && reg_wd_lo[2]))
			kick_pending <= 1'b1;

		if (kick_arm && (kick_lo_seen || kick_idle >= 3'd6)) begin
			kick_arm     <= 1'b0;
			kick_pending <= 1'b1;
		end

		// ---- HPS mailbox side ---------------------------------------
		if (mbx_regs_wr) regs[mbx_addr] <= mbx_wdata;
		mbx_rdata <= regs[mbx_addr];          // 0x65 burst read (registered)

		if (mbx_set_int)  int_pending        <= 1'b1;
		if (mbx_clr_int)  int_pending        <= 1'b0;
		if (mbx_clr_kick) kick_pending       <= 1'b0;
		if (mbx_clr_srst) soft_reset_pending <= 1'b0;

		if (mbx_cache_clr)       cache_clr_cnt <= 7'd64;
		else if (|cache_clr_cnt) cache_clr_cnt <= cache_clr_cnt - 1'b1;
	end
end

// ---------------------------------------------------------------------------
// 0x64 poll word
// ---------------------------------------------------------------------------
assign mbx_status = { 8'd0,
                      regs[8'h21][5],     // SIGP  (CPU asked for reselect)
                      3'd0,
                      dma_req,
                      soft_reset_pending,
                      int_pending,
                      kick_pending };

// ---------------------------------------------------------------------------
// chip / Z2 DMA slot - tied off until PR2 (exec-plan Phase A.2)
// ---------------------------------------------------------------------------
assign dma_req   = 1'b0;
assign dma_rw    = 1'b1;
assign dma_addr  = 31'd0;
assign dma_wdata = 16'd0;
assign dma_bs    = 2'b00;

assign led = kick_pending | int_pending;

assign dbg_bus = { 32'hA4091B01,          // signature
                   regs[8'h0c],           // DSTAT shadow
                   regs[8'h21],           // ISTAT shadow
                   5'd0, soft_reset_pending, int_pending, kick_pending };

endmodule

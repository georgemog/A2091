// ===========================================================================
// A4091 - Commodore/DKB Zorro III DMA Fast SCSI-2 host adapter for Minimig-AGA
//
// Architecture: SOFTWARE SIOP. The 53C710 register model + SCRIPTS VM + SCSI-2
// emulation run on the HPS ARM (support/minimig/a4091_lsi.cpp + a4091_scsi.c).
// This module is the FPGA-side bridge only:
//   * Zorro-III autoconfig ($E80000) + autoboot ROM window   - unchanged
//   * board-window decode + DTACK handshake                  - unchanged
//   * a4091_bridge: 256 B shadow register RAM + kick + IRQ    - replaces the
//     old a4091_siop.v / a4091_target.v / a4091_sd.v RTL SIOP
//
// See A4091/software-siop-plan.md and software-siop-exec-plan.md.
// ===========================================================================

module a4091
(
	input             clk,          // clk_sys 28.6875 MHz
	input             reset,        // active high

	input             enable,       // OSD: board present (gate autoconfig)
	input       [2:0] cfg_scsi_id,  // DIP 1-3: host SCSI ID
	input       [4:0] cfg_dip,      // DIP 4-8: fastbus/autoboot/sync/term/lun

	// ---- Zorro autoconfig ($E80000 config space) -------------------------
	input             ac_cycle,     // 1 = $E8xxxx access, our turn
	input       [5:0] ac_reg,       // config register = chip_addr[6:1]
	input             ac_write,     // 1 = write (base assignment / shutup)
	input      [15:0] ac_wdata,
	output reg  [3:0] ac_rdata,     // nibble on D15:D12
	output reg        ac_done,      // 1 = configured or shut up -> advance chain

	// ---- board window (post-config), from cpu_wrapper.v ------------------
	output     [31:24] board_base,
	output            board_cfgd,
	input             brd_sel,
	input      [23:0] brd_addr,
	input      [15:0] brd_din,
	output     [15:0] brd_dout,
	input             brd_lds,
	input             brd_uds,
	input             brd_rnw,
	output            brd_selack,
	output            brd_ready,

	// ---- chip / Zorro-II bus-master DMA (arbitrated onto the CPU ram port)
	// Bulk DATA is ARM->DDR memcpy and never comes here; this is the rare
	// chip / Z2-fast bounce-buffer path only. Wired in exec-plan PR2.
	output            dma_req,
	output            dma_rw,
	output     [31:1] dma_addr,
	output     [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	output      [1:0] dma_bs,
	input             dma_ack,

	// ---- interrupt ------------------------------------------------------
	output            int2,

	// ---- boot ROM load (HPS ioctl_download) ----------------------------
	input             rom_wr,
	input      [15:0] rom_addr,     // word address into 32KB image (0..0x3FFF)
	input       [7:0] rom_data,

	// ---- HPS bridge mailbox (hps_ext.v) -------------------------------
	input      [7:0]  mbx_addr,
	input             mbx_regs_wr,
	input      [7:0]  mbx_wdata,
	output     [7:0]  mbx_rdata,
	input             mbx_set_int,
	input             mbx_clr_int,
	input             mbx_clr_kick,
	input             mbx_clr_srst,
	input             mbx_cache_clr,
	output     [15:0] mbx_status,
	output            cache_clr,      // -> cpu_cacr[3] of both RAM controllers

	// ---- compact debug bus (HPS-readable via hps_ext 'h68) --------------
	output     [63:0] dbg_bus,

	output            led           // activity LED
);

// ---------------------------------------------------------------------------
// AutoConfig ident table (unchanged - matches a4091-software rom.S)
// ---------------------------------------------------------------------------
localparam [7:0]  AC_TYPE    = 8'h90;        // Z3 | DIAGVALID | size-field 0
localparam [7:0]  AC_PRODUCT = 8'h54;        // 84
localparam [7:0]  AC_FLAGS   = 8'h30;        // EXTENDED | ZORRO_III  -> 16 MB
localparam [15:0] AC_MFG     = 16'h0202;     // 514
localparam [31:0] AC_SERIAL  = 32'h002A_0027; // VERSION 42 (0x2A), REVISION 39 (0x27)
localparam [15:0] AC_ROMVEC  = 16'h0200;     // DiagArea offset within the board window

reg [7:0] ac_byte;
reg [3:0] ac_nibble;
always @(*) begin
	case (ac_reg[5:1])
		5'd0:  ac_byte = AC_TYPE;
		5'd1:  ac_byte = AC_PRODUCT;
		5'd2:  ac_byte = AC_FLAGS;
		5'd3:  ac_byte = 8'h00;
		5'd4:  ac_byte = AC_MFG[15:8];
		5'd5:  ac_byte = AC_MFG[7:0];
		5'd6:  ac_byte = AC_SERIAL[31:24];
		5'd7:  ac_byte = AC_SERIAL[23:16];
		5'd8:  ac_byte = AC_SERIAL[15:8];
		5'd9:  ac_byte = AC_SERIAL[7:0];
		5'd10: ac_byte = AC_ROMVEC[15:8];
		5'd11: ac_byte = AC_ROMVEC[7:0];
		default: ac_byte = 8'h00;
	endcase
	ac_nibble = ac_reg[0] ? ac_byte[3:0] : ac_byte[7:4];
end

reg        cfgd;
reg [7:0]  base_hi;
assign board_base = base_hi;
assign board_cfgd = cfgd;

always @(posedge clk) begin
	if (reset) begin
		cfgd    <= 1'b0;
		base_hi <= 8'h00;
		ac_done <= 1'b0;
	end
	else if (!cfgd && enable && ac_cycle) begin
		if (ac_write) begin
			if (ac_reg == 6'h22) begin
				base_hi <= ac_wdata[15:8];
				cfgd    <= 1'b1;
				ac_done <= 1'b1;
			end
			else if (ac_reg == 6'h26) begin
				ac_done <= 1'b1;   // board declines, advance chain
			end
		end
	end
end

always @(*) ac_rdata = (ac_reg[5:1] == 5'd0) ? ac_nibble : ~ac_nibble;

// ---------------------------------------------------------------------------
// Board window decode (unchanged)
//   0x000000..0x7FFFFF  boot ROM
//   0x800000..0x87FFFF  53C710 registers
//   0x8C0003            DIP switch byte
//   0x8D0000..0x8D03FF  bridge debug window
// ---------------------------------------------------------------------------
wire sel_rom = brd_sel && (brd_addr[23] == 1'b0);
wire sel_reg = brd_sel && (brd_addr[23:19] == 5'b1000_0);
wire sel_dip = brd_sel && (brd_addr[23:1] == (24'h8C0003 >> 1));
wire sel_dbg = brd_sel && (brd_addr[23:10] == 14'h2340);

assign brd_selack = sel_rom | sel_reg | sel_dip | sel_dbg;

// --- boot ROM ----------------------------------------------------------
// Synthesis: the (* ram_init_file *) attribute points Quartus at the .mif,
// so a driver-only change is picked up by MIF/HEX Update (~2 min) instead
// of a full recompile. Simulation: the tb passes -DA4091_ROM_HEX=<path>
// and $readmemh loads the byte-per-line hex instead (iverilog ignores the
// attribute). Keep a4091_rom.mif and a4091_rom.hex in sync - make_hex.sh
// emits both from the same a4091.rom.
(* ram_init_file = "rtl/a4091/a4091_rom.mif" *)
reg [7:0] rom_mem [0:65535];
`ifdef A4091_ROM_HEX
initial $readmemh(`A4091_ROM_HEX, rom_mem);
`endif

reg  [7:0] rom_rb;
reg  [1:0] rom_lane;
always @(posedge clk) begin
	if (rom_wr) rom_mem[rom_addr] <= rom_data;
	rom_rb   <= rom_mem[brd_addr[17:2]];
	rom_lane <= brd_addr[1:0];
end
wire [15:0] rom_q = (rom_lane == 2'd0) ? {rom_rb[7:4], 12'hfff}
                  : (rom_lane == 2'd2) ? {rom_rb[3:0], 12'hfff}
                  :                      16'hffff;

// --- DIP switch byte (unchanged) ---------------------------------------
wire [7:0] dip_byte = ({cfg_dip, 3'b000} ^ 8'hF8) | {5'b0, cfg_scsi_id};

// ---------------------------------------------------------------------------
// 53C710 register window -> a4091_bridge shadow RAM
// Each 16-bit bus word covers two adjacent (byte-swapped) registers:
//   D15:8 (uds) -> reg {base, base&3==0 ? 3 : 1}
//   D7:0  (lds) -> reg {base, base&3==0 ? 2 : 0}
// ---------------------------------------------------------------------------
wire [7:0]  reg_base    = {brd_addr[5:2], 2'b00};
wire [7:0]  reg_hi_addr  = (reg_base | (brd_addr[1] ? 8'd1 : 8'd3)) & 8'h3f;
wire [7:0]  reg_lo_addr  = (reg_base | (brd_addr[1] ? 8'd0 : 8'd2)) & 8'h3f;
wire        reg_hi_stb   = sel_reg & brd_uds;
wire        reg_lo_stb   = sel_reg & brd_lds;

wire [7:0]  brg_rq_hi, brg_rq_lo;
wire        brg_ready;
wire [15:0] reg_dout16 = {brg_rq_hi, brg_rq_lo};

a4091_bridge bridge
(
	.clk        (clk),
	.reset      (reset),

	.reg_hi_addr (reg_hi_addr),
	.reg_lo_addr (reg_lo_addr),
	.reg_rd_hi   (reg_hi_stb &  brd_rnw),
	.reg_rd_lo   (reg_lo_stb &  brd_rnw),
	.reg_wr_hi   (reg_hi_stb & ~brd_rnw),
	.reg_wr_lo   (reg_lo_stb & ~brd_rnw),
	.reg_wd_hi   (brd_din[15:8]),
	.reg_wd_lo   (brd_din[7:0]),
	.reg_rq_hi   (brg_rq_hi),
	.reg_rq_lo   (brg_rq_lo),
	.reg_ready   (brg_ready),

	.int2       (int2),

	.mbx_addr     (mbx_addr),
	.mbx_regs_wr  (mbx_regs_wr),
	.mbx_wdata    (mbx_wdata),
	.mbx_rdata    (mbx_rdata),
	.mbx_set_int  (mbx_set_int),
	.mbx_clr_int  (mbx_clr_int),
	.mbx_clr_kick (mbx_clr_kick),
	.mbx_clr_srst (mbx_clr_srst),
	.mbx_cache_clr(mbx_cache_clr),
	.mbx_status   (mbx_status),
	.cache_clr    (cache_clr),

	.dma_req    (dma_req),
	.dma_rw     (dma_rw),
	.dma_addr   (dma_addr),
	.dma_wdata  (dma_wdata),
	.dma_rdata  (dma_rdata),
	.dma_bs     (dma_bs),
	.dma_ack    (dma_ack),

	.dbg_bus    (dbg_bus),
	.led        (led)
);

// ---------------------------------------------------------------------------
// bridge debug window (0x8D0000) - slim: signature + shadow regs + flags
// ---------------------------------------------------------------------------
reg [15:0] dbg_q;
always @(*) begin
	case (brd_addr[7:1])
	7'h00: dbg_q = 16'hA491;                 // signature
	7'h01: dbg_q = 16'hB012;                 // bridge version
	7'h02: dbg_q = dbg_bus[15:0];            // {srst,int,kick} flags + ISTAT shadow
	7'h03: dbg_q = dbg_bus[31:16];           // DSTAT shadow
	7'h04: dbg_q = mbx_status;
	7'h05: dbg_q = {AC_FLAGS, AC_TYPE};      // ident echo
	default: dbg_q = 16'h0000;
	endcase
end

// ---------------------------------------------------------------------------
// board read data mux + ready (unchanged contract - LEVEL, not a pulse)
// ---------------------------------------------------------------------------
reg        sel_rom_d, sel_dip_d, sel_reg_d, sel_dbg_d;
reg [23:0] brd_addr_d;
always @(posedge clk) begin
	sel_rom_d  <= sel_rom;
	sel_dip_d  <= sel_dip;
	sel_reg_d  <= sel_reg;
	sel_dbg_d  <= sel_dbg;
	brd_addr_d <= brd_addr;
end

wire brd_stable = (brd_addr == brd_addr_d);

assign brd_dout = sel_rom ? rom_q :
                  sel_dip ? {8'hff, dip_byte} :
                  sel_dbg ? dbg_q :
                            reg_dout16;

assign brd_ready = brd_sel & brd_stable & ( (sel_rom & sel_rom_d)
                                          | (sel_dip & sel_dip_d)
                                          | (sel_dbg & sel_dbg_d)
                                          | (sel_reg & sel_reg_d & brg_ready) );

endmodule

// Modified by Princeton University on June 9th, 2015
// ========== Copyright Header Begin ==========================================
//
// OpenSPARC T1 Processor File: cmp_top.v
// Copyright (c) 2006 Sun Microsystems, Inc.  All Rights Reserved.
// DO NOT ALTER OR REMOVE COPYRIGHT NOTICES.
//
// The above named program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License version 2 as published by the Free Software Foundation.
//
// The above named program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this work; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// ========== Copyright Header End ============================================
////////////////////////////////////////////////////////

`ifndef USE_TEST_TOP // don't compile if user wants to use deprecated TOPs
`include "sys.h"
`include "iop.h"
`include "cross_module.tmp.h"
`include "ifu.tmp.h"
`include "define.tmp.h"
`include "piton_system.vh"
`include "jtag.vh"

// /home/alireza/work/git/OpenPiton/OpenPiton-Mess-integration/piton/verif/env/manycore/devices_ariane.xml


`ifdef PITON_DPI
import "DPI-C" function longint read_64b_call (input longint addr);
import "DPI-C" function void write_64b_call (input longint addr, input longint data);
import "DPI-C" function int drive_iob ();
import "DPI-C" function int get_cpx_word (int index);
import "DPI-C" function void report_pc (longint thread_pc);
import "DPI-C" function void init_jbus_model_call(string str, int oram);

`ifndef VERILATOR
// MPI Yummy functions
import "DPI-C" function void initialize();
import "DPI-C" function void finalize();
import "DPI-C" function int getRank();
import "DPI-C" function int getSize();

import "DPI-C" function void mpi_send_yummy(input byte unsigned message, input int dest, input int rank, input int flag);
import "DPI-C" function byte unsigned mpi_receive_yummy(input int origin, input int flag);

import "DPI-C" function longint unsigned mpi_receive_data(input int origin, output byte unsigned valid, input int flag);
import "DPI-C" function void mpi_send_data(input longint unsigned data, input byte unsigned valid, input int dest, input int rank, input int flag);

import "DPI-C" function void barrier();
//add metro_stuff
`endif // ifndef VERILATOR
`endif // ifdef PITON_DPI



`timescale 1ps/1ps
module metro_tile (

`ifdef VERILATOR
	core_ref_clk,
	sys_rst_n,
	pll_rst_n,
	clk_en,
	pll_bypass,
	pll_rangea,
	pll_lock,
	clk_mux_sel,
	async_mux,
	diag_done,
	ok_iob,

	default_chipid,
	default_coreid_x,
	default_coreid_y,
	flat_tileid,
    cpu_enable,
    current_r_id,

	noc_chanel_in,
	noc_chanel_out,
	
    //parameter as output
    smart_max,
    pronoc_T1,
    pronoc_T2,

    //statistic
    hpm_st,
    cache_st, 
    flit_st,
    pck_st,
    
    //To check trap
    inst_done,
    phy_pc_w

`endif

);



`ifdef PITON_PRONOC
	
	`include "pronoc_def.v"
 	`NOC_CONF
	
	typedef struct packed {	
		smartflit_chanel_t  [2:0] smartflit_chanel;  		
	} noc_chanel_t;

	localparam CHIP_SET_ID = T1*T2*T3+2*T1; // endp connected  of west port of router 0-0
	localparam CHIP_SET_PORT = 3; //west port of first router

	localparam CONCENTRATION = T3;

`else

    

	typedef struct packed {	
		logic  [`NOC_DATA_WIDTH-1:0] data1;
		logic  [`NOC_DATA_WIDTH-1:0] data2;
		logic  [`NOC_DATA_WIDTH-1:0] data3;
		logic  [2:0] valid;
		logic  [2:0] yummy;		
	} noc_chanel_t;

    localparam CONCENTRATION = 1;
	localparam SMARTFLIT_CHANEL_w=1;
	localparam CHIP_SET_ID = 0; // endp connected  of west port of router 0-0
	localparam CHIP_SET_PORT = 3; //west port of first router
	localparam RAw=0;
	
	
`endif


localparam NOC_CHANEL_w = $bits(noc_chanel_t); 	


localparam 
	PITON_EAST    =   0,
	PITON_NORTH   =   1, 
	PITON_WEST    =   2,
	PITON_SOUTH   =   3,
    PITON_P       =   4;   

localparam 
    PRONOC_LOCAL   =   0,
    PRONOC_EAST    =   1,
    PRONOC_NORTH   =   2, 
    PRONOC_WEST    =   3,
    PRONOC_SOUTH   =   4,
    PRONOC_P       =   5;



`ifdef VERILATOR
input reg                             core_ref_clk;
input reg                             sys_rst_n;
input reg                             pll_rst_n;
input reg                             clk_en;
input reg                             pll_bypass;
input reg [4:0]                       pll_rangea;
output wire                           pll_lock;
input reg [1:0]                       clk_mux_sel;
input reg                             async_mux;
input                                 diag_done;
input                                 ok_iob;

input      [31 : 0]                   current_r_id;
input wire [13:0]                     default_chipid ;
input wire [7:0]                      default_coreid_x [CONCENTRATION-1 : 0];
input wire [7:0]                      default_coreid_y [CONCENTRATION-1 : 0];
input wire [`JTAG_FLATID_WIDTH-1:0]   flat_tileid [CONCENTRATION-1 : 0];
input  cpu_enable [CONCENTRATION-1 : 0];


input  noc_chanel_t noc_chanel_in	[PITON_P-1 : 0];
output noc_chanel_t noc_chanel_out	[PITON_P-1 : 0];
output [31 : 0] smart_max;
output [31 : 0 ] pronoc_T1;
output [31 : 0 ] pronoc_T2;

output [31 : 0] cache_st [11: 0];
output [63 : 0] flit_st  [0: 5]; // count flit in/out to/from processor for 3 NoCs 
output [63 : 0] pck_st   [0: 5][0:11]; // packet sizes histogram in/out to/from processor for 3 NoCs 
output reg [31 : 0] hpm_st [43 : 0];
output inst_done;
output [63:0] phy_pc_w;  
 
metro_hpm hpm (
    .flat_tileid(flat_tileid[0]),
    .hpm_st(hpm_st),
    .cache_st(cache_st),
    .flit_st(flit_st),
    .pck_st(pck_st),
    .clk(core_ref_clk),
    .rst_n(sys_rst_n),
    .inst_done(inst_done),
    .phy_pc_w(phy_pc_w),
    .roi_start(),
    .roi_en()
);


`endif





//////////////////////
// Type Declarations
//////////////////////

`ifndef VERILATOR
reg                             core_ref_clk;
reg                             sys_rst_n;
reg                             pll_rst_n;
reg                             clk_en;
reg                             pll_bypass;
reg [4:0]                       pll_rangea;
wire                            pll_lock = 1'b1;
reg [1:0]                       clk_mux_sel;
reg                             async_mux;
// For simulation only, monitor stuff.  Only cross-module referenced
// do not delete.
reg                             diag_done;
`endif // ifndef VERILATOR

reg                             io_clk;
reg                             jtag_clk;
reg                             chipset_clk_osc_p;
reg                             chipset_clk_osc_n;
reg                             chipset_clk_osc;
reg                             chipset_clk;
reg                             mem_clk;
reg                             spi_sys_clk;
reg                             chipset_passthru_clk_p;
reg                             chipset_passthru_clk_n;
reg                             passthru_clk_osc_p;
reg                             passthru_clk_osc_n;
reg                             passthru_chipset_clk_p;
reg                             passthru_chipset_clk_n;

reg                             jtag_rst_l;

reg                             jtag_modesel;
reg                             jtag_datain;
wire                            jtag_dataout;


// For simulation only, monitor stuff.  Only cross-module referenced
// do not delete.
reg                             fail_flag;
reg [3:0]                       stub_done;
reg [3:0]                       stub_pass;

`ifndef VERILATOR
reg [`NOC_DATA_WIDTH-1:0] in_N_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] in_E_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] in_W_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] in_S_noc1_data;
reg                       in_N_noc1_valid;
reg                       in_E_noc1_valid;
reg                       in_W_noc1_valid;
reg                       in_S_noc1_valid;
reg                       in_N_noc1_yummy;
reg                       in_E_noc1_yummy;
reg                       in_W_noc1_yummy;
reg                       in_S_noc1_yummy;

reg [`NOC_DATA_WIDTH-1:0] out_N_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] out_E_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] out_W_noc1_data;
reg [`NOC_DATA_WIDTH-1:0] out_S_noc1_data;
reg                       out_N_noc1_valid;
reg                       out_E_noc1_valid;
reg                       out_W_noc1_valid;
reg                       out_S_noc1_valid;
reg                       out_N_noc1_yummy;
reg                       out_E_noc1_yummy;
reg                       out_W_noc1_yummy;
reg                       out_S_noc1_yummy;

reg [`NOC_DATA_WIDTH-1:0] in_N_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] in_E_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] in_W_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] in_S_noc2_data;
reg                       in_N_noc2_valid;
reg                       in_E_noc2_valid;
reg                       in_W_noc2_valid;
reg                       in_S_noc2_valid;
reg                       in_N_noc2_yummy;
reg                       in_E_noc2_yummy;
reg                       in_W_noc2_yummy;
reg                       in_S_noc2_yummy;

reg [`NOC_DATA_WIDTH-1:0] out_N_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] out_E_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] out_W_noc2_data;
reg [`NOC_DATA_WIDTH-1:0] out_S_noc2_data;
reg                       out_N_noc2_valid;
reg                       out_E_noc2_valid;
reg                       out_W_noc2_valid;
reg                       out_S_noc2_valid;
reg                       out_N_noc2_yummy;
reg                       out_E_noc2_yummy;
reg                       out_W_noc2_yummy;
reg                       out_S_noc2_yummy;

reg [`NOC_DATA_WIDTH-1:0] in_N_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] in_E_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] in_W_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] in_S_noc3_data;
reg                       in_N_noc3_valid;
reg                       in_E_noc3_valid;
reg                       in_W_noc3_valid;
reg                       in_S_noc3_valid;
reg                       in_N_noc3_yummy;
reg                       in_E_noc3_yummy;
reg                       in_W_noc3_yummy;
reg                       in_S_noc3_yummy;

reg [`NOC_DATA_WIDTH-1:0] out_N_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] out_E_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] out_W_noc3_data;
reg [`NOC_DATA_WIDTH-1:0] out_S_noc3_data;
reg                       out_N_noc3_valid;
reg                       out_E_noc3_valid;
reg                       out_W_noc3_valid;
reg                       out_S_noc3_valid;
reg                       out_N_noc3_yummy;
reg                       out_E_noc3_yummy;
reg                       out_W_noc3_yummy;
reg                       out_S_noc3_yummy;

reg [13:0]                     default_chipid = 14'b0;
reg [7:0]                      default_coreid_x = 8'b0;
reg [7:0]                      default_coreid_y = 8'b0;
reg [`JTAG_FLATID_WIDTH-1:0]   flat_tileid = `JTAG_FLATID_WIDTH'b0;

`endif // ifndef VERILATOR


////////////////////
// Simulated Clocks
////////////////////

`ifndef VERILATOR
`ifndef USE_FAKE_PLL_AND_CLKMUX
always #5000 core_ref_clk = ~core_ref_clk;                      // 100MHz
`else
always #500 core_ref_clk = ~core_ref_clk;                       // 1000MHz
`endif
`endif

`ifndef SYNC_MUX
always #1429 io_clk = ~io_clk;                                  // 350MHz
`else
always @ * io_clk = core_ref_clk;
`endif

`ifndef VERILATOR
always #50000 jtag_clk = ~jtag_clk;                             // 10MHz

always #2500 chipset_clk_osc_p = ~chipset_clk_osc_p;            // 200MHz
always @ * chipset_clk_osc_n = ~chipset_clk_osc_p;

always #5000 chipset_clk_osc = ~chipset_clk_osc;                // 100MHz

always #2500 chipset_clk = ~chipset_clk;                        // 200MHz

always #3333 passthru_clk_osc_p = ~passthru_clk_osc_p;          // 150MHz
always @ * passthru_clk_osc_n = ~passthru_clk_osc_p;

always #1429 passthru_chipset_clk_p = ~passthru_chipset_clk_p;  // 350MHz
always @ * passthru_chipset_clk_n = ~passthru_chipset_clk_p;

always #1000 mem_clk = ~mem_clk;                                // 500MHz

always #25000 spi_sys_clk = ~spi_sys_clk;                       // 20MHz
`endif

////////////////////////////////////////////////////////
// SIMULATED BOOT SEQUENCE
////////////////////////////////////////////////////////

int           rank; 
int           size; 
int           dest;
byte unsigned valid_aux;

int           tile_x; 
int           tile_y;
int           rankN; 
int           rankE; 
int           rankS; 
int           rankW;


int YUMMY_NOC_1 ;
int DATA_NOC_1  ;
int YUMMY_NOC_2 ;
int DATA_NOC_2  ;
int YUMMY_NOC_3 ;
int DATA_NOC_3  ;

`ifndef VERILATOR
initial
begin
    string filerank;

    YUMMY_NOC_1 = 0;
    DATA_NOC_1  = 1;
    YUMMY_NOC_2 = 2;
    DATA_NOC_2  = 3;
    YUMMY_NOC_3 = 4;
    DATA_NOC_3  = 5;

    //metro initialization
    initialize();
    //barrier();
    rank = getRank();
    size = getSize();
    filerank.itoa(rank);
    $dumpfile({"metro_tile_",filerank,".vcd"});
    $dumpvars(0, metro_tile);

    $display("METRO_TILE INITIALIZING...");
    $display("size: %d", size);
    $display("rank: %d", rank);
    if (rank==0) begin
            dest = 1;
    end else begin
            dest = 0;
    end

    //Guillem's functions that I'm hardcoding in
    //int get_rank_fromXY(int x, int y) { return 1 + ((x)+((`PITON_X_TILES)*y));}
    //int getRankN () {
    //if (tile_y == 0)
    //    return -1;
    //else
    //    return get_rank_fromXY(tile_x, tile_y-1);
    //}
    //int getRankS () {
    //    if (tile_y+1 == `PITON_Y_TILES)
    //        return -1;
    //    else
    //        return get_rank_fromXY(tile_x, tile_y+1);
    //}
    //int getRankE () {
    //    if (tile_x+1 == `PITON_X_TILES)
    //        return -1;
    //    else
    //        return get_rank_fromXY(tile_x+1, tile_y);
    //}
    //int getRankW () {
    //    if (rank==1) { // go to chipset
    //        return 0;
    //    }
    //    else if (tile_x == 0) {
    //        return -1;
    //    }
    //    else {
    //        return get_rank_fromXY(tile_x-1, tile_y);
    //    }
    //}

    tile_x = (rank-1)%`PITON_X_TILES;
    tile_y = (rank-1)/`PITON_X_TILES;

    //rankN
    if (tile_y == 0) begin
        rankN = -1;
    end else begin
        rankN = 1 + ((tile_x)+((`PITON_X_TILES)*(tile_y-1)));
    end
    
    //rankE
    if (tile_x+1 == `PITON_X_TILES) begin
        rankE = -1;
    end else begin
        rankE = 1 + ((tile_x+1)+((`PITON_X_TILES)*tile_y));
    end

    //rankS
    if (tile_y+1 == `PITON_Y_TILES) begin
        rankS = -1;
    end else begin
        rankS = 1 + ((tile_x)+((`PITON_X_TILES)*(tile_y+1)));
    end

    //rankW
    if (rank==1) begin
        rankW = 0;
    end else if (tile_x == 0) begin
        rankW = -1;
    end else begin
        rankW = 1 + ((tile_x-1)+((`PITON_X_TILES)*tile_y));
    end

    default_chipid   = 14'b0;
    default_coreid_x = tile_x;
    default_coreid_y = tile_y;
    flat_tileid = rank-1;

    // These are not referenced elsewhere in this module,
    // but are cross referenced from monitor.v.pyv.  Do not
    // delete
    fail_flag = 1'b0;
    stub_done = 4'b0;
    stub_pass = 4'b0;

    // Clocks initial value
    core_ref_clk = 1'b0;
    io_clk = 1'b0;
    jtag_clk = 1'b0;
    chipset_clk_osc_p = 1'b0;
    chipset_clk_osc_n = 1'b1;
    chipset_clk_osc = 1'b0;
    chipset_clk = 1'b0;
    mem_clk = 1'b0;
    spi_sys_clk = 1'b0;
    chipset_passthru_clk_p = 1'b0;
    chipset_passthru_clk_n = 1'b1;
    passthru_clk_osc_p = 1'b0;
    passthru_clk_osc_n = 1'b1;
    passthru_chipset_clk_p = 1'b0;
    passthru_chipset_clk_n = 1'b1;

    // Resets are held low at start of boot
    sys_rst_n = 1'b0;
    jtag_rst_l = 1'b0;
    pll_rst_n = 1'b0;

    // Mostly DC signals set at start of boot
    clk_en = 1'b0;
    if ($test$plusargs("pll_en"))
    begin
        // PLL is disabled by default
        pll_bypass = 1'b0; // trin: pll_bypass is a switch in the pll; not reliable
        clk_mux_sel[1:0] = 2'b10; // selecting pll
    end
    else
    begin
        pll_bypass = 1'b1; // trin: pll_bypass is a switch in the pll; not reliable
        clk_mux_sel[1:0] = 2'b00; // selecting ref clock
    end
    // rangeA = x10 ? 5'b1 : x5 ? 5'b11110 : x2 ? 5'b10100 : x1 ? 5'b10010 : x20 ? 5'b0 : 5'b1;
    pll_rangea = 5'b00001; // 10x ref clock
    // pll_rangea = 5'b11110; // 5x ref clock
    // pll_rangea = 5'b00000; // 20x ref clock

    // JTAG simulation currently not supported here
    jtag_modesel = 1'b1;
    jtag_datain = 1'b0;

`ifndef SYNC_MUX
    async_mux = 1'b1;
`else
    async_mux = 1'b0;
`endif

`ifndef METRO_TILE
    // Init JBUS model plus some ORAM stuff
    if ($test$plusargs("oram"))
    begin
`ifndef PITON_DPI
        $init_jbus_model("mem.image", 1);
`else // ifndef PITON_DPI
        init_jbus_model_call("mem.image", 1);
`endif // ifndef PITON_DPI
`ifndef __ICARUS__
        force system.chip.ctap_oram_clk_en = 1'b1;
`endif
    end
    else
    begin
`ifndef PITON_DPI
        $init_jbus_model("mem.image", 0);
`else // ifndef PITON_DPI
        $display("init_jbus_model_call");
        init_jbus_model_call("mem.image", 0);
`endif // ifndef PITON_DPI
    end
`endif //METRO_TILE

    in_N_noc1_data  = 0;
    in_E_noc1_data  = 0;
    in_W_noc1_data  = 0;
    in_S_noc1_data  = 0;
    in_N_noc1_valid = 0;
    in_E_noc1_valid = 0;
    in_W_noc1_valid = 0;
    in_S_noc1_valid = 0;
    in_N_noc1_yummy = 0;
    in_E_noc1_yummy = 0;
    in_W_noc1_yummy = 0;
    in_S_noc1_yummy = 0;

    in_N_noc2_data  = 0;
    in_E_noc2_data  = 0;
    in_W_noc2_data  = 0;
    in_S_noc2_data  = 0;
    in_N_noc2_valid = 0;
    in_E_noc2_valid = 0;
    in_W_noc2_valid = 0;
    in_S_noc2_valid = 0;
    in_N_noc2_yummy = 0;
    in_E_noc2_yummy = 0;
    in_W_noc2_yummy = 0;
    in_S_noc2_yummy = 0;

    in_N_noc3_data  = 0;
    in_E_noc3_data  = 0;
    in_W_noc3_data  = 0;
    in_S_noc3_data  = 0;
    in_N_noc3_valid = 0;
    in_E_noc3_valid = 0;
    in_W_noc3_valid = 0;
    in_S_noc3_valid = 0;
    in_N_noc3_yummy = 0;
    in_E_noc3_yummy = 0;
    in_W_noc3_yummy = 0;
    in_S_noc3_yummy = 0;

    // Reset PLL for 100 cycles
    repeat(100)@(posedge core_ref_clk);
    pll_rst_n = 1'b1;

    // Wait for PLL lock
    wait( pll_lock == 1'b1 );

    // After 10 cycles turn on chip-level clock enable
    repeat(10)@(posedge `CHIP_INT_CLK);
    clk_en = 1'b1;

    // After 100 cycles release reset
    repeat(100)@(posedge `CHIP_INT_CLK);
    sys_rst_n = 1'b1;
    jtag_rst_l = 1'b1;

    // Wait for SRAM init
    // trin: 5000 cycles is about the lowest for 64KB L2
    // 128KB L2 requires at least 10000
    repeat(5000)@(posedge `CHIP_INT_CLK); // trin: supports at least 512KB L2 per-tile

    diag_done = 1'b1;
`ifndef METRO_TILE
`ifndef PITONSYS_IOCTRL
    // Signal fake IOB to send wake up packet to first tile
    cmp_top.system.chipset.chipset_impl.ciop_fake_iob.ok_iob = 1'b1;
`endif // endif PITONSYS_IOCTRL
`endif // ifndef METRO_TILE

//ok_iob = 1;

//metro code
$display("TILE INITIALIZED");
$display("tile_y: %d",tile_y);
$display("tile_x: %d",tile_x);
$display("rankN: %d",rankN);
$display("rankS: %d",rankS);
$display("rankW: %d",rankW);
$display("rankE: %d",rankE);

@(posedge core_ref_clk);
for(int i = 0; i < 350000; i = i + 1)
begin
    if (i % 10000 == 0) begin
        $display("TIME: %d", i);
    end
    #500;
    if(rankN != -1) begin
        //SENDING
        //$display("sending N");
        mpi_send_data(out_N_noc1_data, out_N_noc1_valid, rankN, rank, DATA_NOC_1);
        // send yummy
        mpi_send_yummy(out_N_noc1_yummy, rankN, rank, YUMMY_NOC_1);

        // send data
        mpi_send_data(out_N_noc2_data, out_N_noc2_valid, rankN, rank, DATA_NOC_2);
        // send yummy
        mpi_send_yummy(out_N_noc2_yummy, rankN, rank, YUMMY_NOC_2);

        // send data
        mpi_send_data(out_N_noc3_data, out_N_noc3_valid, rankN, rank, DATA_NOC_3);
        // send yummy
        mpi_send_yummy(out_N_noc3_yummy, rankN, rank, YUMMY_NOC_3);

        // RECEIVING
        //$display("receiving N");
        in_N_noc1_data = mpi_receive_data(rankN, valid_aux, DATA_NOC_1);
        in_N_noc1_valid = valid_aux;
        // receive yummy
        in_N_noc1_yummy = mpi_receive_yummy(rankN, YUMMY_NOC_1);
        
        in_N_noc2_data = mpi_receive_data(rankN, valid_aux, DATA_NOC_2);
        in_N_noc2_valid = valid_aux;
        // receive yummy
        in_N_noc2_yummy = mpi_receive_yummy(rankN, YUMMY_NOC_2);

        in_N_noc3_data = mpi_receive_data(rankN, valid_aux, DATA_NOC_3);
        in_N_noc3_valid = valid_aux;
        // receive yummy
        in_N_noc3_yummy = mpi_receive_yummy(rankN, YUMMY_NOC_3);
    end

    if(rankE != -1) begin
        //SENDING
        //$display("sending E");
        mpi_send_data(out_E_noc1_data, out_E_noc1_valid, rankE, rank, DATA_NOC_1);
        // send yummy
        mpi_send_yummy(out_E_noc1_yummy, rankE, rank, YUMMY_NOC_1);

        // send data
        mpi_send_data(out_E_noc2_data, out_E_noc2_valid, rankE, rank, DATA_NOC_2);
        // send yummy
        mpi_send_yummy(out_E_noc2_yummy, rankE, rank, YUMMY_NOC_2);

        // send data
        mpi_send_data(out_E_noc3_data, out_E_noc3_valid, rankE, rank, DATA_NOC_3);
        // send yummy
        mpi_send_yummy(out_E_noc3_yummy, rankE, rank, YUMMY_NOC_3);

        // RECEIVING
        //$display("receiving E");
        in_E_noc1_data = mpi_receive_data(rankE, valid_aux, DATA_NOC_1);
        in_E_noc1_valid = valid_aux;
        // receive yummy
        in_E_noc1_yummy = mpi_receive_yummy(rankE, YUMMY_NOC_1);
        
        in_E_noc2_data = mpi_receive_data(rankE, valid_aux, DATA_NOC_2);
        in_E_noc2_valid = valid_aux;
        // receive yummy
        in_E_noc2_yummy = mpi_receive_yummy(rankE, YUMMY_NOC_2);

        in_E_noc3_data = mpi_receive_data(rankE, valid_aux, DATA_NOC_3);
        in_E_noc3_valid = valid_aux;
        // receive yummy
        in_E_noc3_yummy = mpi_receive_yummy(rankE, YUMMY_NOC_3);
    end

    if(rankS != -1) begin
        //SENDING
        //$display("sending S");
        mpi_send_data(out_S_noc1_data, out_S_noc1_valid, rankS, rank, DATA_NOC_1);
        // send yummy
        mpi_send_yummy(out_S_noc1_yummy, rankS, rank, YUMMY_NOC_1);

        // send data
        mpi_send_data(out_S_noc2_data, out_S_noc2_valid, rankS, rank, DATA_NOC_2);
        // send yummy
        mpi_send_yummy(out_S_noc2_yummy, rankS, rank, YUMMY_NOC_2);

        // send data
        mpi_send_data(out_S_noc3_data, out_S_noc3_valid, rankS, rank, DATA_NOC_3);
        // send yummy
        mpi_send_yummy(out_S_noc3_yummy, rankS, rank, YUMMY_NOC_3);

        // RECEIVING
        //$display("receiving S");
        in_S_noc1_data = mpi_receive_data(rankS, valid_aux, DATA_NOC_1);
        in_S_noc1_valid = valid_aux;
        // receive yummy
        in_S_noc1_yummy = mpi_receive_yummy(rankS, YUMMY_NOC_1);
        
        in_S_noc2_data = mpi_receive_data(rankS, valid_aux, DATA_NOC_2);
        in_S_noc2_valid = valid_aux;
        // receive yummy
        in_S_noc2_yummy = mpi_receive_yummy(rankS, YUMMY_NOC_2);

        in_S_noc3_data = mpi_receive_data(rankS, valid_aux, DATA_NOC_3);
        in_S_noc3_valid = valid_aux;
        // receive yummy
        in_S_noc3_yummy = mpi_receive_yummy(rankS, YUMMY_NOC_3);
    end

    if(rankW != -1) begin
        //SENDING
        //$display("sending W");
        mpi_send_data(out_W_noc1_data, out_W_noc1_valid, rankW, rank, DATA_NOC_1);
        // send yummy
        mpi_send_yummy(out_W_noc1_yummy, rankW, rank, YUMMY_NOC_1);

        // send data
        mpi_send_data(out_W_noc2_data, out_W_noc2_valid, rankW, rank, DATA_NOC_2);
        // send yummy
        mpi_send_yummy(out_W_noc2_yummy, rankW, rank, YUMMY_NOC_2);

        // send data
        mpi_send_data(out_W_noc3_data, out_W_noc3_valid, rankW, rank, DATA_NOC_3);
        // send yummy
        mpi_send_yummy(out_W_noc3_yummy, rankW, rank, YUMMY_NOC_3);

        // RECEIVING
        //$display("receiving W");
        in_W_noc1_data = mpi_receive_data(rankW, valid_aux, DATA_NOC_1);
        in_W_noc1_valid = valid_aux;
        // receive yummy
        in_W_noc1_yummy = mpi_receive_yummy(rankW, YUMMY_NOC_1);
        
        in_W_noc2_data = mpi_receive_data(rankW, valid_aux, DATA_NOC_2);
        in_W_noc2_valid = valid_aux;
        // receive yummy
        in_W_noc2_yummy = mpi_receive_yummy(rankW, YUMMY_NOC_2);

        in_W_noc3_data = mpi_receive_data(rankW, valid_aux, DATA_NOC_3);
        in_W_noc3_valid = valid_aux;
        // receive yummy
        in_W_noc3_yummy = mpi_receive_yummy(rankW, YUMMY_NOC_3);
    end
    #500;
end

$display("Trace done: METRO_TILE_%d",rank);
finalize();
$finish;
end
`endif

`ifdef VERILATOR
`ifndef METRO_TILE
always @(posedge ok_iob) begin
    cmp_top.system.chipset.chipset_impl.ciop_fake_iob.ok_iob = 1'b1;
end
`endif // ifndef METRO_TILE
`endif

////////////////////////////////////////////////////////
// SYNTHESIZABLE TILE
///////////////////////////////////////////////////////


`ifdef PITON_PRONOC 
	wire  pronoc_reset = ~sys_rst_n;

	assign smart_max = SMART_MAX;
	assign pronoc_T1 = T1;
    assign pronoc_T2 = T2;
	
	wire [`DATA_WIDTH-1:0]   dataIn [2:0] [CONCENTRATION-1 : 0];  
	wire validIn [2:0] [CONCENTRATION-1 : 0];
	wire yummyIn [2:0] [CONCENTRATION-1 : 0];
        
	wire [`DATA_WIDTH-1:0]   dataOut [2:0] [CONCENTRATION-1 : 0];  
	wire validOut [2:0] [CONCENTRATION-1 : 0];
	wire yummyOut [2:0] [CONCENTRATION-1 : 0]; 
        
	
	smartflit_chanel_t  tile_chan_i [2:0] [CONCENTRATION-1 : 0];  
	smartflit_chanel_t  tile_chan_o [2:0] [CONCENTRATION-1 : 0];  

	smartflit_chanel_t router_chan_in  [2:0] [MAX_P-1 : 0];
	smartflit_chanel_t router_chan_out [2:0] [MAX_P-1 : 0];
	
	wire [RAw-1 : 0] current_r_addr;
	wire [RAw-1 : 0] current_r_addr_wire [2:0] [CONCENTRATION-1 : 0];
	
	
	genvar n,p,l;
	generate
    
	for(n=0;n<3;n++) begin: N_
	
        for(l=0;l<CONCENTRATION;l++) begin : L_
            pronoc_to_piton_wrapper 
            #(
                .NOC_NUM(n),
                .PORT_NUM(0),
                .FLATID_WIDTH(`JTAG_FLATID_WIDTH)
            )pr2pi
            (
                .default_chipid(default_chipid),
                .default_coreid_x(default_coreid_x[l]), 
                .default_coreid_y(default_coreid_y[l]),
                .flat_tileid(flat_tileid[l]),    
                .reset(pronoc_reset),
                .clk(core_ref_clk),
                .dataOut(dataIn[n][l]),
                .validOut(validIn[n][l]),
                .yummyOut(yummyIn[n][l]),
                .current_r_addr_o(current_r_addr_wire[n][l]),
                .chan_in(tile_chan_i [n][l])
            );    
            
            piton_to_pronoc_wrapper      
            #(
                .NOC_NUM(n),
                .CHIP_SET_PORT(CHIP_SET_PORT),
                .FLATID_WIDTH(`JTAG_FLATID_WIDTH)
            )pi2pr
            (
                .default_chipid (default_chipid),
                .default_coreid_x(default_coreid_x[l]),
                .default_coreid_y(default_coreid_y[l]),
                .flat_tileid(flat_tileid[l]),    
                .reset(pronoc_reset),
                .clk(core_ref_clk),
                .dataIn(dataOut[n][l]),
                .validIn(validOut[n][l]),
                .yummyIn(yummyOut[n][l]),
                .current_r_addr_i(current_r_addr ),
                .chan_out(tile_chan_o [n][l])
            );    
		end//  CONCENTRATION 
		
		
		router_top #(
		   .P               (MAX_P)
		) the_router (
			.current_r_id    (current_r_id),
			.current_r_addr  (current_r_addr), 
			.chan_in         (router_chan_in[n]), 
			.chan_out        (router_chan_out[n]), 
			.router_event    (),
			.clk             (core_ref_clk ), 
			.reset           (pronoc_reset )
		);
		
		//first tile localport connection
		assign router_chan_in[n][0]=	tile_chan_o[n][0];
		assign tile_chan_i[n][0] = router_chan_out[n][0];

        //other tiles localport connection
        for(l=1;l<CONCENTRATION;l++) begin : L_1
            assign router_chan_in[n][4+l]=	tile_chan_o[n][l];
		    assign tile_chan_i[n][l] = router_chan_out[n][4+l];
        end


		
		//always @(posedge core_ref_clk) begin 
					//if(router_chan_in[n][0].flit_chanel.flit_wr) $display("in ROUTER %d CHAN %d PORT %d out=%h",current_r_addr,n,0,router_chan_in[n][0].flit_chanel.flit);
					//if(router_chan_out[n][0].flit_chanel.flit_wr) $display("out ROUTER %d CHAN %d PORT %d out=%h",current_r_addr,n,0,router_chan_out[n][0].flit_chanel.flit);
					
		//end
			
		for(p=0;p<4;p++) begin: P_
				assign router_chan_in[n][p+1]= noc_chanel_in[p].smartflit_chanel[n];
				assign noc_chanel_out[p].smartflit_chanel[n] = router_chan_out[n][p+1];
				
				//always @(posedge core_ref_clk) begin 
					//if(router_chan_in[n][p+1].flit_chanel.flit_wr) $display("in ROUTER %d CHAN %d PORT %d out=%h",current_r_addr,n,p,router_chan_in[n][p+1].flit_chanel.flit);
					
					
					//if(router_chan_out[n][p+1].flit_chanel.flit_wr) $display("out ROUTER %d CHAN %d PORT %d out=%h",current_r_addr,n,p,router_chan_out[n][p+1].flit_chanel.flit);
					
					
				//end
				
				
		end
	end//for
	endgenerate
	

endp_addr_encoder #( .TOPOLOGY("MESH"), .T1(T1), .T2(T2), .T3(1), .EAw(EAw),  .NE(NE)) encode1 ( .id(current_r_id), .code(current_r_addr));
	
`else 

	assign smart_max = 0;
    assign pronoc_T1 = `PITON_X_TILES;
    assign pronoc_T2 = `PITON_Y_TILES;

`endif

wire [31:0] default_total_num_tiles;
assign default_total_num_tiles = `PITON_NUM_TILES;

// Generate tile instances
genvar ll;
generate
for(ll=0;ll<CONCENTRATION;ll++) begin : L_
    tile #(
    `ifdef PITON_ARIANE
        .TILE_TYPE(`ARIANE_RV64_TILE)
    `endif
    `ifdef PITON_LITE_TILE
        .TILE_TYPE(`RISCV_LITE_TILE)
    `endif
    `ifdef PITON_SARG
        .TILE_TYPE(`SARGANTANA_RV64_TILE)
    `endif   
    `ifdef PITON_LOX
        .TILE_TYPE(`LAGARTO_OX_RV64_TILE)
    `endif 
    ) the_tile (
        .clk                (core_ref_clk),
        .rst_n              (sys_rst_n),
        .clk_en             (clk_en & cpu_enable[ll]),
        .default_chipid     (default_chipid),    // the first chip
        .default_coreid_x   (default_coreid_x[ll]),
        .default_coreid_y   (default_coreid_y[ll]),
        .flat_tileid        (flat_tileid[ll]),
        .default_total_num_tiles(default_total_num_tiles),
    `ifdef PITON_ARIANE
        .debug_req_i        ( 1'b0 ),
        .unavailable_o      (      ),
        .timer_irq_i        ( 1'b0 ),
        .ipi_i              ( 1'b0 ),
        .irq_i              ( 1'b0 ),
    `endif
    `ifdef PITON_LAGARTO
        .debug_req_i        ( 1'b0 ),
        .unavailable_o      (      ),
        .timer_irq_i        ( 1'b0 ),
        .ipi_i              ( 1'b0 ),
        .irq_i              ( 1'b0 ),
        .pmu_sig_o (),
    `endif
        // ucb from tiles to jtag
        .tile_jtag_ucb_val   (      ),
        .tile_jtag_ucb_data  (      ),
        // ucb from jtag to tiles
        .jtag_tiles_ucb_val  ( 1'b0 ),
        .jtag_tiles_ucb_data ( 1'b0 ),

    `ifdef PITON_PRONOC 
                
        .dataIn1         (dataIn[0][ll]),
        .validIn1        (validIn[0][ll]),
        .yummyIn1        (yummyIn[0][ll]),
        
        .dataOut1        (dataOut[0][ll]),
        .validOut1       (validOut[0][ll]),
        .yummyOut1       (yummyOut[0][ll]), 
        
        .dataIn2         (dataIn[1][ll]),
        .validIn2        (validIn[1][ll]),
        .yummyIn2        (yummyIn[1][ll]),
        
        .dataOut2        (dataOut[1][ll]),
        .validOut2       (validOut[1][ll]),
        .yummyOut2       (yummyOut[1][ll]), 
        
        .dataIn3         (dataIn[2][ll]),
        .validIn3        (validIn[2][ll]),
        .yummyIn3        (yummyIn[2][ll]),
        
        .dataOut3        (dataOut[2][ll]),
        .validOut3       (validOut[2][ll]),
        .yummyOut3       (yummyOut[2][ll])   
        

    `else
        .dyn0_dataIn_N       ( noc_chanel_in [PITON_NORTH].data1), // ( in_N_noc1_data   ),
        .dyn0_dataIn_E       ( noc_chanel_in [PITON_EAST ].data1), // ( in_E_noc1_data   ),
        .dyn0_dataIn_W       ( noc_chanel_in [PITON_WEST ].data1), // ( in_W_noc1_data   ),
        .dyn0_dataIn_S       ( noc_chanel_in [PITON_SOUTH].data1), // ( in_S_noc1_data   ),
        .dyn0_validIn_N      ( noc_chanel_in [PITON_NORTH].valid[0]), // ( in_N_noc1_valid  ),
        .dyn0_validIn_E      ( noc_chanel_in [PITON_EAST ].valid[0]), // ( in_E_noc1_valid  ),
        .dyn0_validIn_W      ( noc_chanel_in [PITON_WEST ].valid[0]), // ( in_W_noc1_valid  ),
        .dyn0_validIn_S      ( noc_chanel_in [PITON_SOUTH].valid[0]), // ( in_S_noc1_valid  ),
        .dyn0_dNo_yummy      ( noc_chanel_in [PITON_NORTH].yummy[0]), // ( in_N_noc1_yummy  ),           
        .dyn0_dEo_yummy      ( noc_chanel_in [PITON_EAST ].yummy[0]), // ( in_E_noc1_yummy  ),
        .dyn0_dWo_yummy      ( noc_chanel_in [PITON_WEST ].yummy[0]), // ( in_W_noc1_yummy  ),
        .dyn0_dSo_yummy      ( noc_chanel_in [PITON_SOUTH].yummy[0]), // ( in_S_noc1_yummy  ),
                                            
        .dyn0_dNo            ( noc_chanel_out[PITON_NORTH].data1), //( out_N_noc1_data  ),
        .dyn0_dEo            ( noc_chanel_out[PITON_EAST ].data1), //( out_E_noc1_data  ),
        .dyn0_dWo            ( noc_chanel_out[PITON_WEST ].data1), //( out_W_noc1_data  ),
        .dyn0_dSo            ( noc_chanel_out[PITON_SOUTH].data1), //( out_S_noc1_data  ),
        .dyn0_dNo_valid      ( noc_chanel_out[PITON_NORTH].valid[0]), //( out_N_noc1_valid ),
        .dyn0_dEo_valid      ( noc_chanel_out[PITON_EAST ].valid[0]), //( out_E_noc1_valid ),
        .dyn0_dWo_valid      ( noc_chanel_out[PITON_WEST ].valid[0]), //( out_W_noc1_valid ),
        .dyn0_dSo_valid      ( noc_chanel_out[PITON_SOUTH].valid[0]), //( out_S_noc1_valid ),
        .dyn0_yummyOut_N     ( noc_chanel_out[PITON_NORTH].yummy[0]), //( out_N_noc1_yummy ),
        .dyn0_yummyOut_E     ( noc_chanel_out[PITON_EAST ].yummy[0]), //( out_E_noc1_yummy ),
        .dyn0_yummyOut_W     ( noc_chanel_out[PITON_WEST ].yummy[0]), //( out_W_noc1_yummy ),
        .dyn0_yummyOut_S     ( noc_chanel_out[PITON_SOUTH].yummy[0]), //( out_S_noc1_yummy ),
                                            
        .dyn1_dataIn_N       ( noc_chanel_in [PITON_NORTH].data2), // ( in_N_noc2_data   ),
        .dyn1_dataIn_E       ( noc_chanel_in [PITON_EAST ].data2), // ( in_E_noc2_data   ),
        .dyn1_dataIn_W       ( noc_chanel_in [PITON_WEST ].data2), // ( in_W_noc2_data   ),
        .dyn1_dataIn_S       ( noc_chanel_in [PITON_SOUTH].data2), // ( in_S_noc2_data   ),
        .dyn1_validIn_N      ( noc_chanel_in [PITON_NORTH].valid[1]), // ( in_N_noc2_valid  ),
        .dyn1_validIn_E      ( noc_chanel_in [PITON_EAST ].valid[1]), // ( in_E_noc2_valid  ),
        .dyn1_validIn_W      ( noc_chanel_in [PITON_WEST ].valid[1]), // ( in_W_noc2_valid  ),
        .dyn1_validIn_S      ( noc_chanel_in [PITON_SOUTH].valid[1]), // ( in_S_noc2_valid  ),
        .dyn1_dNo_yummy      ( noc_chanel_in [PITON_NORTH].yummy[1]), // ( in_N_noc2_yummy  ),
        .dyn1_dEo_yummy      ( noc_chanel_in [PITON_EAST ].yummy[1]), // ( in_E_noc2_yummy  ),
        .dyn1_dWo_yummy      ( noc_chanel_in [PITON_WEST ].yummy[1]), // ( in_W_noc2_yummy  ),
        .dyn1_dSo_yummy      ( noc_chanel_in [PITON_SOUTH].yummy[1]), // ( in_S_noc2_yummy  ),
                                        
        .dyn1_dNo            ( noc_chanel_out[PITON_NORTH].data2), //( out_N_noc2_data  ),
        .dyn1_dEo            ( noc_chanel_out[PITON_EAST ].data2), //( out_E_noc2_data  ),
        .dyn1_dWo            ( noc_chanel_out[PITON_WEST ].data2), //( out_W_noc2_data  ),
        .dyn1_dSo            ( noc_chanel_out[PITON_SOUTH].data2), //( out_S_noc2_data  ),
        .dyn1_dNo_valid      ( noc_chanel_out[PITON_NORTH].valid[1]), //( out_N_noc2_valid ),
        .dyn1_dEo_valid      ( noc_chanel_out[PITON_EAST ].valid[1]), //( out_E_noc2_valid ),
        .dyn1_dWo_valid      ( noc_chanel_out[PITON_WEST ].valid[1]), //( out_W_noc2_valid ),
        .dyn1_dSo_valid      ( noc_chanel_out[PITON_SOUTH].valid[1]), //( out_S_noc2_valid ),
        .dyn1_yummyOut_N     ( noc_chanel_out[PITON_NORTH].yummy[1]), //( out_N_noc2_yummy ),
        .dyn1_yummyOut_E     ( noc_chanel_out[PITON_EAST ].yummy[1]), //( out_E_noc2_yummy ),
        .dyn1_yummyOut_W     ( noc_chanel_out[PITON_WEST ].yummy[1]), //( out_W_noc2_yummy ),
        .dyn1_yummyOut_S     ( noc_chanel_out[PITON_SOUTH].yummy[1]), //( out_S_noc2_yummy ),
                                            
        .dyn2_dataIn_N       ( noc_chanel_in [PITON_NORTH].data3), // ( in_N_noc3_data   ),
        .dyn2_dataIn_E       ( noc_chanel_in [PITON_EAST ].data3), // ( in_E_noc3_data   ),
        .dyn2_dataIn_W       ( noc_chanel_in [PITON_WEST ].data3), // ( in_W_noc3_data   ),
        .dyn2_dataIn_S       ( noc_chanel_in [PITON_SOUTH].data3), // ( in_S_noc3_data   ),
        .dyn2_validIn_N      ( noc_chanel_in [PITON_NORTH].valid[2]), // ( in_N_noc3_valid  ),
        .dyn2_validIn_E      ( noc_chanel_in [PITON_EAST ].valid[2]), // ( in_E_noc3_valid  ),
        .dyn2_validIn_W      ( noc_chanel_in [PITON_WEST ].valid[2]), // ( in_W_noc3_valid  ),
        .dyn2_validIn_S      ( noc_chanel_in [PITON_SOUTH].valid[2]), // ( in_S_noc3_valid  ),
        .dyn2_dNo_yummy      ( noc_chanel_in [PITON_NORTH].yummy[2]), // ( in_N_noc3_yummy  ),
        .dyn2_dEo_yummy      ( noc_chanel_in [PITON_EAST ].yummy[2]), // ( in_E_noc3_yummy  ),
        .dyn2_dWo_yummy      ( noc_chanel_in [PITON_WEST ].yummy[2]), // ( in_W_noc3_yummy  ),
        .dyn2_dSo_yummy      ( noc_chanel_in [PITON_SOUTH].yummy[2]), // ( in_S_noc3_yummy  ),
                                        
        .dyn2_dNo            ( noc_chanel_out[PITON_NORTH].data3), //( out_N_noc3_data  ),
        .dyn2_dEo            ( noc_chanel_out[PITON_EAST ].data3), //( out_E_noc3_data  ),
        .dyn2_dWo            ( noc_chanel_out[PITON_WEST ].data3), //( out_W_noc3_data  ),
        .dyn2_dSo            ( noc_chanel_out[PITON_SOUTH].data3), //( out_S_noc3_data  ),
        .dyn2_dNo_valid      ( noc_chanel_out[PITON_NORTH].valid[2]), //( out_N_noc3_valid ),
        .dyn2_dEo_valid      ( noc_chanel_out[PITON_EAST ].valid[2]), //( out_E_noc3_valid ),
        .dyn2_dWo_valid      ( noc_chanel_out[PITON_WEST ].valid[2]), //( out_W_noc3_valid ),
        .dyn2_dSo_valid      ( noc_chanel_out[PITON_SOUTH].valid[2]), //( out_S_noc3_valid ),
        .dyn2_yummyOut_N     ( noc_chanel_out[PITON_NORTH].yummy[2]), //( out_N_noc3_yummy ),
        .dyn2_yummyOut_E     ( noc_chanel_out[PITON_EAST ].yummy[2]), //( out_E_noc3_yummy ),
        .dyn2_yummyOut_W     ( noc_chanel_out[PITON_WEST ].yummy[2]), //( out_W_noc3_yummy ),
        .dyn2_yummyOut_S     ( noc_chanel_out[PITON_SOUTH].yummy[2])  //( out_S_noc3_yummy )
        `endif
    );
end// concentration
endgenerate

/*
integer i,j;
always @( posedge core_ref_clk )begin
	for(i=0;i<P;i++) begin 
		for(j=0;j<3;j++) begin 
			if(noc_chanel_in[i].valid[j] )  $display("noc%d_dat_in[%d]=%h",j,i,noc_chanel_in [i].data[j]); 
			if(noc_chanel_out[i].valid[j] )  $display("noc%d_dat_out[%d]=%h",j,i,noc_chanel_out [i].data[j]); 
		end
	end

end
*/

////////////////////////////////////////////////////////
// MONITOR STUFF
////////////////////////////////////////////////////////


`ifndef DISABLE_ALL_MONITORS

    // this is the T1 sparc core monitor
    monitor   monitor(
        .clk    (`CHIP_INT_CLK),
        .cmp_gclk  (`CHIP_INT_CLK),
        .rst_l     (sys_rst_n)
        );

`ifndef MINIMAL_MONITORING
    //integer j;

    // Tri: slam init is taken out because it's too complicated to extend to 64 cores
    // slam_init slam_init () ;

    // The only thing that we will "slam init" is the integer register file
    //  and it is randomized. For some reason if we left it as X's some tests will fail

`ifndef METRO_TILE
`ifndef VERILATOR
    // T1's TSO monitor, stripped of all L2 references
    tso_mon tso_mon(`CHIP_INT_CLK, `CHIP.rst_n_inter_sync);
`endif
`endif //METRO_TILE

    // L15 MONITORS
    cmp_l15_messages_mon l15_messages_mon(
        .clk (`CHIP_INT_CLK)
        );

    // DMBR MONITOR
    dmbr_mon dmbr_mon (
        .clk(`CHIP_INT_CLK)
     );

    //L2 MONITORS
    `ifdef FAKE_L2
    `else
    l2_mon l2_mon(
        .clk (`CHIP_INT_CLK)
    );
    `endif

    //only works if clk == chipset_clk
    //async_fifo_mon async_fifo_mon(
    //   .clk (core_ref_clk)
    //);

`ifndef METRO_TILE
    jtag_mon jtag_mon(
        .clk (jtag_clk)
        );

    iob_mon iob_mon(
        .clk (chipset_clk)
    );
    // sas, more debug info
`endif // ifndef METRO_TILE

    // turn on sas interface after a delay
//    reg   need_sas_sparc_intf_update;
//    initial begin
//        need_sas_sparc_intf_update  = 0;
//        #12500;
//        need_sas_sparc_intf_update  = 1;
//    end // initial begin

`ifdef PITON_OST1
    sas_intf  sas_intf(/*AUTOINST*/
        // Inputs
        .clk       (`CHIP_INT_CLK),      // Templated
        .rst_l     (`CHIP.rst_n_inter_sync));       // Templated
`endif

`ifdef PITON_OST1
    // create sas tasks
    sas_tasks sas_tasks(/*AUTOINST*/
        // Inputs
        .clk      (`CHIP_INT_CLK),      // Templated
        .rst_l        (`CHIP.rst_n_inter_sync));       // Templated
`endif

`ifdef PITON_OST1
    // sparc pipe flow monitor
    sparc_pipe_flow sparc_pipe_flow(/*AUTOINST*/
        // Inputs
        .clk  (`CHIP_INT_CLK));         // Templated
`endif

`ifndef METRO_TILE
    manycore_network_mon network_mon (`CHIP_INT_CLK);
`endif // ifndef METRO_TILE

`endif // MINIMAL_MONITORING
`endif // DISABLE_ALL_MONITORS
    // Alexey
    // UART monitor
    /*reg      prev_tx_state;
    always @(posedge core_ref_clk)
        prev_tx_state <= tx;

    always @(posedge core_ref_clk)
        if (prev_tx_state != tx) begin
            $display("UART: TX changed to %d at", tx, $time);
        end*/

endmodule // cmp_top

`endif


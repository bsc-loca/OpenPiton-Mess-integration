`include "sys.h"
`include "iop.h"
`include "cross_module.tmp.h"
`include "ifu.tmp.h"
`include "define.tmp.h"
`include "piton_system.vh"
`include "jtag.vh"

`ifdef PITON_DPI
import "DPI-C" function longint read_64b_call (input longint addr);
import "DPI-C" function void write_64b_call (input longint addr, input longint data);
import "DPI-C" function int drive_iob ();
import "DPI-C" function int get_cpx_word (int index);
import "DPI-C" function void report_pc (longint thread_pc);
import "DPI-C" function void init_jbus_model_call(string str, int oram);
`endif


`timescale 1ps/1ps
module metro_fake_mem (
	core_ref_clk,
	sys_rst_n,
	noc_chanel_in	,
	noc_chanel_out	,
    smart_max,
    mem_valid_out,
    mem_valid_in,
    flit_o_cnts,
    flit_i_cnts,
	default_chipid,
	default_coreid_x,
	default_coreid_y,
	flat_tileid
`ifdef  PITON_LAT_FILE
`ifdef  METRO_CHIPSET
`ifdef  PITON_EXTRA_MEMS
	,
	got_rd,
	got_wr,
	rd_lat_in
`endif 
`endif 
`endif
);


`ifdef PITON_PRONOC
	`include "pronoc_def.v"
 	`NOC_CONF
	
	typedef struct packed {	
		smartflit_chanel_t  [2:0] smartflit_chanel;  		
	} noc_chanel_t;

`else

	typedef struct packed {	
		logic  [`NOC_DATA_WIDTH-1:0] data1;
		logic  [`NOC_DATA_WIDTH-1:0] data2;
		logic  [`NOC_DATA_WIDTH-1:0] data3;
		logic  [2:0] valid;
		logic  [2:0] yummy;		
	} noc_chanel_t;

`endif

localparam NOC_CHANEL_w = $bits(noc_chanel_t); 	




input                              core_ref_clk;
input                              sys_rst_n;
input  noc_chanel_t noc_chanel_in;
output noc_chanel_t noc_chanel_out;
output [31: 0 ] smart_max;
output [63 : 0] flit_o_cnts,flit_i_cnts;

output mem_valid_out;
output mem_valid_in;

input wire [13:0]                     default_chipid  ;
input wire [7:0]                      default_coreid_x;
input wire [7:0]                      default_coreid_y;
input wire [`JTAG_FLATID_WIDTH-1:0]   flat_tileid;

`ifdef  PITON_LAT_FILE
`ifdef  METRO_CHIPSET
`ifdef  PITON_EXTRA_MEMS
	output 	got_rd,	got_wr;
	input [31 : 0] rd_lat_in;
`endif 
`endif 
`endif

    wire  [`NOC_DATA_WIDTH -1:0] processor_mcx_noc2_data ; //input
    wire                         processor_mcx_noc2_valid; //input
    wire                         processor_mcx_noc2_yummy; //output

    wire  [`NOC_DATA_WIDTH -1:0] mcx_processor_noc3_data; //output
    wire                         mcx_processor_noc3_valid;//output
    wire                         mcx_processor_noc3_yummy;//input
    
    wire  [`NOC_DATA_WIDTH -1:0] intf_mcx_data_noc2; //input
    wire                         intf_mcx_val_noc2; //input
    wire                         intf_mcx_rdy_noc2; //output

    wire  [`NOC_DATA_WIDTH -1:0] mcx_intf_data_noc3; //output
    wire                         mcx_intf_val_noc3;  //output
    wire                         mcx_intf_rdy_noc3;  //input 



wire                            processor_offchip_noc1_valid;
wire [`NOC_DATA_WIDTH-1:0]    processor_offchip_noc1_data;
wire                            processor_offchip_noc1_yummy;
wire                            processor_offchip_noc2_valid;
wire [`NOC_DATA_WIDTH-1:0]    processor_offchip_noc2_data;
wire                            processor_offchip_noc2_yummy;
wire                            processor_offchip_noc3_valid;
wire [`NOC_DATA_WIDTH-1:0]    processor_offchip_noc3_data;
wire                            processor_offchip_noc3_yummy;

wire                            offchip_processor_noc1_valid;
wire [`NOC_DATA_WIDTH-1:0]    offchip_processor_noc1_data;
wire                            offchip_processor_noc1_yummy;
wire                            offchip_processor_noc2_valid;
wire [`NOC_DATA_WIDTH-1:0]    offchip_processor_noc2_data;
wire                            offchip_processor_noc2_yummy;
wire                            offchip_processor_noc3_valid;
wire [`NOC_DATA_WIDTH-1:0]    offchip_processor_noc3_data;
wire                            offchip_processor_noc3_yummy;


`ifdef PITON_PRONOC

    assign smart_max = SMART_MAX;
	wire pronoc_reset = ~ sys_rst_n;

    localparam CHIP_SET_ID = T1*T2*T3+2*T1; // endp connected  of west port of router 0-0
    localparam CHIP_SET_PORT = 3; //west port of first router

    //NOC2
     wire [RAw-1 : 0] tile_0_0_current_r_addr2;
    
     piton_to_pronoc_wrapper #(.FLATID_WIDTH(`JTAG_FLATID_WIDTH),.NOC_NUM(2),.CHIP_SET_PORT(CHIP_SET_PORT)) pi2pr_wrapper2
	(
	.default_chipid(default_chipid), .default_coreid_x(default_coreid_x), .default_coreid_y(default_coreid_y), .flat_tileid(flat_tileid),	
	.reset(pronoc_reset),
	.clk (core_ref_clk),
	.dataIn({`NOC_DATA_WIDTH{1'b0}} ),
	.validIn(1'b0),
	.yummyIn(processor_mcx_noc2_yummy),
	.chan_out(noc_chanel_out.smartflit_chanel[1]),
	.current_r_addr_i(tile_0_0_current_r_addr2)
	);	

	pronoc_to_piton_wrapper  #(.FLATID_WIDTH(`JTAG_FLATID_WIDTH),.NOC_NUM(2),.PORT_NUM(CHIP_SET_PORT)) pr2pi_wrapper2
	(
	.default_chipid(default_chipid), .default_coreid_x(default_coreid_x), .default_coreid_y(default_coreid_y), .flat_tileid(flat_tileid),		
	.reset(pronoc_reset),
	.clk (core_ref_clk),
	.dataOut(processor_mcx_noc2_data),
	.validOut(processor_mcx_noc2_valid),
	.yummyOut( ),
	.chan_in(noc_chanel_in.smartflit_chanel[1]),
	.current_r_addr_o(tile_0_0_current_r_addr2)
	);	


       //NOC3

	wire [RAw-1 : 0] tile_0_0_current_r_addr3;

	piton_to_pronoc_wrapper #(.FLATID_WIDTH(`JTAG_FLATID_WIDTH),.NOC_NUM(3),.CHIP_SET_PORT(CHIP_SET_PORT))  pi2pr_wrapper3
	(
		.default_chipid(default_chipid), .default_coreid_x(default_coreid_x), .default_coreid_y(default_coreid_y), .flat_tileid(flat_tileid),	
		.reset(pronoc_reset),
		.clk (core_ref_clk),
		.dataIn(mcx_processor_noc3_data),
		.validIn(mcx_processor_noc3_valid),
		.yummyIn(1'b0),
		.chan_out(noc_chanel_out.smartflit_chanel[2]),
		.current_r_addr_i(tile_0_0_current_r_addr3)
	);	

	pronoc_to_piton_wrapper  #(.FLATID_WIDTH(`JTAG_FLATID_WIDTH),.NOC_NUM(3),.PORT_NUM(CHIP_SET_PORT)) pr2pi_wrapper3
	(
		.default_chipid(default_chipid), .default_coreid_x(default_coreid_x), .default_coreid_y(default_coreid_y), .flat_tileid(flat_tileid),	
		.reset(pronoc_reset),
		.clk (core_ref_clk),
		.dataOut( ),
		.validOut( ),
		.yummyOut(mcx_processor_noc3_yummy),
		.chan_in(noc_chanel_in.smartflit_chanel[2]),
		.current_r_addr_o(tile_0_0_current_r_addr3)
	);	





`else

    assign smart_max = 0;
    assign processor_mcx_noc2_data = noc_chanel_in.data2;
    assign processor_mcx_noc2_valid= noc_chanel_in.valid [1];
    assign noc_chanel_out.yummy[1] =  processor_mcx_noc2_yummy;

    assign noc_chanel_out.data3 = mcx_processor_noc3_data;  
    assign noc_chanel_out.valid  [2] = mcx_processor_noc3_valid;
    assign mcx_processor_noc3_yummy = noc_chanel_in.yummy[2];

`endif

valrdy_to_credit #(
	.BUFFER_SIZE(4),
    .BUFFER_BITS(3)
) mc_processor_noc3_v2c( 
    .clk      (core_ref_clk), 
    .reset    (~sys_rst_n),  

    .data_in (mcx_intf_data_noc3), 
    .valid_in(mcx_intf_val_noc3 ), 
    .ready_in(mcx_intf_rdy_noc3 ), 

    .data_out (mcx_processor_noc3_data ), 
    .valid_out(mcx_processor_noc3_valid), 
    .yummy_out(mcx_processor_noc3_yummy) 
); 

credit_to_valrdy  processor_mc_noc2_c2v( 
    .clk      (core_ref_clk), 
    .reset    (~sys_rst_n),  

    .data_in (processor_mcx_noc2_data ), 
    .valid_in(processor_mcx_noc2_valid), 
    .yummy_in(processor_mcx_noc2_yummy),

    .data_out (intf_mcx_data_noc2), 
    .valid_out(intf_mcx_val_noc2 ), 
    .ready_out(intf_mcx_rdy_noc2 ) 
);



// Fake Memory Controller
fake_mem_ctrl fake_mc  ( 
    .clk      (core_ref_clk), 
    .rst_n     (sys_rst_n),  
    .noc_valid_in       ( intf_mcx_val_noc2  ), 
    .noc_data_in        ( intf_mcx_data_noc2 ), 
    .noc_ready_in       ( intf_mcx_rdy_noc2  ), 
    .noc_valid_out      ( mcx_intf_val_noc3  ), 
    .noc_data_out       ( mcx_intf_data_noc3 ), 
    .noc_ready_out      ( mcx_intf_rdy_noc3  ) 
`ifdef  PITON_LAT_FILE
`ifdef  METRO_CHIPSET
`ifdef  PITON_EXTRA_MEMS
	,.rd_lat_in(rd_lat_in)
`endif 
`endif 
`endif
);


assign mem_valid_out = mcx_intf_val_noc3 & mcx_intf_rdy_noc3;
assign mem_valid_in  = intf_mcx_val_noc2 & intf_mcx_rdy_noc2;

assign flit_o_cnts=fake_mc.flit_o_cnts;
assign flit_i_cnts=fake_mc.flit_i_cnts;


`ifdef  PITON_LAT_FILE
`ifdef  METRO_CHIPSET
`ifdef  PITON_EXTRA_MEMS
	assign got_rd = fake_mc.got_rd;
	assign got_wr = fake_mc.got_wr;
`endif 
`endif 
`endif

endmodule // cmp_top



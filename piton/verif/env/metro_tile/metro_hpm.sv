module metro_hpm (
    flat_tileid,
    clk,
    rst_n,
    hpm_st,
    cache_st,
    flit_st,
    pck_st,
    roi_start,// RIO (resion of interest) started by reading instruction count in csr
    roi_en,    // ROI Enabled until reading instruction count in csr again.
    inst_done, //for checking trap
    phy_pc_w
);

input clk,rst_n;
output roi_start,  roi_en;
input wire [7:0]   flat_tileid;
output [31 : 0] cache_st [11: 0];
output reg [63 : 0] flit_st  [0: 5]; // count flit in/out to/from processor for 3 NoCs 
output reg [63 : 0] pck_st   [0: 5][0:11]; // packet sizes histogram in/out to/from processor for 3 NoCs 
output inst_done;
output [63:0] phy_pc_w;

//HPM EVENTS
localparam 
  HPM_BRANCH_MISS=0,
  HPM_IS_BRANCH=1,
  HPM_BRANCH_TAKEN=2,
  HPM_EXE_STORE=3,
  HPM_EXE_LOAD=4,
  HPM_ICACHE_REQ=5,
  HPM_ICACHE_KILL=6,
  HPM_STALL_IF=7,
  HPM_STALL_ID=8,
  HPM_STALL_RR=9,
  HPM_STALL_EXE=10,
  HPM_STALL_WB=11,
  HPM_ICACHE_MISS_L2_HIT=12,
  HPM_ICACHE_MISS_KILL=13,
  HPM_ICACHE_BUSY=14,
  HPM_ICACHE_MISS_TIME=15,
  HPM_LOAD_STORE=16,
  HPM_DATA_DEPEND=17,
  HPM_STRUCT_DEPEND=18,
  HPM_GRAD_LIST_FULL=19,
  HPM_FREE_LIST_EMPTY=20,
  HPM_ITLB_ACCESS=21,
  HPM_ITLB_MISS=22,
  HPM_DTLB_ACCESS=23,
  HPM_DTLB_MISS=24,
  HPM_PTW_BUFFER_HIT=25,
  HPM_PTW_BUFFER_MISS=26,
  HPM_ITLB_STALL=27,
  HPM_DCACHE_STALL=28,
  HPM_DCACHE_STALL_REFILL=29,
  HPM_DCACHE_RTAB_ROLLBACK=30,
  HPM_DCACHE_REQ_ONHOLD=31,
  HPM_DCACHE_PREFETCH_REQ=32,
  HPM_DCACHE_READ_REQ=33,
  HPM_DCACHE_WRITE_REQ=34,
  HPM_DCACHE_CMO_REQ=35,
  HPM_DCACHE_UNCACHED_REQ=36,
  HPM_DCACHE_MISS_READ_REQ=37,
  HPM_DCACHE_MISS_WRITE_REQ=38,
  HPM_STALL_IR=39,
  HPM_L2_MISS=40,
  HPM_L2_ACCESS=41,
  HPM_L15_MISS=42,
  HPM_L15_ACCESS=43,
  HPM_CNT_NUM = 44;

 output reg [31 : 0] hpm_st [HPM_CNT_NUM-1 : 0];
 logic  [HPM_CNT_NUM-1 : 0] hpm_st_incr;
 reg  hpm_en;
 wire hpm_reset;


wire csr_read;
wire rd_instruction;



`ifdef PITON_ARIANE
  import ariane_pkg::*;
  assign csr_read = `TOP_MOD_INST.g_ariane_core.core.ariane.i_cva6.csr_regfile_i.csr_read;
  assign rd_instruction = (`TOP_MOD_INST.g_ariane_core.core.ariane.i_cva6.csr_regfile_i.csr_addr.address == riscv::CSR_MINSTRET);
  always @(*) begin 
    hpm_st_incr = 'h0;
    `ifdef EXTERNAL_HPM_EVENT_NUM
    hpm_st_incr [HPM_L15_ACCESS]= `TOP_MOD_INST.hpm_l15_access & hpm_en;
    hpm_st_incr [HPM_L15_MISS]  = `TOP_MOD_INST.hpm_l15_miss & hpm_en;
    hpm_st_incr [HPM_L2_ACCESS] = `TOP_MOD_INST.hpm_l2_access & hpm_en;
    hpm_st_incr [HPM_L2_MISS]   = `TOP_MOD_INST.hpm_l2_miss & hpm_en;
    `endif
  end
`endif

`ifdef PITON_SARG
  import riscv_pkg::*;
  assign csr_read = `TOP_MOD_INST.g_sarg_core.core.core_inst.sargantana_inst.csr_inst.csr_read;
  assign rd_instruction = (`TOP_MOD_INST.g_sarg_core.core.core_inst.sargantana_inst.csr_inst.csr_addr.address == riscv_pkg::CSR_MINSTRET);
  integer i;
  always @(*) begin 
    for(i=0;i<HPM_CNT_NUM;i++) hpm_st_incr[i] = `TOP_MOD_INST.g_sarg_core.core.core_inst.sargantana_inst.hpm_events_d[i+1] & hpm_en;
  end
`endif

always @ (posedge clk)begin
    if(!rst_n) begin 
        hpm_en<=1'b0;
    end
    else if(csr_read & rd_instruction ) begin   
      if(flat_tileid=='h0) begin     
         if(hpm_en==1'b0 ) $display("**********START OF ROI*************** ");
         else      $display("***********END OF ROI**************** ");
      end
      hpm_en<=!hpm_en;
    end
end

/****************
    cache_st
*****************/

`define PATH1 `TOP_MOD_INST.l15.l15.dtag.sram_l15_tag.st
`define PATH2 `TOP_MOD_INST.l15.l15.dcache.sram_l15_data.st
`define PATH3 `TOP_MOD_INST.l2.tag_wrap.l2_tag.l2_tag_array.sram_l2_tag.st 
`define PATH4 `TOP_MOD_INST.l2.data_wrap.l2_data.l2_data_array.sram_l2_data.st 

wire [63 : 0]   lat_sum , req_num;

    
piton_lat_monitor #(
    .REQ_FLIT_WIDTH(`NOC_DATA_WIDTH),
    .RSP_FLIT_WIDTH(`NOC_DATA_WIDTH)
)lat_mon(
    .id (`TOP_MOD_INST.flat_tileid),    
    .req_valid(`TOP_MOD_INST.processor_router_valid_noc1),
    .req_flit_in(`TOP_MOD_INST.processor_router_data_noc1),
    .req_ready(`TOP_MOD_INST.router_processor_ready_noc1),
    .rsp_valid(`TOP_MOD_INST.buffer_processor_valid_noc2),
    .rsp_flit_in(`TOP_MOD_INST.buffer_processor_data_noc2),
    .rsp_ready(`TOP_MOD_INST.processor_router_ready_noc2),
    .lat_sum (lat_sum), 
    .req_num (req_num),
    .reset(!rst_n),
    .clk(clk)
);

assign  cache_st [0] = `PATH1.sum_wr;
assign  cache_st [1] = `PATH1.total;
assign  cache_st [2] = `PATH2.sum_wr;
assign  cache_st [3] = `PATH2.total;
assign  cache_st [4] = `PATH3.sum_wr;
assign  cache_st [5] = `PATH3.total;
assign  cache_st [6] = `PATH4.sum_wr;
assign  cache_st [7] = `PATH4.total;  
assign  cache_st [8] = hpm_st[HPM_L2_ACCESS];
assign  cache_st [9] = hpm_st[HPM_L2_MISS];
assign  cache_st [10] = lat_sum [31 : 0];
assign  cache_st [11] = req_num [31 : 0];


/**********************
   flit_st  & pck_st
**********************/
integer cnt,siz;

wire [5: 0] is_header;
wire [`MSG_LENGTH_WIDTH-1       :0] length [0: 5];

wire [5: 0] flit_in_valid;
wire [`NOC_DATA_WIDTH-1 : 0] data_1;
wire [`NOC_DATA_WIDTH-1 : 0] data_2;
wire [`NOC_DATA_WIDTH-1 : 0] data_3;
wire [`NOC_DATA_WIDTH-1 : 0] data_4;
wire [`NOC_DATA_WIDTH-1 : 0] data_5;
wire [`NOC_DATA_WIDTH-1 : 0] data_6;

 
assign  flit_in_valid[0] =`TOP_MOD_INST.router_buffer_data_val_noc1;
assign  flit_in_valid[1] =`TOP_MOD_INST.router_buffer_data_val_noc2;
assign  flit_in_valid[2] =`TOP_MOD_INST.router_buffer_data_val_noc3;
assign  flit_in_valid[3] =`TOP_MOD_INST.buffer_router_valid_noc1;
assign  flit_in_valid[4] =`TOP_MOD_INST.buffer_router_valid_noc2;
assign  flit_in_valid[5] =`TOP_MOD_INST.buffer_router_valid_noc3;

assign  data_1 = `TOP_MOD_INST.router_buffer_data_noc1;
assign  data_2 = `TOP_MOD_INST.router_buffer_data_noc2;
assign  data_3 = `TOP_MOD_INST.router_buffer_data_noc3;
assign  data_4 = `TOP_MOD_INST.buffer_router_data_noc1;
assign  data_5 = `TOP_MOD_INST.buffer_router_data_noc2;
assign  data_6 = `TOP_MOD_INST.buffer_router_data_noc3;

hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d0(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_1),
    .valid(flit_in_valid[0]),
    .ready(1'b1),
    .is_header(is_header[0]),
    .length(length[0])
);

hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d1(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_2),
    .valid(flit_in_valid[1]),
    .ready(1'b1),
    .is_header(is_header[1]),
    .length(length[1])
);
hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d2(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_3),
    .valid(flit_in_valid[2]),
    .ready(1'b1),
    .is_header(is_header[2]),
    .length(length[2])
);
hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d3(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_4),
    .valid(flit_in_valid[3]),
    .ready(1'b1),
    .is_header(is_header[3]),
    .length(length[3])
);

hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d4(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_5),
    .valid(flit_in_valid[4]),
    .ready(1'b1),
    .is_header(is_header[4]),
    .length(length[4])
);
hdr_pck_size_detect #(
    .FLIT_WIDTH(`NOC_DATA_WIDTH)
)d5(
    .reset(!rst_n),
    .clk(clk),
    .flit_in(data_6),
    .valid(flit_in_valid[5]),
    .ready(1'b1),
    .is_header(is_header[5]),
    .length(length[5])
);


always @ (posedge clk)begin 
  if(!rst_n) begin 
    for(cnt=0;cnt<6;cnt++) begin 
      flit_st[cnt] <=64'd0;  
      for(siz=0;siz<12;siz++) pck_st [cnt][siz] <=64'd0;          
    end
  end else begin 
    if(roi_start)begin 
      //$display("*****RESET FLIT COUNTERS !***************");
      for(cnt=0;cnt<6;cnt++) begin 
        flit_st[cnt] <=64'd0;
        for(siz=0;siz<12;siz++) pck_st [cnt][siz] <=64'd0;        
      end
    end else if(roi_en) begin   
      for(cnt=0;cnt<6;cnt++) begin
        if(flit_in_valid[cnt])begin 
          flit_st[cnt]<=flit_st[cnt]+1;
          if(is_header[cnt] )  pck_st [cnt][length[cnt][3:0]] <= pck_st [cnt][length[cnt][3:0]]+1;
        end
      end   
    end
  end
end

/**********************
  hpm_st
**********************/


always @ (posedge clk)begin 
    if(!rst_n) begin 
        for(cnt=0;cnt<HPM_CNT_NUM;cnt++) begin
            hpm_st[cnt] <=32'd0;
        end
    end else begin 
        for(cnt=0;cnt<HPM_CNT_NUM;cnt++) begin 
            if(hpm_reset) hpm_st[cnt] <=32'd0;  
            else if(hpm_st_incr[cnt]) hpm_st[cnt] <= hpm_st[cnt] +1'b1;
        end
    end
end

assign hpm_reset = (csr_read & rd_instruction) && (hpm_en == 1'b0);
assign roi_en = hpm_en;
assign roi_start = hpm_reset;

/**********************
     Check Traps
**********************/
reg spc0_0_inst_done;
reg [63:0] spc0_0_phy_pc_w;
always @ (posedge clk)begin 
    if(!rst_n) begin 
        spc0_0_inst_done         <= 0;
        spc0_0_phy_pc_w          <= 0;
    end else begin
        `ifdef RTL_ARIANE0          
            spc0_0_inst_done         <= `ARIANE_CORE0.piton_pc_vld;
            spc0_0_phy_pc_w          <= `ARIANE_CORE0.piton_pc;
        `endif
        `ifdef RTL_SARG0           
            spc0_0_inst_done         <= `SARG_CORE0.piton_pc_vld;
            spc0_0_phy_pc_w          <= `SARG_CORE0.piton_pc;
        `endif
        `ifdef RTL_LOX0
            spc0_0_inst_done         <= `LOX_CORE0.debug_commit_valid[0] & ~`LOX_CORE0.core_csr_xcptn_valid_o & ~`LOX_CORE0.csr_core_xcptn_valid_i;
            spc0_0_phy_pc_w          <= `LOX_CORE0.debug_commit_pc[0];
         `endif 
    end
end

assign inst_done = spc0_0_inst_done;
assign phy_pc_w  = spc0_0_phy_pc_w;




endmodule



/***************
     piton_lat_monitor
****************/


module  piton_lat_monitor #(
    parameter REQ_FLIT_WIDTH = 64,
    parameter RSP_FLIT_WIDTH =64
    )(
    input  [7:0]   id,
    input req_valid ,
    input [REQ_FLIT_WIDTH-1 : 0]  req_flit_in,
    input req_ready,

    input rsp_valid,
    input [RSP_FLIT_WIDTH-1 : 0] rsp_flit_in,
    input rsp_ready,
    output reg [63 : 0] lat_sum, req_num,
    input reset,
    input clk
    );

    wire req_header;
    hdr_pck_size_detect #(
        .FLIT_WIDTH(REQ_FLIT_WIDTH)
    ) req_detect_in (
        .reset      (reset),
        .clk        (clk ),
        .flit_in    (req_flit_in),
        .valid      (req_valid ),
        .ready      (req_ready ),
        .is_header  (req_header),
        .length     ()
    );


    wire rsp_header;
    hdr_pck_size_detect #(
        .FLIT_WIDTH(RSP_FLIT_WIDTH)
    ) rsp_detect_in (
        .reset      (reset),
        .clk        (clk ),
        .flit_in    (rsp_flit_in),
        .valid      (rsp_valid ),
        .ready      (rsp_ready ),
        .is_header  (rsp_header),
        .length     ()
    );


    wire [`MSG_TYPE_WIDTH-1:0]   req_msg_type = req_flit_in [`MSG_TYPE];
    wire [`MSG_TYPE_WIDTH-1:0]   rsp_msg_type = rsp_flit_in [`MSG_TYPE];
    wire [`MSG_MSHRID_WIDTH-1 : 0] req_id     = req_flit_in [`MSG_MSHRID];
    wire [`MSG_MSHRID_WIDTH-1 : 0] rsp_id     = rsp_flit_in [`MSG_MSHRID];

    reg [63 : 0] clk_counter;
 
    always @ (posedge clk) begin
        if(reset) clk_counter<=0;
        else clk_counter<=clk_counter+1;
    end

    reg [63 : 0] time_stamp [2**`MSG_MSHRID_WIDTH-1 : 0];
    reg [2**`MSG_MSHRID_WIDTH-1 : 0] valid;
   

    always @ (posedge clk) begin 
        if(reset) begin 
            valid <=0; 
            lat_sum <=64'd0;
            req_num <=64'd0;
        end else begin 
        if(req_valid & req_ready & req_header) begin             
            if(req_msg_type == `MSG_TYPE_LOAD_REQ ) begin 
             //  $display("MSG_TYPE_LOAD_REQ id:%d",req_id );
                time_stamp [req_id]<=clk_counter;
                valid      [req_id]<=1'b1;
            end
            if(req_msg_type == `MSG_TYPE_NC_LOAD_REQ ) begin 
             //   $display("MSG_TYPE_NC_LOAD_REQ id:%d ",req_id );
                time_stamp [req_id]<=clk_counter;
                valid      [req_id]<=1'b1;
            end

            if(req_msg_type == `MSG_TYPE_STORE_REQ ) begin 
             //   $display("MSG_TYPE_STORE_REQ id:%d ",req_id );
            end
        end

        if(rsp_valid & rsp_ready & rsp_header) begin 
            if(rsp_msg_type == `MSG_TYPE_DATA_ACK ) begin 
               // $display(" id:%d ", rsp_id );
                if(valid  [rsp_id ]) begin 
                    valid [rsp_id ] <= 1'b0;
                    lat_sum   <= lat_sum + (clk_counter-time_stamp [rsp_id]);
                    req_num   <= req_num + 1;
                 //   $display(" lat_sum:%d  req_num:%d ", lat_sum,  req_num );    
                end
            end
        end
        end//reset
    end

endmodule


/**************************
hdr_pck_size_detect
**************************/


module hdr_pck_size_detect  #(
    parameter FLIT_WIDTH=64
)(
    reset,
    clk,
    flit_in,
    valid,
    ready,
    is_header,
    length
);
    input reset,clk;
    input valid,ready;
    input [FLIT_WIDTH-1 : 0] flit_in;
    output  is_header;
    output [`MSG_LENGTH_WIDTH-1       :0] length;

    localparam 
        CHANEL_WORLD_NUM = FLIT_WIDTH/64;

    localparam  [1:0] 
		HEADER = 1,
		BODY   = 2;
    reg [1:0] flit_type,flit_type_next; 
    wire [`MSG_LENGTH_WIDTH-1       :0] length_in      =  flit_in [ `MSG_LENGTH ];
    reg  [`MSG_LENGTH_WIDTH-1       :0] remain, remain_next;    

    always @ (*) begin        
        remain_next = remain;
        flit_type_next = flit_type; 
        if(valid & ready) begin
            case(flit_type) 
            HEADER: begin 
                if (length_in >= CHANEL_WORLD_NUM ) begin 
                        flit_type_next = BODY;
                        remain_next = length_in  - CHANEL_WORLD_NUM;                                    
                end                 
            end //HEADER
            BODY: begin 
                if(remain < CHANEL_WORLD_NUM) begin                 
                        flit_type_next = HEADER;                     
                end else if (remain >= CHANEL_WORLD_NUM ) begin             
                        remain_next = remain  - CHANEL_WORLD_NUM;                    
                end
            end //BODY
	    default : begin
		remain_next = remain;
               flit_type_next = flit_type; 
	    end
            endcase
        end
    end//always

    always @ (posedge clk) begin
        if (reset)  begin  
            remain <= {`MSG_LENGTH_WIDTH{1'b0}};
            flit_type <=HEADER;
        end else begin 
            remain <= remain_next;
            flit_type <= flit_type_next;
        end
    end

    assign length    = (length_in > 11) ? 11 : length_in;
    assign is_header = (flit_type == HEADER);

endmodule

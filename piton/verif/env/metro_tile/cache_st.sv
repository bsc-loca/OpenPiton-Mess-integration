module cache_st #(
	parameter NAME ="",
	parameter Aw = 10	
)(
	input [Aw-1 : 0] addr,
	input wr_en,
	input rd_en,
	input clk
	
);

	reg [31 : 0] ram_wr [2**Aw-1 : 0];
	reg [31 : 0] ram_rd [2**Aw-1 : 0];
	
	initial begin 
		for (int i=0;i<2** Aw;i++) begin 
		 	ram_wr [i]=0;
		 	ram_rd [i]=0;
		end	
	end
	
	integer sum_wr =0;
	integer total;
	assign  total = 2** Aw;

	always @(posedge clk) begin 
		if(wr_en && ram_wr [addr] != {32{1'b1}} ) begin 
			if(ram_wr [addr] == 0) sum_wr ++;
			ram_wr [addr] = ram_wr [addr] +1;			
		end
		if(rd_en && ram_rd [addr] != {32{1'b1}} ) ram_rd [addr] = ram_rd [addr] +1;
	end

/*

integer sum;
real percent;



final begin
    sum =0;
    percent = 0;
	for (int i=0;i<2** Aw;i++) begin 
		 	if (ram_wr [i] !=0 ) sum++;
	end
	percent = real'(sum) * 100 / (2**Aw); 
    $display("%s , %m , %0t : percent=%f  \n",NAME, $time,percent);
end

*/


endmodule

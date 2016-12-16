// Code your design here
`timescale 1ns / 1ps

module processor(clk, OP
    );
	 input clk;
	 output [7:0] OP;
	 wire [7:0] data_out, add_out;
	 wire [15:0] data_in;
	 wire MW;
	 
	 CPU cpu(data_in, clk, add_out, data_out, MW);
	 memory Main_mem(MW, add_out, data_out, data_in);

endmodule

module CPU(data_in, clk, add_out, data_out, MW
    );
	 input clk;
	 input [15:0] data_in;
	 output [7:0] add_out, data_out;
	 output MW;
	 wire TD, TA, TB, MB, MD, MM, RW, PI, PL, IL, cond, MC, V, C, N, Z, C_neg, Z_neg;
	 wire [2:0] DR, SA, SB, MS;
	 wire [4:0] FS;
	 wire [7:0] const_in, PC_out, A_data;
	 wire [15:0] inst;
	 wire [5:0] AD;
	 wire [7:0] opt_add, BA, NA, CA;
	 wire [27:0] c_word;
	 
	 assign C_neg = ~C;
	 assign Z_neg = ~Z;
		
	 assign AD[5:3] = DR;
	 assign AD[2:0] = SB;
	 assign opt_add = 8'b0 + inst[15:9];
	 
	 assign DR = inst[8:6]; // Register Address assignment
	 assign SB = inst[2:0];
	 assign SA = inst[5:3];

	 assign NA = c_word[27:20]; // Assigning control signals
	 assign MS = c_word[19:17];
	 assign MC = c_word[16];
	 assign IL = c_word[15];
	 assign PI = c_word[14];
	 assign PL = c_word[13];
	 assign TD = c_word[12];
	 assign TA = c_word[11];
	 assign TB = c_word[10];
	 assign MB = c_word[9];
	 assign FS = c_word[8:4];
	 assign MD = c_word[3];
	 assign RW = c_word[2];
	 assign MM = c_word[1];
	 assign MW = c_word[0];
	 
	 assign const_in = 8'b0 + SB; // assigning constant input
	  
	 
	 mux_8 muxM(MM, A_data, PC_out, add_out);  // memory mux
	 mux_8 muxC(MC, NA, opt_add, BA);			// operation mux
	 mux_cond MuxS(MS, C, V, Z, N, C_neg, Z_neg, cond);	// condition mux
	 instruction_reg IR(data_in, IL, clk, inst);		// Instruction Register
	 program_counter PC(PC_out, AD, PI, PL, clk);	// Program Counter
	 car CAR(cond, BA, clk, CA);	// Control Add. Register
	 control_rom CROM(CA, c_word);	// Control ROM
	 datapath DP(DR,SA, SB, TD, TA, TB, RW, MB, MD, FS, const_in, data_in[7:0], clk,
	 V, C, N, Z, A_data, data_out);
	 
	 
endmodule

module memory(MW, address, DataIn, DataOut
);
input MW;
input [7:0] address;
input [7:0] DataIn;
output reg[15:0] DataOut;
  reg [15:0] mem[0:255];
  initial begin
  mem[0] = 16'b1000010000000010;			//add immediate 2 to R[0]
  mem[1] = 16'b1000010001001011;
  mem[2] = 16'b0000101010001000;  
  end

 

always @ (*) begin
if(MW==1) begin
mem[address] = 16'b0 + DataIn[7:0];
DataOut <= 0;
end
else
DataOut <= mem[address];
end
endmodule


module datapath(DA,AA, BA, TD, TA, TB, RW, MB, MD, FS, const_in, data_in, clk, V, C, N, Z, A_data, B_data); 
	input TD, TA, TB, MB, MD, RW, clk;
 	 input [2:0] DA, AA, BA;
  input [4:0] FS;
  	input [7:0] const_in, data_in;
	output V, C, N, Z;
  	output [7:0] A_data, B_data;
	
  
  wire [7:0] D_in, A_data, B_data, muxB_out, FU_out, data_write;

  RegFile reg_file(AA, BA, DA, TA, TB, TD, RW, data_write, clk, A_data, B_data);
  FunctionUnit FU(A_data, muxB_out, FS, V, C, N, Z, FU_out);
  mux_8 MuxB(MB, B_data, const_in, muxB_out);
  mux_8 MuxD(MD, FU_out, data_in, data_write);

endmodule


module decoder(add, out);
  input [3:0] add ;
output wire [8:0] out;
  assign out[0] = (add==4'd0);
  assign out[1] = (add==4'd1);
  assign out[2] = (add==4'd2);
  assign out[3] = (add==4'd3);
  assign out[4] = (add==4'd4);
  assign out[5] = (add==4'd5);
  assign out[6] = (add==4'd6);
  assign out[7] = (add==4'd7);
  assign out[8] = (add==4'd8);
endmodule

module FunctionUnit(
    input [7:0] ABUS,
    input [7:0] BBUS,
  input [4:0] FS,
    output reg V,
    output  C,
    output reg N,
    output reg Z,
  output  [7:0] FU_OUT
    );
 
  
  reg [8:0] result = 0;
	assign FU_OUT = result[7:0];
	assign C = result[8];
	always @ (ABUS or BBUS or FS) begin
		if (FS == 2'b00)
			result = ABUS ;
			else if (FS == 1)
			result = ABUS + 1;
			else if (FS == 2)
			result = ABUS + BBUS ;
			else if (FS == 3)
			result = ABUS + BBUS + 1 ;
			else if (FS == 4)
			result = ABUS + ~(BBUS) ;
			else if (FS == 5)
			result = ABUS + ~(BBUS) + 1 ;
			else if (FS == 6)
			result = ABUS - 1 ;
			else if (FS == 7)
			result = ABUS ;
			else if (FS == 8)
			result = ABUS & BBUS ;
			else if (FS == 9)
			result = ABUS | BBUS ;
			else if (FS == 10)
			result = ABUS ^ BBUS ;
			else if (FS == 11)
			result = ~(ABUS);
			else if (FS == 12)
			result = BBUS ;
			else if (FS == 13)
			result = BBUS >> 1 ;
			else
			result = BBUS << 1;
	end

	always @(result) begin
      if (FU_OUT==0) 
			Z = 1;// zero flag 
		else 
			Z = 0;

	end
endmodule


module mux_8(
   input sel,
	input [7:0] in0,
	input [7:0] in1,
  output reg  [7:0] out);


always @ (sel or in0 or in1) begin
  if (sel==0)
   out = in0 ;
  else 
   out = in1 ;
end
endmodule

module mux(
    input [3:0] add,
    input [7:0] in0,
    input [7:0] in1,
    input [7:0] in2,
    input [7:0] in3,
	 input [7:0] in4,
	 input [7:0] in5,
	 input [7:0] in6,
	 input [7:0] in7,
	 input [7:0] in8,
    output reg [7:0] out
    );
	 always @ (add or in0 or in1 or in2 or in3 or in4 or in5 or in6 or in7 or in8)
	 begin
	if (add == 4'b0000)
		out = in0 ;
	else if (add == 4'b0001)
		out = in1;
	else if (add == 4'b0010)
		out = in2;
	else if (add == 4'b0011)
		out = in3;
	else if (add == 4'd4)
		out = in4;
	else if (add == 4'd5)
		out = in5;
	else if (add == 4'd6)
		out = in6;
	else if (add == 4'd7)
		out = in7;
	else
		out = in8;
end
endmodule



module RegFile(
    input [2:0] SA,
    input [2:0] SB,
    input [2:0] SD,
	 input TD, 
	 input TA,
	 input TB,
	 input RW,
    input [7:0] DDATA,
    input CLK,
    output [7:0] ADATA,
    output [7:0] BDATA
    );
	
	//wire en;
	wire [8:0] DEC_OUT;
	wire [7:0] R0_OUT;
	wire [7:0] R1_OUT;
	wire [7:0] R2_OUT;
	wire [7:0] R3_OUT;
	wire [7:0] R4_OUT;
	wire [7:0] R5_OUT;
	wire [7:0] R6_OUT;
	wire [7:0] R7_OUT;
	wire [7:0] R8_OUT;
	wire [3:0] AA, BA, DA;
	assign AA[3] = TA;
	assign BA[3] = TB;
	assign DA[3] = TD;
	assign AA[2:0] = SA;
	assign BA[2:0] = SB;
	assign DA[2:0] = SD;
	//reg L0, L1, L2, L3 ;	
	//assign en = RW;
	//decoder DEC(1, DA, DEC_OUT);
	decoder DEC(DA,DEC_OUT);
	//L0 = DEC_OUT[0];
	//L1 = DEC_OUT[1];
	//L2 = DEC_OUT[2];
	//L3 = DEC_OUT[3];
	register R0(DDATA, DEC_OUT[0]&RW, R0_OUT, CLK);
	register R1(DDATA, DEC_OUT[1]&RW, R1_OUT, CLK);
	register R2(DDATA, DEC_OUT[2]&RW, R2_OUT, CLK);
	register R3(DDATA, DEC_OUT[3]&RW, R3_OUT, CLK);
	register R4(DDATA, DEC_OUT[4]&RW, R4_OUT, CLK);
	register R5(DDATA, DEC_OUT[5]&RW, R5_OUT, CLK);
	register R6(DDATA, DEC_OUT[6]&RW, R6_OUT, CLK);
	register R7(DDATA, DEC_OUT[7]&RW, R7_OUT, CLK);
	
	//Temporary Register for storage, invisible to the user
	register R8(DDATA, DEC_OUT[8]&RW, R8_OUT, CLK);
	mux MUX_A(AA, R0_OUT, R1_OUT, R2_OUT, R3_OUT, R4_OUT, R5_OUT, R6_OUT, R7_OUT, R8_OUT, ADATA);
	mux MUX_B(BA, R0_OUT, R1_OUT, R2_OUT, R3_OUT, R4_OUT, R5_OUT, R6_OUT, R7_OUT, R8_OUT, BDATA);
	
	
endmodule



module register(
    input [7:0] data,
    input load,
    output reg [7:0] out,
    input clk
    );
  
	initial begin 
      out = 0;
    end
  
	always @ (posedge clk)
		if (load)
			out = data ;
endmodule
	

module car(cond, BA, clk, NA );
	input cond, clk;
	input [7:0] BA;
	output [7:0] NA;
	reg [7:0] NA = 8'd0;
	
	always @(posedge clk)
	begin
      #1;
		if(cond == 1'd0)
		begin
			NA = NA + 8'd1;
		end
		else
		begin
			NA = BA;
		end
	end
endmodule	

module control_rom(addr, word
);
input [7:0] addr;
output reg [27:0] word = 28'b0000000000111100000000000010;

always@(addr) begin
case(addr[7:0])
8'b00000000: word <= 28'b0000000000111100000000000010;   // Fetch
8'b01000010: word <= 28'b0000000000100000001000100100;
8'b00000101: word <= 28'b0000000000100000000000100100;
default: word <= 28'b0;
endcase
end
endmodule

module instruction_reg(ins, load, clk, out  
    );
	 input [15:0] ins;
	 input load, clk;
	 output reg [15:0] out;
	 
	 always @(posedge clk) begin
	 if (load)
		out = ins;
	end
endmodule


module program_counter(add_out, AD, PI, PL, clk);
  input PI, PL, clk;
  input[5:0] AD;
  output[7:0] add_out;
  reg [7:0] offset;
  reg [7:0] add_out = 0;
  always @(AD)
    begin
  	if (AD[5] == 1)
        begin
       	offset <= 8'b11000000 + AD[5:0];
    	end
  	else 
   		 begin
     	 offset <= 8'b00000000 + AD[5:0];
    end
    end
  
  always @(posedge clk)
    begin
      if ((PI == 1'b1) & (PL == 1'b0))
        begin 
          add_out <= add_out + 1;
        end
      else if ((PL == 1'b1) & (PI == 1'b0))
        begin
           add_out <= (add_out + offset);
        end
    end
endmodule

module mux_cond(sel, in2, in3, in4, in5, in6, in7, out
    );
	 input in2, in3, in4, in5, in6, in7;
	 input [2:0] sel;
	 output reg out;
	 
	 always @(sel or in2 or in3 or in4 or in5 or in6 or in7)
	 begin
		case(sel)
			3'd0: out <= 0;
			3'd1: out <= 1;
			3'd2: out <= in2;
			3'd3: out <= in3;
			3'd4: out <= in4;
			3'd5: out <= in5;
			3'd6: out <= in6;
			3'd7: out <= in7;
			endcase
	end
endmodule

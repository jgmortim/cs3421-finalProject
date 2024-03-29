module ProcessorTest1(output reg F);
	reg clk;							 
	
	Processor DUT(clk);
	
	
	always
	begin
		clk = 1'b1;
		#10;
		clk = 1'b0;
		#10;
	end
	
//	always@(posedge clk)
//	begin
//		#1
//		
//	end

	
endmodule 


module Processor(input CLK);
	wire [31:0] pcPrime;
	wire [31:0] pc;
	wire [31:0] Instr;
	wire [31:0] RD1;
	wire [31:0] RD2;
	wire [31:0] SignImm;
	wire [31:0] ZeroImm;
	wire [4:0] WriteReg;
	wire [31:0] Result;
	wire [31:0] SrcB;
	wire [31:0] ALUResult;
	wire [31:0] ReadData;
	wire [31:0] pcp4;
	wire [31:0] pcb;
	wire [31:0] shiftOut;
	wire cout;
	wire [1:0] PCSrc;
	wire zero;
	wire [31:0] mx2mx;
	wire [31:0] JLine;
	
	wire MemtoReg;
	wire MemWrite;
	wire BranchEQ;
	wire BranchNE;
	wire [2:0] ALUControl;
	wire [1:0] ALUSrc;
	wire RegDst;
	wire RegWrite;
	wire Jump;	
	
	
	PC PC1(CLK, pcPrime, pc);
	InstructionMemory IM(pc, Instr);
	RegFile RF(CLK, RegWrite, Instr[25:21], Instr[20:16], WriteReg, Result, RD1, RD2);
	ControlUnit CU(Instr[31:26], Instr[5:0], MemtoReg, MemWrite, BranchEQ, BranchNE, ALUSrc, RegDst, RegWrite, Jump, ALUControl);
	ALU A(RD1, SrcB, ALUControl, ALUResult);
	DataMem DM(CLK, MemWrite, ALUResult, RD2, ReadData);
	SignExtend SE(Instr[15:0], SignImm);
	ZeroExtend16to32 ZE(Instr[15:0], ZeroImm);
	PCPlus4 pc4(pc, pcp4);
	Shifter SH(SignImm, shiftOut);
	adder32 pcBranch(shiftOut, pcp4, 1'b0, pcb, cout);
	
	assign zero = (ALUResult==0);
	assign PCSrc[1] = BranchEQ & zero;
	assign PCSrc[0] = BranchNE & (!zero);
	assign JLine[31:26] = pcp4[31:26];
	assign JLine[25:0] = Instr[25:0];

	mux4to1 mx0(PCSrc, pcp4, pcb, pcb, 32'bx, mx2mx);
	mux2to1_5 mx1(RegDst, Instr[20:16], Instr[15:11], WriteReg);
	mux4to1 mx2(ALUSrc, RD2, SignImm, ZeroImm, 32'bx, SrcB);
	mux2to1 mx3(MemtoReg, ALUResult, ReadData, Result);
	mux2to1 mxJ(Jump, mx2mx, JLine, pcPrime);

	
endmodule

/* Control Unit */
module ControlUnit(input [5:0] opcode,
			input [5:0] funct,
			output reg MemtoReg,
			output reg MemWrite,
			output reg BranchEQ,
			output reg BranchNE,
			output reg [1:0] ALUSrc,
			output reg RegDst,
			output reg RegWrite,
			output reg Jump,
			output reg [2:0] ALUControl);
			
	always @(opcode, funct)
	begin
		MemtoReg = 1'b0;
		MemWrite = 1'b0;
		BranchEQ = 1'b0;
		BranchNE = 1'b0;
		ALUSrc = 2'b00;
		RegDst = 1'b0;
		RegWrite = 1'b0;
		Jump = 1'b0;
		ALUControl = 3'b000;
		if (opcode == 6'b000000)
		begin
			RegWrite = 1'b1;
			RegDst = 1'b1;
			case (funct)
				6'b100000: ALUControl = 3'b010; // Add
				6'b100010: ALUControl = 3'b110; // Sub
				6'b100100: ALUControl = 3'b000; // And
				6'b100101: ALUControl = 3'b001; // Or
				6'b101010: ALUControl = 3'b111; // SLT
				default: ALUControl = 3'bxxx;
			endcase
		end
		else
		begin
			case (opcode)
				// LW
				6'b100011:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b01;
					MemtoReg = 1'b1;
					ALUControl = 3'b010;
				end
				// SW
				6'b101011:
				begin
					RegDst = 1'bx;
					ALUSrc = 2'b01;
					MemWrite = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b010;
				end
				// BEQ
				6'b000100:
				begin
					RegDst = 1'bx;
					BranchEQ = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b110;
				end
				// ADDI
				6'b001000:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b01;
					ALUControl = 3'b010;
				end
				// J
				6'b000010:
				begin
					RegDst = 1'bx;
					ALUSrc = 2'bxx;
					BranchEQ = 1'bx;
					MemtoReg = 1'bx;
					Jump= 1'b1;
				end
				//ORI
				6'b001101:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b01;
					ALUControl = 3'b001;
				end
				//ANDI
				6'b001100:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b01;
					ALUControl = 3'b000;
				end
				//SLTI
				6'b001010:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b01;
					ALUControl = 3'b111;
				end
				//BNE
				6'b000101:
				begin
					RegDst = 1'bx;
					BranchNE = 1'b1;
					MemtoReg = 1'bx;
					ALUControl = 3'b110;
				end
				// ADDIU
				6'b001001:
				begin
					RegWrite = 1'b1;
					ALUSrc = 2'b10;
					ALUControl = 3'b010;
				end
				default:
				begin
					MemtoReg = 1'bx;
					MemWrite = 1'bx;
					BranchEQ = 1'bx;
					ALUSrc = 2'bxx;
					RegDst = 1'bx;
					RegWrite = 1'bx;
					Jump= 1'bx;
					ALUControl = 3'bxxx;
				end
			endcase
		end // else
	end // always
endmodule
					
					
					
/* Program Counter */
module PC(input CLK,
			input [31:0] dataIn,
			output reg [31:0] dataOut);
			
	reg [31:0] pcreg;
	initial
	begin
		pcreg = 32'b0;
	end
	
	always @(posedge CLK)
	begin
		dataOut = pcreg;
	end
		always @(negedge CLK)
	begin
		pcreg <= dataIn;
	end
endmodule

/* Instruction Memory */
module InstructionMemory(input [31:0] A,
			output reg [31:0] RD);
			
	reg [31:0] instMemory [0:1023];
	initial
	begin
		$readmemh("C:/altera_lite/16.0/Processor/t3test1.out", instMemory);
	end
	
	always @(A)
	begin
		RD <= instMemory[A];
	end
endmodule

/* Register File */
module RegFile(input CLK,
			input WE3,
			input [4:0] A1,
			input [4:0] A2,
			input [4:0] A3,
			input [31:0] WD3,
			output reg [31:0] RD1,
			output reg [31:0] RD2);
			
	reg [31:0] gpreg [0:31];
	reg [31:0] i;
	
	initial
	begin
		for (i=0; i<32; i=i+1)
		begin
			gpreg[i] <= 32'b0;
		end
	end
	
	always @(*)
	begin
		RD1 <= gpreg[A1];
		RD2 <= gpreg[A2];
	end
	
	always @(negedge CLK)
	begin
		if (WE3 == 1'b1)
		begin
				gpreg[A3] <= WD3;
		end
	end
endmodule

/* Data Memory */
module DataMem(input CLK,
			input WE,
			input [31:0] A,
			input [31:0] WD,
			output reg [31:0] RD);

	reg [31:0] dataMemory [0:1023];
	reg [31:0] i;
	
	initial
	begin
		for (i=0; i<1024; i=i+1)
		begin
			dataMemory[i] <= 32'b0;
		end
	end
	
	always @(A)
	begin
		RD <= dataMemory[A];
	end
	
	always @(negedge CLK)
	begin
		if (WE == 1'b1)
		begin
			dataMemory[A] <= WD;
		end
	end
	

endmodule


/* ALU */
module ALU(input [31:0] A,
			input [31:0] B,
			input [2:0] F,
			output [31:0] Y);

	wire [31:0] notB;
	wire [31:0] mx1out;
	wire [31:0] orout;
	wire [31:0] andout;
	wire [31:0] S;
	wire Cout;
	wire [31:0] zeroex;

	assign notB = ~B;
	mux2to1 mx1(F[2], B, notB, mx1out);
	assign orout = A | mx1out;
	assign andout = A & mx1out;
	adder32 add(A, mx1out, F[2], S, Cout);
	zeroExtender ze(S[31], zeroex);
	mux4to1 mx2(F[1:0], andout, orout, S, zeroex, Y);
endmodule

/*Sign Extend*/
module SignExtend(input [15:0] in,
			output [31:0] out);
	assign out[31:16] = {16{in[15]}};
	assign out[15:0] = in[15:0];
endmodule

/* Zero Extender */
module ZeroExtend16to32(input [15:0] dataIn,
				output [31:0] dataOut);
	assign dataOut = 32'b0 + dataIn;
endmodule

/* PC plus 4 */
module PCPlus4(input [31:0] PC,
			output [31:0] out);
	wire Cout;
//	adder32 add4(PC, 4, 1'b0, out, Cout);
	adder32 add4(PC, 32'b00000000000000000000000000000001, 1'b0, out, Cout);
endmodule

/*Shifter*/
module Shifter(input [31:0] in,
			output [31:0] out);
//	assign out = in << 2;
	assign out = in;
endmodule


/*-----------------------------------*/
/* lesser components below this line */
/*-----------------------------------*/


/* 5 bit 2 to 1 mux */
module mux2to1_5(input select,
				input [4:0] w0,
				input [4:0] w1,
				output reg [4:0] y);
				
	always @(*)
	begin
		case (select)
			0: y=w0;
			1: y=w1;
		endcase 
	end
endmodule

/* 2 to 1 mux */
module mux2to1(input select,
				input [31:0] w0,
				input [31:0] w1,
				output reg [31:0] y);
				
	always @(*)
	begin
		case (select)
			0: y=w0;
			1: y=w1;
		endcase 
	end
endmodule

/* 4 to 1 mux */
module mux4to1(input [1:0] select,
				input [31:0] w0,
				input [31:0] w1,
				input [31:0] w2,
				input [31:0] w3,
				output reg [31:0] y);
				
	always @(*)
	begin
		case (select)
			0: y[31:0]=w0[31:0];
			1: y[31:0]=w1[31:0];
			2: y[31:0]=w2[31:0];
			3: y[31:0]=w3[31:0];
		endcase
	end
endmodule



/* 32 bit Zero Extender */
module zeroExtender(input singleBit,
				output [31:0] dataOut);
				
	assign dataOut = 32'b0 + singleBit;
endmodule



/* 32 bit adder */
module adder32(input [31:0] A,
				input [31:0] B,
				input carryIn,
				output [31:0] sum,
				output carryOut);		
				
	wire [6:0] C;
	adder4 add0(A[3:0],   B[3:0], carryIn, sum[3:0],   C[0]);
	adder4 add1(A[7:4],   B[7:4],   C[0],  sum[7:4],   C[1]);
	adder4 add2(A[11:8],  B[11:8],  C[1],  sum[11:8],  C[2]);
	adder4 add3(A[15:12], B[15:12], C[2],  sum[15:12], C[3]);
	adder4 add4(A[19:16], B[19:16], C[3],  sum[19:16], C[4]);
	adder4 add5(A[23:20], B[23:20], C[4],  sum[23:20], C[5]);
	adder4 add6(A[27:24], B[27:24], C[5],  sum[27:24], C[6]);
	adder4 add7(A[31:28], B[31:28], C[6],  sum[31:28], carryOut);
endmodule

/* 4 bit adder */
module adder4(input [3:0] A,
			input [3:0] B,
			input carryIn,
			output reg [3:0] sum,
			output reg carryOut);	
			
	reg [3:0] C;
	always @(*)
	begin
	C[0]=carryIn;
	C[1]=A[0] & B[0] | (A[0] | B[0]) & C[0];
	C[2]=A[1] & B[1] | (A[1] | B[1]) & C[1];
	C[3]=A[2] & B[2] | (A[2] | B[2]) & C[2];
	carryOut=A[3] & B[3] | (A[3] | B[3]) & C[3];
	sum = A ^ B ^ C;
	end
endmodule


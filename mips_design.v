`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.03.2024 17:43:33
// Design Name: MIPS 32 Bit Processor
// Module Name: mips_design
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//

//Instruction Memory
module instruction_fetch(
input [31:0]inst_read_address,
output [31:0]instruction
);begin

	reg [31:0]instruction;
	integer i=0;
	reg [31:0]instruction_memory[0:255];
	initial begin
		$readmemb("instruction.mem",instruction_memory);
	end

	always@(inst_read_address)begin
		instruction = instruction_memory[inst_read_address];
		$display("the instruction is %b for address %b",instruction,inst_read_address);
	end

end
endmodule

//instruction decode
module instruction_decode(input [31:0]instruction,
output wire [5:0] opcode,
output reg [4:0]rs, rt, rd, shamt,
output reg [5:0]funct,
output reg [15:0]immediate,
output reg [25:0]address);

assign opcode = instruction[31:26];

always@(instruction)begin
	//R-type
	if(opcode == 6'h0)begin
		shamt = instruction[10:6];
		rd = instruction[15:11];
		rt = instruction[20:16];
		rs = instruction[25:21];
		funct = instruction[5:0];
	end
	// J type
	else if(opcode == 6'h2 | opcode == 6'h3) begin
		address = instruction[25:0];
	end
	// I type
	else begin
		rt = instruction[20:16];
		rs = instruction[25:21];
		immediate = instruction[15:0];
	end
end

endmodule

//Control Unit
module control_unit(
input [5:0]opcode,funct,//instruction opcode and function
input zero,//zero flag
output reg 	sig_reg_dst, 
			sig_reg_write,
			sig_reg_read, 
			sig_mem_read, 
			sig_mem_write, 
			sig_mem_to_reg);

always@(opcode,funct,zero)begin
	sig_reg_dst     = 1'b0;
	sig_reg_write   = 1'b0;
	sig_mem_read    = 1'b0;
	sig_mem_write   = 1'b0;
	sig_mem_to_reg  = 1'b0;


	case(opcode)
		//Reg Type
		6'b000000:begin //R type instructions
			sig_reg_read = 1'b1;
			sig_reg_dst = 1'b1;
			sig_reg_write = 1'b1;
			// if not jr
			if(funct != 6'h08) begin
				sig_reg_write = 1'b1;
			end
			
		end

		//LW
		6'b100011:begin
			sig_reg_read = 1'b1;
			sig_mem_read = 1'b1;
			sig_mem_to_reg = 1'b1;
			sig_reg_write = 1'b1;
		end

		//SW 
		6'b101011:begin
			sig_reg_read   = 1'b1;
			sig_mem_write   = 1'b1;

		end

		//ADDI
		6'b001001:begin
			sig_reg_read = 1'b1;
			sig_reg_dst = 1'b0;
			sig_reg_write = 1'b1;
		end
		
		//addi
		6'b001000:begin
			sig_reg_read = 1'b1;
			sig_reg_dst = 1'b0;
			sig_mem_to_reg = 1'b0;
			sig_reg_write = 1'b1;
		end

		default:$display("wrong opcode in control unit");
	endcase
end
endmodule




//Register Bank for MIPS 32 bit
module registers(
input [4:0]register_read1,register_read2,write_register,//5 bti address for read and write regiter
input [31:0]write_data,//data to be written into register
input sig_reg_write,sig_reg_read,//signal 1 to write otherwise 0
output reg [31:0]read_data_1,read_data_2//
);
begin
	reg [31:0]registers[0:31];//memory to store value for register
	integer i;
initial begin
	for(i=0;i<32;i=i+1)begin
		registers[i] = 32'b0;
	end
	$writememb("registers.mem",registers);
	$readmemb("registers.mem",registers);
end


always@(write_data)begin
	if(sig_reg_write)begin
		registers[write_register] = write_data;
		$display("write register %b in add %b", registers[write_register],write_register);
	end	
	// write to file
	$writememb("registers.mem",registers);
end

always@(register_read1,register_read2)begin

	if(sig_reg_read)begin
		read_data_1 = registers[register_read1];
		read_data_2 = registers[register_read2];
	end
end
end
endmodule
//Register Module End


	
//ALU Unit for performing all the arithmatic operations
//000 --> Add
//001 --> Mult
//010 --> Sub
//011 --> And
//100 --> Or
module alu(
input [31:0] rs_data,rt_data,//source and target regs data
input [4:0] shamt,
input [15:0] immediate,
input [5:0] opcode,//opcode of the arithmatic operations from control unit
input [5:0] alu_funct,
output reg [31:0] alu_result,//alu_result from the alu unit
output reg zero,//zero flag
output reg sig_branch
);
begin 

reg signed [31:0]sign_rs,sign_rt;
reg [31:0] signExtend, zeroExtend;
always@(alu_funct,rs_data,rt_data,shamt,immediate)begin

		sign_rs = rs_data;
        sign_rt = rt_data;

		//R Type
		if (opcode == 6'h0)begin
			case(alu_funct)
				6'h20:  //add
					alu_result = sign_rs + sign_rt;
				6'h21: //addu 
					alu_result = rs_data + rt_data;
				6'h22:  //sub
					alu_result = sign_rs - sign_rt;
				6'h23:  //subu
					alu_result = rs_data - rt_data;
				6'h24:   //and
					alu_result = rs_data & rt_data;
				default:
					$display("The value of ALU opcode is Invalid");
			endcase
		end

		// I Type
		else begin
			signExtend = {{16{immediate[15]}},immediate};
			zeroExtend = {{16{1'b0}},immediate};
			case(opcode)
				6'h8: //addi
					alu_result = sign_rs + signExtend;
				6'h23:  //lw
					alu_result = sign_rs + signExtend;
				6'h2b:  //sw
					alu_result = sign_rs + signExtend;
				6'h9:  //addiu
					alu_result = rs_data + signExtend;
				6'h4:  //beq
					begin 
						alu_result = sign_rs - sign_rt;
						if(alu_result == 0)begin 
							sig_branch = 1'b1;
						end
						else begin
							sig_branch = 1'b0;
						end

					end
				6'h05:  //bne
				begin
					alu_result = rs_data - rt_data;
					if (alu_result != 0)begin 
						sig_branch = 1'b1;
					end
					else begin
						sig_branch = 1'b0;
						end
				end

			endcase
		end

		if(alu_result == 0)begin
			zero = 1'b1;
		end
		else begin 
			zero = 1'b0;
		end
	end
end
endmodule
//ALU Module End


//data Memory Module for Load and Store the data
module data_memory(
input [31:0] write_address, //write address for the memory
input [31:0] read_address,//readd address for the memory
input [31:0]mem_write_data,//data to be written in memory
input mem_write_en,//Control bit for the data write
input mem_read_en,//contorl bit for the data read
output [31:0]mem_read_data//output data from mem
);

//initialize the variables
//memory intialize
reg [31:0]data_mem[0:255];
integer i;
reg [31:0]mem_read_data;
//memory with zero
initial begin
 	for(i=0;i<32;i=i+1)begin
		data_mem[i] = 32'b0;
	end
	$writememb("data.mem",data_mem);
	$readmemb("data.mem",data_mem);
end


always@(write_address)
	begin 

		if (mem_write_en)
		begin
			data_mem[write_address] = mem_write_data;
			$writememb("data.mem", data_mem); // save the modified memory to file
		end
	end

always@(read_address)
	begin
		if(mem_read_en)begin
			mem_read_data <= data_mem[read_address];  
		end
	end
endmodule

// helper modules
module mux5_2x1 (
    input [4:0] in_0, in_1, 
    input control,
    output reg [4:0] out
    );

    always @(in_0, in_1, control) begin
        if (control) begin
            out = in_1;
        end else begin
            out = in_0;
        end
    end
endmodule

// helper modules
module mux32_2x1 (
    input [31:0] in_0, in_1, 
    input control,
    output reg [31:0] out
    );

    always @(in_0, in_1, control) begin
        if (control) begin
            out = in_1;
        end else begin
            out = in_0;
        end
    end
endmodule


module add_1 (
    input [31:0] pc,
    output reg [31:0] next_pc
    );

    always @(pc) begin
        next_pc = pc + 1;
    end
endmodule



module mips(input clock);

reg [31:0]pc = 32'b0; // program counter
reg [31:0]pc_next; //hold the next program counter

wire [31:0]instruction;

wire[31:0]read_data; //output data from the memory
wire [31:0]read_reg1, read_reg2; //output from the registers
wire [31:0]alu_result;
wire zero; // zero flag from the alu

// output from 4 mux's
wire [4:0] write_register;
wire [31:0] alu_rt;//output from the mux controlling source for the alu
wire [31:0] write_data_reg;
wire [31:0] instr_address; 
wire [31:0] sext_immediate; // output from sign extend
wire [31:0] next_pc, branch_next_pc; // output from pc adders

// Parse instruction
wire [5:0] funct;
wire [4:0] rs, rt, rd, shamt;
wire [25:0] address;
wire [15:0] immediate;
wire [5:0] opcode;
	
// signals
wire sig_reg_dst, sig_reg_write, sig_alu_src, sig_mem_read, sig_mem_write;
wire sig_mem_to_reg, sig_pc_src, sig_branch;
wire [2:0] alu_op;

//instruction fetch
instruction_fetch instr_mem(pc,instruction);

//instruction decode
instruction_decode instr_dec(instruction,opcode,rs,rt,rd,shamt,funct,immediate,address);

//Control Unit FSM
control_unit ctrl(opcode,funct,zero,sig_reg_dst,sig_reg_write,sig_reg_read,sig_mem_read,sig_mem_write,sig_mem_to_reg);

//Register Bank
mux5_2x1 mux1(rt,rd,sig_reg_dst,write_register);
registers regs(rs,rt,write_register,write_data_reg,sig_reg_write,sig_reg_read,read_reg1,read_reg2);

//Instruction Execute
alu alu_unit(read_reg1,read_reg2,shamt,immediate,opcode,funct,alu_result,zero,sig_branch);

//Memory Access
data_memory data_mem(alu_result,alu_result,read_reg2,sig_mem_write,sig_mem_read,read_data);
mux32_2x1 mux3(alu_result,read_data,sig_mem_to_reg,write_data_reg);

// Next PC logic
add_1 pc_adder(pc,next_pc);
// add_shifter branch_pc_adder(next_pc,sext_immediate,branch_next_pc);
// mux32_2x1 mux4(next_pc,branch_next_pc,sig_pc_src,pc_next);//will require for pc operations


always@(posedge clock)begin
	//branch
	if(sig_branch == 1)begin
		pc = pc + 1 + $signed(immediate);
	end
	//jr
	else if(opcode == 6'h0 & funct == 6'h08)begin
		pc = read_reg1;
	end
	//jump
	else if(opcode == 6'h2)begin
		pc = address;
	end
	// normal mode increment
	else begin 
		pc = next_pc;
	end
end

endmodule


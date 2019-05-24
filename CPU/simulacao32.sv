`timescale 1ps/1ps
module simulacao32;

// Sinais externos
logic clk;
logic nrst;

// Sinais da maquina de estado
logic [5:0] state = 0;
logic [5:0] nextState;

// Entrada da Memoria de Instrucao
logic [63:0]rdaddress;
logic [63:0]data;
reg Wr_meminst;

// Saida da Memoria de Instrucao
wire [31:0] Meminst_out;

// Entrada do PC
reg Wr_pc;

// Entrada do Registrador de Instrucao
reg Wr_instr_Reg;

// Saida do Registrador de Instrucao
logic [4:0]rs1;		
logic [4:0]rs2;
logic [4:0]rd;
logic [6:0]opcode;
logic [31:0]inst;

// Entradas da ULA
logic [2:0] ALUFct;
logic [63:0] Writedata;
logic [63:0] Writedatazero;
logic [63:0] Writedataone;
logic [63:0] Writedatadef;
logic [63:0] Writedataover;


// Saidas da ULA
logic [63:0] result;
reg neg;
reg over;
reg zero;
reg maior;
logic menor;
reg igual;

// Entradas do Banco
reg Wr_banco;

// Saidas do Banco
logic [63:0] dataA;
logic [63:0] dataB;

// Entradas do Registrador A
reg Wr_A;

// Saidas do Registrador A
logic [63:0] dataOA;

// Entradas do Registrador B
reg Wr_B;

// Saidas do Registrador B
logic [63:0] dataOB;

// Entradas do MUX 1
reg [1:0] sel_mux1;

// Saidas do MUX 1
logic [63:0] mux1_out;

// Entradas do ALUOut
reg Wr_aluout;

// Saídas do ALUOut
logic [63:0] ALUOut_out;

// Entradas da Memória de dados
reg Wr_memdata;

// Saídas da Memória de dados
logic [63:0] Memdata_out;

// Entradas do Reg. da Memória de dados
reg Wr_memreg;

// Saídas do Reg. da  Memória de dados
logic [63:0] Memreg_out;

// Entradas do MUX 2
reg [1:0] sel_mux2;

// Saidas do MUX 2
logic [63:0] mux2_out;

// Saidas do Extensor de Sinal
logic [63:0] ext_out;

// Saidas do Extensor de Menor
logic [63:0] menor_out;

// Entradas do Shift Left
reg [1:0] sh;
reg [5:0] offset;

// Saidas do Shift Left
logic [63:0] shift_out;

// Entradas do MUX 3
reg [1:0] sel_mux3;

// Saidas do MUX 3
logic [63:0] mux3_out;

// Entradas do MUX 4
reg [1:0] sel_mux4;

// Saidas do MUX 4
logic [63:0] mux4_out;

// Entradas do MUX 5
reg [1:0] sel_mux5;

// Saidas do MUX 5
logic [63:0] mux5_out;

// Entradas do Extend Store
reg [1:0] sel_store;

// Saidas do Extend Store
logic [63:0] store_out;

// Entradas do Extend Load
reg [2:0] sel_load;

// Saida do Extend Load
logic [63:0] load_out;

// Entradas do Registrador EPC
reg Wr_epc;

// Saídas do Registrador EPC
logic [63:0] epc_out;

// Entradas do Registrador de Causa
reg Wr_causa;

// Saídas do Registrador de Causa
logic [63:0] causa_out;

// Entradas do MUX 6
reg [1:0] sel_mux6;

// Saidas do MUX 6
logic [63:0] mux6_out;

// Entradas do MUX 7
reg [1:0] sel_mux7;

// Saidas do MUX 7
logic [63:0] mux7_out;

// Entradas do MUX 8
reg [1:0] sel_mux8;

// Saidas do MUX 8
logic [63:0] mux8_out;

// Componentes
Memoria32 		meminst ( .raddress(rdaddress[31:0]), .waddress(rdaddress[31:0]), .Clk(clk), .Datain(data[31:0]), .Dataout(Meminst_out), .Wr(Wr_meminst) );
register 		PC 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_pc), .DadoIn(mux8_out), .DadoOut(rdaddress) );
Instr_Reg_RISC_V 	instr_Reg ( .Clk(clk), .Reset(nrst), .Load_ir(Wr_instr_Reg), .Entrada(Meminst_out), .Instr19_15(rs1), .Instr24_20(rs2), .Instr11_7(rd), .Instr6_0(opcode), .Instr31_0(inst) ); 
ula64			Aluzinha  ( .A(mux1_out), .B(mux3_out), .Seletor(ALUFct), .S(result), .Overflow(over), .Negativo(neg), .z(zero), .Igual(igual), .Maior(maior), .Menor(menor) );
bancoReg		registers ( .write(Wr_banco), .clock(clk), .reset(nrst), .regreader1(rs1), .regreader2(rs2), .regwriteaddress(rd), .datain(mux2_out), .dataout1(dataA), .dataout2(dataB) );  
register 		regA 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_A), .DadoIn(dataA), .DadoOut(dataOA) );
register 		regB 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_B), .DadoIn(dataB), .DadoOut(dataOB) );
register 		memreg 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_memreg), .DadoIn(load_out), .DadoOut(Memreg_out) );
mux64bits		mux1 ( .sel(sel_mux1), .in0(rdaddress), .in1(dataOA), .in2(epc_out), .in3(Writedatazero), .out(mux1_out) );
register 		ALUOut 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_aluout), .DadoIn(result), .DadoOut(ALUOut_out) );
Memoria64		memdata ( .raddress(mux7_out), .waddress(mux7_out), .Clk(clk), .Datain(mux5_out), .Dataout(Memdata_out), .Wr(Wr_memdata) );
mux64bits		mux2 ( .sel(sel_mux2), .in0(ALUOut_out), .in1(Memreg_out), .in2(ext_out), .in3(menor_out), .out(mux2_out) );
ExtendSignal		extend ( .in(inst), .out(ext_out) );
Deslocamento		shiftLeft ( .Shift(sh), .Entrada(mux4_out), .N(offset), .Saida(shift_out) );
mux64bits		mux3 ( .sel(sel_mux3), .in0(dataOB), .in1(Writedata), .in2(ext_out), .in3(shift_out), .out(mux3_out) );
mux64bits		mux4 ( .sel(sel_mux4), .in0(ext_out), .in1(dataOA), .in2(Writedata), .in3(Writedata), .out(mux4_out) );
mux64bits		mux5 ( .sel(sel_mux5), .in0(dataOB), .in1(store_out), .in2(Writedata), .in3(Writedata), .out(mux5_out) );
ExtendMenor		extendMenor ( .in(menor), .out(menor_out) );
ExtendStore		extendStore ( .Ent_mem(Memdata_out), .Ent_alt(dataOB), .Seletor(sel_store), .Saida(store_out) );
ExtendLoad		extendLoad ( .Ent_mem(Memdata_out), .Seletor(sel_load), .Saida(load_out) );
register 		epc 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_epc), .DadoIn(result), .DadoOut(epc_out) );
register 		causa 	  ( .clk(clk), .reset(nrst), .regWrite(Wr_causa), .DadoIn(mux6_out), .DadoOut(causa_out) );
mux64bits		mux6 ( .sel(sel_mux6), .in0(Writedatazero), .in1(Writedataone), .in2(Writedata), .in3(Writedata), .out(mux6_out) );
mux64bits		mux7 ( .sel(sel_mux7), .in0(ALUOut_out), .in1(Writedatadef), .in2(Writedataover), .in3(Writedata), .out(mux7_out) );
mux64bits		mux8 ( .sel(sel_mux8), .in0(result), .in1(load_out), .in2(Writedata), .in3(Writedata), .out(mux8_out) );

enum logic [5:0] {RESET = 6'd0, BUSCA = 6'd1, MOVTOAB = 6'd2, ADD = 6'd3, SUB = 6'd4, WRBANCO = 6'd5, LD = 6'd6, WRLDREG = 6'd7, ADDI = 6'd8, 
WRBANCOADDI = 6'd9, SD = 6'd10, WRSDDATA = 6'd11, LUI = 6'd12, BEQ = 6'd13, BRANCHSUM = 6'd14, BNE = 6'd15, ENDBRANCH = 6'd16, BGE = 6'd17, BLT = 6'd18, AND = 6'd19, SLT = 6'd20, 
SLTI = 6'd22, JAL = 6'd23, WRBANCOJAL = 6'd24, JALR = 6'd25, WRBANCOJALR = 6'd26, SRLI = 6'd27, WRBANCOSHIFT = 6'd28, SRAI = 6'd29, SLLI = 6'd30, BREAK = 6'd31,
WRSBDATA = 6'd32, SB = 6'd33, WRSHDATA = 6'd34, SH = 6'd35, WRSWDATA = 6'd36, SW = 6'd37, LB = 6'd38, LH = 6'd39, LW = 6'd40, LBU = 6'd41, LHU = 6'd42, LWU = 6'd43, WRLBREG = 6'd44,
WRLHREG = 6'd45, WRLWREG = 6'd46, WRLBUREG = 6'd47, WRLHUREG = 6'd48, WRLWUREG = 6'd49, WRBANCOLOAD = 6'd50, NOP = 6'd51, OPDEF = 6'd52, OVER = 6'd53, WROPDEF = 6'd54, WROVER = 6'd55,
WAITLD = 6'd57, WAITLW = 6'd58, WAITLH = 6'd59, WAITLB = 6'd60, WAITLWU = 6'd61, WAITLHU = 6'd62, WAITLBU = 6'd63} estados;
enum logic [6:0] {TypeR = 7'b0110011, TypeI = 7'b0000011, TypeIaddi = 7'b0010011, TypeS = 7'b0100011, TypeSBbeq = 7'b1100011, TypeSB = 7'b1100111, TypeU = 7'b0110111,
TypeUJ = 7'b1101111, TypeIbreak = 7'b1110011} opcodes;
enum logic [2:0] {add = 3'b000, _and = 3'b111, slt = 3'b010} tipor;

// Gerador de clock e reset
localparam CLKPERIOD = 10000;
localparam CLKDELAY = CLKPERIOD / 2;

initial begin
	clk = 1'b1;
	Writedata = 64'd4;
	Writedatazero = 64'd0;
	Writedataone = 64'd1;
	Writedatadef = 64'd254;
	Writedataover = 64'd255;
	nrst = 1'b0;
	#(CLKPERIOD)
	nrst = 1'b1;
	#(CLKPERIOD)
	#(CLKPERIOD)
	nrst = 1'b0;
end

always #(CLKDELAY) clk = ~clk;

always_ff @(posedge clk or posedge nrst) begin

	if(nrst) 
		state <= 0;
	else begin
		state <= nextState;

	if(rdaddress > 209)	$stop;
		
	end
end

always_comb begin
	case(state)
		RESET: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd0;
			sh = 2'd0;
			offset = 6'd1;
		end

		BUSCA: begin
			Wr_pc = 1;
			Wr_instr_Reg = 1;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd1;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = MOVTOAB;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		MOVTOAB: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 1;
			Wr_A = 1;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			sh = 2'd0;
			offset = 6'd1;
				
			case(opcode)

				TypeI: begin 
					case(inst[14:12])
						3'b000: begin
							nextState = LB;
						end
						3'b001: begin
							nextState = LH;
						end
						3'b010: begin
							nextState = LW;
						end
						3'b011: begin
							nextState = LD;
						end
						3'b100: begin
							nextState = LBU;
						end
						3'b101: begin
							nextState = LHU;
						end
						3'b110: begin
							nextState = LWU;
						end
					endcase
				end

				TypeR: begin
					if(inst[31:25] == 7'b0100000) begin
						nextState = SUB;
					end
					else begin
						case(inst[14:12])

							add: begin
								nextState = ADD;
							end

							_and: begin
								nextState = AND;
							end
							
							slt: begin
								nextState = SLT;
							end

						endcase
					end
				end

				TypeIaddi: begin
					case(inst[14:12])
						3'b000: begin
							if (inst[31:0] == 32'b00000000000000000000000000010011) begin
								nextState = NOP;
							end else begin
								nextState = ADDI;
							end
						end

						3'b001: begin
							nextState = SLLI;
						end

						3'b010: begin
							nextState = SLTI;
						end

						3'b101: begin
							if (inst[31:26] == 6'd0) begin
								nextState = SRLI;
							end else begin
								nextState = SRAI;
							end
						end
					endcase
				end

				TypeS: begin
					case(inst[14:12])

						3'b111: begin
							nextState = SD;
						end

						3'b010: begin
							nextState = SW;
						end

						3'b001: begin
							nextState = SH;
						end

						3'b000: begin
							nextState = SB;
						end

					endcase
				end

				TypeU: begin
					nextState = LUI;
				end

				TypeUJ: begin
					nextState = JAL;
				end

				TypeSBbeq: begin
					nextState = BEQ;
				end

				TypeSB: begin
					case(inst[14:12])

						3'b000: begin
							nextState = JALR;
						end

						3'b001: begin
							nextState = BNE;
						end

						3'b101: begin
							nextState = BGE;
						end

						3'b100: begin
							nextState = BLT;
						end

					endcase
				end

				TypeIbreak: begin
					nextState = BREAK;
				end

				default: begin
					nextState = OPDEF;
				end

			endcase
		end

		ADD: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCO;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		SUB: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCO;
			ALUFct = 3'd2;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRBANCO: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		LD: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WAITLD;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLDREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd1;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		ADDI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOADDI;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRBANCOADDI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		SD: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRSDDATA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRSDDATA: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 1;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	

		end

		LUI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd2;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		BEQ: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
			if(igual == 1) nextState = BRANCHSUM;
			else nextState = BUSCA;
		end

		BRANCHSUM: begin
			Wr_pc = 1;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'd1;
			nextState = ENDBRANCH;
			sh = 2'd0;
			offset = 6'd1;
		end

		ENDBRANCH: begin
			Wr_pc = 0;
			Wr_instr_Reg = 1;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'd1;
			nextState = BUSCA;
			sh = 2'd0;
			offset = 6'd1;
		end

		BNE: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
			if(igual == 0) nextState = BRANCHSUM;
			else nextState = BUSCA;
		end

		BGE: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
			if(menor == 0) nextState = BRANCHSUM;
			else nextState = BUSCA;
		end

		BLT: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
			if(menor == 1) nextState = BRANCHSUM;
			else nextState = BUSCA;
		end

		AND: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCO;
			ALUFct = 3'd3;
			sh = 2'd0;
			offset = 6'd1;
		end

		SLT: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd3;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
		end

		SLTI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd3;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'b111;
			sh = 2'd0;
			offset = 6'd1;
		end

		JAL: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd1;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOJAL;
			ALUFct = 3'b000;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRBANCOJAL: begin
			Wr_pc = 1;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = ENDBRANCH;
			ALUFct = 3'b001;
			sh = 2'd0;
			offset = 6'd1;
		end

		JALR: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd1;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOJALR;
			ALUFct = 3'b000;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRBANCOJALR: begin
			Wr_pc = 1;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = ENDBRANCH;
			ALUFct = 3'b001;
			sh = 2'd0;
			offset = 6'd1;
		end

		SRLI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd3;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd1;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOSHIFT;
			ALUFct = 3'b001;
			sh = 2'd1;
			offset = inst[25:20];
		end

		WRBANCOSHIFT: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd3;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd1;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'b001;
			sh = 2'd1;
			offset = inst[25:20];
		end

		SRAI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd3;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd1;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOSHIFT;
			ALUFct = 3'b001;
			sh = 2'd2;
			offset = inst[25:20];
		end

		SLLI: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd3;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd3;
			sel_mux4 = 2'd1;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRBANCOSHIFT;
			ALUFct = 3'b001;
			sh = 2'd0;
			offset = inst[25:20];
		end

		BREAK: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BREAK;
			ALUFct = 3'd0;
			sh = 2'd0;
			offset = 6'd1;
		end

		SB: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd2;
			sel_load = 3'd0;
			nextState = WRSBDATA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRSBDATA: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 1;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd1;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd2;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		SH: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd1;
			sel_load = 3'd0;
			nextState = WRSHDATA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRSHDATA: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 1;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd1;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd1;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		SW: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRSWDATA;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRSWDATA: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 1;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd1;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LW: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd1;
			nextState = WAITLW;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLWREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd1;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LH: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd2;
			nextState = WAITLH;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLHREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd2;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LB: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd3;
			nextState = WAITLB;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLBREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd3;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LWU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd4;
			nextState = WAITLWU;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLWUREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd4;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LHU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd5;
			nextState = WAITLHU;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLHUREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd5;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		LBU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 1;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd6;
			nextState = WAITLBU;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WRLBUREG: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 1;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd6;
			nextState = WRBANCOLOAD;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		WRBANCOLOAD: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 1;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd2;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		NOP: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd0;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = BUSCA;
			ALUFct = 3'd0;
			sh = 2'd0;
			offset = 6'd1;
		end

		OPDEF: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 1;
			Wr_causa = 1;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd1;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd1;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WROPDEF;
			ALUFct = 3'd2;
			sh = 2'd0;
			offset = 6'd1;
		end

		WROPDEF: begin
			Wr_pc = 1;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd1;
			sel_store = 2'd0;
			sel_load = 3'd6;
			nextState = ENDBRANCH;
			ALUFct = 3'd0;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		OVER: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 1;
			Wr_causa = 1;
			sel_mux1 = 2'd0;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd1;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd1;
			sel_mux7 = 2'd2;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WROVER;
			ALUFct = 3'd2;
			sh = 2'd0;
			offset = 6'd1;
		end

		WROVER: begin
			Wr_pc = 1;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd1;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd1;
			sel_store = 2'd0;
			sel_load = 3'd6;
			nextState = ENDBRANCH;
			ALUFct = 3'd0;	
			sh = 2'd0;
			offset = 6'd1;	
		end

		WAITLD: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLDREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLW: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLWREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLH: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLHREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLB: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLBREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLWU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLWUREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLHU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLHUREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end

		WAITLBU: begin
			Wr_pc = 0;
			Wr_instr_Reg = 0;
			Wr_meminst = 0;
			Wr_B = 0;
			Wr_A = 0;
			Wr_banco = 0;
			Wr_aluout = 0;
			Wr_memdata = 0;
			Wr_memreg = 0;
			Wr_epc = 0;
			Wr_causa = 0;
			sel_mux1 = 2'd1;
			sel_mux2 = 2'd0;
			sel_mux3 = 2'd2;
			sel_mux4 = 2'd0;
			sel_mux5 = 2'd0;
			sel_mux6 = 2'd0;
			sel_mux7 = 2'd0;
			sel_mux8 = 2'd0;
			sel_store = 2'd0;
			sel_load = 3'd0;
			nextState = WRLBUREG;
			ALUFct = 3'd1;
			sh = 2'd0;
			offset = 6'd1;
		end
	endcase

	if(over && state > 6'd2) begin
		nextState = OVER;
	end
end

endmodule


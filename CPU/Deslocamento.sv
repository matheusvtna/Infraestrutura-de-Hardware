module Deslocamento (	
					input logic [1:0] Shift,
					input logic signed [63:0] Entrada,
					input logic [5:0] N,
					output logic [63:0]Saida
					);


always_comb
	case(Shift)
		2'b00: //Shift a Esquerda lógico de N Vezes 
			Saida = Entrada << N;
		2'b01: //Shift a Direita lógico de N Vezes
			Saida = Entrada >> N;
		2'b10: //Shift a Direita Aritmético de N Vezes
			Saida = Entrada >>> N;
		default: //O restante dos valores não faz NADA
			Saida = Entrada;
	endcase // Shift


endmodule:Deslocamento
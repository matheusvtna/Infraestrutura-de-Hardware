module mux64bits
	(input [1:0] sel,
	 input logic [63:0] in0,
	 input logic [63:0] in1,
	 input logic [63:0] in2,
	 input logic [63:0] in3,
	 output logic [63:0] out);

	always_comb begin
		case(sel)
			0: begin
				out = in0;
			end
			1: begin
				out = in1;
			end
			2: begin
				out = in2;
			end
			3: begin
				out = in3;
			end
		endcase
	end
endmodule


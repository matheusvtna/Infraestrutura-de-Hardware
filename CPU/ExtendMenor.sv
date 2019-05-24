module ExtendMenor
	(input logic in,
	 output logic [63:0] out);

	always_comb begin
		out = 64'd0;
		out[0] = in;
	end
endmodule

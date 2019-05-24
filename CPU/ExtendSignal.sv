module ExtendSignal
	(input logic [31:0] in,
	 output logic [63:0] out);

	always_comb begin
		case(in[6:0])
			7'b0010011: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[11] = in[31];
				out[10] = in[30];
				out[9] = in[29];
				out[8] = in[28];
				out[7] = in[27];
				out[6] = in[26];
				out[5] = in[25];
				out[4] = in[24];
				out[3] = in[23];
				out[2] = in[22];
				out[1] = in[21];
				out[0] = in[20];
			end

			7'b0000011: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[11] = in[31];
				out[10] = in[30];
				out[9] = in[29];
				out[8] = in[28];
				out[7] = in[27];
				out[6] = in[26];
				out[5] = in[25];
				out[4] = in[24];
				out[3] = in[23];
				out[2] = in[22];
				out[1] = in[21];
				out[0] = in[20];
			end

			7'b0100011: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[11] = in[31];
				out[10] = in[30];
				out[9] = in[29];
				out[8] = in[28];
				out[7] = in[27];
				out[6] = in[26];
				out[5] = in[25];
				out[4] = in[11];
				out[3] = in[10];
				out[2] = in[9];
				out[1] = in[8];
				out[0] = in[7];
			end

			7'b0110111: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[31] = in[31];
				out[30] = in[30];
				out[29] = in[29];
				out[28] = in[28];
				out[27] = in[27];
				out[26] = in[26];
				out[25] = in[25];
				out[24] = in[24];
				out[23] = in[23];
				out[22] = in[22];
				out[21] = in[21];
				out[20] = in[20];
				out[19] = in[19];
				out[18] = in[18];
				out[17] = in[17];
				out[16] = in[16];
				out[15] = in[15];
				out[14] = in[14];
				out[13] = in[13];
				out[12] = in[12];
				out[11] = 0;
				out[10] = 0;
				out[9] = 0;
				out[8] = 0;
				out[7] = 0;
				out[6] = 0;
				out[5] = 0;
				out[4] = 0;
				out[3] = 0;
				out[2] = 0;
				out[1] = 0;
				out[0] = 0;
			end


			7'b1100011: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[12] = in[31];
				out[11] = in[7];
				out[10] = in[30];
				out[9] = in[29];
				out[8] = in[28];
				out[7] = in[27];
				out[6] = in[26];
				out[5] = in[25];
				out[4] = in[11];
				out[3] = in[10];
				out[2] = in[9];
				out[1] = in[8];
				out[0] = 0;

			end

			7'b1100111: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;
				
				case(in[14:12])
					3'b000: begin
						out[11] = in[31];
						out[10] = in[30];
						out[9] = in[29];
						out[8] = in[28];
						out[7] = in[27];
						out[6] = in[26];
						out[5] = in[25];
						out[4] = in[24];
						out[3] = in[23];
						out[2] = in[22];
						out[1] = in[21];
						out[0] = in[20];
					end
					default: begin
						out[12] = in[31];
						out[11] = in[7];
						out[10] = in[30];
						out[9] = in[29];
						out[8] = in[28];
						out[7] = in[27];
						out[6] = in[26];
						out[5] = in[25];
						out[4] = in[11];
						out[3] = in[10];
						out[2] = in[9];
						out[1] = in[8];
						out[0] = 0;
					end
				endcase
			end


			7'b1101111: begin
				if (in[31] == 1)
					out = 64'b1111111111111111111111111111111111111111111111111111111111111111;	
				else
					out = 64'd0;

				out[20] = in[31];
				out[19] = in[19];
				out[18] = in[18];
				out[17] = in[17];
				out[16] = in[16];
				out[15] = in[15];
				out[14] = in[14];
				out[13] = in[13];
				out[12] = in[12];
				out[11] = in[20];
				out[10] = in[30];
				out[9] = in[29];
				out[8] = in[28];
				out[7] = in[27];
				out[6] = in[26];
				out[5] = in[25];
				out[4] = in[24];
				out[3] = in[23];
				out[2] = in[22];
				out[1] = in[21];
				out[0] = 0;

			end
		endcase
	end
endmodule


/*
 *  NanoRV32 -- A small RV32I[MC] core capable of running RTOS
 *
 *  Modifications authored by Elvis Fox <elvisfox@github.com>
 *
 *  nanoRV32 is a fork of PicoRV32:
 *  https://github.com/YosysHQ/picorv32
 *  Copyright (C) 2015  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PE RFORMANCE OF THIS SOFTWARE.
 *
 */

module nanorv32_timer(
	// clock and reset
	input							resetn,
	input							clk,

	// I/O access
	input		[1:0]				io_mtime_valid,
	input		[1:0]				io_mtimecmp_valid,
	output reg						io_ready,
	input		[31:0]				io_wdata,
	input		[3:0]				io_wstrb,
	output reg	[31:0]				io_rdata,

  input  [63:0]         inst_cnt,
	// interrupt request
	output wire						mtip /*verilator public_flat_rw*/
);

	parameter	[0:0]				ENABLE_MTIMECMP = 1;

	// 64-bit counter
	reg			[31:0]  		  clk_div = 32'd19;
	reg			[31:0]  		  clk_cnt;
	reg			[31:0]				cnt[1:0];
	reg								    inc_cnt1;

	always @(posedge clk) begin
		if(~resetn) begin
			cnt[1] <= 0;
			cnt[0] <= 0;
			inc_cnt1 <= 1'b0;
			clk_cnt <= 32'd0;
			clk_div <= 32'd19;
		end
		else begin
		  if (clk_cnt == clk_div) begin
		      // clk is 80M
		      // Use 4M for timer counter
		      clk_cnt <= 32'h0;
					// Lower part of the counter
					cnt[0] <= cnt[0] + 1'b1;
					inc_cnt1 <= cnt[0] == 32'hffff_fffe;

					// Upper part of the counter
					if(inc_cnt1)
						cnt[1] <= cnt[1] + 1'b1;
			end else begin
			    clk_cnt <= clk_cnt + 32'h1;
			end
		end
	end

	// Interrupt comparator
	reg			[31:0]				cmp_val[1:0];

	wire signed	[32:0]				cmp_sub[1:0];
	reg			[1:0]				cmp_less;
	reg			[1:1]				cmp_equal;

	assign		cmp_sub[1]			= cnt[1] - cmp_val[1];	// subtraction forces synthesiser to use carry chain
	assign		cmp_sub[0]			= cnt[0] - cmp_val[0];


	always @(posedge clk) begin
		if(~resetn) begin
			cmp_less  <= 2'b00;
			cmp_equal <= 1'b0;
		end
		else begin
			cmp_less  <= {cmp_sub[1][32], cmp_sub[0][32]};
			cmp_equal <= cnt[1] == cmp_val[1];
		end
	end

	assign		mtip	=  (cmp_equal ? !cmp_less[0] : !cmp_less[1]) && ENABLE_MTIMECMP;

  // Use executed instruction counter to trigger timer interrupt
	// assign		mtip				= (inst_cnt >= {cmp_val[1],cmp_val[0]}) ? 1 : 0;

	// I/O interface
	integer i;
	always @(posedge clk) begin
		if(~resetn) begin
			for(i = 0; i < 2; i = i + 1)
				cmp_val[i] <= 0;
		end
		else begin
			for(i = 0; i < 2; i = i + 1) begin
				if(io_mtimecmp_valid[i] && ENABLE_MTIMECMP) begin
					if(io_wstrb[3])
						cmp_val[i][31:24]	<= io_wdata[31:24];
					if(io_wstrb[2])
						cmp_val[i][23:16]	<= io_wdata[23:16];
					if(io_wstrb[1])
						cmp_val[i][15:8]	<= io_wdata[15:8];
					if(io_wstrb[0])
						cmp_val[i][7:0]		<= io_wdata[7:0];
				end
			end
		end
	end

	always @* begin
		io_ready = |{io_mtimecmp_valid, io_mtime_valid};

		case({io_mtimecmp_valid, io_mtime_valid})
			4'b1000:	io_rdata = ENABLE_MTIMECMP ? cmp_val[1] : 32'h0000_0000;
			4'b0100:	io_rdata = ENABLE_MTIMECMP ? cmp_val[0] : 32'h0000_0000;
			// Use executed instruction counter as timer counter
			//4'b0010:	io_rdata = inst_cnt[63:32];
			//4'b0001:	io_rdata = inst_cnt[31:0];
			4'b0010:	io_rdata = cnt[1];
			4'b0001:	io_rdata = cnt[0];
			default:	io_rdata = 32'hxxxx_xxxx;
		endcase
	end

endmodule

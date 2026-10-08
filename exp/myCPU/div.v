// Blocking 32-bit divider: radix-4 restoring division with a carry-select
// datapath.  Phase diagram, borrow chain and depth accounting live in
// docs/exp12-13_csr.md §5.9.
//
//   start (1-cycle pulse) -> busy ... -> done (1-cycle pulse)
//   quot / rem are valid in the cycle where done is high
//
//   - magnitudes + final sign fix-up: quotient truncates toward zero,
//     remainder keeps the sign of the dividend
//   - is_signed selects div.w/mod.w (1) vs div.wu/mod.wu (0)
//   - divisor == 0 gives quot = 32'hffffffff, rem = dividend
//
// 21 cycles: start (low half of |src1|/|src2|) -> CNT_MAG (high half) ->
// CNT_D3/CNT_D3B (3*b, one half each) -> 16 iterations (2 bits each) ->
// CNT_FIN/CNT_FIN2 (sign fix-up, one half each, + zero_div override, latch
// quot/rem, raise done).  Every 32/34-bit add is cut into two 16/17-bit beats
// with the carry parked in a register, so no full-width chain fits in one
// cycle: carry-select alone gets flattened back to a ripple by the tool.
//
// Depth work: 3*b is latched so it is not a second adder in series with the
// t3 subtractor; the 34-bit subtractors are carry-select (chunks 12/12/8 + a
// 2-bit head, final selects fused into one LUT each); d2 is pure wiring.

module div(
    input  wire        clk,
    input  wire        reset,
    input  wire        start,
    input  wire        is_signed,
    input  wire [31:0] src1,       // dividend
    input  wire [31:0] src2,       // divisor
    output wire        busy,
    output wire        done,
    output wire [31:0] quot,
    output wire [31:0] rem
);

localparam CNT_MAG  = 5'd0;    // |src1| / |src2| high half (low half latched by start)
localparam CNT_D3   = 5'd1;    // 3*b low half
localparam CNT_D3B  = 5'd2;    // 3*b high half
localparam CNT_LAST = 5'd18;   // last radix-4 iteration (16 iterations: 3..18)
localparam CNT_FIN  = 5'd19;   // sign fix-up low half + carry registers
localparam CNT_FIN2 = 5'd20;   // sign fix-up high half + zero_div, register outputs

reg         busy_r, done_r;
reg  [ 4:0] cnt;
reg  [31:0] a_mag, b_mag;      // |dividend|, |divisor|
reg  [31:0] dq;                // dividend shift register (2 bits per iteration)
reg  [31:0] rem_r;             // partial remainder, kept in [0, b_mag)
reg  [31:0] q_acc;             // quotient, 2 bits shifted in per iteration
reg  [33:0] d3;                // 3*b_mag, latched once (CNT_D3 / CNT_D3B)
reg  [16:0] d3_lo;             // 3*b bits 16:0; d3_cy is the carry into bit 17
reg         d3_cy;
reg  [15:0] m1_lo, m1_hi;      // |src1| halves in flight start -> CNT_MAG
reg  [15:0] m2_lo, m2_hi;      // |src2| halves in flight start -> CNT_MAG
reg         m1_cy, m2_cy;      // carry out of the low-half negation
reg  [15:0] q_lo, r_lo, a_lo;  // sign fix-up low halves
reg         q_cy, r_cy, a_cy;  // carry into each high half
reg         q_sign, r_sign, zero_div;
reg  [31:0] quot_r, rem_r_out;

// ---------------- start cycle: low half of the unsigned magnitudes ---------
// Only bits 15:0 are negated here; CNT_MAG finishes the high half from the
// registered carry, so ~src+1 is never one full-width chain in a single cycle.
wire        s1 = is_signed & src1[31];
wire        s2 = is_signed & src2[31];
wire [15:0] c1l = src1[15:0] ^ {16{s1}};
wire [16:0] l1  = {1'b0, c1l} + {16'b0, s1};
wire [15:0] c1h = src1[31:16] ^ {16{s1}};
wire [15:0] c2l = src2[15:0] ^ {16{s2}};
wire [16:0] l2  = {1'b0, c2l} + {16'b0, s2};
wire [15:0] c2h = src2[31:16] ^ {16{s2}};

// ---------------- CNT_MAG: high half of |src1| / |src2| --------------------
wire [31:0] src1_mag = {m1_hi + {15'b0, m1_cy}, m1_lo};
wire [31:0] src2_mag = {m2_hi + {15'b0, m2_cy}, m2_lo};

// ---------------- CNT_D3 / CNT_D3B: d3 = 2*b + b, one half per cycle -------
wire [33:0] b2x  = {1'b0, b_mag, 1'b0};                         // 2*b
wire [33:0] b1x  = {2'b0, b_mag};                               // b
wire [17:0] d3_l = {1'b0, b1x[16:0]} + {1'b0, b2x[16:0]};       // 3*b bits 17:0
wire [16:0] d3_h = b1x[33:17] + b2x[33:17] + {16'b0, d3_cy};    // carry in, bits 33:17

// ---------------- iteration combinational logic ---------------------------
wire [33:0] shifted = {rem_r, dq[31:30]};
wire [33:0] y1      = {2'b0, b_mag};                            // b
wire [33:0] y2      = {1'b0, b_mag, 1'b0};                      // 2*b, wiring

wire [32:0] cs1 = sub34_cs(shifted, y1);
wire [32:0] cs2 = sub34_cs(shifted, y2);
wire [32:0] cs3 = sub34_cs(shifted, d3);

wire        ge1 = ~cs1[32];
wire        ge2 = ~cs2[32];
wire        ge3 = ~cs3[32];
wire [ 1:0] dig = ge3 ? 2'd3 : ge2 ? 2'd2 : ge1 ? 2'd1 : 2'd0;
wire [31:0] r3  = (dig == 2'd3) ? cs3[31:0]
                : (dig == 2'd2) ? cs2[31:0]
                : (dig == 2'd1) ? cs1[31:0] : shifted[31:0];

// ---------------- CNT_FIN: low half of the sign fix-up ---------------------
wire [31:0] qc = q_acc ^ {32{q_sign}};
wire [16:0] ql = {1'b0, qc[15:0]} + {16'b0, q_sign};
wire [31:0] rc = rem_r ^ {32{r_sign}};
wire [16:0] rl = {1'b0, rc[15:0]} + {16'b0, r_sign};
// divisor == 0 -> rem = dividend, i.e. the sign-fixed |dividend|
wire [31:0] ac = a_mag ^ {32{r_sign}};
wire [16:0] al = {1'b0, ac[15:0]} + {16'b0, r_sign};

// ---------------- CNT_FIN2: high half + zero_div override ------------------
wire [15:0] qh = qc[31:16] + {15'b0, q_cy};
wire [15:0] rh = rc[31:16] + {15'b0, r_cy};
wire [15:0] ah = ac[31:16] + {15'b0, a_cy};

wire [31:0] q_mag = {qh, q_lo};
wire [31:0] r_mag = {rh, r_lo};
wire [31:0] a_sgn = {ah, a_lo};

wire [31:0] quot_w = zero_div ? 32'hffffffff : q_mag;
wire [31:0] rem_w  = zero_div ? a_sgn        : r_mag;

// ---------------- control -------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        busy_r <= 1'b0;
        done_r <= 1'b0;
        cnt    <= 5'd0;
    end
    else begin
        done_r <= 1'b0;
        if (!busy_r) begin
            if (start) begin
                busy_r   <= 1'b1;
                cnt      <= CNT_MAG;
                m1_lo    <= l1[15:0];
                m1_cy    <= l1[16];
                m1_hi    <= c1h;
                m2_lo    <= l2[15:0];
                m2_cy    <= l2[16];
                m2_hi    <= c2h;
                q_sign   <= is_signed & (src1[31] ^ src2[31]);
                r_sign   <= is_signed & src1[31];
                zero_div <= (src2 == 32'd0);
                q_acc    <= 32'd0;
                rem_r    <= 32'd0;
            end
        end
        else if (cnt == CNT_FIN2) begin
            busy_r    <= 1'b0;
            done_r    <= 1'b1;
            quot_r    <= quot_w;
            rem_r_out <= rem_w;
        end
        else if (cnt == CNT_FIN) begin
            cnt  <= CNT_FIN2;
            q_lo <= ql[15:0]; q_cy <= ql[16];
            r_lo <= rl[15:0]; r_cy <= rl[16];
            a_lo <= al[15:0]; a_cy <= al[16];
        end
        else if (cnt == CNT_MAG) begin
            cnt   <= CNT_D3;
            a_mag <= src1_mag;
            b_mag <= src2_mag;
            dq    <= src1_mag;
        end
        else if (cnt == CNT_D3) begin
            cnt   <= CNT_D3B;
            d3_lo <= d3_l[16:0];
            d3_cy <= d3_l[17];
        end
        else if (cnt == CNT_D3B) begin
            cnt <= cnt + 5'd1;
            d3  <= {d3_h, d3_lo};
        end
        else begin
            cnt   <= (cnt == CNT_LAST) ? CNT_FIN : (cnt + 5'd1);
            q_acc <= {q_acc[29:0], dig};
            dq    <= {dq[29:0], 2'b00};
            rem_r <= r3;
        end
    end
end

assign busy = busy_r;
assign done = done_r;
assign quot = quot_r;
assign rem  = rem_r_out;

// 34-bit carry-select borrow subtractor: chunks 12/12/8 + a 2-bit head, the
// upper chunks computed twice (borrow-in 0/1) and selected by the lower
// borrows, with the final selects fused into one LUT each.
function [32:0] sub34_cs;
    input [33:0] x;
    input [33:0] y;
    reg [12:0] w0, w1a, w1b;      // bits 11:0 and 23:12, borrow in bit [12]
    reg [ 8:0] w2a, w2b;          // bits 31:24, borrow in bit [8]
    reg        b2;                // borrow out of bits 31:24
    begin
        w0  = {1'b0, x[11:0] } - {1'b0, y[11:0] };
        w1a = {1'b0, x[23:12]} - {1'b0, y[23:12]};
        w1b = {1'b0, x[23:12]} - {1'b0, y[23:12]} - 13'd1;
        w2a = {1'b0, x[31:24]} - {1'b0, y[31:24]};
        w2b = {1'b0, x[31:24]} - {1'b0, y[31:24]} - 9'd1;

        // five inputs -> one LUT, so the second select does not add a level
        b2 = w0[12] ? (w1b[12] ? w2b[8] : w2a[8])
                    : (w1a[12] ? w2b[8] : w2a[8]);

        sub34_cs[11:0]  = w0[11:0];
        sub34_cs[23:12] = w0[12] ? w1b[11:0] : w1a[11:0];
        sub34_cs[31:24] = w0[12] ? (w1b[12] ? w2b[7:0] : w2a[7:0])
                                 : (w1a[12] ? w2b[7:0] : w2a[7:0]);
        sub34_cs[32]    = (x[33:32] < y[33:32])
                        | ((x[33:32] == y[33:32]) & b2);
    end
endfunction

endmodule

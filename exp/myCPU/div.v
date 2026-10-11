// Blocking 32-bit divider: radix-4 restoring division, 18 cycles.
//
//   start (1-cycle pulse) -> busy ... -> done (1-cycle pulse)
//   quot / rem are valid in the cycle where done is high
//
//   - magnitudes + final sign fix-up: quotient truncates toward zero,
//     remainder keeps the sign of the dividend
//   - is_signed selects div.w/mod.w (1) vs div.wu/mod.wu (0)
//   - divisor == 0 gives quot = 32'hffffffff, rem = dividend
//
// Phases: start (low half of |src|) -> CNT_MAG (high half of |src| AND 3*b) ->
// 16 iterations (2 bits each) -> CNT_FIN (sign fix-up, zero_div override, latch
// outputs, raise done).
//
// The split is not arbitrary: the start cycle is where the operands arrive over
// the EX -> div cross-module routing, so it keeps only the 17-bit low-half
// negation.  The high half and the 3*b CPA that depends on it are both local
// wiring, so they share the next cycle at no cost.
// Depth accounting and the timing report: docs/ex_opt_attempt/div_pass_through_review.md

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

localparam CNT_MAG  = 5'd0;    // |src1| / |src2| high half + 3*b, both local
localparam CNT_LAST = 5'd16;   // last radix-4 iteration (16 iterations: 1..16)
localparam CNT_FIN  = 5'd17;   // sign fix-up + zero_div override + register outputs

reg         busy_r, done_r;
reg  [ 4:0] cnt;
reg  [31:0] a_mag, b_mag;      // |dividend|, |divisor|
reg  [31:0] dq;                // dividend shift register (2 bits per iteration)
reg  [31:0] rem_r;             // partial remainder, kept in [0, b_mag)
reg  [31:0] q_acc;             // quotient, 2 bits shifted in per iteration
reg  [33:0] d3;                // 3*b_mag, latched once
reg  [15:0] m1_lo, m1_hi;      // |src1| halves in flight start -> CNT_MAG
reg  [15:0] m2_lo, m2_hi;      // |src2| halves in flight start -> CNT_MAG
reg         m1_cy, m2_cy;      // carry out of the low-half negation
reg         q_sign, r_sign, zero_div;
reg  [31:0] quot_r, rem_r_out;

// ---------------- start cycle: low half of the unsigned magnitudes ---------
// Only bits 15:0 are negated here; CNT_MAG finishes the high half from the
// registered carry, so ~src+1 is never one full-width chain in the cycle that
// has to absorb the operand routing.
wire        s1, s2;
wire [15:0] c1l, c1h, c2l, c2h;
wire [16:0] l1, l2;

assign s1  = is_signed & src1[31];
assign s2  = is_signed & src2[31];
assign c1l = src1[15:0]  ^ {16{s1}};
assign c1h = src1[31:16] ^ {16{s1}};
assign c2l = src2[15:0]  ^ {16{s2}};
assign c2h = src2[31:16] ^ {16{s2}};
assign l1  = {1'b0, c1l} + {16'b0, s1};
assign l2  = {1'b0, c2l} + {16'b0, s2};

// ---------------- CNT_MAG: high halves + 3*b_mag --------------------------
wire [31:0] src1_mag, src2_mag;
wire [33:0] d3_w;

assign src1_mag = {m1_hi + {15'b0, m1_cy}, m1_lo};
assign src2_mag = {m2_hi + {15'b0, m2_cy}, m2_lo};

// 3*b as 2*b + b: only the +2*b is wiring, so this is a single 34-bit CPA.  It
// sits behind the 16-bit high-half add above, not behind a full-width negation,
// which is what keeps it off the critical path.
assign d3_w = {2'b0, src2_mag} + {1'b0, src2_mag, 1'b0};

// ---------------- iteration combinational logic ---------------------------
wire [33:0] shifted;
wire [33:0] y1;
wire [33:0] y2;
wire [32:0] cs1;
wire [32:0] cs2;
wire [32:0] cs3;
wire        ge1, ge2, ge3;
wire [ 1:0] dig;
wire [31:0] r3;

assign shifted = {rem_r, dq[31:30]};
assign y1      = {2'b0, b_mag};                            // b
assign y2      = {1'b0, b_mag, 1'b0};                      // 2*b, wiring

assign cs1 = sub34_cs(shifted, y1);
assign cs2 = sub34_cs(shifted, y2);
assign cs3 = sub34_cs(shifted, d3);

assign ge1 = ~cs1[32];
assign ge2 = ~cs2[32];
assign ge3 = ~cs3[32];
assign dig = ge3 ? 2'd3 : ge2 ? 2'd2 : ge1 ? 2'd1 : 2'd0;
assign r3  = (dig == 2'd3) ? cs3[31:0]
           : (dig == 2'd2) ? cs2[31:0]
           : (dig == 2'd1) ? cs1[31:0] : shifted[31:0];

// ---------------- CNT_FIN: sign fix-up + output selection ------------------
wire [31:0] qc, rc, ac;
wire [31:0] q_mag, r_mag, a_sgn;
wire [31:0] quot_w, rem_w;

assign qc = q_acc ^ {32{q_sign}};
assign rc = rem_r ^ {32{r_sign}};
assign ac = a_mag ^ {32{r_sign}};

assign q_mag = qc + {31'b0, q_sign};
assign r_mag = rc + {31'b0, r_sign};
// divisor == 0 -> rem = dividend, i.e. the sign-fixed |dividend|
assign a_sgn = ac + {31'b0, r_sign};

assign quot_w = zero_div ? 32'hffffffff : q_mag;
assign rem_w  = zero_div ? a_sgn        : r_mag;

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
        else if (cnt == CNT_FIN) begin
            busy_r    <= 1'b0;
            done_r    <= 1'b1;
            quot_r    <= quot_w;
            rem_r_out <= rem_w;
        end
        else if (cnt == CNT_MAG) begin
            cnt   <= cnt + 5'd1;
            a_mag <= src1_mag;
            b_mag <= src2_mag;
            d3    <= d3_w;
            dq    <= src1_mag;
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
// borrows, with the final selects fused into one LUT each.  Returns the low 32
// bits of the difference plus the borrow in bit 32.
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

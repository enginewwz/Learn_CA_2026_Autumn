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

localparam CNT_LAST = 5'd15;   // last iteration index (16 iterations)
localparam CNT_FIN  = 5'd16;   // one extra cycle to register the outputs

reg         busy_r, done_r;
reg  [ 4:0] cnt;
reg  [31:0] a_mag, b_mag;      // |dividend|, |divisor|
reg  [31:0] dq;                // dividend shift register
reg  [33:0] rem_r;             // partial remainder, kept < b_mag
reg  [31:0] q_acc;             // quotient, shifted in 2 bits per iteration
reg         q_sign, r_sign, zero_div;
reg  [31:0] quot_r, rem_r_out;

// radix-4 restoring digit selection.
// t1/t2/t3 are computed by three PARALLEL subtractors (not cascaded); each is
// 35 bits wide so bit[34] is the borrow, telling whether the digit is >= 1/2/3.
wire [33:0] b34     = {2'b0, b_mag};
wire [33:0] d2      = {b34[32:0], 1'b0};     // 2*divisor
wire [33:0] d3      = d2 + b34;              // 3*divisor
wire [33:0] shifted = {rem_r[31:0], dq[31:30]};

wire [34:0] t1 = {1'b0, shifted} - {1'b0, b34};
wire [34:0] t2 = {1'b0, shifted} - {1'b0, d2};
wire [34:0] t3 = {1'b0, shifted} - {1'b0, d3};
wire        ge1 = ~t1[34];
wire        ge2 = ~t2[34];
wire        ge3 = ~t3[34];
wire [ 1:0] dig  = ge3 ? 2'd3 : ge2 ? 2'd2 : ge1 ? 2'd1 : 2'd0;
wire [33:0] r3   = ge3 ? t3[33:0] : ge2 ? t2[33:0] : ge1 ? t1[33:0] : shifted;

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
                cnt      <= 5'd0;
                a_mag    <= (is_signed && src1[31]) ? (~src1 + 32'd1) : src1;
                b_mag    <= (is_signed && src2[31]) ? (~src2 + 32'd1) : src2;
                dq       <= (is_signed && src1[31]) ? (~src1 + 32'd1) : src1;
                q_sign   <= is_signed & (src1[31] ^ src2[31]);
                r_sign   <= is_signed & src1[31];
                zero_div <= (src2 == 32'd0);
                q_acc    <= 32'd0;
                rem_r    <= 34'd0;
            end
        end
        else if (cnt == CNT_FIN) begin
            busy_r    <= 1'b0;
            done_r    <= 1'b1;
            quot_r    <= zero_div ? 32'hffffffff
                                  : (q_sign ? (~q_acc + 32'd1) : q_acc);
            rem_r_out <= zero_div ? (r_sign ? (~a_mag + 32'd1) : a_mag)
                                  : (r_sign ? (~rem_r[31:0] + 32'd1) : rem_r[31:0]);
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

endmodule

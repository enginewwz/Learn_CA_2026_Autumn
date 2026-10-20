module mmu(
    input  wire        clk,
    input  wire        reset,

    // CPU side: fetch
    input  wire        if_req,          // = inst_req: a legal fetch address is on if_va
    input  wire [31:0] if_va,           // = pc[IF]: the address taken at the last edge
    input  wire        if_allow,        // = allow[IF]: pc[IF] <- nextpc at this edge
    input  wire        if_flush,        // = flush[IF]: this fetch is killed
    output wire        if_pending,      // -> stall[IF]
    output wire [31:0] if_rdata,

    // CPU side: load / store
    input  wire        ld_pre,          // a load in EX that enters MEM at this edge
    input  wire        data_req,        // the MEM slot holds a real access
    input  wire [31:0] data_va,         // = result[MEM]
    input  wire [ 3:0] data_we,         // byte strobes; 4'b0 = read
    input  wire [31:0] data_wdata,
    input  wire        data_abort,      // = mem_excp: this access was found illegal
    input  wire        mem_allow,       // = allow[MEM]
    output wire        mem_pending,     // -> stall[MEM]
    output wire [31:0] data_rdata,

    // AXI3, straight through the internal sram_axi_bridge
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,

    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,

    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready,

    // CSR sideband (tied 0 until the TLB CSRs land)
    input  wire [ 1:0] crmd_plv,        // -> bridge arprot[0]/awprot[0]; later the PLV check
    input  wire [ 9:0] asid
);

localparam IDLE    = 3'b001;
localparam TLB_ACC = 3'b010;
localparam BRI_ACC = 3'b100;

reg [2:0] if_state,      mem_state;
reg [2:0] next_if_state, next_mem_state;

// TLB look-up
wire        if_found, mem_found;
wire [19:0] if_ppn,   mem_ppn;
wire [ 5:0] if_ps,    mem_ps;

wire [31:0] if_pa  = (if_ps  == 6'd21) ? {if_ppn[19:9],  if_va[20:0]}
                                       : {if_ppn[19:0],  if_va[11:0]};
wire [31:0] mem_pa = (mem_ps == 6'd21) ? {mem_ppn[19:9], data_va[20:0]}
                                       : {mem_ppn[19:0], data_va[11:0]};

// TODO(exp19): CRMD.PG/DA, DMW and the TLB exceptions
wire        if_xlat_err  = 1'b0;
wire        mem_xlat_err = 1'b0;

// bridge side commands
wire        bridge_if_req, bridge_if_flush;
wire [31:0] bridge_if_addr;
wire        bridge_if_pending;
wire        bridge_data_req;
wire [31:0] bridge_data_addr, bridge_data_wdata;
wire [ 3:0] bridge_data_we;
wire        bridge_mem_pending;

assign bridge_if_addr   = if_found  ? if_pa  : if_va;
assign bridge_data_addr = mem_found ? mem_pa : data_va;

// fetch path
wire if_live = (if_state != IDLE);
wire if_done = (if_state == BRI_ACC) & ~ bridge_if_pending;
wire if_take = if_req & if_allow;

assign if_pending = if_live & ~ if_done;

always @(*) begin
    case (if_state)
        IDLE: begin
            if (if_take) next_if_state = TLB_ACC;
            else         next_if_state = IDLE;
        end
        TLB_ACC: begin
            if (if_flush | if_xlat_err) next_if_state = IDLE;
            else                        next_if_state = BRI_ACC;
        end
        BRI_ACC: begin
            if (~ bridge_if_pending) begin
                if (if_take) next_if_state = TLB_ACC;
                else         next_if_state = IDLE;
            end
            else next_if_state = BRI_ACC;
        end
        default: next_if_state = IDLE;
    endcase
end

assign bridge_if_req   = (if_state == TLB_ACC);
assign bridge_if_flush = if_flush;

// mem path
wire mem_live  = (mem_state != IDLE);
wire mem_take  = (ld_pre & mem_allow) | (data_req & (mem_state == IDLE));
wire mem_done  = mem_live & ~ bridge_mem_pending;   // write: taken; read: data back
wire mem_abort = data_abort & mem_live;

assign mem_pending       = (mem_live | data_req) & ~ (mem_done | mem_abort);
assign bridge_data_req   = mem_live & ~ data_abort;
assign bridge_data_we    = data_we;
assign bridge_data_wdata = data_wdata;

always @(*) begin
    case (mem_state)
        IDLE: begin
            if (mem_take) next_mem_state = TLB_ACC;
            else        next_mem_state = IDLE;
        end
        TLB_ACC: begin
            if (data_abort | mem_xlat_err) next_mem_state = IDLE;
            else if (mem_done) begin
                if (ld_pre & mem_allow) next_mem_state = TLB_ACC;
                else                    next_mem_state = IDLE;
            end
            else next_mem_state = BRI_ACC;
        end
        BRI_ACC: begin
            if (mem_done | mem_abort) begin
                if (ld_pre & mem_allow) next_mem_state = TLB_ACC;
                else                    next_mem_state = IDLE;
            end
            else next_mem_state = BRI_ACC;
        end
        default: next_mem_state = IDLE;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        if_state <= IDLE;
        mem_state  <= IDLE;
    end
    else begin
        if_state <= next_if_state;
        mem_state  <= next_mem_state;
    end
end

// TLB
tlb u_tlb(
    .clk          (clk          ),
    .reset        (reset        ),

    .s0_vppn      (if_va[31:13] ),
    .s0_va_bit12  (if_va[12]    ),
    .s0_asid      (asid         ),
    .s0_found     (if_found     ),
    .s0_index     (             ),
    .s0_ppn       (if_ppn       ),
    .s0_ps        (if_ps        ),
    .s0_plv       (             ),
    .s0_mat       (             ),
    .s0_d         (             ),
    .s0_v         (             ),

    .s1_vppn      (data_va[31:13]),
    .s1_va_bit12  (data_va[12]  ),
    .s1_asid      (asid         ),
    .s1_found     (mem_found    ),
    .s1_index     (             ),
    .s1_ppn       (mem_ppn      ),
    .s1_ps        (mem_ps       ),
    .s1_plv       (             ),
    .s1_mat       (             ),
    .s1_d         (             ),
    .s1_v         (             ),

    // no TLB instruction / TLB CSR yet
    .invtlb_valid (1'b0),
    .invtlb_op    (5'h0),

    .we           (1'b0),
    .w_index      (4'h0),
    .w_e          (1'b0),
    .w_vppn       (19'h0),
    .w_ps         (6'h0),
    .w_asid       (10'h0),
    .w_g          (1'b0),
    .w_ppn0       (20'h0),
    .w_plv0       (2'h0),
    .w_mat0       (2'h0),
    .w_d0         (1'b0),
    .w_v0         (1'b0),
    .w_ppn1       (20'h0),
    .w_plv1       (2'h0),
    .w_mat1       (2'h0),
    .w_d1         (1'b0),
    .w_v1         (1'b0),

    .r_index      (4'h0)
);

// bridge
sram_axi_bridge u_bridge(
    .clk          (clk                ),
    .reset        (reset              ),

    .inst_req     (bridge_if_req      ),
    .inst_addr    (bridge_if_addr     ),
    .inst_flush   (bridge_if_flush    ),
    .inst_rdata   (if_rdata           ),
    .inst_pending (bridge_if_pending  ),

    .data_req     (bridge_data_req    ),
    .data_we      (bridge_data_we     ),
    .data_addr    (bridge_data_addr   ),
    .data_wdata   (bridge_data_wdata  ),
    .data_rdata   (data_rdata         ),
    .mem_pending  (bridge_mem_pending ),

    .crmd_plv     (crmd_plv           ),

    .arid         (arid    ),
    .araddr       (araddr  ),
    .arlen        (arlen   ),
    .arsize       (arsize  ),
    .arburst      (arburst ),
    .arlock       (arlock  ),
    .arcache      (arcache ),
    .arprot       (arprot  ),
    .arvalid      (arvalid ),
    .arready      (arready ),

    .rid          (rid     ),
    .rdata        (rdata   ),
    .rresp        (rresp   ),
    .rlast        (rlast   ),
    .rvalid       (rvalid  ),
    .rready       (rready  ),

    .awid         (awid    ),
    .awaddr       (awaddr  ),
    .awlen        (awlen   ),
    .awsize       (awsize  ),
    .awburst      (awburst ),
    .awlock       (awlock  ),
    .awcache      (awcache ),
    .awprot       (awprot  ),
    .awvalid      (awvalid ),
    .awready      (awready ),

    .wid          (wid     ),
    .wdata        (wdata   ),
    .wstrb        (wstrb   ),
    .wlast        (wlast   ),
    .wvalid       (wvalid  ),
    .wready       (wready  ),

    .bid          (bid     ),
    .bresp        (bresp   ),
    .bvalid       (bvalid  ),
    .bready       (bready  )
);

endmodule

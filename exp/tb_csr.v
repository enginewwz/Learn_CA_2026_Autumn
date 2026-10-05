// Self-checking testbench for csr.v (not part of the CPU build).
//
//   iverilog -g2005 -o /tmp/tb_csr exp/myCPU/csr.v exp/tb_csr.v && vvp /tmp/tb_csr
//
// Covers the per-field management styles of csr.v:
//   * reset values (CRMD.DA wired to 1, everything else 0)
//   * csrwr / csrxchg masked software writes and read-back
//   * write-first read port (a write is visible to a read in the same cycle)
//   * CRMD/PRMD/ERA/ESTAT on exception entry, and ertn restoring PRMD
//   * ESTAT line-driven IS bits (SWI / HWI / IPI) and the reserved bits
//   * EENTRY[5:0] and ECFG.LIE[10] masks
//   * BADV written by the address-exception port
//   * TCFG/TVAL countdown, TI set, TICLR (write-only, reads 0), periodic reload
//   * the interrupt request |(ECFG.LIE & ESTAT.IS) & CRMD.IE
`timescale 1ns/1ps

module tb_csr;

reg clk = 1'b0;
always #5 clk = ~clk;

reg         reset  = 1'b1;
reg  [13:0] raddr  = 14'd0;
wire [31:0] rdata;
reg         we     = 1'b0;
reg  [13:0] waddr  = 14'd0;
reg  [31:0] wdata  = 32'd0;
reg  [31:0] wmask  = 32'd0;
reg         excp   = 1'b0;
reg  [31:0] excp_pc = 32'd0;
reg  [ 5:0] excp_ecode = 6'd0;
reg  [ 8:0] excp_esub  = 9'd0;
reg         badv_en = 1'b0;
reg  [31:0] badv_i  = 32'd0;
reg         ertn    = 1'b0;
reg  [ 7:0] hwi     = 8'd0;
reg         ipi     = 1'b0;

wire [ 1:0] crmd_plv;
wire        crmd_ie;
wire [31:0] eentry_o, era_o;
wire        int_o;
wire [63:0] timer_o;

// CSR numbers under test
localparam [13:0] CRMD = 14'h000, PRMD = 14'h001, ECFG = 14'h004, ESTAT = 14'h005;
localparam [13:0] ERA  = 14'h006, BADV = 14'h007, EENTRY = 14'h00c;
localparam [13:0] SAVE0 = 14'h030, SAVE1 = 14'h031, SAVE2 = 14'h032, SAVE3 = 14'h033;
localparam [13:0] TID = 14'h040, TCFG = 14'h041, TVAL = 14'h042, TICLR = 14'h044;

csr u_csr(
    .clk            (clk            ),
    .reset          (reset          ),
    .raddr_i        (raddr          ),
    .rdata_o        (rdata          ),
    .we_i           (we             ),
    .waddr_i        (waddr          ),
    .wdata_i        (wdata          ),
    .wmask_i        (wmask          ),
    .excp_i         (excp           ),
    .excp_pc_i      (excp_pc        ),
    .excp_ecode_i   (excp_ecode     ),
    .excp_esubcode_i(excp_esub      ),
    .excp_badv_en_i (badv_en        ),
    .excp_badv_i    (badv_i         ),
    .ertn_i         (ertn           ),
    .hwi_i          (hwi            ),
    .ipi_i          (ipi            ),
    .crmd_plv_o     (crmd_plv       ),
    .crmd_ie_o      (crmd_ie        ),
    .eentry_o       (eentry_o       ),
    .era_o          (era_o          ),
    .int_o          (int_o          ),
    .timer_o        (timer_o        )
);

integer errors = 0;
integer checks = 0;

task step; begin @(posedge clk); #1; end endtask

task chk;
    input cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("CSR ERROR: %0s (t=%0t)", name, $time);
        end
    end
endtask

// software write with a mask (csrwr = all ones, csrxchg = rj)
task wr;
    input [13:0] a;
    input [31:0] d, m;
    begin
        @(negedge clk);
        we = 1'b1; waddr = a; wdata = d; wmask = m;
        step;
        we = 1'b0;
    end
endtask

// full-word read check
task chr;
    input [13:0] a;
    input [31:0] exp;
    input [511:0] name;
    begin
        raddr = a; #1;
        checks = checks + 1;
        if (rdata !== exp) begin
            errors = errors + 1;
            $display("CSR ERROR: %0s: raddr=%h got %h exp %h (t=%0t)",
                     name, a, rdata, exp, $time);
        end
    end
endtask

// masked read check (for ESTAT, which keeps Ecode across tests)
task chrm;
    input [13:0] a;
    input [31:0] msk, exp;
    input [511:0] name;
    begin
        raddr = a; #1;
        checks = checks + 1;
        if ((rdata & msk) !== (exp & msk)) begin
            errors = errors + 1;
            $display("CSR ERROR: %0s: raddr=%h got %h exp %h (t=%0t)",
                     name, a, rdata, exp, $time);
        end
    end
endtask

initial begin
    reset = 1'b1; we = 1'b0; excp = 1'b0; ertn = 1'b0; badv_en = 1'b0;
    repeat (3) step;
    reset = 1'b0;
    step;

    // ------------------------------------------------------------ reset
    chr(CRMD,   32'h0000_0008, "CRMD reset = {DA=1}");
    chr(PRMD,   32'h0000_0000, "PRMD reset");
    chr(ECFG,   32'h0000_0000, "ECFG reset");
    chr(ESTAT,  32'h0000_0000, "ESTAT reset");
    chr(ERA,    32'h0000_0000, "ERA reset");
    chr(BADV,   32'h0000_0000, "BADV reset");
    chr(EENTRY, 32'h0000_0000, "EENTRY reset");
    chr(SAVE0,  32'h0000_0000, "SAVE0 reset");
    chr(TID,    32'h0000_0000, "TID reset");
    chr(TCFG,   32'h0000_0000, "TCFG reset");
    chr(TVAL,   32'h0000_0000, "TVAL reset");
    chr(TICLR,  32'h0000_0000, "TICLR reads 0");

    // --------------------------- crmd_plv_o / crmd_ie_o are write-first
    // (kept shaped for a privilege check; IPE itself is not implemented)
    @(negedge clk);
    we = 1'b1; waddr = CRMD; wdata = 32'h0000_0003; wmask = 32'h0000_0007;
    #1;
    chk(crmd_plv === 2'd3, "crmd_plv_o shows a same-cycle crswr CRMD");
    step;
    we = 1'b0;
    wr(CRMD, 32'h0000_0000, 32'h0000_0007);         // back to PLV=0, IE=0

    // ------------------------------------------- csrwr / csrxchg / SAVE
    wr(SAVE1, 32'hdead_beef, 32'hffff_ffff);
    chr(SAVE1, 32'hdead_beef, "csrwr SAVE1");

    wr(SAVE0, 32'hffff_0000, 32'hffff_ffff);
    wr(SAVE0, 32'h0000_ffff, 32'h00ff_00ff);        // csrxchg masked
    chr(SAVE0, 32'hff00_00ff, "csrxchg SAVE0 = (rd&rj)|(old&~rj)");

    wr(TID, 32'h1111_ffff, 32'hffff_ffff);
    chr(TID, 32'h1111_ffff, "csrwr TID");

    // ------------------------------- read is the registered (old) value
    // Read and write both sit at WB and WB holds one instruction, so there is
    // no write-first: a read in the same cycle as a write must return the OLD
    // value (a csrwr has to hand back the pre-write value to its rd).
    wr(SAVE2, 32'h0bad_f00d, 32'hffff_ffff);        // known old value
    @(negedge clk);
    we = 1'b1; waddr = SAVE2; wdata = 32'h1234_5678; wmask = 32'hffff_ffff;
    raddr = SAVE2; #1;
    chk(rdata === 32'h0bad_f00d, "same-cycle read returns the OLD value");
    raddr = SAVE3; #1;
    chk(rdata === 32'h0000_0000, "same-cycle read of another CSR unaffected");
    step;
    we = 1'b0;
    chr(SAVE2, 32'h1234_5678, "the write lands at the clock edge");

    // ------------------------------------------------- EENTRY / ECFG mask
    wr(EENTRY, 32'h1c00_8fff, 32'hffff_ffff);
    chr(EENTRY, 32'h1c00_8fc0, "EENTRY[5:0] read as 0");
    chk(eentry_o === 32'h1c00_8fc0, "eentry_o");

    wr(ECFG, 32'hffff_ffff, 32'hffff_ffff);
    chr(ECFG, 32'h0000_1bff, "ECFG.LIE write mask 0x1bff (bit10 stays 0)");

    // ------------------------------------------------- ESTAT line-driven bits
    wr(ESTAT, 32'h0000_0003, 32'h0000_0003);
    chrm(ESTAT, 32'h0000_0003, 32'h0000_0003, "ESTAT SWI[1:0] set");
    wr(ESTAT, 32'h0000_0000, 32'h0000_0003);
    chrm(ESTAT, 32'h0000_0003, 32'h0000_0000, "ESTAT SWI[1:0] clear");

    hwi = 8'h80; #1;
    chrm(ESTAT, 32'h0000_0200, 32'h0000_0200, "ESTAT HWI7 -> IS[9]");
    hwi = 8'h00;
    ipi = 1'b1; #1;
    chrm(ESTAT, 32'h0000_1000, 32'h0000_1000, "ESTAT IPI -> IS[12]");
    ipi = 1'b0;

    // ------------------------------------------- CRMD / exception / ertn
    wr(CRMD, 32'h0000_0003, 32'h0000_0007);         // PLV=3
    chr(CRMD, 32'h0000_000b, "CRMD PLV=3 (DA=1)");
    chk(crmd_plv === 2'd3, "crmd_plv_o = 3");

    wr(CRMD, 32'h0000_0007, 32'h0000_0007);         // PLV=3, IE=1
    chk(crmd_ie === 1'b1, "crmd_ie_o = 1");

    // syscall: PRMD <- {PIE,PPLV}, CRMD drops to kernel, ERA <- pc
    @(negedge clk);
    excp = 1'b1; excp_pc = 32'h1c00_1234; excp_ecode = 6'h0b; excp_esub = 9'h0;
    step;
    excp = 1'b0;
    chr(ERA,   32'h1c00_1234, "ERA = exception pc");
    chr(PRMD,  32'h0000_0007, "PRMD saved PPLV=3, PIE=1");
    chr(CRMD,  32'h0000_0008, "CRMD -> kernel, IE=0");
    chrm(ESTAT, 32'h003f_0000, 32'h000b_0000, "ESTAT.Ecode = SYS (0xb)");
    chk(crmd_plv === 2'd0 && crmd_ie === 1'b0, "side-band after exception");

    // ertn restores
    @(negedge clk); ertn = 1'b1; step; ertn = 1'b0;
    chr(CRMD, 32'h0000_000f, "ertn restores PLV=3, IE=1");
    chk(crmd_plv === 2'd3 && crmd_ie === 1'b1, "side-band after ertn");

    // an older csrwr CRMD landing in the same commit slot as the exception is
    // ordered before it: PRMD must save the *new* PLV/IE, CRMD still drops
    wr(CRMD, 32'h0000_0000, 32'h0000_0007);         // back to PLV=0, IE=0
    @(negedge clk);
    we = 1'b1; waddr = CRMD; wdata = 32'h0000_0007; wmask = 32'h0000_0007;
    excp = 1'b1; excp_pc = 32'h1c00_3000; excp_ecode = 6'h0b; excp_esub = 9'h0;
    step;
    we = 1'b0; excp = 1'b0;
    chr(CRMD, 32'h0000_0008, "exception wins over the same-slot csrwr CRMD");
    chr(PRMD, 32'h0000_0007, "PRMD saved the same-slot csrwr CRMD value");

    // ------------------------ entry / return targets are write-first as well
    // the entry is serviced one stage before the instruction reaches WB, so a
    // write still in flight must already be visible to it
    @(negedge clk);
    we = 1'b1; waddr = ERA; wdata = 32'h1c00_5678; wmask = 32'hffff_ffff;
    #1;
    chk(era_o === 32'h1c00_5678, "era_o is write-first");
    step;
    we = 1'b0;
    chr(ERA, 32'h1c00_5678, "ERA latched");

    @(negedge clk);
    we = 1'b1; waddr = EENTRY; wdata = 32'h1c00_9000; wmask = 32'hffff_ffff;
    #1;
    chk(eentry_o === 32'h1c00_9000, "eentry_o is write-first");
    step;
    we = 1'b0;

    // ertn in the same cycle as a PRMD write restores the *new* PRMD
    wr(CRMD, 32'h0000_0008, 32'h0000_0007);         // PLV=0, IE=0
    wr(PRMD, 32'h0000_0003, 32'h0000_0007);         // PPLV=3, PIE=0
    @(negedge clk);
    we = 1'b1; waddr = PRMD; wdata = 32'h0000_0007; wmask = 32'h0000_0007;
    ertn = 1'b1;
    step;
    we = 1'b0; ertn = 1'b0;
    chk(crmd_plv === 2'd3 && crmd_ie === 1'b1, "ertn sees a same-cycle PRMD write");

    // -------------------------------------- BADV from the hardware port
    wr(BADV, 32'h1234_5678, 32'hffff_ffff);
    chr(BADV, 32'h1234_5678, "BADV software write");
    @(negedge clk);
    excp = 1'b1; excp_pc = 32'h1c00_2000; excp_ecode = 6'h9; excp_esub = 9'h0;
    badv_en = 1'b1; badv_i = 32'h1c00_2004;
    step;
    excp = 1'b0; badv_en = 1'b0;
    chr(BADV,  32'h1c00_2004, "BADV hardware write");
    chrm(ESTAT, 32'h003f_0000, 32'h0009_0000, "ESTAT.Ecode = ALE (0x9)");
    chr(ERA,   32'h1c00_2000, "ERA from the ALE exception");

    // ---------------------------------------------------------- timer
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);        // clear TI first
    wr(TCFG,  32'h0000_0011, 32'hffff_ffff);        // En=1, InitVal=4 -> TVAL=16
    chr(TCFG, 32'h0000_0011, "TCFG En=1, InitVal=4");
    chr(TVAL, 32'h0000_0010, "TVAL loaded with InitVal<<2");
    repeat (15) step;
    chr(TVAL, 32'h0000_0001, "TVAL counted down to 1");
    step;
    chrm(ESTAT, 32'h0000_0800, 32'h0000_0800, "TVAL==1 raises TI (IS[11])");
    chr(TVAL, 32'h0000_0000, "TVAL reached 0");

    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);        // clear TI
    chrm(ESTAT, 32'h0000_0800, 32'h0000_0000, "TICLR clears TI");
    chr(TICLR, 32'h0000_0000, "TICLR still reads 0");

    // InitVal = 0 with En = 1 loads a count of zero, which is already expired
    wr(TCFG,  32'h0000_0000, 32'hffff_ffff);        // stop
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);        // clear TI
    wr(TCFG,  32'h0000_0001, 32'hffff_ffff);        // En=1, InitVal=0
    chr(TVAL, 32'h0000_0000, "TVAL loaded with InitVal=0");
    chrm(ESTAT, 32'h0000_0800, 32'h0000_0800, "En=1 + InitVal=0 raises TI at once");

    // ... but En = 0 with InitVal = 0 must not
    wr(TCFG,  32'h0000_0000, 32'hffff_ffff);
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);
    wr(TCFG,  32'h0000_0000, 32'hffff_ffff);        // En=0, InitVal=0
    chrm(ESTAT, 32'h0000_0800, 32'h0000_0000, "En=0 + InitVal=0 leaves TI clear");

    // periodic reload
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);
    wr(TCFG,  32'h0000_0013, 32'hffff_ffff);        // En=1, Periodic=1, InitVal=4
    chr(TVAL, 32'h0000_0010, "periodic TVAL reloaded");
    repeat (16) step;
    chr(TVAL, 32'h0000_0000, "periodic TVAL hit 0");
    step;
    chr(TVAL, 32'h0000_0010, "periodic TVAL wraps back to InitVal<<2");

    // ---------------------------------------------------- interrupt request
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);        // clear TI
    wr(TCFG,  32'h0000_0000, 32'hffff_ffff);        // stop the timer
    wr(ECFG,  32'h0000_0800, 32'hffff_ffff);        // ECFG.LIE[11]
    wr(CRMD,  32'h0000_0004, 32'h0000_0004);        // CRMD.IE = 1
    wr(TCFG,  32'h0000_0011, 32'hffff_ffff);        // fire once
    repeat (17) step;
    chk(int_o === 1'b1, "int_o = |(ECFG.LIE & ESTAT.IS) & CRMD.IE");
    wr(TICLR, 32'h0000_0001, 32'hffff_ffff);
    #1;
    chk(int_o === 1'b0, "int_o clears with TI");

    // --------------------------------------------- free-running 64-bit timer
    chk(timer_o > 64'd0, "timer_o free-runs");

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule

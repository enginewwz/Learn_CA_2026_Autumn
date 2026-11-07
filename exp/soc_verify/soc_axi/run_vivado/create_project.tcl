# Resolve everything relative to this script, so the flow also works when
# Vivado is launched from another directory.
set ScriptDir  [file normalize [file dirname [info script]]]
set ProjectDir [file normalize $ScriptDir/project]

if {[file exists $ProjectDir]} {
    file delete -force $ProjectDir
    puts "Folder deleted: $ProjectDir"
}

create_project -force loongson $ProjectDir -part xc7a200tfbg676-1

# Add conventional sources
add_files -scan_for_includes [glob -nocomplain $ScriptDir/../rtl/*.v]
add_files -scan_for_includes $ScriptDir/../rtl/axi_wrap
add_files -scan_for_includes $ScriptDir/../rtl/CONFREG
add_files -scan_for_includes $ScriptDir/../rtl/ram_wrap

# Add the checked-in IPs: axi_ram / clk_pll / axi_crossbar_1x2.
#
# UG835: "IP and Block Design sources are not added through the add_files
# command. These are compound files that are supported by separate commands
# such as import_ip, read_bd, and read_ip."
# The previous `add_files -quiet [glob .../*.xci]` therefore added nothing
# (and -quiet hid it), leaving get_ips empty -- that is why Vivado could not
# find these three IPs even though the .xci files are on disk.
set IpXciList [lsort [glob -nocomplain $ScriptDir/../rtl/xilinx_ip/*/*.xci]]
if {[llength $IpXciList] == 0} {
    error "no .xci found under $ScriptDir/../rtl/xilinx_ip"
}
puts "checked-in IP .xci: $IpXciList"
foreach xci $IpXciList {
    puts "read_ip $xci"
    read_ip $xci
}

# --- the three cache RAM IPs -------------------------------------------------
# icache.v / dcache.v instantiate three MORE RAMs that are not in
# rtl/xilinx_ip; they used to live only in the git-ignored project/ tree that
# this script deletes at the top.  Regenerate them as blk_mem_gen 8.4:
#   inst_ram : 256 x 128, WRITE_FIRST  (icache line data)
#   data_ram : 256 x  32, NO_CHANGE    (dcache data bank)
#   tagvd    : 256 x  22, NO_CHANGE    (tag + valid + dirty)
# All three are single port, one ena pin, no reset, read latency 1:
#   clka, ena, wea[0:0], addra[7:0], dina[W-1:0], douta[W-1:0]
# local_testbench/cache_ram_sim.v is the iverilog stand-in for the same
# contract.
proc make_cache_ram {name width mode} {
    global ScriptDir
    if {[file exists $ScriptDir/../rtl/xilinx_ip/$name/$name.xci]} {
        puts "cache RAM IP $name: using checked-in ../rtl/xilinx_ip/$name/$name.xci"
        return
    }
    create_ip -name blk_mem_gen -vendor xilinx.com -library ip -version 8.4 \
              -module_name $name
    set_property -dict [list \
        CONFIG.Memory_Type {Single_Port_RAM} \
        CONFIG.PRIM_type_to_Implement {BRAM} \
        CONFIG.Write_Width_A $width \
        CONFIG.Write_Depth_A 256 \
        CONFIG.Read_Width_A $width \
        CONFIG.Operating_Mode_A $mode \
        CONFIG.Enable_A {Use_ENA_Pin} \
        CONFIG.Register_PortA_Output_of_Memory_Primitives {false} \
        CONFIG.Use_RSTA_Pin {false} \
        CONFIG.Use_Byte_Write_Enable {false} \
        CONFIG.Enable_32bit_Address {false} \
        CONFIG.Load_Init_File {false} \
    ] [get_ips $name]
    generate_target {instantiation_template synthesis simulation} [get_ips $name]
    puts "cache RAM IP $name: generated (256 x $width, $mode)"
}

make_cache_ram inst_ram 128 WRITE_FIRST
make_cache_ram data_ram  32 NO_CHANGE
make_cache_ram tagvd     22 NO_CHANGE

# Fail loudly if any IP did not make it into the project, instead of letting
# synthesis/simulation report "module not found" much later.
foreach ip {axi_ram clk_pll axi_crossbar_1x2 inst_ram data_ram tagvd} {
    if {[llength [get_ips -quiet $ip]] == 0} {
        error "IP '$ip' is missing from the project (see the messages above)"
    }
}

# Generate every IP's output products now.  In project mode Vivado would do
# this lazily, but doing it here makes the project immediately usable in a
# batch run as well.
set AllIps [get_ips]
puts "IPs in project: $AllIps"
if {[catch {generate_target all $AllIps} err]} {
    puts "WARNING: generate_target all failed: $err"
}

# Add simulation files
add_files -fileset sim_1 $ScriptDir/../testbench

# Add myCPU
add_files -scan_for_includes $ScriptDir/../../../myCPU

# Add constraints
add_files -fileset constrs_1 -quiet $ScriptDir/constraints

set_property -name "top" -value "tb_top" -objects  [get_filesets sim_1]
set_property -name "xsim.simulate.log_all_signals" -value "1" -objects [get_filesets sim_1]

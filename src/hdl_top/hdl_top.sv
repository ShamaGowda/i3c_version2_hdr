`ifndef HDL_TOP_INCLUDED_
`define HDL_TOP_INCLUDED_

import i3c_globals_pkg::*;
import apb_global_pkg::*;

module hdl_top;

bit clk;
bit rst;

wire I3C_SCL;
wire I3C_SDA;

//---------------------------------
// APB clock/reset
//---------------------------------
wire pclk;
wire preset_n;

assign pclk     = clk;
assign preset_n = rst;

//---------------------------------
// DUT CPU interface signals
//---------------------------------
logic        wr_en;
logic        rd_en;
logic [6:0]  addrs;
logic [31:0] w_reg_data;
logic [7:0]  w_data;

logic [31:0] rd_data;
logic [7:0]  r_data;

logic scl_o;
wire sda_o;
logic sda_oe;

initial begin
$display("HDL TOP");
end

//---------------------------------
// Clock generation
//---------------------------------
initial begin
clk = 1'b0;
forever #10 clk = ~clk;
end

//---------------------------------
// Reset generation
//---------------------------------
initial begin
rst = 1'b1;

repeat (2) @(posedge clk);
rst = 1'b0;

repeat (2) @(posedge clk);
rst = 1'b1;
end

//---------------------------------
// APB interface
//---------------------------------
apb_if apb_intf(.pclk(pclk), .preset_n(preset_n));




//---------------------------------
// I3C Target Interface
//---------------------------------
i3c_if intf_target(
.pclk(clk),
.areset(rst),
.SCL(I3C_SCL),
.SDA(I3C_SDA)
);

//---------------------------------
// I3C Controller Interface
//---------------------------------
i3c_if intf_controller(
.pclk(clk),
.areset(rst),
.SCL(I3C_SCL),
.SDA(I3C_SDA)
);

//---------------------------------
// Pullups for I3C bus
//---------------------------------
pullup p1 (I3C_SCL);
pullup p2 (I3C_SDA);

//---------------------------------
// APB -> DUT wrapper
//---------------------------------
apb_i3c_wrapper wrapper(
.apb(apb_intf),

.wr_en(wr_en),
.rd_en(rd_en),
.addrs(addrs),
.w_reg_data(w_reg_data),
.w_data(w_data),

.rd_data(rd_data),
.r_data(r_data)
);

//---------------------------------
// DUT (I3C MASTER)
//---------------------------------
I3C_TOP dut(
.clk(clk),
.rst_n(rst),

.wr_en(wr_en),
.rd_en(rd_en),
.addrs(addrs),
.w_reg_data(w_reg_data),
.w_data(w_data),

.rd_data(rd_data),
.r_data(r_data),

.scl_i(I3C_SCL),

.sda_i(I3C_SDA),
.sda_o(sda_o),
.sda_oe(sda_oe)
);
//---------------------------------
// I3C bus connection (open drain)
//---------------------------------

//added apb_master agent bfm
apb_master_agent_bfm apb_master_agent_bfm_h(apb_intf);
i3c_controller_agent_bfm i3c_controller_agent_bfm_h(intf_controller);



//---------------------------------
// Target AVIP
//---------------------------------
genvar i;
generate
for (i=0; i<NO_OF_TARGETS; i++) begin : i3c_target_agent_bfm
i3c_target_agent_bfm i3c_target_agent_bfm_h(intf_target);  
end
endgenerate

//---------------------------------
// Dump waves
//---------------------------------
initial begin
$dumpfile("i3c_avip.vcd");
$dumpvars();
end

endmodule : hdl_top

`endif


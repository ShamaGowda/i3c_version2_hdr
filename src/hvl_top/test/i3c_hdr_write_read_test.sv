`ifndef I3C_HDR_WRITE_READ_TEST_INCLUDED_
`define I3C_HDR_WRITE_READ_TEST_INCLUDED_

// i3c_hdr_write_read_test
// Runs HDR WRITE followed immediately by HDR READ to the same target.
// Scoreboard verifies both transactions.
// Expected scoreboard summary:
//   HDR transactions           : 2
//   HDR write byte pass / fail : N / 0
//   HDR read  byte pass / fail : N / 0

class i3c_hdr_write_read_test extends i3c_base_test;
  `uvm_component_utils(i3c_hdr_write_read_test)

  extern function new(string name = "i3c_hdr_write_read_test",
                      uvm_component parent = null);
  extern virtual task run_phase(uvm_phase phase);

endclass : i3c_hdr_write_read_test


function i3c_hdr_write_read_test::new(
  string name = "i3c_hdr_write_read_test",
  uvm_component parent = null);
  super.new(name, parent);
endfunction : new


task i3c_hdr_write_read_test::run_phase(uvm_phase phase);
  i3c_hdr_write_read_virtual_seq hdr_wr_rd_vseq;
  phase.raise_objection(this);

  `uvm_info(get_type_name(),
    "Starting HDR Write+Read test", UVM_LOW)

  hdr_wr_rd_vseq =
    i3c_hdr_write_read_virtual_seq::type_id::create("hdr_wr_rd_vseq");
  hdr_wr_rd_vseq.start(i3c_env_h.top_virtual_seqr_h);

  phase.drop_objection(this);
endtask : run_phase

`endif



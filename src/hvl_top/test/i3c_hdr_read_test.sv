`ifndef I3C_HDR_READ_TEST_INCLUDED_
`define I3C_HDR_READ_TEST_INCLUDED_

class i3c_hdr_read_test extends i3c_base_test;
  `uvm_component_utils(i3c_hdr_read_test)

  extern function new(string name = "i3c_hdr_read_test",
                      uvm_component parent = null);
  extern virtual task run_phase(uvm_phase phase);

endclass : i3c_hdr_read_test


function i3c_hdr_read_test::new(string name = "i3c_hdr_read_test",
                                 uvm_component parent = null);
  super.new(name, parent);
endfunction : new


task i3c_hdr_read_test::run_phase(uvm_phase phase);
  i3c_hdr_read_virtual_seq hdr_read_vseq;
  phase.raise_objection(this);

  hdr_read_vseq =
    i3c_hdr_read_virtual_seq::type_id::create("hdr_read_vseq");
  hdr_read_vseq.start(i3c_env_h.top_virtual_seqr_h);

  phase.drop_objection(this);
endtask : run_phase

`endif

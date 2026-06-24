`ifndef I3C_HDR_WRITE_TEST_INCLUDED_
`define I3C_HDR_WRITE_TEST_INCLUDED_

class i3c_hdr_write_test extends i3c_base_test;
  `uvm_component_utils(i3c_hdr_write_test)

  extern function new(string name = "i3c_hdr_write_test",
                      uvm_component parent = null);
  extern virtual task run_phase(uvm_phase phase);

endclass : i3c_hdr_write_test


function i3c_hdr_write_test::new(string name = "i3c_hdr_write_test",
                                  uvm_component parent = null);
  super.new(name, parent);
endfunction : new


task i3c_hdr_write_test::run_phase(uvm_phase phase);
  i3c_hdr_write_virtual_seq hdr_write_vseq;
  phase.raise_objection(this);

  hdr_write_vseq =
    i3c_hdr_write_virtual_seq::type_id::create("hdr_write_vseq");
  hdr_write_vseq.start(i3c_env_h.top_virtual_seqr_h);

  phase.drop_objection(this);
endtask : run_phase

`endif

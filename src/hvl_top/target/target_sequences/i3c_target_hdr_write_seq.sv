`ifndef I3C_TARGET_HDR_WRITE_SEQ_INCLUDED_
`define I3C_TARGET_HDR_WRITE_SEQ_INCLUDED_

// i3c_target_hdr_write_seq
// Used when the DUT (controller) performs an HDR-DDR WRITE.
// Target side: ACKs address, receives DDR data words.
class i3c_target_hdr_write_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_hdr_write_seq)

  extern function new(string name = "i3c_target_hdr_write_seq");
  extern task body();
endclass : i3c_target_hdr_write_seq


function i3c_target_hdr_write_seq::new(string name = "i3c_target_hdr_write_seq");
  super.new(name);
endfunction : new


task i3c_target_hdr_write_seq::body();
  req = i3c_target_tx::type_id::create("req");
  start_item(req);

  req.targetAddress = p_sequencer.i3c_target_agent_cfg_h.targetAddress;
  req.operation     = WRITE;
  req.txn_type      = i3c_target_tx::HDR_WRITE;

  if (!req.randomize() with {
      targetAddressStatus == ACK;
      txn_type            == i3c_target_tx::HDR_WRITE;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed")
  end else begin
    // All write data ACKed by default (target receives, not checks)
    req.writeDataStatus = new[MAXIMUM_BYTES];
    foreach (req.writeDataStatus[i])
      req.writeDataStatus[i] = ACK;
    `uvm_info(get_type_name(), "HDR_WRITE target seq randomized", UVM_NONE)
    req.print();
  end

  finish_item(req);
  `uvm_info(get_type_name(), "HDR_WRITE target seq done", UVM_NONE)
endtask : body

`endif


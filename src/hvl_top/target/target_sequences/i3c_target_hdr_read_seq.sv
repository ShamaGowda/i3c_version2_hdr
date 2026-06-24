`ifndef I3C_TARGET_HDR_READ_SEQ_INCLUDED_
`define I3C_TARGET_HDR_READ_SEQ_INCLUDED_

// i3c_target_hdr_read_seq
// Used when the DUT (controller) performs an HDR-DDR READ.
// Target side: ACKs address, drives DDR data words back to DUT.
class i3c_target_hdr_read_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_hdr_read_seq)

  rand bit [7:0] read_data_byte;

  extern function new(string name = "i3c_target_hdr_read_seq");
  extern task body();
endclass : i3c_target_hdr_read_seq


function i3c_target_hdr_read_seq::new(string name = "i3c_target_hdr_read_seq");
  super.new(name);
endfunction : new


task i3c_target_hdr_read_seq::body();
  req = i3c_target_tx::type_id::create("req");
  start_item(req);

  req.targetAddress = p_sequencer.i3c_target_agent_cfg_h.targetAddress;
  req.operation     = READ;
  req.txn_type      = i3c_target_tx::HDR_READ;

  if (!req.randomize() with {
      targetAddressStatus == ACK;
      txn_type            == i3c_target_tx::HDR_READ;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed")
  end else begin
    // Populate readData with random data that target will drive back
    req.readData = new[MAXIMUM_BYTES];
    foreach (req.readData[i])
      req.readData[i] = $urandom_range(0, 255);

    // Pre-compute how many bits to drive (matches cmd_len from CTRL)
    req.size = MAXIMUM_BYTES;

    `uvm_info(get_type_name(), "HDR_READ target seq randomized", UVM_NONE)
    req.print();
  end

  finish_item(req);
  `uvm_info(get_type_name(), "HDR_READ target seq done", UVM_NONE)
endtask : body

`endif

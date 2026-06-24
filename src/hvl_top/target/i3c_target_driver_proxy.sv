`ifndef I3C_TARGET_DRIVER_PROXY_INCLUDED_
`define I3C_TARGET_DRIVER_PROXY_INCLUDED_

class i3c_target_driver_proxy extends uvm_driver#(i3c_target_tx);
  `uvm_component_utils(i3c_target_driver_proxy)

  i3c_target_agent_config  i3c_target_agent_cfg_h;
  virtual i3c_target_driver_bfm i3c_target_drv_bfm_h;

  extern function new(string name = "i3c_target_driver_proxy",
                      uvm_component parent = null);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void end_of_elaboration_phase(uvm_phase phase);
  extern virtual task          run_phase(uvm_phase phase);

endclass : i3c_target_driver_proxy


function i3c_target_driver_proxy::new(string name = "i3c_target_driver_proxy",
                                      uvm_component parent = null);
  super.new(name, parent);
endfunction : new


function void i3c_target_driver_proxy::build_phase(uvm_phase phase);
  super.build_phase(phase);
endfunction : build_phase


function void i3c_target_driver_proxy::end_of_elaboration_phase(uvm_phase phase);
  super.end_of_elaboration_phase(phase);
  if(!uvm_config_db #(virtual i3c_target_driver_bfm)::get(
      this, "", "i3c_target_driver_bfm", i3c_target_drv_bfm_h)) begin
    `uvm_fatal("FATAL_SDP_CANNOT_GET_target_DRIVER_BFM",
      "cannot get i3c_target_driver_bfm from uvm_config_db.")
  end
  i3c_target_drv_bfm_h.i3c_target_drv_proxy_h = this;
endfunction : end_of_elaboration_phase


task i3c_target_driver_proxy::run_phase(uvm_phase phase);
  i3c_transfer_bits_s struct_packet;
  i3c_transfer_cfg_s  struct_cfg;

 
  bit [47:0] pid_out;
  bit [7:0]  bcr_out;
  bit [7:0]  dcr_out;
  bit [6:0]  dyn_addr_out;
  bit        daa_ack_out;

  super.run_phase(phase);

  i3c_target_drv_bfm_h.wait_for_system_reset();
  i3c_target_drv_bfm_h.drive_idle_state();

  forever begin

    seq_item_port.get_next_item(req);
    `uvm_info("TGT_DRV_PROXY", "Got item from sequencer", UVM_NONE)

   
    i3c_target_cfg_converter::from_class(i3c_target_agent_cfg_h, struct_cfg);

case(req.txn_type)

  i3c_target_tx::DAA: begin

    `uvm_info("TGT_DRV_PROXY",
      "Transaction type = DAA", UVM_NONE)

    i3c_target_seq_item_converter::from_class(req, struct_packet);

    i3c_target_drv_bfm_h.drive_daa_data(
      struct_packet,
      struct_cfg,
      pid_out,
      bcr_out,
      dcr_out,
      dyn_addr_out,
      daa_ack_out
    );

    req.pid             = pid_out;
    req.bcr             = bcr_out;
    req.dcr             = dcr_out;
    req.dynamic_address = dyn_addr_out;
    req.daa_ack         = daa_ack_out;

    if(daa_ack_out == ACK) begin
      i3c_target_agent_cfg_h.targetAddress = dyn_addr_out;
    end

    i3c_target_seq_item_converter::to_class(struct_packet, req);

  end


  i3c_target_tx::SDR: begin

    `uvm_info("TGT_DRV_PROXY",
      "Transaction type = SDR", UVM_NONE)

    i3c_target_seq_item_converter::from_class(req, struct_packet);

    i3c_target_drv_bfm_h.drive_data(
      struct_packet,
      struct_cfg
    );

    i3c_target_seq_item_converter::to_class(struct_packet, req);

  end

/////////////////////////////////////////////HDR/////////////////////////////////////////////////////////////
  i3c_target_tx::HDR_WRITE: begin

    `uvm_info("TGT_DRV_PROXY",
      "Transaction type = HDR_WRITE", UVM_NONE)

    i3c_target_seq_item_converter::from_class(req, struct_packet);

    i3c_target_drv_bfm_h.drive_hdr_write(
      struct_packet,
      struct_cfg
    );

    i3c_target_seq_item_converter::to_class(struct_packet, req);

  end


  i3c_target_tx::HDR_READ: begin

    `uvm_info("TGT_DRV_PROXY",
      "Transaction type = HDR_READ", UVM_NONE)

    i3c_target_seq_item_converter::from_class(req, struct_packet);

    i3c_target_drv_bfm_h.drive_hdr_read(
      struct_packet,
      struct_cfg
    );

    i3c_target_seq_item_converter::to_class(struct_packet, req);

  end


  default: begin
    `uvm_error("TGT_DRV_PROXY",
      $sformatf("Unsupported txn_type %0d", req.txn_type))
  end

endcase
seq_item_port.item_done();

end 


endtask : run_phase

`endif



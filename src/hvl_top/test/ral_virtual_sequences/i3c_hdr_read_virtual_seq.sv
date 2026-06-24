`ifndef I3C_HDR_READ_VIRTUAL_SEQ_INCLUDED_
`define I3C_HDR_READ_VIRTUAL_SEQ_INCLUDED_

// i3c_hdr_read_virtual_seq
//
// HDR READ: DUT reads DDR data from target.
//   1. Write CTRL with cmd_mode=1, direction=1 (READ), length=N
//   2. DUT performs: SDR address phase → HDR entry → samples DDR → stop
//   3. Read back via RDATAB
//   4. Target BFM drive_hdr_read() drives the DDR frames

class i3c_hdr_read_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hdr_read_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t rdata;

  rand bit [7:0] transfer_len;

  constraint default_len_c {
    transfer_len inside {1, 2, 4};
  }

  function new(string name = "i3c_hdr_read_virtual_seq");
    super.new(name);
  endfunction

  task body();
    i3c_target_hdr_read_seq target_hdr_read_seq;

    super.body();

    if (!this.randomize()) begin
      `uvm_error(get_type_name(), "Randomization failed — using default len=2")
      transfer_len = 2;
    end

    `uvm_info(get_type_name(),
      $sformatf("HDR READ: requesting %0d bytes", transfer_len), UVM_LOW)

    // ── Step 1: start target BFM sequence in background ──────────────────
    fork
      begin
        target_hdr_read_seq =
          i3c_target_hdr_read_seq::type_id::create("target_hdr_read_seq");
        target_hdr_read_seq.start(p_sequencer.i3c_target_seqr_h);
      end
    join_none

    // ── Step 2: configure and trigger CTRL ───────────────────────────────
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.address.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.length.set(transfer_len);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.direction.set(1'b1); // READ

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'b00);

    // KEY: cmd_mode = 1 → HDR path
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));

    // ── Step 3: wait for DUT to complete HDR read ─────────────────────────
    #50us;

    // ── Step 4: read back result from DUT Rx FIFO via RDATAB ─────────────
    for (int i = 0; i < transfer_len; i++) begin
      i3c_env_cfg_h.regBlockHandle.rdatab_inst.read(status, rdata);
      `uvm_info(get_type_name(),
        $sformatf("RDATAB[%0d] = 0x%02x", i, rdata), UVM_MEDIUM)
    end

    `uvm_info(get_type_name(), "HDR READ virtual sequence complete", UVM_LOW)
  endtask

endclass
`endif


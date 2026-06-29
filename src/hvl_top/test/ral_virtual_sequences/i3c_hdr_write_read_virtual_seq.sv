`ifndef I3C_HDR_WRITE_READ_VIRTUAL_SEQ_INCLUDED_
`define I3C_HDR_WRITE_READ_VIRTUAL_SEQ_INCLUDED_

// i3c_hdr_write_read_virtual_seq
//
// Exercises both HDR WRITE and HDR READ in one sequence:
//   Phase 1 — HDR WRITE: CPU loads WDATAB bytes, triggers CTRL with
//             cmd_mode=1 dir=0, DUT sends DDR frames, target receives.
//   Phase 2 — HDR READ:  CPU triggers CTRL with cmd_mode=1 dir=1,
//             target drives DDR frames back, DUT stores in Rx FIFO,
//             CPU reads RDATAB.
//
// The same target address (0x68) is used for both phases.
// Scoreboard sees 2 HDR transactions and verifies both.

class i3c_hdr_write_read_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hdr_write_read_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t rdata;

  // ── Write phase payload ──────────────────────────────────────────────
  rand bit [7:0]  wdata[];
  rand bit [7:0]  write_len;

  // ── Read phase length ────────────────────────────────────────────────
  rand bit [7:0]  read_len;

  constraint len_c {
    write_len inside {1, 2, 4};
    wdata.size() == write_len;
    read_len  inside {1, 2, 4};
  }

  function new(string name = "i3c_hdr_write_read_virtual_seq");
    super.new(name);
  endfunction


  task body();
    super.body();

    if (!this.randomize()) begin
      `uvm_error(get_type_name(), "Randomization failed — using defaults")
      write_len = 2;
      wdata     = new[2];
      wdata[0]  = 8'hA5;
      wdata[1]  = 8'h5A;
      read_len  = 2;
    end

    `uvm_info(get_type_name(), $sformatf(
      "HDR WRITE+READ: write_len=%0d write_data=%p  read_len=%0d",
      write_len, wdata, read_len), UVM_LOW)

    // ════════════════════════════════════════════════════════════════════
    // PHASE 1 — HDR WRITE
    // ════════════════════════════════════════════════════════════════════
    `uvm_info(get_type_name(), "=== PHASE 1: HDR WRITE ===", UVM_LOW)

    // Step 1a: arm target for HDR WRITE in background
    fork
      begin
        i3c_target_hdr_write_seq tgt_write;
        tgt_write = i3c_target_hdr_write_seq::type_id::create("tgt_write");
        tgt_write.start(p_sequencer.i3c_target_seqr_h);
      end
    join_none

    // Step 1b: load TX FIFO via WDATAB
    `uvm_info(get_type_name(), "Step 1: Loading TX FIFO (WDATAB)", UVM_LOW)
    foreach (wdata[i]) begin
      i3c_env_cfg_h.regBlockHandle.wdatab_inst.write(
        status, wdata[i], .parent(this));
      `uvm_info(get_type_name(),
        $sformatf("  WDATAB[%0d] = 0x%02x", i, wdata[i]), UVM_LOW)
    end

    // Step 1c: write CTRL — HDR WRITE
    `uvm_info(get_type_name(),
      "Step 2: Writing CTRL (cmd_mode=1, dir=WRITE, start=1)", UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.address.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.length.set(write_len);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.direction.set(1'b0);  // WRITE
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'b00);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);   // HDR
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info(get_type_name(),
      $sformatf("  CTRL = 0x%08x", ctrl_val), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));

    `uvm_info(get_type_name(),
      "Steps 3-13: DUT performs SDR address → HDR entry → DDR WRITE → STOP",
      UVM_LOW)

    // Wait for DUT to complete HDR write
    #50us;

    `uvm_info(get_type_name(), "Phase 1 (HDR WRITE) complete", UVM_LOW)


    // ════════════════════════════════════════════════════════════════════
    // PHASE 2 — HDR READ
    // ════════════════════════════════════════════════════════════════════
    `uvm_info(get_type_name(), "=== PHASE 2: HDR READ ===", UVM_LOW)

    // Step 2a: arm target for HDR READ in background
    fork
      begin
        i3c_target_hdr_read_seq tgt_read;
        tgt_read = i3c_target_hdr_read_seq::type_id::create("tgt_read");
        tgt_read.start(p_sequencer.i3c_target_seqr_h);
      end
    join_none

    // Step 2b: write CTRL — HDR READ
    `uvm_info(get_type_name(),
      "Step 1: Writing CTRL (cmd_mode=1, dir=READ, start=1)", UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.address.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.length.set(read_len);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.direction.set(1'b1);  // READ
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'b00);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);   // HDR
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info(get_type_name(),
      $sformatf("  CTRL = 0x%08x", ctrl_val), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));

    `uvm_info(get_type_name(),
      "Steps 2-11: DUT performs SDR address → HDR entry → DDR READ → STOP",
      UVM_LOW)

    // Wait for DUT to complete HDR read
    #50us;

    // Step 2c: read back RDATAB bytes
    `uvm_info(get_type_name(),
      "Step 12: Reading RX FIFO via RDATAB", UVM_LOW)
    for (int i = 0; i < read_len; i++) begin
      i3c_env_cfg_h.regBlockHandle.rdatab_inst.read(status, rdata);
      `uvm_info(get_type_name(),
        $sformatf("  RDATAB[%0d] = 0x%02x", i, rdata[7:0]), UVM_LOW)
    end

    `uvm_info(get_type_name(), "Phase 2 (HDR READ) complete", UVM_LOW)
    `uvm_info(get_type_name(), "HDR WRITE+READ sequence complete", UVM_LOW)
  endtask

endclass
`endif


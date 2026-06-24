`ifndef I3C_HDR_WRITE_VIRTUAL_SEQ_INCLUDED_
`define I3C_HDR_WRITE_VIRTUAL_SEQ_INCLUDED_

// i3c_hdr_write_virtual_seq
//
// How HDR WRITE works:
//   1. Write data bytes to WDATAB (DUT Tx FIFO) — these become the DDR payload
//   2. Write CTRL register with:
//        cmd_mode = 1  (bit[26]) ← triggers HDR path in DUT
//        cmd_type = 2'b00        (doesn't matter when cmd_mode=1)
//        direction= 0            (WRITE)
//        length   = N            (bytes to send via DDR)
//        address  = target addr
//        start    = 1
//   3. DUT performs: SDR address phase → HDR entry → DDR frames → stop
//   4. Target BFM drive_hdr_write() responds simultaneously

class i3c_hdr_write_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hdr_write_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;

  rand bit [7:0]  wdata[];         // HDR payload bytes
  rand bit [7:0]  transfer_len;    // number of bytes (CTRL.length)

  constraint default_len_c {
    transfer_len inside {1, 2, 4};  // keep it short for first tests
    wdata.size() == transfer_len;
  }

  function new(string name = "i3c_hdr_write_virtual_seq");
    super.new(name);
  endfunction

  task body();
    i3c_target_hdr_write_seq target_hdr_write_seq;

    super.body();

    if (!this.randomize()) begin
      `uvm_error(get_type_name(), "Randomization failed — using defaults")
      transfer_len = 2;
      wdata        = new[2];
      wdata[0]     = 8'hAB;
      wdata[1]     = 8'hCD;
    end

    `uvm_info(get_type_name(),
      $sformatf("HDR WRITE: len=%0d data=%p", transfer_len, wdata), UVM_LOW)

    // ── Step 1: start target BFM sequence in background ──────────────────
    fork
      begin
        target_hdr_write_seq =
          i3c_target_hdr_write_seq::type_id::create("target_hdr_write_seq");
        target_hdr_write_seq.start(p_sequencer.i3c_target_seqr_h);
      end
    join_none

    // ── Step 2: load payload into DUT Tx FIFO via WDATAB ─────────────────
    foreach (wdata[i]) begin
      i3c_env_cfg_h.regBlockHandle.wdatab_inst.write(
        status, wdata[i], .parent(this));
      `uvm_info(get_type_name(),
        $sformatf("Wrote WDATAB[%0d] = 0x%02x", i, wdata[i]), UVM_HIGH)
    end

    // ── Step 3: configure and trigger CTRL register ───────────────────────
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.address.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.length.set(transfer_len);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.direction.set(1'b0); // WRITE

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'b00); // SDR for addr phase

    // KEY: set cmd_mode = 1 → bit[26] → tells DUT to do HDR after address
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info(get_type_name(),
      $sformatf("CTRL value = 0x%08x (bit26=cmd_mode should be 1)", ctrl_val),
      UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));

    // ── Step 4: wait for DUT HDR transfer to complete ─────────────────────
    #50us;

    `uvm_info(get_type_name(), "HDR WRITE virtual sequence complete", UVM_LOW)
  endtask

endclass
`endif


`ifndef I3C_SCOREBOARD_INCLUDED_
`define I3C_SCOREBOARD_INCLUDED_

class i3c_scoreboard extends uvm_component;
  `uvm_component_utils(i3c_scoreboard)

uvm_tlm_analysis_fifo #(apb_master_tx)  apb_analysis_fifo;  
uvm_tlm_analysis_fifo #(i3c_target_tx)  target_analysis_fifo;

localparam realtime HDR_READ_TIMEOUT_NS = 150000;
  i3c_env_config i3c_env_cfg_h;

  //SDR Counters 
  int apb_tx_count;
  int target_tx_count;
  int write_pass;
  int write_fail;
  int read_pass;
  int read_fail;

  // DAA Counters 
  int daa_addr_pass;
  int daa_addr_fail;
  int daa_parity_pass;
  int daa_parity_fail;
  int daa_devices_seen;

  // HDR Counters
  int hdr_write_pass;
  int hdr_write_fail;
  int hdr_read_pass;
  int hdr_read_fail;
  int hdr_txn_count;

//expected values from CTRL 
  bit [6:0]  exp_address;
  bit [7:0]  exp_length;
  bit        exp_direction;
  bit [1:0]  exp_cmd_type;
  bit [7:0]  exp_ccc;
  bit        exp_cmd_mode;      // bit[26] of CTRL — 1 = HDR mode

  //  SDR data queues 
  bit [7:0]  exp_write_data[$];
  bit [7:0]  exp_rd_wr_data[$];

  // HDR expected data queue (bytes loaded via WDATAB before CTRL write)
  bit [7:0]  exp_hdr_write_data[$];

  // DAA expected state 
  bit [6:0]  daa_next_exp_addr;


  extern function new(string name = "i3c_scoreboard",
                      uvm_component parent = null);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual task          run_phase(uvm_phase phase);
  extern virtual function void check_phase(uvm_phase phase);

  //  SDR tasks 
  extern protected task          collect_apb_transaction();
  extern protected task          compare_with_target();
  extern protected function void decode_ctrl(bit [31:0] ctrl_val);

  //  DAA tasks 
  extern protected function bit  is_daa_transaction();
  extern protected task          compare_with_daa_target();

  //  HDR tasks
  extern protected function bit  is_hdr_transaction();
  extern protected task          compare_hdr_write();
  extern protected task          compare_hdr_read();

endclass : i3c_scoreboard


function i3c_scoreboard::new(string name = "i3c_scoreboard",
                              uvm_component parent = null);
  super.new(name, parent);
endfunction

function void i3c_scoreboard::build_phase(uvm_phase phase);
  super.build_phase(phase);
  apb_analysis_fifo    = new("apb_analysis_fifo",    this);
  target_analysis_fifo = new("target_analysis_fifo", this);

  if(!uvm_config_db #(i3c_env_config)::get(
      this, "", "i3c_env_config", i3c_env_cfg_h))
    `uvm_fatal("SB_CFG", "Cannot get i3c_env_config from config_db")

  daa_next_exp_addr = DAA_FIRST_DYN_ADDR;

endfunction

    
function bit i3c_scoreboard::is_daa_transaction();
  if(exp_cmd_type == CMD_TYPE_DAA)
    return 1;
  if(exp_cmd_type == CMD_TYPE_CCC && exp_ccc == ENTDAA_CCC_CODE)
    return 1;
  return 0;
endfunction

// HDR transaction: CTRL bit[26] (cmd_mode) = 1
function bit i3c_scoreboard::is_hdr_transaction();
  return exp_cmd_mode;
endfunction


// run_phase — dispatch to HDR, DAA, or SDR comparison
task i3c_scoreboard::run_phase(uvm_phase phase);
  super.run_phase(phase);
  forever begin
    collect_apb_transaction();

    if (is_hdr_transaction()) begin
      if (exp_direction == 1'b0) begin
        `uvm_info("SB",
          $sformatf("HDR WRITE transaction detected: addr=0x%0x len=%0d",
                    exp_address, exp_length), UVM_MEDIUM)
        compare_hdr_write();
      end else begin
        `uvm_info("SB",
          $sformatf("HDR READ transaction detected: addr=0x%0x len=%0d",
                    exp_address, exp_length), UVM_MEDIUM)
        compare_hdr_read();
      end
    end else if(is_daa_transaction()) begin
      `uvm_info("SB",
        $sformatf("DAA transaction detected: cmd_type=0x%0x ccc=0x%0x",
                  exp_cmd_type, exp_ccc), UVM_MEDIUM)
      compare_with_daa_target();
    end else begin
      compare_with_target();
    end
  end
endtask


// collect_apb_transaction
// Accumulates WDATAB bytes and breaks on CTRL write with cmd_start=1
task i3c_scoreboard::collect_apb_transaction();
  apb_master_tx apb_pkt;
  exp_write_data.delete();
  exp_hdr_write_data.delete();

  forever begin
    apb_analysis_fifo.get(apb_pkt);
    apb_tx_count++;

    // WDATAB write (addr 0x30) — collect for both SDR and HDR write
    if(apb_pkt.pwrite == apb_global_pkg::WRITE &&
       apb_pkt.paddr[6:0] == 7'h30) begin
      exp_write_data.push_back(apb_pkt.pwdata[7:0]);
      exp_hdr_write_data.push_back(apb_pkt.pwdata[7:0]);
      exp_rd_wr_data.push_back(apb_pkt.pwdata[7:0]);
      `uvm_info("SB",
        $sformatf("WDATAB collected = 0x%0x", apb_pkt.pwdata[7:0]), UVM_HIGH)
    end

    // CTRL write with cmd_start=1 (bit[31])
    if(apb_pkt.pwrite == apb_global_pkg::WRITE &&
       apb_pkt.paddr[6:0] == 7'h0C &&
       apb_pkt.pwdata[31] == 1'b1) begin
      decode_ctrl(apb_pkt.pwdata);
      `uvm_info("SB", $sformatf(
        "CTRL decoded: addr=0x%0x dir=%0b len=%0d cmd_type=0x%0x ccc=0x%0x cmd_mode=%0b",
        exp_address, exp_direction, exp_length,
        exp_cmd_type, exp_ccc, exp_cmd_mode), UVM_MEDIUM)
      break;
    end
  end
endtask


// decode_ctrl — extract all fields including bit[26]=cmd_mode (HDR flag)
function void i3c_scoreboard::decode_ctrl(bit [31:0] ctrl_val);
  exp_address   = ctrl_val[6:0];
  exp_length    = ctrl_val[14:7];
  exp_direction = ctrl_val[15];
  exp_ccc       = ctrl_val[23:16];
  exp_cmd_type  = ctrl_val[25:24];
  exp_cmd_mode  = ctrl_val[26];      // 1 = HDR mode
endfunction


// ── HDR WRITE comparison ──────────────────────────────────────────────
// The DUT sends DDR frames; target BFM receives them and reports via
// target_analysis_port. We compare the bytes the DUT sent (from WDATAB)
// against what the target monitor captured.
task i3c_scoreboard::compare_hdr_write();
  i3c_target_tx tgt;
  hdr_txn_count++;

  // Get the target monitor's captured HDR_WRITE transaction
  target_analysis_fifo.get(tgt);
  target_tx_count++;

  `uvm_info("SB_HDR", $sformatf("HDR WRITE: tgt pkt:\n%s", tgt.sprint()),
            UVM_HIGH)

  // ── Address check
  if (exp_address == tgt.targetAddress)
    `uvm_info("SB_HDR_ADDR_MATCH",
      $sformatf("HDR WRITE addr match: 0x%0x", exp_address), UVM_MEDIUM)
  else
    `uvm_error("SB_HDR_ADDR_MISMATCH",
      $sformatf("HDR WRITE addr: expected 0x%0x got 0x%0x",
                exp_address, tgt.targetAddress))

  // ── txn_type check
  if (tgt.txn_type !== i3c_target_tx::HDR_WRITE)
    `uvm_error("SB_HDR_TXN_TYPE",
      $sformatf("Expected HDR_WRITE from target but got %s",
                tgt.txn_type.name()))

  // ── Data comparison: WDATAB bytes vs target captured writeData
  begin
    int n_exp    = exp_hdr_write_data.size();
    int n_actual = tgt.writeData.size();

    `uvm_info("SB_HDR_WRITE",
      $sformatf("HDR WRITE: APB loaded %0d bytes, CTRL len=%0d, target got %0d bytes",
                n_exp, exp_length, n_actual), UVM_MEDIUM)

    if (n_exp == 0 && n_actual == 0) begin
      `uvm_info("SB_HDR_WRITE", "HDR WRITE: no data payload to compare", UVM_MEDIUM)
    end else begin
      // Compare up to the smaller of the two
      for (int i = 0; i < n_actual; i++) begin
        bit [7:0] exp_val = (i < n_exp) ? exp_hdr_write_data[i] : 8'hFF;
        if (exp_val == tgt.writeData[i]) begin
          `uvm_info("SB_HDR_WDATA_MATCH",
            $sformatf("HDR writeData[%0d]: exp=0x%0x got=0x%0x",
                      i, exp_val, tgt.writeData[i]), UVM_MEDIUM)
          hdr_write_pass++;
        end else begin
          `uvm_error("SB_HDR_WDATA_MISMATCH",
            $sformatf("HDR writeData[%0d]: exp=0x%0x got=0x%0x",
                      i, exp_val, tgt.writeData[i]))
          hdr_write_fail++;
        end
      end

      if (n_exp > n_actual)
        `uvm_info("SB_HDR_SHORT",
          $sformatf("HDR WRITE: DUT sent fewer bytes (%0d) than loaded (%0d)",
                    n_actual, n_exp), UVM_LOW)
    end
  end
endtask : compare_hdr_write


// ── HDR READ comparison ───────────────────────────────────────────────
// The target drives DDR frames back to the DUT; DUT stores in Rx FIFO.
// APB reads RDATAB to retrieve what the DUT captured.
// We compare: target's readData (what it drove) vs APB prdata (what DUT got).

task i3c_scoreboard::compare_hdr_read();
  i3c_target_tx  tgt;
  apb_master_tx  rd_pkt;
  bit [7:0]      apb_read_data[$];
  int            rd_count = 0;
  bit            got_tgt_txn = 0;
  hdr_txn_count++;

  // Get the target transaction (target monitor captures what it drove),
  // bounded by a timeout so a missing/failed target transaction is
  // reported as an error instead of blocking forever.
  fork
    begin
      target_analysis_fifo.get(tgt);
      got_tgt_txn = 1;
    end
    begin
      #(HDR_READ_TIMEOUT_NS);  // define alongside other scoreboard params, e.g. 10000
    end
  join_any
  disable fork;

  if (!got_tgt_txn) begin
    `uvm_error("SB_HDR_READ_TIMEOUT",
      "HDR READ: no target transaction received within timeout — target likely NACK'd address or never entered HDR mode")
    return;
  end

  target_tx_count++;



  `uvm_info("SB_HDR", $sformatf("HDR READ: tgt pkt:\n%s", tgt.sprint()),
            UVM_HIGH)

  // ── Address check
  if (exp_address == tgt.targetAddress)
    `uvm_info("SB_HDR_ADDR_MATCH",
      $sformatf("HDR READ addr match: 0x%0x", exp_address), UVM_MEDIUM)
  else
    `uvm_error("SB_HDR_ADDR_MISMATCH",
      $sformatf("HDR READ addr: expected 0x%0x got 0x%0x",
                exp_address, tgt.targetAddress))

  // ── txn_type check
  if (tgt.txn_type !== i3c_target_tx::HDR_READ)
    `uvm_error("SB_HDR_TXN_TYPE",
      $sformatf("Expected HDR_READ from target but got %s",
                tgt.txn_type.name()))

  // ── Collect RDATAB reads from APB (what DUT stored in Rx FIFO)
  while (rd_count < int'(exp_length)) begin
    apb_analysis_fifo.get(rd_pkt);
    apb_tx_count++;
    if (rd_pkt.pwrite == apb_global_pkg::READ &&
        rd_pkt.paddr[6:0] == 7'h40) begin
      apb_read_data.push_back(rd_pkt.prdata[7:0]);
      `uvm_info("SB_HDR_RDATAB",
        $sformatf("HDR RDATAB[%0d] = 0x%0x", rd_count, rd_pkt.prdata[7:0]),
        UVM_MEDIUM)
      rd_count++;
    end
  end

  // ── Compare: target.readData[i] (what target drove) == apb_read_data[i] (what DUT got)
  begin
    int n_tgt = tgt.readData.size();
    int n_apb = apb_read_data.size();

    `uvm_info("SB_HDR_READ",
      $sformatf("HDR READ: target drove %0d bytes, DUT RDATAB returned %0d bytes",
                n_tgt, n_apb), UVM_MEDIUM)

    for (int i = 0; i < n_apb && i < n_tgt; i++) begin
      if (tgt.readData[i] == apb_read_data[i]) begin
        `uvm_info("SB_HDR_RDATA_MATCH",
          $sformatf("HDR readData[%0d]: target drove 0x%0x, DUT got 0x%0x",
                    i, tgt.readData[i], apb_read_data[i]), UVM_MEDIUM)
        hdr_read_pass++;
      end else begin
        `uvm_error("SB_HDR_RDATA_MISMATCH",
          $sformatf("HDR readData[%0d]: target drove 0x%0x, DUT got 0x%0x",
                    i, tgt.readData[i], apb_read_data[i]))
        hdr_read_fail++;
      end
    end

    if (n_tgt != n_apb)
      `uvm_info("SB_HDR_READ_SIZE",
        $sformatf("HDR READ size: target=%0d APB=%0d", n_tgt, n_apb), UVM_LOW)
  end
endtask : compare_hdr_read


task i3c_scoreboard::compare_with_daa_target();
  i3c_target_tx tgt;
  bit [6:0]     exp_dyn_addr;

  target_analysis_fifo.get(tgt);
  target_tx_count++;
  daa_devices_seen++;

  `uvm_info("SB_DAA",
    $sformatf("DAA target pkt[%0d]:\n%s", daa_devices_seen, tgt.sprint()),
    UVM_HIGH)

  if(tgt.txn_type !== i3c_target_tx::DAA) begin
    `uvm_error("SB_DAA_TXN_TYPE",
      $sformatf("Expected DAA transaction but got txn_type=%s",
                tgt.txn_type.name()))
    return;
  end

  `uvm_info("SB_DAA_CTRL_ADDR",
    $sformatf("CTRL address field = 0x%0h (DAA uses broadcast implicitly)",
              exp_address), UVM_MEDIUM)

  if(exp_cmd_type == CMD_TYPE_CCC) begin
    if(exp_ccc == ENTDAA_CCC_CODE)
      `uvm_info("SB_DAA_CTRL_CCC",
        "CTRL CCC = 0x07 (ENTDAA) ", UVM_MEDIUM)
    else
      `uvm_error("SB_DAA_CTRL_CCC",
        $sformatf("cmd_type=2 but CCC=0x%0h, expected ENTDAA=0x07",
                  exp_ccc))
  end else begin
    `uvm_info("SB_DAA_CTRL_CCC",
      "cmd_type=3 (explicit DAA)", UVM_MEDIUM)
  end

  if(exp_cmd_type == CMD_TYPE_DAA)
    `uvm_info("SB_DAA_CTRL_CMD",
      "CTRL cmd_type = 3 (explicit DAA) ", UVM_MEDIUM)
  else if(exp_cmd_type == CMD_TYPE_CCC && exp_ccc == ENTDAA_CCC_CODE)
    `uvm_info("SB_DAA_CTRL_CMD",
      "CTRL cmd_type = 2 + CCC=ENTDAA  ", UVM_MEDIUM)
  else
    `uvm_error("SB_DAA_CTRL_CMD",
      $sformatf("CTRL cmd_type=0x%0x ccc=0x%0x does not indicate DAA",
                exp_cmd_type, exp_ccc))

  `uvm_info("SB_DAA_DEVICE_INFO",
    $sformatf("Device[%0d]: PID=0x%0h BCR=0x%0h DCR=0x%0h",
              daa_devices_seen-1, tgt.pid, tgt.bcr, tgt.dcr), UVM_LOW)

  exp_dyn_addr = daa_next_exp_addr;
  if(tgt.dynamic_address !== exp_dyn_addr) begin
    `uvm_error("SB_DAA_DYNADDR",
      $sformatf("Dynamic address: expected 0x%0h got 0x%0h",
                exp_dyn_addr, tgt.dynamic_address))
    daa_addr_fail++;
  end else begin
    `uvm_info("SB_DAA_DYNADDR",
      $sformatf("Dynamic address 0x%0h ", tgt.dynamic_address), UVM_MEDIUM)
    daa_addr_pass++;
    daa_next_exp_addr++;
  end

  if(tgt.daa_ack === ACK) begin
    `uvm_info("SB_DAA_PARITY",
      $sformatf("Parity OK for addr 0x%0h (daa_ack=ACK) ",
                tgt.dynamic_address), UVM_MEDIUM)
    daa_parity_pass++;
  end else begin
    `uvm_error("SB_DAA_PARITY",
      $sformatf("Parity FAIL or NACK for addr 0x%0h (daa_ack=NACK)",
                tgt.dynamic_address))
    daa_parity_fail++;
  end

  if(tgt.bcr[7] !== 1'b0)
    `uvm_error("SB_DAA_BCR_ROLE",
      $sformatf("BCR[7] must be 0 (target role) but got 1 for PID 0x%0h",
                tgt.pid))
  else
    `uvm_info("SB_DAA_BCR_ROLE",
      "BCR[7]=0 (target role) ", UVM_MEDIUM)

endtask : compare_with_daa_target


task i3c_scoreboard::compare_with_target();
  i3c_target_tx tgt;

  target_analysis_fifo.get(tgt);
  target_tx_count++;

  `uvm_info("SB", $sformatf("Target pkt:\n%s", tgt.sprint()), UVM_HIGH)

  // Address check 
  if(exp_address == tgt.targetAddress)
    `uvm_info("SB_ADDR_MATCH",
      $sformatf("Address match: 0x%0x", exp_address), UVM_MEDIUM)
  else
    `uvm_error("SB_ADDR_MISMATCH",
      $sformatf("Address: expected 0x%0x  got 0x%0x",
                exp_address, tgt.targetAddress))

  //  Operation check
  begin
    operationType_e exp_op = (exp_direction == 1'b0) ?
                             i3c_globals_pkg::WRITE :
                             i3c_globals_pkg::READ;
    if(exp_op == tgt.operation)
      `uvm_info("SB_OP_MATCH",
        $sformatf("Operation match: %s", exp_op.name()), UVM_MEDIUM)
    else
      `uvm_error("SB_OP_MISMATCH",
        $sformatf("Operation: expected %s  got %s",
                  exp_op.name(), tgt.operation.name()))
  end

  //  WRITE path 
  if(exp_direction == 1'b0) begin
    int actual_bytes = tgt.writeData.size();
    `uvm_info("SB", $sformatf(
      "Write: APB sent %0d bytes, CTRL length=%0d, target received %0d bytes",
      exp_write_data.size(), exp_length, actual_bytes), UVM_MEDIUM)

    for(int i = 0; i < actual_bytes; i++) begin
      bit [7:0] exp_val;
      if(i < exp_write_data.size())
        exp_val = exp_write_data[i];
      else
        exp_val = 8'hFF;

      if(exp_val == tgt.writeData[i][7:0]) begin
        `uvm_info("SB_WDATA_MATCH",
          $sformatf("writeData[%0d]: expected 0x%0x got 0x%0x",
                    i, exp_val, tgt.writeData[i][7:0]), UVM_MEDIUM)
        write_pass++;
      end else begin
        `uvm_error("SB_WDATA_MISMATCH",
          $sformatf("writeData[%0d]: expected 0x%0x got 0x%0x",
                    i, exp_val, tgt.writeData[i][7:0]))
        write_fail++;
      end
    end

    if(exp_write_data.size() > actual_bytes)
      `uvm_info("SB_FIFO_OVERFLOW",
        $sformatf("APB sent %0d bytes, target got %0d, RTL dropped %0d",
                  exp_write_data.size(), actual_bytes,
                  exp_write_data.size() - actual_bytes), UVM_MEDIUM)

  // ── READ
  end else begin
    bit [7:0]     apb_read_data[$];
    apb_master_tx rd_pkt;
    int           rd_count = 0;

    while(rd_count < int'(exp_length)) begin
      apb_analysis_fifo.get(rd_pkt);
      apb_tx_count++;
      if(rd_pkt.pwrite == apb_global_pkg::READ &&
         rd_pkt.paddr[6:0] == 7'h40) begin
        apb_read_data.push_back(rd_pkt.prdata[7:0]);
        `uvm_info("SB",
          $sformatf("RDATAB[%0d] = 0x%0x", rd_count, rd_pkt.prdata[7:0]),
          UVM_HIGH)
        rd_count++;
      end
    end

    if(apb_read_data.size() != tgt.readData.size()) begin
      `uvm_error("SB_RDATA_SIZE",
        $sformatf("Read size mismatch: apb=%0d target=%0d",
                  apb_read_data.size(), tgt.readData.size()))
    end else begin
      for(int i = 0; i < tgt.readData.size(); i++) begin
        bit [7:0] exp_val;
        if(i < exp_rd_wr_data.size())
          exp_val = exp_rd_wr_data[i];
        else begin
          exp_val = 8'hFF;
          `uvm_warning("SB_RDATA_EMPTY", "exp_rd_wr_data queue too small")
        end

        if(exp_val == tgt.readData[i][7:0]) begin
          `uvm_info("SB_RDATA_MATCH",
            $sformatf("readData[%0d]: expected 0x%0x got 0x%0x",
                      i, exp_val, tgt.readData[i][7:0]), UVM_MEDIUM)
          read_pass++;
        end else begin
          `uvm_error("SB_RDATA_MISMATCH",
            $sformatf("readData[%0d]: expected 0x%0x got 0x%0x",
                      i, exp_val, tgt.readData[i][7:0]))
          read_fail++;
        end
      end
    end
    exp_rd_wr_data.delete();
  end
endtask : compare_with_target


function void i3c_scoreboard::check_phase(uvm_phase phase);
  super.check_phase(phase);

  `uvm_info("SB_SUMMARY", $sformatf({
    "\n============= SCOREBOARD SUMMARY =============\n",
    "  APB transactions seen      : %0d\n",
    "  I3C target transactions    : %0d\n",
    "  -- SDR --\n",
    "  Write byte pass / fail     : %0d / %0d\n",
    "  Read  byte pass / fail     : %0d / %0d\n",
    "  -- DAA --\n",
    "  Devices seen               : %0d\n",
    "  Dyn address pass / fail    : %0d / %0d\n", 
    "  Parity / ACK pass / fail   : %0d / %0d\n",
    "  -- HDR --\n",
    "  HDR transactions           : %0d\n",
    "  HDR write byte pass / fail : %0d / %0d\n",
    "  HDR read  byte pass / fail : %0d / %0d\n",
    "=============================================="},
    apb_tx_count,    target_tx_count,
    write_pass,      write_fail,
    read_pass,       read_fail,
    daa_devices_seen,
    daa_addr_pass,   daa_addr_fail,   
    daa_parity_pass, daa_parity_fail,
    hdr_txn_count,
    hdr_write_pass,  hdr_write_fail,
    hdr_read_pass,   hdr_read_fail),
    UVM_NONE)

  if(write_fail != 0)
    `uvm_error("SB_SUMMARY", "Write data mismatches detected")
  if(read_fail != 0)
    `uvm_error("SB_SUMMARY", "Read data mismatches detected")
  if(hdr_write_fail != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("%0d HDR write data mismatches detected", hdr_write_fail))
  if(hdr_read_fail != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("%0d HDR read data mismatches detected", hdr_read_fail))

  if(daa_addr_fail != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("%0d DAA dynamic address mismatches detected", daa_addr_fail))
  if(daa_parity_fail != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("%0d DAA parity/ACK failures detected", daa_parity_fail))

  if(i3c_env_cfg_h.has_daa &&
     daa_devices_seen != i3c_env_cfg_h.no_of_daa_devices)
    `uvm_error("SB_SUMMARY",
      $sformatf("DAA device count: expected %0d, saw %0d",
                i3c_env_cfg_h.no_of_daa_devices, daa_devices_seen))

  if(apb_analysis_fifo.size() != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("APB FIFO not empty: %0d leftover packets",
                apb_analysis_fifo.size()))
  if(target_analysis_fifo.size() != 0)
    `uvm_error("SB_SUMMARY",
      $sformatf("Target FIFO not empty: %0d leftover packets",
                target_analysis_fifo.size()))
endfunction : check_phase
    
`endif    


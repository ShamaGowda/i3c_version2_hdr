`ifndef I3C_TARGET_MONITOR_BFM_INCLUDED_
`define I3C_TARGET_MONITOR_BFM_INCLUDED_
 
import i3c_globals_pkg::*; 
 
interface i3c_target_monitor_bfm(input pclk, 
                                input areset, 
                                input scl_i,
                                input scl_o,
                                input scl_oen,
                                input sda_i,
                                input sda_o,
                                input sda_oen);
 
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import i3c_target_pkg::*;
  import i3c_target_pkg::i3c_target_monitor_proxy;
  
  i3c_target_monitor_proxy i3c_target_mon_proxy_h; 
  i3c_fsm_state_e state;
 
  string name = "I3C_TARGET_MONITOR_BFM";
 
 
  localparam logic [7:0] BCAST_7E_W  = 8'hFC;
  localparam logic [7:0] ENTDAA_CODE = 8'h07;
  localparam logic [7:0] BCAST_7E_R  = 8'hFD;
  localparam int         ARB_BIT_CNT = 64;
 
  initial begin
    $display("target Monitor BFM");
  end
 
  
  task wait_for_reset();
    @(negedge areset);
    @(posedge areset);
  endtask: wait_for_reset
 
  task sample_idle_state();
    @(posedge pclk);
  endtask: sample_idle_state
 
  task wait_for_idle_state();
    @(posedge pclk);
    while(scl_i!=1 && sda_i!=1) begin
      @(posedge pclk);
    end
    state = IDLE;
  endtask: wait_for_idle_state
 
  
  task sample_data(inout i3c_transfer_bits_s struct_packet,
                   inout i3c_transfer_cfg_s  struct_cfg);
    detect_start();
    sample_target_address(struct_packet);
    sample_operation(struct_packet.operation);
    sampleAddressAck(struct_packet.targetAddressStatus);
    if(struct_packet.targetAddressStatus == ACK) begin
      if(struct_packet.operation == WRITE) begin
        sampleWriteDataAndACK(struct_packet, struct_cfg);
      end else begin
        sampleReadDataAndACK(struct_packet, struct_cfg);
      end
    end else begin
      detect_stop();
    end
  endtask: sample_data
 
 
  task sample_daa(output i3c_target_tx daa_txn_q[$]);
    bit        more_devices;
    bit [7:0]  rx_byte;
    bit        addr_ack;
 
    daa_txn_q  = {};
    more_devices = 1;
 
    //START + broadcast 7E+W 
    detect_start();
    `uvm_info(name, "DAA: START detected", UVM_MEDIUM)
 
    sample_byte(rx_byte);
    if(rx_byte !== BCAST_7E_W) begin
      `uvm_error(name, $sformatf(
        "DAA: expected 7E+W (0x%0h), got 0x%0h", BCAST_7E_W, rx_byte))
      return;
    end
    `uvm_info(name, "DAA: 7E+W sampled", UVM_MEDIUM)
 
    sample_ack(addr_ack);
    if(addr_ack !== ACK) begin
      `uvm_info(name, "DAA: NACK on 7E+W - no devices, aborting", UVM_MEDIUM)
      detect_stop();
      return;
    end
 
    // ENTDAA CCC byte 
    sample_byte(rx_byte);
    if(rx_byte !== ENTDAA_CODE) begin
      `uvm_error(name, $sformatf(
        "DAA: expected ENTDAA (0x07), got 0x%0h", rx_byte))
      return;
    end
    `uvm_info(name, "DAA: ENTDAA byte sampled", UVM_MEDIUM)
 
    while(more_devices) begin
      i3c_target_tx  txn;
      bit [63:0]     arb_shift;
      bit [7:0]      assign_byte;
      bit            dyn_parity;
 
      txn = i3c_target_tx::type_id::create("daa_txn");
      txn.txn_type = i3c_target_tx::DAA;
 
      // Repeated START 
      detect_start();
      `uvm_info(name, "DAA: Repeated START detected", UVM_MEDIUM)
 
      sample_byte(rx_byte);
      if(rx_byte !== BCAST_7E_R) begin
        `uvm_error(name, $sformatf(
          "DAA: expected 7E+R (0x%0h), got 0x%0h", BCAST_7E_R, rx_byte))
        return;
      end
      `uvm_info(name, "DAA: 7E+R sampled", UVM_MEDIUM)
 
      // ACK after 7E+R 
      sample_ack(addr_ack);
      if(addr_ack !== ACK) begin
        `uvm_info(name, "DAA: NACK on 7E+R - no more devices", UVM_MEDIUM)
        detect_stop();
        more_devices = 0;
        break;
      end
 
      // ARB_BITS 
      `uvm_info(name, "DAA: sampling 64 ARB bits {PID,BCR,DCR}", UVM_MEDIUM)
      sample_arb_bits(arb_shift);
      txn.pid = arb_shift[63:16];
      txn.bcr = arb_shift[15:8];
      txn.dcr = arb_shift[7:0];
      `uvm_info(name, $sformatf(
        "DAA: PID=0x%0h BCR=0x%0h DCR=0x%0h",
        txn.pid, txn.bcr, txn.dcr), UVM_MEDIUM)
 
 
      sample_byte(assign_byte);
      txn.dynamic_address = assign_byte[7:1];
      dyn_parity          = assign_byte[0];
      `uvm_info(name, $sformatf(
        "DAA: dynamic_address=0x%0h parity=%0b",
        txn.dynamic_address, dyn_parity), UVM_MEDIUM)
 
      if(dyn_parity !== (~^txn.dynamic_address)) begin
        `uvm_error(name, $sformatf(
          "DAA: parity mismatch for addr 0x%0h (got %0b, expected %0b)",
          txn.dynamic_address, dyn_parity, ~^txn.dynamic_address))
      end
 
      sample_ack(addr_ack);
      txn.daa_ack = addr_ack;
      `uvm_info(name, $sformatf(
        "DAA: assign ACK=%0b", txn.daa_ack), UVM_MEDIUM)
 
      daa_txn_q.push_back(txn);
 
     
      detect_next_daa_condition(more_devices);
 
    end
 
    `uvm_info(name, $sformatf(
      "DAA: complete, %0d device(s) assigned", daa_txn_q.size()), UVM_LOW)
 
  endtask: sample_daa
 
 
  task sample_byte(output bit [7:0] data);
    for(int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      data[k] = sda_i;
    end
  endtask: sample_byte
 

 
  task sample_arb_bits(output bit [63:0] arb_shift);
    arb_shift = 64'h0;
    for(int i = 0; i < ARB_BIT_CNT; i++) begin
      detectEdge_scl(POSEDGE);
      arb_shift = {arb_shift[62:0], sda_i};
    end
  endtask: sample_arb_bits
 
 
 
  task detect_next_daa_condition(output bit more_devices);
    bit [1:0] scl_local;
    bit [1:0] sda_local;
 
    scl_local = 2'b11;
    sda_local = 2'b11;
 
    forever begin
      @(negedge pclk);
      #1;
      scl_local = {scl_local[0], scl_i};
      sda_local = {sda_local[0], sda_i};
 
      if(sda_local == POSEDGE && scl_local == 2'b11) begin
        `uvm_info(name, "DAA: STOP detected after ASSIGN", UVM_MEDIUM)
        more_devices = 0;
        state        = STOP;
        return;
      end
 
  
      if(sda_local == NEGEDGE && scl_local == 2'b11) begin
        `uvm_info(name, "DAA: Repeated START detected after ASSIGN", UVM_MEDIUM)
        more_devices = 1;
        state        = START;
        return;
      end
    end
  endtask: detect_next_daa_condition
 
 
 
  task sampleWriteDataAndACK(inout i3c_transfer_bits_s structPacket,
                             input i3c_transfer_cfg_s  structConfig);
    fork
      begin
        for(int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_write_data(structPacket, i,
                            structConfig.dataTransferDirection);
          sampleWdataAck(structPacket.writeDataStatus[i]);
          if(structPacket.writeDataStatus[i] == NACK)
            break;
        end
      end
    join_none
 
    wrDetect_stop();
    disable fork;
  endtask: sampleWriteDataAndACK 
 
 
  task sampleReadDataAndACK(inout i3c_transfer_bits_s structPacket,
                             input i3c_transfer_cfg_s  structConfig);
    fork
      begin
        for(int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_read_data(structPacket, i,
                           structConfig.dataTransferDirection);
          sample_ack(structPacket.readDataStatus[i]);
          if(structPacket.readDataStatus[i] == NACK)
            break;
        end
      end
    join_none
 
    wrDetect_stop();
    disable fork;
  endtask: sampleReadDataAndACK 
  
 
  task detect_start();
    bit [1:0] scl_local;
    bit [1:0] sda_local;
    state = START;
  
    do begin
      @(negedge pclk);
      scl_local = {scl_local[0], scl_i};
      sda_local = {sda_local[0], sda_i};
    end while(!(sda_local == NEGEDGE && scl_local == 2'b11));
    $display("MONITOR :: checking start at time %0t scl_local=%0b sda_local=%0b",
             $time, scl_local, sda_local);
  endtask: detect_start
  
 
  task sample_target_address(inout i3c_transfer_bits_s pkt);
    bit [TARGET_ADDRESS_WIDTH-1:0] address;
    state = ADDRESS;
    for(int k = TARGET_ADDRESS_WIDTH-1; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      address[k] = sda_i;
    end
    pkt.targetAddress = address;
  endtask: sample_target_address
  
 
  task sample_operation(output operationType_e wr_rd);
    bit operation;
    state = WR_BIT;
    detectEdge_scl(POSEDGE);
    operation = sda_i;
    if(operation == 0)
      wr_rd = WRITE;
    else
      wr_rd = READ;
  endtask: sample_operation
  
 
  task sampleAddressAck(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
  endtask: sampleAddressAck
  
 
  task sample_write_data(inout i3c_transfer_bits_s pkt,
                         input int                  i,
                         input dataTransferDirection_e dir);
    bit [DATA_WIDTH-1:0] wdata;
    state = WRITE_DATA;
 
    `uvm_info("DEBUG_TARGET_MONITOR_BFM",
      $sformatf("dir %s ", dir.name()), UVM_HIGH);
    for(int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (dir == MSB_FIRST) ? ((DATA_WIDTH - 1) - k) : k;
      detectEdge_scl(POSEDGE);
      wdata[bit_no] = sda_i;
      `uvm_info("DEBUG_MSHA_MONITOR_BFM",
        $sformatf(" bit_no=%0d sda_i=%0d wdata[bit_no]=%0d\n wdata=%0b",
          bit_no, sda_i, wdata[bit_no], wdata), UVM_HIGH);
      pkt.no_of_i3c_bits_transfer++;
    end
    pkt.writeData[i] = wdata;
    `uvm_info("DEBUG_TARGET_MONITOR_BFM",
      $sformatf(" i=%0d wdata=0x%0h", i, wdata), UVM_HIGH);
  endtask: sample_write_data
  
 
  task sampleWdataAck(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
  endtask: sampleWdataAck
  
 
  task sample_read_data(inout i3c_transfer_bits_s pkt,
                        input int                  i,
                        input dataTransferDirection_e dir);
    bit [DATA_WIDTH-1:0] rdata;
    state = READ_DATA;
    for(int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (dir == MSB_FIRST) ? ((DATA_WIDTH - 1) - k) : k;
      detectEdge_scl(POSEDGE);
      rdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end
    pkt.readData[i] = rdata;
  endtask: sample_read_data
  
 
  task sample_ack(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
  endtask: sample_ack
  
 
  task wrDetect_stop();
    bit [1:0] scl_local;
    bit [1:0] sda_local;
 
    do begin
      @(negedge pclk);
      #1;
      scl_local = {scl_local[0], scl_i};
      sda_local = {sda_local[0], sda_i};
    end while(!(sda_local == POSEDGE && scl_local == 2'b11));
    state = STOP;
    `uvm_info(name, $sformatf("Stop condition is detected"), UVM_HIGH);
  endtask: wrDetect_stop
  
 
  task detect_stop();
    bit [1:0] scl_local;
    bit [1:0] sda_local;
    state = STOP;
  
    do begin
      @(negedge pclk);
      #1;
      scl_local = {scl_local[0], scl_i};
      sda_local = {sda_local[0], sda_i};
    end while(!(sda_local == POSEDGE && scl_local == 2'b11));
  endtask: detect_stop
  
 
  task detectEdge_scl(input edge_detect_e edgeSCL);
    bit [1:0]     scl_local;
    edge_detect_e scl_edge_value;
    scl_local = 2'b11;
 
    do begin
      @(negedge pclk);
      scl_local = {scl_local[0], scl_i};
    end while(!(scl_local == edgeSCL));
 
    scl_edge_value = edge_detect_e'(scl_local);
    `uvm_info("TARGET_DRIVER_BFM",
      $sformatf("scl %s detected", scl_edge_value.name()), UVM_HIGH);
  endtask: detectEdge_scl


///////////////////////////////////////////////HDR//////////////////////

task sample_hdr_write();

 detect_start();

 sample_target_address();

 sample_operation();

 sampleAddressAck();

 sample_hdr_entry();

 sample_hdr_ddr_data();

 sample_hdr_crc();

 sample_hdr_exit();

endtask


task sample_hdr_entry();

   `uvm_info("HDR","HDR Entry detected",UVM_LOW)

endtask

task sample_hdr_ddr_data();

 bit [15:0] word;

 repeat(expected_words)
 begin

    sample_hdr_word(word);

    tx.hdr_data.push_back(word);

 end

endtask


task sample_hdr_crc();

 bit [4:0] rx_crc;
 bit [4:0] calc_crc;

 rx_crc = sample_crc();

 calc_crc = calculate_hdr_crc(tx.hdr_data);

 if(rx_crc != calc_crc)
 begin

   `uvm_error("HDR",
      $sformatf("CRC mismatch exp=%0h got=%0h",
      calc_crc,rx_crc))

 end

endtask



task sample_hdr_exit();

   `uvm_info("HDR","HDR Exit detected",UVM_LOW)

endtask





task sample_hdr_read(
  inout i3c_transfer_bits_s struct_packet,
  inout i3c_transfer_cfg_s struct_cfg
);

  detect_start();
  sample_target_address(struct_packet);
  sample_operation(struct_packet.operation);
  sampleAddressAck(struct_packet.targetAddressStatus);

  if(struct_packet.targetAddressStatus == ACK)
    sampleReadDataAndACK(struct_packet, struct_cfg);
  else
    detect_stop();

endtask


//////////////////////////////////////////////////////////////////////////////////
endinterface : i3c_target_monitor_bfm
 
`endif

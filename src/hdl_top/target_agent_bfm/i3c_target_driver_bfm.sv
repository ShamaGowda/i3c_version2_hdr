`ifndef I3C_TARGET_DRIVER_BFM_INCLUDED_
`define I3C_TARGET_DRIVER_BFM_INCLUDED_

import i3c_globals_pkg::*;

interface i3c_target_driver_bfm(
  input  pclk,
  input  areset,
  input  scl_i,
  output reg scl_o,
  output reg scl_oen,
  input  sda_i,
  output reg sda_o,
  output reg sda_oen
);

  i3c_fsm_state_e state;
  bit [7:0]  rdata;
  bit [1:0]  scl_local = 2'b11;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import i3c_target_pkg::i3c_target_driver_proxy;

  i3c_target_driver_proxy i3c_target_drv_proxy_h;

  bit [DATA_WIDTH-1:0] targetFIFOMemory[$];

  string name = "I3C_TARGET_DRIVER_BFM";

  initial begin
    $display(name);
  end


  task wait_for_system_reset();
    state = RESET_DEACTIVATED;
    @(negedge areset);
    state = RESET_ACTIVATED;
    @(posedge areset);
    state = RESET_DEACTIVATED;
  endtask : wait_for_system_reset

  task drive_idle_state();
    @(posedge pclk);
    drive_scl(1);
    drive_sda(1);
    state <= IDLE;
    `uvm_info(name, "inside idle state", UVM_HIGH)
  endtask : drive_idle_state

  task wait_for_idle_state();
    @(posedge pclk);
    while(scl_i != 1 && sda_i != 1) begin
      @(posedge pclk);
    end
    state = IDLE;
    `uvm_info(name, "I3C bus is free state detected", UVM_NONE)
  endtask : wait_for_idle_state


  task drive_data(inout i3c_transfer_bits_s dataPacketStruck,
                  input i3c_transfer_cfg_s  configPacketStruck);
    `uvm_info(name, "target txn started", UVM_HIGH)
    detect_start();
    sample_target_address(configPacketStruck, dataPacketStruck);
    sample_operation(dataPacketStruck.operation);
    driveAddressAck(dataPacketStruck.targetAddressStatus);

    if(dataPacketStruck.targetAddressStatus == ACK) begin
      `uvm_info(name, "targetAddressStatus is ACK", UVM_HIGH)
      if(dataPacketStruck.operation == WRITE)
        sampleWriteDataAndDriveACK(dataPacketStruck, configPacketStruck);
      else
        driveReadDataAndSampleACK(dataPacketStruck, configPacketStruck);
    end else begin
      `uvm_info(name, "targetAddressStatus is NACK", UVM_HIGH)
      detect_stop();
    end
  endtask : drive_data

  task sampleWriteDataAndDriveACK(
    inout i3c_transfer_bits_s dataPacketStruck,
    input i3c_transfer_cfg_s  configPacketStruck);

    `uvm_info(name, "sampleWriteDataAndDriveACK started", UVM_HIGH)
    fork
      begin
        for(int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_write_data(configPacketStruck, dataPacketStruck, i);
          driveWdataAck(dataPacketStruck.writeDataStatus[i]);
          if(dataPacketStruck.writeDataStatus[i] == NACK)
            break;
        end
      end
    join_none
    `uvm_info(name, "sampleWriteDataAndDriveACK done", UVM_HIGH)
    wrDetect_stop();
    disable fork;
  endtask : sampleWriteDataAndDriveACK

  task driveReadDataAndSampleACK(
    inout i3c_transfer_bits_s dataPacketStruck,
    input i3c_transfer_cfg_s  configPacketStruck);

    `uvm_info(name, "driveReadDataAndSampleACK started", UVM_HIGH)
    fork
      begin
        for(int i = 0; i < MAXIMUM_BYTES; i++) begin
          if(targetFIFOMemory.size() == 0)
            rdata = configPacketStruck.defaultReadData;
          else
            rdata = targetFIFOMemory.pop_front();

          drive_read_data(rdata, dataPacketStruck, i,
                          configPacketStruck.dataTransferDirection);
          sample_ack(dataPacketStruck.readDataStatus[i]);
          if(dataPacketStruck.readDataStatus[i] == NACK)
            break;
        end
      end
    join_none
    wrDetect_stop();
    disable fork;
  endtask : driveReadDataAndSampleACK



task drive_daa_data(inout i3c_transfer_bits_s dataPacketStruck,
                    input i3c_transfer_cfg_s  configPacketStruck,
                    output bit [47:0] pid_out,
                    output bit [7:0]  bcr_out,
                    output bit [7:0]  dcr_out,
                    output bit [6:0]  dyn_addr_out,
                    output bit        daa_ack_out);

  bit [63:0] arb_shift;
  bit [7:0]  dyn_addr_with_parity;

  `uvm_info(name, "DAA transaction started", UVM_NONE)

  detect_start();
  `uvm_info(name, "DAA: START detected", UVM_NONE)

  sample_daa_broadcast_address(dataPacketStruck);
  sample_daa_ccc_byte(dataPacketStruck);
  detect_repeated_start();
  sample_daa_broadcast_read(dataPacketStruck);

  
  drive_daa_pid_bcr_dcr(
    configPacketStruck,
    dataPacketStruck.pid,  
    dataPacketStruck.bcr,  
    dataPacketStruck.dcr,  
    arb_shift,
    pid_out,
    bcr_out,
    dcr_out
  );

  sample_daa_dynamic_address(
    dyn_addr_out,
    dyn_addr_with_parity,
    daa_ack_out
  );

  driveAddressAck(daa_ack_out);
  detect_stop();

  `uvm_info(name, "DAA: STOP detected - DAA complete", UVM_NONE)

  dataPacketStruck.targetAddress       = 7'h7E;
  dataPacketStruck.targetAddressStatus = ACK;

endtask : drive_daa_data



task sample_daa_broadcast_address(inout i3c_transfer_bits_s pkt);
  bit [6:0] addr_bits;
  bit       rw_bit;
  bit [7:0] full_byte;

  `uvm_info(name, "DAA: sampling broadcast 0x7E+W", UVM_HIGH)


  for(int k = 6; k >= 0; k--) begin
    detectEdge_scl(POSEDGE);
    addr_bits[k] = sda_i;
    drive_sda(1);
  end

  
  detectEdge_scl(POSEDGE);
  rw_bit = sda_i;
  drive_sda(1);

  full_byte = {addr_bits, rw_bit};
  `uvm_info(name,
    $sformatf("DAA: broadcast addr = 0x%0x (expect 0xFC)", full_byte),
    UVM_NONE)

 
  detectEdge_scl(NEGEDGE);
  drive_sda(1'b0);          
  detectEdge_scl(POSEDGE);  
  detectEdge_scl(NEGEDGE);   
  drive_sda(1'b1);

endtask : sample_daa_broadcast_address

 
task sample_daa_ccc_byte(inout i3c_transfer_bits_s pkt);
  bit [7:0] ccc_byte;
  `uvm_info(name, "DAA: sampling ENTDAA CCC byte", UVM_HIGH)

  for(int k = 7; k >= 0; k--) begin
    detectEdge_scl(POSEDGE);
    ccc_byte[k] = sda_i;
    drive_sda(1);  
  end
  `uvm_info(name, $sformatf("DAA: CCC byte = 0x%0x (expect 0x07)", ccc_byte), UVM_NONE)


  detectEdge_scl(NEGEDGE);
  drive_sda(1'b0);          
  detectEdge_scl(POSEDGE);   
  detectEdge_scl(NEGEDGE);   
  drive_sda(1'b1);

endtask : sample_daa_ccc_byte

  
  task detect_repeated_start();
    bit [1:0] scl_loc;
    bit [1:0] sda_loc;

  
    do begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
    end while(!(sda_loc == NEGEDGE && scl_loc == 2'b11));

    `uvm_info(name, "DAA: Repeated START detected", UVM_HIGH)
  endtask : detect_repeated_start


 
task sample_daa_broadcast_read(inout i3c_transfer_bits_s pkt);
  bit [6:0] addr_bits;
  bit       rw_bit;
  bit [7:0] full_byte;

  `uvm_info(name, "DAA: sampling broadcast 0x7E+R", UVM_HIGH)

  detectEdge_scl(NEGEDGE); 


  for(int k = 6; k >= 0; k--) begin
    detectEdge_scl(POSEDGE);
    addr_bits[k] = sda_i;
    drive_sda(1);
  end

 
  detectEdge_scl(POSEDGE);
  rw_bit = sda_i;
  drive_sda(1);

  full_byte = {addr_bits, rw_bit};
  `uvm_info(name,
    $sformatf("DAA: broadcast read addr = 0x%0x (expect 0xFD)", full_byte),
    UVM_NONE)


  detectEdge_scl(NEGEDGE);
  drive_sda(1'b0);       
  detectEdge_scl(POSEDGE);   
  detectEdge_scl(NEGEDGE);   
  drive_sda(1'b1);       

endtask : sample_daa_broadcast_read

task drive_daa_pid_bcr_dcr(
  input  i3c_transfer_cfg_s cfg,
  input  bit [47:0]  pid_in,
  input  bit [7:0]   bcr_in,
  input  bit [7:0]   dcr_in,
  output bit [63:0] arb_shift_out,
  output bit [47:0] pid_out,
  output bit [7:0]  bcr_out,
  output bit [7:0]  dcr_out);


  bit [63:0] full_id;
  bit [7:0]  curr_byte;

  full_id = {pid_in, bcr_in, dcr_in};

`uvm_info(name,
    $sformatf("DAA: driving PID=%0h BCR=%0h DCR=%0h",
              pid_in, bcr_in, dcr_in), UVM_NONE)



 
  for(int byte_idx = 7; byte_idx >= 0; byte_idx--) begin
    curr_byte = full_id[byte_idx*8 +: 8];
    for(int bit_idx = 7; bit_idx >= 0; bit_idx--) begin
       drive_sda(curr_byte[bit_idx]);  
    detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
    end
  end


  drive_sda(1'b1);

  arb_shift_out = full_id;
  pid_out       = pid_in;
  bcr_out       = bcr_in;
  dcr_out       = dcr_in;


  `uvm_info(name, "DAA: PID+BCR+DCR driven on bus", UVM_HIGH)
endtask : drive_daa_pid_bcr_dcr



  task sample_daa_dynamic_address(
    output bit [6:0] dyn_addr_out,
    output bit [7:0] full_byte_out,
    output bit       ack_out);

    bit [7:0] addr_byte;
    bit       parity_received;
    bit       parity_calc;

  
  `uvm_info(name, "DAA: sampling dynamic address", UVM_HIGH)


    for(int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      addr_byte[k] = sda_i;
      drive_sda(1);
    end

    dyn_addr_out   = addr_byte[7:1];
    parity_received = addr_byte[0];
    full_byte_out  = addr_byte;


    parity_calc = ~^addr_byte[7:1];

    if(parity_calc == parity_received) begin
      ack_out = ACK;
      `uvm_info(name,
        $sformatf("DAA: dynamic addr=0x%0x parity OK → ACK",
                  dyn_addr_out), UVM_NONE)
    end else begin
      ack_out = NACK;
      `uvm_info(name,
        $sformatf("DAA: dynamic addr=0x%0x parity FAIL → NACK",
                  dyn_addr_out), UVM_NONE)
    end

  endtask : sample_daa_dynamic_address




  task detect_start();
    bit [1:0] scl_local;
    bit [1:0] sda_local;
    state = START;
    `uvm_info(name, "detect_start waiting", UVM_HIGH)
    do begin
      @(negedge pclk);
      scl_local = {scl_local[0], scl_i};
      sda_local = {sda_local[0], sda_i};
    end while(!(sda_local == NEGEDGE && scl_local == 2'b11));
    `uvm_info(name, "Start condition detected", UVM_HIGH)
  endtask : detect_start


task sample_target_address(
  input  i3c_transfer_cfg_s cfg_pkt,
  inout  i3c_transfer_bits_s pkt);

  bit [TARGET_ADDRESS_WIDTH-1:0] local_addr;
  `uvm_info(name, "sample_target_address started", UVM_HIGH)
  state = ADDRESS;


  detectEdge_scl(NEGEDGE);  

  for(int k = TARGET_ADDRESS_WIDTH-1; k >= 0; k--) begin
    detectEdge_scl(POSEDGE);
    local_addr[k] = sda_i;
    `uvm_info(name,
      $sformatf("sampled bit %0d = %0d", k, sda_i), UVM_HIGH)
    drive_sda(1);
  end

  `uvm_info(name,
    $sformatf("DEBUG :: local_addr = 0x%0x", local_addr[6:0]), UVM_NONE)
  pkt.targetAddress = local_addr;

  `uvm_info(name,
    $sformatf("DEBUG :: cfg target_addr = 0x%0x",
              cfg_pkt.targetAddress), UVM_NONE)

  if(local_addr != cfg_pkt.targetAddress) begin
    pkt.targetAddressStatus = NACK;
    `uvm_info(name, "address mismatch NACK", UVM_HIGH)
  end else begin
    pkt.targetAddressStatus = ACK;
    `uvm_info(name, "address match ACK", UVM_HIGH)
  end
endtask : sample_target_address

  task sample_operation(output operationType_e wr_rd);
    bit operation;
    state = WR_BIT;
    detectEdge_scl(POSEDGE);
    operation = sda_i;
    drive_sda(1);

    if(operation == 1'b0) begin
      wr_rd = WRITE;
      `uvm_info(name, "operation = WRITE", UVM_HIGH)
    end else begin
      wr_rd = READ;
      `uvm_info(name, "operation = READ", UVM_HIGH)
    end
  endtask : sample_operation

task driveAddressAck(input bit ack);
  `uvm_info(name, $sformatf("driveAddressAck = %0d", ack), UVM_HIGH)
  state = ACK_NACK;
  detectEdge_scl(NEGEDGE);   
  drive_sda(ack);          
  detectEdge_scl(POSEDGE);   
  detectEdge_scl(NEGEDGE);   
  drive_sda(1'b1);            
endtask : driveAddressAck

//////////////////////////HDR////////////////////////////////////////////////////////////////////////////////
task drive_hdr_write(
  inout i3c_transfer_bits_s pkt,
  input i3c_transfer_cfg_s cfg
);

  `uvm_info(name,
    "HDR WRITE transaction started",
    UVM_HIGH)

  detect_start();

  sample_target_address(cfg,pkt);

  sample_operation(pkt.operation);

  driveAddressAck(pkt.targetAddressStatus);

  if(pkt.targetAddressStatus == ACK)
    sampleWriteDataAndDriveACK(pkt,cfg);

  else
    detect_stop();

endtask


task drive_hdr_read(
  inout i3c_transfer_bits_s pkt,
  input i3c_transfer_cfg_s cfg
);

  `uvm_info(name,
    "HDR READ transaction started",
    UVM_HIGH)

  detect_start();

  sample_target_address(cfg,pkt);

  sample_operation(pkt.operation);

  driveAddressAck(pkt.targetAddressStatus);

  if(pkt.targetAddressStatus == ACK)
    driveReadDataAndSampleACK(pkt,cfg);

  else
    detect_stop();

endtask







  task sample_write_data(
    input  i3c_transfer_cfg_s cfg_pkt,
    inout  i3c_transfer_bits_s pkt,
    input  int i);

    bit [DATA_WIDTH-1:0] wdata;
    state = WRITE_DATA;

    `uvm_info("DEBUG_TARGET_DRIVER_BFM",
      $sformatf("dir %s", cfg_pkt.dataTransferDirection.name()), UVM_HIGH)

    for(int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (cfg_pkt.dataTransferDirection == MSB_FIRST) ?
               ((DATA_WIDTH - 1) - k) : k;
      detectEdge_scl(POSEDGE);
      `uvm_info(name,
        $sformatf("write data bit %0d = %0d", bit_no, sda_i), UVM_HIGH)
      wdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end

    `uvm_info(name,
      $sformatf("DEBUG :: write data = 0x%0x", wdata[7:0]), UVM_NONE)
    targetFIFOMemory.push_back(wdata);
    `uvm_info("DEBUG_READ",
      $sformatf("write fifo size = %0d",
                targetFIFOMemory.size()), UVM_HIGH)
    pkt.writeData[i] = wdata;
  endtask : sample_write_data

task driveWdataAck(input bit ack);
  state = ACK_NACK;
  `uvm_info(name, $sformatf("driveWdataAck = %0d", ack), UVM_HIGH)
  detectEdge_scl(NEGEDGE);
  drive_sda(ack);
  detectEdge_scl(POSEDGE);   
  detectEdge_scl(NEGEDGE);
  drive_sda(1'b1);
endtask : driveWdataAck


  task drive_read_data(
    input bit [7:0]             rdata,
    inout i3c_transfer_bits_s   pkt,
    input int                   i,
    input dataTransferDirection_e dir);

    `uvm_info("DEBUG",
      $sformatf("Driving byte = %0b", rdata), UVM_NONE)
    state = READ_DATA;

    for(int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (dir == MSB_FIRST) ? ((DATA_WIDTH - 1) - k) : k;
      `uvm_info(name,
        $sformatf("read data bit %0d = %0d",
                  bit_no, rdata[bit_no]), UVM_HIGH)
      drive_sda(rdata[bit_no]);
      pkt.no_of_i3c_bits_transfer++;
      detectEdge_scl(NEGEDGE);
    end
    pkt.readData[i] = rdata;
    drive_sda(1);
  endtask : drive_read_data

  task sample_ack(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
  endtask : sample_ack

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
    `uvm_info(name, "Stop condition detected", UVM_HIGH)
  endtask : wrDetect_stop

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
    `uvm_info(name, "Stop condition detected", UVM_HIGH)
  endtask : detect_stop

  task drive_sda(input bit value);
    sda_oen <= value ? TRISTATE_BUF_OFF : TRISTATE_BUF_ON;
    sda_o   <= value;
  endtask : drive_sda

  task drive_scl(input bit value);
    scl_oen <= value ? TRISTATE_BUF_OFF : TRISTATE_BUF_ON;
    scl_o   <= value;
  endtask : drive_scl

  task detectEdge_scl(input edge_detect_e edgeSCL);
    edge_detect_e scl_edge_value;
    do begin
      @(negedge pclk);
      scl_local = {scl_local[0], scl_i};
    end while(!(scl_local == edgeSCL));
    scl_edge_value = edge_detect_e'(scl_local);
    `uvm_info("TARGET_DRIVER_BFM",
      $sformatf("scl %s detected",
                scl_edge_value.name()), UVM_HIGH)
  endtask : detectEdge_scl

endinterface : i3c_target_driver_bfm

`endif

`ifndef I3C_GLOBALS_PKG_INCLUDED_
`define I3C_GLOBALS_PKG_INCLUDED_

package i3c_globals_pkg;

  // NO_OF_TARGETS to be connected to the i3c_interface
  parameter int NO_OF_CONTROLLERS = 1;

  // NO_OF_MASTERS to be connected to the i3c_interface
  parameter int NO_OF_TARGETS = 1;
  
  // The parameter NO_OF_REG is to assign number of registers in a slave
  parameter int NO_OF_REG = 1;
  
  // The parameter for the data width
  parameter int DATA_WIDTH = 8;
  
  // The parameter for the slave address width
  parameter int TARGET_ADDRESS_WIDTH  = 7;
  
  // The parameter for the register address width
  parameter int REGISTER_ADDRESS_WIDTH  = 8;
  
  // The parameter for MAXIMUM_BITS supported per transfer
  parameter int MAXIMUM_BITS = 1024;
  
  // The parameter for MAXIMUM_BYTES supported per transfer
  parameter int MAXIMUM_BYTES = MAXIMUM_BITS/DATA_WIDTH ;
  
  // The parameter for Slave addresses
  parameter TARGET0_ADDRESS = 7'b110_1000;  // 7'h68
  parameter TARGET1_ADDRESS = 7'b110_1100;  // 7'h6C 
  parameter TARGET2_ADDRESS = 7'b111_1100;  // 7'h7C
  parameter TARGET3_ADDRESS = 7'b100_1100;  // 7'h4C
  
  // The parameter for enabling tristate buffer
  parameter bit TRISTATE_BUF_ON  = 1;

  // The parameter for disaling tristate buffer
  parameter bit TRISTATE_BUF_OFF = 0;
  
  parameter BUS_IDLE_TIME = 1;  // 200ns as per spec table 86
  parameter BUS_FREE_TIME = 1;  // 0.5us as per spec page no 365 Table 85

// I3C broadcast address (7-bit)
  parameter bit [6:0] I3C_BROADCAST_ADDR  = 7'h7E;

  // ENTDAA CCC code 
  parameter bit [7:0] ENTDAA_CCC_CODE     = 8'h07;

  // Broadcast address byte on bus {7'h7E, W=0} = 8'hFC
  parameter bit [7:0] BCAST_ADDR_WRITE    = 8'hFC;

  // Broadcast address byte on bus: {7'h7E, R=1} = 8'hFD
  parameter bit [7:0] BCAST_ADDR_READ     = 8'hFD;

  // Total arbitration bits: PID(48)+BCR(8)+DCR(8)
  parameter int       DAA_ARB_BIT_COUNT   = 64;

  parameter bit [6:0] DAA_FIRST_DYN_ADDR  = 7'h08;

  // CTRL register cmd_type encoding
  parameter bit [1:0] CMD_TYPE_DAA        = 2'd3;
  parameter bit [1:0] CMD_TYPE_SDR = 2'b00;
  parameter bit [1:0] CMD_TYPE_CCC = 2'b10;
  
  typedef enum bit {
    MSB_FIRST = 1'b0,
    LSB_FIRST = 1'b1
  } dataTransferDirection_e;
  
  typedef enum bit {
    TRUE = 1'b1,
    FALSE = 1'b0
  } hasCoverage_e;
  
  // Enum: operationType_e
  // 
  // Specifies the read or write request
  // READ - READ request 
  // WRITE - WRITE request
  typedef enum bit {
    WRITE = 1'b0,
    READ = 1'b1
  } operationType_e;
  
  typedef enum bit[1:0] {
    ONLY_WRITE  = 2'b00,
    ONLY_READ   = 2'b01,
    WRITE_READ  = 2'b10
  } writeReadMode_e;

  typedef enum bit[1:0] {
    SDR = 2'b00,
    DAA = 2'b01,
    HDR_WRITE = 2'b10,
    HDR_READ = 2'b11

  } txn_type_e;


  // struct: i3c_bits_transfer_s
  typedef struct {
    bit [TARGET_ADDRESS_WIDTH-1:0]targetAddress;
    bit operation;
    bit targetAddressStatus;
    bit writeDataStatus[MAXIMUM_BYTES];
    bit readDataStatus[MAXIMUM_BYTES];
    bit [DATA_WIDTH-1:0] writeData[MAXIMUM_BYTES];
    bit [DATA_WIDTH-1:0] readData[MAXIMUM_BYTES];
    int no_of_i3c_bits_transfer; 
    bit [REGISTER_ADDRESS_WIDTH-1:0]register_address;
  bit [1:0] txn_type;
    bit [47:0]                       pid;
    bit [7:0]                        bcr;
    bit [7:0]                        dcr;
    bit [6:0]                        dynamic_address;
    bit                              daa_ack;

   } i3c_transfer_bits_s;
  
  
  // struct: i3c_transfer_cfg_s
  // 
  // msb_first: specifies the shift direction
  // operation : read from or write to slave 
  typedef struct {
    dataTransferDirection_e dataTransferDirection;
    bit operation;
    int clockRateDividerValue;
    bit[TARGET_ADDRESS_WIDTH-1:0] targetAddress;
    bit [DATA_WIDTH-1:0]defaultReadData;
 bit [47:0]                       pid;
    bit [7:0]                        bcr;
    bit [7:0]                        dcr;
    bit                              daa_accept_address;  
} i3c_transfer_cfg_s;
  
  // enum: i3c_fsm_state_e
  //
  // declared state
  typedef enum int{
    RESET_DEACTIVATED,
    RESET_ACTIVATED,
    IDLE,
    FREE,
    START, 
    ADDRESS,
    WR_BIT,
    ACK_NACK,
    WRITE_DATA,
    READ_DATA,
    STOP
  }i3c_fsm_state_e;


typedef enum bit [3:0] {
    DAA_IDLE      = 4'd0,
    DAA_SEND_7E_W = 4'd1,
    DAA_ENTDAA    = 4'd2,
    DAA_REP_START = 4'd3,
    DAA_SEND_7E_R = 4'd4,
    DAA_ARB_BITS  = 4'd5,
    DAA_ASSIGN    = 4'd6,
    DAA_LOOP      = 4'd7,
    DAA_STOP      = 4'd8
  } daa_fsm_state_e;
  
  // Enum: edge_detect_e
  //
  // Used for detecting the edge on the signal
  //
  // POSEDGE - posedge on the signal, the transition from 0->1
  // NEGEDGE - negedge on the signal, the transition from 0->1
  //
  typedef enum bit[1:0] {
    POSEDGE = 2'b01,
    NEGEDGE = 2'b10
  } edge_detect_e;

  // Enum: acknowledge_e
  //
  // Specifies the acknowledgement type
  //
  // POS_ACK - positive acknowledgement 
  // NEG_ACK - negative acknowledgement
  typedef enum bit {
    ACK = 1'b0,
    NACK = 1'b1
  } acknowledge_e;

endpackage : i3c_globals_pkg 

`endif



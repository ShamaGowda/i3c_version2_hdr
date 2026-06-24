/*CTRL REG
[31]      start
[30:26]   reserved
[25:24]   cmd_type
[23:16]   CCC
[15]      direction
[14:7]    length
[6:0]     address
*/

class i3c_ctrl_reg extends uvm_reg;

`uvm_object_utils(i3c_ctrl_reg)

  rand uvm_reg_field start;
       uvm_reg_field reserved;
  rand uvm_reg_field cmd_type;
  rand uvm_reg_field cmd_mode;
  rand uvm_reg_field ccc;
  rand uvm_reg_field direction;
  rand uvm_reg_field length;
  rand uvm_reg_field address;

  function new(string name = "i3c_ctrl_reg");
    super.new(name, 32, UVM_NO_COVERAGE);
  endfunction

  virtual function void build();
    address = uvm_reg_field::type_id::create("address");
    address.configure(this, 7, 0, "RW", 0, 0, 1, 0, 0);

    length = uvm_reg_field::type_id::create("length");
    length.configure(this, 8, 7, "RW", 0, 0, 1, 0, 0);

    direction = uvm_reg_field::type_id::create("direction");
    direction.configure(this, 1, 15, "RW", 0, 0, 1, 0, 0);

    ccc = uvm_reg_field::type_id::create("ccc");
    ccc.configure(this, 8, 16, "RW", 0, 0, 1, 0, 0);

    cmd_type = uvm_reg_field::type_id::create("cmd_type");
    cmd_type.configure(this, 2, 24, "RW", 0, 0, 1, 0, 0);

   cmd_mode = uvm_reg_field::type_id::create("cmd_mode");
   cmd_mode.configure(this, 1, 26, "RW", 0, 0, 1, 0, 0);

   
    reserved = uvm_reg_field::type_id::create("reserved");
    reserved.configure(this, 4, 27, "RO", 0, 0, 1, 0, 0);


    start = uvm_reg_field::type_id::create("start");
    start.configure(this, 1, 31, "RW", 0, 0, 1, 0, 0);

  endfunction

endclass



class i3c_wdatab_reg extends uvm_reg;

`uvm_object_utils(i3c_wdatab_reg)

  rand uvm_reg_field tx_data;

  function new(string name = "i3c_wdatab_reg");
    super.new(name, 8, UVM_NO_COVERAGE);
  endfunction

  virtual function void build();
    tx_data = uvm_reg_field::type_id::create("tx_data");
    tx_data.configure(this, 8, 0, "RW", 0, 0, 1, 0, 0);
  endfunction

endclass

//rdatab_reg
class i3c_rdatab_reg extends uvm_reg;
 `uvm_object_utils(i3c_rdatab_reg) 

  uvm_reg_field rx_data;

  function new(string name = "i3c_rdatab_reg");
    super.new(name, 8, UVM_NO_COVERAGE);
  endfunction

  virtual function void build();
    rx_data = uvm_reg_field::type_id::create("rx_data");
    rx_data.configure(this, 8, 0, "RO", 0, 0, 1, 0, 0);
  endfunction

endclass

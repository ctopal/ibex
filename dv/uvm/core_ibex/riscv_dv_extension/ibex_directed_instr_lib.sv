// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0


// Define a short riscv-dv directed instruction stream to setup the hardware breakpoint (TSELECT/TDATA) functionality, and then trigger on an instruction at the end of this stream.
class ibex_breakpoint_stream extends riscv_directed_instr_stream;

  riscv_pseudo_instr   la_instr;
  riscv_instr          ebreak_insn;
  rand int unsigned    num_of_instr;

  constraint instr_c {
    num_of_instr inside {[5:10]};
  }

  `uvm_object_utils(ibex_breakpoint_stream)

  function new(string name = "");
    super.new(name);
  endfunction

  function void post_randomize();
    riscv_instr      instr;
    string           trigger_label, gn;

    // Setup a randomized main body of instructions.
    initialize_instr_list(num_of_instr);
    setup_allowed_instr(1, 1); // no_branch=1/no_load_store=1
    foreach(instr_list[i]) begin
      instr = riscv_instr::type_id::create("instr");
      randomize_instr(instr);
      instr_list[i] = instr;
      // Copy this from the base-class behaviour
      instr_list[i].atomic = 1'b1;
      instr_list[i].has_label = 1'b0;
    end

    // Give the last insn of the main body a label, then set the breakpoint address to that label.
    gn = get_name();
    trigger_label = {gn};
    instr_list[$].label = trigger_label;
    instr_list[$].has_label = 1'b1;

    // Load the address of the trigger point as the (last insn of the stream + 4)
    // Store in gpr[1] ()
    la_instr = riscv_pseudo_instr::type_id::create("la_instr");
    la_instr.pseudo_instr_name = LA;
    la_instr.imm_str           = $sformatf("%0s+4", trigger_label);
    la_instr.rd                = cfg.gpr[1];

    // Create the ebreak insn which will cause us to enter debug mode, and run the
    // special code in the debugrom.
    ebreak_insn = riscv_instr::get_instr(EBREAK);

    // Add the instructions into the stream.
    instr_list = {la_instr,
                  ebreak_insn,
                  instr_list};
  endfunction

endclass

// Define a short riscv-dv directed instruction stream to write random values to MSECCFG CSR
class ibex_rand_mseccfg_stream extends riscv_directed_instr_stream;

  `uvm_object_utils(ibex_rand_mseccfg_stream)

  function new(string name = "");
    super.new(name);
  endfunction

  function void post_randomize();
    riscv_instr          csrrw_instr;
    // Setup a randomized main body of instructions.
    initialize_instr_list(1);
    setup_allowed_instr(1, 1); // no_branch=1/no_load_store=1

    csrrw_instr = riscv_instr::get_instr(CSRRWI);
    csrrw_instr.comment = "lmao";
    csrrw_instr.atomic = 1'b0;
    csrrw_instr.has_label = 1'b1;
    csrrw_instr.csr = MSECCFG;
    csrrw_instr.rd = '0;
    csrrw_instr.imm_str = $sformatf("0x%0x", $urandom_range(7,0));
    //insert_instr(csrrw_instr);
    instr_list = {csrrw_instr};
  endfunction

endclass

// Define a short riscv-dv directed instruction stream to set a valid NA4 address/config
class ibex_valid_na4_cfg_stream extends riscv_directed_instr_stream;

  `uvm_object_utils(ibex_valid_na4_cfg_stream)

  function new(string name = "");
    super.new(name);
  endfunction

  function void post_randomize();
    riscv_instr          cfg_csrrw_instr;
    // Setup a randomized main body of instructions.
    initialize_instr_list(1);
    setup_allowed_instr(1, 1); // no_branch=1/no_load_store=1

    cfg_csrrw_instr = riscv_instr::get_instr(CSRRSI);
    cfg_csrrw_instr.comment = "cfg0 set";
    cfg_csrrw_instr.atomic = 1'b0;
    cfg_csrrw_instr.has_label = 1'b1;
    cfg_csrrw_instr.csr = PMPCFG0;
    cfg_csrrw_instr.rd = '0;
    cfg_csrrw_instr.imm_str = $sformatf("%0d", $urandom_range(16,23));

    //insert_instr(csrrw_instr);
    instr_list = {cfg_csrrw_instr};
  endfunction
endclass

// Define a short riscv-dv directed instruction stream to set a valid NA4 address/config
class ibex_valid_na4_addr_stream extends riscv_directed_instr_stream;

  `uvm_object_utils(ibex_valid_na4_addr_stream)

  function new(string name = "");
    super.new(name);
  endfunction

  function void post_randomize();
    string               instr_label, gn;
    riscv_pseudo_instr   la_instr;
    riscv_instr          addr_csrrw_instr;
    riscv_instr          srli_instr;
    // Setup a randomized main body of instructions.
    initialize_instr_list(3);
    //setup_allowed_instr(1, 1); // no_branch=1/no_load_store=1

    // Load the address of the trigger point as the (last insn of the stream + 4)
    // Store in gpr[1] ()
    la_instr = riscv_pseudo_instr::type_id::create("la_instr");
    la_instr.pseudo_instr_name = LA;
    la_instr.imm_str           = $sformatf("_start");
    la_instr.rd                = cfg.gpr[1];


    srli_instr = riscv_instr::get_instr(SRLI);
    srli_instr.label = instr_label;
    srli_instr.comment = "srli set";
    srli_instr.atomic = 1'b0;
    srli_instr.has_label = 1'b1;
    srli_instr.rs1 = cfg.gpr[1];
    srli_instr.rd = cfg.gpr[1];
    srli_instr.imm_str = $sformatf("2");

    gn = get_name();
    instr_label = {gn};
    addr_csrrw_instr = riscv_instr::get_instr(CSRRW);
    addr_csrrw_instr.label = instr_label;
    addr_csrrw_instr.comment = "addr0 set";
    addr_csrrw_instr.atomic = 1'b0;
    addr_csrrw_instr.has_label = 1'b1;
    addr_csrrw_instr.csr = PMPADDR0;
    addr_csrrw_instr.rs1 = cfg.gpr[1];
    addr_csrrw_instr.rd = '0;
    instr_list = {la_instr, srli_instr, addr_csrrw_instr};
  endfunction

endclass
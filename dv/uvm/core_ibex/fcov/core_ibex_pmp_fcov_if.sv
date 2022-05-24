// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`include "prim_assert.sv"

interface core_ibex_pmp_fcov_if import ibex_pkg::*; #(
    parameter bit          PMPEnable      = 1'b0,
    // Granularity of NAPOT access,
    // 0 = No restriction, 1 = 8 byte, 2 = 16 byte, 3 = 32 byte, etc.
    parameter int unsigned PMPGranularity = 0,
    // Number of implemented regions
    parameter int unsigned PMPNumRegions  = 4
) (
  input clk_i,
  input rst_ni,

  input ibex_pkg::pmp_cfg_t  csr_pmp_cfg     [PMPNumRegions],
  input logic                pmp_req_err     [3],
  input pmp_mseccfg_t        csr_pmp_mseccfg,

  input logic data_req_out,

  input fcov_csr_write
);
  `include "dv_fcov_macros.svh"
  import uvm_pkg::*;

  // Enum to give more readable coverage results for privilege bits. 4 bits are from the pmpcfg CSF
  // (L, X, W, R) and the 5th is the MML bit. Setting the MML bit changes the meaning of the other
  // 4 bits
  typedef enum logic [4:0] {
    NONE        = 5'b00000,
    R           = 5'b00001,
    W           = 5'b00010,
    WR          = 5'b00011,
    X           = 5'b00100,
    XR          = 5'b00101,
    XW          = 5'b00110,
    XWR         = 5'b00111,
    L           = 5'b01000,
    LR          = 5'b01001,
    LW          = 5'b01010,
    LWR         = 5'b01011,
    LX          = 5'b01100,
    LXR         = 5'b01101,
    LXW         = 5'b01110,
    LXWR        = 5'b01111,
    MML_NONE    = 5'b10000,
    MML_RU      = 5'b10001,
    MML_WRM_RU  = 5'b10010,
    MML_WRU     = 5'b10011,
    MML_XU      = 5'b10100,
    MML_XRU     = 5'b10101,
    MML_WRM_WRU = 5'b10110,
    MML_XWRU    = 5'b10111,
    MML_L       = 5'b11000,
    MML_RM      = 5'b11001,
    MML_XM_XU   = 5'b11010,
    MML_WRM     = 5'b11011,
    MML_XM      = 5'b11100,
    MML_XRM     = 5'b11101,
    MML_XRM_XU  = 5'b11110,
    MML_RM_RU   = 5'b11111
  } pmp_priv_bits_e;

  // Break out PMP signals into individually named signals for direct use in `cross` as it cannot
  // deal with hierarchical references or unpacked arrays.
  logic pmp_iside_req_err;
  logic pmp_iside2_req_err;
  logic pmp_dside_req_err;

  assign pmp_iside_req_err  = pmp_req_err[PMP_I];
  assign pmp_iside2_req_err = pmp_req_err[PMP_I2];
  assign pmp_dside_req_err  = pmp_req_err[PMP_D];

  bit en_pmp_fcov;

  logic csr_wdata_mseccfg_rlb;
  logic csr_wdata_mseccfg_mmwp;
  logic csr_wdata_mseccfg_mml;

  assign csr_wdata_mseccfg_rlb = cs_registers_i.csr_wdata_int[CSR_MSECCFG_RLB_BIT];
  assign csr_wdata_mseccfg_mmwp = cs_registers_i.csr_wdata_int[CSR_MSECCFG_MMWP_BIT];
  assign csr_wdata_mseccfg_mml = cs_registers_i.csr_wdata_int[CSR_MSECCFG_MML_BIT];

  initial begin
    if (PMPEnable) begin
      void'($value$plusargs("enable_ibex_fcov=%d", en_pmp_fcov));
    end else begin
      en_pmp_fcov = 1'b0;
    end
  end

  if (PMPEnable) begin : g_pmp_cgs
    logic [PMPNumRegions-1:0] pmp_iside_match;
    logic [PMPNumRegions-1:0] pmp_iside2_match;
    logic [PMPNumRegions-1:0] pmp_dside_match;

    assign pmp_iside_match    = g_pmp.pmp_i.region_match_all[PMP_I];
    assign pmp_iside2_match   = g_pmp.pmp_i.region_match_all[PMP_I2];
    assign pmp_dside_match    = g_pmp.pmp_i.region_match_all[PMP_D];

    for (genvar i_region = 0; i_region < PMPNumRegions; i_region += 1) begin : g_pmp_region_fcov
      pmp_priv_bits_e pmp_region_priv_bits;

      assign pmp_region_priv_bits = pmp_priv_bits_e'({csr_pmp_mseccfg.mml,
                                                      csr_pmp_cfg[i_region].lock,
                                                      csr_pmp_cfg[i_region].exec,
                                                      csr_pmp_cfg[i_region].write,
                                                      csr_pmp_cfg[i_region].read});

      covergroup pmp_region_cg @(posedge clk_i);
        option.per_instance = 1;
        option.name = "pmp_region_cg";

        cp_rlb : coverpoint csr_pmp_mseccfg.rlb;

        cp_mmwp : coverpoint csr_pmp_mseccfg.mmwp;

        cp_mml : coverpoint csr_pmp_mseccfg.mml;

        cp_region_mode : coverpoint csr_pmp_cfg[i_region].mode;

        cp_region_priv_bits : coverpoint pmp_region_priv_bits {
          wildcard illegal_bins illegal = {5'b0??10};
        }

        cp_priv_lvl_iside : coverpoint g_pmp.pmp_priv_lvl[PMP_I] {
          illegal_bins illegal = {PRIV_LVL_H, PRIV_LVL_S};
        }

        cp_req_type_iside : coverpoint g_pmp.pmp_req_type[PMP_I] {
          illegal_bins illegal = {PMP_ACC_WRITE, PMP_ACC_READ};
        }

        cp_priv_lvl_iside2 : coverpoint g_pmp.pmp_priv_lvl[PMP_I2] {
          illegal_bins illegal = {PRIV_LVL_H, PRIV_LVL_S};
        }

        cp_req_type_iside2 : coverpoint g_pmp.pmp_req_type[PMP_I2] {
          illegal_bins illegal = {PMP_ACC_WRITE, PMP_ACC_READ};
        }

        cp_priv_lvl_dside : coverpoint g_pmp.pmp_priv_lvl[PMP_D] {
          illegal_bins illegal = {PRIV_LVL_H, PRIV_LVL_S};
        }

        cp_req_type_dside : coverpoint g_pmp.pmp_req_type[PMP_D] {
          illegal_bins illegal = {PMP_ACC_EXEC};
        }

        pmp_iside_mode_cross : cross cp_region_mode, pmp_iside_req_err
          iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_ichan_access);

        pmp_iside2_mode_cross : cross cp_region_mode, pmp_iside2_req_err
          iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_ichan2_access);

        pmp_dside_mode_cross : cross cp_region_mode, pmp_dside_req_err
          iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_dchan_access);

        pmp_iside_nomatch_cross :
          cross cp_region_priv_bits, cp_req_type_iside, cp_priv_lvl_iside, pmp_iside_req_err,
                cp_mmwp
            iff (pmp_iside_match && csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {
            // Will never see a succesful exec access when execute is disallowed
            illegal_bins illegal_user_allow_exec =
              // In User mode - no match case, we should always deny
              (binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_U} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {0});
            illegal_bins illegal_machine_allow_exec =
              // In Machine mode - no match case, we should deny always except MML=0 & MMWP=0
              (binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {0})
              with ((!cp_mmwp) && !(cp_region_priv_bits & 5'b10000));
            // Will never see an exec access denied when executed is ignored
            wildcard illegal_bins illegal_deny_exec =
              // Ignore Exec in MML=0, MMWP=0 and in Machine mode
              (binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(cp_region_priv_bits) intersect {5'b0????} &&
               binsof(pmp_iside_req_err) intersect {1})
              with (!cp_mmwp);
            }

        pmp_iside2_nomatch_cross :
          cross cp_region_priv_bits, cp_req_type_iside2, cp_priv_lvl_iside2, pmp_iside2_req_err,
                cp_mmwp
            iff (pmp_iside2_match && csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {
            // Will never see a succesful exec access when execute is disallowed
            illegal_bins illegal_user_allow_exec =
              // In User mode - no match case, we should always deny
              (binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_U} &&
               binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside2_req_err) intersect {0});
            illegal_bins illegal_machine_allow_exec =
              // In Machine mode - no match case, we should deny always except MML=0 & MMWP=0
              (binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside2_req_err) intersect {0})
              with ((!cp_mmwp) && !(cp_region_priv_bits & 5'b10000));
            // Will never see an exec access denied when executed is ignored
            wildcard illegal_bins illegal_deny_exec =
              // Ignore Exec in MML=0, MMWP=0 and in Machine mode
              (binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
               binsof(cp_region_priv_bits) intersect {5'b0????} &&
               binsof(pmp_iside2_req_err) intersect {1}) with (!cp_mmwp);
            }

        pmp_dside_nomatch_cross :
          cross cp_region_priv_bits, cp_req_type_dside, cp_priv_lvl_dside, pmp_dside_req_err,
                cp_mmwp
            iff (pmp_dside_match && csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {

            // Will never see a succesful write/read access when it should be denied
            illegal_bins illegal_machine_allow_wr =
              // Deny RW when MMWP = 1 in Machine mode
              (binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_dside) intersect {PMP_ACC_READ, PMP_ACC_WRITE} &&
               binsof(pmp_dside_req_err) intersect {0}) with (cp_mmwp);
            illegal_bins illegal_user_allow_wr =
              // Deny RW everytime in User mode
              (binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_U} &&
               binsof(cp_req_type_dside) intersect {PMP_ACC_READ, PMP_ACC_WRITE} &&
               binsof(pmp_dside_req_err) intersect {0});

            // Will never see a write/read access denied when write/read is ignored
            illegal_bins illegal_deny_wr =
              // Ignore RW when MMWP = 0 in Machine mode
              (binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_dside) intersect {PMP_ACC_READ, PMP_ACC_WRITE} &&
               binsof(pmp_dside_req_err) intersect {1}) with (!cp_mmwp);
            }

        pmp_iside_priv_bits_cross :
          cross cp_region_priv_bits, cp_req_type_iside, cp_priv_lvl_iside, pmp_iside_req_err
            iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_ichan_access &&
                 g_pmp_fcov_signals.fcov_pmp_region_ichan_priority[i_region]                 &&
                 csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {

            // Will never see a succesful exec access when execute is disallowed
            wildcard illegal_bins illegal_allow_exec =
              // Non-MML disallowance conditions
              (binsof(cp_region_priv_bits) intersect {5'b0?0??} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {0})
              with (cp_priv_lvl_iside == PRIV_LVL_U || (cp_region_priv_bits & 5'b01000));
            wildcard illegal_bins illegal_machine_allow_exec =
              // MML related disallowance for Machine Mode
              (binsof(cp_region_priv_bits) intersect {5'b1????} &&
               binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {0})
              with (!(cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRM, MML_XM}));
            wildcard illegal_bins illegal_user_allow_exec =
              // MML related disallowance for User Mode
              (binsof(cp_region_priv_bits) intersect {5'b1????} &&
               binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_U} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {0})
              with (!(cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRU, MML_XU,
                                                  MML_XWRU}));

            // Will never see an exec access denied when executed is allowed
            wildcard illegal_bins illegal_deny_exec =
              // Non-MML allowance conditions
              (binsof(cp_region_priv_bits) intersect {5'b0?1??} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {1});
            wildcard illegal_bins illegal_machine_deny_exec =
              //MML related allowance for Machine Mode
              (binsof(cp_region_priv_bits) intersect {5'b1????} &&
               binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_M} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {1})
              with (cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRM, MML_XM});
            wildcard illegal_bins illegal_user_deny_exec =
              //MML related allowance for User Mode
              (binsof(cp_region_priv_bits) intersect {5'b1????} &&
               binsof(cp_priv_lvl_iside) intersect {PRIV_LVL_U} &&
               binsof(cp_req_type_iside) intersect {PMP_ACC_EXEC} &&
               binsof(pmp_iside_req_err) intersect {1})
              with (cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRU, MML_XU,
                                                MML_XWRU});
        }

        pmp_iside2_priv_bits_cross :
          cross cp_region_priv_bits, cp_req_type_iside2, cp_priv_lvl_iside2, pmp_iside2_req_err
            iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_ichan2_access &&
                 g_pmp_fcov_signals.fcov_pmp_region_ichan2_priority[i_region]                 &&
                 csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {

          // Will never see a succesful exec access when execute is disallowed
          wildcard illegal_bins illegal_allow_exec =
            // Non-MML disallowance conditions
            (binsof(cp_region_priv_bits) intersect {5'b0?0??} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {0})
            with (cp_priv_lvl_iside2 == PRIV_LVL_U || (cp_region_priv_bits & 5'b01000));
          wildcard illegal_bins illegal_machine_allow_exec =
            // MML related disallowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {0})
            with (!(cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRM, MML_XM}));
          wildcard illegal_bins illegal_user_allow_exec =
            // MML related disallowance for User Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {0})
            with (!(cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRU, MML_XU,
                                                MML_XWRU}));

          // Will never see an exec access denied when execute is allowed
            // Non-MML allowance conditions
          wildcard illegal_bins illegal_deny_exec =
            (binsof(cp_region_priv_bits) intersect {5'b0?1??} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {1});
          wildcard illegal_bins illegal_machine_deny_exec =
            //MML related allowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRM, MML_XM});
          wildcard illegal_bins illegal_user_deny_exec =
            //MML related allowance for User Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_iside2) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_iside2) intersect {PMP_ACC_EXEC} &&
             binsof(pmp_iside2_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_XM_XU, MML_XRM_XU, MML_XRU, MML_XU, MML_XWRU});
        }

        pmp_dside_priv_bits_cross :
          cross cp_region_priv_bits, cp_req_type_dside, cp_priv_lvl_dside, pmp_dside_req_err
            iff (g_pmp_fcov_signals.g_pmp_region_fcov[i_region].fcov_pmp_region_dchan_access &&
                 g_pmp_fcov_signals.fcov_pmp_region_dchan_priority[i_region]                 &&
                 csr_pmp_cfg[i_region].mode != PMP_MODE_OFF) {

          // Will never see a succesful read access when read is disallowed
          wildcard illegal_bins illegal_allow_read =
            // Non-MML disallowance conditions
            (binsof(cp_region_priv_bits) intersect {5'b0???0} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {0})
            with (cp_priv_lvl_dside == PRIV_LVL_U || (cp_region_priv_bits & 5'b01000));
          wildcard illegal_bins illegal_machine_allow_read =
            // MML related disallowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {0})
            with (!(cp_region_priv_bits inside {MML_WRM_RU, MML_WRM_WRU, MML_RM_RU, MML_RM,
                                                MML_WRM, MML_XRM, MML_XRM_XU}));
          wildcard illegal_bins illegal_user_allow_read =
            // MML related disallowance for User Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {0})
            with (!(cp_region_priv_bits inside {MML_WRM_RU, MML_WRM_WRU, MML_RM_RU, MML_RU,
                                                MML_WRU, MML_XRU, MML_XWRU}));

          // Will never see a read access denied when read is allowed
          wildcard illegal_bins illegal_deny_read =
            // Non-MML allowance conditions
            (binsof(cp_region_priv_bits) intersect {5'b0???1} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {1});
          wildcard illegal_bins illegal_machine_deny_read =
            //MML related allowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_WRM_RU, MML_WRM_WRU, MML_RM_RU, MML_RM, MML_WRM,
                                              MML_XRM, MML_XRM_XU});
          wildcard illegal_bins illegal_user_deny_read =
            //MML related allowance for User Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_READ} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_WRM_RU, MML_WRM_WRU, MML_RM_RU, MML_RU, MML_WRU,
                                              MML_XRU, MML_XWRU});

          // Will never see a succesful write access when write is disallowed
          wildcard illegal_bins illegal_allow_write =
            // Non-MML disallowance conditions
            (binsof(cp_region_priv_bits) intersect {5'b0??0?} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {0})
            with (cp_priv_lvl_dside == PRIV_LVL_U || (cp_region_priv_bits & 5'b01000));
          wildcard illegal_bins illegal_machine_allow_write =
            // MML related disallowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (!(cp_region_priv_bits inside {MML_WRM_WRU, MML_WRM_RU, MML_WRM}));
          wildcard illegal_bins illegal_user_allow_write =
            // MML related disallowance for User Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (!(cp_region_priv_bits inside {MML_WRM_WRU, MML_WRU, MML_XWRU}));

          // Will never see a write access denied when write is allowed
          wildcard illegal_bins illegal_deny_write =
            // Non-MML allowance conditions
            (binsof(cp_region_priv_bits) intersect {5'b0??1?} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {1});
          wildcard illegal_bins illegal_machine_deny_write =
            //MML related allowance for Machine Mode
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_M} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_WRM_WRU, MML_WRM_RU, MML_WRM});
            //MML related allowance for User Mode
          wildcard illegal_bins illegal_user_deny_write =
            (binsof(cp_region_priv_bits) intersect {5'b1????} &&
             binsof(cp_priv_lvl_dside) intersect {PRIV_LVL_U} &&
             binsof(cp_req_type_dside) intersect {PMP_ACC_WRITE} &&
             binsof(pmp_dside_req_err) intersect {1})
            with (cp_region_priv_bits inside {MML_WRM_WRU, MML_WRU, MML_XWRU});
        }

        rlb_csr_cross : cross cp_rlb, csr_wdata_mseccfg_rlb
          iff (fcov_csr_write && cs_registers_i.csr_addr_i == CSR_MSECCFG) {
            // Enabling RLB when RLB is disabled and locked regions present is illegal
            bins illegal_sticky =
              (binsof(csr_wdata_mseccfg_rlb) intersect {1} &&
               binsof(cp_rlb) intersect {0})
              iff (cs_registers_i.g_pmp_registers.any_pmp_entry_locked);
          }

        mmwp_csr_cross : cross cp_mmwp, csr_wdata_mseccfg_mmwp
          iff (fcov_csr_write && cs_registers_i.csr_addr_i == CSR_MSECCFG) {
            // Disabling MMWP when it is already enabled is illegal
            bins illegal_sticky =
              (binsof(csr_wdata_mseccfg_mmwp) intersect {0} &&
               binsof(cp_mmwp) intersect {1});
          }

        mml_sticky_cross : cross cp_mml, csr_wdata_mseccfg_mml
          iff (fcov_csr_write && cs_registers_i.csr_addr_i == CSR_MSECCFG) {
            // Disabling MML when it is already enabled is illegal
            bins illegal_sticky =
              (binsof(csr_wdata_mseccfg_mml) intersect {0} &&
               binsof(cp_mml) intersect {1});
          }

        `DV_FCOV_EXPR_SEEN(edit_locked_pmpcfg,
          fcov_csr_write &&
          csr_pmp_cfg[i_region].lock &&
          cs_registers_i.g_pmp_registers.g_pmp_csrs[i_region].u_pmp_cfg_csr.wr_en_i &&
          csr_pmp_mseccfg.rlb)

        `DV_FCOV_EXPR_SEEN(edit_locked_pmpaddr,
          fcov_csr_write &&
          csr_pmp_cfg[i_region].lock &&
          cs_registers_i.g_pmp_registers.g_pmp_csrs[i_region].u_pmp_addr_csr.wr_en_i &&
          csr_pmp_mseccfg.rlb)
      endgroup

      `DV_FCOV_INSTANTIATE_CG(pmp_region_cg, en_pmp_fcov)
    end

    covergroup pmp_top_cg @(posedge clk_i);
      option.per_instance = 1;
      option.name = "pmp_top_cg";
      cp_pmp_iside_region_override :
        coverpoint g_pmp.pmp_i.g_access_check[PMP_I].fcov_pmp_region_override
          iff (if_stage_i.if_id_pipe_reg_we);

      cp_pmp_iside2_region_override :
        coverpoint g_pmp.pmp_i.g_access_check[PMP_I2].fcov_pmp_region_override
          iff (if_stage_i.if_id_pipe_reg_we);

      cp_pmp_dside_region_override :
        coverpoint g_pmp.pmp_i.g_access_check[PMP_D].fcov_pmp_region_override iff (data_req_out);

      pmp_instr_edge_cross: cross if_stage_i.instr_is_compressed_id_o,
                                  pmp_iside_req_err, pmp_iside2_req_err;

      misaligned_lsu_access_cross: cross load_store_unit_i.fcov_ls_mis_pmp_err_1,
                                         load_store_unit_i.fcov_ls_mis_pmp_err_2;

    endgroup

    `DV_FCOV_INSTANTIATE_CG(pmp_top_cg, en_pmp_fcov)
  end

endinterface

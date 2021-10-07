// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
module ic_top import ibex_pkg::*; #(
  parameter bit ICacheECC = 1'b0,
  parameter bit Scramble = 1'b0
) (
    input  logic                           clk_i,
    input  logic                           rst_ni,
    input  logic                           req_i,
    input  logic                           branch_i,
    input  logic                           branch_spec_i,
    input  logic                           branch_mispredict_i,
    input  logic [31:0]                    mispredict_addr_i,
    input  logic [31:0]                    addr_i,
    input  logic                           ready_i,
    output logic                           valid_o,
    output logic [31:0]                    rdata_o,
    output logic [31:0]                    addr_o,
    output logic                           err_o,
    output logic                           err_plus2_o,
    output logic                           instr_req_o,
    input  logic                           instr_gnt_i,
    output logic [31:0]                    instr_addr_o,
    input  logic [BUS_SIZE-1:0]            instr_rdata_i,
    input  logic                           instr_err_i,
    input  logic                           instr_pmp_err_i,
    input  logic                           instr_rvalid_i,

    // Scrambling Interface
    input  logic                           scramble_ack_i,
    input  logic [SCRAMBLE_KEY_W-1:0]      scramble_key_i,
    input  logic [SCRAMBLE_NONCE_W-1:0]    scramble_nonce_i,

    input  logic                           icache_enable_i,
    input  logic                           icache_inval_i,
    output logic                           busy_o
);

  localparam int unsigned BusSizeECC  = ICacheECC ? (BUS_SIZE + 7) : BUS_SIZE;
  localparam int unsigned LineSizeECC = BusSizeECC * IC_LINE_BEATS;
  localparam int unsigned TagSizeECC  = ICacheECC ? (IC_TAG_SIZE + 6) : IC_TAG_SIZE;
  localparam int unsigned NumAddrScrRounds  = Scramble ? 2 : 0;
  localparam int unsigned DataNonceWidth    = ICacheECC ? 128 : 64;
  // RAM IO
  logic [IC_NUM_WAYS-1:0]         ic_tag_req;
  logic                           ic_tag_write;
  logic [IC_INDEX_W-1:0]          ic_tag_addr;
  logic [TagSizeECC-1:0]          ic_tag_wdata;
  logic [TagSizeECC-1:0]          ic_tag_rdata [IC_NUM_WAYS];
  logic [IC_NUM_WAYS-1:0]         ic_data_req;
  logic                           ic_data_write;
  logic [IC_INDEX_W-1:0]          ic_data_addr;
  logic [LineSizeECC-1:0]         ic_data_wdata;
  logic [LineSizeECC-1:0]         ic_data_rdata [IC_NUM_WAYS];
  // Scramble signals
  logic                        icache_inval;
  logic [SCRAMBLE_KEY_W-1:0]   scramble_key;
  logic [SCRAMBLE_NONCE_W-1:0] scramble_nonce;
  logic [127:0]                scramble_nonce_buf;
  logic scramble_ack;

  // DUT
  ibex_icache #(
      .ICacheECC       (ICacheECC),
      .BusSizeECC      (BusSizeECC),
      .TagSizeECC      (TagSizeECC),
      .LineSizeECC     (LineSizeECC)
  ) icache_i (
      .clk_i               ( clk_i                      ),
      .rst_ni              ( rst_ni                     ),

      .req_i               ( req_i                      ),

      .branch_i            ( branch_i                   ),
      .branch_spec_i       ( branch_spec_i              ),
      .branch_mispredict_i ( branch_mispredict_i        ),
      .mispredict_addr_i   ( mispredict_addr_i          ),
      .addr_i              ( addr_i                     ),

      .ready_i             ( ready_i                    ),
      .valid_o             ( valid_o                    ),
      .rdata_o             ( rdata_o                    ),
      .addr_o              ( addr_o                     ),
      .err_o               ( err_o                      ),
      .err_plus2_o         ( err_plus2_o                ),

      .instr_req_o         ( instr_req_o                ),
      .instr_addr_o        ( instr_addr_o               ),
      .instr_gnt_i         ( instr_gnt_i                ),
      .instr_rvalid_i      ( instr_rvalid_i             ),
      .instr_rdata_i       ( instr_rdata_i              ),
      .instr_err_i         ( instr_err_i                ),
      .instr_pmp_err_i     ( instr_pmp_err_i            ),

      .ic_tag_req_o        ( ic_tag_req                 ),
      .ic_tag_write_o      ( ic_tag_write               ),
      .ic_tag_addr_o       ( ic_tag_addr                ),
      .ic_tag_wdata_o      ( ic_tag_wdata               ),
      .ic_tag_rdata_i      ( ic_tag_rdata               ),
      .ic_data_req_o       ( ic_data_req                ),
      .ic_data_write_o     ( ic_data_write              ),
      .ic_data_addr_o      ( ic_data_addr               ),
      .ic_data_wdata_o     ( ic_data_wdata              ),
      .ic_data_rdata_i     ( ic_data_rdata              ),
      .ic_scr_key_valid_i  ( scramble_ack               ),

      .icache_enable_i     ( icache_enable_i            ),
      .icache_inval_i      ( icache_inval_i             ),
      .busy_o              ( busy_o                     )
  );

  ///////////////////////////////
  // Scrambling Infrastructure //
  ///////////////////////////////

  if (Scramble) begin : gen_scramble
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        scramble_key   <= 128'h14e8cecae3040d5e12286bb3cc113298;
        scramble_nonce <= 64'hf79780bc735f3843;
        scramble_nonce_buf <= {64'hf79780bc735f3843, 64'hf79780bc735f3843};
        scramble_ack   <= 1'b1;
      end else if (scramble_ack_i) begin
        scramble_key   <= scramble_key_i;
        scramble_nonce <= scramble_nonce_i;
        scramble_nonce_buf <= {scramble_nonce, scramble_nonce_i};
        scramble_ack   <= 1'b1;
      end
      else begin
        scramble_ack <= 1'b0;
      end
    end

    logic unused_scramble = (|scramble_nonce_buf);

  end else begin : gen_noscramble

    logic unused_scramble_inputs = scramble_ack_i & (|scramble_key_i) &
                                   (|scramble_nonce_i) & icache_inval & (|scramble_nonce_buf);

    assign scramble_key = '0;
    assign scramble_nonce = '0;
    assign scramble_nonce_buf = '0;
    assign scramble_ack = '0;
  end

  // RAMs
    for (genvar way = 0; way < IC_NUM_WAYS; way++) begin : gen_rams
      // Tag RAM instantiation
      prim_ram_1p_scr #(
        .Width            (TagSizeECC),
        .Depth            (IC_NUM_LINES),
        .DataBitsPerMask  (TagSizeECC),
        .EnableParity     (0),
        .DiffWidth        (TagSizeECC),
        .NumAddrScrRounds (NumAddrScrRounds)
      ) tag_bank (
        .clk_i,
        .rst_ni,

        .key_valid_i (scramble_ack),
        .key_i       (scramble_key),
        .nonce_i     (scramble_nonce),

        .req_i       (ic_tag_req[way]),

        .gnt_o       (),
        .write_i     (ic_tag_write),
        .addr_i      (ic_tag_addr),
        .wdata_i     (ic_tag_wdata),
        .wmask_i     ({TagSizeECC{1'b1}}),
        .intg_error_i(1'b0),

        .rdata_o     (ic_tag_rdata[way]),
        .rvalid_o    (),
        .raddr_o     (),
        .rerror_o    (),
        .cfg_i       ('0)
      );

      // Data RAM instantiation
      prim_ram_1p_scr #(
        .Width            (LineSizeECC),
        .Depth            (IC_NUM_LINES),
        .DataBitsPerMask  (LineSizeECC),
        .EnableParity     (0),
        .DiffWidth        (LineSizeECC),
        .NumAddrScrRounds (NumAddrScrRounds)
      ) data_bank (
        .clk_i,
        .rst_ni,

        .key_valid_i (scramble_ack),
        .key_i       (scramble_key),
        .nonce_i     (scramble_nonce_buf[DataNonceWidth-1:0]),

        .req_i       (ic_data_req[way]),

        .gnt_o       (),
        .write_i     (ic_data_write),
        .addr_i      (ic_data_addr),
        .wdata_i     (ic_data_wdata),
        .wmask_i     ({LineSizeECC{1'b1}}),
        .intg_error_i(1'b0),

        .rdata_o     (ic_data_rdata[way]),
        .rvalid_o    (),
        .raddr_o     (),
        .rerror_o    (),
        .cfg_i       ('0)
      );
    end


endmodule

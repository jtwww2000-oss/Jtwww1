`timescale 1ns / 1ps
module Rnd #(
    // Keccak-f[1600] 核心参数 (固定)
    localparam STATE_WIDTH = 1600
)(
    input  [STATE_WIDTH-1:0] A_in_flat,
    input  [4:0]    i_round_index, // 0-23
    output [STATE_WIDTH-1:0] Ap_out_flat
);
    // --- 中间连线 (Wires) ---
    wire [STATE_WIDTH-1:0] w_after_theta;
    wire [STATE_WIDTH-1:0] w_after_rho;
    wire [STATE_WIDTH-1:0] w_after_pi;
    wire [STATE_WIDTH-1:0] w_after_chi;

    // Ap = iota(chi(pi(rho(theta(A)))), ir)
    
    theta u_theta (
        .A_in_flat   (A_in_flat),
        .Ap_out_flat (w_after_theta)
    );

    rho u_rho (
        .A_in_flat   (w_after_theta),
        .Ap_out_flat (w_after_rho)
    );

    pi u_pi (
        .A_in_flat   (w_after_rho),
        .Ap_out_flat (w_after_pi)
    );

    chi u_chi (
        .A_in_flat   (w_after_pi),
        .Ap_out_flat (w_after_chi)
    );

    iota u_iota (
        .A_in_flat      (w_after_chi),
        .i_round_index  (i_round_index),
        .Ap_out_flat    (Ap_out_flat)
    );
    
endmodule
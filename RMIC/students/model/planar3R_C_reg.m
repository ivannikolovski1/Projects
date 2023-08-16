function C_reg = planar3R_C_reg(q, dq, params)

  Cq_regmin = planar3R_C_reg_vp(q, dq, params.pkin(1:2));
  Cq_mpv_vec = Cq_regmin*params.beta_b;
  C_reg = reshape(Cq_mpv_vec, 3, 3)';
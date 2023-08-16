function M_reg = planar3R_M_reg(q, params)

Mq_regmin = planar3R_M_reg_vp(q, params.pkin(1:2));
Mq_mpv_vec = Mq_regmin*params.beta_b;
M_reg = vec2symmat(Mq_mpv_vec);


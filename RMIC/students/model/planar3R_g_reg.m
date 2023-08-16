function taug_reg = planar3R_g_reg(q, params)

taug_regmin = planar3R_g_reg_vp(q, params.g_base, ...
    params.pkin(1:2));
taug_reg = taug_regmin*params.beta_b;


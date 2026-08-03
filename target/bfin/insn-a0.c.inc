/*
 * Rotate through CC. The rotation is 33 bits wide - CC sits above bit 31 -
 * so a rotate by one moves bit 31 into CC and CC into bit 0. A negative
 * count rotates right. See "Rotate" in the programming reference.
 */
static bool trans_rot_imm(DisasContext *ctx, arg_rot_imm *a)
{
    gen_helper_rot(dreg(a->dst), tcg_env, dreg(a->src),
                   tcg_constant_i32(a->imm));
    return true;
}

/*
 * Extract a bit field from the scene register. The field is described by the
 * low half of the pattern register - position in bits 12-8, length in bits
 * 4-0 - so it is not known at translation time. Bits past the top of the
 * scene register read as zero, which the helper gets for free by shifting a
 * 32-bit value.
 */
static bool trans_extract_z(DisasContext *ctx, arg_extract_z *a)
{
    gen_helper_bitextract(dreg(a->dst), dreg(a->scene), dreg(a->pat),
                          tcg_constant_i32(0));
    return true;
}

static bool trans_extract_x(DisasContext *ctx, arg_extract_x *a)
{
    gen_helper_bitextract(dreg(a->dst), dreg(a->scene), dreg(a->pat),
                          tcg_constant_i32(1));
    return true;
}

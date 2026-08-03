/* SPDX-License-Identifier: GPL-2.0-or-later */
DEF_HELPER_3(raise_exception, noreturn, env, i32, i32)
DEF_HELPER_1(idle, void, env)
DEF_HELPER_1(rti, void, env)
DEF_HELPER_1(cli, i32, env)
DEF_HELPER_2(sti, void, env, i32)
DEF_HELPER_2(raise_ivg, void, env, i32)
DEF_HELPER_2(read_creg, i32, env, i32)
DEF_HELPER_3(write_creg, void, env, i32, i32)

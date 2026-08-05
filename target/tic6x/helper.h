/* SPDX-License-Identifier: GPL-2.0-or-later */
DEF_HELPER_3(unimplemented, noreturn, env, i32, i32)
DEF_HELPER_2(illegal, noreturn, env, i32)
DEF_HELPER_2(read_creg, i32, env, i32)
DEF_HELPER_3(write_creg, void, env, i32, i32)

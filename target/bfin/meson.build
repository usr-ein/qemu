# SPDX-License-Identifier: GPL-2.0-or-later
gen = [
  decodetree.process('insn16.decode',
                     extra_args: ['--static-decode=decode_insn16',
                                  '--insnwidth=16']),
  decodetree.process('insn32.decode',
                     extra_args: '--static-decode=decode_insn32'),
]

bfin_ss = ss.source_set()
bfin_ss.add(gen)
bfin_ss.add(files(
  'translate.c',
  'helper.c',
  'cpu.c'))

bfin_common_system_ss = ss.source_set()
bfin_common_system_ss.add(files('gdbstub.c'))

target_arch += {'bfin': bfin_ss}
target_common_system_arch += {'bfin': bfin_common_system_ss}

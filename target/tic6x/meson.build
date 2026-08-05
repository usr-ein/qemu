# SPDX-License-Identifier: GPL-2.0-or-later
tic6x_ss = ss.source_set()
tic6x_ss.add(files(
  'translate.c',
  'helper.c',
  'cpu.c'))

tic6x_common_system_ss = ss.source_set()
tic6x_common_system_ss.add(files('gdbstub.c'))

target_arch += {'tic6x': tic6x_ss}
target_common_system_arch += {'tic6x': tic6x_common_system_ss}

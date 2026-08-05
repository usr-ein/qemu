# SPDX-License-Identifier: GPL-2.0-or-later
tic6x_ss = ss.source_set()
tic6x_ss.add(when: 'CONFIG_CDJ2KNXS_DSP', if_true: files('cdj2knxs_dsp.c'))

hw_arch += {'tic6x': tic6x_ss}

# SPDX-License-Identifier: GPL-2.0-or-later
bfin_ss = ss.source_set()
bfin_ss.add(when: 'CONFIG_BF531', if_true: files('bf531.c'))
bfin_ss.add(when: 'CONFIG_CDJ2KNXS_GUI', if_true: files('cdj2knxs_gui.c'))

hw_arch += {'bfin': bfin_ss}

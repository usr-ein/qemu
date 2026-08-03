/*
 * Analog Devices Blackfin CPU QOM header (target agnostic)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef BFIN_CPU_QOM_H
#define BFIN_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_BFIN_CPU "bfin-cpu"

#define BFIN_CPU_TYPE_SUFFIX "-" TYPE_BFIN_CPU
#define BFIN_CPU_TYPE_NAME(model) model BFIN_CPU_TYPE_SUFFIX

#define TYPE_BF531_CPU BFIN_CPU_TYPE_NAME("bf531")

OBJECT_DECLARE_CPU_TYPE(BfinCPU, BfinCPUClass, BFIN_CPU)

#endif

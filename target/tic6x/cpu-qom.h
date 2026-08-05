/*
 * TMS320C674x CPU QOM header (target agnostic)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TIC6X_CPU_QOM_H
#define TIC6X_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_TIC6X_CPU "tic6x-cpu"

#define TIC6X_CPU_TYPE_SUFFIX "-" TYPE_TIC6X_CPU
#define TIC6X_CPU_TYPE_NAME(model) model TIC6X_CPU_TYPE_SUFFIX

/*
 * The part in the player is a D810K013CZKB400, which is a TMS320C6745 or
 * C6747 - same die, and the register map, ball map and pin mux table in
 * SPRS377 all match. Name the CPU for the catalogue part, because that is
 * what the documentation calls it.
 */
#define TYPE_C6747_CPU TIC6X_CPU_TYPE_NAME("c6747")

OBJECT_DECLARE_CPU_TYPE(TIC6XCPU, TIC6XCPUClass, TIC6X_CPU)

#endif

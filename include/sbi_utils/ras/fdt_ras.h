/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#ifndef __FDT_RAS_H__
#define __FDT_RAS_H__

#include <sbi/sbi_types.h>
#include <sbi_utils/fdt/fdt_driver.h>

#ifdef CONFIG_FDT_RAS

void fdt_ras_init(const void *fdt);

#else

static inline void fdt_ras_init(const void *fdt) { }

#endif

#endif

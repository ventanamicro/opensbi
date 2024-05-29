/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems, Inc.
 *
 * Author(s):
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#ifndef __RERI_DRV_H
#define __RERI_DRV_H

#include <sbi_utils/fdt/fdt_helper.h>

int reri_drv_init(const void *fdt, int nodeoff, const struct fdt_match *match);
int reri_drv_sync_hart_errs(u32 hart_id, u32 *pending_vectors);

#endif

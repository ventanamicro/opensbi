/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#ifndef __FDT_MPXY_H__
#define __FDT_MPXY_H__

#include <sbi/sbi_types.h>

#ifdef CONFIG_FDT_MPXY

struct fdt_mpxy {
	const struct fdt_match *match_table;
	int (*init)(void *fdt, int nodeoff, const struct fdt_match *match);
	void (*exit)(void);
};

int fdt_mpxy_init(void);

#else

static inline int fdt_mpxy_init(void) { return 0; }

#endif

#endif

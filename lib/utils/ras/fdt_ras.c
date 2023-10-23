/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems, Inc.
 *
 * Author(s):
 *   Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#include <sbi_utils/ras/fdt_ras.h>

/* List of FDT RAS drivers generated at compile time */
extern const struct fdt_driver *const fdt_ras_drivers[];

void fdt_ras_init(const void *fdt)
{
	/*
	 * Platforms might have multiple RAS devices or might not
	 * have any RAS devices so don't fail.
	 */
	fdt_driver_init_all(fdt, fdt_ras_drivers);
}

/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#include <sbi/sbi_error.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>

/* List of FDT MPXY drivers generated at compile time */
extern struct fdt_mpxy *fdt_mpxy_drivers[];
extern unsigned long fdt_mpxy_drivers_size;

int fdt_mpxy_init(void)
{
	int pos, noff, rc;
	struct fdt_mpxy *drv;
	const struct fdt_match *match;
	void *fdt = fdt_get_address();

	for (pos = 0; pos < fdt_mpxy_drivers_size; pos++) {
		drv = fdt_mpxy_drivers[pos];

		noff = -1;
		while ((noff = fdt_find_match(fdt, noff,
					drv->match_table, &match)) >= 0) {
			/* drv->init must not be NULL */
			if (drv->init == NULL)
				return SBI_EFAIL;

			rc = drv->init(fdt, noff, match);
			if (rc == SBI_ENODEV)
				continue;
			if (rc)
				return rc;

			/*
			 * We will have multiple MPXY devices so we
			 * cannot break here.
			 */
		}
	}

	/* Platforms might not have any MPXY devices so don't fail */
	return 0;
}

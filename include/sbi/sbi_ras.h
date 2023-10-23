/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Ventana Micro Systems, Inc.
 *
 * Author(s):
 *    Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#ifndef __SBI_RAS_H__
#define __SBI_RAS_H__

#include <sbi/sbi_types.h>

/** RAS Agent */
struct sbi_ras_agent {
	/** Name of the RAS agent */
	char name[32];

	/** probe - returns register width if implemented, 0 otherwise */
	int (*ras_probe)(void);

	/** synchronize CPU errors */
	int (*ras_sync_hart_errs)(u32 *pending_vectors, u32 *nr_pending,
				  u32 *nr_remaining);

	/** synchronize device errors */
	int (*ras_sync_dev_errs)(u32 *pending_vectors, u32 *nr_pending,
				 u32 *nr_remaining);
};

int sbi_ras_probe(void);
int sbi_ras_sync_hart_errs(u32 *pending_vectors, u32 *nr_pending,
			   u32 *nr_remaining);
int sbi_ras_sync_dev_errs(u32 *pending_vectors, u32 *nr_pending,
			  u32 *nr_remaining);

const struct sbi_ras_agent *sbi_ras_get_agent(void);
void sbi_ras_set_agent(const struct sbi_ras_agent *agent);

#endif

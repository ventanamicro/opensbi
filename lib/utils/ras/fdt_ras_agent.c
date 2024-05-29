/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems, Inc.
 *
 * Author(s):
 *   Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_ras.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_console.h>
#include <sbi_utils/ras/reri_drv.h>
#include <sbi_utils/fdt/fdt_driver.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mailbox/fdt_mailbox.h>
#include <sbi_utils/mailbox/rpmi_mailbox.h>

static int ra_init_done = 0;

static int sbi_ras_agent_sync_hart_errs(u32 *pending_vectors, u32 *nr_pending,
					u32 *nr_remaining)
{
	u32 hart_id = current_hartid();
	u32 num_errs;

	if (!ra_init_done)
		return -1;

	if ((num_errs = reri_drv_sync_hart_errs(hart_id, pending_vectors)) == 0)
		return SBI_EFAIL;

	*nr_pending = num_errs;
	*nr_remaining = 0;

	return 0;
}

static int sbi_ras_agent_sync_dev_errs(u32 *pending_vectors, u32 *nr_pending,
				       u32 *nr_remaining)
{
	return SBI_SUCCESS;
}

static int sbi_ras_agent_probe(void)
{
	return 0;
}

static struct sbi_ras_agent sbi_ras_agent = {
	.name			= "sbi-ras-agent",
	.ras_sync_hart_errs	= sbi_ras_agent_sync_hart_errs,
	.ras_sync_dev_errs	= sbi_ras_agent_sync_dev_errs,
	.ras_probe		= sbi_ras_agent_probe,
};

static int sbi_ras_agent_cold_init(const void *fdt, int nodeoff,
				   const struct fdt_match *match)
{
	int ret;

	/* initialize reri driver */
	ret = reri_drv_init(fdt, nodeoff, match);
	if (ret)
		return ret;

	/* ready to handle errors */
	sbi_ras_set_agent(&sbi_ras_agent);

	ra_init_done = 1;

	return 0;
}

static const struct fdt_match sbi_ras_agent_match[] = {
	{ .compatible = "riscv,sbi-ras-agent" },
	{},
};

const struct fdt_driver fdt_sbi_ras_agent = {
	.match_table = sbi_ras_agent_match,
	.init = sbi_ras_agent_cold_init,
};

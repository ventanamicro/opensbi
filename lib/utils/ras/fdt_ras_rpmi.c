/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Ventana Micro Systems, Inc.
 *
 * Author(s):
 *   Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#include <libfdt.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_ras.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_console.h>
#include <sbi_utils/fdt/fdt_driver.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mailbox/fdt_mailbox.h>
#include <sbi_utils/mailbox/rpmi_mailbox.h>

struct rpmi_ras {
	struct mbox_chan *chan;
};

static struct rpmi_ras ras;

static int rpmi_ras_sync_hart_errs(u32 *pending_vectors, u32 *nr_pending,
				   u32 *nr_remaining)
{
	int rc = SBI_SUCCESS;
	struct rpmi_ras_sync_hart_err_req req;
	struct rpmi_ras_sync_err_resp resp;

	if (!pending_vectors || !nr_pending || !nr_remaining)
		return SBI_ERR_INVALID_PARAM;

	*nr_pending = *nr_remaining = 0;

	if (!ras.chan)
		return SBI_ERR_INVALID_STATE;

	req.hart_id = current_hartid();

	rc = rpmi_normal_request_with_status(ras.chan,
					     RPMI_RAS_SRV_SYNC_HART_ERR_REQ,
					     &req, rpmi_u32_count(req),
					     rpmi_u32_count(req),
					     &resp, rpmi_u32_count(resp),
					     rpmi_u32_count(resp));

	if (rc) {
		sbi_printf("%s: sync failed, rc: 0x%x\n", __func__, rc);
		return rc;
	}

	if (!resp.status && resp.returned > 0 && resp.returned < MAX_PEND_VECS) {
		memcpy(pending_vectors, resp.pending_vecs,
		       resp.returned * sizeof(u32));
		*nr_pending = resp.returned;
		*nr_remaining = resp.remaining;
	} else {
		if (resp.status) {
			sbi_printf("%s: sync returned status %d\n",
				   __func__, resp.status);
		}

		if (resp.returned < 0 || resp.returned > MAX_PEND_VECS)
			sbi_printf("%s: invalid vector range returned %u\n",
				   __func__, resp.returned);

		return SBI_ERR_FAILED;
	}

	return SBI_SUCCESS;
}

static int rpmi_ras_sync_dev_errs(u32 *pending_vectors, u32 *nr_pending,
				  u32 *nr_remaining)
{
	int rc = SBI_SUCCESS;

	return rc;
}

static int rpmi_ras_probe(void)
{
	int rc;
	struct rpmi_ras_probe_resp resp;
	struct rpmi_ras_probe_req req;

	if (!ras.chan)
		return SBI_ERR_INVALID_STATE;

	rc = rpmi_normal_request_with_status(
			ras.chan, RPMI_RAS_SRV_PROBE_REQ,
			&req, rpmi_u32_count(req), rpmi_u32_count(req),
			&resp, rpmi_u32_count(resp), rpmi_u32_count(resp));
	if (rc)
		return rc;

	return 0;
}

static struct sbi_ras_agent sbi_rpmi_ras_agent = {
	.name			= "rpmi-ras-agent",
	.ras_sync_hart_errs	= rpmi_ras_sync_hart_errs,
	.ras_sync_dev_errs	= rpmi_ras_sync_dev_errs,
	.ras_probe		= rpmi_ras_probe,
};

static int rpmi_ras_cold_init(const void *fdt, int nodeoff,
			      const struct fdt_match *match)
{
	int rc;

	if (ras.chan)
		return 0;

	/*
	 * If channel request failed then other end does not support
	 * RAS service group so do nothing.
	 */
	rc = fdt_mailbox_request_chan(fdt, nodeoff, 0, &ras.chan);
	if (rc)
		return rc;

	sbi_ras_set_agent(&sbi_rpmi_ras_agent);

	sbi_ras_probe();

	return 0;
}

static const struct fdt_match rpmi_ras_match[] = {
	{ .compatible = "riscv,rpmi-ras" },
	{},
};

const struct fdt_driver fdt_ras_rpmi = {
	.match_table = rpmi_ras_match,
	.init = rpmi_ras_cold_init,
};

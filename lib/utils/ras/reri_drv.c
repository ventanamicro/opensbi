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
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/ras/riscv_reri_regs.h>
#include <sbi_utils/ras/apei_tables.h>
#include <sbi_utils/ras/ghes.h>
#include <sbi_utils/ras/reri_drv.h>

union reri_device {
	struct reri_device_dram {
		uint64_t addr;
		uint64_t size;
		uint32_t sse_vector;
		uint16_t src_id;
	} dram;

	struct reri_device_hart {
		uint64_t addr;
		uint64_t size;
		uint32_t sse_vector; /* Start of sse vectors */
		uint32_t max_harts;
		uint16_t src_id;
	} harts;
};

#define RERI_DEV_DRAM		0
#define RERI_DEV_HART		1
#define MAX_RERI_DEVICES	2

static union reri_device reri_devices[MAX_RERI_DEVICES];

#define RERI_HART_COMPAT	"riscv,reri-harts"
#define RERI_DRAM_COMPAT	"riscv,reri-dram"
#define APEI_MEM_COMPAT		"riscv,apei-mem"

static uint64_t riscv_reri_dev_read_u64(void *dev_addr)
{
	return *((volatile uint64_t *)dev_addr);
}

static void riscv_reri_dev_write_u64(void *dev_addr, uint64_t value)
{
	*((volatile uint64_t *)dev_addr) = value;
}

static void riscv_reri_clear_valid_bit(void *control_addr)
{
	uint64_t control;

	control = riscv_reri_dev_read_u64(control_addr);

	/* set SINV */
	control |= 0x4;

	riscv_reri_dev_write_u64(control_addr, control);
}

static int riscv_reri_get_hart_addr(int hart_id, uint64_t *hart_addr,
				    uint64_t *size)
{
	uint64_t baddr, sz;

	baddr = reri_devices[RERI_DEV_HART].harts.addr;
	sz = reri_devices[RERI_DEV_HART].harts.size;
	baddr = hart_id * sz + baddr;
	*hart_addr = baddr;

	return 0;
}

static uint32_t riscv_reri_get_hart_sse_vector(int hart_id)
{
	uint32_t start_vec;

	if (hart_id > 0)
		return -1;

	start_vec = reri_devices[RERI_DEV_HART].harts.sse_vector;

	return start_vec + hart_id;
}

static int fdt_parse_reri_device(const void *fdt, int nodeoff)
{
	int ret = SBI_SUCCESS;
	uint64_t addr, size;
	const fdt32_t *sse_vec_p, *max_harts_p, *src_id_p;
	uint32_t sse_vec, max_harts;
	uint16_t src_id;
	int len, i;

	if ((ret = fdt_node_check_compatible(fdt, nodeoff,
					     RERI_DRAM_COMPAT)) == 0) {
		if ((ret = fdt_get_node_addr_size(fdt, nodeoff, 0, &addr,
						  &size)) == 0) {
			reri_devices[RERI_DEV_DRAM].dram.addr = addr;
			reri_devices[RERI_DEV_DRAM].dram.size = size;
		} else {
			goto _err_out;
		}

		sse_vec_p = fdt_getprop(fdt, nodeoff, "sse-vector", &len);
		if (!sse_vec_p)
			return SBI_ENOENT;

		sse_vec = fdt32_to_cpu(*sse_vec_p);
		reri_devices[RERI_DEV_DRAM].dram.sse_vector = sse_vec;

		src_id_p = fdt_getprop(fdt, nodeoff, "source-id", &len);
		if (!src_id_p)
			return SBI_ENOENT;

		src_id = fdt32_to_cpu(*src_id_p);
		reri_devices[RERI_DEV_DRAM].dram.src_id = src_id;
		if ((ret = acpi_ghes_new_error_source(src_id, sse_vec)) < 0) {
			sbi_printf("Failed to create new DRAM error source\n");
			return ret;
		}
	} else if ((ret = fdt_node_check_compatible(fdt, nodeoff,
						    RERI_HART_COMPAT)) == 0) {
		if ((ret = fdt_get_node_addr_size(fdt, nodeoff, 0, &addr,
						  &size)) == 0) {
			reri_devices[RERI_DEV_HART].harts.addr = addr;
			reri_devices[RERI_DEV_HART].harts.size = size;
		} else {
			goto _err_out;
		}

		sse_vec_p = fdt_getprop(fdt, nodeoff, "sse-vector", &len);
		if (!sse_vec_p)
			return SBI_ENOENT;

		sse_vec = fdt32_to_cpu(*sse_vec_p);
		reri_devices[RERI_DEV_HART].harts.sse_vector = sse_vec;

		max_harts_p = fdt_getprop(fdt, nodeoff, "max-harts", &len);
		if (!max_harts_p)
			return SBI_ENOENT;

		max_harts = fdt32_to_cpu(*max_harts_p);
		reri_devices[RERI_DEV_HART].harts.max_harts = max_harts;

		src_id_p = fdt_getprop(fdt, nodeoff, "source-id", &len);
		if (!src_id_p)
			return SBI_ENOENT;

		src_id = fdt32_to_cpu(*src_id_p);
		reri_devices[RERI_DEV_HART].harts.src_id = src_id;
		for (i = 0; i < max_harts; i++) {
			if ((ret = acpi_ghes_new_error_source(src_id + i, sse_vec+i)) < 0)
				return ret;
		}
	}

_err_out:
	return ret;
}

int reri_drv_init(const void *fdt, int nodeoff, const struct fdt_match *match)
{
	int ret, doffset, moffset, len;
	uint64_t addr, size;
	const fdt32_t *rm_handle_p;
	uint32_t rm_handle;
	struct sbi_domain_memregion reg;

	rm_handle_p = fdt_getprop(fdt, nodeoff, "reserved-memory-handle", &len);
	if (!rm_handle_p)
		return SBI_ENOENT;

	rm_handle = fdt32_to_cpu(*rm_handle_p);
	moffset = fdt_node_offset_by_phandle(fdt, rm_handle);
	if (moffset < 0)
		return SBI_ENOENT;

	if ((ret = fdt_get_node_addr_size(fdt, moffset, 0, &addr,
					  &size)) == 0) {
		/* HACK: why size is zero? */
		if (size == 0)
			size = 0x80000;

		sbi_domain_memregion_init(addr, size,
					  SBI_DOMAIN_MEMREGION_SHARED_SURW_MRW,
					  &reg);
		ret = sbi_domain_root_add_memregion(&reg);
		if (ret)
			return ret;

		acpi_ghes_init(addr, size);
	}

	fdt_for_each_subnode(doffset, fdt, nodeoff) {
		if (fdt_parse_reri_device(fdt, doffset) != 0)
			continue;
	}

	return SBI_SUCCESS;
}

int reri_drv_sync_hart_errs(u32 hart_id, u32 *pending_vectors)
{
	riscv_reri_error_bank *heb;
	riscv_reri_status status;
	uint64_t hart_addr, err_size;
	uint64_t eaddr;
	acpi_ghes_error_info einfo;

	if (riscv_reri_get_hart_addr(hart_id, &hart_addr, &err_size) != 0)
		return 0;

	heb = (riscv_reri_error_bank *)(ulong)hart_addr;

	status.value = riscv_reri_dev_read_u64(&heb->records[0].status_i.value);

	eaddr = riscv_reri_dev_read_u64(&heb->records[0].addr_i);

	/* Error is valid process it */
	if (status.v == 1) {
		riscv_reri_clear_valid_bit(&heb->records[0].control_i.value);
		if (status.ce)
			einfo.info.gpe.sev = 2;
		else if (status.de)
			einfo.info.gpe.sev = 0; /* deferred, recoverable? */
		else if (status.ue)
			einfo.info.gpe.sev = 1; /* fatal error */
		else
			einfo.info.gpe.sev = 3; /* Unknown */

		einfo.info.gpe.validation_bits = (GPE_PROC_TYPE_VALID |
						  GPE_PROC_ISA_VALID |
						  GPE_PROC_ERR_TYPE_VALID);

		einfo.info.gpe.proc_type = GHES_PROC_TYPE_RISCV;
		einfo.info.gpe.proc_isa = GHES_PROC_ISA_RISCV64;

		if (status.tt &&
		    (status.tt >= 4 && status.tt <= 7)) {
			einfo.info.gpe.validation_bits |= GPE_OP_VALID;

			/* Transaction type */
			switch(status.tt) {
			case RERI_TT_IMPLICIT_READ:
				einfo.info.gpe.operation = 3;
				break;
			case RERI_TT_EXPLICIT_READ:
				einfo.info.gpe.operation = 1;
				break;
			case RERI_TT_IMPLICIT_WRITE:
			case RERI_TT_EXPLICIT_WRITE:
				einfo.info.gpe.operation = 2;
				break;
			default:
				einfo.info.gpe.operation = 0;
				break;
			}

			/* Translate error codes from RERI */
			switch(status.ec) {
			case RERI_EC_CBA:
			case RERI_EC_CSD:
			case RERI_EC_CAS:
			case RERI_EC_CUE:
				einfo.info.gpe.proc_err_type = 0x01;
				break;
			case RERI_EC_TPD:
			case RERI_EC_TPA:
			case RERI_EC_TPU:
				einfo.info.gpe.proc_err_type = 0x02;
				break;
			case RERI_EC_SBE:
				einfo.info.gpe.proc_err_type = 0x04;
				break;
			case RERI_EC_HSE:
			case RERI_EC_ITD:
			case RERI_EC_ITO:
			case RERI_EC_IWE:
			case RERI_EC_IDE:
			case RERI_EC_SMU:
			case RERI_EC_SMD:
			case RERI_EC_SMS:
			case RERI_EC_PIO:
			case RERI_EC_PUS:
			case RERI_EC_PTO:
			case RERI_EC_SIC:
				einfo.info.gpe.proc_err_type = 0x08;
				break;
			default:
				einfo.info.gpe.proc_err_type = 0x00;
				break;
			}
		}

		/* Address type */
		if (status.at) {
			einfo.info.gpe.validation_bits |= GPE_TARGET_ADDR_VALID;
			einfo.info.gpe.target_addr = eaddr;
		}

		einfo.etype = ERROR_TYPE_GENERIC_CPU;

		/* Update the CPER record */
		acpi_ghes_record_errors(ACPI_GHES_GENERIC_CPU_ERROR_SOURCE_ID,
					&einfo);

		*pending_vectors = riscv_reri_get_hart_sse_vector(hart_id);

		/* TODO: Return number of errors recorded */
		return 1;
	}

	return 0;
}

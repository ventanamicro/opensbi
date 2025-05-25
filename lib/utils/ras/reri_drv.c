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
#include <sbi/sbi_heap.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_sse.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/ras/riscv_reri_regs.h>
#include <sbi_utils/ras/apei_tables.h>
#include <sbi_utils/ras/ghes.h>
#include <sbi_utils/ras/reri_drv.h>
#include <sbi_utils/ras/ras_agent_einj.h>

struct reri_generic_dev {
	uint64_t addr;
	uint64_t size;
	uint32_t sse_vector;
	uint16_t src_id;
	uint16_t res;
};

typedef struct reri_generic_dev reri_dram_dev_t;

typedef struct reri_hart_dev {
	struct reri_generic_dev dev;
	int hart_id;
} reri_hart_dev_t;

static reri_hart_dev_t *reri_hart_devices = NULL;
static reri_dram_dev_t reri_dram_dev;
static uint32_t reri_nr_harts = 0;

#define RERI_HART_COMPAT	"riscv,reri-harts"
#define RERI_DRAM_COMPAT	"riscv,reri-dram"
#define APEI_MEM_COMPAT		"riscv,apei-mem"
#define RERI_ERR_BANK_SIZE	0x1000

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

static reri_hart_dev_t *get_reri_hart_dev(int hart_id)
{
		int i;

	for (i = 0; i < reri_nr_harts; i++) {
		if (reri_hart_devices[i].hart_id == hart_id) {
			return &reri_hart_devices[i];
		}
	}

	return NULL;
}

static int riscv_reri_get_hart_addr(int hart_id, uint64_t *hart_addr,
				    uint64_t *size)
{
	reri_hart_dev_t *reri_hart;

	reri_hart = get_reri_hart_dev(hart_id);
	if (!reri_hart)
		return SBI_ENOENT;

	*hart_addr = reri_hart->dev.addr;
	*size = reri_hart->dev.size;

	return SBI_SUCCESS;
}

static uint32_t riscv_reri_get_hart_sse_vector(int hart_id, uint32_t *sse_vector)
{
	reri_hart_dev_t *reri_hart;

	reri_hart = get_reri_hart_dev(hart_id);
	if (!reri_hart)
		return SBI_ENOENT;

	*sse_vector = reri_hart->dev.sse_vector;

	return 0;
}

static uint32_t riscv_reri_get_hart_src_id(int hart_id, uint32_t *hart_src_id)
{
	reri_hart_dev_t *reri_hart;

	reri_hart = get_reri_hart_dev(hart_id);
	if (!reri_hart)
		return SBI_ENOENT;

	*hart_src_id = reri_hart->dev.src_id;

	return 0;
}

static int fdt_parse_reri_device(const void *fdt, int nodeoff)
{
	int len, i, nr_harts, hart_phandle, cpu_offset, ret = SBI_SUCCESS;
	const fdt32_t *sse_vec_p, *src_id_p, *target_harts_p, *hart_id_p;
	const char *cpu_status;
	uint64_t addr, size;
	uint32_t sse_vec;
	uint16_t src_id;

	if ((ret = fdt_node_check_compatible(fdt, nodeoff,
					     RERI_DRAM_COMPAT)) == 0) {
		ret = fdt_get_node_addr_size(fdt, nodeoff, 0, &addr, &size);
		if (ret < 0)
			return ret;
		reri_dram_dev.addr = addr;
		reri_dram_dev.size = size;

		/*
		 * This should be M-mode and S-mode shared region for
		 * error injection support.
		 */
		ret = sbi_domain_root_add_memrange(addr, size, PAGE_SIZE,
						   (SBI_DOMAIN_MEMREGION_MMIO |
						    SBI_DOMAIN_MEMREGION_SHARED_SURW_MRW));
		if (ret < 0)
			return ret;

		sse_vec_p = fdt_getprop(fdt, nodeoff, "sse-event-id", &len);
		if (!sse_vec_p)
			return SBI_ENOENT;
		sse_vec = fdt32_to_cpu(*sse_vec_p);
		reri_dram_dev.sse_vector = sse_vec;

		src_id_p = fdt_getprop(fdt, nodeoff, "source-id", &len);
		if (!src_id_p)
			return SBI_ENOENT;
		src_id = fdt32_to_cpu(*src_id_p);
		reri_dram_dev.src_id = src_id;

		if ((ret = acpi_ghes_new_error_source(src_id, sse_vec)) < 0) {
			sbi_printf("Failed to create new DRAM error source\n");
			return ret;
		}

		ret = sbi_sse_add_event(sse_vec, NULL);
		ret = (ret != SBI_EALREADY) ? ret : 0;
		if (ret) {
			sbi_printf("Failed to add SSE event %u for error source %u\n",
				   sse_vec, src_id);
			return ret;
		}
	} else if ((ret = fdt_node_check_compatible(fdt, nodeoff,
						    RERI_HART_COMPAT)) == 0) {
		ret = fdt_get_node_addr_size(fdt, nodeoff, 0, &addr, &size);
		if (ret < 0)
			return ret;

		/*
		 * This should be M-mode and S-mode shared region for
		 * error injection support.
		 */
		ret = sbi_domain_root_add_memrange(addr, size, PAGE_SIZE,
						   (SBI_DOMAIN_MEMREGION_MMIO |
						    SBI_DOMAIN_MEMREGION_SHARED_SURW_MRW));
		if (ret < 0)
			return ret;

		sse_vec_p = fdt_getprop(fdt, nodeoff, "sse-event-id", &len);
		if (!sse_vec_p)
			return SBI_ENOENT;
		sse_vec = fdt32_to_cpu(*sse_vec_p);

		src_id_p = fdt_getprop(fdt, nodeoff, "base-source-id", &len);
		if (!src_id_p)
			return SBI_ENOENT;
		src_id = fdt32_to_cpu(*src_id_p);

		target_harts_p = fdt_getprop(fdt, nodeoff, "target-harts", &len);
		if (target_harts_p && len >= sizeof(fdt32_t)) {
			reri_nr_harts = nr_harts = len / sizeof(fdt32_t);
			reri_hart_devices = (reri_hart_dev_t *)sbi_malloc(sizeof(reri_hart_dev_t)
									  * nr_harts);
			if (!reri_hart_devices)
				return SBI_ENOMEM;

			memset(reri_hart_devices, 0, sizeof(reri_hart_dev_t) * nr_harts);

			for (i = 0; i < nr_harts; i++) {
				reri_hart_devices[i].hart_id = -1; /* set of invalid */

				hart_phandle = fdt32_to_cpu(target_harts_p[i]);

				cpu_offset = fdt_node_offset_by_phandle(fdt, hart_phandle);
				if (cpu_offset < 0)
					return SBI_ENOENT;

				cpu_status = fdt_getprop(fdt, cpu_offset, "status", &len);
				if (cpu_status &&
				    strncmp(cpu_status, "okay", strlen("okay")) != 0 &&
				    strncmp(cpu_status, "ok", strlen("ok")) != 0)
					continue;

				hart_id_p = fdt_getprop(fdt, cpu_offset, "reg", &len);
				if (!hart_id_p)
					continue;

				if ((ret = acpi_ghes_new_error_source(src_id, sse_vec)) < 0)
					continue;

				if ((ret = einj_register_error_source(src_id, addr + (i * RERI_ERR_BANK_SIZE))) != 0) {
					sbi_printf("Failed register error source %u with EINJ framework."
						   "Error injection will fail.\n",
						   src_id);
					return ret;
				}

				ret = sbi_sse_add_event(sse_vec, NULL);
				ret = (ret != SBI_EALREADY) ? ret : 0;
				if (ret) {
					sbi_printf("Failed to add SSE event %u\n", sse_vec);
					return ret;
				}

				reri_hart_devices[i].dev.addr = (addr + (i * RERI_ERR_BANK_SIZE));
				reri_hart_devices[i].dev.size = RERI_ERR_BANK_SIZE;
				reri_hart_devices[i].dev.sse_vector = sse_vec;
				reri_hart_devices[i].dev.src_id = src_id++;
				reri_hart_devices[i].hart_id = fdt32_to_cpu(*hart_id_p);
			}
		} else {
			return SBI_ENOENT;
		}
	}

	return ret;
}

int reri_drv_init(const void *fdt, int nodeoff, const struct fdt_match *match)
{
	int ret, doffset, moffset, len;
	uint64_t addr, size;
	const fdt32_t *rm_handle_p;
	uint32_t rm_handle;

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

		ret = sbi_domain_root_add_memrange(addr, size, PAGE_SIZE,
						   SBI_DOMAIN_MEMREGION_SHARED_SURW_MRW);
		if (ret < 0)
			return ret;

		acpi_ghes_init(addr, size);

		/*
		 * NOTE: Initialize error injection framework. EINJ uses the GHES reserved memory
		 * for the entries and the GAS regions. acpi_ghes_init must always be called
		 * before einj_init.
		 */
		ret = einj_init(fdt, nodeoff);
		if (ret)
			return ret;
	}

	fdt_for_each_subnode(doffset, fdt, nodeoff) {
		if (fdt_parse_reri_device(fdt, doffset) != 0)
			continue;
	}

	return SBI_SUCCESS;
}

int reri_drv_sync_hart_errs(u32 hart_id, u32 *pending_vectors)
{
	int ret;
	riscv_reri_error_bank *heb;
	riscv_reri_status status;
	uint64_t hart_addr, err_size;
	uint64_t eaddr;
	acpi_ghes_error_info einfo;
	uint32_t hart_src_id, sse_vector;

	if (riscv_reri_get_hart_addr(hart_id, &hart_addr, &err_size) != 0)
		return 0;

	if (riscv_reri_get_hart_src_id(hart_id, &hart_src_id) != 0)
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
		acpi_ghes_record_errors(hart_src_id, &einfo);

		if ((ret = riscv_reri_get_hart_sse_vector(hart_id, &sse_vector)) != 0)
			return ret;

		*pending_vectors = sse_vector;

		/* TODO: Return number of errors recorded */
		return 1;
	}

	return 0;
}

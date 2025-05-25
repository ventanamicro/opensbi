/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Ventana Micro Systems, Inc.
 *
 * Author(s):
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#ifndef __RAS_AGENT_EINJ_H
#define __RAS_AGENT_EINJ_H

#include <sbi_utils/ras/ghes.h>

typedef struct __packed einj_inst_entry {
	uint8_t action;
	uint8_t instruction;
	uint8_t flags;
	uint8_t reserved;
	acpi_gas register_region;
	uint64_t value;
	uint64_t mask;
} einj_inst_entry_t;

#ifdef CONFIG_FDT_SBI_RAS_AGENT_EINJ

int einj_init(const void *fdt, int nodeoff);
int einj_register_error_source(uint16_t src_id, uint64_t err_src_reri_addr);
int einj_get_total_injection_entries(void);
einj_inst_entry_t *einj_get_instruction(int index);
void einj_trigger_operation(void);
void einj_execute_operation(void);

#else /* CONFIG_RAS_AGENT_EINJ */

#define __unused __attribute__((unused))

static __unused int einj_init(const void *fdt, int nodeoff)
{
	return 0;
}

static __unused int einj_register_error_source(uint16_t src_id, uint64_t err_src_reri_addr)
{
	return 0;
}

static __unused int einj_get_total_injection_entries(void)
{
	return 0;
}

static __unused einj_inst_entry_t * einj_get_instruction(int index)
{
	return NULL;
}

static __unused void einj_trigger_operation(void) { }
static __unused void einj_execute_operation(void) { }

#endif

#endif /* __RAS_AGENT_EINJ_H */

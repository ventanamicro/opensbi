/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Ventana Micro Systems, Inc.
 *
 * Author(s):
 *   Himanshu Chauhan <hchauhan@ventanamicro.com>
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_ras.h>
#include <sbi/sbi_mpxy.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_hart.h>
#include <sbi_utils/ras/riscv_reri_regs.h>
#include <sbi_utils/ras/apei_tables.h>
#include <sbi_utils/ras/ghes.h>
#include <sbi_utils/ras/ras_agent_einj.h>
#include <sbi_utils/ras/ras_agent_mpxy.h>

enum einj_err_type_bit_pos {
	proc_correctable,
	proc_uncorrectable_non_fatal,
	proc_uncorrectable_fatal,
	mem_correctable,
	mem_uncorrectable_non_fatal,
	mem_uncorrectable_fatal,
	pcie_correctable,
	pcie_uncorrectable_non_fatal,
	pcie_uncorrectable_fatal,
	plat_correctable,
	plat_uncorrectable_non_fatal,
	plat_uncorrectable_fatal,
	cxlcp_correctable, /* CXL.cache Prototol */
	cxlcp_uncorrectable_non_fatal,
	cxlcp_uncorrectable_fatal,
	cxlmp_correctable, /* CXL.mem Protocol */
	cxlmp_uncorrectable_non_fatal,
	cxlmp_uncorrectable_fatal,
	/* 18:29 RESERVED */
	einjv2_etype = 30,
	vendor_defined_err_type,
};

enum {
	einj_action_begin_injection_operation,
	einj_action_get_trigger_action_table,
	einj_action_set_error_type,
	einj_action_get_error_type,
	einj_action_end_operation,
	einj_action_execute_operation,
	einj_action_check_busy_status,
	einj_action_get_command_status,
	einj_action_set_error_type_with_address,
	einj_action_get_execute_operation_timings,
	einj_action_einjv2_set_error_type,
	einj_action_einjv2_get_error_type,
	einj_action_trigger_error = 0xFF,
};

enum {
	einj_inst_read_register,
	einj_inst_read_register_value,
	einj_inst_write_register,
	einj_inst_write_register_value,
	einj_inst_noop,
};

typedef struct einj_inst_cont {
	einj_inst_entry_t inst;
	struct sbi_dlist node;
} einj_inst_cont_t;

typedef struct __packed einj_err_type_wtih_addr_data {
	uint32_t err_type;
	uint32_t vendor_etype_ext_offs;
	uint32_t flags;
	uint32_t proc_id;
	uint64_t mem_addr;
	uint64_t mem_addr_range;
	uint32_t pcie_bdf;
} einj_err_type_with_addr_data_t;

struct einj_reri_err_src {
	uint64_t addr;
	uint16_t src_id;

	struct sbi_dlist node;
};

typedef struct __packed einj_err_trigger_table_header {
	uint32_t hdr_sz;
	uint32_t revision;
	uint32_t tbl_sz;
	uint32_t entry_cnt;
} einj_err_trigger_table_header_t;

static SBI_LIST_HEAD(einj_reri_err_src_list);
static SBI_LIST_HEAD(einj_inst_list);

static uint64_t *supported_err_types = NULL;
static uint64_t *error_to_inject = NULL;
static int einj_total_injection_entries = 0;
static uint32_t mpxy_chan_id = 0;

#define EINJ_BUSY_BIT	 0

enum {
	COMMAND_STATUS_SUCCESS,
	COMMAND_STATUS_UNKNOWN_FAIL,
	COMMAND_STATUS_INVAL_ACCESS
};

#define EINJ_FFH_TYPE_BIT_SHIFT         60
#define EINJ_FFH_TYPE_BIT_MASK          (0xful)
#define EINJ_FFH_CHAN_ID_BIT_SHIFT      8
#define EINJ_FFH_CHAN_ID_BIT_MASK       (0xffffffULL)
#define EINJ_FFH_MSG_ID_BIT_SHIFT       0
#define EINJ_FFH_MSG_ID_BIT_MASK        (0xfful)

#define MAKE_FFH_ADDR(_type, _chan_id, _msg_id)				\
	({								\
		uint64_t _ffh_ = ((((uint64_t)_type << EINJ_FFH_TYPE_BIT_SHIFT) & EINJ_FFH_TYPE_BIT_MASK) \
				  | (((uint64_t)_chan_id << EINJ_FFH_CHAN_ID_BIT_SHIFT) & EINJ_FFH_CHAN_ID_BIT_MASK) \
				  | (((uint64_t)_msg_id << EINJ_FFH_MSG_ID_BIT_SHIFT) & EINJ_FFH_MSG_ID_BIT_MASK)); \
		(_ffh_);						\
	})

#define GET_GAS_ADDRESS_REGION(_op_entries)				\
	({								\
		uint64_t **_raddr = (uint64_t **)(ulong)_op_entries->register_region.address; \
		void *_region = (void *)_raddr;				\
		(_region);						\
	})

static void *einj_mem_alloc(uint64_t size)
{
	return acpi_ghes_alloc(size);
}

static einj_inst_cont_t *einj_alloc_inst_cont(int num_entries)
{
	einj_inst_cont_t *con = NULL;
	einj_inst_cont_t *t = NULL;
	int i = 0;

	t = con = (einj_inst_cont_t *)einj_mem_alloc(sizeof(einj_inst_cont_t) * num_entries);

	if (con == NULL)
		return NULL;

	for (i = 0; i < num_entries; i++) {
		SBI_INIT_LIST_HEAD(&t->node);
		t++;
	}

	return con;
}

static uint64_t *einj_alloc_register(void)
{
	return (uint64_t *)(einj_mem_alloc(sizeof(uint64_t)));
}

static inline einj_inst_cont_t *to_einj_inst_cont(struct sbi_dlist *node)
{
	return container_of(node, struct einj_inst_cont, node);
}

einj_inst_entry_t * einj_get_inst_for_act(int action, int instruction)
{
	struct sbi_dlist *pos;
	einj_inst_cont_t *eicont;
	einj_inst_entry_t *inst;

	sbi_list_for_each(pos, &einj_inst_list) {
		eicont = to_einj_inst_cont(pos);
		inst = &eicont->inst;

		if (inst->action == action
		    && inst->instruction == instruction) {
			return inst;
		}
	}

	return NULL;
}

void *einj_get_gas_region(int action, int instruction)
{
	void *gaddr;
	einj_inst_entry_t *inst;

	inst = einj_get_inst_for_act(action, instruction);

	if (inst == NULL)
		return NULL;

	gaddr = GET_GAS_ADDRESS_REGION(inst);

	return gaddr;
}

static void einj_build_begin_op_entries(void)
{
	int nr_entries = 1;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	inc->inst.action = einj_action_begin_injection_operation;
	inc->inst.instruction = einj_inst_noop;
	sbi_list_add(&(inc->node), &(einj_inst_list));

	einj_total_injection_entries += nr_entries;
}

static void einj_build_end_op_entries(void)
{
	int nr_entries = 1;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	inc->inst.action = einj_action_end_operation;
	inc->inst.instruction = einj_inst_noop;
	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

uint64_t *g_einj_busy_status_addr;
void *g_trigger_action_table_phys = NULL;
riscv_reri_error_record *g_current_err_rec = NULL;

static void einj_build_check_busy_status_op_entries(void)
{
	int nr_entries = 1;
	einj_inst_cont_t *inc = NULL;
	uint64_t *raddr = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	g_einj_busy_status_addr = raddr = einj_alloc_register();

	if (raddr == NULL)
		return;

	inc->inst.action = einj_action_check_busy_status;
	inc->inst.instruction = einj_inst_read_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint32_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_get_trigger_error_action_table_entries(void)
{
	int nr_entries = 1;
	einj_inst_cont_t *inc = NULL;
	uint64_t *raddr = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	raddr = einj_alloc_register();

	if (raddr == NULL)
		return;

	inc->inst.action = einj_action_get_trigger_action_table;
	inc->inst.instruction = einj_inst_read_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint32_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	*raddr = (ulong)g_trigger_action_table_phys;

	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_trigger_action_table(void)
{
	int nr_entries = 1;
	einj_err_trigger_table_header_t *hdr;
	einj_inst_entry_t *inst;

	g_trigger_action_table_phys = einj_mem_alloc(sizeof(*hdr) + (sizeof(*inst) * nr_entries));

	if (g_trigger_action_table_phys == NULL)
		return;

	hdr = (einj_err_trigger_table_header_t *)g_trigger_action_table_phys;
	inst = (einj_inst_entry_t *)(((uint8_t *)g_trigger_action_table_phys) + sizeof(einj_err_trigger_table_header_t));

	hdr->hdr_sz = sizeof(einj_err_trigger_table_header_t);
	hdr->revision = 0;
	hdr->entry_cnt = nr_entries;
	hdr->tbl_sz = (sizeof(einj_err_trigger_table_header_t) + (nr_entries * sizeof(einj_inst_entry_t)));

	inst->action = einj_action_trigger_error;
	inst->instruction = einj_inst_write_register;
	inst->flags = 0UL;
	inst->register_region.asid = AML_AS_FFH;
	inst->register_region.reg_bwidth = sizeof(uint64_t);
	inst->register_region.reg_boffs = 0;
	inst->register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inst->register_region.address = MAKE_FFH_ADDR(0, mpxy_chan_id, RAS_EINJ_TRIGGER_ERROR);
}

static void einj_build_execute_op_entries(void)
{
	int nr_entries = 1;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	inc->inst.action = einj_action_execute_operation;
	inc->inst.instruction = einj_inst_write_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_FFH;
	inc->inst.register_region.reg_bwidth = sizeof(uint64_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = MAKE_FFH_ADDR(0, mpxy_chan_id, RAS_EINJ_EXECUTE_OPERATION);
	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_get_err_type_entries(void)
{
	int nr_entries = 1;
	uint64_t *raddr;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	raddr = einj_alloc_register();
	inc->inst.action = einj_action_get_error_type;
	inc->inst.instruction = einj_inst_read_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint32_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	*raddr = (uint64_t)(*supported_err_types);
	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_get_command_status_op_entries(void)
{
	int nr_entries = 1;
	uint64_t *raddr;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	raddr = einj_alloc_register();

	inc->inst.action = einj_action_get_command_status;
	inc->inst.instruction = einj_inst_read_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint64_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	*raddr = 0;

	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_set_err_type_entries(void)
{
	int nr_entries = 1;
	uint64_t *raddr;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	raddr = einj_alloc_register();
	error_to_inject = einj_mem_alloc(sizeof(uint64_t));

	if (error_to_inject == NULL)
		return;

	*error_to_inject = 0;

	inc->inst.action = einj_action_set_error_type;
	inc->inst.instruction = einj_inst_write_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint64_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	*raddr = (ulong)error_to_inject;
	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

static void einj_build_set_err_type_with_addr_entries(void)
{
	int nr_entries = 1;
	uint64_t *raddr = NULL;
	einj_inst_cont_t *inc = NULL;

	inc = einj_alloc_inst_cont(nr_entries);

	if (inc == NULL)
		return;

	raddr = einj_alloc_register();
	if (raddr == NULL)
		return;

	inc->inst.action = einj_action_set_error_type_with_address;
	inc->inst.instruction = einj_inst_write_register;
	inc->inst.flags = 0UL;
	inc->inst.register_region.asid = AML_AS_SYSTEM_MEMORY;
	inc->inst.register_region.reg_bwidth = sizeof(uint64_t);
	inc->inst.register_region.reg_boffs = 0;
	inc->inst.register_region.access_sz = GAS_ACCESS_SZ_QWORD;
	inc->inst.register_region.address = (ulong)raddr;
	inc->inst.mask = 0xFFFFFFFFUL;

	sbi_list_add(&(inc->node), &(einj_inst_list));
	einj_total_injection_entries += nr_entries;
}

int einj_init(const void *fdt, int nodeoff)
{
	const fdt32_t *chan_id_p;
	int len;

	chan_id_p = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (!chan_id_p)
		return SBI_ENOENT;

	mpxy_chan_id = fdt32_to_cpu(*chan_id_p);

	supported_err_types = einj_mem_alloc(sizeof(uint64_t));

	if (supported_err_types == NULL)
		return SBI_ENOMEM;

	*supported_err_types = ((0x1UL << proc_correctable)
				| (0x1UL << proc_uncorrectable_non_fatal));

	einj_build_begin_op_entries();
	einj_build_end_op_entries();
	einj_build_get_err_type_entries();
	einj_build_set_err_type_entries();
	einj_build_set_err_type_with_addr_entries();
	einj_build_execute_op_entries();
	einj_build_get_command_status_op_entries();
	einj_build_check_busy_status_op_entries();
	einj_build_trigger_action_table();
	einj_build_get_trigger_error_action_table_entries();

	return SBI_SUCCESS;
}

int einj_register_error_source(uint16_t src_id, uint64_t err_src_reri_addr)
{
	int rc = 0;

	struct einj_reri_err_src *einj_src = sbi_malloc(sizeof(struct einj_reri_err_src));

	if (einj_src == NULL) {
		rc = SBI_ENOMEM;
		goto out;
	}

	einj_src->addr = err_src_reri_addr;
	einj_src->src_id = src_id;
	SBI_INIT_LIST_HEAD(&einj_src->node);
	sbi_list_add(&(einj_src->node), &(einj_reri_err_src_list));

 out:
	return rc;
}

int einj_get_total_injection_entries(void)
{
	return einj_total_injection_entries;
}

static inline struct einj_reri_err_src *to_reri_err_src(struct sbi_dlist *node)
{
	return container_of(node, struct einj_reri_err_src, node);
}

static void set_proc_error_with_type(uint32_t hart_id, uint32_t err_type)
{
	struct sbi_dlist *pos;
	uint64_t *command_status = NULL;
	struct einj_reri_err_src *reri_src;
	riscv_reri_error_record *rec;

	command_status = (uint64_t *)einj_get_gas_region(einj_action_get_command_status,
							 einj_inst_read_register);

	if ((err_type & (err_type - 1)) != 0) {
		sbi_printf("More than 1 error type set in error type field\n");
		*command_status = COMMAND_STATUS_INVAL_ACCESS;
		return;
	}

	switch(err_type) {
	case proc_correctable:
	case proc_uncorrectable_non_fatal:
	case proc_uncorrectable_fatal:
		sbi_list_for_each(pos, &(einj_reri_err_src_list)) {
			reri_src = to_reri_err_src(pos);
			if (reri_src->src_id == hart_id) {
				riscv_reri_error_bank *bank = (riscv_reri_error_bank *)(ulong)reri_src->addr;
				rec = (riscv_reri_error_record *)(&bank->records[0]);
				/*
				 * Both SET_ERROR_TYPE or SET_ERROR_TYPE_WITH_ADDRESS don't provide a way to
				 * specify details of the errors. For processor only the processor ID
				 * is specified. So setting others to a fixed sane default.
				 */
				g_current_err_rec = rec;
				rec->control_i.ces = 2;
				rec->control_i.ueds = 2;
				rec->control_i.uecs = 2;
				rec->status_i.ec = RERI_ERR_HART_STATE;
				rec->status_i.tt = TT_IMPLICIT_READ;
				if (err_type == proc_correctable) {
					rec->status_i.ce = 1;
					rec->status_i.de = 0;
					rec->status_i.ue = 0;
				} else if (err_type == proc_uncorrectable_non_fatal) {
					rec->status_i.ce = 0;
					rec->status_i.de = 1;
					rec->status_i.ue = 0;
				} else if (err_type == proc_uncorrectable_fatal) {
					rec->status_i.ce = 0;
					rec->status_i.de = 0;
					rec->status_i.ue = 1;
				}
				*command_status = COMMAND_STATUS_SUCCESS;
				return;
			}
		}

		sbi_printf("%s: Could not find the error source for hart-%u\n", __func__, hart_id);
	}
}

static void set_err_with_address(einj_err_type_with_addr_data_t *wdata)
{
	uint64_t *command_status = NULL;
	uint32_t hart_id;

	command_status = (uint64_t *)einj_get_gas_region(einj_action_get_command_status,
							 einj_inst_read_register);

	if ((wdata->err_type & (wdata->err_type - 1)) != 0) {
		sbi_printf("More than 1 error type set in error type field\n");
		*command_status = COMMAND_STATUS_INVAL_ACCESS;
		return;
	}

	switch(wdata->err_type) {
	case proc_correctable:
	case proc_uncorrectable_non_fatal:
	case proc_uncorrectable_fatal:
		if (wdata->flags & 0x1UL) {
			hart_id = wdata->proc_id;
			set_proc_error_with_type(hart_id, wdata->err_type);
		} else {
			sbi_printf("Processor correctable error requested but processor flag is not valid\n");
			*command_status = COMMAND_STATUS_INVAL_ACCESS;
			return;
		}
	}
}

static void set_err(void)
{
	uint64_t *set_type = (uint64_t *)einj_get_gas_region(einj_action_set_error_type,
							     einj_inst_write_register);

	switch(*set_type) {
	case proc_correctable:
	case proc_uncorrectable_non_fatal:
	case proc_uncorrectable_fatal:
		set_proc_error_with_type(0, *set_type);
		break;
	default:
		sbi_printf("%s: Set error type 0x%lx not supported\n", __func__, (ulong)*set_type);
		break;
	}
}

einj_inst_entry_t *einj_get_instruction(int index)
{
	struct sbi_dlist *pos;
	einj_inst_cont_t *eicont;
	int rind = 0;

	sbi_list_for_each(pos, &einj_inst_list) {
		if (rind == index) {
			eicont = to_einj_inst_cont(pos);
			return &eicont->inst;
		}
		rind++;
	}

	return NULL;
}

static inline void set_status_busy(void)
{
	einj_inst_entry_t *inst = einj_get_inst_for_act(einj_action_check_busy_status,
							einj_inst_read_register);
	volatile uint64_t *status = NULL;

	if (inst && inst->register_region.address) {
		status = (uint64_t *)(ulong)inst->register_region.address;
		/* Mark busy in executing operation */
		*status |= (0x1UL << EINJ_BUSY_BIT);
	}
}

static inline void set_status_free(void)
{
	einj_inst_entry_t *inst = einj_get_inst_for_act(einj_action_check_busy_status,
							einj_inst_read_register);
	volatile uint64_t *status = NULL;

	if (inst && inst->register_region.address) {
		status = (uint64_t *)(ulong)inst->register_region.address;
		/* operation done */
		*status &= ~(0x1UL << EINJ_BUSY_BIT);
	}
}

void einj_execute_operation(void)
{
	einj_err_type_with_addr_data_t *wa_data =
		(einj_err_type_with_addr_data_t *)einj_get_gas_region(einj_action_set_error_type_with_address,
								      einj_inst_write_register);

	set_status_busy();

	if (wa_data->err_type) {
		set_err_with_address(wa_data);
	} else {
		set_err();
	}

	set_status_free();
}

void einj_trigger_operation(void)
{
	if (g_current_err_rec == NULL) {
		sbi_printf("%s: Trigger address is NULL!\n", __func__);
		return;
	}

	g_current_err_rec->control_i.eid = 10;

	g_current_err_rec = NULL;
}

/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Intel Corporation. All rights reserved.
 */

#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_rpxy.h>
#include <libfdt.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/rpxy/fdt_rpxy.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_console.h>

#define MM_VERSION_MAJOR        1
#define MM_VERSION_MAJOR_SHIFT  16
#define MM_VERSION_MAJOR_MASK   0x7FFF
#define MM_VERSION_MINOR        0
#define MM_VERSION_MINOR_SHIFT  0
#define MM_VERSION_MINOR_MASK   0xFFFF
#define MM_VERSION_FORM(major, minor) ((major << MM_VERSION_MAJOR_SHIFT) | \
                                       (minor))
#define MM_VERSION_COMPILED     MM_VERSION_FORM(MM_VERSION_MAJOR, \
                                                MM_VERSION_MINOR)

/** SMM ServiceGroups IDs */
enum rpmi_mm_servicegroup_id {
	RPMI_SRVGRP_ID_MIN = 0,
	RPMI_SRVGRP_MM = 0x000A,
	RPMI_SRVGRP_ID_MAX_COUNT,
};

/** SMM ServiceGroup Service IDs */
enum rpmi_mm_service_id {
	RPMI_MM_SRV_VERSION = 0x01,
	RPMI_MM_SRV_COMMUNICATE = 0x02,
	RPMI_MM_SRV_COMPLETE = 0x03,
};

struct mm_boot_info {
	uint64_t mm_mem_base;
	uint64_t mm_mem_limit;
	uint64_t mm_image_base;
	uint64_t mm_stack_base;
	uint64_t mm_heap_base;
	uint64_t mm_ns_comm_buf_base;
	uint64_t mm_shared_buf_base;
	uint64_t mm_image_size;
	uint64_t mm_pcpu_stack_size;
	uint64_t mm_heap_size;
	uint64_t mm_ns_comm_buf_size;
	uint64_t mm_shared_buf_size;
	uint32_t num_mem_region;
};

struct rpxy_mm_data {
	u32 service_group_id;
	int num_services;
	struct sbi_rpxy_service *services;
};

static struct sbi_domain *tdomain;
static struct sbi_domain *udomain;

static int mm_boot_info_setup(void *fdt, int nodeoff, const struct fdt_match *match)
{
	const u32 *prop_value;
	u64 base64, size64;
	int len;

	struct mm_boot_info *boot_info = NULL;
	prop_value = fdt_getprop(fdt, nodeoff, "boot-info-address", &len);
	if (!prop_value || len < 8)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	boot_info = (struct mm_boot_info *)base64;

	prop_value = fdt_getprop(fdt, nodeoff, "num-regions", &len);
	if (!prop_value || len < 4)
		return SBI_EINVAL;
	boot_info->num_mem_region = (unsigned long)fdt32_to_cpu(*prop_value);

	prop_value = fdt_getprop(fdt, nodeoff, "memory-region", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_mem_base	= base64;
	boot_info->mm_mem_limit	= base64 + size64;

	prop_value = fdt_getprop(fdt, nodeoff, "image-region", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_image_base	= base64;
	boot_info->mm_image_size	= size64;

	prop_value = fdt_getprop(fdt, nodeoff, "heap-region", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_heap_base	= base64;
	boot_info->mm_heap_size	= size64;

	prop_value = fdt_getprop(fdt, nodeoff, "stack-region", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_stack_base	= base64 + size64 -1;

	prop_value = fdt_getprop(fdt, nodeoff, "pcpu-stack-size", &len);
	if (!prop_value || len < 4)
		return SBI_EINVAL;
	boot_info->mm_pcpu_stack_size = (unsigned long)fdt32_to_cpu(*prop_value);

	prop_value = fdt_getprop(fdt, nodeoff, "shared-buffer", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_shared_buf_base	= base64;
	boot_info->mm_shared_buf_size	= size64;

	prop_value = fdt_getprop(fdt, nodeoff, "ns-comm-buffer", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_ns_comm_buf_base	= base64;
	boot_info->mm_ns_comm_buf_size	= size64;

	return 0;
}

static int rpxy_mm_handler(struct sbi_rpxy_service_group *grp,
				  struct sbi_rpxy_service *srv,
				  void *tx, u32 tx_len,
				  void *rx, u32 rx_len,
				  unsigned long *ack_len)
{
	int srv_id = srv->id;
	struct rpxy_state *rs;

	if (RPMI_MM_SRV_VERSION == srv_id) {
		*((uint32_t *)rx) = MM_VERSION_COMPILED;
	} else if (RPMI_MM_SRV_COMMUNICATE == srv_id) {
		if(!udomain)
			udomain = sbi_domain_thishart_ptr();

		if(tdomain) {
			/* Get per-hart RPXY share memory with tdomain */
			rs = sbi_hartindex_to_domain_rs(
				sbi_hartid_to_hartindex(current_hartid()), tdomain);
			if (rs && rs->shmem_addr && tx && ((void *)rs->shmem_addr != tx)) {
				sbi_memcpy((void *)rs->shmem_addr, tx, tx_len);
			}

			sbi_domain_context_enter(tdomain);
		}
	} else if (RPMI_MM_SRV_COMPLETE == srv_id) {
		if(!tdomain)
			tdomain = sbi_domain_thishart_ptr();

		if(udomain) {
			/* Get per-hart RPXY share memory with udomain */
			rs = sbi_hartindex_to_domain_rs(
				sbi_hartid_to_hartindex(current_hartid()), udomain);
			if (rs && rs->shmem_addr && tx && ((void *)rs->shmem_addr != tx)) {
				sbi_memcpy((void *)rs->shmem_addr, tx, tx_len);
			}
		}

		sbi_domain_context_exit();
	}

	return 0;
}

static int rpxy_mm_init(void *fdt, int nodeoff,
			  const struct fdt_match *match)
{
	int rc;
	struct sbi_rpxy_service_group *group;
	const struct rpxy_mm_data *data = match->data;
	group = sbi_zalloc(sizeof(*group));
	if (!group)
		return SBI_ENOMEM;

	rc = mm_boot_info_setup(fdt, nodeoff, match);
	if (rc) {
		sbi_free(group);
		return 0;
	}

	/* Setup RPXY service group */
	group->transport_id = 0;
	group->service_group_id = data->service_group_id;
	group->max_message_data_len = -1;
	group->num_services = data->num_services;
	group->services = data->services;
	group->send_message = rpxy_mm_handler;
	/* Register RPXY service group */
	rc = sbi_rpxy_register_service_group(group);
	if (rc) {
		sbi_free(group);
		return rc;
	}

	return 0;
}

static struct sbi_rpxy_service mm_services[] = {
{
	.id = RPMI_MM_SRV_VERSION,
	.min_tx_len = 0,
	.max_tx_len = 0,
	.min_rx_len = sizeof(u32),
	.max_rx_len = sizeof(u32),
},
{
	.id = RPMI_MM_SRV_COMMUNICATE,
	.min_tx_len = 0,
	.max_tx_len = 0x8000,
	.min_rx_len = 0,
	.max_rx_len = 0x8000,
},
{
	.id = RPMI_MM_SRV_COMPLETE,
	.min_tx_len = 0,
	.max_tx_len = 0x8000,
	.min_rx_len = 0,
	.max_rx_len = 0x8000,
},
};

static struct rpxy_mm_data mm_data = {
	.service_group_id = RPMI_SRVGRP_MM,
	.num_services = array_size(mm_services),
	.services = mm_services,
};

static const struct fdt_match rpxy_mm_match[] = {
	{ .compatible = "riscv,sbi-rpxy-mm", .data = &mm_data },
	{},
};

struct fdt_rpxy fdt_rpxy_mm = {
	.match_table = rpxy_mm_match,
	.init = rpxy_mm_init,
};

/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_mpxy.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_console.h>

/** Offset of pointer to MPXY state in scratch space */
static unsigned long mpxy_state_offset;

/** List of MPXY proxy channels */
static SBI_LIST_HEAD(mpxy_channel_list);

/** Invalid Physical Address(all bits 1) */
#define INVALID_ADDR		(-1U)

/** MPXY Attribute size in bytes */
#define ATTR_SIZE			(4)

/** Channel Capability - MSI */
#define CAP_MSI_POS		0
#define CAP_MSI_MASK		(1U << CAP_MSI_POS)
/** Channel Capability - SSE */
#define CAP_SSE_POS		1
#define CAP_SSE_MASK		(1U << CAP_SSE_POS)

#if __riscv_xlen == 64
#define SHMEM_PHYS_ADDR(_hi, _lo) (_lo)
#elif __riscv_xlen == 32
#define SHMEM_PHYS_ADDR(_hi, _lo) (((u64)(_hi) << 32) | (_lo))
#else
#error "Undefined XLEN"
#endif

/** Per hart shared memory */
struct mpxy_shmem {
	unsigned long shmem_size;
	unsigned long shmem_addr_lo;
	unsigned long shmem_addr_hi;
};

struct mpxy_state {
	/* MSI support in MPXY */
	bool msi_avail;
	/* SSE support in MPXY */
	bool sse_avail;
	/* MPXY Shared memory details */
	struct mpxy_shmem shmem;
};

/** Disable hart shared memory */
static inline void sbi_mpxy_shmem_disable(struct mpxy_state *rs)
{
	rs->shmem.shmem_size = 0;
	rs->shmem.shmem_addr_lo = INVALID_ADDR;
	rs->shmem.shmem_addr_hi = INVALID_ADDR;
}

/** Check if shared memory is already setup on hart */
static inline bool mpxy_shmem_enabled(struct mpxy_state *rs)
{
	return (rs->shmem.shmem_addr_lo == INVALID_ADDR
		&& rs->shmem.shmem_addr_hi == INVALID_ADDR) ?
		false : true;
}

/** Get hart shared memory base address */
static inline void *hart_shmem_base(struct mpxy_state *rs)
{
	return (void *)(unsigned long)SHMEM_PHYS_ADDR(rs->shmem.shmem_addr_hi,
						rs->shmem.shmem_addr_lo);
}


/** Make sure all attributes are packed for direct memcpy in ATTR_READ */
#define assert_field_offset(field, attr_offset)				\
	_Static_assert(							\
		((offsetof(struct sbi_mpxy_channel_attrs, field)) /	\
		 sizeof(u32)) == attr_offset,				\
		"field " #field						\
		" from struct sbi_mpxy_channel_attrs invalid offset, expected " #attr_offset)

assert_field_offset(msg_proto_id, SBI_MPXY_ATTR_MSG_PROT_ID);
assert_field_offset(msg_proto_version, SBI_MPXY_ATTR_MSG_PROT_VER);
assert_field_offset(msg_data_maxlen, SBI_MPXY_ATTR_MSG_MAX_LEN);
assert_field_offset(msg_send_timeout, SBI_MPXY_ATTR_MSG_SEND_TIMEOUT);
assert_field_offset(capability, SBI_MPXY_ATTR_CHANNEL_CAPABILITY);
assert_field_offset(msi_control, SBI_MPXY_ATTR_MSI_CONTROL);
assert_field_offset(msi_info.msi_addr_lo, SBI_MPXY_ATTR_MSI_ADDR_LO);
assert_field_offset(msi_info.msi_addr_hi, SBI_MPXY_ATTR_MSI_ADDR_HI);
assert_field_offset(msi_info.msi_data, SBI_MPXY_ATTR_MSI_DATA);
assert_field_offset(sse_event_id, SBI_MPXY_ATTR_SSE_EVENT_ID);
assert_field_offset(eventsstate_ctrl, SBI_MPXY_ATTR_EVENTS_STATE_CONTROL);

/**
 * Check if the attribute is a standard attribute or
 * a message protocol specific attribute
 * attr_id[31] = 0 for standard
 * attr_id[31] = 1 for message protocol specific
 */
static inline bool mpxy_is_std_attr(u32 attr_id)
{
	return (attr_id >> 31) ? false : true;
}

/** Find channel_id in registered channels list */
static struct sbi_mpxy_channel *mpxy_find_channel(u32 channel_id)
{
	struct sbi_mpxy_channel *channel;

	sbi_list_for_each_entry(channel, &mpxy_channel_list, head)
		if (channel->channel_id == channel_id)
			return channel;

	return NULL;
}

/** Copy attributes word size */
static void mpxy_copy_std_attrs(u32 *outmem, u32 *inmem, u32 count)
{
	int idx;
	for (idx = 0; idx < count; idx++)
		outmem[idx] = inmem[idx];
}

/** Check if any channel is registered with mpxy framework */
bool sbi_mpxy_channel_available(void)
{
	return sbi_list_empty(&mpxy_channel_list) ? false : true;
}

static void mpxy_std_attrs_init(struct sbi_mpxy_channel *channel)
{
	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	/* Reset values */
	channel->attrs.msi_control = 0;
	channel->attrs.msi_info.msi_data = 0;
	channel->attrs.msi_info.msi_addr_lo = INVALID_ADDR;
	channel->attrs.msi_info.msi_addr_hi = INVALID_ADDR;
	channel->attrs.capability = 0;
	channel->attrs.eventsstate_ctrl = 0;

	/**
	 * Check if MSI or SSE available for notification interrrupt.
	 * Priority given to MSI if both MSI and SSE are avaialble.
	 */
	if (rs->msi_avail)
		channel->attrs.capability =
			CAP_ENABLE(channel->attrs.capability, CAP_MSI_MASK);
	else if (rs->sse_avail) {
		channel->attrs.capability =
			CAP_ENABLE(channel->attrs.capability, CAP_SSE_MASK);
		/* TODO: Assign SSE EVENT_ID for the channel */
	}

	/**
	 * Enable Events State in channel capability if message protocol
	 * provides callback to switch
	 */
	if (channel->switch_eventsstate)
		channel->attrs.capability =
			CAP_ENABLE(channel->attrs.capability,
					CAP_EVENTSSTATE_MASK);
}

/**
 * Register a channel with MPXY framework.
 * Called by message protocol drivers
 */
int sbi_mpxy_register_channel(struct sbi_mpxy_channel *channel)
{
	if (!channel)
		return SBI_EINVAL;

	if (mpxy_find_channel(channel->channel_id))
		return SBI_EALREADY;

	/* Initialize channel specific attributes */
	mpxy_std_attrs_init(channel);

	SBI_INIT_LIST_HEAD(&channel->head);
	sbi_list_add_tail(&channel->head, &mpxy_channel_list);

	return SBI_OK;
}

int sbi_mpxy_init(struct sbi_scratch *scratch)
{
	mpxy_state_offset = sbi_scratch_alloc_type_offset(struct mpxy_state);
	if (!mpxy_state_offset)
		return SBI_ENOMEM;

	/** TODO: Proper support for checking msi support from platform.
	 * Currently disable msi and sse and use polling
	 **/
	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);
	rs->msi_avail = false;
	rs->sse_avail = false;

	sbi_mpxy_shmem_disable(rs);

	return sbi_platform_mpxy_init(sbi_platform_ptr(scratch));
}

int sbi_mpxy_set_shmem(unsigned long shmem_size, unsigned long shmem_phys_lo,
		       unsigned long shmem_phys_hi, unsigned long flags)
{
	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);
	struct mpxy_state prev_rs;

	/** Disable shared memory if both hi and lo have all bit 1s */
	if (shmem_phys_lo == INVALID_ADDR &&
	    shmem_phys_hi == INVALID_ADDR) {
		sbi_mpxy_shmem_disable(rs);
		return SBI_SUCCESS;
	}

	if (flags >= SBI_EXT_MPXY_SHMEM_FLAG_MAX_IDX)
		return SBI_ERR_INVALID_PARAM;

	/** Check shared memory size and address aligned to 4K Page */
	if (!shmem_size || (shmem_size & ~PAGE_MASK) ||
	    (shmem_phys_lo & ~PAGE_MASK))
		return SBI_ERR_INVALID_PARAM;

	if (!sbi_domain_check_addr_range(sbi_domain_thishart_ptr(),
				SHMEM_PHYS_ADDR(shmem_phys_hi, shmem_phys_lo),
				shmem_size, PRV_S,
				SBI_DOMAIN_READ | SBI_DOMAIN_WRITE))
		return SBI_ERR_INVALID_ADDRESS;

	/** Save the current shmem details in new shmem region */
	if (flags == SBI_EXT_MPXY_SHMEM_FLAG_OVERWRITE_RETURN) {
		prev_rs.shmem.shmem_size    = rs->shmem.shmem_size;
		prev_rs.shmem.shmem_addr_lo = rs->shmem.shmem_addr_lo;
		prev_rs.shmem.shmem_addr_hi = rs->shmem.shmem_addr_hi;

		sbi_memcpy((void *)(unsigned long)SHMEM_PHYS_ADDR(shmem_phys_hi, shmem_phys_lo),
			   &prev_rs,
			   sizeof(unsigned long) * 3);
	}

	/** Setup the new shared memory */
	rs->shmem.shmem_size	= shmem_size;
	rs->shmem.shmem_addr_lo = shmem_phys_lo;
	rs->shmem.shmem_addr_hi = shmem_phys_hi;

	return SBI_SUCCESS;
}

int sbi_mpxy_read_attrs(u32 channel_id, u32 base_attr_id, u32 attr_count)
{
	int ret;
	u32 *attr_ptr, end_id;
	void *shmem_base;

	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	if (!mpxy_shmem_enabled(rs))
		return SBI_ERR_NO_SHMEM;

	struct sbi_mpxy_channel *channel = mpxy_find_channel(channel_id);
	if (!channel)
		return SBI_ERR_NOT_SUPPORTED;

	/* Sanity check for base_attr_id and attr_count */
	if (!attr_count ||
		(attr_count > rs->shmem.shmem_size / ATTR_SIZE) ||
		base_attr_id >= SBI_MPXY_ATTR_STD_ATTR_MAX_IDX)
		return SBI_ERR_INVALID_PARAM;

	shmem_base = hart_shmem_base(rs);
	end_id = base_attr_id + attr_count - 1;

	sbi_hart_map_saddr((unsigned long)hart_shmem_base(rs),
				rs->shmem.shmem_size);

	/* Standard attributes range check */
	if (mpxy_is_std_attr(base_attr_id)) {
		if (end_id >= SBI_MPXY_ATTR_STD_ATTR_MAX_IDX) {
			ret = SBI_EBAD_RANGE;
			goto out;
		}

		attr_ptr = (u32 *)&channel->attrs;
		mpxy_copy_std_attrs((u32 *)shmem_base, &attr_ptr[base_attr_id],
				    attr_count);
	} else {
		/**
		 * Even if the message protocol driver does not provide
		 * read attribute callback, return bad range error instead
		 * of not supported to let client distinguish it from channel
		 * id not supported.
		 */
		if (!channel->read_attributes ||
				end_id >= SBI_MPXY_ATTR_MSGPROTO_ATTR_END) {
			ret = SBI_ERR_BAD_RANGE;
			goto out;
		}

		/* Function expected to return the SBI supported errors */
		ret = channel->read_attributes(channel,
					       (u32 *)shmem_base,
					       base_attr_id, attr_count);
	}
out:
	sbi_hart_unmap_saddr();
	return ret;
}

/**
 * Verify the channel standard attribute wrt to write permission
 * and the value to be set if valid or not.
 * Only attributes needs to be checked which are defined Read/Write
 * permission. Other with Readonly permission will result in error.
 *
 * Attributes values to be written must also be checked because
 * before writing a range of attributes, we need to make sure that
 * either complete range of attributes is written successfully or not
 * at all.
 */
static int mpxy_check_write_std_attr(struct sbi_mpxy_channel *channel,
				     u32 attr_id, u32 attr_val)
{
	int ret = SBI_SUCCESS;
	struct sbi_mpxy_channel_attrs *attrs = &channel->attrs;

	switch(attr_id) {
	case SBI_MPXY_ATTR_MSI_CONTROL:
		if (attr_val > 1)
			ret = SBI_ERR_INVALID_PARAM;
		if (attr_val == 1 &&
		    (attrs->msi_info.msi_addr_lo == INVALID_ADDR) &&
		    (attrs->msi_info.msi_addr_hi == INVALID_ADDR))
			ret = SBI_ERR_DENIED;
		break;
	case SBI_MPXY_ATTR_MSI_ADDR_LO:
	case SBI_MPXY_ATTR_MSI_ADDR_HI:
	case SBI_MPXY_ATTR_MSI_DATA:
		ret = SBI_SUCCESS;
		break;
	case SBI_MPXY_ATTR_EVENTS_STATE_CONTROL:
		if (attr_val > 1)
			ret = SBI_ERR_INVALID_PARAM;
		break;
	default:
		/** All RO access attributes falls under default */
		ret = SBI_ERR_BAD_RANGE;
	};

	return ret;
}

/**
 * Write the attribute value
 */
static void mpxy_write_std_attr(struct sbi_mpxy_channel *channel, u32 attr_id,
			        u32 attr_val)
{
	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	struct sbi_mpxy_channel_attrs *attrs = &channel->attrs;

	switch(attr_id) {
	case SBI_MPXY_ATTR_MSI_CONTROL:
		if (rs->msi_avail && attr_val <= 1)
			attrs->msi_control = attr_val;
		break;
	case SBI_MPXY_ATTR_MSI_ADDR_LO:
		if (rs->msi_avail)
			attrs->msi_info.msi_addr_lo = attr_val;
		break;
	case SBI_MPXY_ATTR_MSI_ADDR_HI:
		if (rs->msi_avail)
			attrs->msi_info.msi_addr_hi = attr_val;
		break;
	case SBI_MPXY_ATTR_MSI_DATA:
		if (rs->msi_avail)
			attrs->msi_info.msi_data = attr_val;
		break;
	case SBI_MPXY_ATTR_EVENTS_STATE_CONTROL:
		if (CAP_GET(attrs->capability, CAP_EVENTSSTATE_MASK)) {
			attrs->eventsstate_ctrl = attr_val;
			/* call message protocol callback */
			channel->switch_eventsstate(attr_val);
		}

		break;
	};
}

int sbi_mpxy_write_attrs(u32 channel_id, u32 base_attr_id, u32 attr_count)
{
	int ret, mem_idx;
	void *shmem_base;
	u32 *mem_ptr, attr_id, end_id, attr_val;

	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	if (!mpxy_shmem_enabled(rs))
		return SBI_ERR_NO_SHMEM;

	struct sbi_mpxy_channel *channel = mpxy_find_channel(channel_id);
	if (!channel)
		return SBI_ERR_NOT_SUPPORTED;

	if (!attr_count ||
		(attr_count > rs->shmem.shmem_size / ATTR_SIZE) ||
		base_attr_id >= SBI_MPXY_ATTR_STD_ATTR_MAX_IDX)
		return SBI_ERR_INVALID_PARAM;

	shmem_base = hart_shmem_base(rs);
	end_id = base_attr_id + attr_count - 1;

	sbi_hart_map_saddr((unsigned long)shmem_base, rs->shmem.shmem_size);

	mem_ptr = (u32 *)shmem_base;

	if (mpxy_is_std_attr(base_attr_id)) {
		if (end_id >= SBI_MPXY_ATTR_STD_ATTR_MAX_IDX) {
			ret = SBI_ERR_BAD_RANGE;
			goto out;
		}

		/** Verify the attribute ids range and values */
		mem_idx = 0;
		for (attr_id = base_attr_id; attr_id <= end_id; attr_id++) {
			attr_val = mem_ptr[mem_idx++];
			ret = mpxy_check_write_std_attr(channel,
							attr_id, attr_val);
			if (ret)
				goto out;
		}

		/* Write the attribute ids values */
		mem_idx = 0;
		for (attr_id = base_attr_id; attr_id <= end_id; attr_id++) {
			attr_val = mem_ptr[mem_idx++];
			mpxy_write_std_attr(channel, attr_id, attr_val);
		}
	} else {/**
		 * Message protocol specific attributes:
		 * If attributes belong to message protocol, they
		 * are simply passed to the message protocol driver
		 * callback after checking the valid range.
		 * Attributes contiguous range & permission & other checks
		 * are done by the mpxy and message protocol glue layer.
		 */
		/**
		 * Even if the message protocol driver does not provide
		 * write attribute callback, return bad range error instead
		 * of not supported to let client distinguish it from channel
		 * id not supported.
		 */
		if (!channel->write_attributes ||
				end_id >= SBI_MPXY_ATTR_MSGPROTO_ATTR_END) {
			ret = SBI_ERR_BAD_RANGE;
			goto out;
		}

		/* Function expected to return the SBI supported errors */
		ret = channel->write_attributes(channel,
					       (u32 *)shmem_base,
					       base_attr_id, attr_count);
	}
out:
	sbi_hart_unmap_saddr();
	return ret;
}

int sbi_mpxy_send_message(u32 channel_id, u8 msg_id, unsigned long msg_data_len,
			  unsigned long *resp_data_len)
{
	int ret;
	void *msgbuf, *shmem_base;

	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	if (!mpxy_shmem_enabled(rs))
		return SBI_ERR_NO_SHMEM;

	struct sbi_mpxy_channel *channel = mpxy_find_channel(channel_id);
	if (!channel)
		return SBI_ERR_NOT_SUPPORTED;

	if (!channel->send_message)
		return SBI_ERR_NOT_IMPLEMENTED;

	if (msg_data_len > rs->shmem.shmem_size ||
		msg_data_len > channel->attrs.msg_data_maxlen)
		return SBI_ERR_INVALID_PARAM;

	shmem_base = hart_shmem_base(rs);
	sbi_hart_map_saddr((unsigned long)shmem_base, rs->shmem.shmem_size);
	msgbuf = shmem_base;

	ret = channel->send_message(channel, msg_id, msgbuf, msg_data_len,
				    resp_data_len ? shmem_base : NULL,
				    resp_data_len ? rs->shmem.shmem_size : 0,
				    resp_data_len);
	sbi_hart_unmap_saddr();
	if (ret)
		return ret;

	if (resp_data_len &&
	    (*resp_data_len > rs->shmem.shmem_size ||
	     *resp_data_len > channel->attrs.msg_data_maxlen))
		return SBI_ERR_FAILED;

	return SBI_SUCCESS;
}

int sbi_mpxy_get_notification_events(u32 channel_id, unsigned long *events_len)
{
	int ret;
	void *eventsbuf, *shmem_base;

	struct mpxy_state *rs =
		sbi_scratch_thishart_offset_ptr(mpxy_state_offset);

	if (!mpxy_shmem_enabled(rs))
		return SBI_ERR_NO_SHMEM;

	struct sbi_mpxy_channel *channel = mpxy_find_channel(channel_id);
	if (!channel)
		return SBI_ERR_NOT_SUPPORTED;

	if (!channel->get_notification_events)
		return SBI_ERR_NOT_IMPLEMENTED;

	shmem_base = hart_shmem_base(rs);
	sbi_hart_map_saddr((unsigned long)shmem_base, rs->shmem.shmem_size);
	eventsbuf = shmem_base;
	ret = channel->get_notification_events(channel, eventsbuf,
					       rs->shmem.shmem_size,
					       events_len);
	sbi_hart_unmap_saddr();

	if (ret)
		return ret;

	if (*events_len > rs->shmem.shmem_size)
		return SBI_ERR_FAILED;

	return SBI_SUCCESS;
}

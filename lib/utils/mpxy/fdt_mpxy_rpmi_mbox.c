/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_mpxy.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>
#include <sbi_utils/mailbox/fdt_mailbox.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>
#include <sbi/sbi_console.h>

#define RPMI_MAJOR_VER		(0x0000)
#define RPMI_MINOR_VER		(0x0001)
#define RPMI_MSG_SEND_TIMEOUT	(10)	/* microseconds */

#define SERVICE_DATA_SENTINAL	(0xff) /* FIXME: Its a valid service id */

struct mpxy_mbox_data {
	u32 servicegrp_id;
	u32 notifications_support;
	void *priv_data;
};

/**
 * RPMI service data for a service group
 * FIXME: Remove the dependency of sentinal value
 */
struct rpmi_service_data {
	u8 id;
	u32 min_tx_len;
	u32 max_tx_len;
	u32 min_rx_len;
	u32 max_rx_len;
};

/**
 * MPXY mbox instance per MPXY channel. This
 * ties an MPXY channel with an RPMI Service group
 */
struct mpxy_mbox {
	struct mbox_chan *chan;
	struct rpmi_service_data *srv_data;
	struct sbi_mpxy_channel channel;
};

/**
 * Discover the RPMI service data using message_id
 * MPXY message_id == RPMI service_id
 */
static struct rpmi_service_data *mpxy_find_rpmi_srvid(u32 message_id,
						struct rpmi_service_data *srv)
{
	int mid = 0;
	for (mid = 0; srv[mid].id != SERVICE_DATA_SENTINAL; mid++) {
		if (srv[mid].id == (u8)message_id)
			return &srv[mid];
	}

	return NULL;
}

/**
 * TODO: Message Protocol specific attributes support
 */
static int mpxy_mbox_read_attributes(struct sbi_mpxy_channel *channel,
				     u32 *outmem, u32 base_attr_id,
				     u32 attr_count)
{
	sbi_printf("read msgproto attrs: attr_id: %x, attr_count: %u\n",
			base_attr_id, attr_count);
	return SBI_ENOTSUPP;
}

/**
 * TODO: Message Protocol specific attributes support
 */
static int mpxy_mbox_write_attributes(struct sbi_mpxy_channel *channel,
				     u32 *outmem, u32 base_attr_id,
				     u32 attr_count)
{
	sbi_printf("write msgproto attrs: attr_id: %x, attr_count: %u\n",
			base_attr_id, attr_count);

	return SBI_ENOTSUPP;
}

static int mpxy_mbox_send_message(struct sbi_mpxy_channel *channel,
				  u32 message_id, void *tx, u32 tx_len,
				  void *rx, u32 rx_max_len,
				  unsigned long *ack_len)
{
	int ret;
	u32 rx_len = 0;
	struct mbox_xfer xfer;
	struct rpmi_message_args args = {0};
	struct mpxy_mbox *rmb =
		container_of(channel, struct mpxy_mbox, channel);
	struct rpmi_service_data *srv =
			mpxy_find_rpmi_srvid(message_id, rmb->srv_data);
	if (!srv)
		return SBI_EFAIL;

	if (tx_len < srv->min_tx_len || tx_len > srv->max_tx_len)
		return SBI_EFAIL;

	if (ack_len) {
		if (srv->min_rx_len == srv->max_rx_len)
			rx_len = srv->min_rx_len;
		else if (srv->max_rx_len < channel->attrs.msg_data_maxlen)
			rx_len = srv->max_rx_len;
		else
			rx_len = channel->attrs.msg_data_maxlen;

		args.type = RPMI_MSG_NORMAL_REQUEST;
		args.flags = (rx) ? 0 : RPMI_MSG_FLAGS_NO_RX;
		args.service_id = srv->id;
		mbox_xfer_init_txrx(&xfer, &args,
				    tx, tx_len, RPMI_DEF_TX_TIMEOUT,
				    rx, rx_len, RPMI_DEF_TX_TIMEOUT);
	}
	else {
		args.type = RPMI_MSG_POSTED_REQUEST;
		args.flags = RPMI_MSG_FLAGS_NO_RX;
		args.service_id = srv->id;
		mbox_xfer_init_tx(&xfer, &args,
				  tx, tx_len, RPMI_DEF_TX_TIMEOUT);
	}

	ret = mbox_chan_xfer(rmb->chan, &xfer);
	if (ret)
		return (ret == SBI_ETIMEDOUT) ? SBI_ETIMEDOUT : SBI_EFAIL;

	if (ack_len)
		*ack_len = args.rx_data_len;

	return SBI_OK;
}

static int mpxy_mbox_get_notifications(struct sbi_mpxy_channel *channel,
				       void *eventsbuf, u32 bufsize,
				       unsigned long *events_len)
{

	return SBI_ENOTSUPP;
}

static int mpxy_mbox_init(void *fdt, int nodeoff,
			  const struct fdt_match *match)
{
	int rc, len;
	const fdt32_t *val;
	u32 channel_id;
	struct mpxy_mbox *rmb;
	struct mbox_chan *chan;
	const struct mpxy_mbox_data *data = match->data;

	/* Allocate context for RPXY mbox client */
	rmb = sbi_zalloc(sizeof(*rmb));
	if (!rmb)
		return SBI_ENOMEM;

	/*
	 * If channel request failed then other end does not support
	 * service group so do nothing.
	 */
	rc = fdt_mailbox_request_chan(fdt, nodeoff, 0, &chan);
	if (rc) {
		sbi_free(rmb);
		return 0;
	}

	/* Match channel service group id */
	if (data->servicegrp_id != chan->chan_args[0]) {
		mbox_controller_free_chan(chan);
		sbi_free(rmb);
		return SBI_EINVAL;
	}

	val = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (len > 0 && val)
		channel_id = fdt32_to_cpu(*val);
	else {
		mbox_controller_free_chan(chan);
		sbi_free(rmb);
		return SBI_ENODEV;
	}

	/* Setup MPXY mbox client */
	/* Channel ID*/
	rmb->channel.channel_id = channel_id;
	/* Callback for read RPMI attributes */
	rmb->channel.read_attributes = mpxy_mbox_read_attributes;
	/* Callback for write RPMI attributes */
	rmb->channel.write_attributes = mpxy_mbox_write_attributes;
	/* Callback for sending RPMI message */
	rmb->channel.send_message = mpxy_mbox_send_message;
	/* Callback to get RPMI notifications */
	rmb->channel.get_notification_events = mpxy_mbox_get_notifications;

	/* No callback to switch events state data */
	rmb->channel.switch_eventsstate = NULL;

	/* RPMI Message Protocol ID */
	rmb->channel.attrs.msg_proto_id = SBI_MPXY_MSGPROTO_RPMI_ID;
	/* RPMI Message Protocol Version */
	rmb->channel.attrs.msg_proto_version =
		SBI_MPXY_MSGPROTO_VERSION(RPMI_MAJOR_VER, RPMI_MINOR_VER);

	/* RPMI supported max message data length(bytes), same for
	 * all service groups */
	rmb->channel.attrs.msg_data_maxlen = RPMI_MSG_DATA_SIZE;
	/* RPMI message send timeout(milliseconds)
	 * same for all service groups */
	rmb->channel.attrs.msg_send_timeout = RPMI_MSG_SEND_TIMEOUT;

	rmb->srv_data = (struct rpmi_service_data *)data->priv_data;
	rmb->chan = chan;

	/* Register RPXY service group */
	rc = sbi_mpxy_register_channel(&rmb->channel);
	if (rc) {
		mbox_controller_free_chan(chan);
		sbi_free(rmb);
		return rc;
	}

	return SBI_OK;
}

static struct rpmi_service_data clock_services[] = {
{
	.id = RPMI_CLOCK_SRV_GET_NUM_CLOCKS,
	.min_tx_len = 0,
	.max_tx_len = 0,
	.min_rx_len = sizeof(struct rpmi_clock_get_num_clocks_resp),
	.max_rx_len = sizeof(struct rpmi_clock_get_num_clocks_resp),
},
{
	.id = RPMI_CLOCK_SRV_GET_ATTRIBUTES,
	.min_tx_len = sizeof(struct rpmi_clock_get_attributes_req),
	.max_tx_len = sizeof(struct rpmi_clock_get_attributes_req),
	.min_rx_len = sizeof(struct rpmi_clock_get_attributes_resp),
	.max_rx_len = sizeof(struct rpmi_clock_get_attributes_resp),
},
{
	.id = RPMI_CLOCK_SRV_GET_SUPPORTED_RATES,
	.min_tx_len = sizeof(struct rpmi_clock_get_supported_rates_req),
	.max_tx_len = sizeof(struct rpmi_clock_get_supported_rates_req),
	.min_rx_len = sizeof(struct rpmi_clock_get_supported_rates_resp),
	.max_rx_len = -1U,
},
{
	.id = RPMI_CLOCK_SRV_SET_CONFIG,
	.min_tx_len = sizeof(struct rpmi_clock_set_config_req),
	.max_tx_len = sizeof(struct rpmi_clock_set_config_req),
	.min_rx_len = sizeof(struct rpmi_clock_set_config_resp),
	.max_rx_len = sizeof(struct rpmi_clock_set_config_resp),
},
{
	.id = RPMI_CLOCK_SRV_GET_CONFIG,
	.min_tx_len = sizeof(struct rpmi_clock_get_config_req),
	.max_tx_len = sizeof(struct rpmi_clock_get_config_req),
	.min_rx_len = sizeof(struct rpmi_clock_get_config_resp),
	.max_rx_len = sizeof(struct rpmi_clock_get_config_resp),
},
{
	.id = RPMI_CLOCK_SRV_SET_RATE,
	.min_tx_len = sizeof(struct rpmi_clock_set_rate_req),
	.max_tx_len = sizeof(struct rpmi_clock_set_rate_req),
	.min_rx_len = sizeof(struct rpmi_clock_set_rate_resp),
	.max_rx_len = sizeof(struct rpmi_clock_set_rate_resp),
},
{
	.id = RPMI_CLOCK_SRV_GET_RATE,
	.min_tx_len = sizeof(struct rpmi_clock_get_rate_req),
	.max_tx_len = sizeof(struct rpmi_clock_get_rate_req),
	.min_rx_len = sizeof(struct rpmi_clock_get_rate_resp),
	.max_rx_len = sizeof(struct rpmi_clock_get_rate_resp),
},
{	SERVICE_DATA_SENTINAL },
};

static struct mpxy_mbox_data clock_data = {
	.servicegrp_id = RPMI_SRVGRP_CLOCK,
	.notifications_support = 1,
	.priv_data = clock_services,
};

static const struct fdt_match mpxy_mbox_match[] = {
	{ .compatible = "riscv,rpmi-mpxy-clk", .data = &clock_data },
	{ },
};

struct fdt_mpxy fdt_mpxy_rpmi_mbox = {
	.match_table = mpxy_mbox_match,
	.init = mpxy_mbox_init,
};

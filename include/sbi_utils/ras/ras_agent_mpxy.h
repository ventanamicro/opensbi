/*
 * RISC-V RERI Registers Definitions
 *
 * Copyright (c) 2024 Ventana Micro Systems, Inc.
 *
 * Author(s):
 * Himanshu Chauhan <hchauhan@ventanamicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef __RAS_AGENT_MPXY_H
#define __RAS_AGENT_MPXY_H

/* RAS Agent Services on MPXY/RPMI */
#define RAS_GET_NUM_ERR_SRCS		0x1
#define RAS_GET_ERR_SRCS_ID_LIST	0x2
#define RAS_GET_ERR_SRC_DESC		0x3

/* Used to generate EINJ table */
#define RAS_EINJ_GET_NUM_INSTRUCTIONS	0x4
#define RAS_EINJ_GET_INSTRUCTION	0x5

/* Used during error injection/trigger */
#define RAS_EINJ_EXECUTE_OPERATION	0x6
#define RAS_EINJ_TRIGGER_ERROR		0x7

int ras_mpxy_init(const void *fdt, int nodeoff);

#endif

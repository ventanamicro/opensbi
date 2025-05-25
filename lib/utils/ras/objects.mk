#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2025 Ventana Micro Systems Inc.
#
# Authors:
#   Himanshu Chauhan <hchauhan@ventanamicro.com>
#

carray-fdt_early_drivers-$(CONFIG_FDT_RAS_RPMI) += fdt_ras_rpmi
libsbiutils-objs-$(CONFIG_FDT_RAS_RPMI) += ras/fdt_ras_rpmi.o

libsbiutils-objs-$(CONFIG_FDT_SBI_RAS_AGENT) += ras/ghes.o

carray-fdt_early_drivers-$(CONFIG_FDT_SBI_RAS_AGENT) += fdt_sbi_ras_agent
libsbiutils-objs-$(CONFIG_FDT_SBI_RAS_AGENT) += ras/fdt_ras_agent.o
libsbiutils-objs-$(CONFIG_FDT_SBI_RAS_AGENT) += ras/reri_drv.o
libsbiutils-objs-$(CONFIG_FDT_SBI_RAS_AGENT) += ras/ras_agent_mpxy.o
libsbiutils-objs-$(CONFIG_FDT_SBI_RAS_AGENT_EINJ) += ras/ras_agent_einj.o

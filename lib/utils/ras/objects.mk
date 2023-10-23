#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2023 Ventana Micro Systems Inc.
#
# Authors:
#   Himanshu Chauhan <hchauhan@ventanamicro.com>
#

libsbiutils-objs-$(CONFIG_FDT_RAS) += ras/fdt_ras.o
libsbiutils-objs-$(CONFIG_FDT_RAS) += ras/fdt_ras_drivers.o

carray-fdt_ras_drivers-$(CONFIG_FDT_RAS_RPMI) += fdt_ras_rpmi
libsbiutils-objs-$(CONFIG_FDT_RAS_RPMI) += ras/fdt_ras_rpmi.o

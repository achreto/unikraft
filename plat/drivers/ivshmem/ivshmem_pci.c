/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Sharan Santhanam <sharan.santhanam@neclab.eu>
 *
 * Copyright (c) 2018, NEC Europe Ltd., NEC Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <uk/config.h>
#include <uk/arch/types.h>
#include <errno.h>
#include <uk/alloc.h>
#include <uk/print.h>
#include <uk/plat/lcpu.h>
#include <uk/plat/irq.h>
#include <pci/pci_bus.h>
#include <virtio/virtio_config.h>
#include <virtio/virtio_bus.h>
#include <virtio/virtqueue.h>
#include <virtio/virtio_pci.h>
#include <stdio.h>

#define IVSHMEM_PCI_DEVICE_ID (0x1110)
#define VENDOR_QUMRANET_VIRTIO (0x1AF4)

// the device allocator
static struct uk_alloc *a;

/// XXX: some hacky way to expose the ivshmem base and size for now...
uint64_t ivshmem_base = 0;
uint64_t ivshmem_size = 0;

/**
 * The structure declares a pci device.
 */
struct ivshmem_pci_dev {
	struct pci_device *pdev;

	__u64 ivshmem_base;
	__u64 ivshmem_size;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Some PCI Utilities
////////////////////////////////////////////////////////////////////////////////////////////////////

#define PCI_BAR_IS_IO(base) (base & 0x1)
#define PCI_BAR_IS_64BIT(base) (((base >> 1) & 0x3) == 2)
#define PCI_BAR_MASK(base) (base & ~0xf)

#define PCI_CONF_READ(type, ret, a, s)                                         \
	do {                                                                   \
		uint32_t _conf_data;                                           \
		outl(PCI_CONFIG_ADDR, (a) | PCI_CONF_##s);                     \
		_conf_data = ((inl(PCI_CONFIG_DATA) >> PCI_CONF_##s##_SHFT)    \
			      & PCI_CONF_##s##_MASK);                          \
		*(ret) = (type)_conf_data;                                     \
	} while (0)

#define PCI_CONF_WRITE(type, val, a, s)                                        \
	do {                                                                   \
		outl(PCI_CONFIG_ADDR, (a) | PCI_CONF_##s);                     \
		outl(PCI_CONFIG_DATA, val);                                    \
	} while (0)

static void pci_read_bar(uint32_t config_addr, int i, uint32_t *bar)
{
	switch (i) {
	case 0:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR0);
		break;
	case 1:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR1);
		break;
	case 2:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR2);
		break;
	case 3:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR3);
		break;
	case 4:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR4);
		break;
	case 5:
		PCI_CONF_READ(uint32_t, bar, config_addr, BAR5);
		break;
	}
}

static void pci_write_bar(uint32_t config_addr, int i, uint32_t val)
{
	switch (i) {
	case 0:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR0);
		break;
	case 1:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR1);
		break;
	case 2:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR2);
		break;
	case 3:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR3);
		break;
	case 4:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR4);
		break;
	case 5:
		PCI_CONF_WRITE(uint32_t, val, config_addr, BAR5);
		break;
	}
}

static uint64_t pci_bar_size(uint64_t base)
{
	if (base == 0) {
		return 0;
	}

	for (int mask = 1;; mask <<= 1) {
		if (base & mask) {
			return mask;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Some PCI Utilities
////////////////////////////////////////////////////////////////////////////////////////////////////

static int ivshmem_pci_add_dev(struct pci_device *pci_dev)
{
	int rc = 0;

	struct ivshmem_pci_dev *spci_dev = NULL;

	UK_ASSERT(pci_dev != NULL);

	printf("#### ivshmem_pci_add_dev\n");

	spci_dev = uk_malloc(a, sizeof(*spci_dev));
	if (!spci_dev) {
		uk_pr_err("Failed to allocate ivshmem-pci device\n");
		return -ENOMEM;
	}

	uint32_t config_addr = (PCI_ENABLE_BIT)
			       | (pci_dev->addr.bus << PCI_BUS_SHIFT)
			       | (pci_dev->addr.devid << PCI_DEVICE_SHIFT);

	uint64_t bars[6] = {0};
	uint64_t barsizes[6] = {0};
	int cbar = 0;

	for (int i = 0; i < 6; i++) {
		uint32_t base0, base1, size0, size1;

		pci_read_bar(config_addr, i, &base0);

		// figure out the size
		pci_write_bar(config_addr, i, 0xffffffff);
		pci_read_bar(config_addr, i, &size0);
		pci_write_bar(config_addr, i, base0);

		if (size0 == 0)
			continue;

		if (PCI_BAR_IS_IO(base0)) {
			continue;
		}

		if (!PCI_BAR_IS_64BIT(base0)) {
			barsizes[cbar] = pci_bar_size(size0);
			bars[cbar++] = PCI_BAR_MASK(base0);
			continue;
		}

		pci_read_bar(config_addr, ++i, &base1);
		pci_write_bar(config_addr, i, 0xffffffff);
		pci_read_bar(config_addr, i, &size1);
		pci_write_bar(config_addr, i, base1);

		barsizes[cbar] =
		    pci_bar_size((uint64_t)size1 << 32 | PCI_BAR_MASK(size0));
		bars[cbar++] = (uint64_t)base1 << 32 | PCI_BAR_MASK(base0);
	}

	for (int i = 0; i < cbar; i++) {
		if (barsizes[i] > 0x1000) {
			spci_dev->ivshmem_base = bars[i];
			spci_dev->ivshmem_size = barsizes[i];
			break;
		}
	}

	printf("#### ivshmem_pci_add_dev: [%lx..%lx] (%zu kB)\n",
	       spci_dev->ivshmem_base,
	       spci_dev->ivshmem_base + spci_dev->ivshmem_size,
	       spci_dev->ivshmem_size / 1024);

	ivshmem_base = spci_dev->ivshmem_base;
	ivshmem_size = spci_dev->ivshmem_size;

	return 0;
}

static int ivshmem_pci_drv_init(struct uk_alloc *drv_allocator)
{
	/* driver initialization */
	if (!drv_allocator)
		return -EINVAL;

	a = drv_allocator;
	return 0;
}

static const struct pci_device_id ivshmem_pci_ids[] = {
    {PCI_DEVICE_ID(VENDOR_QUMRANET_VIRTIO, IVSHMEM_PCI_DEVICE_ID)},
    /* End of Driver List */
    {PCI_ANY_DEVICE_ID},
};

static struct pci_driver ivshmem_pci_drv = {.device_ids = ivshmem_pci_ids,
					    .init = ivshmem_pci_drv_init,
					    .add_dev = ivshmem_pci_add_dev};
PCI_REGISTER_DRIVER(&ivshmem_pci_drv);

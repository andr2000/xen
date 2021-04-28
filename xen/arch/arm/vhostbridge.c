/*
 * xen/arch/arm/vhostbridge.h
 * Copyright (c) 2021 EPAM Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Code is partially based on QEMU implementation of the PCI host bridge.
 */

#include <xen/pci.h>
#include <xen/sched.h>
#include <xen/vpci.h>

#include "vhostbridge.h"

/* Size of the standard PCIe config space: 4KB */
#define PCIE_CONFIG_SPACE_SIZE  0x1000

struct vhostbridge_priv {
    /* Physical host bridge we are emulating. */
    const struct pci_dev *pdev;
    /* Memory base register configuration. */
    const paddr_t addr_mem;
    const paddr_t sz_mem;
    /* Prefetchable memory base register configuration. */
    paddr_t addr_pref_mem;
    paddr_t sz_pref_mem;
    /* I/O base register configuration. */
    paddr_t addr_io_mem;
    paddr_t sz_io_mem;

    /* PCI configuration space. */
    uint8_t config[PCIE_CONFIG_SPACE_SIZE];
    struct vpci_header header[PCI_HEADER_BRIDGE_NR_BARS];
};

static uint32_t cfg_read(uint8_t *cfg, unsigned int reg, unsigned int size)
{
    switch (size)
    {
    case 1:
        return cfg[reg];
    case 2:
        return *((uint16_t *)(cfg + reg));
    case 4:
        return *((uint32_t *)(cfg + reg));
    default:
        break;
    }
    return ~0;
}

static void cfg_write(uint8_t *cfg, unsigned int reg, unsigned int size,
                      uint32_t data)
{
    switch (size)
    {
    case 1:
        cfg[reg] = (uint8_t)data;
        break;
    case 2:
        *((uint16_t *)(cfg + reg)) = (uint16_t)data;
        break;
    case 4:
        *((uint32_t *)(cfg + reg)) = data;
        break;
    default:
        break;
    }
}

#define PCI_CLASS_BRIDGE_PCI     0x0604

int vhostbridge_init(struct domain *d, const struct pci_dev *pdev,
               const paddr_t addr_mem, const paddr_t sz_mem,
               const paddr_t addr_pref_mem, const paddr_t sz_pref_mem,
               const paddr_t addr_io_mem, const paddr_t sz_io_mem)
{
    struct vhostbridge_priv *priv;

    if ( !pdev )
    {
        printk(XENLOG_G_ERR
               "d%d: vhostbridge: Can't find physical PCI host bridge\n",
               d->domain_id);
        return -EINVAL;
    }

    priv = xzalloc(struct vhostbridge_priv);
    if ( !priv )
        return -ENOMEM;

    d->vhostbridge_priv = priv;

    priv->pdev = pdev;

    cfg_write(priv->config, PCI_VENDOR_ID, 2, 0x1af4);
    cfg_write(priv->config, PCI_DEVICE_ID, 2, 0x1100);
    cfg_write(priv->config, PCI_STATUS, 2,
              PCI_STATUS_66MHZ | PCI_STATUS_FAST_BACK);
    cfg_write(priv->config, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);
    cfg_write(priv->config, PCI_HEADER_TYPE, 1, PCI_HEADER_TYPE_BRIDGE);

    cfg_write(priv->config, PCI_MEMORY_BASE, 2, addr_mem >> 16);
    cfg_write(priv->config, PCI_MEMORY_LIMIT, 2, (addr_mem + sz_mem - 1) >> 16);
    printk("PCI_MEMORY BASE %x LIMIT %x\n",
           cfg_read(priv->config, PCI_MEMORY_BASE, 2),
           cfg_read(priv->config, PCI_MEMORY_LIMIT, 2));

    cfg_write(priv->config, PCI_PREF_MEMORY_BASE, 2, addr_pref_mem);
    cfg_write(priv->config, PCI_PREF_MEMORY_LIMIT, 2, (addr_pref_mem +
              sz_pref_mem - 1));
    cfg_write(priv->config, PCI_PREF_BASE_UPPER32, 2, addr_pref_mem >> 32);
    cfg_write(priv->config, PCI_PREF_LIMIT_UPPER32, 2, (addr_pref_mem +
              sz_pref_mem - 1) >> 32);

    return 0;
}

void vhostbridge_fini(struct domain *d)
{
    if ( d->vhostbridge_priv )
    {
        xfree(d->vhostbridge_priv);
        d->vhostbridge_priv = NULL;
    }
}

uint32_t vhostbridge_read(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                    unsigned int size)
{
    uint8_t *cfg = ((struct vhostbridge_priv *)d->vhostbridge_priv)->config;
    uint32_t data = ~0;

    data = cfg_read(cfg, reg, size);
    switch (reg)
    {
        /*
         * The Base Address registers are optional registers used to map
         * internal (device-specific) registers into Memory or I/O Spaces.
         * We do not emulate any, so return as 0.
         */
        case PCI_BASE_ADDRESS_0:
            /* fallthrough */
        case PCI_BASE_ADDRESS_1:
            data = 0;
            break;
        /* No expansion ROM supported. */
        case PCI_ROM_ADDRESS1:
            data = 0;
            break;
        case PCI_MEMORY_BASE:
            printk("%s PCI_MEMORY_BASE\n", __func__);
            break;
        case PCI_MEMORY_LIMIT:
            printk("%s PCI_MEMORY_LIMIT\n", __func__);
            break;
        default:
            break;
    }
    printk("%s %pp reg %x sz %d val %08x\n", __func__, &sbdf, reg, size, data);
    return data;
}

void vhostbridge_write(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                 unsigned int size, uint32_t data)
{
    uint8_t *cfg = ((struct vhostbridge_priv *)d->vhostbridge_priv)->config;

    /* TODO: Skip writing to read-only registers. */
    cfg_write(cfg, reg, size, data);
    switch (reg)
    {
        case PCI_MEMORY_BASE:
            printk("%s PCI_MEMORY_BASE\n", __func__);
            break;
        case PCI_MEMORY_LIMIT:
            printk("%s PCI_MEMORY_LIMIT\n", __func__);
            break;
        default:
            break;
    }
    printk("%s %pp reg %x sz %d val %08x\n", __func__, &sbdf, reg, size, data);
}

/*
 * xen/arch/arm/vecam.h
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

#include "vecam.h"

/* Size of the standard PCIe config space: 4KB */
#define PCIE_CONFIG_SPACE_SIZE  0x1000

struct vecam_priv {
    /* Physical host bridge we are emulating. */
    const struct pci_dev *pdev;
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

int vecam_init(struct domain *d, const struct pci_dev *pdev)
{
    struct vecam_priv *priv;

    if ( !pdev )
    {
        printk(XENLOG_G_ERR
               "d%d: vECAM: Can't find physical PCI host bridge\n",
               d->domain_id);
        return -EINVAL;
    }

    priv = xzalloc(struct vecam_priv);
    if ( !priv )
        return -ENOMEM;

    d->vecam_priv = priv;

    priv->pdev = pdev;

    cfg_write(priv->config, PCI_VENDOR_ID, 2, 0x1af4);
    cfg_write(priv->config, PCI_DEVICE_ID, 2, 0x1100);
    cfg_write(priv->config, PCI_STATUS, 2,
              PCI_STATUS_66MHZ | PCI_STATUS_FAST_BACK);
    cfg_write(priv->config, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);
    cfg_write(priv->config, PCI_HEADER_TYPE, 1, PCI_HEADER_TYPE_BRIDGE);

    return 0;
}

void vecam_fini(struct domain *d)
{
    if ( d->vecam_priv )
    {
        xfree(d->vecam_priv);
        d->vecam_priv = NULL;
    }
}

uint32_t vecam_read(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                    unsigned int size)
{
    uint8_t *cfg = ((struct vecam_priv *)d->vecam_priv)->config;
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
        case PCI_PREF_MEMORY_BASE:
#define PCI_PREF_MEMORY_LIMIT	0x26
        default:
            break;
    }
    printk("%s %pp reg %x sz %d val %08x\n", __func__, &sbdf, reg, size, data);
    return data;
}

void vecam_write(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                 unsigned int size, uint32_t data)
{
    uint8_t *cfg = ((struct vecam_priv *)d->vecam_priv)->config;

    printk("%s %pp reg %x sz %d val %08x\n", __func__, &sbdf, reg, size, data);
    /* TODO: Skip writing to read-only registers. */
    cfg_write(cfg, reg, size, data);
}

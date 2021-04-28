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

#include "vecam.h"

/* Size of the standard PCIe config space: 4KB */
#define PCIE_CONFIG_SPACE_SIZE  0x1000

struct vecam_priv {
    /* PCI configuration space. */
    uint8_t config[PCIE_CONFIG_SPACE_SIZE];
};

static void cfg_write16(uint8_t *cfg, int ofs, uint16_t val)
{
    *((uint16_t *)(cfg + ofs)) = val;
}

static uint16_t cfg_read16(uint8_t *cfg, int ofs)
{
    return *((uint16_t *)(cfg + ofs));
}

static void cfg_write32(uint8_t *cfg, int ofs, uint32_t val)
{
    *((uint32_t *)(cfg + ofs)) = val;
}

static uint32_t cfg_read32(uint8_t *cfg, int ofs)
{
    return *((uint32_t *)(cfg + ofs));
}

#define PCI_CLASS_BRIDGE_PCI     0x0604

int vecam_init(struct domain *d)
{
    uint8_t *cfg;

    d->vecam_priv = xzalloc(struct vecam_priv);
    if ( !d->vecam_priv )
        return -ENOMEM;

    cfg = ((struct vecam_priv *)d->vecam_priv)->config;

    cfg_write16(cfg, PCI_VENDOR_ID, 0x1af4);
    cfg_write16(cfg, PCI_DEVICE_ID, 0x1100);
    cfg_write16(cfg, PCI_STATUS, PCI_STATUS_66MHZ | PCI_STATUS_FAST_BACK);
    cfg_write16(cfg, PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_PCI);
    cfg[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_BRIDGE;
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

    printk("%s %pp reg %x sz %d\n", __func__, &sbdf, reg, size);
    switch (size)
    {
    case 1:
        return cfg[reg];
    case 2:
        return cfg_read16(cfg, reg);
    case 4:
        return cfg_read32(cfg, reg);
    default:
        break;
    }
    return ~(uint32_t)0;
}

void vecam_write(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                 unsigned int size, uint32_t data)
{
    uint8_t *cfg = ((struct vecam_priv *)d->vecam_priv)->config;

    printk("%s %pp reg %x sz %d\n", __func__, &sbdf, reg, size);
    switch (size)
    {
    case 1:
        cfg[reg] = (uint8_t)data;
    case 2:
        cfg_write16(cfg, reg, data);
    case 4:
        cfg_write32(cfg, reg, data);
    default:
        break;
    }
}

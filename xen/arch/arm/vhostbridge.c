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

#include "pci-bridge-emul.h"
#include "vhostbridge.h"

struct vhostbridge_priv {
    /* Physical host bridge we are emulating. */
    const struct pci_dev *pdev;
    struct pci_bridge_emul bridge;

    /* Memory base register configuration. */
    const paddr_t addr_mem;
    const paddr_t sz_mem;
    /* Prefetchable memory base register configuration. */
    paddr_t addr_pref_mem;
    paddr_t sz_pref_mem;
    /* I/O base register configuration. */
    paddr_t addr_io_mem;
    paddr_t sz_io_mem;
};

#if 0
struct reg_name {
  unsigned int cap;
  unsigned int offset;
  unsigned int width;
  const char *name;
};

static const struct reg_name pci_reg_names[] = {
  {       0, 0x00, 2, "VENDOR_ID" },
  {       0, 0x02, 2, "DEVICE_ID" },
  {       0, 0x04, 2, "COMMAND" },
  {       0, 0x06, 2, "STATUS" },
  {       0, 0x08, 1, "REVISION" },
  {       0, 0x09, 1, "CLASS_PROG" },
  {       0, 0x0a, 2, "CLASS_DEVICE" },
  {       0, 0x0c, 1, "CACHE_LINE_SIZE" },
  {       0, 0x0d, 1, "LATENCY_TIMER" },
  {       0, 0x0e, 1, "HEADER_TYPE" },
  {       0, 0x0f, 1, "BIST" },
  {       0, 0x10, 4, "BASE_ADDRESS_0" },
  {       0, 0x14, 4, "BASE_ADDRESS_1" },

#ifdef type0
  {       0, 0x18, 4, "BASE_ADDRESS_2" },
  {       0, 0x1c, 4, "BASE_ADDRESS_3" },
  {       0, 0x20, 4, "BASE_ADDRESS_4" },
  {       0, 0x24, 4, "BASE_ADDRESS_5" },
  {       0, 0x28, 4, "CARDBUS_CIS" },
  {       0, 0x2c, 2, "SUBSYSTEM_VENDOR_ID" },
  {       0, 0x2e, 2, "SUBSYSTEM_ID" },
  {       0, 0x30, 4, "ROM_ADDRESS" },
  {       0, 0x3c, 1, "INTERRUPT_LINE" },
  {       0, 0x3d, 1, "INTERRUPT_PIN" },
  {       0, 0x3e, 1, "MIN_GNT" },
  {       0, 0x3f, 1, "MAX_LAT" },
#endif
  {       0, 0x18, 1, "PRIMARY_BUS" },
  {       0, 0x19, 1, "SECONDARY_BUS" },
  {       0, 0x1a, 1, "SUBORDINATE_BUS" },
  {       0, 0x1b, 1, "SEC_LATENCY_TIMER" },
  {       0, 0x1c, 1, "IO_BASE" },
  {       0, 0x1d, 1, "IO_LIMIT" },
  {       0, 0x1e, 2, "SEC_STATUS" },
  {       0, 0x20, 2, "MEMORY_BASE" },
  {       0, 0x22, 2, "MEMORY_LIMIT" },
  {       0, 0x24, 2, "PREF_MEMORY_BASE" },
  {       0, 0x26, 2, "PREF_MEMORY_LIMIT" },
  {       0, 0x28, 4, "PREF_BASE_UPPER32" },
  {       0, 0x2c, 4, "PREF_LIMIT_UPPER32" },
  {       0, 0x30, 2, "IO_BASE_UPPER16" },
  {       0, 0x32, 2, "IO_LIMIT_UPPER16" },
  {       0, 0x34, 1, "CAPABILITIES_POINTER" },
  {       0, 0x38, 4, "BRIDGE_ROM_ADDRESS" },
  {       0, 0x3c, 1, "INTERRUPT_LINE" },
  {       0, 0x3d, 1, "INTERRUPT_PIN" },
  {       0, 0x3e, 2, "BRIDGE_CONTROL" },
#ifdef type2
  {       0, 0x10, 4, "CB_CARDBUS_BASE" },
  {       0, 0x14, 2, "CB_CAPABILITIES" },
  {       0, 0x16, 2, "CB_SEC_STATUS" },
  {       0, 0x18, 1, "CB_BUS_NUMBER" },
  {       0, 0x19, 1, "CB_CARDBUS_NUMBER" },
  {       0, 0x1a, 1, "CB_SUBORDINATE_BUS" },
  {       0, 0x1b, 1, "CB_CARDBUS_LATENCY" },
  {       0, 0x1c, 4, "CB_MEMORY_BASE_0" },
  {       0, 0x20, 4, "CB_MEMORY_LIMIT_0" },
  {       0, 0x24, 4, "CB_MEMORY_BASE_1" },
  {       0, 0x28, 4, "CB_MEMORY_LIMIT_1" },
  {       0, 0x2c, 2, "CB_IO_BASE_0" },
  {       0, 0x2e, 2, "CB_IO_BASE_0_HI" },
  {       0, 0x30, 2, "CB_IO_LIMIT_0" },
  {       0, 0x32, 2, "CB_IO_LIMIT_0_HI" },
  {       0, 0x34, 2, "CB_IO_BASE_1" },
  {       0, 0x36, 2, "CB_IO_BASE_1_HI" },
  {       0, 0x38, 2, "CB_IO_LIMIT_1" },
  {       0, 0x3a, 2, "CB_IO_LIMIT_1_HI" },
  {       0, 0x40, 2, "CB_SUBSYSTEM_VENDOR_ID" },
  {       0, 0x42, 2, "CB_SUBSYSTEM_ID" },
  {       0, 0x44, 4, "CB_LEGACY_MODE_BASE" },
#endif
  { 0x10001,    0, 0, "CAP_PM" },
  { 0x10002,    0, 0, "CAP_AGP" },
  { 0x10003,    0, 0, "CAP_VPD" },
  { 0x10004,    0, 0, "CAP_SLOTID" },
  { 0x10005,    0, 0, "CAP_MSI" },
  { 0x10006,    0, 0, "CAP_CHSWP" },
  { 0x10007,    0, 0, "CAP_PCIX" },
  { 0x10008,    0, 0, "CAP_HT" },
  { 0x10009,    0, 0, "CAP_VNDR" },
  { 0x1000a,    0, 0, "CAP_DBG" },
  { 0x1000b,    0, 0, "CAP_CCRC" },
  { 0x1000c,    0, 0, "CAP_HOTPLUG" },
  { 0x1000d,    0, 0, "CAP_SSVID" },
  { 0x1000e,    0, 0, "CAP_AGP3" },
  { 0x1000f,    0, 0, "CAP_SECURE" },
  { 0x10010,    0, 0, "CAP_EXP" },
  { 0x10011,    0, 0, "CAP_MSIX" },
  { 0x10012,    0, 0, "CAP_SATA" },
  { 0x10013,    0, 0, "CAP_AF" },
  { 0x10014,    0, 0, "CAP_EA" },
  { 0x20001,	0, 0, "ECAP_AER" },
  { 0x20002,	0, 0, "ECAP_VC" },
  { 0x20003,	0, 0, "ECAP_DSN" },
  { 0x20004,	0, 0, "ECAP_PB" },
  { 0x20005,	0, 0, "ECAP_RCLINK" },
  { 0x20006,	0, 0, "ECAP_RCILINK" },
  { 0x20007,	0, 0, "ECAP_RCEC" },
  { 0x20008,	0, 0, "ECAP_MFVC" },
  { 0x20009,	0, 0, "ECAP_VC2" },
  { 0x2000a,	0, 0, "ECAP_RBCB" },
  { 0x2000b,	0, 0, "ECAP_VNDR" },
  { 0x2000d,	0, 0, "ECAP_ACS" },
  { 0x2000e,	0, 0, "ECAP_ARI" },
  { 0x2000f,	0, 0, "ECAP_ATS" },
  { 0x20010,	0, 0, "ECAP_SRIOV" },
  { 0x20011,	0, 0, "ECAP_MRIOV" },
  { 0x20012,	0, 0, "ECAP_MCAST" },
  { 0x20013,	0, 0, "ECAP_PRI" },
  { 0x20015,	0, 0, "ECAP_REBAR" },
  { 0x20016,	0, 0, "ECAP_DPA" },
  { 0x20017,	0, 0, "ECAP_TPH" },
  { 0x20018,	0, 0, "ECAP_LTR" },
  { 0x20019,	0, 0, "ECAP_SECPCI" },
  { 0x2001a,	0, 0, "ECAP_PMUX" },
  { 0x2001b,	0, 0, "ECAP_PASID" },
  { 0x2001c,	0, 0, "ECAP_LNR" },
  { 0x2001d,	0, 0, "ECAP_DPC" },
  { 0x2001e,	0, 0, "ECAP_L1PM" },
  { 0x2001f,	0, 0, "ECAP_PTM" },
  { 0x20020,	0, 0, "ECAP_M_PCIE" },
  { 0x20021,	0, 0, "ECAP_FRS" },
  { 0x20022,	0, 0, "ECAP_RTR" },
  { 0x20023,	0, 0, "ECAP_DVSEC" },
  { 0x20024,	0, 0, "ECAP_VF_REBAR" },
  { 0x20025,	0, 0, "ECAP_DLNK" },
  { 0x20026,	0, 0, "ECAP_16GT" },
  { 0x20027,	0, 0, "ECAP_LMR" },
  { 0x20028,	0, 0, "ECAP_HIER_ID" },
  { 0x20029,	0, 0, "ECAP_NPEM" },
  {       0,    0, 0, NULL }
};

static const char *get_reg_name(int reg)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(pci_reg_names); i++)
    if (reg == pci_reg_names[i].offset)
        return pci_reg_names[i].name;
    return "N/A";
}
#endif
static pci_bridge_emul_read_status_t
vhostbridge_emul_ops_conf_read(struct pci_bridge_emul *bridge,
                               int reg, u32 *value)
{
//    struct vhostbridge_priv *priv = bridge->data;

    return PCI_BRIDGE_EMUL_NOT_HANDLED;
}

static void
vhostbridge_emul_ops_conf_write(struct pci_bridge_emul *bridge,
                                int reg, u32 old, u32 new, u32 mask)
{
//    struct vhostbridge_priv *priv = bridge->data;

    switch (reg) {
    default:
        break;
    }
}

static struct pci_bridge_emul_ops vhostbridge_emul_ops = {
    .read_pcie = vhostbridge_emul_ops_conf_read,
    .write_pcie = vhostbridge_emul_ops_conf_write,
};

int vhostbridge_init(struct domain *d, const struct pci_dev *pdev,
               const paddr_t addr_mem, const paddr_t sz_mem,
               const paddr_t addr_pref_mem, const paddr_t sz_pref_mem,
               const paddr_t addr_io_mem, const paddr_t sz_io_mem)
{
    struct vhostbridge_priv *priv;
    struct pci_bridge_emul *bridge;

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

    bridge = &priv->bridge;

    bridge->conf.vendor = 0x1af4;
    bridge->conf.device = 0x1100;
    bridge->conf.class_revision = 0;

    /* We support 32 bits I/O addressing */
    bridge->conf.iobase = PCI_IO_RANGE_TYPE_32;
    bridge->conf.iolimit = PCI_IO_RANGE_TYPE_32;

//    bridge->conf.membase = 0x2000;
//    bridge->conf.memlimit = 0x2ff0;

    /* Support 64 bits memory pref */
    bridge->conf.pref_mem_base = cpu_to_le16(PCI_PREF_RANGE_TYPE_64);
    bridge->conf.pref_mem_limit = cpu_to_le16(PCI_PREF_RANGE_TYPE_64);

    bridge->has_pcie = true;
    bridge->data = priv;
    bridge->ops = &vhostbridge_emul_ops;

    /* Support interrupt A for MSI feature */
    bridge->conf.intpin = 1;

    pci_bridge_emul_init(bridge, 0);
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
    struct vhostbridge_priv *priv = d->vhostbridge_priv;
    struct pci_bridge_emul *bridge = &priv->bridge;
    uint32_t data;

    if ( pci_bridge_emul_conf_read(bridge, reg, size, &data) )
        data = ~0;
#if 0
    printk("%s %pp %s reg %x sz %d val %08x\n", __func__, &sbdf,
           get_reg_name(reg), reg, size, data);
#endif
    return data;
}

void vhostbridge_write(struct domain *d, pci_sbdf_t sbdf, unsigned int reg,
                 unsigned int size, uint32_t data)
{
    struct vhostbridge_priv *priv = d->vhostbridge_priv;
    struct pci_bridge_emul *bridge = &priv->bridge;

    if ( pci_bridge_emul_conf_write(bridge, reg, size, data) )
        data = ~0;
#if 0
    printk("%s %pp %s reg %x sz %d val %08x\n", __func__, &sbdf,
           get_reg_name(reg), reg, size, data);
#endif
}

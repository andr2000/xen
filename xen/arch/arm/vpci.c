/*
 * xen/arch/arm/vpci.c
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
 */
#include <xen/sched.h>

#include <asm/mmio.h>

#define REGISTER_OFFSET(addr)  ( (addr) & 0x00000fff)

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
struct vpci_mmio_priv {
    /*
     * Set to true if the MMIO handlers were set up for the emulated
     * ECAM host PCI bridge.
     */
    bool is_virt_ecam;
};
#endif

/* Do some sanity checks. */
static bool vpci_mmio_access_allowed(unsigned int reg, unsigned int len)
{
    /* Check access size. */
    if ( len > 8 )
        return false;

    /* Check that access is size aligned. */
    if ( (reg & (len - 1)) )
        return false;

    return true;
}

static int vpci_mmio_read(struct vcpu *v, mmio_info_t *info,
                          register_t *r, void *p)
{
    unsigned int reg;
    pci_sbdf_t sbdf;
    unsigned long data = ~0UL;
    unsigned int size = 1U << info->dabt.size;
#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    struct vpci_mmio_priv *priv = (struct vpci_mmio_priv *)p;
#endif

    sbdf.sbdf = MMCFG_BDF(info->gpa);
    reg = REGISTER_OFFSET(info->gpa);

    if ( !vpci_mmio_access_allowed(reg, size) )
        return 0;

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    /*
     * For the passed through devices we need to map their virtual SBDF
     * to the physical PCI device being passed through.
     */
    if ( priv->is_virt_ecam &&
         !vpci_translate_virtual_device(v->domain, &sbdf) )
            return 1;
#endif

    data = vpci_read(sbdf, reg, min(4u, size));
    if ( size == 8 )
        data |= (uint64_t)vpci_read(sbdf, reg + 4, 4) << 32;

    *r = data;

    return 1;
}

static int vpci_mmio_write(struct vcpu *v, mmio_info_t *info,
                           register_t r, void *p)
{
    unsigned int reg;
    pci_sbdf_t sbdf;
    unsigned long data = r;
    unsigned int size = 1U << info->dabt.size;
#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    struct vpci_mmio_priv *priv = (struct vpci_mmio_priv *)p;
#endif

    sbdf.sbdf = MMCFG_BDF(info->gpa);
    reg = REGISTER_OFFSET(info->gpa);

    if ( !vpci_mmio_access_allowed(reg, size) )
        return 0;

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    /*
     * For the passed through devices we need to map their virtual SBDF
     * to the physical PCI device being passed through.
     */
    if ( priv->is_virt_ecam &&
         !vpci_translate_virtual_device(v->domain, &sbdf) )
            return 1;
#endif

    vpci_write(sbdf, reg, min(4u, size), data);
    if ( size == 8 )
        vpci_write(sbdf, reg + 4, 4, data >> 32);

    return 1;
}

static const struct mmio_handler_ops vpci_mmio_handler = {
    .read  = vpci_mmio_read,
    .write = vpci_mmio_write,
};

/*
 * There are three  originators for the PCI configuration space access:
 * 1. The domain that owns physical host bridge: MMIO handlers are
 *    there so we can update vPCI register handlers with the values
 *    written by the hardware domain, e.g. physical view of the registers/
 *    configuration space.
 * 2. Guest access to the passed through PCI devices: we need to properly
 *    map virtual bus topology to the physical one, e.g. pass the configuration
 *    space access to the corresponding physical devices.
 * 3. Emulated host PCI bridge access. It doesn't exist in the physical
 *    topology, e.g. it can't be mapped to some physical host bridge.
 *    So, all access to the host bridge itself needs to be trapped and
 *    emulated.
 */
static int vpci_setup_mmio_handler(struct domain *d,
                                   struct pci_host_bridge *bridge)
{
#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
    struct vpci_mmio_priv *priv;

    priv = xzalloc(struct vpci_mmio_priv);
    if ( !priv )
        return -ENOMEM;

    priv->is_virt_ecam = !is_hardware_domain(d);
#else
    void *priv = NULL;
#endif

    if ( is_hardware_domain(d) )
    {
        struct pci_config_window *cfg = bridge->cfg;

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
        bridge->mmio_priv = priv;
#endif
        register_mmio_handler(d, &vpci_mmio_handler,
                              cfg->phys_addr, cfg->size,
                              priv);
    }
    else
    {
#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
        d->vpci_mmio_priv = priv;
#endif
        /* Guest domains use what is programmed in their device tree. */
        register_mmio_handler(d, &vpci_mmio_handler,
                              GUEST_VPCI_ECAM_BASE, GUEST_VPCI_ECAM_SIZE,
                              priv);
    }
    return 0;
}

int domain_vpci_init(struct domain *d)
{
    if ( !has_vpci(d) )
        return 0;

    return pci_host_iterate_bridges(d, vpci_setup_mmio_handler);
}

#ifdef CONFIG_HAS_VPCI_GUEST_SUPPORT
static int domain_vpci_free_cb(struct domain *d,
                               struct pci_host_bridge *bridge)
{
    if ( is_hardware_domain(d) )
        XFREE(bridge->mmio_priv);
    else
        XFREE(d->vpci_mmio_priv);
    return 0;
}

void domain_vpci_free(struct domain *d)
{
    if ( !has_vpci(d) )
        return;

    pci_host_iterate_bridges(d, domain_vpci_free_cb);
}
#endif

int domain_vpci_get_num_mmio_handlers(struct domain *d)
{
    int count;

    if ( is_hardware_domain(d) )
        /* For each PCI host bridge's configuration space. */
        count = pci_host_get_num_bridges();
    else
        /*
         * VPCI_MSIX_MEM_NUM handlers for MSI-X tables per each PCI device
         * being passed through. Maximum number of supported devices
         * is 32 as virtual bus topology emulates the devices as embedded
         * endpoints.
         * +1 for a single emulated host bridge's configuration space.
         */
        count = VPCI_MSIX_MEM_NUM * 32 + 1;

    return count;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */


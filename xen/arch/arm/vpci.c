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

    sbdf.sbdf = MMCFG_BDF(info->gpa);
    reg = REGISTER_OFFSET(info->gpa);

    if ( !vpci_mmio_access_allowed(reg, size) )
        return 0;

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

    sbdf.sbdf = MMCFG_BDF(info->gpa);
    reg = REGISTER_OFFSET(info->gpa);

    if ( !vpci_mmio_access_allowed(reg, size) )
        return 0;

    vpci_write(sbdf, reg, min(4u, size), data);
    if ( size == 8 )
        vpci_write(sbdf, reg + 4, 4, data >> 32);

    return 1;
}

static const struct mmio_handler_ops vpci_mmio_handler = {
    .read  = vpci_mmio_read,
    .write = vpci_mmio_write,
};

int domain_vpci_init(struct domain *d)
{
    if ( !has_vpci(d) )
        return 0;

    register_mmio_handler(d, &vpci_mmio_handler,
                          GUEST_VPCI_ECAM_BASE, GUEST_VPCI_ECAM_SIZE, NULL);

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */


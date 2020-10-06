/******************************************************************************
 * Arch-specific physdev.c
 *
 * Copyright (c) 2012, Citrix Systems
 */

#include <xen/types.h>
#include <xen/lib.h>
#include <xen/errno.h>
#include <xen/sched.h>
#include <asm/hypercall.h>
#include <xen/guest_access.h>
#include <xsm/xsm.h>

int do_physdev_op(int cmd, XEN_GUEST_HANDLE_PARAM(void) arg)
{
    int ret = 0;

    switch ( cmd )
    {
#ifdef CONFIG_HAS_PCI
        case PHYSDEVOP_pci_device_add:
            {
                struct physdev_pci_device_add add;
                struct pci_dev_info pdev_info;
                nodeid_t node = NUMA_NO_NODE;

                ret = -EFAULT;
                if ( copy_from_guest(&add, arg, 1) != 0 )
                    break;

                pdev_info.is_extfn = !!(add.flags & XEN_PCI_DEV_EXTFN);
                if ( add.flags & XEN_PCI_DEV_VIRTFN )
                {
                    pdev_info.is_virtfn = 1;
                    pdev_info.physfn.bus = add.physfn.bus;
                    pdev_info.physfn.devfn = add.physfn.devfn;
                }
                else
                    pdev_info.is_virtfn = 0;

                ret = pci_add_device(add.seg, add.bus, add.devfn,
                                &pdev_info, node);

                break;
            }
            case PHYSDEVOP_pci_device_set_resources:
            {
                struct physdev_pci_device_resources res;
                struct pci_mmio_resource pci_res[PCI_NUM_RESOURCES];
                int i, j;

                ret = -EFAULT;
                if ( copy_from_guest(&res, arg, 1) != 0 )
                    break;

                memset(pci_res, 0, sizeof(pci_res));
                j = 0;
                for (i = 0; i < PCI_NUM_RESOURCES; i++)
                {
                    /* TODO: there are 2 places which already define the
                     * below IORESORCE_XXX: tools and device_tree.c.
                     * Need to move it somewhere.
                     */
#define IORESOURCE_IO           0x00000100      /* PCI/ISA I/O ports */
#define IORESOURCE_MEM          0x00000200

                    if ((res.resource[i].flags &
                         (IORESOURCE_IO | IORESOURCE_MEM)) == 0)
                            continue;

                    if ( !res.resource[i].start )
                        continue;

                    if ( !res.resource[i].length)
                        continue;

                    pci_res[j].start = res.resource[i].start;
                    pci_res[j].length = res.resource[i].length;
                    pci_res[j].flags = res.resource[i].flags;
                    j++;
                }

                ret = pci_set_device_resources(res.seg, res.bus, res.devfn,
                                               res.irq, pci_res);
                break;
            }

            case PHYSDEVOP_pci_device_get_resources:
            {
                struct physdev_pci_device_resources res;
                struct pci_mmio_resource pci_res[PCI_NUM_RESOURCES];
                int i;

                ret = -EFAULT;
                if ( copy_from_guest(&res, arg, 1) != 0 )
                    break;

                ret = pci_get_device_resources(res.seg, res.bus, res.devfn,
                                               &res.irq, pci_res);
                if ( ret )
                    break;

                for (i = 0; i < PCI_NUM_RESOURCES; i++)
                {
                    res.resource[i].start = pci_res[i].start;
                    res.resource[i].length = pci_res[i].length;
                    res.resource[i].flags = pci_res[i].flags;
                }

                if ( copy_to_guest(arg, &res, 1) != 0 )
                    ret = -EFAULT;

                break;
            }
#endif
        default:
            gdprintk(XENLOG_DEBUG, "PHYSDEVOP cmd=%d: not implemented\n", cmd);
            ret = -ENOSYS;
    }

    return ret;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

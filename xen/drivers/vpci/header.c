/*
 * Generic functionality for handling accesses to the PCI header from the
 * configuration space.
 *
 * Copyright (C) 2017 Citrix Systems R&D
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <xen/sched.h>
#include <xen/softirq.h>
#include <xen/vpci.h>

#include <asm/event.h>
#include <asm/p2m.h>

#define MAPPABLE_BAR(x)                                                 \
    ((x)->type == VPCI_BAR_MEM32 || (x)->type == VPCI_BAR_MEM64_LO ||   \
     (x)->type == VPCI_BAR_ROM)

struct map_data {
    struct domain *d;
    bool map;
    struct pci_dev *pdev;
};

static struct vpci_header *get_vpci_header(struct domain *d,
                                           const struct pci_dev *pdev);

static struct vpci_header *get_hwdom_vpci_header(const struct pci_dev *pdev)
{
    if ( unlikely(list_empty(&pdev->vpci->headers)) )
        return get_vpci_header(hardware_domain, pdev);

    /* hwdom's header is always the very first entry. */
    return list_first_entry(&pdev->vpci->headers, struct vpci_header, node);
}

static struct vpci_header *get_vpci_header(struct domain *d,
                                           const struct pci_dev *pdev)
{
    struct list_head *prev;
    struct vpci_header *header;
    struct vpci *vpci = pdev->vpci;

    list_for_each( prev, &vpci->headers )
    {
        struct vpci_header *this = list_entry(prev, struct vpci_header, node);

        if ( this->domain_id == d->domain_id )
            return this;
    }
    printk(XENLOG_DEBUG "--------------------------------------" \
           "Adding new vPCI BAR headers for domain %d: " PRI_pci" \n",
           d->domain_id, pdev->sbdf.seg, pdev->sbdf.bus,
           pdev->sbdf.dev, pdev->sbdf.fn);
    header = xzalloc(struct vpci_header);
    if ( !header )
    {
        printk(XENLOG_ERR
               "Failed to add new vPCI BAR headers for domain %d: " PRI_pci" \n",
               d->domain_id, pdev->sbdf.seg, pdev->sbdf.bus,
               pdev->sbdf.dev, pdev->sbdf.fn);
        return NULL;
    }

    if ( !is_hardware_domain(d) )
    {
        struct vpci_header *hwdom_header = get_hwdom_vpci_header(pdev);
#ifdef CONFIG_ARM
        /*
         * Non-ECAM host bridges in hwdom go directly to PCI
         * config space, not through vpci. Thus hwdom's vpci BARs are
         * never updated.
         */
        pci_host_bridge_update_bar_header(pdev, hwdom_header);
#endif
        /* Make a copy of the hwdom's BARs as the initial state for vBARs. */
        memcpy(header, hwdom_header, sizeof(*header));
    }

    header->domain_id = d->domain_id;
    list_add_tail(&header->node, &vpci->headers);
    return header;
}

static struct vpci_bar *get_vpci_bar(struct domain *d,
                                     const struct pci_dev *pdev,
                                     int bar_idx)
{
    struct vpci_header *vheader;

    vheader = get_vpci_header(d, pdev);
    if ( !vheader )
        return NULL;

    return &vheader->bars[bar_idx];
}

static int map_range(unsigned long s, unsigned long e, void *data,
                     unsigned long *c)
{
    const struct map_data *map = data;
    mfn_t mfn;
    int rc, bar_idx;
    struct vpci_header *header = get_hwdom_vpci_header(map->pdev);

    bar_idx = s & ~PCI_BASE_ADDRESS_MEM_MASK;
    mfn = _mfn(PFN_DOWN(header->bars[bar_idx].addr));
    for ( ; ; )
    {
        unsigned long size = e - s + 1;

        /*
         * ARM TODOs:
         * - On ARM whether the memory is prefetchable or not should be passed
         *   to map_mmio_regions in order to decide which memory attributes
         *   should be used.
         *
         * - {un}map_mmio_regions doesn't support preemption.
         */
        rc = map->map ? map_mmio_regions(map->d, _gfn(PFN_DOWN(s)),
                                         PFN_UP(size), mfn)
                      : unmap_mmio_regions(map->d, _gfn(PFN_DOWN(s)),
                                           PFN_UP(size), mfn);
        if ( rc == 0 )
        {
            *c += size;
            break;
        }
        if ( rc < 0 )
        {
            printk(XENLOG_G_WARNING
                   "Failed to %smap [%lx, %lx] for d%d: %d\n",
                   map->map ? "" : "un", PFN_DOWN(s), PFN_DOWN(e),
                   map->d->domain_id, rc);
            break;
        }
        /*
         * Range set is setup with memory addresses, but we map/unmap with
         * frame numbers. As we pass BAR index in the lower bits of the
         * address we can't update the partially consumed size by adding
         * rc << PAGE_SHIFT, so need to update the size to match range
         * set's expectation.
         */
        size = (rc << PAGE_SHIFT) - (s & ~PAGE_MASK);
        ASSERT(rc <= PFN_DOWN(round_pgup(size)));
        *c = size;
        s += size;
        mfn = mfn_add(mfn, rc);
        if ( general_preempt_check() )
                return -ERESTART;
    }

    return rc;
}

/*
 * The rom_only parameter is used to signal the map/unmap helpers that the ROM
 * BAR's enable bit has changed with the memory decoding bit already enabled.
 * If rom_only is not set then it's the memory decoding bit that changed.
 */
static void modify_decoding(const struct pci_dev *pdev, uint16_t cmd,
                            bool rom_only)
{
    struct vpci_header *header = get_hwdom_vpci_header(pdev);
    bool map = cmd & PCI_COMMAND_MEMORY;
    unsigned int i;

    /*
     * Make sure there are no mappings in the MSIX MMIO areas, so that accesses
     * can be trapped (and emulated) by Xen when the memory decoding bit is
     * enabled.
     *
     * FIXME: punching holes after the p2m has been set up might be racy for
     * DomU usage, needs to be revisited.
     */
#ifdef CONFIG_X86
    if ( map && !rom_only && vpci_make_msix_hole(pdev) )
        return;

#endif
    for ( i = 0; i < ARRAY_SIZE(header->bars); i++ )
    {
        if ( !MAPPABLE_BAR(&header->bars[i]) )
            continue;

        if ( rom_only && header->bars[i].type == VPCI_BAR_ROM )
        {
            unsigned int rom_pos = (i == PCI_HEADER_NORMAL_NR_BARS)
                                   ? PCI_ROM_ADDRESS : PCI_ROM_ADDRESS1;
            uint32_t val = header->bars[i].addr |
                           (map ? PCI_ROM_ADDRESS_ENABLE : 0);

            header->bars[i].enabled = header->rom_enabled = map;
            pci_conf_write32(pdev->sbdf, rom_pos, val);
            return;
        }

        if ( !rom_only &&
             (header->bars[i].type != VPCI_BAR_ROM || header->rom_enabled) )
            header->bars[i].enabled = map;
    }

    if ( !rom_only )
        pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd);
    else
        ASSERT_UNREACHABLE();
}

bool vpci_process_pending(struct vcpu *v)
{
    if ( v->vpci.mem )
    {
        struct map_data data = {
            .d = v->domain,
            .map = v->vpci.cmd & PCI_COMMAND_MEMORY,
            .pdev = v->vpci.pdev,
        };
        int rc = rangeset_consume_ranges(v->vpci.mem, map_range, &data);

        if ( rc == -ERESTART )
            return true;

        spin_lock(&v->vpci.pdev->vpci->lock);
        /* Disable memory decoding unconditionally on failure. */
        modify_decoding(v->vpci.pdev,
                        rc ? v->vpci.cmd & ~PCI_COMMAND_MEMORY : v->vpci.cmd,
                        !rc && v->vpci.rom_only);
        spin_unlock(&v->vpci.pdev->vpci->lock);

        rangeset_destroy(v->vpci.mem);
        v->vpci.mem = NULL;
        if ( rc )
            /*
             * FIXME: in case of failure remove the device from the domain.
             * Note that there might still be leftover mappings. While this is
             * safe for Dom0, for DomUs the domain will likely need to be
             * killed in order to avoid leaking stale p2m mappings on
             * failure.
             */
            vpci_remove_device(v->vpci.pdev);
    }

    return false;
}

static int __init apply_map(struct domain *d, const struct pci_dev *pdev,
                            struct rangeset *mem, uint16_t cmd)
{
    struct map_data data = { .d = d, .map = true,
        .pdev = (struct pci_dev *)pdev };
    int rc;

    while ( (rc = rangeset_consume_ranges(mem, map_range, &data)) == -ERESTART )
        process_pending_softirqs();
    rangeset_destroy(mem);
    if ( !rc )
        modify_decoding(pdev, cmd, false);

    return rc;
}

static void defer_map(struct domain *d, struct pci_dev *pdev,
                      struct rangeset *mem, uint16_t cmd, bool rom_only)
{
    struct vcpu *curr = current;

    /*
     * FIXME: when deferring the {un}map the state of the device should not
     * be trusted. For example the enable bit is toggled after the device
     * is mapped. This can lead to parallel mapping operations being
     * started for the same device if the domain is not well-behaved.
     */
    curr->vpci.pdev = pdev;
    curr->vpci.mem = mem;
    curr->vpci.cmd = cmd;
    curr->vpci.rom_only = rom_only;
    /*
     * Raise a scheduler softirq in order to prevent the guest from resuming
     * execution with pending mapping operations, to trigger the invocation
     * of vpci_process_pending().
     */
    raise_softirq(SCHEDULE_SOFTIRQ);
}

static int modify_bars(const struct pci_dev *pdev, uint16_t cmd, bool rom_only)
{
    struct vpci_header *header;
    struct rangeset *mem = rangeset_new(NULL, NULL, 0);
    struct pci_dev *tmp, *dev = NULL;
#ifdef CONFIG_X86
    const struct vpci_msix *msix = pdev->vpci->msix;
#endif
    unsigned int i;
    int rc;

    if ( !mem )
        return -ENOMEM;

    if ( is_hardware_domain(current->domain) )
        header = get_hwdom_vpci_header(pdev);
    else
        header = get_vpci_header(current->domain, pdev);

    /*
     * Create a rangeset that represents the current device BARs memory region
     * and compare it against all the currently active BAR memory regions. If
     * an overlap is found, subtract it from the region to be mapped/unmapped.
     *
     * First fill the rangeset with all the BARs of this device or with the ROM
     * BAR only, depending on whether the guest is toggling the memory decode
     * bit of the command register, or the enable bit of the ROM BAR register.
     *
     * Use the PCI reserved bits of the BAR to pass BAR's index.
     */
    for ( i = 0; i < ARRAY_SIZE(header->bars); i++ )
    {
        const struct vpci_bar *bar = &header->bars[i];
        unsigned long start = (bar->addr & PCI_BASE_ADDRESS_MEM_MASK) | i;
        unsigned long end = (bar->addr & PCI_BASE_ADDRESS_MEM_MASK) +
            bar->size - 1;

        if ( !MAPPABLE_BAR(bar) ||
             (rom_only ? bar->type != VPCI_BAR_ROM
                       : (bar->type == VPCI_BAR_ROM && !header->rom_enabled)) )
            continue;

        rc = rangeset_add_range(mem, start, end);
        if ( rc )
        {
            printk(XENLOG_G_WARNING "Failed to add [%lx, %lx]: %d\n",
                   start, end, rc);
            rangeset_destroy(mem);
            return rc;
        }
    }

#ifdef CONFIG_X86
    /* Remove any MSIX regions if present. */
    for ( i = 0; msix && i < ARRAY_SIZE(msix->tables); i++ )
    {
        unsigned long start = (vmsix_table_addr(pdev->vpci, i) &
                               PCI_BASE_ADDRESS_MEM_MASK) | i;
        unsigned long end = (vmsix_table_addr(pdev->vpci, i) &
                             PCI_BASE_ADDRESS_MEM_MASK ) +
                             vmsix_table_size(pdev->vpci, i) - 1;

        rc = rangeset_remove_range(mem, start, end);
        if ( rc )
        {
            printk(XENLOG_G_WARNING
                   "Failed to remove MSIX table [%lx, %lx]: %d\n",
                   start, end, rc);
            rangeset_destroy(mem);
            return rc;
        }
    }
#endif

    /*
     * Check for overlaps with other BARs. Note that only BARs that are
     * currently mapped (enabled) are checked for overlaps.
     */
    for_each_pdev ( pdev->domain, tmp )
    {
        struct vpci_header *header;

        if ( tmp == pdev )
        {
            /*
             * Need to store the device so it's not constified and defer_map
             * can modify it in case of error.
             */
            dev = tmp;
            if ( !rom_only )
                /*
                 * If memory decoding is toggled avoid checking against the
                 * same device, or else all regions will be removed from the
                 * memory map in the unmap case.
                 */
                continue;
        }

        header = get_vpci_header(tmp->domain, pdev);

        for ( i = 0; i < ARRAY_SIZE(header->bars); i++ )
        {
            const struct vpci_bar *bar = &header->bars[i];
            unsigned long start = (bar->addr & PCI_BASE_ADDRESS_MEM_MASK) | i;
            unsigned long end = (bar->addr & PCI_BASE_ADDRESS_MEM_MASK)
                + bar->size - 1;

            if ( !bar->enabled || !rangeset_overlaps_range(mem, start, end) ||
                 /*
                  * If only the ROM enable bit is toggled check against other
                  * BARs in the same device for overlaps, but not against the
                  * same ROM BAR.
                  */
                 (rom_only && tmp == pdev && bar->type == VPCI_BAR_ROM) )
                continue;

            rc = rangeset_remove_range(mem, start, end);
            if ( rc )
            {
                printk(XENLOG_G_WARNING "Failed to remove [%lx, %lx]: %d\n",
                       start, end, rc);
                rangeset_destroy(mem);
                return rc;
            }
        }
    }

    ASSERT(dev);

    if ( system_state < SYS_STATE_active )
    {
        /*
         * Mappings might be created when building Dom0 if the memory decoding
         * bit of PCI devices is enabled. In that case it's not possible to
         * defer the operation, so call apply_map in order to create the
         * mappings right away. Note that at build time this function will only
         * be called iff the memory decoding bit is enabled, thus the operation
         * will always be to establish mappings and process all the BARs.
         */
        ASSERT((cmd & PCI_COMMAND_MEMORY) && !rom_only);
        return apply_map(pdev->domain, pdev, mem, cmd);
    }

    defer_map(dev->domain, dev, mem, cmd, rom_only);

    return 0;
}

static void cmd_write(const struct pci_dev *pdev, unsigned int reg,
                      uint32_t cmd, void *data)
{
    uint16_t current_cmd = pci_conf_read16(pdev->sbdf, reg);

    /*
     * Let Dom0 play with all the bits directly except for the memory
     * decoding one.
     */
    if ( (cmd ^ current_cmd) & PCI_COMMAND_MEMORY )
        /*
         * Ignore the error. No memory has been added or removed from the p2m
         * (because the actual p2m changes are deferred in defer_map) and the
         * memory decoding bit has not been changed, so leave everything as-is,
         * hoping the guest will realize and try again.
         */
        modify_bars(pdev, cmd, false);
    else
        pci_conf_write16(pdev->sbdf, reg, cmd);
}

static void bar_write_hwdom(const struct pci_dev *pdev, unsigned int reg,
                      uint32_t val, void *data)
{
    struct vpci_bar *bar = data;
    uint8_t slot = PCI_SLOT(pdev->devfn), func = PCI_FUNC(pdev->devfn);
    bool hi = false;

    if ( bar->type == VPCI_BAR_MEM64_HI )
    {
        ASSERT(reg > PCI_BASE_ADDRESS_0);
        bar--;
        hi = true;
    }
    else
        val &= PCI_BASE_ADDRESS_MEM_MASK;

    if ( pci_conf_read16(pdev->sbdf, PCI_COMMAND) & PCI_COMMAND_MEMORY )
    {
        /* If the value written is the current one avoid printing a warning. */
        if ( val != (uint32_t)(bar->addr >> (hi ? 32 : 0)) )
        {
            struct vpci_header *header = get_hwdom_vpci_header(pdev);

            gprintk(XENLOG_WARNING,
                    "%04x:%02x:%02x.%u: ignored BAR %lu write with memory decoding enabled\n",
                    pdev->seg, pdev->bus, slot, func,
                    bar - header->bars + hi);
        }
        return;
    }

    /*
     * Update the cached address, so that when memory decoding is enabled
     * Xen can map the BAR into the guest p2m.
     */
    bar->addr &= ~(0xffffffffull << (hi ? 32 : 0));
    bar->addr |= (uint64_t)val << (hi ? 32 : 0);

    /* Make sure Xen writes back the same value for the BAR RO bits. */
    if ( !hi )
    {
        val |= bar->type == VPCI_BAR_MEM32 ? PCI_BASE_ADDRESS_MEM_TYPE_32
                                           : PCI_BASE_ADDRESS_MEM_TYPE_64;
        val |= bar->prefetchable ? PCI_BASE_ADDRESS_MEM_PREFETCH : 0;
    }

    pci_conf_write32(pdev->sbdf, reg, val);
}

static uint32_t bar_read_hwdom(const struct pci_dev *pdev, unsigned int reg,
                               void *data)
{
    return vpci_hw_read32(pdev, reg, data);
}

static void bar_write_guest(const struct pci_dev *pdev, unsigned int reg,
                            uint32_t val, void *data)
{
    struct vpci_bar *vbar = data;
    bool hi = false;

    if ( vbar->type == VPCI_BAR_MEM64_HI )
    {
        ASSERT(reg > PCI_BASE_ADDRESS_0);
        vbar--;
        hi = true;
    }
    vbar->addr &= ~(0xffffffffull << (hi ? 32 : 0));
    vbar->addr |= (uint64_t)val << (hi ? 32 : 0);
}

static uint32_t bar_read_guest(const struct pci_dev *pdev, unsigned int reg,
                               void *data)
{
    struct vpci_bar *vbar = data;
    uint32_t val;
    bool hi = false;

    if ( vbar->type == VPCI_BAR_MEM64_HI )
    {
        ASSERT(reg > PCI_BASE_ADDRESS_0);
        vbar--;
        hi = true;
    }

    if ( vbar->type == VPCI_BAR_MEM64_LO || vbar->type == VPCI_BAR_MEM64_HI )
    {
        if ( hi )
            val = vbar->addr >> 32;
        else
            val = vbar->addr & 0xffffffff;
        if ( val == ~0 )
        {
            /* Guests detects BAR's properties and sizes. */
            if ( !hi )
            {
                val = 0xffffffff & ~(vbar->size - 1);
                val |= vbar->type == VPCI_BAR_MEM32 ? PCI_BASE_ADDRESS_MEM_TYPE_32
                                                    : PCI_BASE_ADDRESS_MEM_TYPE_64;
                val |= vbar->prefetchable ? PCI_BASE_ADDRESS_MEM_PREFETCH : 0;
            }
            else
                val = vbar->size >> 32;
            vbar->addr &= ~(0xffffffffull << (hi ? 32 : 0));
            vbar->addr |= (uint64_t)val << (hi ? 32 : 0);
        }
    }
    else if ( vbar->type == VPCI_BAR_MEM32 )
    {
        val = vbar->addr;
        if ( val == ~0 )
        {
            if ( !hi )
            {
                val = 0xffffffff & ~(vbar->size - 1);
                val |= vbar->type == VPCI_BAR_MEM32 ? PCI_BASE_ADDRESS_MEM_TYPE_32
                                                    : PCI_BASE_ADDRESS_MEM_TYPE_64;
                val |= vbar->prefetchable ? PCI_BASE_ADDRESS_MEM_PREFETCH : 0;
            }
        }
    }
    else
    {
        val = vbar->addr;
    }
    return val;
}

static void rom_write(const struct pci_dev *pdev, unsigned int reg,
                      uint32_t val, void *data)
{
    struct vpci_header *header = get_hwdom_vpci_header(pdev);
    struct vpci_bar *rom = data;
    uint8_t slot = PCI_SLOT(pdev->devfn), func = PCI_FUNC(pdev->devfn);
    uint16_t cmd = pci_conf_read16(pdev->sbdf, PCI_COMMAND);
    bool new_enabled = val & PCI_ROM_ADDRESS_ENABLE;

    if ( (cmd & PCI_COMMAND_MEMORY) && header->rom_enabled && new_enabled )
    {
        gprintk(XENLOG_WARNING,
                "%04x:%02x:%02x.%u: ignored ROM BAR write with memory decoding enabled\n",
                pdev->seg, pdev->bus, slot, func);
        return;
    }

    if ( !header->rom_enabled )
        /*
         * If the ROM BAR is not enabled update the address field so the
         * correct address is mapped into the p2m.
         */
        rom->addr = val & PCI_ROM_ADDRESS_MASK;

    if ( !(cmd & PCI_COMMAND_MEMORY) || header->rom_enabled == new_enabled )
    {
        /* Just update the ROM BAR field. */
        header->rom_enabled = new_enabled;
        pci_conf_write32(pdev->sbdf, reg, val);
    }
    /*
     * Pass PCI_COMMAND_MEMORY or 0 to signal a map/unmap request, note that
     * this fabricated command is never going to be written to the register.
     */
    else if ( modify_bars(pdev, new_enabled ? PCI_COMMAND_MEMORY : 0, true) )
        /*
         * No memory has been added or removed from the p2m (because the actual
         * p2m changes are deferred in defer_map) and the ROM enable bit has
         * not been changed, so leave everything as-is, hoping the guest will
         * realize and try again. It's important to not update rom->addr in the
         * unmap case if modify_bars has failed, or future attempts would
         * attempt to unmap the wrong address.
         */
        return;

    if ( !new_enabled )
        rom->addr = val & PCI_ROM_ADDRESS_MASK;
}

static uint32_t bar_read_dispatch(const struct pci_dev *pdev, unsigned int reg,
                                  void *data)
{
    struct vpci_bar *vbar, *bar = data;

    if ( is_hardware_domain(current->domain) )
        return bar_read_hwdom(pdev, reg, data);

    vbar = get_vpci_bar(current->domain, pdev, bar->index);
    if ( !vbar )
        return ~0;

    return bar_read_guest(pdev, reg, vbar);
}

static void bar_write_dispatch(const struct pci_dev *pdev, unsigned int reg,
                               uint32_t val, void *data)
{
    struct vpci_bar *bar = data;

    if ( is_hardware_domain(current->domain) )
        bar_write_hwdom(pdev, reg, val, data);
    else
    {
        struct vpci_bar *vbar = get_vpci_bar(current->domain, pdev, bar->index);

        if ( !vbar )
            return;
        bar_write_guest(pdev, reg, val, vbar);
    }
}

/*
 * FIXME: This is called early while adding vPCI handlers which is done
 * by and for hwdom.
 */
static int init_bars(struct pci_dev *pdev)
{
    uint16_t cmd;
    uint64_t addr, size;
    unsigned int i, num_bars, rom_reg;
    struct vpci_header *header;
    struct vpci_bar *bars;
    int rc;

    header = get_hwdom_vpci_header(pdev);
    if ( !header )
        return -ENOMEM;
    bars = header->bars;

    switch ( pci_conf_read8(pdev->sbdf, PCI_HEADER_TYPE) & 0x7f )
    {
    case PCI_HEADER_TYPE_NORMAL:
        num_bars = PCI_HEADER_NORMAL_NR_BARS;
        rom_reg = PCI_ROM_ADDRESS;
        break;

    case PCI_HEADER_TYPE_BRIDGE:
        num_bars = PCI_HEADER_BRIDGE_NR_BARS;
        rom_reg = PCI_ROM_ADDRESS1;
        break;

    default:
        return -EOPNOTSUPP;
    }

    /* Setup a handler for the command register. */
    rc = vpci_add_register(pdev->vpci, vpci_hw_read16, cmd_write, PCI_COMMAND,
                           2, header);
    if ( rc )
        return rc;

    if ( pdev->ignore_bars )
        return 0;

    /* Disable memory decoding before sizing. */
    cmd = pci_conf_read16(pdev->sbdf, PCI_COMMAND);
    if ( cmd & PCI_COMMAND_MEMORY )
        pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd & ~PCI_COMMAND_MEMORY);

    for ( i = 0; i < num_bars; i++ )
    {
        uint8_t reg = PCI_BASE_ADDRESS_0 + i * 4;
        uint32_t val;

        bars[i].index = i;
        if ( i && bars[i - 1].type == VPCI_BAR_MEM64_LO )
        {
            bars[i].type = VPCI_BAR_MEM64_HI;
            rc = vpci_add_register(pdev->vpci, bar_read_dispatch,
                                   bar_write_dispatch, reg, 4, &bars[i]);
            if ( rc )
            {
                pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd);
                return rc;
            }

            continue;
        }

        val = pci_conf_read32(pdev->sbdf, reg);
        if ( (val & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO )
        {
            bars[i].type = VPCI_BAR_IO;
            continue;
        }
        if ( (val & PCI_BASE_ADDRESS_MEM_TYPE_MASK) ==
             PCI_BASE_ADDRESS_MEM_TYPE_64 )
            bars[i].type = VPCI_BAR_MEM64_LO;
        else
            bars[i].type = VPCI_BAR_MEM32;

        rc = pci_size_mem_bar(pdev->sbdf, reg, &addr, &size,
                              (i == num_bars - 1) ? PCI_BAR_LAST : 0);
        if ( rc < 0 )
        {
            pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd);
            return rc;
        }

        if ( size == 0 )
        {
            bars[i].type = VPCI_BAR_EMPTY;
            continue;
        }

        bars[i].addr = addr;
        bars[i].size = size;
        bars[i].prefetchable = val & PCI_BASE_ADDRESS_MEM_PREFETCH;

        rc = vpci_add_register(pdev->vpci, bar_read_dispatch,
                               bar_write_dispatch, reg, 4, &bars[i]);
        if ( rc )
        {
            pci_conf_write16(pdev->sbdf, PCI_COMMAND, cmd);
            return rc;
        }
    }

    /* Check expansion ROM. */
    rc = pci_size_mem_bar(pdev->sbdf, rom_reg, &addr, &size, PCI_BAR_ROM);
    if ( rc > 0 && size )
    {
        struct vpci_bar *rom = &header->bars[num_bars];

        rom->type = VPCI_BAR_ROM;
        rom->size = size;
        rom->addr = addr;
        rom->index = num_bars;
        header->rom_enabled = pci_conf_read32(pdev->sbdf, rom_reg) &
                              PCI_ROM_ADDRESS_ENABLE;

        rc = vpci_add_register(pdev->vpci, vpci_hw_read32, rom_write, rom_reg,
                               4, rom);
        if ( rc )
            rom->type = VPCI_BAR_EMPTY;
    }

    return (cmd & PCI_COMMAND_MEMORY) ? modify_bars(pdev, cmd, false) : 0;
}
REGISTER_VPCI_INIT(init_bars, VPCI_PRIORITY_MIDDLE);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */

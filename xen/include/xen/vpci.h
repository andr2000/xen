#ifndef _XEN_VPCI_H_
#define _XEN_VPCI_H_

#ifdef CONFIG_HAS_VPCI

#include <xen/pci.h>
#include <xen/types.h>
#include <xen/list.h>

typedef uint32_t vpci_read_t(const struct pci_dev *pdev, unsigned int reg,
                             void *data);

typedef void vpci_write_t(const struct pci_dev *pdev, unsigned int reg,
                          uint32_t val, void *data);

typedef int vpci_init_t(struct pci_dev *dev);
typedef void vpci_teardown_t(struct pci_dev *dev);

struct vpci_handler {
    vpci_init_t *init;
    vpci_teardown_t *teardown;
};

#define VPCI_PRIORITY_HIGH      "1"
#define VPCI_PRIORITY_MIDDLE    "5"
#define VPCI_PRIORITY_LOW       "9"

#define REGISTER_VPCI_INIT(i, t, p)                                     \
  const static struct vpci_handler i ## t ## _entry                     \
               __used_section(".data.vpci." p) = { .init = (i),         \
                                                   .teardown = (t), }

/* Add vPCI handlers to device. */
int __must_check vpci_add_handlers(struct pci_dev *dev);

/* Allocate a new pending task. */
struct vpci_vcpu_pending_task *
vpci_alloc_pending_task(const struct pci_dev *pdev);

/* Add a new pending task to the execution list. */
void vpci_add_pending_task(struct vpci_vcpu_pending_task *task);

/* Remove all pending vPCI tasks for vCPU. */
void vpci_remove_pending_tasks(struct vcpu *v);

/* Notify vPCI that device is assigned to guest. */
int __must_check vpci_assign_device(struct domain *d, struct pci_dev *dev);

/* Notify vPCI that device is de-assigned from guest. */
int __must_check vpci_deassign_device(struct domain *d, struct pci_dev *dev);

/* Remove all handlers and free vpci related structures. */
void vpci_remove_device(struct pci_dev *pdev);
/* Remove all handlers for the device given. */
void vpci_remove_device_registers(struct pci_dev *pdev);

/* Add/remove a register handler. Must be called holding the vpci_lock. */
int __must_check vpci_add_register(struct vpci *vpci,
                                   vpci_read_t *read_handler,
                                   vpci_write_t *write_handler,
                                   unsigned int offset, unsigned int size,
                                   void *data);
int __must_check vpci_remove_register(struct vpci *vpci, unsigned int offset,
                                      unsigned int size);

/* Generic read/write handlers for the PCI config space. */
uint32_t vpci_read(pci_sbdf_t sbdf, unsigned int reg, unsigned int size);
void vpci_write(pci_sbdf_t sbdf, unsigned int reg, unsigned int size,
                uint32_t data);

/* Passthrough handlers. */
uint32_t vpci_hw_read16(const struct pci_dev *pdev, unsigned int reg,
                        void *data);
uint32_t vpci_hw_read32(const struct pci_dev *pdev, unsigned int reg,
                        void *data);

/*
 * Check for pending vPCI operations on this vcpu. Returns true if the vcpu
 * should not run.
 */
bool __must_check vpci_process_pending(struct vcpu *v);

/* Add/remove BAR handlers for a domain. */
int vpci_bar_add_handlers(const struct domain *d, struct pci_dev *pdev);
int vpci_bar_remove_handlers(const struct domain *d, struct pci_dev *pdev);

struct vpci {
    /* List of vPCI handlers for a device. */
    struct list_head handlers;

#ifdef __XEN__
    /* Hide the rest of the vpci struct from the user-space test harness. */
    struct vpci_header {
        /* Information about the PCI BARs of this device. */
        struct vpci_bar {
            /* Physical view of the BAR. */
            uint64_t addr;
            /* Guest view of the BAR. */
            uint64_t guest_addr;
            uint64_t size;
            struct rangeset *mem;
            enum {
                VPCI_BAR_EMPTY,
                VPCI_BAR_IO,
                VPCI_BAR_MEM32,
                VPCI_BAR_MEM64_LO,
                VPCI_BAR_MEM64_HI,
                VPCI_BAR_ROM,
            } type;
            bool prefetchable : 1;
            /* Store whether the BAR is mapped into guest p2m. */
            bool enabled      : 1;
#define PCI_HEADER_NORMAL_NR_BARS        6
#define PCI_HEADER_BRIDGE_NR_BARS        2
        } bars[PCI_HEADER_NORMAL_NR_BARS + 1];
        /* At most 6 BARS + 1 expansion ROM BAR. */

        /*
         * Store whether the ROM enable bit is set (doesn't imply ROM BAR
         * is mapped into guest p2m) if there's a ROM BAR on the device.
         */
        bool rom_enabled      : 1;
        /* FIXME: currently there's no support for SR-IOV. */
    } header;

#ifdef CONFIG_HAS_PCI_MSI
    /* MSI data. */
    struct vpci_msi {
      /* Address. */
        uint64_t address;
        /* Mask bitfield. */
        uint32_t mask;
        /* Data. */
        uint16_t data;
        /* Number of vectors configured. */
        uint8_t vectors     : 6;
        /* Supports per-vector masking? */
        bool masking        : 1;
        /* 64-bit address capable? */
        bool address64      : 1;
        /* Enabled? */
        bool enabled        : 1;
        /* Arch-specific data. */
        struct vpci_arch_msi arch;
    } *msi;

    /* MSI-X data. */
    struct vpci_msix {
        struct pci_dev *pdev;
        /* List link. */
        struct list_head next;
        /* Table information. */
#define VPCI_MSIX_TABLE     0
#define VPCI_MSIX_PBA       1
#define VPCI_MSIX_MEM_NUM   2
        uint32_t tables[VPCI_MSIX_MEM_NUM];
        /* Maximum number of vectors supported by the device. */
        uint16_t max_entries : 12;
        /* MSI-X enabled? */
        bool enabled         : 1;
        /* Masked? */
        bool masked          : 1;
        /* Entries. */
        struct vpci_msix_entry {
            uint64_t addr;
            uint32_t data;
            bool masked  : 1;
            bool updated : 1;
            struct vpci_arch_msix_entry arch;
        } entries[];
    } *msix;
#endif /* CONFIG_HAS_PCI_MSI */
#endif
};

/*
 * There could be multiple operations pending for different
 * PCI devices. The structure below is used to represent such
 * an operation.
 */
struct vpci_vcpu_pending_task {
    struct list_head list;
    enum {
        MODIFY_MEMORY,
    } what;
    struct pci_dev *pdev;
    union {
        struct {
            /* Store state while {un}mapping of PCI BARs. */
            uint16_t cmd;
            bool rom_only : 1;
        } memory;
    };
};

struct vpci_vcpu {
    /* List of pending operations. */
    struct list_head pending_task_list;
};

#ifdef __XEN__
#ifdef CONFIG_HAS_PCI_MSI
void vpci_dump_msi(void);

/* Make sure there's a hole in the p2m for the MSIX mmio areas. */
int vpci_make_msix_hole(const struct pci_dev *pdev);

/* Arch-specific vPCI MSI helpers. */
void vpci_msi_arch_mask(struct vpci_msi *msi, const struct pci_dev *pdev,
                        unsigned int entry, bool mask);
int __must_check vpci_msi_arch_enable(struct vpci_msi *msi,
                                      const struct pci_dev *pdev,
                                      unsigned int vectors);
void vpci_msi_arch_disable(struct vpci_msi *msi, const struct pci_dev *pdev);
void vpci_msi_arch_update(struct vpci_msi *msi, const struct pci_dev *pdev);
void vpci_msi_arch_init(struct vpci_msi *msi);
void vpci_msi_arch_print(const struct vpci_msi *msi);

/* Arch-specific vPCI MSI-X helpers. */
void vpci_msix_arch_mask_entry(struct vpci_msix_entry *entry,
                               const struct pci_dev *pdev, bool mask);
int __must_check vpci_msix_arch_enable_entry(struct vpci_msix_entry *entry,
                                             const struct pci_dev *pdev,
                                             paddr_t table_base);
int __must_check vpci_msix_arch_disable_entry(struct vpci_msix_entry *entry,
                                              const struct pci_dev *pdev);
void vpci_msix_arch_init_entry(struct vpci_msix_entry *entry);
int vpci_msix_arch_print(const struct vpci_msix *msix);

/*
 * Helper functions to fetch MSIX related data. They are used by both the
 * emulated MSIX code and the BAR handlers.
 */
static inline paddr_t vmsix_table_base(const struct vpci *vpci, unsigned int nr)
{
    return vpci->header.bars[vpci->msix->tables[nr] & PCI_MSIX_BIRMASK].addr;
}

static inline paddr_t vmsix_table_addr(const struct vpci *vpci, unsigned int nr)
{
    return vmsix_table_base(vpci, nr) +
           (vpci->msix->tables[nr] & ~PCI_MSIX_BIRMASK);
}

/*
 * Note regarding the size calculation of the PBA: the spec mentions "The last
 * QWORD will not necessarily be fully populated", so it implies that the PBA
 * size is 64-bit aligned.
 */
static inline size_t vmsix_table_size(const struct vpci *vpci, unsigned int nr)
{
    return
        (nr == VPCI_MSIX_TABLE) ? vpci->msix->max_entries * PCI_MSIX_ENTRY_SIZE
                                : ROUNDUP(DIV_ROUND_UP(vpci->msix->max_entries,
                                                       8), 8);
}

static inline unsigned int vmsix_entry_nr(const struct vpci_msix *msix,
                                          const struct vpci_msix_entry *entry)
{
    return entry - msix->entries;
}
#endif /* CONFIG_HAS_PCI_MSI */
#endif /* __XEN__ */

#else /* !CONFIG_HAS_VPCI */
struct vpci_vcpu {};

static inline int vpci_add_handlers(struct pci_dev *pdev)
{
    return 0;
}

static inline int vpci_assign_device(struct domain *d, struct pci_dev *dev)
{
    return 0;
};

static inline int vpci_deassign_device(struct domain *d, struct pci_dev *dev)
{
    return 0;
};

static inline void vpci_remove_device(struct pci_dev *pdev) { }

static inline void vpci_dump_msi(void) { }

static inline uint32_t vpci_read(pci_sbdf_t sbdf, unsigned int reg,
                                 unsigned int size)
{
    ASSERT_UNREACHABLE();
    return ~(uint32_t)0;
}

static inline void vpci_write(pci_sbdf_t sbdf, unsigned int reg,
                              unsigned int size, uint32_t data)
{
    ASSERT_UNREACHABLE();
}

static inline bool vpci_process_pending(struct vcpu *v)
{
    ASSERT_UNREACHABLE();
    return false;
}
#endif

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */

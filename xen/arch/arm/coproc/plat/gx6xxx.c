/*
 * xen/arch/arm/coproc/plat/gx6xxx.c
 *
 * COPROC GPU GX6XXX platform specific code
 *
 * Oleksandr Andrushchenko <oleksandr_andrushchenko@epam.com>
 * Copyright (C) 2017 EPAM Systems Inc.
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

#include <asm/io.h>
#include <xen/delay.h>
#include <xen/domain_page.h>
#include <xen/err.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/vmap.h>

#include "../coproc.h"
#include "common.h"

#define DT_MATCH_GX6XXX DT_MATCH_COMPATIBLE("renesas,gsx")

#define GX6XXX_NUM_IRQ  1
#define GX6XXX_NUM_MMIO 1

#if 1
#define GX6XXX_DEBUG 1
#endif

enum vgx6xxx_state
{
    /* initialization sequence has started - collecting register values
     * so those can be used for real GPU initialization */
    VGX6XXX_STATE_INITIALIZING,
    /* scheduler is running, at least one context switch was made */
    VGX6XXX_STATE_RUNNING,
    /* asked to switch from but waiting for GPU to finish current jobs */
    VGX6XXX_STATE_IN_TRANSIT,
    /* context is off - queueing requests and interrupts */
    VGX6XXX_STATE_WAITING,
};
#define VGX6XXX_STATE_DEFAULT   VGX6XXX_STATE_INITIALIZING

static const char *vgx6xxx_state_to_str(enum vgx6xxx_state state)
{
    switch ( state )
    {
    case VGX6XXX_STATE_INITIALIZING:
        return "INITIALIZING";
    case VGX6XXX_STATE_RUNNING:
        return "RUNNING";
    case VGX6XXX_STATE_IN_TRANSIT:
        return "IN_TRANSIT";
    case VGX6XXX_STATE_WAITING:
        return "WAITING";
    default:
        return "-=UNKNOWN=-";
    }
}

struct vgx6xxx_info
{
    /* current state of the vcoproc */
    enum vgx6xxx_state state;

    /* set if scheduler has been started for this vcoproc */
    bool scheduler_started;

    /*
     ***************************************************************************
     *                           REGISTERS
     ***************************************************************************
     */
    /* This is the current IRQ status register value reported/updated
     * to/from domains. Set on real IRQ from GPU, low 32-bits
     */
    uint32_t reg_val_irq_status_lo;
    /* Current value of the soft reset register, used to determine
     * when FW starts to run
     */
    uint32_t reg_val_cr_soft_reset_lo;
    uint32_t reg_val_cr_soft_reset_hi;

    /* number of writes to RGX_CR_MTS_SCHEDULE while not in running state */
    int reg_cr_mts_schedule_lo_wait_cnt;

    /*
     ***************************************************************************
     * FIXME: Value of the registers below must be saved on write
     ***************************************************************************
     */
    /* FIXME: META boot control register - low 32-bits are used */
    /* FIXME: this must be tracked when written, reset on read */
    uint32_t reg_val_cr_meta_boot_lo;

    uint32_t reg_val_cr_mts_garten_wrapper_config_lo;
    uint32_t reg_val_cr_mts_garten_wrapper_config_hi;

    /*
     ***************************************************************************
     * FIXME: Value of the registers remain constant once written
     * and can be read back
     ***************************************************************************
     */
    /* FIXME: SLC control register - low 32-bits are used */
    uint32_t reg_val_cr_slc_ctrl_misc_lo;
    uint32_t reg_val_cr_axi_ace_lite_configuration_lo;
    uint32_t reg_val_cr_axi_ace_lite_configuration_hi;
    /* FIXME: address of kernel page catalog, MMU page table?
     */
    uint32_t reg_val_cr_bif_cat_base0_lo;
    uint32_t reg_val_cr_bif_cat_base0_hi;
};

struct gx6xxx_info
{
    struct vcoproc_instance *curr;
    /* FIXME: IRQ registers are 64-bit, but only low 32-bits are used */
    uint32_t *reg_vaddr_irq_status;
    uint32_t *reg_vaddr_irq_clear;
};

static inline void gx6xxx_set_state(struct vcoproc_instance *vcoproc,
                                    enum vgx6xxx_state state)
{
    struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)vcoproc->priv;

    dev_dbg(vcoproc->coproc->dev,
            "Domain %d going from %s to %s\n", vcoproc->domain->domain_id,
            vgx6xxx_state_to_str(vinfo->state), vgx6xxx_state_to_str(state));
    vinfo->state = state;
}

#define RGX_CR_META_SP_MSLVIRQSTATUS                  (0x0AC8U)
#define RGX_CR_META_SP_MSLVIRQSTATUS_MASKFULL         (IMG_UINT64_C(0x000000000000000C))
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT3_SHIFT  (3U)
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT3_CLRMSK (0XFFFFFFF7U)
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT3_EN     (0X00000008U)
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_SHIFT  (2U)
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_CLRMSK (0XFFFFFFFBU)
#define RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_EN     (0X00000004U)

#define RGXFW_CR_IRQ_STATUS                           RGX_CR_META_SP_MSLVIRQSTATUS
#define RGXFW_CR_IRQ_STATUS_EVENT_EN                  RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_EN
#define RGXFW_CR_IRQ_CLEAR                            RGX_CR_META_SP_MSLVIRQSTATUS
#define RGXFW_CR_IRQ_CLEAR_MASK                       RGX_CR_META_SP_MSLVIRQSTATUS_TRIGVECT2_CLRMSK

#define RGX_CR_META_SP_MSLVDATAX                      (0x0A00U)
#define RGX_CR_SOFT_RESET                             (0x0100U)
#define RGX_CR_TIMER                                  (0x0160U)
#define RGX_CR_META_SP_MSLVCTRL0                      (0x0A10U)
#define RGX_CR_META_SP_MSLVCTRL1                      (0x0A18U)
#define RGX_CR_MTS_GARTEN_WRAPPER_CONFIG              (0x0B50U)
#define RGX_CR_META_BOOT                              (0x0BF8U)
#define RGX_CR_BIF_CAT_BASE0                          (0x1200U)
#define RGX_CR_SLC_CTRL_MISC                          (0x3800U)
#define RGX_CR_AXI_ACE_LITE_CONFIGURATION             (0x38C0U)

#define RGX_CR_MTS_SCHEDULE                           (0x0B00U)
#define RGX_CR_MTS_SCHEDULE_TASK_COUNTED              (0X00000010U)

#define RGX_CR_SIDEKICK_IDLE                (0x03C8U)
#define RGX_CR_SIDEKICK_IDLE_MASKFULL       (0x000000000000007F)
#define RGX_CR_SIDEKICK_IDLE_FB_CDC_EN      (0X00000040U)
#define RGX_CR_SIDEKICK_IDLE_MMU_EN         (0X00000020U)
#define RGX_CR_SIDEKICK_IDLE_BIF128_EN      (0X00000010U)
#define RGX_CR_SIDEKICK_IDLE_TLA_EN         (0X00000008U)
#define RGX_CR_SIDEKICK_IDLE_GARTEN_EN      (0X00000004U)
#define RGX_CR_SIDEKICK_IDLE_HOSTIF_EN      (0X00000002U)
#define RGX_CR_SIDEKICK_IDLE_SOCIF_EN       (0X00000001U)

#define RGX_CR_SLC_IDLE                     (0x3898U)
#define RGX_CR_SLC_IDLE_MASKFULL            (0x00000000000000FF)

#define RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC                (0x0B40U)
#define RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_MASKFULL       (0x000000000000FFFF)
#define RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK (0XFFFF0000U)

#define RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC                 (0x0B30U)
#define RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_MASKFULL        (0x000000000000FFFF)
#define RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK (0XFFFF0000U)

#define RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC                (0x0B48U)
#define RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_MASKFULL       (0x000000000000FFFF)
#define RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK (0XFFFF0000U)

#define RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC                 (0x0B38U)
#define RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_MASKFULL        (0x000000000000FFFF)
#define RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK (0XFFFF0000U)

#define RGX_CR_BIF_STATUS_MMU                             (0x1358U)
#define RGX_CR_BIF_STATUS_MMU_MASKFULL                    (0x00000000000000FF)
#define RGX_CR_BIF_STATUS_MMU_REQUESTS_CLRMSK             (0XFFFFFF00U)

#define RGX_CR_BIFPM_STATUS_MMU                           (0x1350U)
#define RGX_CR_BIFPM_STATUS_MMU_MASKFULL                  (0x00000000000000FF)
#define RGX_CR_BIFPM_STATUS_MMU_REQUESTS_CLRMSK           (0XFFFFFF00U)

#define RGX_CR_BIFPM_READS_EXT_STATUS                     (0x1338U)
#define RGX_CR_BIFPM_READS_EXT_STATUS_MASKFULL            (0x000000000000FFFF)
#define RGX_CR_BIFPM_READS_EXT_STATUS_BANK0_CLRMSK        (0XFFFF0000U)

#define RGX_CR_SLC_STATUS1                                (0x3870U)
#define RGX_CR_SLC_STATUS1_MASKFULL                       (0x800003FF03FFFFFFUL)

#define RGX_CR_META_SP_MSLVCTRL1_READY_EN                 (0X01000000U)
#define RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN          (0X04000000U)

#define RGX_CR_META_SP_MSLVDATAT                          (0x0A08U)

#define REG_LO32(a) ( (a) )
#define REG_HI32(a) ( (a) + sizeof(uint32_t) )

bool gx6xxx_debug = true;

#ifdef GX6XXX_DEBUG
static void gx6xxx_print_reg(const char *prefix, uint32_t reg, uint32_t val)
{
    char *name;

    if ( !gx6xxx_debug )
        return;
    switch (reg) {
    case RGX_CR_SOFT_RESET:
        name = "RGX_CR_SOFT_RESET LO";
        break;
    case RGX_CR_SOFT_RESET + 4:
        name = "RGX_CR_SOFT_RESET HI";
        break;
    case RGX_CR_SLC_CTRL_MISC:
        name = "RGX_CR_SLC_CTRL_MISC LO";
        break;
    case RGX_CR_SLC_CTRL_MISC + 4:
        name = "RGX_CR_SLC_CTRL_MISC HI";
        break;
    case RGX_CR_META_BOOT:
        name = "RGX_CR_META_BOOT LO";
        break;
    case RGX_CR_META_BOOT + 4:
        name = "RGX_CR_META_BOOT HI";
        break;
    case RGX_CR_META_SP_MSLVIRQSTATUS:
        name = "RGXFW_CR_IRQ_STATUS/CLEAR";
        break;
    case RGX_CR_TIMER:
        name = "RGX_CR_TIMER LO";
        break;
    case RGX_CR_TIMER + 4:
        name = "RGX_CR_TIMER HI";
        break;
    case RGX_CR_MTS_GARTEN_WRAPPER_CONFIG:
        name = "RGX_CR_MTS_GARTEN_WRAPPER_CONFIG LO";
        break;
    case RGX_CR_MTS_GARTEN_WRAPPER_CONFIG + 4:
        name = "RGX_CR_MTS_GARTEN_WRAPPER_CONFIG HI";
        break;
    case RGX_CR_AXI_ACE_LITE_CONFIGURATION:
        name = "RGX_CR_AXI_ACE_LITE_CONFIGURATION LO";
        break;
    case RGX_CR_AXI_ACE_LITE_CONFIGURATION + 4:
        name = "RGX_CR_AXI_ACE_LITE_CONFIGURATION HI";
        break;
    case RGX_CR_BIF_CAT_BASE0:
        name = "RGX_CR_BIF_CAT_BASE0 LO";
        break;
    case RGX_CR_BIF_CAT_BASE0 + 4:
        name = "RGX_CR_BIF_CAT_BASE0 HI";
        break;
    case RGX_CR_META_SP_MSLVCTRL1:
        name = "RGX_CR_META_SP_MSLVCTRL1 LO";
        break;
    case RGX_CR_META_SP_MSLVCTRL1 + 4:
        name = "RGX_CR_META_SP_MSLVCTRL1 HI";
        break;
    case RGX_CR_MTS_SCHEDULE:
        name = "RGX_CR_MTS_SCHEDULE LO";
        break;
    case RGX_CR_MTS_SCHEDULE + 4:
        name = "RGX_CR_MTS_SCHEDULE HI";
        break;
    case RGX_CR_META_SP_MSLVCTRL0:
        name = "RGX_CR_META_SP_MSLVCTRL0 LO";
        break;
    case RGX_CR_META_SP_MSLVCTRL0 + 4:
        name = "RGX_CR_META_SP_MSLVCTRL0 HI";
        break;
    case RGX_CR_META_SP_MSLVDATAX:
        name = "RGX_CR_META_SP_MSLVDATAX LO";
        break;
    case RGX_CR_META_SP_MSLVDATAX + 4:
        name = "RGX_CR_META_SP_MSLVDATAX HI";
        break;
    case RGX_CR_SIDEKICK_IDLE:
        name = "RGX_CR_SIDEKICK_IDLE LO";
        break;
    case RGX_CR_SLC_IDLE:
        name = "RGX_CR_SLC_IDLE LO";
        break;
    case RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC:
        name = "RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC LO";
        break;
    case RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC:
        name = "RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC LO";
        break;
    case RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC:
        name = "RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC LO";
        break;
    case RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC:
        name = "RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC LO";
        break;
    case RGX_CR_META_SP_MSLVDATAT:
        name = "RGX_CR_META_SP_MSLVDATAT LO";
        break;
    case RGX_CR_BIF_STATUS_MMU:
        name = "RGX_CR_BIF_STATUS_MMU LO";
        break;
    case RGX_CR_BIFPM_STATUS_MMU:
        name = "RGX_CR_BIFPM_STATUS_MMU LO";
        break;
    case RGX_CR_BIFPM_READS_EXT_STATUS:
        name = "RGX_CR_BIFPM_READS_EXT_STATUS LO";
        break;
    case RGX_CR_SLC_STATUS1:
        name = "RGX_CR_SLC_STATUS1 LO";
        break;
    case RGX_CR_SLC_STATUS1 + 4:
        name = "RGX_CR_SLC_STATUS1 HI";
        break;
    default:
        name = "??";
        printk("Unknown register %08x\n", reg);
        break;
    }
    printk("%s: %s -> %08x\n", prefix, name, val);
}
#else
#define gx6xxx_print_reg(a, b, c) {}
#endif

static inline uint32_t gx6xxx_read32(struct coproc_device *coproc,
                                     uint32_t offset)
{
#ifdef GX6XXX_DEBUG
    uint32_t val = readl((char *)coproc->mmios[0].base + offset);

    gx6xxx_print_reg(__FUNCTION__, offset, val);
    return val;
#else
    return readl((char *)coproc->mmios[0].base + offset);
#endif
}

static inline void gx6xxx_write32(struct coproc_device *coproc,
                                  uint32_t offset, uint32_t val)
{
    gx6xxx_print_reg(__FUNCTION__, offset, val);
    writel(val, (char *)coproc->mmios[0].base + offset);
}

static inline void gx6xxx_store32(uint32_t offset, uint32_t *reg, uint32_t val)
{
    gx6xxx_print_reg(__FUNCTION__, offset, val);
    *reg = val;
}

static inline uint64_t gx6xxx_read64(struct coproc_device *coproc,
                                     uint32_t offset)
{
#ifdef GX6XXX_DEBUG
    uint64_t val = readq((char *)coproc->mmios[0].base + offset);

    gx6xxx_print_reg(__FUNCTION__, REG_LO32(offset), val & 0xffffffff);
    gx6xxx_print_reg(__FUNCTION__, REG_HI32(offset), val >> 32);
    return val;
#else
    return readq((char *)coproc->mmios[0].base + offset);
#endif
}

static inline void gx6xxx_write64(struct coproc_device *coproc,
                                  uint32_t offset, uint64_t val)
{
    gx6xxx_print_reg(__FUNCTION__, REG_LO32(offset), val & 0xffffffff);
    gx6xxx_print_reg(__FUNCTION__, REG_HI32(offset), val >> 32);
    writeq(val, (char *)coproc->mmios[0].base + offset);
}

#define RGXFWIF_POW_STATES \
  X(RGXFWIF_POW_OFF)            /* idle and handshaked with the host (ready to full power down) */ \
  X(RGXFWIF_POW_ON)             /* running HW mds */ \
  X(RGXFWIF_POW_FORCED_IDLE)    /* forced idle */ \
  X(RGXFWIF_POW_IDLE)           /* idle waiting for host handshake */

typedef enum _RGXFWIF_POW_STATE_
{
#define X(NAME) NAME,
    RGXFWIF_POW_STATES
#undef X
} RGXFWIF_POW_STATE;

#define RGXFW_TRACE_BUFFER_LINESIZE (30)

/*! Total size of RGXFWIF_TRACEBUF dword (needs to be a multiple of RGXFW_TRACE_BUFFER_LINESIZE) */
#define RGXFW_TRACE_BUFFER_SIZE     (400*RGXFW_TRACE_BUFFER_LINESIZE)
#define RGXFW_TRACE_BUFFER_ASSERT_SIZE 200

typedef struct _RGXFWIF_ASSERTBUF_
{
    char    szPath[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
    char    szInfo[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
    uint32_t  ui32LineNum;
} __attribute__ ((aligned (8))) RGXFWIF_ASSERTBUF;

typedef struct _RGXFWIF_TRACEBUF_SPACE_
{
    uint32_t          ui32TracePointer;

    uint32_t pui32RGXFWIfTraceBuffer;
    uint32_t *pui32TraceBuffer;   /* To be used by host when reading from trace buffer */

    RGXFWIF_ASSERTBUF   sAssertBuf;
} __attribute__ ((aligned (8))) RGXFWIF_TRACEBUF_SPACE;

#define RGXFW_THREAD_NUM 1
#define RGXFWIF_DM_MAX          (5)
#define RGXFWIF_HWDM_MAX        (RGXFWIF_DM_MAX)

/* Firmware HWR states */
#define RGXFWIF_HWR_HARDWARE_OK     (0x1 << 0)  /*!< Tells if the HW state is ok or locked up */
#define RGXFWIF_HWR_ANALYSIS_DONE   (0x1 << 2)  /*!< Tells if the analysis of a GPU lockup has already been performed */
#define RGXFWIF_HWR_GENERAL_LOCKUP  (0x1 << 3)  /*!< Tells if a DM unrelated lockup has been detected */
typedef uint32_t RGXFWIF_HWR_STATEFLAGS;

#define RGXFW_OS_STATE_ACTIVE_OS                    (1 << 0)    /*!< Non active operating systems should not be served by the FW */
#define RGXFW_OS_STATE_FREELIST_OK                  (1 << 1)    /*!< Pending freelist reconstruction from that particular OS */
#define RGXFW_OS_STATE_OFFLOADING                   (1 << 2)    /*!< Transient state while all the OS resources in the FW are cleaned up */

typedef uint32_t RGXFWIF_HWR_RECOVERYFLAGS;

typedef struct _RGXFWIF_TRACEBUF_
{
    uint32_t              ui32LogType;
    volatile RGXFWIF_POW_STATE      ePowState;
    RGXFWIF_TRACEBUF_SPACE  sTraceBuf[RGXFW_THREAD_NUM];

    uint32_t              aui32HwrDmLockedUpCount[RGXFWIF_DM_MAX];
    uint32_t              aui32HwrDmOverranCount[RGXFWIF_DM_MAX];
    uint32_t              aui32HwrDmRecoveredCount[RGXFWIF_DM_MAX];
    uint32_t              aui32HwrDmFalseDetectCount[RGXFWIF_DM_MAX];
    uint32_t              ui32HwrCounter;

    uint32_t              aui32CrPollAddr[RGXFW_THREAD_NUM];
    uint32_t              aui32CrPollMask[RGXFW_THREAD_NUM];

    RGXFWIF_HWR_STATEFLAGS      ui32HWRStateFlags;
    RGXFWIF_HWR_RECOVERYFLAGS   aui32HWRRecoveryFlags[RGXFWIF_HWDM_MAX];

    volatile uint32_t     ui32HWPerfRIdx;
    volatile uint32_t     ui32HWPerfWIdx;
    volatile uint32_t     ui32HWPerfWrapCount;
    uint32_t              ui32HWPerfSize;       /* Constant after setup, needed in FW */
    uint32_t              ui32HWPerfDropCount;  /* The number of times the FW drops a packet due to buffer full */

    /* These next three items are only valid at runtime when the FW is built
     * with RGX_HWPERF_UTILIZATION & RGX_HWPERF_DROP_TRACKING defined
     * in rgxfw_hwperf.c */
    uint32_t              ui32HWPerfUt;         /* Buffer utilisation, high watermark of bytes in use */
    uint32_t              ui32FirstDropOrdinal;/* The ordinal of the first packet the FW dropped */
    uint32_t              ui32LastDropOrdinal; /* The ordinal of the last packet the FW dropped */

    volatile uint32_t         aui32InterruptCount[RGXFW_THREAD_NUM]; /*!< Interrupt count from Threads > */
    uint32_t              ui32KCCBCmdsExecuted;
    uint64_t  __attribute__ ((aligned (8)))          ui64StartIdleTime;
    uint32_t              ui32PowMonEnergy;   /* Non-volatile power monitor energy count */

#define RGXFWIF_MAX_PCX 16
    uint32_t              ui32T1PCX[RGXFWIF_MAX_PCX];
    uint32_t              ui32T1PCXWOff;

    uint32_t                  ui32OSStateFlags[1];     /*!< State flags for each Operating System > */
} __attribute__ ((aligned (8))) RGXFWIF_TRACEBUF;

static void gx6xxx_shared_page_print_irq(struct vcoproc_instance *vcoproc,
                                   struct vgx6xxx_info *vinfo)
{
    mfn_t mfn;
    uint32_t *vaddr = NULL;
    uint64_t ipa;

    ipa = vinfo->reg_val_cr_bif_cat_base0_lo |
          (uint64_t)vinfo->reg_val_cr_bif_cat_base0_hi << 32;
    printk("Map IPA %lx\n", ipa);
    mfn = p2m_lookup(vcoproc->domain, _gfn(paddr_to_pfn(ipa)), NULL);
    printk("MFN is %lx\n", mfn);
    if ( mfn_eq(mfn, INVALID_MFN) )
    {
        printk("Failed to lookup BIF catalog base address\n");
        return;
    }

    flush_page_to_ram(mfn);
    vaddr = (uint32_t *)map_domain_page(mfn);
//    printk("===================================aui32InterruptCount %d\n", vaddr->aui32InterruptCount[0]);
    {
        int i, j;
        uint32_t *ptr = (uint32_t *)vaddr;

        for (i = 0; i < 4096 / sizeof(uint32_t) / 4; i++)
        {
            for (j = 0; j < 4; j++)
                printk(" %08x", *ptr++);
            printk("\n");
        }
    }

    if (vaddr)
        unmap_domain_page(vaddr);
}
static bool gx6xxx_check_start_condition(struct vcoproc_instance *vcoproc,
                                         struct vgx6xxx_info *vinfo)
{
    bool start = false;

    /* start condition is all zeros in the RGX_CR_SOFT_RESET register */
    if ( unlikely(!vinfo->reg_val_cr_soft_reset_lo &&
                  !vinfo->reg_val_cr_soft_reset_hi) )
    {
        if ( likely(!vinfo->scheduler_started) )
        {
            dev_dbg(vcoproc->coproc->dev, "Domain %d start condition met\n",
                    vcoproc->domain->domain_id);
            start = true;
        }
    }
    return start;
}

static bool gx6xxx_on_reg_write(uint32_t offset, uint32_t val,
                                struct vcoproc_instance *vcoproc)
{
    struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)vcoproc->priv;
    bool start = false;

    switch ( offset )
    {
    case REG_LO32(RGX_CR_META_BOOT):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_meta_boot_lo, val);
        break;
    case REG_LO32(RGX_CR_SOFT_RESET):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_soft_reset_lo, val);
        start = gx6xxx_check_start_condition(vcoproc, vinfo);
        break;
    case REG_HI32(RGX_CR_SOFT_RESET):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_soft_reset_hi, val);
        start = gx6xxx_check_start_condition(vcoproc, vinfo);
        break;
    case REG_LO32(RGX_CR_MTS_GARTEN_WRAPPER_CONFIG):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_mts_garten_wrapper_config_lo,
                       val);
        break;
    case REG_HI32(RGX_CR_MTS_GARTEN_WRAPPER_CONFIG):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_mts_garten_wrapper_config_hi,
                       val);
        break;
    case REG_LO32(RGX_CR_BIF_CAT_BASE0):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_bif_cat_base0_lo, val);
        break;
    case REG_HI32(RGX_CR_BIF_CAT_BASE0):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_bif_cat_base0_hi, val);
        break;
    case REG_LO32(RGX_CR_SLC_CTRL_MISC):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_slc_ctrl_misc_lo,
                       val);
        break;
    case REG_LO32(RGX_CR_AXI_ACE_LITE_CONFIGURATION):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_axi_ace_lite_configuration_lo,
                       val);
        break;
    case REG_HI32(RGX_CR_AXI_ACE_LITE_CONFIGURATION):
        gx6xxx_store32(offset, &vinfo->reg_val_cr_axi_ace_lite_configuration_hi,
                       val);
        break;
    default:
        break;
    }
    return start;
}

static int gx6xxx_mmio_read(struct vcpu *v, mmio_info_t *info,
                            register_t *r, void *priv)
{
    struct mmio *mmio = priv;
    struct vcoproc_rw_context ctx;
    struct vgx6xxx_info *vinfo;
    unsigned long flags;

    vcoproc_get_rw_context(v->domain, mmio, info, &ctx);
    spin_lock_irqsave(&ctx.coproc->vcoprocs_lock, flags);
    vinfo = (struct vgx6xxx_info *)ctx.vcoproc->priv;

    if ( unlikely((ctx.offset == REG_LO32(RGX_CR_TIMER)) ||
                  (ctx.offset == REG_HI32(RGX_CR_TIMER))) )
    {
        /*
         * FIXME: this is a special case: drivers will calibrate
         * delays(?) upon power on, so no possibility to defer this reading
         * without failure in the future. Thus, allow in any state
         * FIXME: assume timer register can be read always, even if GPU
         * hasn't been initialized/FW runs yet
         */
        *r = gx6xxx_read32(ctx.coproc, ctx.offset);
        goto out;
    }
    /* allow reading cached IRQ status in any state */
    if ( likely(ctx.offset == RGXFW_CR_IRQ_STATUS) )
    {
        *r = vinfo->reg_val_irq_status_lo;
        goto out;
    }
    if ( vinfo->state == VGX6XXX_STATE_RUNNING )
    {
        *r = gx6xxx_read32(ctx.coproc, ctx.offset);
    }
    else if ( (vinfo->state == VGX6XXX_STATE_WAITING) ||
              (vinfo->state == VGX6XXX_STATE_IN_TRANSIT) )
    {
    }
    else if ( vinfo->state == VGX6XXX_STATE_INITIALIZING )
    {
        /* FIXME: in this state we only expect dummy reads
         * of RGX_CR_SOFT_RESET. Just return all 0.
         */
        if ( likely((ctx.offset == REG_LO32(RGX_CR_SOFT_RESET)) ||
                    (ctx.offset == REG_HI32(RGX_CR_SOFT_RESET))) )
        {
            *r = 0;
            goto out;
        }
        BUG();
    }
out:
    spin_unlock_irqrestore(&ctx.coproc->vcoprocs_lock, flags);
    return 1;
}

static int gx6xxx_mmio_write(struct vcpu *v, mmio_info_t *info,
                             register_t r, void *priv)
{
    struct mmio *mmio = priv;
    struct vcoproc_rw_context ctx;
    struct vgx6xxx_info *vinfo;
    unsigned long flags;

    vcoproc_get_rw_context(v->domain, mmio, info, &ctx);
    spin_lock_irqsave(&ctx.coproc->vcoprocs_lock, flags);
    vinfo = (struct vgx6xxx_info *)ctx.vcoproc->priv;
#ifdef GX6XXX_DEBUG
    /* XXX: this code is used for DomU test GPU driver to start
     * vcoproc's scheduler
     */
    if ( unlikely(ctx.offset == 0) ) {
        if ( !vinfo->scheduler_started )
        {
            vinfo->scheduler_started = true;
            spin_unlock_irqrestore(&ctx.coproc->vcoprocs_lock, flags);
            vcoproc_scheduler_vcoproc_wake(ctx.coproc->sched, ctx.vcoproc);
            spin_lock_irqsave(&ctx.coproc->vcoprocs_lock, flags);
        }
        goto out;
    }
#endif
    /* FIXME: the very first read/write will change state to initializing */
    /* allow writing cached IRQ status in any state */
    if (ctx.offset == RGXFW_CR_IRQ_STATUS) {
        struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)ctx.vcoproc->priv;

        vinfo->reg_val_irq_status_lo = r;
        goto out;
    }
    if ( vinfo->state == VGX6XXX_STATE_RUNNING )
    {
        gx6xxx_write32(ctx.coproc, ctx.offset, r);

    }
    else if ( (vinfo->state == VGX6XXX_STATE_WAITING) ||
              (vinfo->state == VGX6XXX_STATE_IN_TRANSIT) )
    {
        if ( likely(ctx.offset == RGX_CR_MTS_SCHEDULE) )
        {
            BUG_ON( r != RGX_CR_MTS_SCHEDULE_TASK_COUNTED);
            vinfo->reg_cr_mts_schedule_lo_wait_cnt++;
            goto out;
        }
        dev_err(ctx.coproc->dev, "Unexpected write at %08x val %08x\n",
                ctx.offset, (uint32_t)r);
        BUG();
    }
    else if ( vinfo->state == VGX6XXX_STATE_INITIALIZING )
    {
        /* FIXME: in this state we only save values of the registers
         * so those can be used during real initialization
         */
        if ( unlikely(gx6xxx_on_reg_write(ctx.offset, r, ctx.vcoproc)) )
        {
            vinfo->scheduler_started = true;
            spin_unlock_irqrestore(&ctx.coproc->vcoprocs_lock, flags);
            vcoproc_scheduler_vcoproc_wake(ctx.coproc->sched, ctx.vcoproc);
            spin_lock_irqsave(&ctx.coproc->vcoprocs_lock, flags);
        }
    }
out:
    spin_unlock_irqrestore(&ctx.coproc->vcoprocs_lock, flags);
    return 1;
}

static void gx6xxx_irq_handler(int irq, void *dev,
                               struct cpu_user_regs *regs)
{
    struct coproc_device *coproc = dev;
    struct gx6xxx_info *info = (struct gx6xxx_info *)coproc->priv;
    uint32_t irq_status;
    unsigned long flags;

    printk("> %s dom %d\n", __FUNCTION__, info->curr->domain->domain_id);

    gx6xxx_shared_page_print_irq(info->curr, info->curr->priv);

    spin_lock_irqsave(&coproc->vcoprocs_lock, flags);
#if 0
    irq_status = readl(info->reg_vaddr_irq_status);
#else
    irq_status = gx6xxx_read32(coproc, RGXFW_CR_IRQ_STATUS);
#endif
    if (irq_status & RGXFW_CR_IRQ_STATUS_EVENT_EN)
    {
        struct vcoproc_instance *vcoproc = info->curr;
        struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)vcoproc->priv;

#if 0
        writel(RGXFW_CR_IRQ_CLEAR_MASK, info->reg_vaddr_irq_clear);
#else
        gx6xxx_write32(coproc, RGXFW_CR_IRQ_STATUS, RGXFW_CR_IRQ_CLEAR_MASK);
#endif
        /* Save interrupt status register, so we can deliver to domain later. */
        vinfo->reg_val_irq_status_lo = irq_status;
        vgic_vcpu_inject_spi(vcoproc->domain, irq);
    }
    spin_unlock_irqrestore(&coproc->vcoprocs_lock, flags);
    printk("< %s dom %d\n", __FUNCTION__, info->curr->domain->domain_id);
    if ( info->curr->domain->domain_id )
        printk("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++ delivering to Dom %d\n",
                info->curr->domain->domain_id);
}

static const struct mmio_handler_ops gx6xxx_mmio_handler = {
    .read = gx6xxx_mmio_read,
    .write = gx6xxx_mmio_write,
};

#define RGX_CR_SOFT_RESET_MASKFULL          (0x00E7FFFFFFFFFC1D)
#define RGX_CR_SOFT_RESET_ALL               (RGX_CR_SOFT_RESET_MASKFULL)

#define RGX_CR_SOFT_RESET_DUST_A_CORE_EN    (0X0000000020000000)
#define RGX_CR_SOFT_RESET_DUST_B_CORE_EN    (0X0000000040000000)
#define RGX_CR_SOFT_RESET_DUST_C_CORE_EN    (0X0000000800000000)
#define RGX_CR_SOFT_RESET_DUST_D_CORE_EN    (0X0000001000000000)
#define RGX_CR_SOFT_RESET_DUST_E_CORE_EN    (0X0000002000000000)
#define RGX_CR_SOFT_RESET_DUST_F_CORE_EN    (0X0000004000000000)
#define RGX_CR_SOFT_RESET_DUST_G_CORE_EN    (0X0000008000000000)
#define RGX_CR_SOFT_RESET_DUST_H_CORE_EN    (0X0000010000000000)

#define RGX_CR_SOFT_RESET_DUST_n_CORE_EN    (RGX_CR_SOFT_RESET_DUST_A_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_B_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_C_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_D_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_E_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_F_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_G_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_H_CORE_EN)

#define RGX_CR_SOFT_RESET_RASCAL_CORE_EN    (0X0000000080000000)
#define RGX_CR_SOFT_RESET_GARTEN_EN         (0X0000000100000000)

#define RGX_CR_SOFT_RESET_RASCALDUSTS_EN    (RGX_CR_SOFT_RESET_RASCAL_CORE_EN | \
                                             RGX_CR_SOFT_RESET_DUST_n_CORE_EN)

static int gx6xxx_ctx_gpu_start(struct coproc_device *coproc,
                                struct vgx6xxx_info *vinfo)
{
    /* perform soft-reset */
    gx6xxx_write64(coproc, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_ALL);
    gx6xxx_write64(coproc, RGX_CR_SOFT_RESET,
                   RGX_CR_SOFT_RESET_ALL ^ RGX_CR_SOFT_RESET_RASCALDUSTS_EN);
    (void)gx6xxx_read64(coproc, RGX_CR_SOFT_RESET);

    /* start everything, but META */
    gx6xxx_write64(coproc, RGX_CR_SOFT_RESET, RGX_CR_SOFT_RESET_GARTEN_EN);

    gx6xxx_write32(coproc, RGX_CR_SLC_CTRL_MISC,
                   vinfo->reg_val_cr_slc_ctrl_misc_lo);
    gx6xxx_write32(coproc, RGX_CR_META_BOOT,
                   vinfo->reg_val_cr_meta_boot_lo);
    gx6xxx_write64(coproc, RGX_CR_MTS_GARTEN_WRAPPER_CONFIG,
                   vinfo->reg_val_cr_mts_garten_wrapper_config_lo |
                   (uint64_t)vinfo->reg_val_cr_mts_garten_wrapper_config_hi << 32);
    gx6xxx_write64(coproc, RGX_CR_AXI_ACE_LITE_CONFIGURATION,
                   vinfo->reg_val_cr_axi_ace_lite_configuration_lo |
                   (uint64_t)vinfo->reg_val_cr_axi_ace_lite_configuration_hi << 32);
    gx6xxx_write64(coproc, RGX_CR_BIF_CAT_BASE0,
                   vinfo->reg_val_cr_bif_cat_base0_lo |
                   (uint64_t)vinfo->reg_val_cr_bif_cat_base0_hi << 32);

    /* wait for at least 16 cycles */
    udelay(32);

    gx6xxx_write64(coproc, RGX_CR_SOFT_RESET, 0x0);
    (void)gx6xxx_read64(coproc, RGX_CR_SOFT_RESET);

    /* wait for at least 16 cycles */
    udelay(32);

    /* FIXME: if slave is booting then it needs a kick to start */
    return 0;
}

int gx6xxx_poll_reg32(struct coproc_device *coproc, uint32_t offset,
                      uint32_t expected, uint32_t mask)
{
    uint32_t val;
    int retry = 10;

    gx6xxx_debug = false;
    do
    {
        /* read current register value and mask only those bits requested */
        val = gx6xxx_read32(coproc, offset) & mask;
        if ( val == expected )
        {
            gx6xxx_debug = true;
            return 0;
        }
        cpu_relax();
    } while (retry--);
    printk("%s expected %08x got %08x ))))))))))))))))))))))))))))))))))))))))\n",
                    __FUNCTION__, expected, val);
    gx6xxx_debug = true;
    return -ETIMEDOUT;
}

int gx6xxx_poll_reg64(struct coproc_device *coproc, uint32_t offset,
                      uint64_t expected, uint64_t mask)
{
    uint64_t val;
    int retry = 10;

    gx6xxx_debug = false;
    do
    {
        /* read current register value and mask only those bits requested */
        val = gx6xxx_read64(coproc, offset) & mask;
        if ( val == expected )
        {
            gx6xxx_debug = true;
            return 0;
        }
        cpu_relax();
    } while (retry--);
    gx6xxx_debug = true;
    printk("%s expected %016lx got %016lx ))))))))))))))))))))))))))))))))))))))))\n",
                    __FUNCTION__, expected, val);
    return -ETIMEDOUT;
}

static int gx6xxx_write_via_slave_port32(struct coproc_device *coproc,
                                         uint32_t offset,
                                         uint32_t val)
{
    int ret;

    /* Wait for Slave Port to be Ready */
    ret = gx6xxx_poll_reg32(coproc, RGX_CR_META_SP_MSLVCTRL1,
                          RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
                          RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN);
    if ( ret < 0 )
        return ret;

    /* Issue a Write */
    gx6xxx_write32(coproc, RGX_CR_META_SP_MSLVCTRL0, offset);
    gx6xxx_write32(coproc, RGX_CR_META_SP_MSLVDATAT, val);

    return 0;
}

/* try stopping the GPU: 0 on success, <0 if still busy */
static int gx6xxx_ctx_gpu_stop(struct coproc_device *coproc,
                               struct vgx6xxx_info *vinfo)
{
    int ret;

    printk("%s GPU stopping =============================================\n", __FUNCTION__);

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_SIDEKICK_IDLE,
                    RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN),
                    RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN));
    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_SLC_IDLE,
                            RGX_CR_SLC_IDLE_MASKFULL,
                            RGX_CR_SLC_IDLE_MASKFULL);
    if ( ret < 0 )
        return ret;

    gx6xxx_write32(coproc, RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC,
                  RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK
                  & RGX_CR_MTS_INTCTX_THREAD0_DM_ASSOC_MASKFULL);

    gx6xxx_write32(coproc, RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC,
                  RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_DM_ASSOC_CLRMSK
                  & RGX_CR_MTS_BGCTX_THREAD0_DM_ASSOC_MASKFULL);

    gx6xxx_write32(coproc, RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC,
                  RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK
                  & RGX_CR_MTS_INTCTX_THREAD1_DM_ASSOC_MASKFULL);

    gx6xxx_write32(coproc, RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC,
                  RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_DM_ASSOC_CLRMSK
                  & RGX_CR_MTS_BGCTX_THREAD1_DM_ASSOC_MASKFULL);

    /* Disabling threads is only required for pdumps to stop the fw gracefully */

#define META_CR_CTRLREG_BASE(T)                 (0x04800000 + 0x1000*(T))

#define META_CR_COREREG_ENABLE          (0x0000000)
#define META_CR_TXENABLE_ENABLE_BIT     (0x00000001)   /* Set if running */
#define META_CR_T0ENABLE_OFFSET         (META_CR_CTRLREG_BASE(0) + META_CR_COREREG_ENABLE)

    /* Disable thread 0 */
    ret = gx6xxx_write_via_slave_port32(coproc,
                                      META_CR_T0ENABLE_OFFSET,
                                      ~META_CR_TXENABLE_ENABLE_BIT);
    if ( ret < 0 )
        return ret;

#define META_CR_T1ENABLE_OFFSET         (META_CR_CTRLREG_BASE(1) + META_CR_COREREG_ENABLE)

    /* Disable thread 1 */
    ret = gx6xxx_write_via_slave_port32(coproc,
                                      META_CR_T1ENABLE_OFFSET,
                                      ~META_CR_TXENABLE_ENABLE_BIT);
    if ( ret < 0 )
        return ret;

    /* Clear down any irq raised by META (done after disabling the FW
     * threads to avoid a race condition).
     * This is only really needed for PDumps but we do it anyway driver-live.
     */
    gx6xxx_write32(coproc, RGX_CR_META_SP_MSLVIRQSTATUS, 0x0);

    /* Wait for the Slave Port to finish all the transactions */
    ret = gx6xxx_poll_reg32(coproc, RGX_CR_META_SP_MSLVCTRL1,
                          RGX_CR_META_SP_MSLVCTRL1_READY_EN | RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
                          RGX_CR_META_SP_MSLVCTRL1_READY_EN | RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN);
    if ( ret < 0 )
        return ret;

    /* Extra Idle checks */
    ret = gx6xxx_poll_reg32(coproc, RGX_CR_BIF_STATUS_MMU,
                          0,
                          RGX_CR_BIF_STATUS_MMU_MASKFULL);
    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_BIFPM_STATUS_MMU,
                          0,
                          RGX_CR_BIFPM_STATUS_MMU_MASKFULL);
    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_BIFPM_READS_EXT_STATUS,
                          0,
                          RGX_CR_BIFPM_READS_EXT_STATUS_MASKFULL);
    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg64(coproc, RGX_CR_SLC_STATUS1,
                          0,
                          RGX_CR_SLC_STATUS1_MASKFULL);
    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_SLC_IDLE,
                          RGX_CR_SLC_IDLE_MASKFULL,
                          RGX_CR_SLC_IDLE_MASKFULL);

    if ( ret < 0 )
        return ret;

    ret = gx6xxx_poll_reg32(coproc, RGX_CR_SIDEKICK_IDLE,
                          RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN),
                          RGX_CR_SIDEKICK_IDLE_MASKFULL^(RGX_CR_SIDEKICK_IDLE_GARTEN_EN|RGX_CR_SIDEKICK_IDLE_SOCIF_EN|RGX_CR_SIDEKICK_IDLE_HOSTIF_EN));

    if ( ret < 0 )
        return ret;

#if 0
    {
        uint32_t ui32RegValue;

        eError = RGXReadMetaRegThroughSP(hPrivate,
                                         META_CR_TxVECINT_BHALT,
                                         &ui32RegValue);
        if (eError != PVRSRV_OK) return eError;

        if ((ui32RegValue & 0xFFFFFFFFU) == 0x0)
        {
            /* Wait for Sidekick/Jones to signal IDLE including
             * the Garten Wrapper if there is no debugger attached
             * (TxVECINT_BHALT = 0x0) */
            eError = RGXPollReg32(hPrivate,
                                  RGX_CR_SIDEKICK_IDLE,
                                  RGX_CR_SIDEKICK_IDLE_GARTEN_EN,
                                  RGX_CR_SIDEKICK_IDLE_GARTEN_EN);
            if (eError != PVRSRV_OK) return eError;
        }
    }
#endif

    printk("%s GPU stopped =============================================\n", __FUNCTION__);
    return 0;
}

static s_time_t gx6xxx_ctx_switch_from(struct vcoproc_instance *curr)
{
    struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)curr->priv;
    struct coproc_device *coproc = curr->coproc;
    s_time_t wait_time = 0;
    unsigned long flags;

    printk("%s dom %d\n", __FUNCTION__, curr->domain->domain_id);
#if 1
    if ( curr->domain->domain_id )
        return 0;
#endif
    spin_lock_irqsave(&coproc->vcoprocs_lock, flags);
    if ( vinfo->state == VGX6XXX_STATE_RUNNING )
    {
        int ret;

        /* FIXME: go into waiting state now, so from now on all read/write
         * operations do not reach HW
         */
        gx6xxx_set_state(curr, VGX6XXX_STATE_WAITING);
        /* try stopping the GPU */
        /* FIXME: let late interrupts a chance to fire */
        spin_unlock_irqrestore(&coproc->vcoprocs_lock, flags);
        ret = gx6xxx_ctx_gpu_stop(coproc, vinfo);
        spin_lock_irqsave(&coproc->vcoprocs_lock, flags);
        if ( ret < 0 )
        {
            wait_time = MILLISECS(1);
            gx6xxx_set_state(curr, VGX6XXX_STATE_IN_TRANSIT);
            goto out;
        }
    }
    else if ( vinfo->state == VGX6XXX_STATE_IN_TRANSIT )
    {
        int ret;

        /* try stopping the GPU harder */
        /* FIXME: let late interrupts a chance to fire */
        spin_unlock_irqrestore(&coproc->vcoprocs_lock, flags);
        ret = gx6xxx_ctx_gpu_stop(coproc, vinfo);
        spin_lock_irqsave(&coproc->vcoprocs_lock, flags);
        if ( ret < 0 )
        {
            wait_time = MILLISECS(1);
            goto out;
        }
        gx6xxx_set_state(curr, VGX6XXX_STATE_WAITING);
    }
    else
    {
        gx6xxx_set_state(curr, vinfo->state);
        BUG();
    }
out:
    spin_unlock_irqrestore(&coproc->vcoprocs_lock, flags);
    return wait_time;
}

static int gx6xxx_ctx_switch_to(struct vcoproc_instance *next)
{
    struct gx6xxx_info *info = (struct gx6xxx_info *)next->coproc->priv;
    struct vgx6xxx_info *vinfo = (struct vgx6xxx_info *)next->priv;
    unsigned long flags;

    printk("%s dom %d\n", __FUNCTION__, next->domain->domain_id);
#if 1
    if ( next->domain->domain_id )
        return 0;
#endif
    spin_lock_irqsave(&next->coproc->vcoprocs_lock, flags);
    info->curr = next;
    if ( vinfo->state == VGX6XXX_STATE_WAITING )
    {
        gx6xxx_ctx_gpu_start(next->coproc, vinfo);
        gx6xxx_set_state(next, VGX6XXX_STATE_RUNNING);
        /* flush scheduled work */
        if ( likely(vinfo->reg_cr_mts_schedule_lo_wait_cnt) )
            do
            {
                gx6xxx_write32(next->coproc, RGX_CR_MTS_SCHEDULE,
                               RGX_CR_MTS_SCHEDULE_TASK_COUNTED);
            }
            while (--vinfo->reg_cr_mts_schedule_lo_wait_cnt);
    }
    else if ( vinfo->state == VGX6XXX_STATE_INITIALIZING )
    {
        gx6xxx_set_state(next, VGX6XXX_STATE_RUNNING);
        gx6xxx_ctx_gpu_start(next->coproc, vinfo);
    }
    else
    {
        gx6xxx_set_state(next, vinfo->state);
        BUG();
    }
    spin_unlock_irqrestore(&next->coproc->vcoprocs_lock, flags);
    return 0;
}

static int gx6xxx_vcoproc_init(struct vcoproc_instance *vcoproc)
{
    struct mmio *mmio = &vcoproc->coproc->mmios[0];
    struct vgx6xxx_info *vinfo;

    vcoproc->priv = xzalloc(struct vgx6xxx_info);
    if ( !vcoproc->priv )
    {
        dev_err(vcoproc->coproc->dev,
                "failed to allocate vcoproc private data\n");
        return -ENOMEM;
    }
    vinfo = (struct vgx6xxx_info *)vcoproc->priv;

    vinfo->state = VGX6XXX_STATE_DEFAULT;

    vinfo->reg_val_cr_soft_reset_lo = UINT32_MAX;
    vinfo->reg_val_cr_soft_reset_hi = UINT32_MAX;

    register_mmio_handler(vcoproc->domain, &gx6xxx_mmio_handler,
                          mmio->addr, mmio->size, mmio);

    return 0;
}

static void gx6xxx_vcoproc_deinit(struct vcoproc_instance *vcoproc)
{
    xfree(vcoproc->priv);
}

static const struct coproc_ops gx6xxx_vcoproc_ops = {
    .vcoproc_init        = gx6xxx_vcoproc_init,
    .vcoproc_deinit      = gx6xxx_vcoproc_deinit,
    .ctx_switch_from     = gx6xxx_ctx_switch_from,
    .ctx_switch_to       = gx6xxx_ctx_switch_to,
};

static int gx6xxx_dt_probe(struct platform_device *pdev)
{
    struct coproc_device *coproc;
    struct device *dev = &pdev->dev;
    struct gx6xxx_info *info;
    char *reg_base;
    int ret;

    coproc = coproc_alloc(pdev, &gx6xxx_vcoproc_ops);
    if ( IS_ERR_OR_NULL(coproc) )
        return PTR_ERR(coproc);

    if ( (coproc->num_irqs != GX6XXX_NUM_IRQ) ||
         (coproc->num_mmios != GX6XXX_NUM_MMIO) )
    {
        dev_err(dev, "wrong number of IRQs/MMIOs\n");
        ret = -EINVAL;
        goto out_release_coproc;
    }
    coproc->priv = xzalloc(struct gx6xxx_info);
    if ( !coproc->priv )
    {
        dev_err(dev, "failed to allocate coproc private data\n");
        ret = -ENOMEM;
        goto out_release_priv;
    }
    info = (struct gx6xxx_info *)coproc->priv;
    reg_base = (char *)coproc->mmios[0].base;
    info->reg_vaddr_irq_status = (uint32_t *)(reg_base + RGXFW_CR_IRQ_STATUS);
    info->reg_vaddr_irq_clear = (uint32_t *)(reg_base + RGXFW_CR_IRQ_CLEAR);

    ret = request_irq(coproc->irqs[0], IRQF_SHARED,
                      gx6xxx_irq_handler, "GPU GX6xxx irq", coproc);
    if ( ret )
    {
        dev_err(dev, "failed to request irq (%u)\n", coproc->irqs[0]);
        goto out_release_priv;
    }

    ret = coproc_register(coproc);
    if ( ret )
    {
        dev_err(dev, "failed to register coproc (%d)\n", ret);
        goto out_release_irqs;
    }

    return 0;

out_release_irqs:
    release_irq(coproc->irqs[0], coproc);
out_release_priv:
    xfree(coproc->priv);
out_release_coproc:
    coproc_release(coproc);
    return ret;
}

static __init int gx6xxx_init(struct dt_device_node *dev, const void *data)
{
    int ret;

    dt_device_set_used_by(dev, DOMID_XEN);

    ret = gx6xxx_dt_probe(dev);
    if ( ret )
        return ret;

    return 0;
}

static const struct dt_device_match gx6xxx_dt_match[] __initconst =
{
    DT_MATCH_GX6XXX,
    { /* sentinel */ },
};

DT_DEVICE_START(coproc_gpu_gx6xxx, "COPROC_GPU_GX6XXX", DEVICE_COPROC)
    .dt_match = gx6xxx_dt_match,
    .init = gx6xxx_init,
DT_DEVICE_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

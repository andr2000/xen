#ifndef __ASM_TRACE_H__
#define __ASM_TRACE_H__

/* Trace events for ARM IRQs */

#define TRC_XT                         (TRC_HW_IRQ + 0x800)
#define TRC_XT_IRQ_START               (TRC_XT + 0x1)
#define TRC_XT_IRQ_END                 (TRC_XT + 0x2)
#define TRC_XT_IRQ_CTXT_SW_START       (TRC_XT + 0x3)
#define TRC_XT_IRQ_CTXT_SW_END         (TRC_XT + 0x4)
#define TRC_XT_VIRT_TMR_SAVE_START     (TRC_XT + 0x5)
#define TRC_XT_VIRT_TMR_SAVE_END       (TRC_XT + 0x6)
#define TRC_XT_VIRT_TMR_RESTORE_START  (TRC_XT + 0x7)
#define TRC_XT_VIRT_TMR_RESTORE_END    (TRC_XT + 0x8)

#define DVCPUID(v) ((uint32_t)(((v)->domain->domain_id << 16) + (v)->vcpu_id))

#endif /* __ASM_TRACE_H__ */
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

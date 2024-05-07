#include "bsp_api.h"
#include "usr_nmi_handler.h"

void user_nmi_handler(bsp_grp_irq_t irq)
{
    if (BSP_GRP_IRQ_MPU_STACK == irq)
    {
        __BKPT(0);
    }
}

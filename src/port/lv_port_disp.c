/*********************
 *      INCLUDES
 *********************/
#include "lvgl_thread.h"
#include <stdbool.h>
#include "lv_port_disp.h"
#include "src/display/lv_display_private.h"

/*********************
 *      DEFINES
 *********************/


#define RGB_565_BLACK  (0)
#define RGB_565_RED    (0x1F << 11)
#define RGB_565_GREEN  (0x3F << 5)
#define RGB_565_BLUE   (0x1F << 0)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void glcdc_flush_finish_event(lv_event_t * event);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/





void lv_port_disp_init(void)
{
    fsp_err_t err;
    err = RM_LVGL_PORT_Open(&g_lvgl_port_ctrl, &g_lvgl_port_cfg);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    lv_display_add_event_cb(g_lvgl_port_ctrl.p_lv_display, glcdc_flush_finish_event, LV_EVENT_FLUSH_FINISH, NULL);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void glcdc_flush_finish_event(lv_event_t * event)
{
    FSP_PARAMETER_NOT_USED(event);
    lv_display_t * disp;

    if (LV_EVENT_FLUSH_FINISH == lv_event_get_code(event))
    {
        /* Enable Backlight */
        R_IOPORT_PinWrite(&g_ioport_ctrl, DISP_BLEN, BSP_IO_LEVEL_HIGH);

        disp = lv_event_get_target(event);

        /* now the backlight in enabled, remove the event callback */
        lv_display_remove_event_cb_with_user_data(disp, glcdc_flush_finish_event, NULL);
    }
}


#include "board_init.h"

#include "lvgl.h"
#include "port/lv_port_indev.h"
#include "lvgl/src/drivers/display/renesas_glcdc/lv_renesas_glcdc.h"
#include "common_data.h"


void board_init(void)
{
    /* Display off the backlight */
    R_IOPORT_PinWrite(&g_ioport_ctrl, LCD_DISPON, BSP_IO_LEVEL_LOW);

#if 1
    lv_display_t * disp = lv_renesas_glcdc_direct_create();
#else
    /* WARNING
    Partial render should use a screen size of 1 10th instead of 50th.
    We are running out of memory, so this is only for testing purposes
    */
    static lv_color_t partial_draw_buf[DISPLAY_HSIZE_INPUT0 * DISPLAY_VSIZE_INPUT0 / 50] BSP_PLACE_IN_SECTION(".bss") BSP_ALIGN_VARIABLE(1024);

    lv_display_t * disp = lv_renesas_glcdc_partial_create(partial_draw_buf, NULL, sizeof(partial_draw_buf));
#endif

    /* enable the display */
    R_IOPORT_PinWrite(&g_ioport_ctrl, LCD_DISPON, BSP_IO_LEVEL_HIGH);

    /* Enable the backlight */
    R_IOPORT_PinWrite(&g_ioport_ctrl, DISP_BLEN, BSP_IO_LEVEL_HIGH);

    lv_display_set_default(disp);

    lv_port_indev_init();
}

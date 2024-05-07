/*********************
 *      INCLUDES
 *********************/
#include "lvgl_thread.h"
#include <stdbool.h>
#include "lv_port_disp.h"
#include "lvgl/src/display/lv_display_private.h"

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
static void disp_init(void);
static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void vsync_wait_cb(struct _lv_display_t * disp);


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
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/

    lv_display_t * disp = lv_display_create(DISPLAY_HSIZE_INPUT0, DISPLAY_VSIZE_INPUT0);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_flush_wait_cb(disp, vsync_wait_cb);
    lv_display_set_buffers(disp, &fb_background[0][0], &fb_background[1][0], sizeof(fb_background[0]), LV_DISPLAY_RENDER_MODE_DIRECT);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    fsp_err_t err;
    uint8_t * p_fb = &fb_background[1][0];

    /* Display off the backlight */
    R_IOPORT_PinWrite(&g_ioport_ctrl, LCD_DISPON, BSP_IO_LEVEL_LOW);


    /* Fill the Frame buffer with a colour, to zero out info from previous execution runs */
    uint32_t count;
    uint16_t * p = (uint16_t *)&fb_background[0][0];

    for (count = 0; count < sizeof(fb_background)/2; count++)
    {
        *p++ = RGB_565_BLACK;
    }

    err = R_GLCDC_Open(&g_display0_ctrl, &g_display0_cfg);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0); //TODO: Better error handling
    }

    err = R_GLCDC_Start(&g_display0_ctrl);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0); //TODO: Better error handling
    }

    do
    {
        err =
            R_GLCDC_BufferChange(&g_display0_ctrl,
                                 (uint8_t *) p_fb,
                                 (display_frame_layer_t) 0);
    } while (FSP_ERR_INVALID_UPDATE_TIMING == err);

    /* enable the display */
    R_IOPORT_PinWrite(&g_ioport_ctrl, LCD_DISPON, BSP_IO_LEVEL_HIGH);

    /* Enable the backlight */
    R_IOPORT_PinWrite(&g_ioport_ctrl, DISP_BLEN, BSP_IO_LEVEL_HIGH);
}

void glcdc_callback(display_callback_args_t *p_args)
{
    if (DISPLAY_EVENT_LINE_DETECTION == p_args->event)
    {
#if BSP_CFG_RTOS == 2               // FreeRTOS
       BaseType_t context_switch;

       //
       // Set Vsync semaphore
       //
       xSemaphoreGiveFromISR(_SemaphoreVsync, &context_switch);

       //
       // Return to the highest priority available task
       //
       portYIELD_FROM_ISR(context_switch);
#else
#endif
    }
    else if (DISPLAY_EVENT_GR1_UNDERFLOW == p_args->event)
    {
        __BKPT(0); //Layer 1 Underrun
    }
    else if (DISPLAY_EVENT_GR2_UNDERFLOW == p_args->event)
    {
        __BKPT(0); //Layer 2 Underrun
    }
    else //DISPLAY_EVENT_FRAME_END
    {
        __BKPT(0);
    }

}

static void vsync_wait_cb(lv_display_t * display)
{
    if(!lv_display_flush_is_last(display)) return;

#if BSP_CFG_RTOS == 2              // FreeRTOS
    //
    // If Vsync semaphore has already been set, clear it then wait to avoid tearing
    //
    if (uxSemaphoreGetCount(_SemaphoreVsync))
    {
        xSemaphoreTake(_SemaphoreVsync, 10);
    }

    xSemaphoreTake(_SemaphoreVsync, portMAX_DELAY);
  #endif

}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{

    FSP_PARAMETER_NOT_USED(area);
    //Display the frame buffer pointed by px_map

    if(!lv_display_flush_is_last(display)) return;

    R_GLCDC_BufferChange(&g_display0_ctrl,
            (uint8_t *) px_map,
            (display_frame_layer_t) 0);
}

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "LVGL_thread.h"
#include "touch_ft5x06/touch_ft5x06.h"


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_t * indev, lv_indev_data_t * data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(int32_t * x, int32_t * y, touch_event_t * touch_event);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    /**
     * Here you will find example implementation of input devices supported by LittelvGL:
     *  - Touchpad
     *  - Mouse (with cursor support)
     *  - Keypad (supports GUI usage only with key)
     *  - Encoder (supports GUI usage only with: left, right, push)
     *  - Button (external buttons to press points on the screen)
     *
     *  The `..._read()` function are only examples.
     *  You should shape them according to your hardware
     */


    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad if you have*/
    touchpad_init();

    /*Register a touchpad input device*/
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchpad, touchpad_read);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void touchpad_init(void)
{
    fsp_err_t err;

    ft5x06_init(&g_i2c_touch, BSP_IO_PORT_03_PIN_04);

    /* Enable touch IRQ */
    err = R_ICU_ExternalIrqOpen(g_touch_irq.p_ctrl, g_touch_irq.p_cfg);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    err = R_ICU_ExternalIrqEnable(g_touch_irq.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_t * indev_drv, lv_indev_data_t * data)
{
    FSP_PARAMETER_NOT_USED(indev_drv);
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    static touch_event_t touch_event = TOUCH_EVENT_NONE;

    /*Save the pressed coordinates and the state*/
    if(touchpad_is_pressed()) {
        touchpad_get_xy(&last_x, &last_y, &touch_event);
        if ((TOUCH_EVENT_DOWN == touch_event) || (TOUCH_EVENT_MOVE == touch_event))
        {
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;


}

static bool touchpad_is_pressed(void)
{
    BaseType_t status;
    bool touch_pressed = false;

    status = xSemaphoreTake( g_irq_binary_semaphore, 0 );
    if(pdTRUE == status)
    {
        touch_pressed = true;
    }

    return touch_pressed;
}

touch_data_t g_touch_data;

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(int32_t * x, int32_t * y, touch_event_t * touch_event)
{
    ft5x06_payload_get (&g_touch_data);

    *touch_event = g_touch_data.point[0].event;
    *x = g_touch_data.point[0].x;
    *y = g_touch_data.point[0].y;
}

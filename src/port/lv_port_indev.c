/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "lvgl_thread.h"

#define FT5X06_NUM_POINTS    5

#define FT5X06_DOWN          0
#define FT5X06_UP            1
#define FT5X06_CONTACT       2
#define FT5X06_NO_EVENT      3

#define FT5X06_REG_TD_STATUS 0x02


#define I2C_TRANSFER_COMPLETE  (1<<0)
#define I2C_TRANSFER_ABORT     (1<<1)

#define I2C_TIMEOUT_MS         1000/portTICK_PERIOD_MS

typedef struct st_ft5x06_touch
{
    uint8_t  x_msb : 4;
    uint8_t        : 2;
    uint8_t  event : 2;
    uint8_t  x_lsb;

    uint8_t  y_msb : 4;
    uint8_t  id    : 4;
    uint8_t  y_lsb : 8;

    uint8_t  res1;
    uint8_t  res2;
} ft5x06_touch_t;

/* Complete FT5X06 data payload (number of active points + all five touch points) */
typedef struct st_ft5x06_payload
{
    uint8_t        num_points_active;
    ft5x06_touch_t data_raw[FT5X06_NUM_POINTS];
} ft5x06_payload_t;


typedef struct st_touch_coord
{
    uint16_t      x;
    uint16_t      y;
    lv_indev_state_t event;
} touch_coord_t;

typedef struct st_touch_data
{
    uint8_t       num_points;
    touch_coord_t point[FT5X06_NUM_POINTS];
} touch_data_t;

#define extract_e(t) ((uint8_t) ((t).event))
#define extract_x(t) ((int16_t) (((t).x_msb << 8) | ((t).x_lsb)))
#define extract_y(t) ((int16_t) (((t).y_msb << 8) | ((t).y_lsb)))

touch_data_t g_touch_data;

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

void touchpad_read(lv_indev_t * indev, lv_indev_data_t * data);
fsp_err_t i2c_wait(void);
bool touchpad_is_pressed(void);
void touchpad_get_xy(lv_indev_data_t * data);

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

    /*------------------
     * Touchpad
     * -----------------*/
    fsp_err_t err;

    R_BSP_PinAccessEnable();

    /** Reset touch chip by setting GPIO reset pin low. */
    R_BSP_PinWrite(TOUCH_RESET, BSP_IO_LEVEL_LOW);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

    /** Release touch chip from reset */
    R_BSP_PinWrite(TOUCH_RESET, BSP_IO_LEVEL_HIGH);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

    R_BSP_PinAccessDisable();

    /*Register a touchpad input device*/
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchpad, touchpad_read);

#if (1 == INDEV_EVENT_DRIVEN)
    /* Update the input device's running mode to LV_INDEV_MODE_EVENT */
    lv_indev_set_mode(indev_touchpad, LV_INDEV_MODE_EVENT);
#endif


    /* Open the I2C bus if it is not already open. */
    rm_comms_i2c_bus_extended_cfg_t * p_extend          = (rm_comms_i2c_bus_extended_cfg_t *) g_comms_i2c_device0_cfg.p_extend;
    i2c_master_instance_t           * p_driver_instance = (i2c_master_instance_t *) p_extend->p_driver_instance;

    err = p_driver_instance->p_api->open(p_driver_instance->p_ctrl, p_driver_instance->p_cfg);
    assert(FSP_SUCCESS == err);

#if (2 == BSP_CFG_RTOS)
    /* Create a semaphore for blocking if a semaphore is not NULL */
    if (NULL != p_extend->p_blocking_semaphore)
    {
              // FreeRTOS
        *(p_extend->p_blocking_semaphore->p_semaphore_handle) =
            xSemaphoreCreateCountingStatic((UBaseType_t) 1,
                                           (UBaseType_t) 0,
                                           p_extend->p_blocking_semaphore->p_semaphore_memory);
    }
    /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != p_extend->p_bus_recursive_mutex)
    {
            // FreeRTOS
        *(p_extend->p_bus_recursive_mutex->p_mutex_handle) =
            xSemaphoreCreateRecursiveMutexStatic(p_extend->p_bus_recursive_mutex->p_mutex_memory);
    }
#endif

    /* Need to initialise the Touch Controller before the LCD, as only a Single Reset line shared between them */
    err = RM_COMMS_I2C_Open(&g_comms_i2c_device0_ctrl, &g_comms_i2c_device0_cfg);
    assert(FSP_SUCCESS == err);

    err = R_ICU_ExternalIrqOpen(&g_touch_irq_ctrl, &g_touch_irq_cfg);
    assert(FSP_SUCCESS == err);

    err = R_ICU_ExternalIrqEnable(&g_touch_irq_ctrl);
    assert(FSP_SUCCESS == err);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

void touch_irq_callback(external_irq_callback_args_t *p_args)
{
    if (0 == p_args->channel)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Set touch IRQ semaphore */
        /* Unblock the task by releasing the semaphore. */
        xSemaphoreGiveFromISR( g_irq_binary_semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void comms_i2c_callback(rm_comms_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xResult = pdFAIL;

    if (RM_COMMS_EVENT_OPERATION_COMPLETE == p_args->event)
    {
        xResult = xEventGroupSetBitsFromISR(g_i2c_event_group, I2C_TRANSFER_COMPLETE, &xHigherPriorityTaskWoken );
    }
    else if (RM_COMMS_EVENT_ERROR == p_args->event)
    {
        xResult = xEventGroupSetBitsFromISR(g_i2c_event_group, I2C_TRANSFER_ABORT, &xHigherPriorityTaskWoken );
    }
    else
    {
       //should never get here.
    }

    /* Was the message posted successfully? */
    if( pdFAIL != xResult)
    {
        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
        switch should be requested.  The macro used is port specific and will
        be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
        the documentation page for the port being used. */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}

fsp_err_t i2c_wait(void)
{
    fsp_err_t ret = FSP_SUCCESS;
    EventBits_t uxBits;

    uxBits =  xEventGroupWaitBits(g_i2c_event_group,
                    I2C_TRANSFER_COMPLETE | I2C_TRANSFER_ABORT,
                    pdTRUE, //Clearbits before returning
                    pdFALSE, //either bit will do
                    I2C_TIMEOUT_MS  );

    if ((I2C_TRANSFER_COMPLETE & uxBits) == I2C_TRANSFER_COMPLETE)
    {
        ret = FSP_SUCCESS;
    }
    else if ((I2C_TRANSFER_ABORT & uxBits) == I2C_TRANSFER_ABORT)
    {
        ret = FSP_ERR_ABORTED;
    }
    else
    {
        /* xEventGroupWaitBits() returned because of timeout */
        ret = FSP_ERR_TIMEOUT;
    }

    return ret;
}

bool touchpad_is_pressed(void)
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

/*------------------
 * Touchpad
 * -----------------*/

/*Will be called by the library to read the touchpad*/
void touchpad_read(lv_indev_t * indev_drv, lv_indev_data_t * data)
{
    FSP_PARAMETER_NOT_USED(indev_drv);

    data->state = LV_INDEV_STATE_RELEASED;

#if (1 == INDEV_EVENT_DRIVEN)
    touchpad_get_xy(data);
#else
    /*Save the pressed coordinates and the state*/
    static lv_indev_state_t last_state = LV_INDEV_STATE_RELEASED;
    static lv_point_t last_point = {.x = 0, .y = 0};

    if(touchpad_is_pressed()) {
        touchpad_get_xy(data);
        last_state = data->state;
        last_point.x = data->point.x;
        last_point.y = data->point.y;
    }
    else
    {
        data->state   = last_state;
        data->point.x = last_point.x;
        data->point.y = last_point.y;
    }
#endif

}

void touchpad_get_xy(lv_indev_data_t * data)
{

    fsp_err_t err;
    rm_comms_write_read_params_t write_read_params;
    ft5x06_payload_t touch_payload;
    uint8_t event;
    uint8_t reg;
    uint8_t touch_count;

    reg = FT5X06_REG_TD_STATUS;

    write_read_params.src_bytes  = 1;
    write_read_params.p_src      = &reg;
    write_read_params.p_dest     = (uint8_t *)&touch_payload;
    write_read_params.dest_bytes = sizeof(ft5x06_payload_t);

    /* Write TD_STATUS address */
    /* Read TD_STATUS through all five TOUCHn_** register sets */
    err = RM_COMMS_I2C_WriteRead(&g_comms_i2c_device0_ctrl, write_read_params);
    assert (FSP_SUCCESS == err);

    err = i2c_wait();
    assert (FSP_SUCCESS == err);

    touch_count = touch_payload.num_points_active & 0x0F;

    if (FT5X06_NUM_POINTS < touch_count)
    {
        assert(0);
    }

    g_touch_data.point[0].event = LV_INDEV_STATE_RELEASED;

    /* Process the raw data for the touch point(s) into useful data */
    for(uint8_t i = 0; i < touch_count; i++)
    {
       g_touch_data.point[i].x = (uint16_t) extract_x(touch_payload.data_raw[i]);
       g_touch_data.point[i].y = (uint16_t) extract_y(touch_payload.data_raw[i]);
       event                   = extract_e(touch_payload.data_raw[i]);

       /* Set event type based on received data */
       switch(event)
       {
           case FT5X06_DOWN:
           case FT5X06_CONTACT:
               g_touch_data.point[i].event = LV_INDEV_STATE_PRESSED;
               break;
           case FT5X06_UP:
           case FT5X06_NO_EVENT:
           default:
               g_touch_data.point[i].event = LV_INDEV_STATE_RELEASED;
               break;
       }
    }

    data->state = g_touch_data.point[0].event;
    if (LV_INDEV_STATE_PRESSED ==  g_touch_data.point[0].event)
    {
        data->point.x = g_touch_data.point[0].x;
        data->point.y = g_touch_data.point[0].y;
    }

}

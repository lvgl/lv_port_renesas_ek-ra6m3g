#include <string.h>
#include "common_data.h"
#include "touch_ft5x06.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define FT5X06_DOWN          0
#define FT5X06_UP            1
#define FT5X06_CONTACT       2

#define FT5X06_REG_TD_STATUS 0x02

#define extract_e(t) ((uint8_t) ((t).event))
#define extract_x(t) ((int16_t) (((t).x_msb << 8) | ((t).x_lsb)))
#define extract_y(t) ((int16_t) (((t).y_msb << 8) | ((t).y_lsb)))

/**********************************************************************************************************************
 * Type definitions
 **********************************************************************************************************************/
/* Driver-specific touch point register mapping */
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

#define I2C_TRANSFER_COMPLETE  (1<<0)
#define I2C_TRANSFER_ABORT     (1<<1)

#define I2C_TIMEOUT_MS         1000/portTICK_PERIOD_MS

/**********************************************************************************************************************
 * Static global variables
 **********************************************************************************************************************/
static i2c_master_instance_t const * gp_i2c_instance;
static bsp_io_port_pin_t             g_reset_pin;

/**********************************************************************************************************************
 * Function definitions
 **********************************************************************************************************************/
fsp_err_t i2c_wait(void);

void touch_irq_cb(external_irq_callback_args_t * p_args)
{
    if(0 == p_args->channel)
     {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Set touch IRQ semaphore */
        /* Unblock the task by releasing the semaphore. */
        xSemaphoreGiveFromISR( g_irq_binary_semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}


/* Called from touch i2c isr routine */
void touch_i2c_callback(i2c_master_callback_args_t * p_args)
{
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t xResult = pdFAIL;

      /* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
      xHigherPriorityTaskWoken = pdFALSE;

    if ((I2C_MASTER_EVENT_TX_COMPLETE == p_args->event) || (I2C_MASTER_EVENT_RX_COMPLETE == p_args->event))
    {
        xResult = xEventGroupSetBitsFromISR(g_i2c_event_group, I2C_TRANSFER_COMPLETE, &xHigherPriorityTaskWoken );
    }
    else if (I2C_MASTER_EVENT_ABORTED == p_args->event)
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

/*******************************************************************************************************************//**
 * Basic function to wait for I2C comms completion
 **********************************************************************************************************************/

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

/*******************************************************************************************************************//**
 * Reset the FT5X06
 **********************************************************************************************************************/
static void ft5x06_reset ()
{
    R_BSP_PinAccessEnable();

    /** Reset touch chip by setting GPIO reset pin low. */
    R_BSP_PinWrite(g_reset_pin, BSP_IO_LEVEL_LOW);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

    /** Release touch chip from reset */
    R_BSP_PinWrite(g_reset_pin, BSP_IO_LEVEL_HIGH);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);

    R_BSP_PinAccessDisable();
}

/*******************************************************************************************************************//**
 * Initialize the connection with the FT5X06 touch controller
 *
 * @param      p_i2c_instance  I2C Master instance to use for communication
 * @param      i2c_semaphore   Semaphore indicating I2C completion
 * @param[in]  reset_pin       Pin connected to FT5X06 reset line
 **********************************************************************************************************************/
fsp_err_t ft5x06_init(i2c_master_instance_t const * p_i2c_instance, bsp_io_port_pin_t reset_pin)
{
    /* Initialize control variables */
    gp_i2c_instance = p_i2c_instance;
    g_reset_pin     = reset_pin;
    fsp_err_t err;

    /* Reset FT5X06 controller */
    ft5x06_reset();

    /* Open I2C peripheral */
    err = R_IIC_MASTER_Open(gp_i2c_instance->p_ctrl, gp_i2c_instance->p_cfg);

    return err;
}

/*******************************************************************************************************************//**
 * Get all touch data from the FT5X06 touch controller
 * @param      touch_data      Pointer to struct for output touch data
 **********************************************************************************************************************/
void ft5x06_payload_get (touch_data_t * touch_data)
{
    touch_coord_t    new_touch;
    ft5x06_payload_t touch_payload;
    fsp_err_t err;

    /* Clear payload struct */
    memset(&touch_payload, 0, sizeof(ft5x06_payload_t));

    /* Read the data about the touch point(s) */
    uint8_t reg = FT5X06_REG_TD_STATUS;

    /* Write TD_STATUS address */
    err = R_IIC_MASTER_Write(gp_i2c_instance->p_ctrl, &reg, 1, true);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    err = i2c_wait();
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    /* Read TD_STATUS through all five TOUCHn_** register sets */
    err = R_IIC_MASTER_Read(gp_i2c_instance->p_ctrl, (uint8_t *) &touch_payload, sizeof(ft5x06_payload_t), false);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    err = i2c_wait();
    if (FSP_SUCCESS != err)
    {
        __BKPT(0);
    }

    /* Process the raw data for the touch point(s) into useful data */
    for(uint8_t i = 0; i < FT5X06_NUM_POINTS; i++)
    {
        new_touch.x     = (uint16_t) extract_x(touch_payload.data_raw[i]);
        new_touch.y     = (uint16_t) extract_y(touch_payload.data_raw[i]);
        new_touch.event = extract_e(touch_payload.data_raw[i]);

        /* Set event type based on received data */
        switch(new_touch.event)
        {
            case FT5X06_DOWN:
                touch_data->point[i].event = TOUCH_EVENT_DOWN;
                break;
            case FT5X06_UP:
                touch_data->point[i].event = TOUCH_EVENT_UP;
                break;
            case FT5X06_CONTACT:
                /* Check if the point is moving or not */
                if ((touch_data->point[i].x != new_touch.x) || (touch_data->point[i].y != new_touch.y))
                {
                    touch_data->point[i].event = TOUCH_EVENT_MOVE;
                }
                else
                {
                    touch_data->point[i].event = TOUCH_EVENT_HOLD;
                }
                break;
            default:
                touch_data->point[i].event = TOUCH_EVENT_NONE;
                break;
        }

        /* Set new coordinates */
        touch_data->point[i].x = new_touch.x;
        touch_data->point[i].y = new_touch.y;
    }

    /* Pass the number of active touch points through */
    touch_data->num_points = touch_payload.num_points_active;
}

/**
 ******************************************************************************
 * @file     main.c
 * @author   正点原子团队(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    新建工程实验-HAL库版本 实验
 * @license  Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 STM32F103 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 ******************************************************************************
 */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

#include "./bsp/bsp_led.h" 
#include "./bsp/bsp_beep.h"   
#include "./bsp/bsp_key.h"
#include "./bsp/bsp_key_exti.h"
#include "./bsp/bsp_i2c_ee.h"
#include "./bsp/bsp_spi_flash.h"
#include "./FatFs/ff.h"
#include "./bsp/bsp_FatFs_test.h"
#include "./bsp/bsp_sram.h"  
#include "./bsp/bsp_ili9341_lcd.h"
#include "./bsp/bsp_xpt2046_lcd.h"
#include "./bsp/bsp_adc.h"
#include "./bsp/bsp_touchpad.h"
#include "./bsp/bsp_iwdg.h"  
#include "./bsp/bsp_wwdg.h"   
#include "./bsp/bsp_rtc.h"
#include "./bsp/bsp_internal_flash.h"  
#include "./bsp/sdio_test.h"

#include "FreeRTOS.h"
#include "task.h"


/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t Test_Task_Handle = NULL;/* LED任务句柄 */
static TaskHandle_t KEY_Task_Handle = NULL;/* KEY任务句柄 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */


/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
 
 
/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */
static void Test_Task(void* pvParameters);/* Test_Task任务实现 */
static void KEY_Task(void* pvParameters);/* KEY_Task任务实现 */
static void BSP_Init(void);/* 用于初始化板载相关资源 */


/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{
    /*系统初始化*/
    HAL_Init();                              /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);      /* 设置时钟, 72Mhz */
    delay_init(72);                          /* 延时初始化 */
    usart_init(115200);                      /* 串口初始化为115200 */
    
    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
    /* 开发板硬件初始化 */
    BSP_Init();

    printf("这是一个[野火]-STM32全系列开发板-FreeRTOS固件库实验！\n\n");
    printf("按下KEY1挂起任务，按下KEY2恢复任务\n");

    /* 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
    /* 启动任务调度 */           
    if(pdPASS == xReturn)
        vTaskStartScheduler();   /* 启动任务，开启调度 */
    else
        return -1;

    while(1)
    { 
        ;
    }
}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	
    /*bsp初始化*/
    LED_GPIO_Config();
    BEEP_GPIO_Config();
    Key_GPIO_Config();     
//    EXTI_Key_Config();
//    I2C_EE_Init();           /* I2C 外设初(AT24C02)始化 */
//    SPI_FLASH_Init();        /* 8M串行flash W25Q64初始化 */
//    FSMC_SRAM_Init();        /* 初始化外部SRAM */     
//    ILI9341_Init ();         /* LCD 初始化 */
//    XPT2046_Init();          /* 触摸屏初始化 */
//    ADCx_Init();             /* adc初始化 */
//    TPAD_Init();             /* 初始化电容按键获取未被触摸时参数 */
//    IWDG_Config(IWDG_PRESCALER_64 ,625);    /* IWDG 1s 超时溢出 */
//    WWDG_Config(0x7F,0X5F,WWDG_PRESCALER_8);   /* 初始化WWDG：配置计数器初始值，配置上窗口值，启动WWDG，使能提前唤醒中断 */
//    RTC_CLK_Config();        /* RTC配置：选择时钟源，设置RTC_CLK的分频系数 */

}


/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  taskENTER_CRITICAL();           //进入临界区
  
  /* 创建Test_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Test_Task, /* 任务入口函数 */
                        (const char*    )"Test_Task",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&Test_Task_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    printf("创建Test_Task任务成功!\r\n");
  /* 创建KEY_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* 任务入口函数 */
                        (const char*    )"KEY_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建KEY_Task任务成功!\r\n");
  
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}
/**********************************************************************
  * @ 函数名  ： Test_Task
  * @ 功能说明： Test_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Test_Task(void* parameter)
{	
  while (1)
  {
    LED1_ON;  
    printf("Test_Task Running,LED1_ON\r\n");  
    vTaskDelay(500);   /* 延时500个tick */
    
    LED1_OFF;     
    printf("Test_Task Running,LED1_OFF\r\n");
    vTaskDelay(500);   /* 延时500个tick */
  }
}
/**********************************************************************
  * @ 函数名  ： Test_Task
  * @ 功能说明： Test_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void KEY_Task(void* parameter)
{	
  uint8_t led_suspend_flag = 0;
  while (1)
  {
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
    {/* K1 被按下 */
      if(led_suspend_flag == 0)
      {
          printf("挂起Test任务！\n");   
          vTaskSuspend(Test_Task_Handle);/* 挂起LED任务 */
          led_suspend_flag = 1;
          printf("挂起Test任务成功！\n");
          BEEP(IO_ON);  
          vTaskDelay(50);  
          BEEP(IO_OFF);  
      }
      else
      {
          printf("恢复Test任务！\n");
          vTaskResume(Test_Task_Handle);/* 恢复LED任务！ */
          led_suspend_flag = 0;
          printf("恢复Test任务成功！\n");
          BEEP(IO_ON);  
          vTaskDelay(50);  
          BEEP(IO_OFF);  
      } 
    } 
    vTaskDelay(20);/* 延时20个tick */
  }
}

void bsp_test(void)
{
    /*测试函数*/
//    I2C_Test();
//    SPI_Test();
//    FatFs_Test();
//    my_SRAM_Test();
//    LCD_Test();         //LCD测试      放到while(1)中测试 需禁用看门狗
//    Touch_Lcd_Test();   //触摸屏测试   放到while(1)中测试 需禁用看门狗
//    ADC_Test();         //adc测试      放到while(1)中测试 需禁用看门狗
//    Touch_key_Test();   //触摸按键测试 放到while(1)中测试 需禁用看门狗  
//    Rtc_Test();
//    Internal_Flash_Test();
//    SD_Test();
    
//        LED_RED_TOGGLE();
//        delay_ms(500); 
//        HAL_IWDG_Refresh(&IWDG_Handle);                              //独立看门狗喂狗
//        uint16_t wwdg_count = get_wwdg_cout();                          
//        if(wwdg_count <= 0X5F) //需要在计数值为0x5F到0x40之间时喂狗  
//        {                                                           
//            WWDG_Feed();                                             //窗口看门狗喂狗
//        }
}



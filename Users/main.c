/**
 ******************************************************************************
 * @file     main.c
 * @author   ����ԭ���Ŷ�(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    �½�����ʵ��-HAL��汾 ʵ��
 * @license  Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ******************************************************************************
 * @attention
 * 
 * ʵ��ƽ̨:����ԭ�� STM32F103 ������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
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


/**************************** ������ ********************************/
/* 
 * ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 * �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 * ����������ΪNULL��
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* ���������� */
static TaskHandle_t Test_Task_Handle = NULL;/* LED������ */
static TaskHandle_t KEY_Task_Handle = NULL;/* KEY������ */

/********************************** �ں˶����� *********************************/
/*
 * �ź�������Ϣ���У��¼���־�飬�����ʱ����Щ�������ں˵Ķ���Ҫ��ʹ����Щ�ں�
 * ���󣬱����ȴ����������ɹ�֮��᷵��һ����Ӧ�ľ����ʵ���Ͼ���һ��ָ�룬������
 * �ǾͿ���ͨ��������������Щ�ں˶���
 *
 * �ں˶���˵���˾���һ��ȫ�ֵ����ݽṹ��ͨ����Щ���ݽṹ���ǿ���ʵ��������ͨ�ţ�
 * �������¼�ͬ���ȸ��ֹ��ܡ�������Щ���ܵ�ʵ��������ͨ��������Щ�ں˶���ĺ���
 * ����ɵ�
 * 
 */


/******************************* ȫ�ֱ������� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */
 
 
/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */
static void Test_Task(void* pvParameters);/* Test_Task����ʵ�� */
static void KEY_Task(void* pvParameters);/* KEY_Task����ʵ�� */
static void BSP_Init(void);/* ���ڳ�ʼ�����������Դ */


/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   ��һ����������Ӳ����ʼ�� 
            �ڶ���������APPӦ������
            ������������FreeRTOS����ʼ���������
  ****************************************************************/
int main(void)
{
    /*ϵͳ��ʼ��*/
    HAL_Init();                              /* ��ʼ��HAL�� */
    sys_stm32_clock_init(RCC_PLL_MUL9);      /* ����ʱ��, 72Mhz */
    delay_init(72);                          /* ��ʱ��ʼ�� */
    usart_init(115200);                      /* ���ڳ�ʼ��Ϊ115200 */
    
    BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
    /* ������Ӳ����ʼ�� */
    BSP_Init();

    printf("����һ��[Ұ��]-STM32ȫϵ�п�����-FreeRTOS�̼���ʵ�飡\n\n");
    printf("����KEY1�������񣬰���KEY2�ָ�����\n");

    /* ����AppTaskCreate���� */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 
    /* ����������� */           
    if(pdPASS == xReturn)
        vTaskStartScheduler();   /* �������񣬿������� */
    else
        return -1;

    while(1)
    { 
        ;
    }
}

/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� �弶�����ʼ�������а����ϵĳ�ʼ�����ɷ��������������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
	 * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
	 * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
	 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	
    /*bsp��ʼ��*/
    LED_GPIO_Config();
    BEEP_GPIO_Config();
    Key_GPIO_Config();     
//    EXTI_Key_Config();
//    I2C_EE_Init();           /* I2C �����(AT24C02)ʼ�� */
//    SPI_FLASH_Init();        /* 8M����flash W25Q64��ʼ�� */
//    FSMC_SRAM_Init();        /* ��ʼ���ⲿSRAM */     
//    ILI9341_Init ();         /* LCD ��ʼ�� */
//    XPT2046_Init();          /* ��������ʼ�� */
//    ADCx_Init();             /* adc��ʼ�� */
//    TPAD_Init();             /* ��ʼ�����ݰ�����ȡδ������ʱ���� */
//    IWDG_Config(IWDG_PRESCALER_64 ,625);    /* IWDG 1s ��ʱ��� */
//    WWDG_Config(0x7F,0X5F,WWDG_PRESCALER_8);   /* ��ʼ��WWDG�����ü�������ʼֵ�������ϴ���ֵ������WWDG��ʹ����ǰ�����ж� */
//    RTC_CLK_Config();        /* RTC���ã�ѡ��ʱ��Դ������RTC_CLK�ķ�Ƶϵ�� */

}


/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
  taskENTER_CRITICAL();           //�����ٽ���
  
  /* ����Test_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )Test_Task, /* ������ں��� */
                        (const char*    )"Test_Task",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&Test_Task_Handle);/* ������ƿ�ָ�� */
  if(pdPASS == xReturn)
    printf("����Test_Task����ɹ�!\r\n");
  /* ����KEY_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* ������ں��� */
                        (const char*    )"KEY_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("����KEY_Task����ɹ�!\r\n");
  
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}
/**********************************************************************
  * @ ������  �� Test_Task
  * @ ����˵���� Test_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void Test_Task(void* parameter)
{	
  while (1)
  {
    LED1_ON;  
    printf("Test_Task Running,LED1_ON\r\n");  
    vTaskDelay(500);   /* ��ʱ500��tick */
    
    LED1_OFF;     
    printf("Test_Task Running,LED1_OFF\r\n");
    vTaskDelay(500);   /* ��ʱ500��tick */
  }
}
/**********************************************************************
  * @ ������  �� Test_Task
  * @ ����˵���� Test_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void KEY_Task(void* parameter)
{	
  uint8_t led_suspend_flag = 0;
  while (1)
  {
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
    {/* K1 ������ */
      if(led_suspend_flag == 0)
      {
          printf("����Test����\n");   
          vTaskSuspend(Test_Task_Handle);/* ����LED���� */
          led_suspend_flag = 1;
          printf("����Test����ɹ���\n");
          BEEP(IO_ON);  
          vTaskDelay(50);  
          BEEP(IO_OFF);  
      }
      else
      {
          printf("�ָ�Test����\n");
          vTaskResume(Test_Task_Handle);/* �ָ�LED���� */
          led_suspend_flag = 0;
          printf("�ָ�Test����ɹ���\n");
          BEEP(IO_ON);  
          vTaskDelay(50);  
          BEEP(IO_OFF);  
      } 
    } 
    vTaskDelay(20);/* ��ʱ20��tick */
  }
}

void bsp_test(void)
{
    /*���Ժ���*/
//    I2C_Test();
//    SPI_Test();
//    FatFs_Test();
//    my_SRAM_Test();
//    LCD_Test();         //LCD����      �ŵ�while(1)�в��� ����ÿ��Ź�
//    Touch_Lcd_Test();   //����������   �ŵ�while(1)�в��� ����ÿ��Ź�
//    ADC_Test();         //adc����      �ŵ�while(1)�в��� ����ÿ��Ź�
//    Touch_key_Test();   //������������ �ŵ�while(1)�в��� ����ÿ��Ź�  
//    Rtc_Test();
//    Internal_Flash_Test();
//    SD_Test();
    
//        LED_RED_TOGGLE();
//        delay_ms(500); 
//        HAL_IWDG_Refresh(&IWDG_Handle);                              //�������Ź�ι��
//        uint16_t wwdg_count = get_wwdg_cout();                          
//        if(wwdg_count <= 0X5F) //��Ҫ�ڼ���ֵΪ0x5F��0x40֮��ʱι��  
//        {                                                           
//            WWDG_Feed();                                             //���ڿ��Ź�ι��
//        }
}



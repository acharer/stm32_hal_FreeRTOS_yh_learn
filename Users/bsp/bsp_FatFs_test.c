#include "./bsp/bsp_FatFs_test.h"
#include "./FatFs/ff.h"
#include "./SYSTEM/usart/usart.h"

BYTE work[FF_MAX_SS];//格式化设备工作区

FATFS fs;						/* FatFs文件系统对象 */
FIL fnew;						/* 文件对象 */
FRESULT res_flash;              /* 文件操作结果 */
UINT fnum;            		    /* 文件成功读写数量 */
BYTE ReadBuffer[1024]={0};      /* 读缓冲区 */
BYTE WriteBuffer[] = "欢迎使用野火STM32开发板 今天是个好日子，新建文件系统测试文件\r\n"; 

uint8_t FatFs_Test(void)
{
    printf("****** 这是一个SPI FLASH 文件系统实验 ******\r\n");
  
	//在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化
	//初始化函数调用流程如下
	//f_mount()->find_volume()->disk_initialize->SPI_FLASH_Init()
	res_flash = f_mount(&fs,"1:",1);
	
    /*------------------------------------------------------- 格式化测试 ------------------------------------------------------*/  
	/* 如果没有文件系统就格式化创建创建文件系统 */
	if(res_flash == FR_NO_FILESYSTEM)
	{
		printf("》FLASH还没有文件系统，即将进行格式化...\r\n");
        /* 格式化 */
		res_flash=f_mkfs("1:", 0, work, sizeof(work));						
		
		if(res_flash == FR_OK)
		{
			printf("》FLASH已成功格式化文件系统。\r\n");
            /* 格式化后，先取消挂载 */
			res_flash = f_mount(NULL,"1:",1);			
            /* 重新挂载	*/			
			res_flash = f_mount(&fs,"1:",1);
		}
		else
		{
			printf("《《格式化失败。》》\r\n");
			while(1);
		}
	}
    else if(res_flash!=FR_OK)
    {
        printf("！！外部Flash挂载文件系统失败。(%d)\r\n",res_flash);
        printf("请下载 SPI—读写串行FLASH 例程测试，如果正常，在该例程f_mount语句下if语句前临时多添加一句 res_flash = FR_NO_FILESYSTEM; 让重新直接执行格式化流程\r\n");
        while(1);
    }
    else
    {
        printf("》文件系统挂载成功，可以进行读写测试\r\n");
    }
  
    /*------------------------------------------------------- 文件系统测试：写测试 ------------------------------------------------------*/
	/* 打开文件，每次都以新建的形式打开，属性为可写 */
	printf("\r\n****** 即将进行文件写入测试... ******\r\n");	
	res_flash = f_open(&fnew, "1:FatFs读写测试文件.txt",FA_CREATE_ALWAYS | FA_WRITE );
	if ( res_flash == FR_OK )
	{
		printf("》打开/创建FatFs读写测试文件.txt文件成功，向文件写入数据。\r\n");
        /* 将指定存储区内容写入到文件内 */
		res_flash=f_write(&fnew,WriteBuffer,sizeof(WriteBuffer),&fnum);
        if(res_flash==FR_OK)
        {
          printf("》文件写入成功，写入字节数据：%d\n",fnum);
          printf("》向文件写入的数据为：\r\n%s\r\n",WriteBuffer);
        }
        else
        {
          printf("！！文件写入失败：(%d)\n",res_flash);
        }    
    /* 不再读写，关闭文件 */
        f_close(&fnew);
    }
    else
    {	
        printf("！！打开/创建文件失败。\r\n");
    }
	
    /*------------------------------------------------------- 文件系统测试：读测试 ------------------------------------------------------*/
	printf("****** 即将进行文件读取测试... ******\r\n");
	res_flash = f_open(&fnew, "1:FatFs读写测试文件.txt",FA_OPEN_EXISTING | FA_READ); 	 
	if(res_flash == FR_OK)
	{
		printf("》打开文件成功。\r\n");
		res_flash = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum); 
        if(res_flash==FR_OK)
        {
            printf("》文件读取成功,读到字节数据：%d\r\n",fnum);
            printf("》读取得的文件数据为：\r\n%s \r\n", ReadBuffer);	
        }
        else
        {
            printf("！！文件读取失败：(%d)\n",res_flash);
            return 0;
        }		
	}
	else
	{
		printf("！！打开文件失败。\r\n");
        return 0;
	}
	/* 不再读写，关闭文件 */
	f_close(&fnew);	
  
	/* 不再使用文件系统，取消挂载文件系统 */
	f_mount(NULL,"1:",1);
    return 1;
}



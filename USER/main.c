#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "string.h"		
#include "math.h"
#include "ov2640.h"
#include "dcmi.h"
#include "usmart.h"
//#include "sdmmc_sdcard.h"
//#include "pcf8574.h"
//#include "malloc.h"
//#include "ff.h"
//#include "exfuns.h"
//#include "fontupd.h"
//#include "text.h"
//#include "piclib.h"	
//#include "lcd.h"
//#include "ltdc.h"
//#include "sdram.h"
//#include "w25qxx.h"
//#include "nand.h"  
//#include "mpu.h"
/************************************************
 ALIENTEK 阿波罗STM32F7开发板 实验46
 照相机实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

vu8 bmp_request=0;						//bmp拍照请求:0,无bmp拍照请求;1,有bmp拍照请求,需要在帧中断里面,关闭DCMI接口.
u8 ovx_mode=1;							//bit0:0,RGB565模式;1,JPEG模式 
u16 curline=0;							//摄像头输出数据,当前行编号
u16 yoffset=0;							//y方向的偏移量

#define jpeg_buf_size   48*1024		//定义JPEG数据缓存jpeg_buf的大小(4M字节)->48K
#define jpeg_line_size	2*1024			//定义DMA接收数据时,一行数据的最大值->2K

u32 *dcmi_line_buf[2];					//RGB屏时,摄像头采用一行一行读取,定义行缓存  
u32 *jpeg_data_buf;						//JPEG数据缓存buf 

volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										//0,数据没有采集完;
										//1,数据采集完了,但是还没处理;
										//2,数据已经处理完成了,可以开始下一帧接
uint32_t JpegBuffer0[jpeg_line_size/4];  //2k的jpeg的缓冲内存0
uint32_t JpegBuffer1[jpeg_line_size/4];  //2k的jpeg的缓冲内存1
uint32_t JpegBuf[jpeg_buf_size/4];
void REDLEDToggle(void);
void BLUELEDToggle(void);
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
void jpeg_data_process(void)
{
	u16 i;
	u16 rlen;			//剩余数据长度
	u32 *pbuf;
	BLUELEDToggle();
	curline=yoffset;	//行数复位
	if(ovx_mode&0X01)	//只有在JPEG格式下,才需要做处理.
	{
		printf("jpeg_data_ok is %d\r\n",jpeg_data_ok);
		if(jpeg_data_ok==0)	//jpeg数据还未采集完?
		{
            __HAL_DMA_DISABLE(&DMADMCI_Handler);//关闭DMA
            while(DMA2_Stream1->CR&0X01);	//等待DMA2_Stream1可配置 
			rlen=jpeg_line_size-__HAL_DMA_GET_COUNTER(&DMADMCI_Handler);//得到剩余数据长度	
			pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾,继续添加
			if(DMADMCI_Handler.Instance->CR&(1<<19))for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的剩余数据
			else for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的剩余数据 
			jpeg_data_len+=rlen;			//加上剩余长度
			
			jpeg_data_ok=1; 				//标记JPEG数据采集完按成,等待其他函数处理
		}
		if(jpeg_data_ok==2)	//上一次的jpeg数据已经被处理了
		{
            __HAL_DMA_SET_COUNTER(&DMADMCI_Handler,jpeg_line_size);	//传输长度为jpeg_buf_size*4字节
			__HAL_DMA_ENABLE(&DMADMCI_Handler); //打开DMA
			jpeg_data_ok=0;					//标记数据未采集
			jpeg_data_len=0;				//数据重新开始
		}
	}else
	{  
	}  
}

//jpeg数据接收回调函数
void jpeg_dcmi_rx_callback(void)
{  	
	u16 i;
	u32 *pbuf;
	REDLEDToggle();
	pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾
  //  printf("jpeg_dcmi_rx_callback,data_len=%d\r\n",jpeg_data_len);
	if(DMADMCI_Handler.Instance->CR&(1<<19))//buf0已满,正常处理buf1
	{ 
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移
	}else //buf1已满,正常处理buf0
	{
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移 
	} 
    SCB_CleanInvalidateDCache();        //清除无效化DCache
}

//OV5640拍照jpg图片
//返回值:0,成功
//    其他,错误代码
u8 ov2640_jpg_photo(u8 *pname)
{
	//FIL* f_jpg; 
	u8 res=0,headok=0;
	u32 bwr;
	u32 i,jpgstart,jpglen;
	u8* pbuf;
	//todo: 更新DMA以及文件缓冲大小，输出
	//f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//开辟FIL字节的内存区域 
	//if(f_jpg==NULL)return 0XFF;				//内存申请失败.
	ovx_mode=1;
	jpeg_data_ok=0;
	//sw_ov2640_mode();						//切换为OV2640模式 
	OV2640_JPEG_Mode();						//JPEG模式  
	res = OV2640_ImageWin_Set(0,0,1600,1200);
	//printf("OV2640_ImageWin_Set_RetValue: %d\r\n",res);
	res = OV2640_OutSize_Set(1600,1200);          //拍照尺寸为1600*1200
	//printf("OV2640_OutSize_Set_RetValue: %d\r\n",res);
	dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG接收数据回调函数
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,2,1);//DCMI DMA配置  
  printf("Jpeg Data as follows :\r\n");
	while(1){
		DCMI_Start(); 			//启动传输 
		/*while(jpeg_data_ok!=1);	//等待第一帧图片采集完
		jpeg_data_ok=2;			//忽略本帧图片,启动下一帧采集 
		while(jpeg_data_ok!=1);	//等待第二帧图片采集完,第二帧,才保存到SD卡去.
		*/
		while(jpeg_data_ok!=1);
					
		//发送jpeg_data_buf中的数据
		printf("the size of JPEG is %d\r\n",jpeg_data_len);
		for(i=0;i<jpeg_data_len;i++)
			printf("%x",jpeg_data_buf[i]);
		printf("\r\n-----------------------------------------------\r\n");
		jpeg_data_ok =2 ;
		DCMI_Stop(); 			//停止DMA搬运
		break;
	}
	return 0;
	//ovx_mode=0; 
	//sw_sdcard_mode();		//切换为SD卡模式
	//res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//模式0,或者尝试打开失败,则创建新文件	 
	if(res==0)
	{
		//printf("jpeg data size:%d\r\n",jpeg_data_len*4);//串口打印JPEG文件大小
		pbuf=(u8*)jpeg_data_buf;
		jpglen=0;	//设置jpg文件大小为0
		headok=0;	//清除jpg头标记
		for(i=0;i<jpeg_data_len*4;i++)//查找0XFF,0XD8和0XFF,0XD9,获取jpg文件大小
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//找到FF D8
			{
				jpgstart=i;
				headok=1;	//标记找到jpg头(FF D8)
			}
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//找到头以后,再找FF D9
			{
				jpglen=i-jpgstart+2;
				break;
			}
		}
		if(jpglen)			//正常的jpeg数据 
		{
			pbuf+=jpgstart;	//偏移到0XFF,0XD8处
			//res=f_write(f_jpg,pbuf,jpglen,&bwr);
			if(bwr!=jpglen)res=0XFE; 
			//printf("get useful jpeg and the res = %x\r\n",res);
		}else res=0XFD; 
	}
	jpeg_data_len=0;
	//f_close(f_jpg); 
	//sw_ov2640_mode();		//切换为OV2640模式
	//OV2640_RGB565_Mode();	//RGB565模式  
 /*	dcmi_rx_callback = handle_dcmi_callback;
	if(lcdltdc.pwidth!=0)	//RGB屏
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB屏接收数据回调函数
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,1,1);//DCMI DMA配置  
	}else					//MCU 屏
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,1,0);			//DCMI DMA配置,MCU屏,竖屏
	}
	myfree(SRAMIN,f_jpg); */
	return res;
}  

void BLUELEDinit(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();               //使能GPIOB时钟
	  /*Configure GPIO pins : PBPin PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void REDLEDinit(){
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();               //使能GPIOB时钟
	  /*Configure GPIO pins : PBPin PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void BLUELEDToggle(){
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
}

void REDLEDToggle(){
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
}

void LEDtest(){
	BLUELEDToggle();
	HAL_Delay(500);
	REDLEDToggle();
	HAL_Delay(500);
	REDLEDToggle();
	HAL_Delay(500);
	BLUELEDToggle();
}

void OV2640_PCLK(){
	
}
int main(void)
{			
//  u8 ov2640ret ;	
	u8 *pname ;					//带路径的文件名 
	dcmi_line_buf[0] = JpegBuffer0;
	dcmi_line_buf[1] = JpegBuffer1;
	jpeg_data_buf = JpegBuf;
  Write_Through();                //开启强制透写！
  Cache_Enable();                 //打开L1-Cache,和cubemx生成相同
  HAL_Init();				        //初始化HAL库
  SystemClock_Config();   //设置时钟,168Mhz 
  delay_init(168);                //延时初始化
	uart1_init(115200);		        //串口1初始化，逻辑判断暂时不需要串口
	BLUELEDinit();
	//REDLEDinit();
	//LEDtest();
	//printf("usart init over\r\n");
  //send a character to know the status
  while(OV2640_Init()!=0);				    //初始化OV2640
	printf("ov2640 init over\r\n");
	BLUELEDToggle();
	HAL_Delay(500);
	BLUELEDToggle();
	//printf("ov2640ret = %d\r\n",ov2640ret); //输出初始化OV2640后的返回值
    //Show_Str(30,210,230,16,"OV2640 正常",16,0); 
	//自动对焦初始化
	OV2640_RGB565_Mode();	//RGB565模式,切换为JPEG格式
	OV2640_JPEG_Mode();
	OV2640_Light_Mode(0);	//自动模式
	OV2640_Color_Saturation(3);//色彩饱和度0
	OV2640_Brightness(4);	//亮度0
	OV2640_Contrast(3);		//对比度0
	
	DCMI_Init();			//DCMI配置
	ov2640_jpg_photo(pname);  //内部有DMA初始化


}
void HAL_MspInit(void)
{
  //__HAL_RCC_PWR_CLK_ENABLE();
  //__HAL_RCC_SYSCFG_CLK_ENABLE();
  
}


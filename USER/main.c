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
 ALIENTEK ������STM32F7������ ʵ��46
 �����ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

vu8 bmp_request=0;						//bmp��������:0,��bmp��������;1,��bmp��������,��Ҫ��֡�ж�����,�ر�DCMI�ӿ�.
u8 ovx_mode=1;							//bit0:0,RGB565ģʽ;1,JPEGģʽ 
u16 curline=0;							//����ͷ�������,��ǰ�б��
u16 yoffset=0;							//y�����ƫ����

#define jpeg_buf_size   48*1024		//����JPEG���ݻ���jpeg_buf�Ĵ�С(4M�ֽ�)->48K
#define jpeg_line_size	2*1024			//����DMA��������ʱ,һ�����ݵ����ֵ->2K

u32 *dcmi_line_buf[2];					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л���  
u32 *jpeg_data_buf;						//JPEG���ݻ���buf 

volatile u32 jpeg_data_len=0; 			//buf�е�JPEG��Ч���ݳ��� 
volatile u8 jpeg_data_ok=0;				//JPEG���ݲɼ���ɱ�־ 
										//0,����û�вɼ���;
										//1,���ݲɼ�����,���ǻ�û����;
										//2,�����Ѿ����������,���Կ�ʼ��һ֡��
uint32_t JpegBuffer0[jpeg_line_size/4];  //2k��jpeg�Ļ����ڴ�0
uint32_t JpegBuffer1[jpeg_line_size/4];  //2k��jpeg�Ļ����ڴ�1
uint32_t JpegBuf[jpeg_buf_size/4];
void REDLEDToggle(void);
void BLUELEDToggle(void);
//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.
void jpeg_data_process(void)
{
	u16 i;
	u16 rlen;			//ʣ�����ݳ���
	u32 *pbuf;
	BLUELEDToggle();
	curline=yoffset;	//������λ
	if(ovx_mode&0X01)	//ֻ����JPEG��ʽ��,����Ҫ������.
	{
		printf("jpeg_data_ok is %d\r\n",jpeg_data_ok);
		if(jpeg_data_ok==0)	//jpeg���ݻ�δ�ɼ���?
		{
            __HAL_DMA_DISABLE(&DMADMCI_Handler);//�ر�DMA
            while(DMA2_Stream1->CR&0X01);	//�ȴ�DMA2_Stream1������ 
			rlen=jpeg_line_size-__HAL_DMA_GET_COUNTER(&DMADMCI_Handler);//�õ�ʣ�����ݳ���	
			pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ,�������
			if(DMADMCI_Handler.Instance->CR&(1<<19))for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1�����ʣ������
			else for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0�����ʣ������ 
			jpeg_data_len+=rlen;			//����ʣ�೤��
			
			jpeg_data_ok=1; 				//���JPEG���ݲɼ��갴��,�ȴ�������������
		}
		if(jpeg_data_ok==2)	//��һ�ε�jpeg�����Ѿ���������
		{
            __HAL_DMA_SET_COUNTER(&DMADMCI_Handler,jpeg_line_size);	//���䳤��Ϊjpeg_buf_size*4�ֽ�
			__HAL_DMA_ENABLE(&DMADMCI_Handler); //��DMA
			jpeg_data_ok=0;					//�������δ�ɼ�
			jpeg_data_len=0;				//�������¿�ʼ
		}
	}else
	{  
	}  
}

//jpeg���ݽ��ջص�����
void jpeg_dcmi_rx_callback(void)
{  	
	u16 i;
	u32 *pbuf;
	REDLEDToggle();
	pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ
  //  printf("jpeg_dcmi_rx_callback,data_len=%d\r\n",jpeg_data_len);
	if(DMADMCI_Handler.Instance->CR&(1<<19))//buf0����,��������buf1
	{ 
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
		jpeg_data_len+=jpeg_line_size;//ƫ��
	}else //buf1����,��������buf0
	{
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1���������
		jpeg_data_len+=jpeg_line_size;//ƫ�� 
	} 
    SCB_CleanInvalidateDCache();        //�����Ч��DCache
}

//OV5640����jpgͼƬ
//����ֵ:0,�ɹ�
//    ����,�������
u8 ov2640_jpg_photo(u8 *pname)
{
	//FIL* f_jpg; 
	u8 res=0,headok=0;
	u32 bwr;
	u32 i,jpgstart,jpglen;
	u8* pbuf;
	//todo: ����DMA�Լ��ļ������С�����
	//f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//����FIL�ֽڵ��ڴ����� 
	//if(f_jpg==NULL)return 0XFF;				//�ڴ�����ʧ��.
	ovx_mode=1;
	jpeg_data_ok=0;
	//sw_ov2640_mode();						//�л�ΪOV2640ģʽ 
	OV2640_JPEG_Mode();						//JPEGģʽ  
	res = OV2640_ImageWin_Set(0,0,1600,1200);
	//printf("OV2640_ImageWin_Set_RetValue: %d\r\n",res);
	res = OV2640_OutSize_Set(1600,1200);          //���ճߴ�Ϊ1600*1200
	//printf("OV2640_OutSize_Set_RetValue: %d\r\n",res);
	dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG�������ݻص�����
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,2,1);//DCMI DMA����  
  printf("Jpeg Data as follows :\r\n");
	while(1){
		DCMI_Start(); 			//�������� 
		/*while(jpeg_data_ok!=1);	//�ȴ���һ֡ͼƬ�ɼ���
		jpeg_data_ok=2;			//���Ա�֡ͼƬ,������һ֡�ɼ� 
		while(jpeg_data_ok!=1);	//�ȴ��ڶ�֡ͼƬ�ɼ���,�ڶ�֡,�ű��浽SD��ȥ.
		*/
		while(jpeg_data_ok!=1);
					
		//����jpeg_data_buf�е�����
		printf("the size of JPEG is %d\r\n",jpeg_data_len);
		for(i=0;i<jpeg_data_len;i++)
			printf("%x",jpeg_data_buf[i]);
		printf("\r\n-----------------------------------------------\r\n");
		jpeg_data_ok =2 ;
		DCMI_Stop(); 			//ֹͣDMA����
		break;
	}
	return 0;
	//ovx_mode=0; 
	//sw_sdcard_mode();		//�л�ΪSD��ģʽ
	//res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//ģʽ0,���߳��Դ�ʧ��,�򴴽����ļ�	 
	if(res==0)
	{
		//printf("jpeg data size:%d\r\n",jpeg_data_len*4);//���ڴ�ӡJPEG�ļ���С
		pbuf=(u8*)jpeg_data_buf;
		jpglen=0;	//����jpg�ļ���СΪ0
		headok=0;	//���jpgͷ���
		for(i=0;i<jpeg_data_len*4;i++)//����0XFF,0XD8��0XFF,0XD9,��ȡjpg�ļ���С
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//�ҵ�FF D8
			{
				jpgstart=i;
				headok=1;	//����ҵ�jpgͷ(FF D8)
			}
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//�ҵ�ͷ�Ժ�,����FF D9
			{
				jpglen=i-jpgstart+2;
				break;
			}
		}
		if(jpglen)			//������jpeg���� 
		{
			pbuf+=jpgstart;	//ƫ�Ƶ�0XFF,0XD8��
			//res=f_write(f_jpg,pbuf,jpglen,&bwr);
			if(bwr!=jpglen)res=0XFE; 
			//printf("get useful jpeg and the res = %x\r\n",res);
		}else res=0XFD; 
	}
	jpeg_data_len=0;
	//f_close(f_jpg); 
	//sw_ov2640_mode();		//�л�ΪOV2640ģʽ
	//OV2640_RGB565_Mode();	//RGB565ģʽ  
 /*	dcmi_rx_callback = handle_dcmi_callback;
	if(lcdltdc.pwidth!=0)	//RGB��
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB���������ݻص�����
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,1,1);//DCMI DMA����  
	}else					//MCU ��
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,1,0);			//DCMI DMA����,MCU��,����
	}
	myfree(SRAMIN,f_jpg); */
	return res;
}  

void BLUELEDinit(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();               //ʹ��GPIOBʱ��
	  /*Configure GPIO pins : PBPin PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void REDLEDinit(){
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();               //ʹ��GPIOBʱ��
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
	u8 *pname ;					//��·�����ļ��� 
	dcmi_line_buf[0] = JpegBuffer0;
	dcmi_line_buf[1] = JpegBuffer1;
	jpeg_data_buf = JpegBuf;
  Write_Through();                //����ǿ��͸д��
  Cache_Enable();                 //��L1-Cache,��cubemx������ͬ
  HAL_Init();				        //��ʼ��HAL��
  SystemClock_Config();   //����ʱ��,168Mhz 
  delay_init(168);                //��ʱ��ʼ��
	uart1_init(115200);		        //����1��ʼ�����߼��ж���ʱ����Ҫ����
	BLUELEDinit();
	//REDLEDinit();
	//LEDtest();
	//printf("usart init over\r\n");
  //send a character to know the status
  while(OV2640_Init()!=0);				    //��ʼ��OV2640
	printf("ov2640 init over\r\n");
	BLUELEDToggle();
	HAL_Delay(500);
	BLUELEDToggle();
	//printf("ov2640ret = %d\r\n",ov2640ret); //�����ʼ��OV2640��ķ���ֵ
    //Show_Str(30,210,230,16,"OV2640 ����",16,0); 
	//�Զ��Խ���ʼ��
	OV2640_RGB565_Mode();	//RGB565ģʽ,�л�ΪJPEG��ʽ
	OV2640_JPEG_Mode();
	OV2640_Light_Mode(0);	//�Զ�ģʽ
	OV2640_Color_Saturation(3);//ɫ�ʱ��Ͷ�0
	OV2640_Brightness(4);	//����0
	OV2640_Contrast(3);		//�Աȶ�0
	
	DCMI_Init();			//DCMI����
	ov2640_jpg_photo(pname);  //�ڲ���DMA��ʼ��


}
void HAL_MspInit(void)
{
  //__HAL_RCC_PWR_CLK_ENABLE();
  //__HAL_RCC_SYSCFG_CLK_ENABLE();
  
}


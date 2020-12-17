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
#include "tim.h"
vu8 bmp_request=0;						//bmp��������:0,��bmp��������;1,��bmp��������,��Ҫ��֡�ж�����,�ر�DCMI�ӿ�.
u8 ovx_mode=1;							//bit0:0,RGB565ģʽ;1,JPEGģʽ 
u16 curline=0;							//����ͷ�������,��ǰ�б��
u16 yoffset=0;							//y�����ƫ����

#define jpeg_buf_size   12*1024/4		//����JPEG���ݻ���jpeg_buf�Ĵ�С(4M�ֽ�)->12K
#define jpeg_line_size	400/4			//����DMA��������ʱ,һ�����ݵ����ֵ->2KB

u32 *dcmi_line_buf[2];					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л���  
u32 *jpeg_data_buf;						//JPEG���ݻ���buf 

volatile u32 jpeg_data_len=0; 			//buf�е�JPEG��Ч���ݳ��� 
volatile u8 jpeg_data_ok=0;				//JPEG���ݲɼ���ɱ�־ 
										//0,����û�вɼ���;
										//1,���ݲɼ�����,���ǻ�û����;
										//2,�����Ѿ����������,���Կ�ʼ��һ֡��
uint32_t JpegBuffer0[jpeg_line_size];  //2k��jpeg�Ļ����ڴ�0
//uint32_t JpegBuffer1[jpeg_line_size];  //2k��jpeg�Ļ����ڴ�1
uint32_t JpegBuf[jpeg_buf_size];
void REDLEDToggle(void);
void BLUELEDToggle(void);
//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.
void jpeg_data_process(void)
{
	u16 i;
	u16 rlen;			//ʣ�����ݳ���
	u32 *pbuf;
	//printf("\r\nVSYNC\r\n");
//	DCMI_Start();
//      __HAL_DMA_DISABLE(&DMADMCI_Handler);//�ر�DMA
//      while(DMA2_Stream1->CR&0X01);	//�ȴ�DMA2_Stream1������ 
//			rlen=(jpeg_line_size-__HAL_DMA_GET_COUNTER(&DMADMCI_Handler));//�õ�ʣ�����ݳ���	
//			printf("\r\nrlen = %d\r\n",rlen);
			//���������
			//	pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ,�������
//			if(DMADMCI_Handler.Instance->CR&(1<<19))
//			{		
//				for(i=0;i<rlen;i++){
//					pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf1�����ʣ������
//					printf("%x",pbuf[i]);
//				}
//			}
//			else 
//			{
//				for(i=0;i<rlen;i++){
//					pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0�����ʣ������ 
//					printf("%x",pbuf[i]);
//				}
//			}
//			jpeg_data_len+=rlen;			//����ʣ�೤��
//			printf("\r\nVSdata\r\n");
      REDLEDToggle();
			jpeg_data_ok=1; 				//���JPEG���ݲɼ��갴��,�ȴ�������������
}

//jpeg���ݽ��ջص�����
void jpeg_dcmi_rx_callback(void)
{
//	u16 i; 
//	u32 *pbuf;
//	u32 size = jpeg_line_size;
//	//REDLEDToggle();
//	pbuf=jpeg_data_buf;//ƫ�Ƶ���Ч����ĩβ
	/*
	//  printf("jpeg_dcmi_rx_callback,data_len=%d\r\n",jpeg_data_len);
	if(DMADMCI_Handler.Instance->CR&(1<<19))//buf0����,��������buf1
	{ 
		printf("\r\n---------buf0-----------\r\n");
		for(i=0;i<size;i++){
			pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
			printf("%x",pbuf[i]);
		}
		
		jpeg_data_len+=jpeg_line_size;//ƫ��
	}
	else //buf1����,��������buf0
	{	
		printf("\r\n---------buf1-----------\r\n");
		for(i=0;i<size;i++){
			pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1���������
			printf("%x",pbuf[i]);
		}
		jpeg_data_len+=jpeg_line_size;//ƫ�� 
	} 
		//DMADMCI_Handler.Instance->CR = DMADMCI_Handler.Instance->CR ^(1<<19);
    SCB_CleanInvalidateDCache();        //�����Ч��DCache
		*/
		//��������
//		for(i=0;i<size;i++){
//			pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
//			printf("%x",pbuf[i]);
//		}
//    __HAL_DMA_SET_COUNTER(&DMADMCI_Handler,jpeg_line_size);	//���䳤��Ϊjpeg_buf_size*4�ֽ�
//		__HAL_DMA_ENABLE(&DMADMCI_Handler); //��DMA
//		jpeg_data_len=1;				
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
}

void RGB_dcmi_rx_callback(void){
	u16 i; 
	u32 *pbuf;
	u32 size = jpeg_line_size;
	pbuf=jpeg_data_buf;//ƫ�Ƶ���Ч����ĩβ
	/*
	//  printf("jpeg_dcmi_rx_callback,data_len=%d\r\n",jpeg_data_len);
	if(DMADMCI_Handler.Instance->CR&(1<<19))//buf0����,��������buf1
	{ 
		printf("\r\n---------buf0-----------\r\n");
		for(i=0;i<size;i++){
			pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
			printf("%x",pbuf[i]);
		}	
		jpeg_data_len+=jpeg_line_size;//ƫ��
	}
	else //buf1����,��������buf0
	{	
		printf("\r\n---------buf1-----------\r\n");
		for(i=0;i<size;i++){
			pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1���������
			printf("%x",pbuf[i]);
		}
		jpeg_data_len+=jpeg_line_size;//ƫ�� 
	} 
		//DMADMCI_Handler.Instance->CR = DMADMCI_Handler.Instance->CR ^(1<<19);
    SCB_CleanInvalidateDCache();        //�����Ч��DCache
		*/
		//��������
		for(i=0;i<size;i++){
			pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
			printf("%x",pbuf[i]);
		}
		printf("\r\n");
    __HAL_DMA_SET_COUNTER(&DMADMCI_Handler,jpeg_line_size);	//���䳤��Ϊjpeg_buf_size*4�ֽ�
		__HAL_DMA_ENABLE(&DMADMCI_Handler); //��DMA
		jpeg_data_len=1;		
}	

//OV5640����jpgͼƬ
//����ֵ:0,�ɹ�
//    ����,�������
u8 ov2640_jpg_photo()
{
	u32 i ;
	u32* pbuf;
	//todo: ����DMA�Լ��ļ������С�����
	//f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//����FIL�ֽڵ��ڴ����� 
	//if(f_jpg==NULL)return 0XFF;				//�ڴ�����ʧ��.
	ovx_mode=1;
	jpeg_data_ok=0;
	//sw_ov2640_mode();						//�л�ΪOV2640ģʽ 
	OV2640_JPEG_Mode();						//JPEGģʽ  
	OV2640_ImageWin_Set(0,0,1600,1200);
	OV2640_OutSize_Set(1600,1200);          //���ճߴ�Ϊ1600*1200
	dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG�������ݻص�����
	//DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,2,1);//DCMI DMA����  
	DCMI_DMA_Init((u32)dcmi_line_buf[0],0,jpeg_line_size,2,1);
 // printf("Jpeg Data as follows :");
	while(1){
	jpeg_data_ok=0;
	DCMI_DMA_Init((u32)dcmi_line_buf[0],0,jpeg_line_size,2,1);
	DCMI_Start(); 			//�������� 
		/*while(jpeg_data_ok!=1);	//�ȴ���һ֡ͼƬ�ɼ���
		jpeg_data_ok=2;			//���Ա�֡ͼƬ,������һ֡�ɼ� 
		while(jpeg_data_ok!=1);	//�ȴ��ڶ�֡ͼƬ�ɼ���,�ڶ�֡,�ű��浽SD��ȥ.
		*/
	while(jpeg_data_ok!=1);
					
		//����jpeg_data_buf�е�����
//		printf("the size of JPEG is %d\r\n",jpeg_data_len);
//		pbuf=jpeg_data_buf;
//		for(i=0;i<jpeg_data_len;i++)
//			printf("%x",pbuf[i]);
//		printf("\r\n-----------------------------------------------\r\n");
//		jpeg_data_ok =2 ;
		DCMI_Stop(); 			//ֹͣDMA����
//		break;
	}
	return 0;
}  

u8 ov2640_RGB_photo(){
	dcmi_rx_callback = RGB_dcmi_rx_callback;
	DCMI_Start();
	while(jpeg_data_ok!=1);
	DCMI_Stop();
	return 0;
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
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
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
	u8 type = 1 ;  //0->RGB,1->JPEG
	dcmi_line_buf[0] = JpegBuffer0;
	jpeg_data_buf = JpegBuf;
  Write_Through();                //����ǿ��͸д��
  Cache_Enable();                 //��L1-Cache,��cubemx������ͬ
  HAL_Init();				        //��ʼ��HAL��
  SystemClock_Config();   //����ʱ��,168Mhz 
 // delay_init(168);                //��ʱ��ʼ��
	//uart1_init(115200);		        //����1��ʼ�����߼��ж���ʱ����Ҫ����
	BLUELEDinit();
	MX_TIM3_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	//BLUELEDToggle();
//	REDLEDinit();
//
//  while(OV2640_Init()!=0);				    //��ʼ��OV2640
//	//printf("ov2640 init over\r\n");
//	BLUELEDToggle();
//	HAL_Delay(500);
//	BLUELEDToggle();
//	//printf("ov2640ret = %d\r\n",ov2640ret); //�����ʼ��OV2640��ķ���ֵ
//    //Show_Str(30,210,230,16,"OV2640 ����",16,0); 
//	//�Զ��Խ���ʼ��
//	OV2640_RGB565_Mode();	//RGB565ģʽ,�л�ΪJPEG��ʽ
//	OV2640_Light_Mode(0);	//�Զ�ģʽ
//	OV2640_Color_Saturation(3);//ɫ�ʱ��Ͷ�0
//	OV2640_Brightness(4);	//����0
//	OV2640_Contrast(3);		//�Աȶ�0
//	
//	DCMI_Init();			//DCMI����
//	if(type == 1)
//		ov2640_jpg_photo();  //�ڲ���DMA��ʼ��
//	else 
//		ov2640_RGB_photo();
//	//printf("\r\nOver\r\n");
	while(1){
	}
}
void HAL_MspInit(void)
{
  //__HAL_RCC_PWR_CLK_ENABLE();
  //__HAL_RCC_SYSCFG_CLK_ENABLE();
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	     while(1){
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
//	}
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
}

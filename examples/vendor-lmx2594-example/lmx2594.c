#include "lmx2594.h"
#include "delay.h"
#include "datatransfer.h"

/*************************************
 * 函数名：Beeper_GPIO_Config()
 * 描述  ：写控制字用到的I/O口
 * 输入  ：无
 * 输出  ：无
 ************************************/
uint32_t R[113];            //定义LMX2594寄存器数组
 
void Lmx2594_GPIO_Config(void)
{		
		GPIO_InitTypeDef GPIO_InitStructure;																								/*定义一个GPIO_InitTypeDef类型的结构体*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 															/*开启GPIOA的外设时钟*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6;								/*选择要控制的GPIOA引脚*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;																		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;																		/*设置引脚速率为50MHz */ 
		GPIO_Init(GPIOA, &GPIO_InitStructure);																							/*调用库函数，初始化GPIOA*/	
		LMX2594_CE=1;  
}

/*************************************
 * 函数名：SendData(uint32_t a)
 * 描  述：写控制字用到的I/O口
 * 输  入：uint32_t a
 * 输  出：无
 * 引脚使用：PA4接CSB，PA5接SCK，PA6接SDI
 ************************************/


void SendData(uint32_t a)															/*定义发送数据函数*/
{

		uint32_t  k;			//定义中间变量
    uint8_t i;				//定义循环次数变量
    k = a;						//将控制字的值赋给k
		LMX2594_LE=0;								//LE置低电平
		LMX2594_CLK=1;									//CLK置高电平

          for(i=0;i<24;i++)															//LMX2594控制字为24位3字节，设置循环次数
          {  
								LMX2594_CLK=0;			//CLK置低
								if ( k&0x800000 )												//如果最高位是1，则将DATA引脚置高
								{
										LMX2594_DATA=1;		  //DATA置高电平
								}
								else																		//如果最高位是0，则将DATA引脚置低
								{
										LMX2594_DATA=0;				//DATA置低电平
								}
//								delay_us(1);														//延时1us
								LMX2594_CLK=1;				//CLK管脚拉高
//								delay_us(1);     												//延时1us
								k=k<<1;																	//将控制字左移1位，下一个循环写入
	        }
	  LMX2594_LE=1;										//写入完毕，LE拉高，将控制字装入相应寄存器
//	  delay_us(1);																				//延时1us
}



void SendDataArray( uint32_t *a, uint32_t num )
	
{
			uint32_t *p;
			for(p=a+num-1;p>=a;p--)
			{
				SendData(*p);
			}
}

void set_fre(u32 freq,u8 power)
{
	double Chanspace = 1;
	Registerdata((double)freq,Chanspace,power);
	SendData(0x00241E);					//复位芯片
	SendData(0x00241C);					//去掉芯片复位
	SendDataArray(R, 113);			//依次写入113个寄存器参数
	SendData(R[0]);							//再写入1次R[0]，确保芯片工作起来
}

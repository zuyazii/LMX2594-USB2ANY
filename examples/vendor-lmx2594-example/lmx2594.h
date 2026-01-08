#ifndef __LMX2594_H
#define	__LMX2594_H
#include "stm32f10x.h"


#define LMX2594_CLK 	PAout(6)
#define LMX2594_LE 		PAout(2)
#define LMX2594_DATA 	PAout(3)
#define LMX2594_CE 		PAout(1)
#define LMX2594_MUX 	PAin(7)

				
void SendData(uint32_t a);					//定义写控制字程序
void Lmx2594_GPIO_Config(void);			//写控制字引脚初始化函数
void Frequencyfixed(uint32_t f1, uint32_t t1);
void Frequencysweep(uint32_t f1, uint32_t f2, uint32_t t1, uint32_t d1);
void Frequencyhopping(uint32_t f1, uint32_t f2,uint32_t t1, uint32_t d1);
void SendDataArray(uint32_t *a, uint32_t num);
//void mode1(uint32_t starthz, uint32_t stophz, uint32_t stephz);


void set_fre(u32 freq,u8 power);
#endif /* __LMX2594_H */


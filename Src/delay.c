#include "delay.h"
#include "sys.h"


static u8  fac_us=0;//us延时倍乘数			   
//static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数



			   
//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init(u8 SYSCLK)
{

// 	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8 );
//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	
	fac_us=SYSCLK;		//不论是否使用ucos,fac_us都需要使用
	    

//	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   

}								    


//延时nus
//nus为要延时的us数.	
//nus:0~23860929(最大值即2^32/fac_us@fac_us=180)	
void delay_us(u32 nus)
{		
	u32 ticks,t_start,t_now,tcnt = 0;	    	 
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	t_start=SysTick->VAL;        			//刚进入时的计数器值
	while(1)
	{
		t_now=SysTick->VAL;	
		if(t_now!=t_start)
		{	    
			if(t_now<t_start)
				tcnt += t_start - t_now;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else 
				tcnt += reload - t_now + t_start;	    
			t_start = t_now;	
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	}
}
#if 0
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对168M条件下,nms<=798ms 
void delay_xms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 

//延时nms 
//nms:0~65535
void delay_ms(u16 nms)
{	 	 
	u8 repeat=nms/540;	//这里用540,是考虑到某些客户可能超频使用,
						//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
	u16 remain=nms%540;
	while(repeat)
	{
		delay_xms(540);
		repeat--;
	}
	if(remain)delay_xms(remain);
	
} 
#else

//延时nms
//nms:要延时的ms数
void delay_ms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

#endif

































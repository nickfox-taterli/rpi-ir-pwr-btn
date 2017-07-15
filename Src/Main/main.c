#include "stm8s.h"

/* 用S103做开发要兼容S003的话,只能用TIM1/TIM2,然后只有PA可以发生中断.S103没此限制.不能再003调试,因为003的Flash没多少次擦写寿命.接近OTP了. */

/* PA1 红外,不能修改! */

/* 因为核心板BUG,所以5V接核心板3V3,MCU就工作在5V了. */

/* 可修改内容开始,下面引脚请不要落在PA. */

#define SIGOUT_GPIO_PORT  (GPIOB)
#define SIGOUT_GPIO_PINS  (GPIO_PIN_5)

#define SIGIN_GPIO_PORT  (GPIOB)
#define SIGIN_GPIO_PINS  (GPIO_PIN_4)

#define KEY_GPIO_PORT  (GPIOC)
#define KEY_GPIO_PINS  (GPIO_PIN_3)

#define PWROUT_GPIO_PORT  (GPIOC)
#define PWROUT_GPIO_PINS  (GPIO_PIN_4)

#define PWR_IR_CODE 0xA2 /* 关机按键数值 */

/* 调整下面数值,越大越容错.越小越精确. */

/* 红外引导码阈值,中心数值12500 */
#define IR_PRESIG_LEN_L 10500
#define IR_PRESIG_LEN_H 18500

/* 红外0码阈值,中心数值1250 */
#define IR_ZERO_LEN_L 800
#define IR_ZERO_LEN_H 1500

/* 红外1码阈值,中心数值2250 */
#define IR_ONE_LEN_L 1800
#define IR_ONE_LEN_H 3000

/* 可修改内容结束 */

uint8_t Ir_Status = 0;               //红外接收处理状态
uint8_t  Ir_Receive_Count = 0;       //红外接收数据位计数
uint32_t Ir_Receive_Data = 0;        //32位的红外接收数据
uint8_t Ir_receive_ok = 0;           //红外接收完成标志

__IO uint32_t TimingDelay = 0;

__IO uint8_t Ir_Code_Recive = 0;

void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while (TimingDelay != 0);
}


uint8_t Ir_Process(void)
{
    uint8_t Ir_num = 0;                //最终处理后的键值返回值
    uint8_t Address_H, Address_L;      //地址码,地址反码
    uint8_t Data_H, Data_L;            //数据码,数据反码

    if(Ir_receive_ok == 1)        //接收完成
    {
        Address_H = Ir_Receive_Data >> 24;            //得到地址码
        Address_L = (Ir_Receive_Data >> 16) & 0xff;          //得到地址反码
        //if((Address_H==(u8)~Address_L)&&(Address_H==REMOTE_ID))//检验遥控识别码(ID)及地址
        if((Address_H == (u8)~Address_L)) //检验遥控识别码(ID)及地址
        {
            Data_H = Ir_Receive_Data >> 8;          //得到数据码
            Data_L = Ir_Receive_Data;               //得到数据反码
            if(Data_H == (u8)~Data_L)               //接收数据码正确
            {
                Ir_num = Data_H;                    //正确键值
                Ir_receive_ok = 0;
            }
        }

    }
    return  Ir_num;      //返回键值
}

void Ir_Receive_Handle(void)
{
    uint16_t Interval_tim = 0; //两个下降沿间隔时间

    switch(Ir_Status)
    {
    case 0://第一个下降沿，定时器开始计数
        Ir_Status = 1;
        TIM2_Cmd(ENABLE);                   //Enable TIM2
        TIM2_SetCounter(0);                 //定时器计数值清零
        break;
    case 1://第二个下降沿，定时器关闭，读取定时器计数值
        TIM2_Cmd(DISABLE);
        Interval_tim = 0;
        Interval_tim = TIM2_GetCounter();   //读取定时器计数值
        TIM2_SetCounter(0);                 //定时器计数值清零
        TIM2_Cmd(ENABLE);                   //Enable TIM2
        if( (Interval_tim >= IR_PRESIG_LEN_L) && (Interval_tim <= IR_PRESIG_LEN_H) ) //判断引导码是否正确9+4.5ms
        {
            Ir_Status = 2;                  //进入下一状态

        }
        else                                //引导码错误，从新接收
        {
            Ir_Status = 0;
            Ir_Receive_Count = 0;
        }
        break;
    case 2://进入32位数据接收
        TIM2_Cmd(DISABLE);
        Interval_tim = 0;
        Interval_tim = TIM2_GetCounter();
        TIM2_SetCounter(0);
        TIM2_Cmd(ENABLE); //Enable TIM2
        if(Ir_Receive_Count < 32)
        {
          /* 时间不合适可以适当调整. */
            if( (Interval_tim >= IR_ZERO_LEN_L) && (Interval_tim <= IR_ZERO_LEN_H) )  //间隔1.12ms ->0
            {
                Ir_Receive_Data = Ir_Receive_Data << 1;
                Ir_Receive_Count++;
            }
            else if( (Interval_tim >= IR_ONE_LEN_L) && (Interval_tim <= IR_ONE_LEN_H) ) //间隔2.25ms ->1
            {
                Ir_Receive_Data = Ir_Receive_Data << 1;
                Ir_Receive_Data = Ir_Receive_Data | 0x0001;
                Ir_Receive_Count++;
            }
            else//不是0,1 接收错误，从新接收
            {
                Ir_Status = 0;
                Ir_Receive_Data = 0;
                Ir_Receive_Count = 0;
            }
            break;
        }
        else//超出接收数据位数，接收下一个
        {
            Ir_receive_ok = 1; //红外接收完成
            Ir_Status = 0;
            Ir_Receive_Count = 0;
            break;
        }


        break;
    default :
        break;
    }
}

void Pi_PowerDown(void)
{
      if(!GPIO_ReadInputPin(SIGIN_GPIO_PORT, (GPIO_Pin_TypeDef)SIGIN_GPIO_PINS))
    {
        return; /* 驱动不对或者PI没准备好,不继续执行. */
    }

    /* 关机脉冲 */
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(1);
    GPIO_WriteHigh(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(80);
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(20 * 1000); /* 等关机时间,调试时候关闭. */
    GPIO_WriteLow(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);


}

void Pi_PowerUp(void)
{
    GPIO_WriteHigh(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);

}
/* 等下吧顺序调换下,应该一开始就检查驱动. */

void main(void)
{
    /* 禁止HSI分频 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    /* 定时器分频,这样设置是1ms一次中断. */
    TIM1_TimeBaseInit(128,TIM1_COUNTERMODE_UP, 124,0);
    TIM1_ARRPreloadConfig(ENABLE);

    /* 溢出中断 */
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
    /* 开中断 */
    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    /* 开启TIM4 */
    TIM1_Cmd(ENABLE);
    /* PD4 - IDRA_RX */
    GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_PU_IT);
    /* PD口(所有IO)下降沿中断 */
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
    /* 重置TIM2 */
    TIM2_DeInit();
    /* 1US中断 */
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, 60000);
    /* 清中断标志 */
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);
    /* 配置中断 */
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
    /* 暂时不使能 */
    TIM2_Cmd(DISABLE);

    /* 总中断 */
    enableInterrupts();

    /*  通知PI关机的按钮初始化 */
    GPIO_Init(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);

    /*  检查驱动安装 */
    GPIO_Init(SIGIN_GPIO_PORT, (GPIO_Pin_TypeDef)SIGIN_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);

    /*  开关机按键 */
    GPIO_Init(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);

    /* 电源使能 */
    GPIO_Init(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);    /* H = 插上模块立马开机 L = 插上模块不开机 */

    while (1)
    {
        /* 接收红外数据 */
        Ir_Code_Recive = Ir_Process();
        if(Ir_Code_Recive == PWR_IR_CODE)  /* 假设A2是关机按钮 */
        {
            /* Pi 已经工作中 */
            if(((BitStatus)(PWROUT_GPIO_PORT->ODR & (uint8_t)PWROUT_GPIO_PINS)))
            {

                Pi_PowerDown();
            }
            else   /* Pi 未工作 */
            {
                Pi_PowerUp();
            }
            Ir_Code_Recive = 0x00;
        }

        if(GPIO_ReadInputPin(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS))
        {
            Delay(20);
            if(GPIO_ReadInputPin(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS))
            {
              
              while(GPIO_ReadInputPin(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS)); /* 等按键释放 */
                /* 此时真的按下了. */

                /* Pi 已经工作中 */
                if(((BitStatus)(PWROUT_GPIO_PORT->ODR & (uint8_t)PWROUT_GPIO_PINS)))
                {

                    Pi_PowerDown();
                }
                else   /* Pi 未工作 */
                {
                    Pi_PowerUp();
                }

            }

        }
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while (1)
    {
    }
}
#endif

#include "stm8s.h"

/* ��S103������Ҫ����S003�Ļ�,ֻ����TIM1/TIM2,Ȼ��ֻ��PA���Է����ж�.S103û������.������003����,��Ϊ003��Flashû���ٴβ�д����.�ӽ�OTP��. */

/* PA1 ����,�����޸�! */

/* ��Ϊ���İ�BUG,����5V�Ӻ��İ�3V3,MCU�͹�����5V��. */

/* ���޸����ݿ�ʼ,���������벻Ҫ����PA. */

#define SIGOUT_GPIO_PORT  (GPIOB)
#define SIGOUT_GPIO_PINS  (GPIO_PIN_5)

#define SIGIN_GPIO_PORT  (GPIOB)
#define SIGIN_GPIO_PINS  (GPIO_PIN_4)

#define KEY_GPIO_PORT  (GPIOC)
#define KEY_GPIO_PINS  (GPIO_PIN_3)

#define PWROUT_GPIO_PORT  (GPIOC)
#define PWROUT_GPIO_PINS  (GPIO_PIN_4)

#define PWR_IR_CODE 0xA2 /* �ػ�������ֵ */

/* ����������ֵ,Խ��Խ�ݴ�.ԽСԽ��ȷ. */

/* ������������ֵ,������ֵ12500 */
#define IR_PRESIG_LEN_L 10500
#define IR_PRESIG_LEN_H 18500

/* ����0����ֵ,������ֵ1250 */
#define IR_ZERO_LEN_L 800
#define IR_ZERO_LEN_H 1500

/* ����1����ֵ,������ֵ2250 */
#define IR_ONE_LEN_L 1800
#define IR_ONE_LEN_H 3000

/* ���޸����ݽ��� */

uint8_t Ir_Status = 0;               //������մ���״̬
uint8_t  Ir_Receive_Count = 0;       //�����������λ����
uint32_t Ir_Receive_Data = 0;        //32λ�ĺ����������
uint8_t Ir_receive_ok = 0;           //���������ɱ�־

__IO uint32_t TimingDelay = 0;

__IO uint8_t Ir_Code_Recive = 0;

void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while (TimingDelay != 0);
}


uint8_t Ir_Process(void)
{
    uint8_t Ir_num = 0;                //���մ����ļ�ֵ����ֵ
    uint8_t Address_H, Address_L;      //��ַ��,��ַ����
    uint8_t Data_H, Data_L;            //������,���ݷ���

    if(Ir_receive_ok == 1)        //�������
    {
        Address_H = Ir_Receive_Data >> 24;            //�õ���ַ��
        Address_L = (Ir_Receive_Data >> 16) & 0xff;          //�õ���ַ����
        //if((Address_H==(u8)~Address_L)&&(Address_H==REMOTE_ID))//����ң��ʶ����(ID)����ַ
        if((Address_H == (u8)~Address_L)) //����ң��ʶ����(ID)����ַ
        {
            Data_H = Ir_Receive_Data >> 8;          //�õ�������
            Data_L = Ir_Receive_Data;               //�õ����ݷ���
            if(Data_H == (u8)~Data_L)               //������������ȷ
            {
                Ir_num = Data_H;                    //��ȷ��ֵ
                Ir_receive_ok = 0;
            }
        }

    }
    return  Ir_num;      //���ؼ�ֵ
}

void Ir_Receive_Handle(void)
{
    uint16_t Interval_tim = 0; //�����½��ؼ��ʱ��

    switch(Ir_Status)
    {
    case 0://��һ���½��أ���ʱ����ʼ����
        Ir_Status = 1;
        TIM2_Cmd(ENABLE);                   //Enable TIM2
        TIM2_SetCounter(0);                 //��ʱ������ֵ����
        break;
    case 1://�ڶ����½��أ���ʱ���رգ���ȡ��ʱ������ֵ
        TIM2_Cmd(DISABLE);
        Interval_tim = 0;
        Interval_tim = TIM2_GetCounter();   //��ȡ��ʱ������ֵ
        TIM2_SetCounter(0);                 //��ʱ������ֵ����
        TIM2_Cmd(ENABLE);                   //Enable TIM2
        if( (Interval_tim >= IR_PRESIG_LEN_L) && (Interval_tim <= IR_PRESIG_LEN_H) ) //�ж��������Ƿ���ȷ9+4.5ms
        {
            Ir_Status = 2;                  //������һ״̬

        }
        else                                //��������󣬴��½���
        {
            Ir_Status = 0;
            Ir_Receive_Count = 0;
        }
        break;
    case 2://����32λ���ݽ���
        TIM2_Cmd(DISABLE);
        Interval_tim = 0;
        Interval_tim = TIM2_GetCounter();
        TIM2_SetCounter(0);
        TIM2_Cmd(ENABLE); //Enable TIM2
        if(Ir_Receive_Count < 32)
        {
          /* ʱ�䲻���ʿ����ʵ�����. */
            if( (Interval_tim >= IR_ZERO_LEN_L) && (Interval_tim <= IR_ZERO_LEN_H) )  //���1.12ms ->0
            {
                Ir_Receive_Data = Ir_Receive_Data << 1;
                Ir_Receive_Count++;
            }
            else if( (Interval_tim >= IR_ONE_LEN_L) && (Interval_tim <= IR_ONE_LEN_H) ) //���2.25ms ->1
            {
                Ir_Receive_Data = Ir_Receive_Data << 1;
                Ir_Receive_Data = Ir_Receive_Data | 0x0001;
                Ir_Receive_Count++;
            }
            else//����0,1 ���մ��󣬴��½���
            {
                Ir_Status = 0;
                Ir_Receive_Data = 0;
                Ir_Receive_Count = 0;
            }
            break;
        }
        else//������������λ����������һ��
        {
            Ir_receive_ok = 1; //����������
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
        return; /* �������Ի���PIû׼����,������ִ��. */
    }

    /* �ػ����� */
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(1);
    GPIO_WriteHigh(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(80);
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);
    Delay(20 * 1000); /* �ȹػ�ʱ��,����ʱ��ر�. */
    GPIO_WriteLow(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);


}

void Pi_PowerUp(void)
{
    GPIO_WriteHigh(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);

}
/* ���°�˳�������,Ӧ��һ��ʼ�ͼ������. */

void main(void)
{
    /* ��ֹHSI��Ƶ */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    /* ��ʱ����Ƶ,����������1msһ���ж�. */
    TIM1_TimeBaseInit(128,TIM1_COUNTERMODE_UP, 124,0);
    TIM1_ARRPreloadConfig(ENABLE);

    /* ����ж� */
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
    /* ���ж� */
    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    /* ����TIM4 */
    TIM1_Cmd(ENABLE);
    /* PD4 - IDRA_RX */
    GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_PU_IT);
    /* PD��(����IO)�½����ж� */
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
    /* ����TIM2 */
    TIM2_DeInit();
    /* 1US�ж� */
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, 60000);
    /* ���жϱ�־ */
    TIM2_ClearFlag(TIM2_FLAG_UPDATE);
    /* �����ж� */
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
    /* ��ʱ��ʹ�� */
    TIM2_Cmd(DISABLE);

    /* ���ж� */
    enableInterrupts();

    /*  ֪ͨPI�ػ��İ�ť��ʼ�� */
    GPIO_Init(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(SIGOUT_GPIO_PORT, (GPIO_Pin_TypeDef)SIGOUT_GPIO_PINS);

    /*  ���������װ */
    GPIO_Init(SIGIN_GPIO_PORT, (GPIO_Pin_TypeDef)SIGIN_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);

    /*  ���ػ����� */
    GPIO_Init(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS, GPIO_MODE_IN_FL_NO_IT);

    /* ��Դʹ�� */
    GPIO_Init(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(PWROUT_GPIO_PORT, (GPIO_Pin_TypeDef)PWROUT_GPIO_PINS);    /* H = ����ģ�������� L = ����ģ�鲻���� */

    while (1)
    {
        /* ���պ������� */
        Ir_Code_Recive = Ir_Process();
        if(Ir_Code_Recive == PWR_IR_CODE)  /* ����A2�ǹػ���ť */
        {
            /* Pi �Ѿ������� */
            if(((BitStatus)(PWROUT_GPIO_PORT->ODR & (uint8_t)PWROUT_GPIO_PINS)))
            {

                Pi_PowerDown();
            }
            else   /* Pi δ���� */
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
              
              while(GPIO_ReadInputPin(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS)); /* �Ȱ����ͷ� */
                /* ��ʱ��İ�����. */

                /* Pi �Ѿ������� */
                if(((BitStatus)(PWROUT_GPIO_PORT->ODR & (uint8_t)PWROUT_GPIO_PINS)))
                {

                    Pi_PowerDown();
                }
                else   /* Pi δ���� */
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

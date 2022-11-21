//
// Created by YanYuanbin on 22-10-12.
//
#include "cmsis_os.h"

#include "ChassisTask.h"
#include "InsTask.h"

#include "tim.h"

#include "motor.h"
#include "myusart.h"

#include "pid.h"
#include "kalman.h"


//0��1��
#define RED_OR_BLUE   0


#if RED_OR_BLUE
//Red
ActPoint_t GoalPoint[18]=
{
	[0]={.y = 0,.x = 0},//0
	[1]={.y = 10,.x = 0},//0
	[2]={.x = -1600,.y=-20}, //0
	[3]={.x = -1600,.y=-750}, //90
	[4]={.x = -4000,.y=-750}, //0
	[5]={.x = -4200,.y=-1540},//90
	[6]={.x = -4200,.y=-1165},//90
	[7]={.x = -4200,.y=-790},//90
	[8]={.x = -4210,.y=-415},//90
	[9]={.x = -4210,.y=-40},//90
	[10] = {.x = -4000,.y = -750},//90
	[11] = {.x = -550,.y = -750},//0
	[12] = {.x = -440,.y = -750},//90
	[13] = {.x = -440,.y = -1490},//90
	[14] = {.x = -435,.y = -1240},//90
	[15] = {.x = -435,.y = -990},//90
	[16] = {.x = -430,.y = -740},//90
	[17] = {.x = -430,.y = -490},//90
};

#else
//Blue
ActPoint_t GoalPoint[18]=
{
	[0]={.y = 0,.x = 0},//0
	[1]={.y = 10,.x = 0},//0
	[2]={.x = -1600,.y=20}, //0
	[3]={.x = -1600,.y=750}, //90
	[4]={.x = -4000,.y=750}, //0
	[5]={.x = -4200,.y=1380},//90
	[6]={.x = -4200,.y=1005},//90
	[7]={.x = -4200,.y=630},//90
	[8]={.x = -4200,.y=255},//90
	[9]={.x = -4190,.y=-120},//90
	[10] = {.x = -4000,.y = 750},//90
	[11] = {.x = -550,.y = 750},//0
	[12] = {.x = -440,.y = 750},//90
	[13] = {.x = -430,.y = 1530},//90
	[14] = {.x = -435,.y = 1280},//90
	[15] = {.x = -440,.y = 1030},//90
	[16] = {.x = -440,.y = 780},//90
	[17] = {.x = -435,.y = 530},//90
};

#endif

//ƽ����ֵ����ǰ�㵽Ŀ������
float midangle=-4.8f,distance =0;

//Ŀ��㼰���������
Actline_t Goalline;
//�л�Ŀ������
uint32_t Switch_Cnt=0;
//X/Y���������ٶ�
float target_x = 0,target_y =0;

//ת���̽����鼰�±�
float Table[10];
uint8_t tableCnt =0;

//�Ƿ����
bool IF_Arm_Down = false;
//����/ȡ���
float Arm_down=0.f,Arm_up=0.f;

//�����ٶȼ�ƫ�������
float speed,turnErr;
//ƽ��������
float leftExp,rightExp;
//���PID�������ֵ
float armout,tableout,leftout,rightout,sideout;

//һ�׵�ͨ�ṹ��
Lowpass_Filter_t Left_Speed,Right_Speed;

//PID�ṹ��
PID_TypeDef_t LeftWheel_pid,RightWheel_pid,PitAngle_pid,PitGyro_pid,Speed_pid,Sideway_pid,Turn_pid     \
							,ArmAngle_pid,ArmSpeed_pid,TableAngle_pid,TableSpeed_pid,PostureY_pid,PostureX_pid;
							
/* Private fuction prototypes -----------------------------------------------------*/
static float f_FirstOrder_Lowpass_Filter(Lowpass_Filter_t *std,float value,float k);
static bool GetBall_Ctrl(void);
static bool OutBall_Ctrl(void);
static void Point_Action(float distance);

/* USER CODE BEGIN Header_ChassisTask */
/**
* @brief Function implementing the ChasTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask */
void ChassisTask(void const * argument)
{
	osDelay(8000);
  /* USER CODE BEGIN ChassisTask */
	TickType_t systick = 0;
	//PID��ʼ��
	PID_Init(&PostureY_pid,  0,0.f,1800.f,30.f,0.f,0);
	PID_Init(&PostureX_pid, 0,0.f,10000.f,50.f,0.f,0);

	PID_Init(&Speed_pid,    0,0.f,3.f,0.001f,0.f,0);
	PID_Init(&PitAngle_pid, 0,0.f,5000.f, 16.f,0.f,0);
	PID_Init(&PitGyro_pid,  0,600.f,10000.f,14.f,0.6f,50.f);
	PID_Init(&Turn_pid,     0,0.f,2500.f,-100.f,0.f,0.f);
	
	PID_Init(&RightWheel_pid,0,2000.f,16300.f,13.2f,0.2f,0);
	PID_Init(&LeftWheel_pid, 0,2000.f,16300.f,13.2f,0.2f,0);
	PID_Init(&Sideway_pid,   0,1000.f,10000.f,8.4f,0.1f,0);

	PID_Init(&ArmAngle_pid,  0,0.f,6000.f,100.f,0.f,0);
	PID_Init(&ArmSpeed_pid,  0,2000.f,16000.f,16.8f,0.1f,0);

	PID_Init(&TableAngle_pid,  0,0.f,3000.f,50.f,0.f,0);
	PID_Init(&TableSpeed_pid,  0,1000.f,8000.f,6.2f,0.01f,0);
	
	//��ȡת���̽ǣ��ϵ��ǵø�λ����������
	Table[0] = Balance[Turntable].Data.angle;
	for(int i=0;i<4;i++)
	{
		Table[i+1] = Table[i]+72.f;
	}
	for(int i=5;i<10;i++)
	{
		Table[i] = Table[9-i]-36.f;
	}
	//��ȡ��/ȡ��۽ǣ�ע��ƽ���ϵ�
	Arm_down = Balance[ArmPole].Data.angle - Balance[ArmPole].Data.angle/ABS(Balance[ArmPole].Data.angle)*0.8f;
	Arm_up   = Balance[ArmPole].Data.angle+102.5f;
	//��/��������λ
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1600);

  /* Infinite loop */	
  for(;;)
  {
		systick = osKernelSysTick();
		//��翪��ʹ���ܳ�
		if(Game_Start >= 4)
		{
			distance = ABS(sqrtf(powf((Presentline.point.x-Goalline.point.x),2.f)
							 + powf((Presentline.point.y-Goalline.point.y),2.f)));
			Point_Action(distance);
		}
		//���������ٶ�
		speed = ( -f_FirstOrder_Lowpass_Filter(&Left_Speed,Balance[Left_Wheel].Data.velocity,0.9f)
					    +f_FirstOrder_Lowpass_Filter(&Right_Speed,Balance[Right_Wheel].Data.velocity,0.9f))/2.f;
		//�����ƶ���ʽ�л��ٶȽ���
		if(Goalline.angle == 90.f)
		{
			target_y = f_PID_Calculate(&PostureY_pid,Goalline.point.x - Presentline.point.x);
			target_x = f_PID_Calculate(&PostureX_pid,Presentline.point.y - Goalline.point.y);
		}else if(Goalline.angle == 0.f)
		{
			target_y = f_PID_Calculate(&PostureY_pid,Presentline.point.y - Goalline.point.y);	
			target_x = f_PID_Calculate(&PostureX_pid,Presentline.point.x - Goalline.point.x);		
		}
		//����ת��ǣ�װ�ص���PID����
		if(Goalline.angle-Presentline.angle > 0.1f)
		{
			leftExp = f_PID_Calculate(&PitGyro_pid,
								f_PID_Calculate(&PitAngle_pid,midangle+
								f_PID_Calculate(&Speed_pid,target_y-speed)-Imu.pit_angle) - Imu.pit_gyro)
							+ f_PID_Calculate(&Turn_pid,sqrtf(Goalline.angle-Presentline.angle));
		}
		else if(Goalline.angle-Presentline.angle < -0.1f)
		{
			rightExp=-f_PID_Calculate(&PitGyro_pid,
								f_PID_Calculate(&PitAngle_pid,midangle+
								f_PID_Calculate(&Speed_pid,target_y-speed)-Imu.pit_angle) - Imu.pit_gyro)
							+ f_PID_Calculate(&Turn_pid,-sqrtf(Presentline.angle-Goalline.angle));
		}
		//���ر�������ֹ��������
		if(ABS(Imu.pit_angle) > 30.f )
		{
			leftout  = f_PID_Calculate(&LeftWheel_pid ,0 - Balance[Left_Wheel].Data.velocity);
			rightout = f_PID_Calculate(&RightWheel_pid,0 - Balance[Right_Wheel].Data.velocity);
			sideout  = f_PID_Calculate(&Sideway_pid,   0 - Balance[SideWay].Data.velocity);
		}else
		{
			leftout  = f_PID_Calculate(&LeftWheel_pid ,leftExp  - Balance[Left_Wheel].Data.velocity);
			rightout = f_PID_Calculate(&RightWheel_pid,rightExp - Balance[Right_Wheel].Data.velocity);
			sideout  = f_PID_Calculate(&Sideway_pid,   target_x - Balance[SideWay].Data.velocity);
		}
		
		//װ�ؿ�/ȡ���PID����
		if(IF_Arm_Down)
		{
			armout = f_PID_Calculate(&ArmSpeed_pid,
							 f_PID_Calculate(&ArmAngle_pid,Arm_down - Balance[ArmPole].Data.angle)-Balance[ArmPole].Data.velocity);
		}else
		{
			armout = f_PID_Calculate(&ArmSpeed_pid,
							 f_PID_Calculate(&ArmAngle_pid,Arm_up - Balance[ArmPole].Data.angle)-Balance[ArmPole].Data.velocity);
		}

		//װ�ػ�����PID����
		tableout = f_PID_Calculate(&TableSpeed_pid,
							 f_PID_Calculate(&TableAngle_pid,Table[tableCnt]-Balance[Turntable].Data.angle)-Balance[Turntable].Data.velocity);
		
		//��翪��ʹ��CANͨ��
		if(Game_Start > 2)
		{
			hcan1TxFrame.data[0] = (int16_t)rightout >> 8;
			hcan1TxFrame.data[1] = (int16_t)rightout;
			hcan1TxFrame.data[2] = (int16_t)armout >> 8;
			hcan1TxFrame.data[3] = (int16_t)armout;
			hcan1TxFrame.data[4] = (int16_t)sideout >> 8;
			hcan1TxFrame.data[5] = (int16_t)sideout;	
			hcan1TxFrame.data[6] = (int16_t)leftout >> 8;
			hcan1TxFrame.data[7] = (int16_t)leftout;
			
			hcan2TxFrame.data[4] = (int16_t)tableout>> 8;
			hcan2TxFrame.data[5] = (int16_t)tableout;

			USER_CAN_TxMessage(&hcan1,&hcan1TxFrame);
			USER_CAN_TxMessage(&hcan2,&hcan2TxFrame);
		}
		
    osDelayUntil(&systick,1);
  }
  /* USER CODE END ChassisTask */
}

static float f_FirstOrder_Lowpass_Filter(Lowpass_Filter_t *std,float value,float k)
{

	std->output_last = std->output;
	std->value = value;

	std->output = k*std->value + (1.0f-k)*std->output_last;
	
	return std->output;
}

static bool GetBall_Ctrl(void)
{
	bool result = false;
	static bool Reset = false;
	static TickType_t Arm_Cnt = 0;
	
	if(!Reset)
	{
		Arm_Cnt = osKernelSysTick();
		//����
		IF_Arm_Down = true;
		Reset = true;
	}
	if(Reset && (osKernelSysTick()-Arm_Cnt) > 1200)
	{
		//̧��
		IF_Arm_Down = false;
	}
	if(Reset && (osKernelSysTick()-Arm_Cnt) > 2700)
	{
		result = true;
		Reset = false;
	}
	return result;
}

static bool OutBall_Ctrl(void)
{
	bool result = false;
	static bool Reset = false;
	static TickType_t Out_Cnt = 0;

	if(!Reset)
	{
		Out_Cnt = osKernelSysTick();
		//����
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,400);
		Reset = true;
	}
	if(Reset && (osKernelSysTick()-Out_Cnt) > 1500)
	{
		//����Ϊ������׼��
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1600);
	}
	if(Reset && (osKernelSysTick()-Out_Cnt) > 2000)
	{
		result = true;
		Reset = false;
	}
	return result;
}

static void Point_Action(float distance)
{
		//�л�����
		static uint16_t Switch_Cnt;
		//Ŀ��������±�
		static uint8_t point_Cnt =0;
		//ȡ����ʹ�ܱ�־λ
		static bool IF_GET_ENABLE = false;
		static bool IF_OUT_ENABLE = false;
	
		//Ŀ���װ��
		Goalline.point = GoalPoint[point_Cnt];
	
		//��Ϊ����Ŀ���
		if(distance <= 60)
		{
			Switch_Cnt++;
		}else
		{
			Switch_Cnt = 0;
		}
		//���ж�Ӧ����
		switch(Switch_Cnt)
		{
			case 800:
				if(point_Cnt == 2 || point_Cnt == 4 || point_Cnt == 11)
				{
						Goalline.angle = 90.f;
				}
				if(point_Cnt == 3 || point_Cnt == 10)
				{
						Goalline.angle = 0.f;
				}
			break;
				
			case 1200:
				if(point_Cnt <5 || (point_Cnt >9 && point_Cnt <13))
				{
					point_Cnt++;
				}
			break;
				
			case 2500:
				if(point_Cnt >12)
				{
						IF_OUT_ENABLE = true;
				}
				if(point_Cnt >=5 && point_Cnt <=9)
				{
						IF_GET_ENABLE = true;
				}
			break;
				
			default:
					if(point_Cnt >17)point_Cnt=17;
			break;
		}
		//ִ��ȡ����
		if(IF_GET_ENABLE)
		{
			if(GetBall_Ctrl())
			{
				tableCnt++;
				point_Cnt++;
				if(tableCnt > 5)tableCnt=5;
				IF_GET_ENABLE = false;
			}
		}
		if(IF_OUT_ENABLE)
		{
			if(OutBall_Ctrl())
			{
				tableCnt++;
				point_Cnt++;
				if(tableCnt >9)tableCnt = 9;
				IF_OUT_ENABLE = false;					
			}
		}
}

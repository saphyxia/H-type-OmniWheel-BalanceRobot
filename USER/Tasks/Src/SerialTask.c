//
// Created by YanYuanbin on 22-10-5.
//

#include "cmsis_os.h"

#include "SerialTask.h"
#include "InsTask.h"
#include "ChassisTask.h"

#include "myusart.h"

#include "tim.h"

uint16_t ActionDiDiDi =0;

/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the mySerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
void SerialTask(void const * argument)
{
  /* USER CODE BEGIN SerialTask */
	TickType_t systick = 0;
  
  /* Infinite loop */
  for(;;)
  {
		systick = osKernelSysTick();
		
		if(IF_Action_ENABLE && ActionDiDiDi<1000)
		{
			ActionDiDiDi+=20;
		}
		if(ActionDiDiDi <=900)
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,ActionDiDiDi);
		}else
		{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
		}
		
		Cnt_Singe_Flag = false;
    osDelayUntil(&systick,50);
  }
  /* USER CODE END SerialTask */
}





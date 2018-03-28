#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_control.h"
#include "flip.h"

#include <rtthread.h>
///*FreeRTOS相关头文件*/
//#include "FreeRTOS.h"
//#include "task.h"

/* s四轴自稳控制代码	*/

static bool isInit;
static bool isCalibPass;
static setpoint_t 	setpoint;	/*设置目标状态*/
static sensorData_t sensorData;	/*传感器数据*/
static state_t 		state;		/*四轴姿态*/
static control_t 	control;	/*四轴控制参数*/

void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;

	stateControlInit();		/*姿态PID初始化*/
	powerControlInit();		/*电机初始化*/

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerControlTest();

	return pass;
}


void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	
	uint32_t laskWakeTime = rt_tick_get();
	//u32 lastWakeTime = getSysTickCnt();
	
	ledseqRun(SYS_LED, seq_alive);
	
	while(!sensorsAreCalibrated())  //传感器数据校准
	{
		rt_thread_delay(100);
		//vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
	}
	isCalibPass = true;
	
	
	rt_thread_delay(100);
	//vTaskDelay(1000);	/*延时等待传感器稳定*/
	
	while(1) 
	{
		rt_thread_delay(100);
		//vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms周期延时*/

		sensorsAcquire(&sensorData, tick);					/*获取6轴和气压数据*/
		stateEstimator(&state, &sensorData, tick);			/*四元数 欧拉角计算*/
		
		commanderGetSetpoint(&setpoint, &state);			/*目标数据和飞行模式设定*/	
		flyerFlipCheck(&setpoint, &control, &state, tick);	/*翻滚检测*/
		anomalDetec(&sensorData, &state, &control);			/*异常检测*/
		stateControl(&control, &sensorData, &state, &setpoint, tick);/*PID控制*/		
		powerControl(&control);								/*控制电机输出*/

		tick++;
	}
}

void getAttitudeData(attitude_t* get)
{
	get->pitch = -state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = -state.attitude.yaw;
//	*get = state.attitude;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

float getPositionZ(void)
{
	return state.position.z;
}

bool getIsCalibPass(void)
{
	return isCalibPass;
}

void getState(state_t* get)
{
	*get = state;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}

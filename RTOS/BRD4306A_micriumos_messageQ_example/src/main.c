/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/*
*********************************************************************************************************
*
*                                             EXAMPLE MAIN
*
* File : main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*********************************************************************************************************
*/

#include  "bsp_os.h"
#include  "bsp.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include  <common/include/rtos_prio.h>

#include  "em_gpio.h"
#include  "retargetserial.h"
#include  "sleep.h"
#include  <stdio.h>

/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/
#define  APP_GRAPHICS_TASK_PRIO              23u
#define  APP_GRAPHICS_TASK_STK_SIZE         512u

#define  EX_MAIN_START_TASK_PRIO              21u
#define  EX_MAIN_START_TASK_STK_SIZE         512u

/*
*********************************************************************************************************
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/
                                                                /* Start Task Stack.                                    */
static  CPU_STK  Ex_MainStartTaskStk[EX_MAIN_START_TASK_STK_SIZE];
                                                                /* Start Task TCB.                                      */
static  OS_TCB   Ex_MainStartTaskTCB;

/* Graphics Task Stack.                                 */
static  CPU_STK                AppGraphics_TaskStk[APP_GRAPHICS_TASK_STK_SIZE];
/* Graphics Task TCB.                                   */
static  OS_TCB                 AppGraphics_TaskTCB;

static OS_Q    demoMsgQueue;

/*
*********************************************************************************************************
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*********************************************************************************************************
*/

static void Ex_MainStartTask (void  *p_arg);
static void idleHook(void);
static void setupHooks(void);

static void AppGraphics_Task(void  *p_arg);
void  AppGraphicsInit (void);


/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
int main(void)
{
    RTOS_ERR  err;
    OS_TASK_CFG tickTaskCfg = {
      .StkBasePtr = DEF_NULL,
      .StkSize    = 256u,
      .Prio       = KERNEL_TICK_TASK_PRIO_DFLT,
      .RateHz     = 1000u
    };

    CPU_Init();                                                 /* Initialize CPU.                                      */
    BSP_SystemInit();                                           /* Initialize System.                                   */

    RETARGET_SerialInit();
    RETARGET_SerialCrLf(1);

    OS_TRACE_INIT();
    OS_ConfigureTickTask(&tickTaskCfg);
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    BSP_TickInit();                                             /* Initialize Kernel tick source.                       */
    setupHooks();

    OSQCreate((OS_Q     *)&demoMsgQueue,
              (CPU_CHAR *)"Demo Queue",
              (OS_MSG_QTY) 32,
              (RTOS_ERR *)&err);

    OSTaskCreate(&Ex_MainStartTaskTCB,                          /* Create the Start Task.                               */
                 "Ex Main Start Task",
                  Ex_MainStartTask,
                  DEF_NULL,
                  EX_MAIN_START_TASK_PRIO,
                 &Ex_MainStartTaskStk[0],
                 (EX_MAIN_START_TASK_STK_SIZE / 10u),
                  EX_MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSStart(&err);                                              /* Start the kernel.                                    */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}

/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          Ex_MainStartTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainStartTask (void  *p_arg)
{
    RTOS_ERR  err;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#if (OS_CFG_STAT_TASK_EN == DEF_ENABLED)
    OSStatTaskCPUUsageInit(&err);                               /* Initialize CPU Usage.                                */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);
    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    AppGraphicsInit();                                          /* Initialize the graphics task                         */

    BSP_LedsInit();
    while (DEF_ON) {
        BSP_LedToggle(0);

        OSQPost((OS_Q*      )&demoMsgQueue,
        		(void *)"msgFromMainTask",
                16,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (RTOS_ERR*  ) &err);

        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_PERIODIC, &err);
    }
}

/***************************************************************************//**
 * @brief
 *   This is the idle hook.
 *
 * @detail
 *   This will be called by the Micrium OS idle task when there is no other
 *   task ready to run. We just enter the lowest possible energy mode.
 ******************************************************************************/
static void idleHook(void)
{
  /* Put MCU in the lowest sleep mode available, usually EM2 */
  SLEEP_Sleep();
}

/***************************************************************************//**
 * @brief
 *   Setup the Micrium OS hooks. We are only using the idle task hook in this
 *   example. See the Mcirium OS documentation for more information on the
 *   other available hooks.
 ******************************************************************************/
static void setupHooks(void)
{
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();
    /* Don't allow EM3, since we use LF clocks. */
    SLEEP_SleepBlockBegin(sleepEM3);
    OS_AppIdleTaskHookPtr = idleHook;
    CPU_CRITICAL_EXIT();
}



/*
*********************************************************************************************************
*                                         AppGraphicsInit()
*
* Description : Initialize the graphics task
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void  AppGraphicsInit (void)
{
	RTOS_ERR  err;

	OSTaskCreate(&AppGraphics_TaskTCB,                          /* Create the Graphics Task.                            */
				"Graphics Task",
				AppGraphics_Task,
				DEF_NULL,
				APP_GRAPHICS_TASK_PRIO,
				&AppGraphics_TaskStk[0],
				(APP_GRAPHICS_TASK_STK_SIZE / 10u),
				APP_GRAPHICS_TASK_STK_SIZE,
				0u,
				0u,
				DEF_NULL,
				(OS_OPT_TASK_STK_CLR),
				&err);

	/*   Check error code.                                  */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          AppGraphics_Task()
*
* Description : The main graphics task
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  AppGraphics_Task (void  *p_arg)
{
	RTOS_ERR    err;
	OS_MSG_SIZE demoMsgSize;

	while (DEF_ON) {

		  OSQPend((OS_Q*)&demoMsgQueue,
				 (OS_TICK     ) 0,
				 (OS_OPT      ) OS_OPT_PEND_BLOCKING,
				 (OS_MSG_SIZE*)&demoMsgSize,
				 (CPU_TS*     ) DEF_NULL,
				 (RTOS_ERR*   ) &err);

		BSP_LedToggle(1);
//		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_PERIODIC, &err);
	}
}

#include "Qpos.h"
#include "stdio.h"


uint8_t uListChange = 0; //链表更改状态

volatile uint32_t QposTick=0;  //系统Tick数
volatile uint8_t TickFreq=0;    //系统节拍频率


/**
 * 系统堆内存管理模块
 */
uint32_t OsHeap[Qpos_heap_size];  //系统堆内存
uint32_t uHeapLevel=0;              //当前堆水位

/**
 * func:注册内存块
 * return:注册内存块首地址
 * param:注册大小
 */
uint32_t* pHeapMemRegister(uint32_t size)   
{
    uint32_t *point = &OsHeap[uHeapLevel];
    if((uHeapLevel+size)<Qpos_heap_size)
    {
        uHeapLevel = uHeapLevel+size;
        return point;
    }
    else
    {
        return NULL;
    }
}

/**
 * func:注销内存块(废弃)
 */
void vHeapMemDelete(uint32_t size)
{
    if((uHeapLevel-size)>0)
    {
        uHeapLevel = uHeapLevel-size;
    }
}


/**
 * 任务列表管理模块
 */
TCB_t *pxCurrentTCB;   //当前任务控制块指针
TaskList xTasklist1;   //第一优先级队列
TaskList xTasklist2;    //第二优先级队列
TaskList xTasklist0;    //第零优先级队列
TaskList xTaskdelay;    //延时队列


/**
 * func:任务链表初始化
 * param:链表地址
 */
void vTaskListInit(TaskList* TaskList)
{
    TaskList->TaskNumber = 0;
    TaskList->RootTask.sp = 0x00;
    TaskList->RootTask.pxNext = &(TaskList->RootTask);
    TaskList->RootTask.pxPrev = &(TaskList->RootTask);
}

/**
 * func:添加任务节点
 * param:链表地址，任务控制块地址
 */
void vTaskListAdd(TaskList* TaskList,TCB_t* AddTCB)
{
    AddTCB->pxNext = &(TaskList->RootTask);
    AddTCB->pxPrev = TaskList->RootTask.pxPrev;
    (TaskList->RootTask.pxPrev)->pxNext = AddTCB;
    TaskList->RootTask.pxPrev = AddTCB;


    TaskList->TaskNumber++;
}
/**
 * func:删除任务节点
 * param:链表地址：任务控制块地址
 */
void vTaskListDelete(TaskList* TaskList,TCB_t* DeleteTCB)
{
    (DeleteTCB->pxPrev)->pxNext = DeleteTCB->pxNext;
    (DeleteTCB->pxNext)->pxPrev = DeleteTCB->pxPrev;
    TaskList->TaskNumber--;
}
/**
 * 任务管理模块
 */

 /**
  * func:线程堆栈初始化
  * param:栈顶地址，入口函数地址，入参地址
  * return:栈顶地址减去寄存器保存区后地址
  */
uint32_t * pStackInit(uint32_t *stack_top, void (*task_entry)(void* param),void* param) {
    stack_top--;              // xPSR
    *stack_top = 0x01000000;  // xPSR, 必须是 Thumb 状态

    stack_top--;
    *stack_top = (uint32_t)task_entry;  // PC = 任务入口

    stack_top--;
    *stack_top = 0xFFFFFFFD;  // LR = return to Thread mode using PSP

    stack_top--; *stack_top = 0; // R12
    stack_top--; *stack_top = 0; // R3
    stack_top--; *stack_top = 0; // R2
    stack_top--; *stack_top = 0; // R1
    stack_top--; *stack_top = (uint32_t)param; // R0

    // 手动保存区 R4-R11
    stack_top--; *stack_top = 0; // R11
    stack_top--; *stack_top = 0; // R10
    stack_top--; *stack_top = 0; // R9
    stack_top--; *stack_top = 0; // R8
    stack_top--; *stack_top = 0; // R7
    stack_top--; *stack_top = 0; // R6
    stack_top--; *stack_top = 0; // R5
    stack_top--; *stack_top = 0; // R4

    return stack_top;
}

void vSchedulerLock();
void vSchedulerUnlock();


/**
 * func:任务创建
 * param:任务控制块，入口函数，分配栈大小，任务优先级(0-2),入参地址
 */
TCB_t * vTaskCreate(void (*task_entry)(void* param),uint32_t stack_size,uint32_t priority,void* param)
{
    
	vSchedulerLock();
    uListChange = 1;
    TCB_t * task = (TCB_t*)(pHeapMemRegister(sizeof(TCB_t)));
    uint32_t*stack = pHeapMemRegister(stack_size);
    task->sp =  pStackInit(stack+stack_size-1,task_entry,param);
    task->priority = priority;
    
    switch (priority)
    {
    case 0:
        vTaskListAdd(&xTasklist0,task);
        break;
    case 1:
        vTaskListAdd(&xTasklist1,task);
        break;
    
    case 2:
        vTaskListAdd(&xTasklist2,task); 
        break;
    }
    vSchedulerUnlock();
    return task;
}

/**
 * func:任务删除
 * param:任务控制块
 */
void vTaskDelete(TCB_t* task)
{
    uListChange = 1;
    vSchedulerLock();
    switch (task->priority)
    {
    case 0:
        vTaskListDelete(&xTasklist0,task);
        break;
    case 1:
        vTaskListDelete(&xTasklist1,task);
        break;
    
    case 2:
        vTaskListDelete(&xTasklist2,task); 
        break;
    }
    vSchedulerUnlock();
}

/**
 * func:挂起任务指定时间
 * param:时间(ms)
 */
void vTaskDelay(uint32_t time)
{
    vSchedulerLock();   //进入临界区
    uint32_t timetowake=0;

    switch (pxCurrentTCB->priority)
    {
    case 0:
        time= time+1 -  xTasklist0.TaskNumber;
        break;
    case 1:
        time= time+1 -  xTasklist1.TaskNumber;
        break;
    case 2:
        time= time+1 -  xTasklist2.TaskNumber;
        break;
    }
    
    timetowake = QposTick+time;
    pxCurrentTCB->xWakeTime = timetowake;
    vTaskDelete(pxCurrentTCB);
    vTaskListAdd(&xTaskdelay,pxCurrentTCB);
    

    vSchedulerUnlock();//退出临界区
	port_trigger_PendSV_Handler();  //挂起pend异常，启动调度
}

/**
 * func:初始化第一次调度
 */
__attribute__((naked)) void vStartFirstTask()
{
    __asm volatile 
    (
        /* 读取向量表地址 */
        ".align 8             \n"
        "ldr r0, =0xE000ED08       \n"
        "ldr r0, [r0]              \n"
        "ldr r0, [r0]              \n"

        /* 设置 MSP */
        "msr msp, r0               \n"

        /* 开启中断 */
        "cpsie i                   \n"
        "cpsie f                   \n"
        "dsb                       \n"
        "isb                       \n"

        /* 调用 SVC 启动第一个任务 */
        "svc 0                     \n"
        "nop                       \n"
        "nop                       \n"
        :
        :
        : "r0"  /* 告诉编译器：r0 被修改了 */
    );
}





/**
 * 调度器模块
 */

 /**
  * func:调度函数，触发一次任务调度
  */
void vTaskSwitchContext()
{
    if(xTaskdelay.TaskNumber!=0)
    {
        TCB_t* DelayTask = xTaskdelay.RootTask.pxNext;
        for (uint8_t i = 0; i < (xTaskdelay.TaskNumber); i++)
        {
            if (QposTick>(DelayTask->xWakeTime))
            {
				uListChange = 1;
				vTaskListDelete(&xTaskdelay,DelayTask);
                switch (DelayTask->priority)
                {
                case 0:
                    vTaskListAdd(&xTasklist0,DelayTask);
                    break;
                case 1:
                    vTaskListAdd(&xTasklist1,DelayTask);
                    break;
                case 2:
                    vTaskListAdd(&xTasklist2,DelayTask); 
                    break;
                }
                
            }
            DelayTask = DelayTask->pxNext;
        }
        
    }

    if(uListChange == 1)
    {
        if(xTasklist2.TaskNumber !=0)
        {
                pxCurrentTCB = &(xTasklist2.RootTask);
        }
        else if(xTasklist1.TaskNumber !=0)
        {
                pxCurrentTCB = &(xTasklist1.RootTask);
        }
        else
        {
                pxCurrentTCB =&(xTasklist0.RootTask);
        }
		uListChange =0;
    }


    if ((pxCurrentTCB->pxNext)->sp != 0 )
    {
        pxCurrentTCB = pxCurrentTCB->pxNext;
    }
    else
    {
        pxCurrentTCB = pxCurrentTCB->pxNext->pxNext;
    }
    
    

	// //i = !i;
	// if(i)
	// {
	// 	pxCurrentTCB=&task_a;
	// 	i=0;
	// }
	// else
	// {
	// 	i=1;
	// 	pxCurrentTCB=&task_b;
	// }
}





volatile int iSchedulerLocked = 0;


/**
 * func:调度器锁定
 */
void vSchedulerLock()
{
    __disable_irq();
    iSchedulerLocked++;
    __enable_irq();
}

/**
 * func;:调度器解锁
 */
void vSchedulerUnlock()
{
    __disable_irq();
    iSchedulerLocked--;
    __enable_irq();
}


TCB_t xFreeTask;
/**
 * func:空闲任务
 */
void vFreeTask()
{
    while (1)
    {
        /* code */
    }
    
}




/**
 * 初始化
 */
void vSystickInit(uint8_t uwTickFreq);

/**
 * func:系统初始化
 */
void vQposInit()
{
    vTaskListInit(&xTasklist1); 
    vTaskListInit(&xTasklist2);
    vTaskListInit(&xTasklist0);
    vTaskListInit(&xTaskdelay);
    xTasklist0.RootTask.priority = 0;
    xTasklist1.RootTask.priority = 1;
    xTasklist2.RootTask.priority = 2;

    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1); //设定pendsv中断优先级为最低(15)
    vSystickInit(1);    //初始化系统节拍频率为1KHZ
    vTaskCreate(vFreeTask,50,0,0);   //创建空闲任务
}

/**
 * func:启动调度系统
 */
void vQposStart()
{
	vTaskSwitchContext();  //触发一次调度
    vStartFirstTask();
}



uint32_t uGetOsTick()
{
    return QposTick;
}




/**
 * 异常入口及异常控制
 */
void vTriggerContextSwitch()
{
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}
void vSystickInit(uint8_t uwTickFreq)
{
    TickFreq = uwTickFreq;
      /* Configure the SysTick to have interrupt in 1ms time basis*/
	HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq));


  /* Configure the SysTick IRQ priority */
  if (14 < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, 14, 0U);
    uwTickPrio = 15;
  }

    SysTick->VAL   = 0;                           
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void PortPendSVhandler(void)
{
	__asm volatile 
    (
    ".align 8                     \n"  
    "cpsid   i \n"                  // 关中断
    "mrs     r0, psp\n"           // 获取当前任务 PSP
    "stmdb   r0!, {r4-r11} \n"     // 保存寄存器 R4~R11 到任务栈中

    "ldr     r1, =pxCurrentTCB\n"
    "ldr     r2, [r1]     \n"      // 取出当前任务 TCB
    "str     r0, [r2]  \n"         // 保存 PSP 到 TCB 中

    "bl      vTaskSwitchContext \n"// 切换任务（更新 pxCurrentTCB）

    "ldr     r1, =pxCurrentTCB \n"
    "ldr     r2, [r1]  \n"         // 获取新任务 TCB
    "ldr     r0, [r2]  \n"         // 取出新任务的 PSP

    "ldmia   r0!, {r4-r11} \n"     // 恢复新任务的寄存器
    "msr     psp, r0   \n"         // 写回 PSP

    "cpsie   i \n"
    "mov lr,#0xfffffffd  \n"
    "bx      lr \n"                // 返回，恢复新任务上下文
    "nop   \n"
	);
}
void PortSVChandler(void)
{
    __asm volatile 
    (
        ".align 8             \n"
        "   ldr r1, =pxCurrentTCB \n"
        "   ldr r2, [r1]\n"
        "   ldr r0, [r2]\n"
        "   ldmia r0!, {r4-r11}\n"
        "   msr psp, r0\n"
        "   isb			\n"
        "   mov r0, #0\n"
        "   msr basepri, r0\n"
        // "mrs     r0, control\n"
        // "orr     r1, r0, #3 \n"
        // "msr     control,r1 \n"
        // "dsb                       \n"
        // "isb                       \n"
        "   mov lr,#0xfffffffd  \n"
        "   bx lr\n"
		:
		: 
		: "r0", "r1", "r3", "memory"
    );
}
void port_trigger_PendSV_Handler(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


void PortSysTickHandler(void)
{
    QposTick+=TickFreq;
    if(iSchedulerLocked == 0)
    {
        port_trigger_PendSV_Handler();
    }
}









uint8_t i=0;










//void cesi()
//{
//    task_a.sp = pStackInit(&stack_a[50],taska);
//    task_b.sp = pStackInit(&stack_b[50],taskb);
//NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
//	pxCurrentTCB = &task_a;
//    vSystickInit(1);
//    //__set_CONTROL(__get_CONTROL() | 0x03);
//    vStartFirstTask();
//}















#ifndef __QPOS_H
#define __QPOS_H
#include "main.h"
#define Qpos_heap_size 4096  //单位4bytes

#define PortSVChandler SVC_Handler                  //SVC异常入口
#define PortPendSVhandler PendSV_Handler            //PendSV异常入口
#define PortSysTickHandler SysTick_Handler          //Systick异常入口







/**
 * 任务状态枚举类型
 */
typedef enum TaskState
{
    run=0,ready,pend,delay
}TaskState;


/**
 * 任务控制块结构体
 */
typedef struct TCB_t{
    uint32_t *sp;    // 任务栈顶
    uint8_t priority;//任务优先级
    TaskState state;//任务状态
    uint32_t xWakeTime;//Delay时唤醒时间
    struct TCB_t *pxPrev;//下一任务地址
    struct TCB_t *pxNext;//上一任务地址
}TCB_t;
/**
 * 任务链表结构体
 */
typedef struct
{
    uint32_t TaskNumber; //节点个数
    TCB_t RootTask;     //根节点
}TaskList;



void vQposInit();
void vQposStart();
TCB_t * vTaskCreate(void (*task_entry)(void* param),uint32_t stack_size,uint32_t priority,void* param);
void vTaskDelete(TCB_t* task);
void vTaskDelay(uint32_t time);
uint32_t uGetOsTick();

void asas();
void cesi();
// void PortPendSVhandler();
// void PortSVChandler();
void port_trigger_PendSV_Handler(void);

#endif
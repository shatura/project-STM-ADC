//
// Файл для контроля TASK во FreeRTOS
//

#include "FeeeRTOS.h"
#include "task.h"
#include "queue.h"

defaultTaskHandle;

const defaultTaskHandle = {.name = "defaultTask" ,.priority = (osPriority_t) osPriorityNormal, .stack_size = 128*4 };

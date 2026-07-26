#ifndef APP_H
#define APP_H

/**
 * @file App.h
 * @brief 整车程序对 main.c 暴露的最小入口。
 *
 * 项目只有一辆车，因此这里不公开 App 对象或句柄。所有硬件绑定、运行数据
 * 和状态转换都由 App.c 内唯一的整车状态机管理。
 */

#include "stm32f1xx_hal.h"

/** 初始化核心硬件和应用模块；失败时保持电机关闭并进入 FAULT。 */
HAL_StatusTypeDef App_Init(void);

/** 非阻塞推进一次整车状态机；main.c 应在 while(1) 中持续调用。 */
void App_Update(void);

#endif /* APP_H */

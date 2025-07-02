#ifndef DELAY_H
#define DELAY_H

#include "stm32f1xx_hal.h" // Thay đổi nếu bạn dùng dòng chip khác, ví dụ stm32f4xx_hal.h

#ifdef __cplusplus
extern "C" {
#endif

void Delay_Init(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms); // Gọi HAL_Delay bên trong

#ifdef __cplusplus
}
#endif

#endif // DELAY_H

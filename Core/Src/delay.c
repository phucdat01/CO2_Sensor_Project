#include "delay.h"

// Khởi tạo DWT để sử dụng bộ đếm chu kỳ CPU
void Delay_Init(void)
{
    // Kích hoạt DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Đặt lại bộ đếm
    DWT->CYCCNT = 0;
    // Bật bộ đếm
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Hàm delay đơn vị micro giây
void Delay_us(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - startTick) < delayTicks);
}

// Hàm delay đơn vị mili giây, sử dụng HAL
void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

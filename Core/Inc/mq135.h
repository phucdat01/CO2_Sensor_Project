#ifndef MQ135_H
#define MQ135_H

#include "main.h" // Bao gồm các định nghĩa HAL từ STM32CubeIDE

// Các hằng số giữ nguyên từ thư viện gốc
#define PARA 116.6020682
#define PARB 2.769034857
#define CORA .00035
#define CORB .02718
#define CORC 1.39538
#define CORD .0018
#define CORE -.003333333
#define CORF -.001923077
#define CORG 1.130128205
#define ATMOCO2 429.64 // Global CO2 2024

// Cấu trúc lưu trữ thông tin MQ135
typedef struct {
    ADC_HandleTypeDef* hadc; // Con trỏ đến cấu trúc ADC
    uint32_t adcChannel;     // Kênh ADC (ví dụ: ADC_CHANNEL_0)
    float rload;             // Điện trở tải trên board (kOhm)
    float rzero;             // Điện trở chuẩn tại CO2 khí quyển
} MQ135;

// Khai báo các hàm
void MQ135_Init(MQ135* sensor, ADC_HandleTypeDef* hadc, uint32_t adcChannel, float rzero, float rload);
float MQ135_GetCorrectionFactor(float t, float h);
float MQ135_GetResistance(MQ135* sensor);
float MQ135_GetCorrectedResistance(MQ135* sensor, float t, float h);
float MQ135_GetPPM(MQ135* sensor);
float MQ135_GetCorrectedPPM(MQ135* sensor, float t, float h);
float MQ135_GetRZero(MQ135* sensor);
float MQ135_GetCorrectedRZero(MQ135* sensor, float t, float h);

#endif

#include "MQ135.h"
#include <math.h>

void MQ135_Init(MQ135* sensor, ADC_HandleTypeDef* hadc, uint32_t adcChannel, float rzero, float rload) {
    sensor->hadc = hadc;
    sensor->adcChannel = adcChannel;
    sensor->rzero = rzero;
    sensor->rload = rload;
}

float MQ135_GetCorrectionFactor(float t, float h) {
    if (t < 20) {
        return CORA * t * t - CORB * t + CORC - (h - 33.) * CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

float MQ135_GetResistance(MQ135* sensor) {
    HAL_ADC_Start(sensor->hadc);
    if (HAL_ADC_PollForConversion(sensor->hadc, 200) != HAL_OK) {
    	return -1; // Lỗi ADC
    }
    uint32_t adcValue = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);
    float voltage = (adcValue * 3.3) / 4095.0;
    if (voltage <= 0) {
    	return -1;
    }
    float resistance = ((3.3 - voltage) * sensor->rload) / voltage;
    return resistance;
}

float MQ135_GetCorrectedResistance(MQ135* sensor, float t, float h) {
    return MQ135_GetResistance(sensor) / MQ135_GetCorrectionFactor(t, h);
}

float MQ135_GetPPM(MQ135* sensor) {
    return PARA * pow((MQ135_GetResistance(sensor) / sensor->rzero), -PARB);
}

float MQ135_GetCorrectedPPM(MQ135* sensor, float t, float h) {
    return PARA * pow((MQ135_GetCorrectedResistance(sensor, t, h) / sensor->rzero), -PARB);
}

float MQ135_GetRZero(MQ135* sensor) {
    return MQ135_GetResistance(sensor) * pow((ATMOCO2 / PARA), (1. / PARB));
}

float MQ135_GetCorrectedRZero(MQ135* sensor, float t, float h) {
    return MQ135_GetCorrectedResistance(sensor, t, h) * pow((ATMOCO2 / PARA), (1. / PARB));
}

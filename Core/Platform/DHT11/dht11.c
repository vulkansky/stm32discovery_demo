/*
 * dht11.c
 *
 *  Created on: 20 окт. 2021 г.
 *      Author: Kirill
 */

#include "dht11.h"


#define lineDown() 		LL_GPIO_ResetOutputPin(sensor->DHT_Port, sensor->DHT_Pin)
#define lineUp()		LL_GPIO_SetOutputPin(sensor->DHT_Port, sensor->DHT_Pin)
#define getLine()		(LL_GPIO_ReadInputPort(sensor->DHT_Port)&sensor->DHT_Pin)
#define Delay(d)		LL_mDelay(d)

static void goToOutput(DHT_sensor *sensor) {

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  //По умолчанию на линии высокий уровень
  lineUp();

  //Настройка порта на выход
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  if(sensor->pullUp == 1) {
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  } else {
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  }
  LL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

static void goToInput(DHT_sensor *sensor) {

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  //Настройка порта на вход
	 GPIO_InitStruct.Pin = DHT11_Pin;
	 GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	  if(sensor->pullUp == 1) {
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  } else {
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  }
	 LL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

DHT_data DHT_getData(DHT_sensor *sensor) {
	DHT_data data = {0.0f, 0.0f};

	#if DHT_POLLING_CONTROL == 1
	/* Ограничение по частоте опроса датчика */
	//Определение интервала опроса в зависимости от датчика
	uint16_t pollingInterval;
	if (sensor->type == DHT11) {
		pollingInterval = DHT_POLLING_INTERVAL_DHT11;
	} else {
		pollingInterval = DHT_POLLING_INTERVAL_DHT22;
	}

//	//Если частота превышена, то возврат последнего удачного значения
//	if (HAL_GetTick()-sensor->lastPollingTime < pollingInterval) {
//		data.hum = sensor->lastHum;
//		data.temp = sensor->lastTemp;
//		return data;
//	}
//	sensor->lastPollingTime = HAL_GetTick();
	#endif

	/* Запрос данных у датчика */
	//Перевод пина "на выход"
	goToOutput(sensor);
	//Опускание линии данных на 15 мс
	lineDown();
	Delay(15);
	//Подъём линии, перевод порта "на вход"
	lineUp();
	goToInput(sensor);

	/* Ожидание ответа от датчика */
	uint16_t timeout = 0;
	//Ожидание спада
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}
	timeout = 0;
	//Ожидание подъёма
	while(!getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}
	timeout = 0;
	//Ожидание спада
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}

	/* Чтение ответа от датчика */
	uint8_t rawData[5] = {0,0,0,0,0};
	for(uint8_t a = 0; a < 5; a++) {
		for(uint8_t b = 7; b != 255; b--) {
			uint32_t hT = 0, lT = 0;
			//Пока линия в низком уровне, инкремент переменной lT
			while(!getLine()) lT++;
			//Пока линия в высоком уровне, инкремент переменной hT
			timeout = 0;
			while(getLine()) hT++;
			//Если hT больше lT, то пришла единица
			if(hT > lT) rawData[a] |= (1<<b);
		}
	}
	/* Проверка целостности данных */
	if((uint8_t)(rawData[0] + rawData[1] + rawData[2] + rawData[3]) == rawData[4]) {
		//Если контрольная сумма совпадает, то конвертация и возврат полученных значений
		if (sensor->type == DHT22) {
			data.hum = (float)(((uint16_t)rawData[0]<<8) | rawData[1])*0.1f;
			//Проверка на отрицательность температуры
			if(!(rawData[2] & (1<<7))) {
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*0.1f;
			}	else {
				rawData[2] &= ~(1<<7);
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*-0.1f;
			}
		}
		if (sensor->type == DHT11) {
			data.hum = (float)rawData[0];
			data.temp = (float)rawData[2];;
		}
	}

	#if DHT_POLLING_CONTROL == 1
	sensor->lastHum = data.hum;
	sensor->lastTemp = data.temp;
	#endif

	return data;
}

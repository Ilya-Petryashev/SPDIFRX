# SPDIFRX
Приемник цифрового звука на основе STM32F7.
В STM32CubeMX был сгенерирован код инициализации.
В MDK-ARM Keil uVision 5 был написан программный код, заполняющий массив принятых данных и массив служебных данных.
В случае успешного приема (или если приемник не дождался данных на входе) программа входит в бесконечый цикл мигания зеленым светодиодом.
В случае ошибки вызывается обработчик ошибки - в нем загорается красный светодиод, далее следует бесконечный цикл.

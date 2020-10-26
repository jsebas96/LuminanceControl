# Evaluación de Estrategias de Control basadas en Diferentes Enfoques para un Entorno Cerrado de Iluminación.
Códigos utilizados para el desarrollo de estrategias de control enfocadas al control de iluminación de ambientes cerrados.

## Motivación
* Consumo energético elevado: 2900 TWh anual en iluminación
* Se espera un crecimiento del 50% de la demanda en servicios de iluminación para las próximas dos décadas.
* Aparición de tecnologías que disminuyen el consumo (luminarias LED)
* Desarrollo de herramientas basadas en IoT, dispositivos inalámbricos como sensores y actuadores.

## Objetivo
Evaluar el desempeño de diferentes estrategias de control para un sistema de iluminación en un ambiente cerrado, aplicado al laboratoro de robótica de la Universidad de Nariño.

## Controladores
+ PI : Proporcional Integral
+ RD : Dinámicas de Replicadores
+ MPC: Controlador Predictivo basado en Modelo

## Elementos Empleados
+ Sensor de iluminancia: TSL2561 (I2C Interface)
+ Filtrado y envío de Datos: Tarjeta de desarrollo ESP32(Bluetooth)
+ Servidor para almacenamiento y control (envío de señal con radio Z-Wave): PC con SO Raspbian
+ Actuador: Dimmer que traduce la señal para las luminarias (recepción protocolo Z-Wave)

## Diagrama de Flujo del Sistema
![alt text](https://github.com/jsebas96/LuminanceControl/blob/master/diagram.jpeg)

## Resultado
El desarrollo de este tipo de aplicaciones proporciona una solución inteligente para el establecimiento de un adecuado nivel de iluminación en ambientes cerrados pero además, gestiona de manera adecuada la energía pues los recursos se administran de mejor manera disminuyendo el consumo eléctrico, incluso con la tećnicas de control más básicas.

## Documentación
https://cutt.ly/lumcontrol

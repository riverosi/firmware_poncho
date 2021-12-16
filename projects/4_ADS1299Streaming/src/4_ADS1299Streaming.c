/* Copyright 2020,
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Initials     Name
 * ---------------------------
 * SM Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20200923 v0.0.1 SM initial version
 */

/*!\mainpage
*
* \section genDesc Descripción de la aplicacion
*	Sistema de adquisición monocanal de una señal de ECG. Utiliza el canal 2 en modo diferencial para adquirir, a una frecuencia de muestreo de 250Hz,
*	los datos en 24bits. Estos datos son enviados por UART USB en un formato compatible con el hardware de OpenBCI. Dando posibilidad de utilizarlo
*	con cualquier software que implemente este tipo de datos.
*
* ---
*
* \subsection subSec01 Funcionamiento
*	Parpadeo del led azul RGB a una frecuencia de 1Hz y envío de los datos convertidos cada 4ms por UART USB.
*	Para utilizar en la EDU CIAA NXP, al ser un poncho para dicha placa consultar la bibligrfía en la página
*	https://ponchodebiopotenciales.wordpress.com/ para más información
*
*/

/*==================[inclusions]=============================================*/
#include "4_ADS1299Streaming.h"

#include "ADS1299.h"
#include "led.h"
#include "uart.h"
#include "systick.h"
#include "delay.h"
#include "dsp_EMG.h"

/*==================[macros and definitions]=================================*/
#define BUFFLEN 128
#define BAUD_RATE_RS485 921600
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
void SystickInt(void);
void ReadData(void);
void ConfigADS(void);

/*==================[internal data definition]===============================*/
bool new_data = false;
int32_t channel_data[8];
uint8_t state[3];
RINGBUFF_T rbRx;
float32_t dataBuff[BUFFLEN];
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/*/fn void SysInit(void)
 * \brief Inicializacion principal
 *
 */
void SysInit(void)
{
	SystemCoreClockUpdate();
	Chip_SetupIrcClocking();
	LedsInit();
	ADS1299Init();
    SystickInit(500, SystickInt);
    fpuInit();//Enable FPU
	serial_config rs485_init = {SERIAL_PORT_RS485, BAUD_RATE_RS485, NULL};
	UartInit(&rs485_init);
    RingBuffer_Init(&rbRx, dataBuff, sizeof(float32_t), BUFFLEN);
}

void SystickInt(void)
{
	LedToggle(LED_RGB_B);
}

void ReadData(void)
{
	new_data = true;
}

void ConfigADS(void)
{
	/* Configura el canal 2 en modo diferencial, frecuencia de muestreo de 250Hz y el driver de pierna derecha */
	ADS1299SetChannelsToDefaultConfigForEMG();
	//ADS1299SetChannelsToDefaultConfigForECG();
	/* Cambia la ganancia a 1 para poder utilizarlo con un generador de funciones y que no sature el ADS */
	//ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL2, ADS1299_GAIN01);

	/* Datos convertidos */
	/* Indica a traves de la funcion de interrupcion ReadData que hay un dato nuevo en el vector channel_data y state.
	 * Los datos son de 24bits en complemento a 2.
	 */
	ADS1299ActivateInt(ReadData, channel_data, state);

	/* Comienzo de la conversión continua */
	ADS1299StartStreaming();
}

/*==================[external functions definition]==========================*/
int main(void)
{
	uint8_t uart_buffer[24] = {};
	float32_t aux;
	 union {
		float32_t ptpValue;
		uint8_t buffer[4];
	 } u_buffer;

	SysInit();
	ConfigADS();

	while(1)
	{
		if(new_data)
		{
			//Envio del canal 1 a traves de la UART USB en formato de datos de OpenBCI
			uart_buffer[0] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>16);
			uart_buffer[1] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>8);
			uart_buffer[2] = (uint8_t) (channel_data[ADS1299_CHANNEL1]);

			uart_buffer[3] = (uint8_t) (channel_data[ADS1299_CHANNEL2]>>16);
			uart_buffer[4] = (uint8_t) (channel_data[ADS1299_CHANNEL2]>>8);
			uart_buffer[5] = (uint8_t) (channel_data[ADS1299_CHANNEL2]);

			uart_buffer[6] = (uint8_t) (channel_data[ADS1299_CHANNEL3]>>16);
			uart_buffer[7] = (uint8_t) (channel_data[ADS1299_CHANNEL3]>>8);
			uart_buffer[8] = (uint8_t) (channel_data[ADS1299_CHANNEL3]);

			uart_buffer[9] = (uint8_t) (channel_data[ADS1299_CHANNEL4]>>16);
			uart_buffer[10] = (uint8_t) (channel_data[ADS1299_CHANNEL4]>>8);
			uart_buffer[11] = (uint8_t) (channel_data[ADS1299_CHANNEL4]);

			uart_buffer[12] = (uint8_t) (channel_data[ADS1299_CHANNEL5]>>16);
			uart_buffer[13] = (uint8_t) (channel_data[ADS1299_CHANNEL5]>>8);
			uart_buffer[14] = (uint8_t) (channel_data[ADS1299_CHANNEL5]);

			uart_buffer[15] = (uint8_t) (channel_data[ADS1299_CHANNEL6]>>16);
			uart_buffer[16] = (uint8_t) (channel_data[ADS1299_CHANNEL6]>>8);
			uart_buffer[17] = (uint8_t) (channel_data[ADS1299_CHANNEL6]);

			aux = (float32_t)(~(channel_data[0] - 1));
			RingBuffer_Insert(&rbRx, &aux);//insert element in ring buffer
			ADS1299SendUART(uart_buffer);
			new_data = false;
		}

		if (RingBuffer_IsFull(&rbRx))
		{
			u_buffer.ptpValue = dsp_emg_rms_f32(dataBuff, BUFFLEN);//Compute RMS value
			for (int var = 0; var < 4; var++) //Send by RS485
			{
				UartSendByte(SERIAL_PORT_RS485, &u_buffer.buffer[var]);
			}
			RingBuffer_Flush(&rbRx);//Flush ring buffer
		}

	}
	return 0;
}
/*==================[end of file]============================================*/


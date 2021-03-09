/*
 * main.c
 *
 *  Created on: 2021 Mar 07 23:23:48
 *  Author: HOME
 */

#include <xmc_common.h>
#include <stdio.h>
#include "systimer.h"
#include "uart.h"

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. 
 */

int main(void)
{
	 SYSTIMER_STATUS_t status;

	 UART_Init(&UART_0);

	 status= SYSTIMER_Init(&SYSTIMER_0);

	 if(status != SYSTIMER_STATUS_SUCCESS)
	 {
		 printf("system timer init fail\r\n");
		 return 0;
	 }

	 //every 10ms uart receive check
	 uint32_t TimerId=0;
     TimerId = (uint32_t)SYSTIMER_CreateTimer(TENMILLSEC,SYSTIMER_MODE_PERIODIC,UART_Receive_Check,&UART_0);
	 if (TimerId != 0U)
	 {
		printf("timer create success\r\n");
	    status = SYSTIMER_StartTimer(TimerId);
	   if (status == SYSTIMER_STATUS_SUCCESS)
	   {
		   printf("timer start success\r\n");
	   }
	 }

	const char * sendTxt= "Start xmc1400 boot kit\r\n";
	UART_Transmit(&UART_0,(uint8_t*)sendTxt,strlen(sendTxt));

  while(1U)
  {
	 // delay(65000);
	 // UART_Receive_Check(&UART_0);


  }
}

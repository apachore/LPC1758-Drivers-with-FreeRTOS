/*
 * hw_UART.hpp
 *
 *  Created on: Mar 4, 2016
 *      Author: Amit
 */

#ifndef L5_APPLICATION_HW_UART_HPP_
#define L5_APPLICATION_HW_UART_HPP_

#include "tasks.hpp"
#include <stdio.h>
#include "sys_config.h"
#include "LPC17xx.h"
#include "io.hpp"


#define baud_rate 9600
#define uart_x 3

bool data_received_flag = 0;

extern "C"
{
void UART2_IRQHandler(void)//Original UART2_IRQHandler from uart2.cpp is commented
{
    LE.toggle(1);
    QueueHandle_t UART_Rec_Buff = scheduler_task::getSharedObject("UART_Receive_Q");
    uint16_t Int_ID_R = (LPC_UART2->IIR & 0xE);
    if (Int_ID_R == 4)
    {
        LE.toggle(2);
        char ch;
        ch = LPC_UART2->RBR;
        if(!xQueueSendFromISR(UART_Rec_Buff,&ch,NULL))
            LE.toggle(3);
        data_received_flag = 1;
    }
}
}

extern "C"
{
void UART3_IRQHandler(void)//Original UART3_IRQHandler from uart3.cpp is commented
{
    LE.toggle(1);
    QueueHandle_t UART_Rec_Buff = scheduler_task::getSharedObject("UART_Receive_Q");
    uint16_t Int_ID_R = (LPC_UART3->IIR & 0xE);
    if (Int_ID_R == 4)
    {
        LE.toggle(2);
        char ch;
        ch = LPC_UART3->RBR;
        if(!xQueueSendFromISR(UART_Rec_Buff,&ch,NULL))
            LE.toggle(3);
        data_received_flag = 1;
    }
}
}


class UARTTask : public scheduler_task
{
    public: UARTTask(uint8_t priority): scheduler_task("UARTTask", 2048, priority)
    {

    }

    QueueHandle_t UART_Rec_Q =0;

    void Baudrate_Setup(LPC_UART_TypeDef * uart, int baudrate)
    {
        uart -> LCR = (1<<7);//DLAB bit set to 1

        uint16_t DL_Value = (sys_get_cpu_clock()/((16*baudrate)+0.5));
        //printf("DL: %x\n",DL_Value);

        uart ->DLM = (DL_Value >> 8);
        uart ->DLL = (DL_Value >> 0);

        uart ->FDR = 0x10;

        uart -> LCR = 3;//DLAB set back to 0 and 8 bit transfer mode selected
    }


    bool init()
    {
        switch(uart_x)
        {
            case 2:
                LPC_SC -> PCONP |= (1<<24); //Enable UART2(24)UART3(25) in Peripheral Control Reg

                //Peripheral Clock setup
                LPC_SC -> PCLKSEL1 &= ~(3 << 16); //Initial zero at 16 & 17 bits to make 00(18 & 19 for UART3)
                LPC_SC -> PCLKSEL1 |= (1 << 16); //01 selection for clock = CCLK for UART2(18 & 19 for UART3)

                Baudrate_Setup(LPC_UART2, baud_rate);

                //Pin Selection from MUX
                LPC_PINCON -> PINSEL4 &= ~(0xF << 16);  //Pin selection made to 00-00, Pin P2.8, P2.9 initialized to 00
                LPC_PINCON -> PINSEL4 |= (0xA << 16); // 10-10----- Pin select 10 for TXD1, RXD2

                //Interrupt Initialization
                LPC_UART2 ->IER = (1<<0);//Enable RDA interrupt
                NVIC_EnableIRQ(UART2_IRQn);

                break;

            case 3:
                LPC_SC -> PCONP |= (1<<25); //Enable UART2(24)UART3(25) in Peripheral Control Reg

                //Peripheral Clock setup
                LPC_SC -> PCLKSEL1 &= ~(3 << 18); //Initial zero at 16 & 17 bits to make 00(18 & 19 for UART3)
                LPC_SC -> PCLKSEL1 |= (1 << 18); //01 selection for clock = CCLK for UART2(18 & 19 for UART3)

                Baudrate_Setup(LPC_UART3, baud_rate);

                //Pin Selection from MUX
                LPC_PINCON -> PINSEL9 &= ~(0xF << 24);
                LPC_PINCON -> PINSEL9 |= (0xF << 24);
                //4.28,4.29, PINSEL9 24,25,26,27 - TXD3,RXD3 - F - for UART 3

                //Interrupt Initialization
                LPC_UART3 ->IER = (1<<0);//Enable RDA interrupt
                NVIC_EnableIRQ(UART3_IRQn);

                break;

            default:
                printf("UART initialization failed. Please select correct UART");
                break;
        }

        //Receive Queue - Received data is soreed in this queue after in interrupt processing
        UART_Rec_Q = xQueueCreate(1024,sizeof(char));
        addSharedObject("UART_Receive_Q",UART_Rec_Q);

        return true;
    }

    void uart_send_byte(LPC_UART_TypeDef * uart, char data)
    {
        uart->THR = data;
        while(!(uart->LSR &(1<<5)));
    }

    void uart_send_line(const char data[])
    {
        LPC_UART_TypeDef * uart;
        uart = (uart_x == 2 ? LPC_UART2:LPC_UART3);
        for (int i=0;data[i]!='\0';i++)
        {
            uart_send_byte(uart,data[i]);
        }
    }

    void uart_q_receive()
    {
        char ch;
        while(xQueueReceive(UART_Rec_Q,&ch,0))
        {
            printf("%c",ch);
        }
        data_received_flag = 0;
    }

    bool run(void *p)
    {
        //char str[50] = "Hello from Amit";
        //uart_send_line(str);
        //vTaskDelay(1000);

        if(data_received_flag)
            uart_q_receive();

        return true;
    }
};


CMD_HANDLER_FUNC(UartSend)
{
    UARTTask UarFrmCmd(PRIORITY_HIGH);
    UarFrmCmd.uart_send_line(cmdParams());
    return true;
}
#endif /* L5_APPLICATION_HW_UART_HPP_ */

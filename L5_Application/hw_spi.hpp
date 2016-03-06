/*
 * hw_spi.hpp
 *
 *  Created on: Feb 25, 2016
 *      Author: Amit
 */

#ifndef L5_APPLICATION_HW_SPI_HPP_
#define L5_APPLICATION_HW_SPI_HPP_

#include "tasks.hpp"
#include <stdio.h>
#include "L4_IO/fat/disk/spi_flash.h"

typedef enum {
    opcode_read_status_reg        = 0xD7,
    opcode_read_device_id         = 0x9F,
    opcode_cont_read_low_freq     = 0x03,
    opcode_main_mem_page_read     = 0xD2,

    Peripheral_BSY_BIT            = (1<<4),
    Manufacturer_ID               = 0x1F,
    Device_ID                     = 0x2500,
    SSP1_Chip_Select_P0_6         = 6,

    Dummy_Byte                    = 0xFF

} flash_opcode;

class SPITask : public scheduler_task
{
        public: SPITask(uint8_t priority):
                scheduler_task("SPITask", 2048, priority)
           {

           }

        bool init()
        {
            LPC_SC -> PCONP |= (1<<10); //Enable SSP1 in Control Register

            //Peripheral Clock setup
            LPC_SC -> PCLKSEL0 &= ~(3 << 20); //Initial zero at 20 & 21 bits to make 00
            LPC_SC -> PCLKSEL0 &= ~(1 << 20); //01 selection for clock = CCLK
            LPC_SSP1 -> CPSR = 2; // Output Clock CCLK/2

            //Pin Selection from MUX
            LPC_PINCON -> PINSEL0 &= ~(0xFF << 12);  //Pin selection made to zero, Pin P0.6 selected as GPIO
            LPC_PINCON -> PINSEL0 |= (0x2A << 14); // xx-10-10-10----- Pin select 10 for SCK1,MISO1,MOSI1

            //Configure P0.6 as Output for Chip select
            LPC_GPIO0 ->FIODIR |= (1<<SSP1_Chip_Select_P0_6);

            //8 bit transfer selected
            LPC_SSP1 -> CR0 = 7;
            LPC_SSP1 -> CR1 = (1<<1);

            //printf("Init success\n");

            cs_ssp1();
            Read_Dev_ID();
            ds_ssp1();

            return true;
        }

        char ssp_byte_exchange(char data)
        {
            LPC_SSP1 -> DR = data;
            while (LPC_SSP1 -> SR & Peripheral_BSY_BIT);
            return LPC_SSP1 -> DR;
        }

        void cs_ssp1()     {LPC_GPIO0 -> FIOCLR = (1<<SSP1_Chip_Select_P0_6);}

        void ds_ssp1()     {LPC_GPIO0 -> FIOSET = (1<<SSP1_Chip_Select_P0_6);}

        void Read_Dev_ID()
        {
            uint8_t Rec_MF_ID;
            uint16_t Rec_Dev_ID = 0;

            ssp_byte_exchange(opcode_read_device_id);
            Rec_MF_ID = (uint8_t)ssp_byte_exchange(Dummy_Byte);
            Rec_Dev_ID = Rec_Dev_ID | (((uint8_t)ssp_byte_exchange(Dummy_Byte) << 8) |
                                       (uint8_t)ssp_byte_exchange(Dummy_Byte));

            if (Rec_MF_ID != Manufacturer_ID)
            {
                printf("Error: Manufacturer ID mismatch\n");
                exit(0);
            }

            if (Rec_Dev_ID != Device_ID)
            {
                printf("Error: Device ID mismatch\n");
                exit(0);
            }

            printf("Manufacturer ID:    %x\n", Rec_MF_ID);
            printf("Device ID:          %x\n", Rec_Dev_ID);
        }

        void Read_Flash_Status_Reg()
        {
            uint8_t Rec_Status_Reg;
            ssp_byte_exchange(opcode_read_status_reg);
            Rec_Status_Reg = (uint8_t)ssp_byte_exchange(Dummy_Byte);
            printf("Flash Status Register: %x\n",Rec_Status_Reg);

            printf("Bit 7:   %d - Device is %s\n", ((Rec_Status_Reg & (1<<7))>>7), (Rec_Status_Reg & (1<<7)) == 1 ? "busy":"ready");
            printf("Bit 6:   %d - Main and Buffer comparison  %s\n", ((Rec_Status_Reg & (1<<6))>>6), (Rec_Status_Reg & (1<<6)) == 0 ? "matched":"mismatched");
            printf("Bit 5-2: %x - Device Density is %d\n", ((Rec_Status_Reg & (0xF<<2))>>2),((Rec_Status_Reg & (0xF<<2))>>2));
            printf("Bit 1:   %d - Sector Protection is %s\n", ((Rec_Status_Reg & (1<<1))>>1),(Rec_Status_Reg & (1<<1)) == 1 ? "enabled":"disabled");
            printf("Bit 0:   %d - Page size configuration is %s\n", (Rec_Status_Reg & (1<<0)),
                    (Rec_Status_Reg & (1<<0)) == 1 ? "binary page size (256 bytes)":"Atmel DataFlash page size(264bytes)");
        }

        void Read_Boot_Sector()//Boot sector Address is set to 00H
        {
            uint i;
            uint16_t page_no = 0x07E;
            uint8_t byte_offset=0x00;
            uint32_t addr = 0x00000000;
            addr = addr|((page_no<<8)|byte_offset);
            volatile uint8_t data[512];
            uint8_t opcode_address[] = {opcode_cont_read_low_freq,(uint8_t)(addr>>16),
                                        (uint8_t)(addr>>8),(uint8_t)(addr>>0)};

            for (i=0;i<sizeof(opcode_address);i++)
            {
                printf("Address %x \n",opcode_address[i]);
                ssp_byte_exchange(opcode_address[i]);
            }

            for (i=0;i<512;i++)
            {
                data[i] = (uint8_t)ssp_byte_exchange(Dummy_Byte);
            }
            i=0;
            /*while(i<512)
            {
                for (j=0;j<16;j++)
                {
                    printf("%x ",data[i]);
                    i++;
                }
                printf("\n");
            }*/

            printf("Bytes per sector:              %d\n",((data[12]<<8)|data[11]));
            printf("Number of sectors per Cluster: %d\n",data[13]);
            printf("Total number of Sectors:       %d\n",((data[20]<<8)|data[19]));


        }

        bool run(void *p)
        {
            cs_ssp1();
            //Read_Dev_ID(); //Read device ID moved to init function Feb 27
            Read_Flash_Status_Reg();
            ds_ssp1();
            cs_ssp1();
            Read_Boot_Sector();
            ds_ssp1();

            vTaskDelay(1000);

            return true;
        }
};

#endif /* L5_APPLICATION_HW_SPI_HPP_ */

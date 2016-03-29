/*
 * hw_i2c_slave.hpp
 *
 *  Created on: Mar 12, 2016
 *      Author: Amit
 */

#ifndef L5_APPLICATION_HW_I2C_SLAVE_HPP_
#define L5_APPLICATION_HW_I2C_SLAVE_HPP_

#include "i2c2.hpp"
#include "io.hpp"


class i2cSlave : public scheduler_task
{
    private:
        uint8_t slaveAddr = 0xC0;
        public:
        I2C2& i2cslave = I2C2::getInstance();
        uint8_t buffer[10] = {0};
        i2cSlave(uint8_t priority) :
            scheduler_task("i2cSlave", 512*4, priority)
        {
            /* Nothing to init */
        }
        bool init()
        {
          i2cslave.initSlave(slaveAddr,&buffer[0],sizeof(buffer));
        }

        bool run(void *p)
        {
            //init();
            if(buffer[1]!=0)
            {
                LD.setLeftDigit('H');
                LD.setRightDigit('I');
                LE.toggle(1);
                vTaskDelay(100);

            }
            return true;
        }
};



#endif /* L5_APPLICATION_HW_I2C_SLAVE_HPP_ */

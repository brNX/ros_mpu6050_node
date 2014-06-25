/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Bruno Gouveia on 10/28/2013
*********************************************************************/
#include <ros/ros.h>
#include "i2ckernel.h"
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>

using namespace cereal;

//TODO: use of linux i2c dev

void I2Ckernel::_open(const char* devicename){
  
  //TODO: check if changing speed is possible from driver
   if (!i2c_fd) {

     i2c_fd = open(devicename, O_RDWR);

     if (i2c_fd < 0) {
        ROS_FATAL("I2C_ROS - %s - Failed to Open I2C Bus %s", __FUNCTION__,devicename);
        i2c_fd = 0;
        ROS_BREAK();
      }
	  
      ROS_INFO("I2C_ROS - %s - Opened I2C Bus %s", __FUNCTION__,devicename);
    }

}

void I2Ckernel::_close(){
	if (i2c_fd) {
                close(i2c_fd);
                i2c_fd = 0;
                current_slave = 0;
        }
}

int I2Ckernel::_read(uint8_t address, uint8_t reg, uint8_t* bytes, int numBytes){
    
		int tries, result, total;

        if (!_write(address, reg, NULL,0)){
			ROS_INFO("I2C_ROS - %s - error writing register %x", __FUNCTION__,reg);
			return 0;
		}

        total = 0;
        tries = 0;

        while (total < numBytes && tries < 5) {
                result = read(i2c_fd, bytes + total, numBytes - total);

                if (result < 0) {
                        ROS_WARN("I2C_ROS - %s - Error read", __FUNCTION__);
                        break;
                }

                total += result;

                if (total == numBytes)
                        break;

                tries++;    
				usleep(10);
        }

        if (total < numBytes){
				ROS_INFO("I2C_ROS - %s - error total < numBytes", __FUNCTION__);
                return 0;
		}
	
    return 1;
}

int I2Ckernel::_write(uint8_t address, uint8_t reg, uint8_t* bytes, int numBytes){
    
  int result, i;

        if (numBytes > MAX_WRITE_LEN) {
				ROS_WARN("I2C_ROS - %s - Max write length exceeded", __FUNCTION__);
                return 0;
        }

        if (i2c_select_slave(address)){
			ROS_INFO("I2C_ROS - %s - i2c_select_slave(address)", __FUNCTION__);
			return 0;
		}

        if (numBytes == 0) {
                result = write(i2c_fd, &reg, 1);

                if (result < 0) {
                        ROS_WARN("I2C_ROS - %s - error write:1",__FUNCTION__);
                        return 0;
                }
                else if (result != 1) {
                        ROS_WARN("I2C_ROS - %s - Write fail:1 Tried 1 Wrote 0",__FUNCTION__);
                        return 0;
                }
        }
        else {
                txBuff[0] = reg;

                for (i = 0; i < numBytes; i++)
                        txBuff[i+1] = bytes[i];

                result = write(i2c_fd, txBuff, numBytes + 1);

                if (result < 0) {
                        ROS_WARN("I2C_ROS - %s - error Write:2",__FUNCTION__);
                        return 0;
                }
                else if (result < (int)numBytes) {
                        ROS_WARN("I2C_ROS - %s - Write fail:2 Tried %u Wrote %d",__FUNCTION__,numBytes, result); 
                        return 0;
                }
        }
  
    return 1;
}

int I2Ckernel::i2c_select_slave(unsigned char slave_addr)
{
        if (current_slave == slave_addr)
                return 0;

        if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
                ROS_WARN("I2C_ROS - %s - Failed to Select I2C Slave %x", __FUNCTION__,slave_addr);
                return 1;
        }

        current_slave = slave_addr;

        return 0;
}

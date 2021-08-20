__author__ = 'Geir Istad'
"""
MPU6050 Python I2C Class
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Code based on
I2Cdev library collection - MPU6050 I2C device class
by Jeff Rowberg <jeff@rowberg.net>
============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
"""

import time
import MPU6050

class MPU6050IRQHandler:
    __mpu = MPU6050
    __FIFO_buffer = list()
    __count = 0
    __packet_size = None
    __detected_error = False
    __logging = False
    __log_file = None
    __start_time = None
    __debug = None

    # def __init__(self, a_i2c_bus, a_device_address, a_x_accel_offset,
    #             a_y_accel_offset, a_z_accel_offset, a_x_gyro_offset,
    #             a_y_gyro_offset, a_z_gyro_offset, a_enable_debug_output):
    #    self.__mpu = MPU6050(a_i2c_bus, a_device_address, a_x_accel_offset,
    #                         a_y_accel_offset, a_z_accel_offset,
    #                         a_x_gyro_offset, a_y_gyro_offset, a_z_gyro_offset,
    #                         a_enable_debug_output)
    def __init__(self, a_mpu, a_logging=False, a_debug=False):
        self.__mpu = a_mpu
        self.__FIFO_buffer = [0] * 64
        self.__mpu.dmp_initialize()
        self.__mpu.set_DMP_enabled(True)
        self.__packet_size = self.__mpu.DMP_get_FIFO_packet_size()
        mpu_int_status = self.__mpu.get_int_status()
        if a_logging:
            self.__start_time = time.clock()
            self.__logging = True
            self.__log_file = open(a_log_file, 'ab')
        self.__debug = a_debug

    def action(self, channel):
        if self.__detected_error:
            # Clear FIFO and reset MPU
            mpu_int_status = self.__mpu.get_int_status()
            self.__mpu.reset_FIFO()
            self.__detected_error = False
            return

        try:
            FIFO_count = self.__mpu.get_FIFO_count()
            mpu_int_status = self.__mpu.get_int_status()
        except:
            self.__detected_error = True
            return

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            try:
                self.__mpu.reset_FIFO()
            except:
                self.__detected_error = True
                return

        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading,
            # default is 42 bytes
            while FIFO_count < self.__packet_size:
                try:
                    FIFO_count = self.__mpu.get_FIFO_count()
                except:
                    self.__detected_error = True
                    return

            while FIFO_count > self.__packet_size:

                try:
                    self.__FIFO_buffer = \
                        self.__mpu.get_FIFO_bytes(self.__packet_size)
                except:
                    self.__detected_error = True
                    return
                accel = \
                    self.__mpu.DMP_get_acceleration_int16(self.__FIFO_buffer)
                quat = self.__mpu.DMP_get_quaternion_int16(self.__FIFO_buffer)
                grav = self.__mpu.DMP_get_gravity(quat)
                roll_pitch_yaw = self.__mpu.DMP_get_euler_roll_pitch_yaw(quat,
                                                                         grav)
                if self.__logging:
                    delta_time = time.clock() - self.__start_time
                    data_concat = ['%.4f' % delta_time] + \
                                  [accel.x, accel.y, accel.z] + \
                                  ['%.3f' % roll_pitch_yaw.x,
                                   '%.3f' % roll_pitch_yaw.y,
                                   '%.3f' % roll_pitch_yaw.z]

                if (self.__debug) and (self.__count % 100 == 0):
                    print('roll: ' + str(roll_pitch_yaw.x))
                    print('pitch: ' + str(roll_pitch_yaw.y))
                    print('yaw: ' + str(roll_pitch_yaw.z))
                self.__count += 1
                FIFO_count -= self.__packet_size

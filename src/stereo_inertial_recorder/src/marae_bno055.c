/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bno055_support.c
* @date 10/01/2020
* @version  2.0.6
*
*/

/*---------------------------------------------------------------------------*
*  Includes
*---------------------------------------------------------------------------*/
#include "bno055.c"
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c.h>
#include <linux/i2c-dev.h>		//Needed for I2C port


#define BNO055_API 1
/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*   sensor data using I2C communication
*----------------------------------------------------------------------------*/
#ifdef  BNO055_API
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 my_bno055_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The API is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *  will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 my_bno055_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek);

#endif

/********************End of I2C APIs declarations***********************/

/* This API is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bno055_data_readout_template(void);

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *  BNO055_t having the following parameters
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Burst read function pointer: BNO055_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bno055_t bno055;

int file_i2c;

/* Do all initialization of BNO055, then start calibration loop
 *
 * Return value of 0 indicates Error
 * Return value of anything else indicates the I2C file active
 */
int init_and_calib_bno055(void)
{
    u8 power_mode_on = BNO055_POWER_MODE_NORMAL;
    s32 init_res = BNO055_ERROR;
    s32 power_on_res = BNO055_ERROR;
    s32 comres = ((u8)2);
    u8 d_sys_calib = ((u8)5);
    u8 d_mag_calib = ((u8)5);
    u8 d_gyro_calib = ((u8)5);
    u8 d_accel_calib = ((u8)5);
    u8 d_intr_rst = ((u8)3);
    u8 op_mode = ((u8)3);

    //----- OPEN THE I2C BUS -----
    char *filename = (char*)"/dev/i2c-1";
    if ((file_i2c = open(filename, O_RDWR)) < 0) {
        printf("Failed to open the i2c bus\n");
        return 0;
    }

    I2C_routine();
    init_res = bno055_init(&bno055);
    power_on_res = bno055_set_power_mode(power_mode_on);

    if (init_res == BNO055_SUCCESS && power_on_res == BNO055_SUCCESS) {
      printf("BNO055 Initialized\n");
    } else {
      printf("Failed setup of BNO055\n");
    }

    /*start test op mode */
    comres = bno055_get_operation_mode(&op_mode);
    if(comres != BNO055_SUCCESS) {
        printf("Failed to get op mode\n");
    } else {
        printf("Op mode is starting at %d\n", op_mode);
    }

    /* set to fusion fast magnetic calib off */
    comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF_FMC_OFF);
    if (comres != BNO055_SUCCESS) {
        printf("Failed to set BNO055 to NDOF_FMC_OFF mode\n");
    }

    comres = bno055_get_intr_rst(&d_intr_rst);
    if (comres != BNO055_SUCCESS) {
        printf("Failed to get intr rst\n");
    } else {
	printf("Intr rst val:%d\n", d_intr_rst);
    }

    //TODO change back to 3 after testing
    while (d_sys_calib != 2) {

	comres = bno055_get_mag_calib_stat(&d_mag_calib);
	if (comres != BNO055_SUCCESS) {
            printf("Failed to get mag calib\n");
    	}
	comres = bno055_get_accel_calib_stat(&d_accel_calib);
	if (comres != BNO055_SUCCESS) {
            printf("Failed to get accel calib\n");
        }
	comres = bno055_get_gyro_calib_stat(&d_gyro_calib);
	if (comres != BNO055_SUCCESS) {
            printf("Failed to get gyro calib\n");
        }
        comres = bno055_get_sys_calib_stat(&d_sys_calib);
	if (comres != BNO055_SUCCESS) {
            printf("Failed to get sys calib\n");
        }
	printf("BNO055 Calib is %d with mag:%d accel:%d gyro:%d\n", d_sys_calib, d_mag_calib, d_accel_calib, d_gyro_calib);
	sleep(1);
    }

    printf("Calibration Successful!\n");
    return file_i2c;
}

/* Obtain Formatted Data in BNO055 
 *
 * Probably going to specify some data return type AFTER verifying that C/C++ works as expected
 * covering errors, and all data outputs expected
 */
// at first, let's have 1 indicate failure and 0 indicate success
struct marae_data_t get_bno055_data(void) {
    struct marae_data_t data;
    u8 sys_calib;
    struct bno055_quaternion_t quaternion_wxyz;
    struct bno055_linear_accel_double_t d_linear_accel_xyz;
    struct bno055_gyro_double_t d_gyro_xyz;
    s32 comm;

    //check that device is still calibrated (TODO, figure out, do I need this constant check?)
    /*
    comm = bno055_get_sys_calib_stat(&sys_calib);
    if (comm != BNO055_SUCCESS) {
        printf("Failed to get sys calib\n");
	data.status = 1;
        return data;
    }
    if (sys_calib != 3) {
        printf("BNO055 is no longer fully calibrated! Now at %d/3\n", sys_calib);
	data.status = 2;
	return data;
    }*/

    //obtain all relevant motion data
    comm = bno055_read_quaternion_wxyz(&quaternion_wxyz);
    if (comm != BNO055_SUCCESS) {
        printf("Failed to get quaternion value\n");
	data.status = 3;
	return data;
    }
    comm = bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
    if (comm != BNO055_SUCCESS) {
        printf("Failed to get lin_accel value\n");
        data.status = 4;
	return data;
    }
    comm = bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);
    if (comm != BNO055_SUCCESS) {
        printf("Failed to get gyro value\n");
        data.status = 5;
	return data;
    }

    /* Everything succeeded! woop woop */
    data.status = 0;
    data.quaternion = quaternion_wxyz;
    data.lin_accel = d_linear_accel_xyz;
    data.ang_vel = d_gyro_xyz;
    return data;
}

/* Safely shut down BNO055 
 * 0 is success, 1 is failure */
int close_bno055(void) {
    u8 power_mode = BNO055_POWER_MODE_SUSPEND;
    /* set the power mode as SUSPEND*/
    return bno055_set_power_mode(power_mode);
}


/* This API is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bno055_data_readout_template(void)
{
    /* Variable used to return value of
     * communication routine*/
    s32 comres = BNO055_ERROR;

    /* variable used to set the power mode of the sensor*/
    u8 power_mode = BNO055_INIT_VALUE;

    /*********read raw accel data***********/
    /* variable used to read the accel x data */
    s16 accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the accel y data */
    s16 accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the accel z data */
    s16 accel_dataz = BNO055_INIT_VALUE;

    /* variable used to read the accel xyz data */
    struct bno055_accel_t accel_xyz;

    /*********read raw mag data***********/
    /* variable used to read the mag x data */
    s16 mag_datax = BNO055_INIT_VALUE;

    /* variable used to read the mag y data */
    s16 mag_datay = BNO055_INIT_VALUE;

    /* variable used to read the mag z data */
    s16 mag_dataz = BNO055_INIT_VALUE;

    /* structure used to read the mag xyz data */
    struct bno055_mag_t mag_xyz;

    /***********read raw gyro data***********/
    /* variable used to read the gyro x data */
    s16 gyro_datax = BNO055_INIT_VALUE;

    /* variable used to read the gyro y data */
    s16 gyro_datay = BNO055_INIT_VALUE;

    /* variable used to read the gyro z data */
    s16 gyro_dataz = BNO055_INIT_VALUE;

    /* structure used to read the gyro xyz data */
    struct bno055_gyro_t gyro_xyz;

    /*************read raw Euler data************/
    /* variable used to read the euler h data */
    s16 euler_data_h = BNO055_INIT_VALUE;

    /* variable used to read the euler r data */
    s16 euler_data_r = BNO055_INIT_VALUE;

    /* variable used to read the euler p data */
    s16 euler_data_p = BNO055_INIT_VALUE;

    /* structure used to read the euler hrp data */
    struct bno055_euler_t euler_hrp;

    /************read raw quaternion data**************/
    /* variable used to read the quaternion w data */
    s16 quaternion_data_w = BNO055_INIT_VALUE;

    /* variable used to read the quaternion x data */
    s16 quaternion_data_x = BNO055_INIT_VALUE;

    /* variable used to read the quaternion y data */
    s16 quaternion_data_y = BNO055_INIT_VALUE;

    /* variable used to read the quaternion z data */
    s16 quaternion_data_z = BNO055_INIT_VALUE;

    /* structure used to read the quaternion wxyz data */
    struct bno055_quaternion_t quaternion_wxyz;

    /************read raw linear acceleration data***********/
    /* variable used to read the linear accel x data */
    s16 linear_accel_data_x = BNO055_INIT_VALUE;

    /* variable used to read the linear accel y data */
    s16 linear_accel_data_y = BNO055_INIT_VALUE;

    /* variable used to read the linear accel z data */
    s16 linear_accel_data_z = BNO055_INIT_VALUE;

    /* structure used to read the linear accel xyz data */
    struct bno055_linear_accel_t linear_acce_xyz;

    /*****************read raw gravity sensor data****************/
    /* variable used to read the gravity x data */
    s16 gravity_data_x = BNO055_INIT_VALUE;

    /* variable used to read the gravity y data */
    s16 gravity_data_y = BNO055_INIT_VALUE;

    /* variable used to read the gravity z data */
    s16 gravity_data_z = BNO055_INIT_VALUE;

    /* structure used to read the gravity xyz data */
    struct bno055_gravity_t gravity_xyz;

    /*************read accel converted data***************/
    /* variable used to read the accel x data output as m/s2 or mg */
    double d_accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the accel y data output as m/s2 or mg */
    double d_accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the accel z data output as m/s2 or mg */
    double d_accel_dataz = BNO055_INIT_VALUE;

    /* structure used to read the accel xyz data output as m/s2 or mg */
    struct bno055_accel_double_t d_accel_xyz;

    /******************read mag converted data********************/
    /* variable used to read the mag x data output as uT*/
    double d_mag_datax = BNO055_INIT_VALUE;

    /* variable used to read the mag y data output as uT*/
    double d_mag_datay = BNO055_INIT_VALUE;

    /* variable used to read the mag z data output as uT*/
    double d_mag_dataz = BNO055_INIT_VALUE;

    /* structure used to read the mag xyz data output as uT*/
    struct bno055_mag_double_t d_mag_xyz;

    /*****************read gyro converted data************************/
    /* variable used to read the gyro x data output as dps or rps */
    double d_gyro_datax = BNO055_INIT_VALUE;

    /* variable used to read the gyro y data output as dps or rps */
    double d_gyro_datay = BNO055_INIT_VALUE;

    /* variable used to read the gyro z data output as dps or rps */
    double d_gyro_dataz = BNO055_INIT_VALUE;

    /* structure used to read the gyro xyz data output as dps or rps */
    struct bno055_gyro_double_t d_gyro_xyz;

    /*******************read euler converted data*******************/

    /* variable used to read the euler h data output
     * as degree or radians*/
    double d_euler_data_h = BNO055_INIT_VALUE;

    /* variable used to read the euler r data output
     * as degree or radians*/
    double d_euler_data_r = BNO055_INIT_VALUE;

    /* variable used to read the euler p data output
     * as degree or radians*/
    double d_euler_data_p = BNO055_INIT_VALUE;

    /* structure used to read the euler hrp data output
     * as as degree or radians */
    struct bno055_euler_double_t d_euler_hpr;

    /*********read linear acceleration converted data**********/
    /* variable used to read the linear accel x data output as m/s2*/
    double d_linear_accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the linear accel y data output as m/s2*/
    double d_linear_accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the linear accel z data output as m/s2*/
    double d_linear_accel_dataz = BNO055_INIT_VALUE;

    /* structure used to read the linear accel xyz data output as m/s2*/
    struct bno055_linear_accel_double_t d_linear_accel_xyz;

    /********************Gravity converted data**********************/
    /* variable used to read the gravity sensor x data output as m/s2*/
    double d_gravity_data_x = BNO055_INIT_VALUE;

    /* variable used to read the gravity sensor y data output as m/s2*/
    double d_gravity_data_y = BNO055_INIT_VALUE;

    /* variable used to read the gravity sensor z data output as m/s2*/
    double d_gravity_data_z = BNO055_INIT_VALUE;

    /* structure used to read the gravity xyz data output as m/s2*/
    struct bno055_gravity_double_t d_gravity_xyz;

    /*---------------------------------------------------------------------------*
     *********************** START INITIALIZATION ************************
     *--------------------------------------------------------------------------*/
#ifdef  BNO055_API

    /*  Based on the user need configure I2C interface.
     *  It is example code to explain how to use the bno055 API*/
    I2C_routine();
#endif

    /*--------------------------------------------------------------------------*
     *  This API used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
     *  Page id
     *  Accel revision id
     *  Mag revision id
     *  Gyro revision id
     *  Boot loader revision id
     *  Software revision id
     *-------------------------------------------------------------------------*/
    comres = bno055_init(&bno055);

    u8 op_mode = ((u8)3);
    u8 pow_mode = ((u8)10);
    comres = bno055_get_operation_mode(&op_mode);
    if (comres == BNO055_SUCCESS) {
        printf("Get First Op Mode worked and resulted is %x\n", op_mode);
    }

    comres = bno055_get_power_mode(&pow_mode);
    if (comres == BNO055_SUCCESS) {
	printf("Get First Pow Mode worked and result is %d\n", pow_mode);
    }
    /*  For initializing the BNO sensor it is required to the operation mode
     * of the sensor as NORMAL
     * Normal mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/


    power_mode = BNO055_POWER_MODE_NORMAL;

    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(power_mode);

    comres = bno055_get_power_mode(&pow_mode);
    if (comres == BNO055_SUCCESS) {
        printf("Get Second Pow Mode worked and result is %d\n", pow_mode);
    }

    comres = bno055_get_sys_stat_code(&pow_mode);
    if (comres == BNO055_SUCCESS) {
	printf("Sys status is %d\n", pow_mode);
    }

    comres = bno055_get_sys_error_code(&pow_mode);
    if (comres == BNO055_SUCCESS) {
        printf("Sys error is %d\n", pow_mode);
    }

    comres = bno055_get_operation_mode(&op_mode);
    if (comres == BNO055_SUCCESS) {
    	printf("Get Op Mode worked and resulted in %x\n", op_mode);
    }

    comres = bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
    if (comres != BNO055_SUCCESS) {
        printf("Set Power to Suspend failed\n");
    }

    /*----------------------------------------------------------------*
     ************************* END INITIALIZATION *************************
     *-----------------------------------------------------------------*/

    /************************* START READ RAW SENSOR DATA****************/

    /*  Using BNO055 sensor we can read the following sensor data and
     * virtual sensor data
     * Sensor data:
     * Accel
     * Mag
     * Gyro
     * Virtual sensor data
     * Euler
     * Quaternion
     * Linear acceleration
     * Gravity sensor */

    /*  For reading sensor raw data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * SENSOR MODE
     * 0x01 - BNO055_OPERATION_MODE_ACCONLY
     * 0x02 - BNO055_OPERATION_MODE_MAGONLY
     * 0x03 - BNO055_OPERATION_MODE_GYRONLY
     * 0x04 - BNO055_OPERATION_MODE_ACCMAG
     * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
     * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
     * 0x07 - BNO055_OPERATION_MODE_AMG
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

    /*  Raw accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x08 to 0x0D*/
    comres += bno055_read_accel_x(&accel_datax);
    comres += bno055_read_accel_y(&accel_datay);
    comres += bno055_read_accel_z(&accel_dataz);
    comres += bno055_read_accel_xyz(&accel_xyz);

    /*  Raw mag X, Y and Z data can read from the register
     * page - page 0
     * register - 0x0E to 0x13*/
    comres += bno055_read_mag_x(&mag_datax);
    comres += bno055_read_mag_y(&mag_datay);
    comres += bno055_read_mag_z(&mag_dataz);
    comres += bno055_read_mag_xyz(&mag_xyz);

    /*  Raw gyro X, Y and Z data can read from the register
     * page - page 0
     * register - 0x14 to 0x19*/
    comres += bno055_read_gyro_x(&gyro_datax);
    comres += bno055_read_gyro_y(&gyro_datay);
    comres += bno055_read_gyro_z(&gyro_dataz);
    comres += bno055_read_gyro_xyz(&gyro_xyz);

    /************************* END READ RAW SENSOR DATA****************/

    /************************* START READ RAW FUSION DATA ********
     * For reading fusion data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * FUSION MODE
     * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
     * 0x09 - BNO055_OPERATION_MODE_COMPASS
     * 0x0A - BNO055_OPERATION_MODE_M4G
     * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
     * 0x0C - BNO055_OPERATION_MODE_NDOF
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    /*  Raw Euler H, R and P data can read from the register
     * page - page 0
     * register - 0x1A to 0x1E */
    comres += bno055_read_euler_h(&euler_data_h);
    comres += bno055_read_euler_r(&euler_data_r);
    comres += bno055_read_euler_p(&euler_data_p);
    comres += bno055_read_euler_hrp(&euler_hrp);

    /*  Raw Quaternion W, X, Y and Z data can read from the register
     * page - page 0
     * register - 0x20 to 0x27 */
    comres += bno055_read_quaternion_w(&quaternion_data_w);
    comres += bno055_read_quaternion_x(&quaternion_data_x);
    comres += bno055_read_quaternion_y(&quaternion_data_y);
    comres += bno055_read_quaternion_z(&quaternion_data_z);
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);

    /*  Raw Linear accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x28 to 0x2D */
    comres += bno055_read_linear_accel_x(&linear_accel_data_x);
    comres += bno055_read_linear_accel_y(&linear_accel_data_y);
    comres += bno055_read_linear_accel_z(&linear_accel_data_z);
    comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);

    /*  Raw Gravity sensor X, Y and Z data can read from the register
     * page - page 0
     * register - 0x2E to 0x33 */
    comres += bno055_read_gravity_x(&gravity_data_x);
    comres += bno055_read_gravity_y(&gravity_data_y);
    comres += bno055_read_gravity_z(&gravity_data_z);
    comres += bno055_read_gravity_xyz(&gravity_xyz);
    
    /************************* END READ RAW FUSION DATA  ************/
    /******************START READ CONVERTED SENSOR DATA****************/

    /*  API used to read accel data output as double  - m/s2 and mg
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
    comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
    comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
    comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
    comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
    comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
    comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
    comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);

    /*  API used to read mag data output as double  - uT(micro Tesla)
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
    comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
    comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
    comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);

    /*  API used to read gyro data output as double  - dps and rps
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
    comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
    comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
    comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
    comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
    comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
    comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
    comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

    /*  API used to read Euler data output as double  - degree and radians
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
    comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
    comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
    comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
    comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
    comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
    comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
    comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);

    /*  API used to read Linear acceleration data output as m/s2
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_linear_accel_x_msq(&d_linear_accel_datax);
    comres += bno055_convert_double_linear_accel_y_msq(&d_linear_accel_datay);
    comres += bno055_convert_double_linear_accel_z_msq(&d_linear_accel_dataz);
    comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);

    /*  API used to read Gravity sensor data output as m/s2
     * float functions also available in the BNO055 API */
    comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
    comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
    comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
    comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);

    /*-----------------------------------------------------------------------*
     ************************* START DE-INITIALIZATION ***********************
     *-------------------------------------------------------------------------*/

    /*  For de - initializing the BNO sensor it is required
     * to the operation mode of the sensor as SUSPEND
     * Suspend mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/
    power_mode = BNO055_POWER_MODE_SUSPEND;

    /* set the power mode as SUSPEND*/
    comres += bno055_set_power_mode(power_mode);

    /*---------------------------------------------------------------------*
    ************************* END DE-INITIALIZATION **********************
    *---------------------------------------------------------------------*/
    return comres;
}

#ifdef  BNO055_API

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
    bno055.bus_write = my_bno055_bus_write;//BNO055_I2C_bus_write;
    bno055.bus_read = my_bno055_bus_read;//BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    return BNO055_INIT_VALUE;
}

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 8
#define I2C0           5

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        //array[stringpos] = *(reg_data + stringpos);
	array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    }

    return (s8)BNO055_iERROR;
}

/*
 * Please take the below APIs as your reference for
 * write the data using I2C communication
 * "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
 * add your I2C write APIs here
 * BNO055_iERROR is an return value of I2C read API
 * Please select your valid return value
 * In the driver BNO055_SUCCESS defined as 0
 * and FAILURE defined as -1
 * Note :
 * This is a full duplex operation,
 * The first read data is discarded, for that extra write operation
 * have to be initiated. For that cnt+1 operation done
 * in the I2C write string function
 * For more information please refer data sheet SPI communication:
 */
//return (s8)BNO055_iERROR;
//}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;

    /* Please take the below API as your reference
     * for read the data using I2C communication
     * add your I2C read API here.
     * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
     * ARRAY, ARRAY, 1, CNT)"
     * BNO055_iERROR is an return value of SPI write API
     * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
     */
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = array[stringpos];
    }

    return (s8)BNO055_iERROR;
}

/* personal implementation of i2c write of bno055 data */
s8 my_bno055_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    unsigned char outbuf[cnt+1];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];
    int i;

    messages[0].addr = dev_addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(outbuf);
    messages[0].buf = outbuf;

    outbuf[0] = reg_addr;
    for (i = 1; i <= cnt; i++) {
        outbuf[i] = *(reg_data + (i-1));
    }

    packets.msgs = messages;
    packets.nmsgs = 1;
    if(ioctl(file_i2c, I2C_RDWR, &packets) < 0) {
	return 1;
    }

    /* succeeds */
    return 0;
}


/* personal implementation of i2c read of bno055 data */
s8 my_bno055_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    unsigned char outbuf;
    unsigned char inbuf[cnt];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    int i;

    outbuf = reg_addr;
    messages[0].addr = dev_addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(outbuf);
    messages[0].buf = &outbuf;

    messages[1].addr = dev_addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = sizeof(inbuf);
    messages[1].buf = inbuf;

    packets.msgs = messages;
    packets.nmsgs = 2;
    if (ioctl(file_i2c, I2C_RDWR, &packets) < 0) {
        return 1;
    }

    for (i = 0; i < cnt; i++) {
        *(reg_data + i) = inbuf[i];
    }
    return 0;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
    /*Here you can write your own delay routine*/
}

#endif
/*
int main(void)
{
  printf("Starting...\n");
  int i;

  s32 func_val;
  func_val = init_and_calib_bno055();
  printf("Init and Calib returned %d\n", func_val);

  for (i=0; i< 200; i++) {
      func_val = get_bno055_data();
      printf("BNO055 func call returned %d\n", func_val);
  }

  func_val = close_bno055();
  printf("BNO055 close func returned %d\n", func_val);
  
  return 0;
}
*/

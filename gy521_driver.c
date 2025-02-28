#include <linux/module.h>   // macros module_init() and module_exit()
#include <linux/init.h>     // macros __init and __exit
#include <linux/kernel.h>   // pr_*
#include <linux/fs.h>       // file_operations
#include <linux/device.h>   // class_create(), device_create()
#include <linux/i2c.h>      // i2c_*
#include "gy521_registermap.h"

#define CLASS_NAME "gy521_class"
#define DEVICE_NAME "gy521_device"

#define MPU6050_ADDR_1 0x68 // AD0 connected on GND
#define MPU6050_ADDR_2 0x69 // AD0 connected on Vcc


static int major_number;                    // Register number in /dev
static struct class* gy521_class = NULL;    // User Space
static struct device* gy521_device = NULL;  // /dev

struct i2c_adapter *stored_adapter = NULL;         // Access to I2C bus (connection with the hardware)
static int gy521_addr = 0;                         // To store the real I2C addr
static struct i2c_board_info gy521_info = {               // Device info
    .type = "gy521", 
};
static struct i2c_client *gy521_client = NULL;     // Represent gy521 device on the I2C

// Device operations

static int gy521_write_byte(struct i2c_client *client, u8 reg, u8 value)
{
    // Perform the I2C write operation to the specified register.
    int ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0) {
        pr_err("GY-521: Failed to write register 0x%02x\n", reg);
    }

    return ret;
}

static int gy521_read_byte(struct i2c_client *client, u8 reg)
{
    // Read a byte from the specified register.
    int ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
        pr_err("GY-521: Failed to read register 0x%02x\n", reg);
    }

    return ret;
}

static int gy521_read_word(struct i2c_client *client, u8 reg)
{
    s32 ret;
    u8 data[2];

    // Read two consecutive bytes of data from the specified register.
    ret = i2c_smbus_read_i2c_block_data(client, reg, 2, data);
    if (ret < 0) {
        pr_err("GY-521: Failed to read word from register 0x%02x\n", reg);
        return ret;
    }

    return (data[0] << 8) | data[1];
}

static int gy521_read_self_test(struct i2c_client *client){  
    int selftest_x = gy521_read_byte(client, GY521_SELF_TEST_X);
    int selftest_y = gy521_read_byte(client, GY521_SELF_TEST_Y);
    int selftest_z = gy521_read_byte(client, GY521_SELF_TEST_Z);
    int selftest_a = gy521_read_byte(client, GY521_SELF_TEST_A);

    return (selftest_x << 24) | (selftest_y << 16) | (selftest_z << 8) | selftest_a;
}

static int gy521_sample_rate_divider(struct i2c_client *client, u8 smplrt_div){  // Write
    return gy521_write_byte(gy521_client, GY521_SMPLRT_DIV, smplrt_div); // smplrt_div -> 0x00-0xFF
}

static int gy521_write_config(struct i2c_client *client, u8 ext_sync_set, u8 dlpf_cfg){

    u8 config;

    config = ((ext_sync_set << 3) & 0x38) | (dlpf_cfg & 0x07);

    return gy521_write_byte(client, GY521_CONFIG, config);
}

static int gy521_read_config(struct i2c_client *client){

    return gy521_read_byte(client, GY521_CONFIG);
}

static int gy521_write_gyro_config(struct i2c_client *client, u8 xg_st, u8 yg_st, u8 zg_st, u8 fs_sel){  
    u8 reg_data = 0;

    reg_data |= (xg_st << 7);
    reg_data |= (yg_st << 6);
    reg_data |= (zg_st << 5);
    reg_data |= (fs_sel & 0x03) << 3;

    return gy521_write_byte(client, GY521_GYRO_CONFIG, reg_data);
}

static int gy521_read_gyro_config(struct i2c_client *client){

    return gy521_read_byte(client, GY521_GYRO_CONFIG);
}

static int gy521_write_accel_config(struct i2c_client *client, u8 xa_st, u8 ya_st, u8 za_st, u8 afs_sel){  
    u8 reg_data = 0;

    reg_data |= (xa_st << 7);
    reg_data |= (ya_st << 6);
    reg_data |= (za_st << 5);
    reg_data |= (afs_sel & 0x03) << 3;

    return gy521_write_byte(client, GY521_ACCEL_CONFIG, reg_data);
}

static int gy521_read_accel_config(struct i2c_client *client){

    return gy521_read_byte(client, GY521_ACCEL_CONFIG);
}

static int gy521_write_fifo_en(struct i2c_client *client, u8 fifo_en){  

    return gy521_write_byte(client, GY521_FIFO_EN, fifo_en);
}

static int gy521_read_fifo_en(struct i2c_client *client){ 

    return gy521_read_byte(client, GY521_FIFO_EN);
}

static int gy521_write_i2c_mst_ctrl(struct i2c_client *client, u8 mult_mst_en, u8 wait_for_es, u8 slv_3_fifo_en, u8 i2c_mst_p_nsr, u8 i2c_mst_clk){  
    u8 reg_data;

    reg_data |= (mult_mst_en << 7);
    reg_data |= (wait_for_es << 6);
    reg_data |= (slv_3_fifo_en << 5);
    reg_data |= (i2c_mst_p_nsr << 4);
    reg_data |= (i2c_mst_clk & 0x0F);

    return gy521_write_byte(client, GY521_I2C_MST_CTRL, reg_data);
}

static int gy521_read_i2c_mst_ctrl(struct i2c_client *client){ 

    return gy521_read_byte(client, GY521_I2C_MST_CTRL);
}

static int gy521_write_i2c_slv0_addr(struct i2c_client *client, u8 i2c_slv0_rw, u8 i2c_slv0_addr){
    u8 reg_data;

    reg_data |= (i2c_slv0_rw << 7);
    reg_data |= (i2c_slv0_addr & 0x7F);

    return gy521_write_byte(client, GY521_I2C_SLV0_ADDR, reg_data);
}

static int gy521_read_i2c_slv0_addr(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV0_ADDR);
}

static int gy521_write_i2c_slv0_reg(struct i2c_client *client, u8 i2c_slv0_reg){
    return gy521_write_byte(client, GY521_I2C_SLV0_REG, i2c_slv0_reg);
}

static int gy521_read_i2c_slv0_reg(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV0_REG);
}

static int gy521_write_i2c_slv0_ctrl(struct i2c_client *client, u8 i2c_slv0_en, u8 i2c_slv0_byte_sw, u8 i2c_slv0_reg_dis, u8 i2c_slv0_grp, u8 i2c_slv0_len){
    u8 reg_data;
   
    reg_data |= (i2c_slv0_en << 7);       
    reg_data |= (i2c_slv0_byte_sw << 6);  
    reg_data |= (i2c_slv0_reg_dis << 5); 
    reg_data |= (i2c_slv0_grp << 4);      
    reg_data |= (i2c_slv0_len & 0x0F);  

    return gy521_write_byte(client, GY521_I2C_SLV0_CTRL, reg_data);
}

static int gy521_read_i2c_slv0_ctrl(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV0_CTRL);
}

static int gy521_write_i2c_slv1_addr(struct i2c_client *client, u8 i2c_slv1_rw, u8 i2c_slv1_addr){
    u8 reg_data;

    reg_data |= (i2c_slv1_rw << 7);
    reg_data |= (i2c_slv1_addr & 0x7F);

    return gy521_write_byte(client, GY521_I2C_SLV1_ADDR, reg_data);
}

static int gy521_read_i2c_slv1_addr(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV1_ADDR);
}

static int gy521_write_i2c_slv1_reg(struct i2c_client *client, u8 i2c_slv1_reg){
    return gy521_write_byte(client, GY521_I2C_SLV1_REG, i2c_slv1_reg);
}

static int gy521_read_i2c_slv1_reg(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV1_REG);
}

static int gy521_write_i2c_slv1_ctrl(struct i2c_client *client, u8 i2c_slv1_en, u8 i2c_slv1_byte_sw, u8 i2c_slv1_reg_dis, u8 i2c_slv1_grp, u8 i2c_slv1_len){
    u8 reg_data;
   
    reg_data |= (i2c_slv1_en << 7);       
    reg_data |= (i2c_slv1_byte_sw << 6);  
    reg_data |= (i2c_slv1_reg_dis << 5); 
    reg_data |= (i2c_slv1_grp << 4);      
    reg_data |= (i2c_slv1_len & 0x0F);  

    return gy521_write_byte(client, GY521_I2C_SLV1_CTRL, reg_data);
}

static int gy521_read_i2c_slv1_ctrl(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV1_CTRL);
}

static int gy521_write_i2c_slv2_addr(struct i2c_client *client, u8 i2c_slv2_rw, u8 i2c_slv2_addr){
    u8 reg_data;

    reg_data |= (i2c_slv2_rw << 7);
    reg_data |= (i2c_slv2_addr & 0x7F);

    return gy521_write_byte(client, GY521_I2C_SLV2_ADDR, reg_data);
}

static int gy521_read_i2c_slv2_addr(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV2_ADDR);
}

static int gy521_write_i2c_slv2_reg(struct i2c_client *client, u8 i2c_slv2_reg){
    return gy521_write_byte(client, GY521_I2C_SLV2_REG, i2c_slv2_reg);
}

static int gy521_read_i2c_slv2_reg(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV2_REG);
}

static int gy521_write_i2c_slv2_ctrl(struct i2c_client *client, u8 i2c_slv2_en, u8 i2c_slv2_byte_sw, u8 i2c_slv2_reg_dis, u8 i2c_slv2_grp, u8 i2c_slv2_len){
    u8 reg_data;
   
    reg_data |= (i2c_slv2_en << 7);       
    reg_data |= (i2c_slv2_byte_sw << 6);  
    reg_data |= (i2c_slv2_reg_dis << 5); 
    reg_data |= (i2c_slv2_grp << 4);      
    reg_data |= (i2c_slv2_len & 0x0F);  

    return gy521_write_byte(client, GY521_I2C_SLV2_CTRL, reg_data);
}

static int gy521_read_i2c_slv2_ctrl(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV2_CTRL);
}

static int gy521_write_i2c_slv3_addr(struct i2c_client *client, u8 i2c_slv3_rw, u8 i2c_slv3_addr){
    u8 reg_data;

    reg_data |= (i2c_slv3_rw << 7);
    reg_data |= (i2c_slv3_addr & 0x7F);

    return gy521_write_byte(client, GY521_I2C_SLV3_ADDR, reg_data);
}

static int gy521_read_i2c_slv3_addr(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV3_ADDR);
}

static int gy521_write_i2c_slv3_reg(struct i2c_client *client, u8 i2c_slv3_reg){
    return gy521_write_byte(client, GY521_I2C_SLV3_REG, i2c_slv3_reg);
}

static int gy521_read_i2c_slv3_reg(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV3_REG);
}

static int gy521_write_i2c_slv3_ctrl(struct i2c_client *client, u8 i2c_slv3_en, u8 i2c_slv3_byte_sw, u8 i2c_slv3_reg_dis, u8 i2c_slv3_grp, u8 i2c_slv3_len){
    u8 reg_data;
   
    reg_data |= (i2c_slv3_en << 7);       
    reg_data |= (i2c_slv3_byte_sw << 6);  
    reg_data |= (i2c_slv3_reg_dis << 5); 
    reg_data |= (i2c_slv3_grp << 4);      
    reg_data |= (i2c_slv3_len & 0x0F);  

    return gy521_write_byte(client, GY521_I2C_SLV3_CTRL, reg_data);
}

static int gy521_read_i2c_slv3_ctrl(struct i2c_client *client){

    return gy521_read_byte(client, GY521_I2C_SLV3_CTRL);
}

static int gy521_write_i2c_slv4_addr(struct i2c_client *client, u8 i2c_slv4_rw, u8 i2c_slv4_addr){
    u8 reg_data;

    reg_data |= (i2c_slv4_rw << 7);
    reg_data |= (i2c_slv4_addr & 0x7F);

    return gy521_write_byte(client, GY521_I2C_SLV4_ADDR, reg_data);
}

static int gy521_read_i2c_slv4_addr(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV4_ADDR);
}

static int gy521_write_i2c_slv4_reg(struct i2c_client *client, u8 i2c_slv4_reg){
    return gy521_write_byte(client, GY521_I2C_SLV4_REG, i2c_slv4_reg);
}

static int gy521_read_i2c_slv4_reg(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV4_REG);
}

static int gy521_write_i2c_slv4_do(struct i2c_client *client, u8 i2c_slv4_do){
    return gy521_write_byte(client, GY521_I2C_SLV4_DO, i2c_slv4_do);
}

static int gy521_read_i2c_slv4_do(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV4_DO);
}

static int gy521_write_i2c_slv4_ctrl(struct i2c_client *client, u8 i2c_slv4_en, u8 i2c_slv4_byte_sw, u8 i2c_slv4_reg_dis, u8 i2c_slv4_grp, u8 i2c_slv4_len){
    u8 reg_data;
   
    reg_data |= (i2c_slv4_en << 7);       
    reg_data |= (i2c_slv4_byte_sw << 6);  
    reg_data |= (i2c_slv4_reg_dis << 5); 
    reg_data |= (i2c_slv4_grp << 4);      
    reg_data |= (i2c_slv4_len & 0x0F);  

    return gy521_write_byte(client, GY521_I2C_SLV4_CTRL, reg_data);
}

static int gy521_read_i2c_slv4_ctrl(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV4_CTRL);
}

static int gy521_write_i2c_slv4_di(struct i2c_client *client, u8 i2c_slv4_di){
    return gy521_write_byte(client, GY521_I2C_SLV4_DI, i2c_slv4_di);
}

static int gy521_read_i2c_slv4_di(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV4_DI);
}

static int gy521_read_i2c_mst_status(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_MST_STATUS);
}

static int gy521_write_i2c_int_pin_cfg(struct i2c_client *client, u8 int_level, u8 int_open, u8 latch_int_en, u8 int_rd_clear, u8 fsync_int_level, u8 fsync_int_en, u8 i2c_bypass_en){
    u8 reg_data;
   
    reg_data |= (int_level << 7);       
    reg_data |= (int_open << 6);  
    reg_data |= (latch_int_en << 5); 
    reg_data |= (int_rd_clear << 4);
    reg_data |= (fsync_int_level << 3);     
    reg_data |= (fsync_int_en << 3);
    reg_data |= (i2c_bypass_en << 1);
    
    return gy521_write_byte(client, GY521_INT_PIN_CFG, reg_data);
}

static int gy521_read_int_pin_cfg(struct i2c_client *client){
    return gy521_read_byte(client, GY521_INT_PIN_CFG);
}

static int gy521_write_i2c_int_enable(struct i2c_client *client, u8 fifo_oflow_en, u8 i2c_mst_en, u8 data_rdy_en){
    u8 reg_data;
   
    reg_data |= (fifo_oflow_en << 4);       
    reg_data |= (i2c_mst_en << 3);  
    reg_data |= (data_rdy_en << 0); 
    
    return gy521_write_byte(client, GY521_INT_ENABLE, reg_data);
}

static int gy521_read_int_enable(struct i2c_client *client){
    return gy521_read_byte(client, GY521_INT_ENABLE);
}

static int gy521_read_int_status(struct i2c_client *client){
    return gy521_read_byte(client, GY521_INT_STATUS);
}

static int gy521_read_accel_xout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_ACCEL_XOUT_H);
}

static int gy521_read_accel_yout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_ACCEL_YOUT_H);
}

static int gy521_read_accel_zout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_ACCEL_ZOUT_H);
}

static int gy521_read_temp_out(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_TEMP_OUT_H);
}

static int gy521_read_gyro_xout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_GYRO_XOUT_H);
}

static int gy521_read_gyro_yout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_GYRO_YOUT_H);
}

static int gy521_read_gyro_zout(struct i2c_client *client){
    return gy521_read_word(gy521_client, GY521_GYRO_ZOUT_H);
}

static int gy521_read_ext_sens_data_xx(struct i2c_client *client, u8 reg_offset){
    if (reg_offset > 23) {
        pr_err("GY-521: Invalid external sensor data register offset: %d\n", reg_offset);
        return -EINVAL;
    }
    return gy521_read_byte(client, GY521_EXT_SENS_DATA_00 + reg_offset);
}

static int gy521_write_i2c_slv0_do(struct i2c_client *client, u8 i2c_slv0_do){
    return gy521_write_byte(client, GY521_I2C_SLV0_DO, i2c_slv0_do);
}

static int gy521_read_i2c_slv0_do(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV0_DO);
}

static int gy521_write_i2c_slv1_do(struct i2c_client *client, u8 i2c_slv1_do){
    return gy521_write_byte(client, GY521_I2C_SLV1_DO, i2c_slv1_do);
}

static int gy521_read_i2c_slv1_do(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV1_DO);
}

static int gy521_write_i2c_slv2_do(struct i2c_client *client, u8 i2c_slv2_do){
    return gy521_write_byte(client, GY521_I2C_SLV2_DO, i2c_slv2_do);
}

static int gy521_read_i2c_slv2_do(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV2_DO);
}

static int gy521_write_i2c_slv3_do(struct i2c_client *client, u8 i2c_slv3_do){
    return gy521_write_byte(client, GY521_I2C_SLV3_DO, i2c_slv3_do);
}

static int gy521_read_i2c_slv3_do(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_SLV3_DO);
}

static int gy521_write_i2c_mst_delay_ctrl(struct i2c_client *client, u8 delay_es_shadow, u8 i2c_slv4_dly_en, u8 i2c_slv3_dly_en, u8 i2c_slv2_dly_en, u8 i2c_slv1_dly_en, u8 i2c_slv0_dly_en){
    u8 reg_data;
   
    reg_data |= (delay_es_shadow << 7);       
    reg_data |= (i2c_slv4_dly_en << 4);  
    reg_data |= (i2c_slv3_dly_en << 3); 
    reg_data |= (i2c_slv2_dly_en << 2); 
    reg_data |= (i2c_slv1_dly_en << 1); 
    reg_data |= (i2c_slv0_dly_en << 0); 
    
    return gy521_write_byte(client, GY521_I2C_MST_DELAY_CTRL, reg_data);
}

static int gy521_read_i2c_mst_delay_ctrl(struct i2c_client *client){
    return gy521_read_byte(client, GY521_I2C_MST_DELAY_CTRL);
}

static int gy521_write_signal_path_reset(struct i2c_client *client, u8 gyro_reset, u8 accel_reset, u8 temp_reset){
    u8 reg_data;
   
    reg_data |= (gyro_reset << 2); 
    reg_data |= (accel_reset << 1); 
    reg_data |= (temp_reset << 0); 
    
    return gy521_write_byte(client, GY521_SIGNAL_PATH_RESET, reg_data);
}

static int gy521_write_user_ctrl(struct i2c_client *client, u8 fifo_en, u8 i2c_mst_en, u8 i2c_if_dis, u8 fifo_reset, u8 i2c_mst_reset, u8 sig_cond_reset){
    u8 reg_data;
   
    reg_data |= (fifo_en << 6);       
    reg_data |= (i2c_mst_en << 5);  
    reg_data |= (i2c_if_dis << 4); 
    reg_data |= (fifo_reset << 2); 
    reg_data |= (i2c_mst_reset << 1); 
    reg_data |= (sig_cond_reset << 0); 
    
    return gy521_write_byte(client, GY521_USER_CTRL, reg_data);
}

static int gy521_read_user_ctrl(struct i2c_client *client){
    return gy521_read_byte(client, GY521_USER_CTRL);
}

static int gy521_write_pwr_mgmt_1(struct i2c_client *client, u8 device_reset, u8 sleep, u8 cycle, u8 temp_dis, u8 clksel){
    u8 reg_data;
   
    reg_data |= (device_reset << 7);       
    reg_data |= (sleep << 6);  
    reg_data |= (cycle << 5); 
    reg_data |= (temp_dis << 3); 
    reg_data |= (clksel & 0x07); 
    
    return gy521_write_byte(client, GY521_PWR_MGMT_1, reg_data);
}

static int gy521_read_pwr_mgmt_1(struct i2c_client *client){
    return gy521_read_byte(client, GY521_PWR_MGMT_1);
}

static int gy521_write_pwr_mgmt_2(struct i2c_client *client, u8 lp_wake_ctrl, u8 stby_xa, u8 stby_ya, u8 stby_za, u8 stby_xg, u8 stby_yg, u8 stby_zg){
    u8 reg_data;
   
    reg_data |= ((lp_wake_ctrl & 0x03) << 6);        
    reg_data |= (stby_xa << 5);  
    reg_data |= (stby_ya << 4); 
    reg_data |= (stby_za << 3); 
    reg_data |= (stby_xg << 2); 
    reg_data |= (stby_yg << 1); 
    reg_data |= (stby_zg << 0); 
    
    return gy521_write_byte(client, GY521_PWR_MGMT_2, reg_data);
}

static int gy521_read_pwr_mgmt_2(struct i2c_client *client){
    return gy521_read_byte(client, GY521_PWR_MGMT_2);
}

static int gy521_read_fifo_count(struct i2c_client *client){
    return gy521_read_word(client, GY521_FIFO_COUNTH);
}

static int gy521_write_fifo_r_w(struct i2c_client *client, u8 fifo_data){
    return gy521_write_byte(client, GY521_FIFO_R_W, fifo_data);
}

static int gy521_read_fifo_r_w(struct i2c_client *client){
    return gy521_read_byte(client, GY521_FIFO_R_W);
}

static int gy521_read_who_am_i(struct i2c_client *client){
    return gy521_read_byte(client, GY521_WHO_AM_I);
}

// Checks
static int check_addr(struct i2c_adapter *adapter, int addr)
{
        struct i2c_client *client;
        int ret;

        // Create a "dummy" I2C client at the specified address.
        client = i2c_new_dummy_device(adapter, addr);
        if(!client)
            return -ENODEV;
        
        // Attempt to read the "WHO_AM_I" register from the GY-521 device.
        ret = gy521_read_who_am_i(client);

        // Unregister the "dummy" I2C client after usage.
        i2c_unregister_device(client);

        return ret;
}

static int check_dev(struct device *dev,void *data)
{
    // Check if the device is valid and I2C type
    if (!dev || dev->type != &i2c_adapter_type) {
        pr_err("GY-521: Invalid or non-I2C device");
        return -ENODEV;
    }
    pr_info("GY-521: device found: %s", dev_name(dev));

    // Convert dev to an I2C adapter
    struct i2c_adapter *adapter = to_i2c_adapter(dev);
    if (!adapter) {
        pr_err("GY-521: adapter is NULL");
        return -ENODEV;
    }
    pr_info("GY-521: adapter found");
    
    // Check the first I2C address
    if (check_addr(adapter, MPU6050_ADDR_1) == 0) {
        pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_1);
        gy521_addr = MPU6050_ADDR_1;
        stored_adapter = adapter;
        return 0;
    }
   
    // Check the second I2C address
    if (check_addr(adapter, MPU6050_ADDR_2) == 0) {
        pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_2);
        gy521_addr = MPU6050_ADDR_2;
        stored_adapter = adapter;
        return 0;
    }

    pr_err("GY-521: No device found at the expected I2C addresses");
    return -ENODEV;
}

// Handle the device from user-space
static int gy521_open(struct inode *inode, struct file *file){
    pr_info("GY-521: device is opened");

    int ret = gy521_write_byte(gy521_client, GY521_PWR_MGMT_1, 0x00);
    if (ret < 0){
        pr_err("GY-521: Failed to activate device");
        return ret;
    }

    return 0;
}

static ssize_t gy521_read(struct file *file, char *buffer, size_t len, loff_t *offset)
{
    int accel_x, accel_y, accel_z;
    int gyro_x, gyro_y, gyro_z;
    char data[128];

    // Read data acelerometer (2 bytes each axis)
    accel_x = gy521_read_word(gy521_client, GY521_ACCEL_XOUT_H);
    accel_y = gy521_read_word(gy521_client, GY521_ACCEL_YOUT_H);
    accel_z = gy521_read_word(gy521_client, GY521_ACCEL_ZOUT_H);

    // Read data gyroscope (2 bytes each axis)
    gyro_x = gy521_read_word(gy521_client, GY521_GYRO_XOUT_H );
    gyro_y = gy521_read_word(gy521_client, GY521_GYRO_YOUT_H );
    gyro_z = gy521_read_word(gy521_client, GY521_GYRO_ZOUT_H );

    // Format the data to send to user space
    snprintf(data, sizeof(data), 
            "Accel X: %d, Y: %d, Z: %d\nGyro X: %d, Y: %d, Z: %d\n",
             accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

    // Copy the data to the user space buffer
    if (copy_to_user(buffer, data, strlen(data))) {
        pr_err("Failed to send data to user\n");
        return -EFAULT;
    }

    return strlen(data);
}


static int gy521_release(struct inode *inode, struct file *file){
    pr_info("GY-521: device is closed");

    return 0;
}

static struct file_operations fops = { // Operations to be used in user space
    .open = gy521_open,
    .read = gy521_read,
    .release = gy521_release,
};

// Init
static int __init gy521_init(void){

    pr_info("GY-521: driver is loaded");

    major_number = register_chrdev(0, DEVICE_NAME, &fops); // To register the device with the system
    if (major_number < 0){
        pr_err("GY-521: register device failed");
        return major_number;
    }

    gy521_class = class_create(CLASS_NAME);
    if (IS_ERR(gy521_class)){
        pr_err("GY-521: create device class failed");
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(gy521_class);
    }
 
    gy521_device = device_create(gy521_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(gy521_device)){
        pr_err("GY-521: create device failed");
        class_destroy(gy521_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(gy521_device);
    }
    
    i2c_for_each_dev(NULL, check_dev);
    
    gy521_info.addr = gy521_addr;
    
    if (gy521_addr == 0){
        pr_err("GY-521: device not found on the I2C bus");
        return -ENODEV;
    }
    
    gy521_client = i2c_new_client_device(stored_adapter, &gy521_info);
    if (!gy521_client){
        pr_err("GY-521: create I2C client failed");
        return -ENODEV;
    }
    
    return 0;
}

// Exit
static void __exit gy521_exit(void){
    pr_info("GY-521 I2C driver is unloaded");

    if (gy521_client){
        i2c_unregister_device(gy521_client);
    }
    
    device_destroy(gy521_class, MKDEV(major_number, 0));
    class_destroy(gy521_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    
}

module_init(gy521_init);
module_exit(gy521_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Gómez López");
MODULE_DESCRIPTION("GY-521 I2C Driver");
MODULE_VERSION("0.1");

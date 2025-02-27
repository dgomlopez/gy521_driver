#include <linux/module.h>   // macros module_init() and module_exit()
#include <linux/init.h>     // macros __init and __exit
#include <linux/kernel.h>   // pr_*
#include <linux/fs.h>       // file_operations
#include <linux/device.h>   // class_create(), device_create()
#include <linux/i2c.h>      // i2c_*

#define CLASS_NAME "gy521_class"
#define DEVICE_NAME "gy521_device"

#define MPU6050_ADDR_1 0x68 // AD0 connected on GND
#define MPU6050_ADDR_2 0x69 // AD0 connected on Vcc
#define WHO_AM_I_REG 0x75   // To verify the correct device address
#define PWR_MGMT_1 0x6B

#define ACCEL_X_REG 0x3B
#define ACCEL_Y_REG 0x3D
#define ACCEL_Z_REG 0x3F
#define GYRO_X_REG 0x43
#define GYRO_Y_REG 0x45
#define GYRO_Z_REG 0x47


static int major_number;                    // Register number in /dev
static struct class* gy521_class = NULL;    // User Space
static struct device* gy521_device = NULL;  // /dev

struct i2c_adapter *stored_adapter = NULL;         // Access to I2C bus (connection with the hardware)
static int gy521_addr = 0;                         // To store the real I2C addr
static struct i2c_board_info gy521_info = {               // Device info
    .type = "gy521", 
};
static struct i2c_client *gy521_client = NULL;     // Represent gy521 device on the I2C
static int check_addr(struct i2c_adapter *adapter, int addr){
        struct i2c_client *client;
        int ret;

        client = i2c_new_dummy_device(adapter, addr);
        if(!client)
            return -ENODEV;
        
        ret = i2c_smbus_read_byte_data(client, WHO_AM_I_REG);
        i2c_unregister_device(client);

        if (ret < 0)
            return -ENODEV;
        
        return 0;
}

static int check_device(struct device *dev, void *data)
{
    if (!dev){
        pr_err("GY-521: I2C device is NULL");
    }
    
    pr_info("GY-521: I2C device found: %s", dev_name(dev));

    if (dev->type != &i2c_adapter_type)
        return 0;

    struct i2c_adapter *adapter = to_i2c_adapter(dev);

    if (!adapter) {
        pr_err("GY-521: adapter is NULL");
        return 0;
    }

    pr_info("GY-521: adapter found");
    
    if (check_addr(adapter, MPU6050_ADDR_1) == 0) {
        pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_1);
        gy521_addr = MPU6050_ADDR_1;
        stored_adapter = adapter;
        return 1;
    }
   
    if (check_addr(adapter, MPU6050_ADDR_2) == 0) {
        pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_2);
        gy521_addr = MPU6050_ADDR_2;
        stored_adapter = adapter;
        return 1;
    }
  
    return 0;
}

// Device operations
static int gy521_write_byte(struct i2c_client *client, u8 reg, u8 value){
    int ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0) {
        pr_err("GY-521: Failed to write register 0x%02x\n", reg);
    }
    return ret;
}

static int gy521_read_word(struct i2c_client *client, u8 reg)
{
    s32 ret;
    u8 data[2];

    ret = i2c_smbus_read_i2c_block_data(client, reg, 2, data);
    if (ret < 0) {
        pr_err("GY-521: Failed to read word from register 0x%02x\n", reg);
        return ret;
    }

    return (data[0] << 8) | data[1];
}

// Handle the device from user-space
static int gy521_open(struct inode *inode, struct file *file){
    pr_info("GY-521: device is opened");

    int ret = gy521_write_byte(gy521_client, PWR_MGMT_1, 0x00);
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
    accel_x = gy521_read_word(gy521_client, ACCEL_X_REG);
    accel_y = gy521_read_word(gy521_client, ACCEL_Y_REG);
    accel_z = gy521_read_word(gy521_client, ACCEL_Z_REG);

    // Read data gyroscope (2 bytes each axis)
    gyro_x = gy521_read_word(gy521_client, GYRO_X_REG);
    gyro_y = gy521_read_word(gy521_client, GYRO_Y_REG);
    gyro_z = gy521_read_word(gy521_client, GYRO_Z_REG);

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
    
    i2c_for_each_dev(NULL, check_device);
    
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

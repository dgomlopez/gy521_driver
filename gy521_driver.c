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
#define WHO_AM_I_REG 0x75   // To verify the correct address


static int major_number;                    // Register number in /dev
static struct class* gy521_class = NULL;    // user-space
static struct device* gy521_device = NULL;  // /dev

static struct i2c_adapter *adapter = NULL;         // Access to I2C (connection with the hardware)
struct i2c_adapter *stored_adapter = NULL;
static int gy521_addr = 0;
struct i2c_board_info gy521_info = {
    I2C_BOARD_INFO("gy521", gy521_addr)
};
static struct i2c_client *gy521_client = NULL;     // Represent gy521 device on the I2C
static int check_addr(struct i2c_adapter *adapter, int addr){
        struct i2c_client *client;
        char buf;
        int ret;

        client = i2c_new_dummy(adapter, addr);
        if(!client)
            return -ENODEV;
        
        ret = i2c_smbus_read_byte_data(client, WHO_AM_I_REG);
        i2c_unregister_device(client);

        if (ret < 0)
            return -ENODEV;
        
        return 0;
}

// Handle device from user-space
static int gy521_open(struct inode *inode, struct file *file){
    pr_info("GY-521: device is opened");

    return 0;
}

static int gy521_release(struct inode *inode, struct file *file){
    pr_info("GY-521: device is closed");

    return 0;
}

static struct file_operations fops = {
    .open = gy521_open,
    .release = gy521_release,
};

static int __init gy521_init(void){

    pr_info("GY-521: driver is loaded");

    major_number = register_chrdev(0, DEVICE_NAME, &fops); // To register the device on the system
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

    i2c_for_each_adapter(adapter) {
        if (check_addr(adapter,MPU6050_ADDR_1) == 0){
            pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_1);
            gy521_addr = MPU6050_ADDR_1;
            stored_adapter = adapter;
            break;
        }
        if (check_addr(adapter,MPU6050_ADDR_2) == 0){
            pr_info("GY-521: device found at 0x%02X", MPU6050_ADDR_2);
            gy521_addr = MPU6050_ADDR_2;
            stored_adapter = adapter;
            break;
        }
    }

    gy521_info.addr = gy521_addr;

    if (gy521_addr == 0){
        pr_err("GY-521: device not found on I2C bus");
        return -ENODEV;
    }

    gy521_client = i2c_new_device(stored_adapter, &gy521_info);
    if (!gy521_client){
        pr_err("GY-521: create I2C client failed");
        return -ENODEV;
    }
    
    return 0;
}

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

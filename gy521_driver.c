#include <linux/module.h>    // macros module_init() and module_exit()
#include <linux/init.h>     // macros __init and __exit


static int __init gy521_init(void){

}

static void __exit gy521_exit(void){

}

module_init(gy521_init);
module_exit(gy521_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Gómez López");
MODULE_DESCRIPTION("GY-521 I2C Driver");
MODULE_VERSION("0.1");

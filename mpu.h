#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/kthread.h>


#define CRIT_FAIL -1
#define DEBUG_MSG true
#define error_notice(...) {if(unlikely(DEBUG_MSG==true))printk(KERN_ALERT __VA_ARGS__);}while(0)

struct mpu
{
	struct i2c_client *client;
	struct miscdevice misc;
	DECLARE_KFIFO_PTR( fifo, u8 );
	spinlock_t lock;
	unsigned int gpio_numb;
	volatile unsigned long  state;
	struct task_struct	*rx_thread;
	wait_queue_head_t	read_wait;
	size_t needed;
	u16 overflow_count;
	u16 fifo_sz;
        u8 splrt;
};

static s8 crit_fail = 0;

static struct mpu *mpu6050;

static inline void close_mpu(struct mpu* mpu){clear_bit(1, &mpu->state);}
static inline void open_mpu(struct mpu* mpu){set_bit(1, &mpu->state);}
static inline int is_mpu_open(struct mpu* mpu)
{return test_bit(1, &mpu->state);}

static inline void sleeping(struct mpu* mpu){clear_bit(2, &mpu->state);}
static inline void retrieving_data(struct mpu* mpu){set_bit(2, &mpu->state);}
static inline int is_it_retrieving(struct mpu* mpu)
{return test_bit(2, &mpu->state);}

static inline void took_care(struct mpu* mpu){clear_bit(3, &mpu->state);}
static inline void something_happened(struct mpu* mpu){set_bit(3, &mpu->state);}
static inline int did_something_happened(struct mpu* mpu)
{return test_bit(3, &mpu->state);}


#define FIFO_SZ 96
#define IRQ_GPIO 4 //juste en dessous du SCL

#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define SMPLRT 0x19
#define CFG 0x1A
#define GYRO_CFG 0x1B
#define ACCEL_CFG 0x1C
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define INT_PIN_CFG 0x37
#define INT_EN 0x38
#define INT_STATUS 0x3A
#define AOUT_X_H 0x3B
#define AOUT_X_L 0x3C
#define AOUT_Y_H 0x3D
#define AOUT_Y_L 0x3E
#define AOUT_Z_H 0x3F
#define AOUT_Z_L 0x40
#define TOUT_H 0x41
#define TOUT_L 0x42
#define GOUT_X_H 0x43
#define GOUT_X_L 0x44
#define GOUT_Y_H 0x45
#define GOUT_Y_L 0x46
#define GOUT_Z_H 0x47
#define GOUT_Z_L 0x48
#define SIG_PATH_RES 0x68 
#define USER_CTRL 0x6A
#define PWR_MGMT1 0x6B
#define PWR_MGMT2 0x6C
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75

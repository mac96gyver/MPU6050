#include "mpu.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LOUIS GUERLIN");
MODULE_DESCRIPTION("Driver I2C du MPU6050");

static struct i2c_board_info mpu_device[] = {
	{
		I2C_BOARD_INFO("mpu_6050", 0x68),
		//.irq = n° de la pin d'irq associée,
	},
};

static int mpu_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	//Some variables
	u8 i = 0;
	s32 ret = 0;
	u8 address = 0;
	s16 temp = 0;
	s16 out = 0,outst = 0;
	s16 gtrim[3] = {6147, -6147, 8421};
	s16 atrim[3] = {3635, 3973, 3864};
	s16 str[3] = {0, 0, 0};

	//A verification just in case
	address = i2c_smbus_read_byte_data(client,WHO_AM_I);
	printk("MPU6050 : I2C's address is 0x%x\n",address);

	//Reset the MPU6050
	ret = i2c_smbus_write_byte_data(client,PWR_MGMT1,0x80); 
	msleep(100); 

	ret = i2c_smbus_write_byte_data(client,PWR_MGMT1,0x00); 
	msleep(100);

	ret = i2c_smbus_write_byte_data(client,USER_CTRL,0x00);  
	msleep(100);

	ret = i2c_smbus_write_byte_data(client,USER_CTRL,0x07);  
	msleep(100);

	ret = i2c_smbus_write_byte_data(client,USER_CTRL,0x00);  
	msleep(100);
	 
	ret = i2c_smbus_write_byte_data(client,SIG_PATH_RES,0x07);
	msleep(100);

	ret = i2c_smbus_write_byte_data(client,SIG_PATH_RES,0x00);
	msleep(100);


	
	if (0x00 != i2c_smbus_read_byte_data(client,PWR_MGMT1))
		error_notice("MPU6050 :  Is in sleep mode...\n");
	
	//To see if the MPU6050 is working -> Read the temp (~approx)
	temp = i2c_smbus_read_byte_data(client,TOUT_H)<<8;
	temp |= i2c_smbus_read_byte_data(client,TOUT_L);
	temp=temp/340+37;
	printk("MPU6050 : The temperature of the room is %d °C\n",temp);
	
	//Self test of the gyroscope
	for (i=0;i<3;i++)
	{
		out = i2c_smbus_read_byte_data(client,GOUT_X_H+2*i)<<8;	
		out |= i2c_smbus_read_byte_data(client,GOUT_X_L+2*i);
		ret = i2c_smbus_write_byte_data(client,GYRO_CFG,0x80>>i);
		outst = i2c_smbus_read_byte_data(client,GOUT_X_H+2*i)<<8;	
		outst |= i2c_smbus_read_byte_data(client,GOUT_X_L+2*i);
		ret = i2c_smbus_write_byte_data(client,GYRO_CFG,0x00);
		str[i]=(out-outst-gtrim[i])/gtrim[i];
	}
	printk("MPU6050 : Self_Test_GX = %d\n",str[0]);
	printk("MPU6050 : Self_Test_GY = %d\n",str[1]);
	printk("MPU6050 : Self_Test_GZ = %d\n",str[2]);

	//Self test of the accelerometer
	for (i=0;i<3;i++)
	{
		out = i2c_smbus_read_byte_data(client,AOUT_X_H+2*i)<<8;	
		out |= i2c_smbus_read_byte_data(client,AOUT_X_L+2*i);
		ret = i2c_smbus_write_byte_data(client,ACCEL_CFG,0x80>>i);
		outst = i2c_smbus_read_byte_data(client,AOUT_X_H+2*i)<<8;	
		outst |= i2c_smbus_read_byte_data(client,AOUT_X_L+2*i);
		ret = i2c_smbus_write_byte_data(client,ACCEL_CFG,0x00);
		str[i]=(out-outst-atrim[i])/atrim[i];
	}

	printk("MPU6050 : Self_Test_AX = %d\n",str[0]);
	printk("MPU6050 : Self_Test_AY = %d\n",str[1]);
	printk("MPU6050 : Self_Test_AZ = %d\n",str[2]);

	//Setup of the MPU

	//CONFIG: Low pass filter activated and fsync deactivated
	ret = i2c_smbus_write_byte_data(client,CFG,0x01);
	msleep(100); 
	//SAMPLE RATE: 1000Hz (LP activated)
	ret = i2c_smbus_write_byte_data(client,SMPLRT,0x00);
	msleep(100); 
	//GYROSCOPE: +/-250°/s
	ret = i2c_smbus_write_byte_data(client,GYRO_CFG,0x08);
	msleep(100); 
	//ACCELEROMETER: max 2g (ie 2 m/s/s)
	ret = i2c_smbus_write_byte_data(client,ACCEL_CFG,0x08);
	msleep(100); 
	//INTERRUPTION PIN CONFIG: The interruption pin is cleared after 50 us
	ret = i2c_smbus_write_byte_data(client,INT_PIN_CFG,0x10);
	msleep(100); 
	//INT EN: When an overflow is reached,
	//        an interruption is generated
	ret = i2c_smbus_write_byte_data(client,INT_EN,0x10);
	msleep(100); 	
	//POWER MANAGEMENT: Turning ON the MPU with the Temp sens deactivated
	i2c_smbus_write_byte_data(client,PWR_MGMT1,0x09);
	msleep(100); 
        //If there was an error while writting into MPU's registers
	if (ret != 0)
	{
		error_notice("MPU6050 : I2C might not actually work...\n");
	}

	return ret;
}
	

static int mpu_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static struct i2c_device_id mpu_idtable[] = {
	{ "mpu_6050", 0 },//nom, identifiant des donnees du pilote
	{ },
};

MODULE_DEVICE_TABLE(i2c, mpu_idtable);
/* TODO: transform this irq in fifo of detection thread */
static inline irqreturn_t mpu_interrupt_event_thread(int irq, void *data)
{
	struct mpu *mpu6050=(struct mpu *)data;
	something_happened(mpu6050);
	return IRQ_HANDLED;
}

static inline int mpu_data_thread(int stop)
{
	int i;
	u8 try = 0;
	u8 out[24];
	u16 inc = 24;
	u16 count = 0;
	u16 content = 0;
	s32 ret=0;
	i2c_smbus_write_byte_data(mpu6050->client,FIFO_EN,0x78);
	i2c_smbus_write_byte_data(mpu6050->client,USER_CTRL,0x40);
	while (!kthread_should_stop())
	{
		if(!is_it_retrieving(mpu6050))
		{
			retrieving_data(mpu6050);
			
			count = i2c_smbus_read_byte_data(mpu6050->client,FIFO_COUNT_H)<<8;
			count |= i2c_smbus_read_byte_data(mpu6050->client,FIFO_COUNT_L);
			

			content = count/inc;
			for (i=0;i<content;i=i+1)
			{

				while(((ret=i2c_smbus_read_i2c_block_data(mpu6050->client,
									  FIFO_R_W,inc,out))!=inc)
				      && try<10)
				{
					if (ret>=0)
						inc = inc-ret;
					try++;
				
				}
				inc = 24;
				try = 0;
				kfifo_in_spinlocked(&mpu6050->fifo,out,inc,&mpu6050->lock);
				/*printk("MPU6050 : AX = %hd ",(short int)(out[0]<<8|out[1]));
				printk("AY = %hd ",(short int)(out[2]<<8|out[3]));
				printk("AZ = %hd\n",(short int)(out[4]<<8|out[5]));
				printk("MPU6050 : GX = %hd ",(short int)(out[6]<<8|out[7]));
				printk("GY = %hd ",(short int)(out[8]<<8|out[9]));
				printk("GZ = %hd\n",(short int)(out[10]<<8|out[11]));
				printk("MPU6050 : AX = %hd ",(short int)(out[12]<<8|out[13]));
				printk("AY = %hd ",(short int)(out[14]<<8|out[15]));
				printk("AZ = %hd\n",(short int)(out[16]<<8|out[17]));
				printk("MPU6050 : GX = %hd ",(short int)(out[18]<<8|out[19]));
				printk("GY = %hd ",(short int)(out[20]<<8|out[21]));
				printk("GZ = %hd\n",(short int)(out[22]<<8|out[23]));*/
				
			}
		        if (mpu6050->needed<kfifo_len(&mpu6050->fifo))
				wake_up_interruptible(&mpu6050->read_wait);
			
			sleeping(mpu6050);
		}
		if(likely(!did_something_happened(mpu6050)))
		{
			__set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/100);//10ms
			__set_current_state(TASK_RUNNING);
		}
		else
		{
			took_care(mpu6050);
			mpu6050->splrt += 1;
		        i2c_smbus_write_byte_data(mpu6050->client,
							SMPLRT,mpu6050->splrt);
		        error_notice("MPU6050 : Sample rate is too high !\n");
			error_notice("MPU6050 : New sample rate is %d Hz\n",
				     1000/(mpu6050->splrt+1));
		}
	}
	return 0;
}	



static struct i2c_driver mpu_driver = {
	.driver =
	{
		.name = "mpu_6050",
	},
	.id_table = mpu_idtable,
	.probe = mpu_i2c_probe,
	.remove = mpu_i2c_remove,
};

static ssize_t read_function(struct file *file, char *buf,
			     size_t count, loff_t *ppos)
{
        struct mpu *mpu = (struct mpu*) file->private_data;
	int tmp;
	int err = 0;
	unsigned int a;
	if (mpu->fifo_sz<=*ppos || count==0)
	{
		return 0;
	}
	
	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&mpu->read_wait, &wait);
	
	if (count%12 != 0)
	{
		count = count-count%12;
	}
	
	
	while (true)
	{
		/* Normal Operation : A packet has been received */
		if ((a=kfifo_len(&mpu->fifo))>count)
		{
			break;
		}

		/* Signal received from kernel */
		if (signal_pending(current))
		{
			err = 0;
			remove_wait_queue( &mpu->read_wait, &wait );
			return -ERESTARTSYS;
		}

		if ( err )
		{
			remove_wait_queue( &mpu->read_wait, &wait );
			err = 0;
			return -ETIMEDOUT;
		}

		err = -ETIMEDOUT;
		mpu->needed = count;
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		__set_current_state(TASK_RUNNING);
		mpu->needed = 0;
	}

	if(kfifo_to_user(&mpu6050->fifo,buf,count,&tmp)) 
	{ 
		remove_wait_queue( &mpu->read_wait, &wait );
		return -EINVAL; 
	} 
	
	*ppos+=tmp;

	remove_wait_queue( &mpu->read_wait, &wait );

	return count;
}

static ssize_t write_function(struct file *file, const char *buf,
			      size_t count, loff_t *ppos)
{
    
	return 0;
}

static int open_function(struct inode *inode, struct file *file)
{
	file->private_data = (struct mpu*) mpu6050;
	if (!is_mpu_open(mpu6050))
		open_mpu(mpu6050);
	else return 1;
	
	return 0;
}

static int release_function(struct inode *inode, struct file *file)
{
	file->private_data = (struct mpu*) mpu6050;
	close_mpu(mpu6050);
	return 0;
}

struct file_operations fops =
{
	.read = read_function,
	.write = write_function,
	.open = open_function,
	.release = release_function
};


static int __init init_i2c(void)
{
	mpu6050=kzalloc(sizeof(struct mpu),GFP_KERNEL);
	if (mpu6050==NULL)
	{
		error_notice("MPU6050 : Can't allocate memory...\n");
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	mpu6050->client=i2c_new_device(i2c_get_adapter(1),mpu_device);
	if (mpu6050->client==NULL)
	{
		error_notice("MPU6050 : Can't register MPU's i2c client...\n");
		kfree(mpu6050);
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	if (i2c_add_driver(&mpu_driver)!=0)
	{
		error_notice("MPU6050 : Can't add MPU's driver...\n");
		i2c_unregister_device(mpu6050->client);
		kfree(mpu6050);
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	printk("MPU6050 : I2C interface successfully probed!\n");

	mpu6050->misc.name = "MPU6050";
	mpu6050->misc.minor = MISC_DYNAMIC_MINOR;
	mpu6050->misc.fops = &fops;
	
	if(misc_register(&(mpu6050->misc))!=0)
	{
		error_notice("MPU6050 : Can't register the misc_device...\n");
		i2c_del_driver(&mpu_driver);
		i2c_unregister_device(mpu6050->client);
		kfree(mpu6050);
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	if (kfifo_alloc(&mpu6050->fifo,FIFO_SZ,GFP_KERNEL))
	{
		error_notice("MPU6050 : Kfifo_alloc error...\n");
		misc_deregister(&mpu6050->misc);
		i2c_del_driver(&mpu_driver);
		i2c_unregister_device(mpu6050->client);
		kfree(mpu6050);
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	
	spin_lock_init(&mpu6050->lock);
	mpu6050->splrt = 0;
	mpu6050->fifo_sz = FIFO_SZ;
	mpu6050->state = 0;
	mpu6050->needed = 0;
	init_waitqueue_head(&mpu6050->read_wait);
	if (gpio_request(IRQ_GPIO, "mpu.irq")) 
	{
		error_notice("MPU6050 : Wasn't able to request the GPIO..."); 
        	kfifo_free(&mpu6050->fifo); 
        	misc_deregister(&mpu6050->misc); 
		i2c_del_driver(&mpu_driver); 
       	i2c_unregister_device(mpu6050->client); 
		kfree(mpu6050);
		crit_fail=CRIT_FAIL;
		return CRIT_FAIL;
	}
	

	 if(gpio_direction_input(IRQ_GPIO)) 
	 { 
	 	error_notice("MPU6050 : Unable to set the GPIO direction...\n"); 
	 	gpio_free(IRQ_GPIO); 
	 	kfifo_free(&mpu6050->fifo); 
	 	misc_deregister(&mpu6050->misc); 
		i2c_del_driver(&mpu_driver); 
	 	i2c_unregister_device(mpu6050->client); 
	 	kfree(mpu6050); 
	 	crit_fail=CRIT_FAIL; 
	 	return CRIT_FAIL; 
	 }
	
	 mpu6050->gpio_numb=gpio_to_irq(IRQ_GPIO); 
	
	 if (request_irq(mpu6050->gpio_numb, 
	 		      mpu_interrupt_event_thread, 
			 IRQF_TRIGGER_RISING|IRQF_ONESHOT, 
	 		      "mpu.pkt_handler", mpu6050)) 
	 { 
		
		error_notice("MPU6050 : Cannot get IRQ\n"); 
	 	gpio_free(IRQ_GPIO); 
	 	kfifo_free(&mpu6050->fifo); 
	 	misc_deregister(&mpu6050->misc); 
	 	i2c_del_driver(&mpu_driver); 
	 	i2c_unregister_device(mpu6050->client); 
	 	kfree(mpu6050); 
		crit_fail=CRIT_FAIL; 
	 	return CRIT_FAIL; 
		} 
	 
	 
	 mpu6050->rx_thread=kthread_run((void *)mpu_data_thread,NULL,"mpu.transfer"); 

	 if (mpu6050->rx_thread==NULL) 
	 { 
	 	error_notice("MPU6050 : Cannot get IRQ\n"); 
	 	free_irq(mpu6050->gpio_numb, mpu6050); 
		gpio_free(IRQ_GPIO); 
        	kfifo_free(&mpu6050->fifo); 
		misc_deregister(&mpu6050->misc); 
		i2c_del_driver(&mpu_driver); 
		i2c_unregister_device(mpu6050->client); 
		kfree(mpu6050); 
		crit_fail=CRIT_FAIL; 
	}

	
	 printk("MPU6050 : Initialisation OK !\n");

	return 0;

}

static void __exit exit_i2c(void)
{
	if (crit_fail == 0)
	{
		kthread_stop(mpu6050->rx_thread);
		free_irq(mpu6050->gpio_numb, mpu6050);
		gpio_free(IRQ_GPIO);
		kfifo_free(&mpu6050->fifo);
		misc_deregister(&mpu6050->misc);
		i2c_del_driver(&mpu_driver);
		i2c_unregister_device(mpu6050->client);
		kfree(mpu6050);
		printk("MPU6050 : MPU closed!\n");
	}
	
	return;
}

module_init(init_i2c);
module_exit(exit_i2c);


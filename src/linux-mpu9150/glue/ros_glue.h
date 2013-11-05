#ifndef ROS_GLUE_H
#define ROS_GLUE_H

#define log_i		printf
#define log_e		printf
#define min(a, b) 	((a < b) ? a : b)

static inline int reg_int_cb(struct int_param_s *int_param)
{
	return 0;
}

void __no_operation(void);
int delay_ms(unsigned long num_ms);
int get_ms(unsigned long *count);

extern int i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data);
extern int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif // ROS_GLUE_H

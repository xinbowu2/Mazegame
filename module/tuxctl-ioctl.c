/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__) 

#define DECIMAL_MASK 0x10
#define BUTTON_OTHER_MASK 0x0F	
#define BUTTON_RU_MASK 0x09
#define BUTTON_D_MASK 0x04
#define BUTTON_L_MASK 0x02

#define LED0_MASK 0x0000000F
#define LED1_MASK 0x000000F0
#define LED2_MASK 0x00000F00
#define LED3_MASK 0x0000F000
#define LED0_ON 0x00010000
#define LED1_ON 0x00020000
#define LED2_ON 0x00040000
#define LED3_ON 0x00080000
#define LED0_DEC 0x01000000
#define LED1_DEC 0x02000000
#define LED2_DEC 0x04000000
#define LED3_DEC 0x08000000
#define DECIMAL_MASK 0x10


//define buffers containning messages
char init_buf[2]={MTCP_BIOC_ON,MTCP_LED_USR};
char led_buf[6]={MTCP_LED_SET,0x0F,0x00,0x00,0x00,0x00};
char reset_buf[8]={MTCP_BIOC_ON,MTCP_LED_USR,MTCP_LED_SET,0x0F,0x00,0x00,0x00,0x00};

//helper variables
int num_cmd; //number of commands in process
unsigned long button_state=255;
int hex2segment[17]={0xE7,0x06,0xCB,0x8F,0x2E,0xAD,0xED,0x86,0xEF,0xAF,0xEE,0x6D,0xE1,0x4F,0xE9,0xE8,0x00};

void send2TC(struct tty_struct* tty, char* buf, int n, int num_cmd);

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */

 /*
 * tuxctl_handle_packet
 *   DESCRIPTION: handle data packages received from the tux controller.
 *   INPUTS: tty -- text telephone
 *			 packet -- data packet (three bytes)
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: process the packet based on opcode in the the first byte
 */   
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
	unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];


    int i;//for possible loops 

    switch (a){
    	case MTCP_RESET:
    		num_cmd = 0;
    		for(i=2;i<6;i++)
				reset_buf[i+2]=led_buf[i]; //key led display, while resetting
			
    		send2TC(tty,reset_buf,sizeof(reset_buf), num_cmd);
    		num_cmd += 3; //three commands in reset
    		break;

    	case MTCP_BIOC_EVENT: 
    		//storage button press information from the packet to button_state
    		button_state = (b & BUTTON_OTHER_MASK) | (((c & BUTTON_RU_MASK) | ((c & BUTTON_L_MASK)<<1) | 
    			((c & BUTTON_D_MASK)>>1)) << 4); 
    		break;

    	case MTCP_ACK:
    		if (num_cmd > 0)
    			num_cmd--; //decrement number of commands, when receiving MTCP_ACK, which indicates completion of a command 
    		break;

    }

    /*printk("packet : %x %x %x\n", a, b, c); */
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/

/* tuxctl_ioctl()
 * Choose functions based on the cmd 
 * There are three functions: INIT, BUTTON and SET_LED 
 * INIT: Initialize the tux controller
 * BUTTON: Read the button state
 * SET_LED: set led to display values
 */

 /*
 * tuxctl_ioctl
 *   DESCRIPTION: control tux controller's behavior based on a command input
 *   INPUTS: tty -- text telephone
 *			 file -- the opened device file
 *			 cmd -- command
 *			 arg -- arguments for some commands 
 *   OUTPUTS: none
 *   RETURN VALUE: return 0 for processing successful and -EINVAL for failure
 *   SIDE EFFECTS: the tux controller will function as required by a command
 */   
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{

  int ack;
  switch (cmd) {
	case TUX_INIT:
		num_cmd = 0;
		send2TC(tty,init_buf,sizeof(init_buf),num_cmd);
		num_cmd += 2; //Two commands in init
		return 0;

	case TUX_BUTTONS: 
		//when the pointer is valid
		if (arg != 0){
			copy_to_user((int*)arg,&button_state,sizeof(button_state));
			return 0;
		}
		else
			return -EINVAL;

	case TUX_SET_LED:
	    if(arg&LED0_ON){
			led_buf[2]=hex2segment[arg&LED0_MASK];
			if(arg&LED0_DEC){
				led_buf[2]=led_buf[2]|DECIMAL_MASK;
			}
		}
		else
			led_buf[2]=hex2segment[16];
		
		if(arg&LED1_ON){
			led_buf[3]=hex2segment[(arg&LED1_MASK)>>4];
			if(arg&LED1_DEC){
				led_buf[3]=led_buf[3]|DECIMAL_MASK;
			}
		}
		else
			led_buf[3]=hex2segment[16];

		if(arg&LED2_ON){
			led_buf[4]=hex2segment[(arg&LED2_MASK)>>8];
			if(arg&LED2_DEC){
				led_buf[4]=led_buf[4]|DECIMAL_MASK;
			}
		}
		else
			led_buf[4]=hex2segment[16];

		if(arg&LED3_ON){
			led_buf[5]=hex2segment[(arg&LED3_MASK)>>12];
			if(arg&LED3_DEC){
				led_buf[5]=led_buf[5]|DECIMAL_MASK;
			}
		}
		else
			led_buf[5]=hex2segment[16];
		
		//send2TC(tty,led_buf,sizeof(led_buf),num_cmd);
		//num_cmd++;
		tuxctl_ldisc_put(tty,led_buf,sizeof(led_buf));
		num_cmd++;
		
	    return 0;

	default:
	    return -EINVAL;
    }
}

/*
 * send2TC
 *   DESCRIPTION: send data to tux controller while checking if there is no sending command in process
 *   INPUTS: tty -- text telephone
 *			 buf t-- a buffer containing data to be sent to the tux controller 
 *			 n -- the size of the buffer 
 *			 num_cmd -- number of commands in process
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: send the data contained in the buffer to the tux controller
 */   
void send2TC(struct tty_struct* tty, char* buf, int n, int num_cmd){
    if(num_cmd==0){
        tuxctl_ldisc_put(tty,buf,n);
    }
}


#include "serial_reg.h"

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/delay.h>

MODULE_LICENSE("Dual BSD/GPL");

#define MODULE_NAME "serp"
#define SERIAL_PORT_BASE_ADDR 0x3f8
#define DEFAULT_MAJOR_NUMBER 4343
#define DEFAULT_MINOR_NUMBER 0
#define MAX_NUMBER_OF_DEVICES 4 

/*********************************************************************/
//                   global variables
static dev_t dev_num = MKDEV(DEFAULT_MAJOR_NUMBER, DEFAULT_MINOR_NUMBER);
struct cdev cdev;  // global struct of the character device
static int number_of_devices = 1;
/*********************************************************************/

int serial_port_open(struct inode *inodep, struct file * filp)
{
    nonseekable_open(inodep, filp);
    filp->private_data = &cdev;
    printk(KERN_ALERT "INFO: UART device opened.\n");
    return 0;
}

int serial_port_release(struct inode *inodep, struct file * filp)
{
    printk(KERN_ALERT "INFO: the UART device was closed.\n");
    return 0;
}


// read implementation
ssize_t serial_port_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    unsigned char lsr_register = 0; // line status register of UART
    int i;
    int dormant_cycles = 0;  // counter for the number of cycles the UART is not active
    unsigned char error_flags;

    printk(KERN_ALERT "INFO: UART device is being read.\n");

    // allocate a kernel buffer for reading data from UART
    unsigned char *buffer = (unsigned char *)kzalloc(count * sizeof(unsigned char), GFP_KERNEL);
    if (!buffer) 
    {
        printk(KERN_ALERT "ERROR: Read() failed to allocate memory in kernel space!\n");
        return -1;
    }


    // read the bytes from UART device
    for(i = 0; i < count; i++)
    {
        // first: read the value of the line status register
        lsr_register = inb(SERIAL_PORT_BASE_ADDR + UART_LSR);

        // keep checking the Data Ready flag of the line status register
        // this flag indicates that a new byte is received in the uart 
        while((lsr_register & UART_LSR_DR) == 0)
        {
            // read the value of LSR again
            lsr_register = inb(SERIAL_PORT_BASE_ADDR + UART_LSR);

            // read the error flags from LSR and check them
            error_flags = lsr_register & (UART_LSR_FE | UART_LSR_PE | UART_LSR_OE);
            if(error_flags != 0) 
            {
                printk(KERN_ALERT "ERROR: Read() failed to receive a character from UART because of a line error!\n");
                return -EIO;
            }

            // sleep for 5ms to give the hardware a chance to re-fill the receive buffer
            msleep_interruptible(5);
            
            if(i > 0)
            {
                // increment the dormant_cycles counter
                dormant_cycles++;
            }

            // check if the UART line is idle for a very long period
            if(dormant_cycles > 100)
            {
                // if the UART was idle or dormant for a long period
                // copy to the user what's read from UART so far and return
                if(copy_to_user(buf,buffer,i) != 0)
                {
                    printk(KERN_ALERT "ERROR: Read() failed to move data from Kernel space to User space!\n");
                    kfree(buffer);
                    return -1; // return an error condition
                }
                else
                {
                    kfree(buffer);
                }
                return i;  // return before reading the whole (count) bytes, because UART line hanged
            }
        }

        // the UART device just received 1 character, read it into the kernel buffer
        // read from the RX buffer, also clears the DR bit of LSR
        buffer[i] = inb(SERIAL_PORT_BASE_ADDR + UART_RX);

        // reset the dormant cycles counter to prepare for the next character
        dormant_cycles = 0;
    }

    // copy the bytes from the kernel space buffer to the user space buffer
    if(copy_to_user(buf, buffer, i) != 0)
    {
        printk(KERN_ALERT "ERROR: Read() failed to move data from Kernel space to User space!\n");
        kfree(buffer);
        return -1;
    }
    else
    {
        // copy_to_user() succeeded
        kfree(buffer);
    }

    // return the number of characters successfully read
    return i;
}

// write implementation
ssize_t serial_port_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    int i;

    unsigned char *buffer = (unsigned char *)kzalloc(count, GFP_KERNEL);
    if (!buffer) 
    {
        printk(KERN_ALERT "ERROR: Write() failed to allocate memory in kernel space!\n");
        return -1;
    }

    printk(KERN_ALERT "INFO: UART device is being written.\n");


    // transfer the data from user memory to kernel memory
    if(copy_from_user(buffer, buf, count) != 0)
    {
        printk(KERN_ALERT "ERROR: Write() failed to move data from userspace to kernel space!\n");
        kfree(buffer);
        return -1;
    }

    // start writing data from the kernel memory to the UART device
    for(i = 0; i < count; ++i)
    {
        // if the transmitter holding register empty flag is not set, wait until it is set
        // before trying to write another byte into UART
        while(!(inb(SERIAL_PORT_BASE_ADDR + UART_LSR) & UART_LSR_THRE))
        {
            schedule(); // tell the kernel we are waiting for a hardware flag (instead of spinlock)
        }

        // the transmitter buffer is empty now (content is sent)
        // it's safe to write a new byte into it
        outb(buffer[i], SERIAL_PORT_BASE_ADDR + UART_TX);
    }

    // done writing all the data, free the kernel buffer
    kfree(buffer);

    // return the number of characters successfully written into UART
    return i;
}


/*****************************************************************************/
// global file operations structure
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .open = serial_port_open,
    .release = serial_port_release,
    .read = serial_port_read,
    .write = serial_port_write,
};
/*****************************************************************************/


// driver initialization function
static int serial_polling_init(void)
{
    int result;

    // reserve device numbers for our driver
    int major_number = MAJOR(dev_num);
    if (major_number)
    {
        result = register_chrdev_region(dev_num, MAX_NUMBER_OF_DEVICES, MODULE_NAME);
    }
    else
    {
        result = alloc_chrdev_region(&dev_num, 0, MAX_NUMBER_OF_DEVICES, MODULE_NAME);
    }

    if(result < 0)
    {
        printk(KERN_ALERT "ERROR: failed to allocate device numbers for %s!\n", MODULE_NAME);
        return -1;
    }

    // initialize a device
    cdev_init(&cdev, &fops);
    
    // modify the new device parameters
    cdev.owner = THIS_MODULE;
    cdev.ops = &fops;

    // add the new device to our driver
    result = cdev_add(&cdev, dev_num, number_of_devices);
    if(result < 0)
    {
        printk(KERN_ALERT "ERROR: cannot add a new character device to this driver %s\n", MODULE_NAME);
        return 0;
    }

    // request access to the IO port region of UART
    if(!request_region(SERIAL_PORT_BASE_ADDR, number_of_devices, MODULE_NAME))
    {
        printk(KERN_ALERT "ERROR: cannot access IO port address space for UART\n");
        return -1;
    }


    unsigned char lcr_register = 0; // line control register of UART
    // initialize the UART device parameters
    outb(0x00, SERIAL_PORT_BASE_ADDR + UART_LCR); // initialize the LCR register
    outb(0x00, SERIAL_PORT_BASE_ADDR + UART_IER); // initialize the IER register
    
    // set parameters in the LCR register for serial line:
    // even parity, 2 stop bits, 8-bit characters
    lcr_register = UART_LCR_EPAR | UART_LCR_PARITY | UART_LCR_STOP | UART_LCR_WLEN8;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);
    
    // set the baudrate
    // first, set the DLAB bit to access the Divisor Latch registers
    lcr_register = lcr_register | UART_LCR_DLAB;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);

    // set the baudrate divisor values in the Divisor Latch register (LSB and MSB)
    outb(UART_DIV_1200, SERIAL_PORT_BASE_ADDR + UART_DLL);
    outb(0x00, SERIAL_PORT_BASE_ADDR + UART_DLM);

    // reset the DLAB bit in LCR register
    lcr_register = lcr_register & ~UART_LCR_DLAB;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);

    return 0;
}

// driver cleanup function
static void serial_polling_cleanup(void)
{
    release_region(SERIAL_PORT_BASE_ADDR, number_of_devices);
    cdev_del(&cdev);
    unregister_chrdev_region(dev_num, MAX_NUMBER_OF_DEVICES);
    printk(KERN_ALERT "Goodbye, Cruel world\n");
}

module_init(serial_polling_init);
module_exit(serial_polling_cleanup);

#include <linux/init.h>
#include<linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mm.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<asm/uaccess.h>
#include<linux/ioport.h>
#include<asm/io.h>
#include<linux/sched.h>
#include<linux/delay.h>
#include"serial_reg.h"
#include<linux/interrupt.h>
#include<linux/kfifo.h>
#include<asm/semaphore.h>
#include<linux/wait.h>
#include<linux/errno.h>
#include<linux/ioctl.h>
#include<asm/uaccess.h>

MODULE_LICENSE("Dual BSD/GPL");

#define MODULE_NAME "seri"
#define SERIAL_PORT_BASE_ADDR 0x3f8
#define MAX_NUMBER_OF_DEVICES 4
#define DEFAULT_MAJOR_NUMBER 0
#define DEFAULT_MINOR_NUMBER 0
#define IRQ_NUMBER 4
#define QUEUE_SIZE 32768

typedef struct {
    struct cdev cdev;
    struct semaphore device_state_semaphore;
    struct kfifo *rx_queue;
    struct kfifo *tx_queue;

    int num_sent;
    int rx_error;
    int tx_end;
    int rx_not_empty;
    int num_accesses;

    wait_queue_head_t rx_wait_queue;
    wait_queue_head_t tx_wait_queue;

    spinlock_t queue_lock;
    spinlock_t device_state_spinlock;
} device_state_t;

/*********************************************************************/
//                   global variables
static dev_t dev_num = MKDEV(DEFAULT_MAJOR_NUMBER, DEFAULT_MINOR_NUMBER);
device_state_t global_device_state;
/*********************************************************************/

// open a device file
int serial_interrupts_open(struct inode *inodep, struct file *filep)
{
    // the container_of macro gets the address of a struct
    // using the address of a sub-element of the struct and 
    // the struct datatype and the sub-element name
    device_state_t *the_device_state = container_of(inodep->i_cdev, device_state_t, cdev);
    
    // exclusive region for incrementing the device access counter
    // because many users could try to open() the device at the same time
    spin_lock(the_device_state->device_state_spinlock);
    if (the_device_state->num_accesses == 1)
    {
        // we are trying to open a device file that's already opened
        // but only one access at a time is allowed
        // so, return with an error status
        printk(KERN_ALERT "ERROR: multiple users are trying to open the same device at the same time.\n");
        return -EBUSY;
    }
    else 
    {
        // no other user has opened our device
        the_device_state->num_accesses++;
    }
    spin_unlock(the_device_state->device_state_spinlock);  // exit the exclusive region for the device state
    
    // the device is not seekable because it's a UART
    nonseekable_open(inodep, filep);

    // save a pointer to the device state in the filep->private_data
    filep->private_data = the_device_state;
    printk(KERN_ALERT "NOTE: somebody opened the serial_interrupts driver file.\n");
    return 0;
}

// implements the write operation, blocks the caller
ssize_t serial_interrupts_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
    int result;
    unsigned char first_element = 0;
    unsigned char *kernel_buffer;
    device_state_t *the_device_state;

    printk(KERN_ALERT "NOTE: somebody is writing into the serial_interrupts driver file.\n");

    // get the device state from the pointer in the filep passed to us
    the_device_state = (device_state_t *)filep->private_data;

    // if the user is trying to write in non-blocking mode
    // return an error because we will wait until the device finishes all transmission
    if (filep->f_flags & O_NONBLOCK)
    {
        printk(KERN_ALERT "ERROR: user trying to write device in non-blocking mode, try again in blocking mode.\n");
        return -1;
    }
    
    // allocate a kernel buffer to move the user data into
    kernel_buffer = (unsigned char *)kzalloc(count, GFP_KERNEL);
    if(kernel_buffer == NULL)
    {
        printk(KERN_ALERT "ERROR: failed to allocate a kernel buffer.\n");
        return -2;
    }

    // accessing the device state is protected by a semaphore
    down_interruptible(&the_device_state->device_state_semaphore);
    the_device_state->num_sent = 0;
    the_device_state->tx_end = 0;
    up(&the_device_state->device_state_semaphore);
    
    // copy data from user buffer to kernel buffer
    result = copy_from_user(kernel_buffer, buff, count); 
    if(result != 0)
    {
        printk(KERN_ALERT "ERROR: failed to copy data from user to kernel buffer.\n");
        kfree(kernel_buffer);
        return -3;
    }
    
    result = kfifo_put(the_device_state->tx_queue, kernel_buffer, count); 
    if(result == 0)
    {
        // tx queue is full
        printk(KERN_ALERT "ERROR: trying to write to the tx queue but it's full.\n");
        return -4;
    }
    
    // read the first element from the tx queue
    kfifo_get(the_device_state->tx_queue, &first_element, 1);

    // initiate the transmission by writing the first character from the queue to the uart tx buffer
    // this will generate an interrupt when the transmission is complete
    outb(first_element, SERIAL_PORT_BASE_ADDR + UART_TX);
    
    // update the device state
    down_interruptible(&the_device_state->device_state_semaphore);
    the_device_state->num_sent++;
    up(&the_device_state->device_state_semaphore);
    
    // wait until the device finishes transmitting all characters
    while(the_device_state->tx_end != 1)
    {
        // wait until the interrupt handler signals transmission end, or the timeout happens
        wait_event_interruptible_timeout(the_device_state->tx_wait_queue, (the_device_state->tx_end == 1), 500);
        // if the timeout happens and tx_end is still not 1, the while loop ensures that we wait again
    }
    
    kfree(kernel_buffer);
    return the_device_state->num_sent;
}

// release (close) the device file
int serial_interrupts_release(struct inode * inodep, struct file * filep)
{
    device_state_t *the_device_state = (device_state_t *)filep->private_data;

    // decrement the current number of concurrent accesses to the device
    // in an exclusive are, because many users can try to release at the same time
    spin_lock(&the_device_state->device_state_spinlock);
    the_device_state->num_accesses--;
    spin_unlock(&(the_device_state->device_state_spinlock)); // exit the exclusive area

    printk(KERN_ALERT "NOTE: somebody closed the serial_interrupts driver file.\n");
    return 0;  
}

// this implements the read operation
// this function will block if the rx queue is empty
ssize_t serial_interrupts_read(struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
    // get the global variable that holds our device state
    int num_characters_read = 0; 
    int result;
    int end_reception = 0;
    int waiting_timeout = 0;
    unsigned char *kernel_buffer;
    device_state_t *the_device_state = (device_state_t *)filep->private_data;

    printk(KERN_ALERT "NOTE: somebody is reading the serial_interrupts driver file.\n");

    if (filep->f_flags & O_NONBLOCK)
    {
        // the user is trying to open the file in non-blocking mode which is impossible
        printk("ERROR %d: user trying to read device in non-blocking mode, try again.\n", EAGAIN);
        return -1;
    }

    // allocate a kernel buffer to read data from the device into it
    kernel_buffer = (unsigned char *)kzalloc(count, GFP_KERNEL);
    if (kernel_buffer == NULL)
    {
        printk("ERROR: failed to allocate a kernel buffer.\n");
        return -2;
    }

    // decrement the device semaphore for accessing the device state global variable (like lock())
    down_interruptible(&the_device_state->device_state_semaphore);
    the_device_state->rx_error = 0;
    the_device_state->rx_not_empty = 0;
    up(&the_device_state->device_state_semaphore); // increment the semaphore again to allow others to access the device state
    
    while (num_characters_read < count)
    {
        if (the_device_state->rx_not_empty == 0)
        {
            // the rx queue is empty, we haven't received anything yet, wait for a long period
            waiting_timeout = 1500;
        }
        else
        {
            // the rx queue is not empty, we are already receiving some data, wait for a short period
            waiting_timeout = 300;
        }

        // this is a macro that waits for the condition to be true or for the timeout to happen
        result = wait_event_interruptible_timeout(the_device_state->rx_wait_queue,               // waiting queue
                                                  (kfifo_len(the_device_state->rx_queue) != 0),  // condiiton (rx queue is not empty)
                                                  waiting_timeout);                              // timeout in jiffies (kernel unit)
        
        switch(result)
        {
            case -ERESTARTSYS:
                // user interrupted the waiting by pressing ctrl+c
                printk(KERN_ALERT "ERROR: some one interrupted the read operation using ctrl + c\n");
                kfree(kernel_buffer);
                return -1;
                break;
        
            case  0:
                // timeout occured and the condition remained false (rx queue is still empty)
                printk(KERN_ALERT "WARNING: timeout occured and the rx queue was empty but only %d elements were read.\n", 
                       num_characters_read);
                // signal the end of reception
                end_reception = 1;
                break;
            
            case 1:
                // timeout occured and the condition was true after the timeout (rx queue is not empty)
                // read all the characters left in the rx queue into the kernel_buffer
                num_characters_read += kfifo_get(the_device_state->rx_queue, &kernel_buffer[num_characters_read], (count - num_characters_read));
                printk(KERN_ALERT "WARNING: timeout occured and the rx queue was not empty.\n");
                end_reception = 1;
                break;
            
            default:
                // condition became true before the time out happened
                num_characters_read += kfifo_get(the_device_state->rx_queue, &kernel_buffer[num_characters_read], (count - num_characters_read));
                break;
        }

        if (the_device_state->rx_error == 1)
        {
            down_interruptible(&the_device_state->device_state_semaphore);
            // reset error state
            the_device_state->rx_error = 0;
            up(&the_device_state->device_state_semaphore);
            printk(KERN_ALERT "ERROR: while reading an rx error happened.\n");
            return -EIO;
        }

        if (end_reception)
        {
            // if the end_reception flag was raised, stop trying to read from the rx queue
            break; // break out of the while loop
        }
    }
    // finished reading from the rx queue now

    // copy the read data from the kernel to the user buffer
    result = copy_to_user(buff, kernel_buffer, num_characters_read);
    if (result != 0)
    {
        printk(KERN_ALERT "ERROR: failed to copy data from kernel to user buffer.\n");
        kfree(kernel_buffer);
        return -3;
    }

    kfree(kernel_buffer);
    return num_characters_read;
}

// this function handles all kinds of interrupts when they happen immediately
irqreturn_t irq_handler(int irq, void *dev_id)
{
    device_state_t *the_device_state = (device_state_t *) dev_id;
    unsigned char rx_char = 0;
    unsigned char tx_char = 0;
    unsigned char iir_register = 0;
    
    iir_register = inb(SERIAL_PORT_BASE_ADDR + UART_IIR);
    
    if(iir_register & UART_IIR_RDI)
    {
        // this is an Rx interrupt
        // read the Rx buffer
        rx_char = inb(SERIAL_PORT_BASE_ADDR + UART_RX);
        // store the data in our Rx queue
        kfifo_put(the_device_state->rx_queue, &rx_char, 1);
        // set the flag that indicates that the device queue has at least one element
        the_device_state->rx_not_empty = 1;
        // notify and wakeup anyone waiting to receive data
        wake_up_interruptible(&the_device_state->rx_wait_queue);
    }
    else if (iir_register & UART_IIR_THRI)
    {
        // this is a Tx interrupt: the Tx buffer is sent and ready for more data
        if(kfifo_get(the_device_state->tx_queue, &tx_char, 1))
        {
            // write into the Tx buffer
            outb(tx_char, SERIAL_PORT_BASE_ADDR + UART_TX);
            // increment the counter for number of sent bytes
            the_device_state->num_sent++;
            // notify and wakeup anyone waiting to transmit data
            wake_up_interruptible(&the_device_state->tx_wait_queue);
        }
        else
        {
            // signal that transmission of the whole queue has ended
            the_device_state->tx_end = 1;
            // notify and wakeup anyone waiting to transmit data
            wake_up_interruptible(&the_device_state->tx_wait_queue);
        }
    }
    else if (iir_register & UART_IIR_RLSI)
    {
        // this is an error interrupt (receiver line status interrupt)
        // signal that there's an rx error
        the_device_state->rx_error = 1;
    }
    return IRQ_HANDLED;
}


// initialise the UART device registers with default configuration
int uart_hardware_init(void)
{
    unsigned char lcr_register = 0;
    unsigned char ier_register = 0;
    unsigned char dll_register = 0;

    // request access to the UART IO address region
    if(!request_region(SERIAL_PORT_BASE_ADDR, 1, MODULE_NAME))
    {
        printk("ERROR: failed to reserve UART io address region.\n");
        return -1;
    }

    // configure lcr register
    outb(0x00, SERIAL_PORT_BASE_ADDR + UART_LCR);
    lcr_register = UART_LCR_PARITY |  UART_LCR_WLEN8  | UART_LCR_EPAR | UART_LCR_STOP;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);

    // configure the IER register by enabling interrupts that we need
    ier_register =  UART_IER_THRI  | UART_IER_RLSI | UART_IER_RDI; // enable interrupts
    outb(ier_register, SERIAL_PORT_BASE_ADDR + UART_IER);

    // gain access to the baud rate registers DLL and DLM
    lcr_register = lcr_register | UART_LCR_DLAB;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);
   
    // set the baudrate
    dll_register = UART_DIV_1200; // the baudrate configuration
    outb(dll_register, SERIAL_PORT_BASE_ADDR + UART_DLL);
    outb(0x00, SERIAL_PORT_BASE_ADDR + UART_DLM);

    // remove access to the baudrate registers
    lcr_register = lcr_register & ~UART_LCR_DLAB;
    outb(lcr_register, SERIAL_PORT_BASE_ADDR + UART_LCR);

    // successfully initialised UART registers
    return 0;
}

static struct file_operations fops = {
    .read = serial_interrupts_read, 
    .open = serial_interrupts_open, 
    .write = serial_interrupts_write, 
    .release = serial_interrupts_release, 
    .owner = THIS_MODULE,
    .llseek = no_llseek,
};

// module initialisation function
static int serial_interrupts_init(void)
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

    if (result < 0) 
    {
        printk(KERN_ALERT "ERROR: failed to allocate device numbers for our driver\n");
        return -1;
    }

    // initialize the UART hardware registers    
    result = uart_hardware_init();
    if (result < 0)
    {
        printk(KERN_ALERT "ERROR: failed to initialise UART registers\n");
        return -1;
    }
    
    // initialise a device
    cdev_init(&global_device_state.cdev, &fops);
    global_device_state.cdev.owner = THIS_MODULE;
    global_device_state.cdev.ops = &fops;

    // add the new device to our driver
    result = cdev_add(&global_device_state.cdev, dev_num, 1);
    if(result < 0)
    {
        printk(KERN_ALERT "ERROR: cannot add a new character device to this driver %s\n", MODULE_NAME);
        return 0;
    }

    // initialise global_device_state access locks
    spin_lock_init(&global_device_state.device_state_spinlock); // uses spinlocking
    sema_init(&global_device_state.device_state_semaphore, 1);  // uses sleeping (maximum 1 user at the same time)

    // initialise the tx and rx queue locks
    // protects tx and rx queues from concurrent access
    // using kfifo_put and kfifo_get functions
    spin_lock_init(&global_device_state.queue_lock);

    // initialise tx and rx waiters queues
    // used if multiple users wait on the tx or rx queue at the same time
    init_waitqueue_head(&global_device_state.rx_wait_queue);
    init_waitqueue_head(&global_device_state.tx_wait_queue);

    // allocate the tx and rx queues
    global_device_state.rx_queue = kfifo_alloc(QUEUE_SIZE, GFP_KERNEL, &global_device_state.queue_lock);
    global_device_state.tx_queue = kfifo_alloc(QUEUE_SIZE, GFP_KERNEL, &global_device_state.queue_lock);
    if((global_device_state.tx_queue == NULL) || (global_device_state.rx_queue == NULL))
    {
        printk(KERN_ALERT "ERROR: failed to allocate kernel tx and rx queues\n");
        return -1;
    }
    
    // initialisation of the interrupt handler routine
    result = request_irq(IRQ_NUMBER, irq_handler, 0, MODULE_NAME, &global_device_state);
    if(result != 0)
    {
        printk(KERN_ALERT "ERROR: failed to register the IRQ handler\n");
        return -1;
    } 
   
    printk(KERN_ALERT "device initialised successfully\n");
    return 0;
}

// module cleanup function
static void serial_interrupts_exit(void)
{
    // unregister the irq
    free_irq(IRQ_NUMBER, &global_device_state);
    
    // free all used kernel memory queues
    kfifo_free(global_device_state.rx_queue);
    kfifo_free(global_device_state.tx_queue);
    
    // release uart memory region access
    release_region(SERIAL_PORT_BASE_ADDR, 1);

    // remove the charachter device
    cdev_del(&global_device_state.cdev);
    
    // un-register the device numbers
    unregister_chrdev_region(dev_num, MAX_NUMBER_OF_DEVICES);
    
    printk(KERN_ALERT "Good Bye, Cruel World!\n");
}

module_init(serial_interrupts_init);
module_exit(serial_interrupts_exit);

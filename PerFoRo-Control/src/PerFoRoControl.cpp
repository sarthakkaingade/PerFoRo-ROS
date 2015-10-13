#include "PerFoRoControl.h"

/**
 * PerFoRo Control
*/

PerFoRo_Control::PerFoRo_Control() :
	_n(),
	_mode_perforo_sub(_n.subscribe("/ModePerFoRo", 1000, &PerFoRo_Control::ModeCallback,this)),
	_navigate_perforo_sub(_n.subscribe("/NavigatePerFoRo", 1000, &PerFoRo_Control::NavigatePerFoRoCallback,this))
{
	
}	

void PerFoRo_Control::ModeCallback(const PerFoRoControl::MODE msg)
{
  ROS_INFO("PerFoRo Control: I heard Mode Command: [%d]", msg.MODE);
}

void PerFoRo_Control::NavigatePerFoRoCallback(const PerFoRoControl::NavigatePerFoRo msg)
{
  ROS_INFO("PerFoRo Control: I heard Navigate Command: [%d]", msg.command);
}

void PerFoRo_Control::init()
{
	ROS_INFO("Initialized PerFoRo Control Module");
}

int PerFoRo_Control::open_port(const char* port)
{
    int fd; /* File descriptor for the port */
    
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        /* Could not open the port. */
        return(-1);
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }
    
    return (fd);
}

bool PerFoRo_Control::setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    //struct termios options;
    
    struct termios  config;
    if(!isatty(fd))
    {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }
    if(tcgetattr(fd, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
        return false;
    }
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OPOST);

    #ifdef OLCUC 
        config.c_oflag &= ~OLCUC; 
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif

    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0
    
    // Get the current options for the port
    //tcgetattr(fd, &options);
    
    switch (baud)
    {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;
            
            break;
    }
    
    //
    // Finally, apply the configuration
    //
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }
    return true;
}

void PerFoRo_Control::close_port(int fd)
{
    close(fd);
}

void PerFoRo_Control::setup_serial()
{
	fd = open_port(uart_name0);
        if (fd == -1)
        {
            std::cout<<"\tFailed to open port "<<uart_name0<<std::endl;
        }
        else
        {
            PORTOPEN = true;
            uart_name = uart_name0;
        }
        
        if(!PORTOPEN)
        {
            fd = open_port(uart_name1);
            
            if(fd == -1)
            {
                std::cout<<"\tFailed to open port "<<uart_name1<<std::endl;
            }
            else
            {
                PORTOPEN = true;
                uart_name = uart_name1;
            }
        }
        
        if(!PORTOPEN)
        {
            exit(EXIT_FAILURE);
        }
        
        int baudrate = 38400;
        bool setup = setup_port(fd, baudrate, 8, 1, false, false);
        if (!setup)
        {
            exit(EXIT_FAILURE);
        }

        if (fd == -1 || fd == 0)
        {
            exit(EXIT_FAILURE);
        }
        else
        {
            fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
        }
        
        int noErrors = 0;
        if(fd < 0)
        {
            exit(noErrors);
        }
}

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "PerFoRo_Control");

	PerFoRo_Control PFRC;
	PFRC.init();
	PFRC.setup_serial();

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();
	
	return 0;
}

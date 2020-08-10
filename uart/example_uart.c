#include "ql_oe.h"

#include <stdio.h>
#include <termios.h>

static pthread_t thrd_uart_rcv1;
static pthread_t thrd_uart_rcv2;
static void* UartRecv1_Proc(void* arg);
static void* UartRecv2_Proc(void* arg);

#define UART_SERX_DATA_BIT_8					8
#define UART_SERX_EVENT_NONE					'N'
#define UART_SERX_STOP_BIT_1					1

int uart_setopt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

#define CONFIG_DEBUG

struct uart_dcb {
	int bits;
	char event;
	int stop;
	int baudrate;
};

struct uart_config {
	const char *name;
	int flag;
	int baudrate;
	struct uart_dcb dcb;
};

struct uart_prop {
//	struct uart_config *config;
};

struct {
	int uartfd1;
	int uartfd2;
} uart_inner;

int uart_init(struct uart_config config)
{
	int uartfd;
	int ret;

	uartfd = open(config.name, config.flag);

	uart_setopt(uartfd, config.dcb.baudrate, config.dcb.bits, config.dcb.event,
				config.dcb.stop);

	return uartfd;
}

int uart_config_read1(struct uart_config *config)
{
	config->name = "/dev/ttyHS0";
	config->baudrate = 115200;
	config->dcb.bits = UART_SERX_DATA_BIT_8;
	config->dcb.stop = UART_SERX_STOP_BIT_1;
	config->dcb.event = UART_SERX_EVENT_NONE;
	config->dcb.baudrate = B_115200;
}

int uart_config_read2(struct uart_config *config)
{
	config->name = "/dev/ttyGS0";
	config->baudrate = 115200;
	config->dcb.bits = UART_SERX_DATA_BIT_8;
	config->dcb.stop = UART_SERX_STOP_BIT_1;
	config->dcb.event = UART_SERX_EVENT_NONE;
	config->dcb.baudrate = B_115200;
}

int main(int argc, char* argv[])
{
	struct uart_config config;

	uart_inner.uartfd1 = -1;
	uart_inner.uartfd2 = -1;

	uart_config_read1(&config);
	uart_inner.uartfd1 = uart_init(config);
	uart_config_read2(&config);
	uart_inner.uartfd2 = uart_init(config);

	/*
	*  Create a thread to handle uart rx data
	*/
	if (pthread_create(&thrd_uart_rcv1, NULL, UartRecv1_Proc, NULL) != 0)
	{
		printf("Fail to create thread!\n");
	}
	if (pthread_create(&thrd_uart_rcv2, NULL, UartRecv2_Proc, NULL) != 0)
	{
		printf("Fail to create thread!\n");
	}

	do {
		sleep(10);
	} while(1);

	return 0;
}

static void* UartRecv1_Proc(void* arg)
{
	int iRet;
	fd_set fdset;
	struct timeval timeout = {3, 0};	// timeout 3s
	char buffer[4096] = {0};

	int fd_uart = uart_inner.uartfd1;

	while (uart_inner.uartfd1 >=0)
	{
		FD_ZERO(&fdset);
		FD_SET(fd_uart, &fdset);
		iRet = select(fd_uart + 1, &fdset, NULL, NULL, &timeout);
		if (-1 == iRet)
		{
			printf("< failed to select >\n");
			exit(-1);
		}
		else if (0 == iRet)
		{// no data in Rx buffer
			printf("< Uart1: no data >\n");
			timeout.tv_sec  = 3;
			timeout.tv_usec = 0;
		}
		else
		{// data is in Rx data
			if (FD_ISSET(fd_uart, &fdset))
			{
				do {
					memset(buffer, 0x0, sizeof(buffer));
					iRet = Ql_UART_Read(fd_uart, buffer, 4096);
#ifdef CONFIG_DEBUG
					printf("Recv: %s", buffer);
#endif
					Ql_UART_Write(uart_inner.uartfd2, buffer, iRet);
				} while (0);
			}
		}
	}
	return (void *)1;
}

static void* UartRecv2_Proc(void* arg)
{
	int iRet;
	fd_set fdset;
	struct timeval timeout = {3, 0};	// timeout 3s
	char buffer[4096] = {0};

	int fd_uart = uart_inner.uartfd2;

	while (uart_inner.uartfd2 >=0)
	{
		FD_ZERO(&fdset);
		FD_SET(fd_uart, &fdset);
		iRet = select(fd_uart + 1, &fdset, NULL, NULL, &timeout);
		if (-1 == iRet)
		{
			printf("< failed to select >\n");
			exit(-1);
		}
		else if (0 == iRet)
		{// no data in Rx buffer
			printf("< Uart2: no data >\n");
			timeout.tv_sec  = 3;
			timeout.tv_usec = 0;
		}
		else
		{// data is in Rx data
			if (FD_ISSET(fd_uart, &fdset))
			{
				do {
					memset(buffer, 0x0, sizeof(buffer));
					iRet = Ql_UART_Read(fd_uart, buffer, 4096);
#ifdef CONFIG_DEBUG
					printf("Recv: %s", buffer);
#endif
					Ql_UART_Write(uart_inner.uartfd1, buffer, iRet);

				} while (0);
			}
		}
	}
	return (void *)1;
}



int uart_setopt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)!=0) {
		return -1;
	}
	bzero(&newtio, sizeof( newtio ));
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;             //mask the character size bits

	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;              //data: 7bits
		break;
	case 8:
		newtio.c_cflag |= CS8;              //data: 8bits
		break;
	}

	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}
	switch( nSpeed )          //set the bps
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}

	if( nStop == 1 )                //set the 1bit stop
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )         //set the 2bit stop
		newtio.c_cflag |=  CSTOPB;
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIOFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		return -1;
	}
	return 0;
}

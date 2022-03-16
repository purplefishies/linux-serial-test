// SPDX-License-Identifier: MIT

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <linux/serial.h>
#include <errno.h>
#include <sys/file.h>

static void dump_serial_port_stats(void);
/*
 * glibc for MIPS has its own bits/termios.h which does not define
 * CMSPAR, so we vampirise the value from the generic bits/termios.h
 */
#ifndef CMSPAR
#define CMSPAR 010000000000
#endif

/*
 * Define modem line bits
 */
#ifndef TIOCM_LOOP
#define TIOCM_LOOP	0x8000
#endif

// command line args
typedef struct cmdopts_t
{
    int baud ;
    char* port ;
    int divisor ;
    int rx_dump ;
    int rx_dump_ascii ;
    int tx_detailed ;
    int stats ;
    int stop_on_error ;
    int single_byte ;
    int another_byte ;
    int rts_cts ;
    int two_stop_bit;
    int parity ;
    int odd_parity ;
    int stick_parity ;
    int loopback ;
    int dump_err ;
    int no_rx ;
    int no_tx ;
    int rx_delay ;
    int tx_delay ;
    int tx_bytes ;
    int tx_time ;
    int rx_time ;
    int ascii_range ;
    int write_after_read ;
    int count;
    int select;
    fd_set sensor_fds;
    int timeout;
} cmdopts_t ;


#define START_CLOCK(A)
#define SERIAL_PORT_PROCESS()

cmdopts_t clopts = (cmdopts_t){
    .baud = 0,
    .port = NULL,
    .divisor = 0,
    .rx_dump = 0,
    .rx_dump_ascii = 0,
    .tx_detailed = 0,
    .stats = 1,
    .stop_on_error = 0,
    .single_byte = -1,
    .another_byte = -1,
    .rts_cts = 0,
    .two_stop_bit = 0,
    .parity = 0,
    .odd_parity = 0,
    .stick_parity = 0,
    .loopback = 0,
    .dump_err = 0,
    .no_rx = 0,
    .no_tx = 0,
    .rx_delay = 0,
    .tx_delay = 0,
    .tx_bytes = 0,
    .tx_time = 0,
    .rx_time = 0,
    .ascii_range = 0,
    .write_after_read = 0,
    .select = 0,
    .timeout = 1000
};

typedef struct serial_stats_t
{
    // Module variables
    unsigned char _write_count_value ;
    unsigned char _read_count_value ;
    int _fd ;
    unsigned char* _write_data;
    ssize_t _write_size;
    // keep our own counts for cases where the driver stats don't work
    long long int _write_count ;
    long long int _read_count ;
    long long int _error_count ;
} serial_stats_t;

serial_stats_t stats = (serial_stats_t){
    ._write_count_value = 0,
    ._read_count_value = 0,
    ._fd = -1,
    ._write_data = 0,
    ._write_size = 0,
    // keep our own counts for cases where the driver stats don't work
    ._write_count = 0,
    ._read_count = 0,
    ._error_count = 0,  
};

typedef enum poll_type_t { 
    USE_POLL = 0,
    USE_SELECT = 1
}poll_type_t ;

typedef union pollselect_t {
    struct pollfd poll;
    fd_set select;
} pollselect_t;

typedef struct fdselect_t {
    pollselect_t ps;
    poll_type_t type;
}fdselect_t ;

fdselect_t fdselect;


typedef struct time_stats_t 
{
    struct timespec start_time;
    struct timespec last_stat;
    struct timespec last_timeout;
    struct timespec last_read;
    struct timespec last_write;
    struct timespec current;
} time_stats_t;


time_stats_t timing = (time_stats_t){0};

void exit_handler(void)
{
    clopts.no_rx = clopts.no_tx = 1;
    dump_serial_port_stats();
    if (stats._fd >= 0)
    {
        flock(stats._fd, LOCK_UN);
        close(stats._fd);
    }

    if (clopts.port)
    {
        free(clopts.port);
        clopts.port = NULL;
    }

    if (stats._write_data)
    {
        free(stats._write_data);
        stats._write_data = NULL;
    }
}

static void dump_data(unsigned char * b, int count)
{
	printf("%i bytes: ", count);
	int i;
	for (i=0; i < count; i++) {
		printf("%02x ", b[i]);
	}

	printf("\n");
}

static void dump_data_ascii(unsigned char * b, int count)
{
	int i;
	for (i=0; i < count; i++) {
		printf("%c", b[i]);
	}
}

static void set_baud_divisor(int speed, int custom_divisor)
{
	// default baud was not found, so try to set a custom divisor
	struct serial_struct ss;
	int ret;

	if (ioctl(stats._fd, TIOCGSERIAL, &ss) < 0) {
		ret = -errno;
		perror("TIOCGSERIAL failed");
		/* exit(ret); */
	}

	ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
	if (custom_divisor) {
		ss.custom_divisor = custom_divisor;
	} else {
		ss.custom_divisor = (ss.baud_base + (speed/2)) / speed;
		int closest_speed = ss.baud_base / ss.custom_divisor;

		if (closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100) {
			fprintf(stderr, "Cannot set speed to %d, closest is %d\n", speed, closest_speed);
			exit(-EINVAL);
		}

		printf("closest baud = %i, base = %i, divisor = %i\n", closest_speed, ss.baud_base,
				ss.custom_divisor);
	}

	if (ioctl(stats._fd, TIOCSSERIAL, &ss) < 0) {
		ret = -errno;
		perror("TIOCSSERIAL failed");
		exit(ret);
	}
}

static void clear_custom_speed_flag()
{
	struct serial_struct ss;
	int ret;

	if (ioctl(stats._fd, TIOCGSERIAL, &ss) < 0) {
		// return silently as some devices do not support TIOCGSERIAL
		return;
	}

	if ((ss.flags & ASYNC_SPD_MASK) != ASYNC_SPD_CUST)
		return;

	ss.flags &= ~ASYNC_SPD_MASK;

	if (ioctl(stats._fd, TIOCSSERIAL, &ss) < 0) {
		ret = -errno;
		perror("TIOCSSERIAL failed");
		exit(ret);
	}
}

// converts integer baud to Linux define
static int get_baud(int baud)
{
	switch (baud) {
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
#ifdef B1000000
	case 1000000:
		return B1000000;
#endif
#ifdef B1152000
	case 1152000:
		return B1152000;
#endif
#ifdef B1500000
	case 1500000:
		return B1500000;
#endif
#ifdef B2000000
	case 2000000:
		return B2000000;
#endif
#ifdef B2500000
	case 2500000:
		return B2500000;
#endif
#ifdef B3000000
	case 3000000:
		return B3000000;
#endif
#ifdef B3500000
	case 3500000:
		return B3500000;
#endif
#ifdef B4000000
	case 4000000:
		return B4000000;
#endif
	default:
		return -1;
	}
}

void set_modem_lines(int fd, int bits, int mask)
{
	int status, ret;
	/* if (ioctl(fd, TIOCMGET, &status) < 0) { */
	/* 	ret = -errno; */
	/* 	perror("TIOCMGET failed"); */
	/* 	/\* exit(ret); *\/ */
	/* } */

	/* status = (status & ~mask) | (bits & mask); */

	/* if (ioctl(fd, TIOCMSET, &status) < 0) { */
	/* 	ret = -errno; */
	/* 	perror("TIOCMSET failed"); */
	/* 	/\* exit(ret); *\/ */
	/* } */
}

static void display_help(void)
{
    printf("Usage: linux-serial-test [OPTION]\n"
           "\n"
           "  -h, --help\n"
           "  -b, --baud         Baud rate, 115200, etc (115200 is default)\n"
           "  -p, --port         Port (/dev/ttyS0, etc) (must be specified)\n"
           "  -d, --divisor      UART Baud rate divisor (can be used to set custom baud rates)\n"
           "  -R, --rx_dump      Dump Rx data (ascii, raw)\n"
           /* "  -T, --detailed_tx  Detailed Tx data\n" */
           /* "  -s, --stats        Dump serial port stats every 5s\n" */
           /* "  -S, --stop-on-err  Stop program if we encounter an error\n" */
           /* "  -y, --single-byte  Send specified byte to the serial port\n" */
           /* "  -z, --second-byte  Send another specified byte to the serial port\n" */
           /* "  -c, --rts-cts      Enable RTS/CTS flow control\n" */
           /* "  -B, --2-stop-bit   Use two stop bits per character\n" */
           /* "  -P, --parity       Use parity bit (odd, even, mark, space)\n" */
           "  -C, --count        Stop after this many bytes has been written\n"
           "  -k, --loopback     Use internal hardware loop back\n"
           "  -K, --write-follow Write follows the read count (can be used for multi-serial loopback)\n"
           "  -e, --dump-err     Display errors\n"
           "  -r, --no-rx        Don't receive data (can be used to test flow control)\n"
           "                     when serial driver buffer is full\n"
           "  -t, --no-tx        Don't transmit data\n"
           "  -l, --rx-delay     Delay between reading data (ms) (can be used to test flow control)\n"
           "  -a, --tx-delay     Delay between writing data (ms)\n"
           "  -w, --tx-bytes     Number of bytes for each write (default is to repeatedly write 1024 bytes\n"
           "                     until no more are accepted)\n"
           "  -o, --tx-time      Number of seconds to transmit for (defaults to 0, meaning no limit)\n"
           "  -i, --rx-time      Number of seconds to receive for (defaults to 0, meaning no limit)\n"
           "  -A, --ascii        Output bytes range from 32 to 126 (default is 0 to 255)\n"
           "\n"
           );
}

static void process_command_line(int argc, char * argv[])
{
    for (;;)
    {
        int option_index = 0;
        static const char* short_options = "hb:p:d:R:TsSy:z:cBertq:Ql:a:w:o:i:P:kKA";
        // clang-format off
        static const struct option long_options[] = {
            { "help"            , no_argument         , 0   , 0 }     ,
            { "baud"            , required_argument   , 0   , 'b' }   ,
            { "port"            , required_argument   , 0   , 'p' }   ,
            { "divisor"         , required_argument   , 0   , 'd' }   ,
            { "rx_dump"         , required_argument   , 0   , 'R' }   ,
            { "select"          , no_argument         , 0   , 'S' }   ,
            { "timeout"         , required_argument   , 0   , 'T' }   ,
            /* { "detailed_tx"     , no_argument         , 0   , 'T' }   , */
            /* { "stats"           , no_argument         , 0   , 's' }   , */
            /* { "stop-on-err"     , no_argument         , 0   , 'S' }   , */
            /* { "single-byte"     , no_argument         , 0   , 'y' }   , */
            /* { "second-byte"     , no_argument         , 0   , 'z' }   , */
            /* { "rts-cts"         , no_argument         , 0   , 'c' }   , */
            /* { "2-stop-bit"      , no_argument         , 0   , 'B' }   , */
            { "count"           , required_argument   , 0   , 'C' }   ,
            { "parity"          , required_argument   , 0   , 'P' }   ,
            { "loopback"        , no_argument         , 0   , 'k' }   ,
            { "write-follows"   , no_argument         , 0   , 'K' }   ,
            { "dump-err"        , no_argument         , 0   , 'e' }   ,
            { "no-rx"           , no_argument         , 0   , 'r' }   ,
            { "no-tx"           , no_argument         , 0   , 't' }   ,
            { "rx-delay"        , required_argument   , 0   , 'l' }   ,
            { "tx-delay"        , required_argument   , 0   , 'a' }   ,
            { "tx-bytes"        , required_argument   , 0   , 'w' }   ,
            { "tx-time"         , required_argument   , 0   , 'o' }   ,
            { "rx-time"         , required_argument   , 0   , 'i' }   ,
            { "ascii"           , no_argument         , 0   , 'A' }   ,
            { 0                 , 0                   , 0   ,  0  }   ,
        };
        // clang-format on
        int c = getopt_long(argc, argv, short_options, long_options, &option_index);

        if (c == EOF)
        {
            break;
        }

        switch (c)
        {
            case 0:
            case 'h':
                display_help();
                exit(0);
                break;
            case 'b':
                clopts.baud = atoi(optarg);
                break;
            case 'p':
                clopts.port = strdup(optarg);
                break;
            case 'd':
                clopts.divisor = strtol(optarg, NULL, 0);
                break;
            case 'R':
                clopts.rx_dump = 1;
                clopts.rx_dump_ascii = !strcmp(optarg, "ascii");
                break;
            case 'S':
                clopts.select = 1;
                break;

           /* "  -T, --detailed_tx  Detailed Tx data\n" */
           /* "  -s, --stats        Dump serial port stats every 5s\n" */
           /* "  -S, --stop-on-err  Stop program if we encounter an error\n" */
           /* "  -y, --single-byte  Send specified byte to the serial port\n" */
           /* "  -z, --second-byte  Send another specified byte to the serial port\n" */
           /* "  -c, --rts-cts      Enable RTS/CTS flow control\n" */
           /* "  -B, --2-stop-bit   Use two stop bits per character\n" */
           /* "  -P, --parity       Use parity bit (odd, even, mark, space)\n" */

            case 'T':
                clopts.timeout = atoi(optarg);
                break;
            /* case 's': */
            /*     clopts.stats = 1; */
            /*     break; */
            /* case 'S': */
            /*     clopts.stop_on_error = 1; */
            /*     break; */
            /* case 'y': */
            /* { */
            /*     char* endptr; */
            /*     clopts.single_byte = strtol(optarg, &endptr, 0); */
            /*     break; */
            /* } */
            /* case 'z': */
            /* { */
            /*     char* endptr; */
            /*     clopts.another_byte = strtol(optarg, &endptr, 0); */
            /*     break; */
            /* } */
            /* case 'c': */
            /*     clopts.rts_cts = 1; */
            /*     break; */
            /* case 'B': */
            /*     clopts .two_stop_bit = 1; */
            /*     break; */
            case 'C':
                clopts.count = atoi(optarg);
                break;
            case 'P':
                clopts.parity = 1;
                clopts.odd_parity = (!strcmp(optarg, "mark") || !strcmp(optarg, "odd"));
                clopts.stick_parity = (!strcmp(optarg, "mark") || !strcmp(optarg, "space"));
                break;
            case 'k':
                clopts.loopback = 1;
                break;
            case 'K':
                clopts.write_after_read = 1;
                break;
            case 'e':
                clopts.dump_err = 1;
                break;
            case 'r':
                clopts.no_rx = 1;
                break;
            case 't':
                clopts.no_tx = 1;
                break;
            case 'l':
            {
                char* endptr;
                clopts.rx_delay = strtol(optarg, &endptr, 0);
                break;
            }
            case 'a':
            {
                char* endptr;
                clopts.tx_delay = strtol(optarg, &endptr, 0);
                break;
            }
            case 'w':
            {
                char* endptr;
                clopts.tx_bytes = strtol(optarg, &endptr, 0);
                break;
            }
            case 'o':
            {
                char* endptr;
                clopts.tx_time = strtol(optarg, &endptr, 0);
                break;
            }
            case 'i':
            {
                char* endptr;
                clopts.rx_time = strtol(optarg, &endptr, 0);
                break;
            }
            case 'A':
                clopts.ascii_range = 1;
                break;
        }


    }

    if (clopts.baud && !clopts.divisor)
        clopts.baud = get_baud(clopts.baud);
    else
        clopts.baud = B115200;

    if (!clopts.port) {
        fprintf(stderr, "ERROR: Port argument required\n");
        display_help();
        exit(-EINVAL);
    }
    
    if (clopts.baud <= 0 || clopts.divisor) {
        printf("NOTE: non standard baud rate, trying custom divisor\n");
        clopts.baud = B115200;
        /* set_baud_divisor(clopts.baud, clopts.divisor); */
    }

}

static void dump_serial_port_stats(void)
{
    struct serial_icounter_struct icount = { 0 };

    printf("%s: count for this session: rx=%lld, tx=%lld, rx err=%lld\n", clopts.port, stats._read_count,
           stats._write_count, stats._error_count);

}

static unsigned char next_count_value(unsigned char c)
{
	c++;
	if (clopts.ascii_range && c == 127)
		c = 32;
	return c;
}

static void process_read_data(void)
{
	unsigned char rb[1024];
	int c = read(stats._fd, &rb, sizeof(rb));
	if (c > 0) {
		if (clopts.rx_dump) {
			if (clopts.rx_dump_ascii)
				dump_data_ascii(rb, c);
			else
				dump_data(rb, c);
		}

		// verify read count is incrementing
		int i;
		for (i = 0; i < c; i++) {
			if (rb[i] != stats._read_count_value) {
				if (clopts.dump_err) {
					printf("Error, count: %lld, expected %02x, got %02x\n",
							stats._read_count + i, stats._read_count_value, rb[i]);
				}
				stats._error_count++;
				if (clopts.stop_on_error) {
					dump_serial_port_stats();
					exit(-EIO);
				}
				stats._read_count_value = rb[i];
			}
			stats._read_count_value = next_count_value(stats._read_count_value);
		}
		stats._read_count += c;
	}
}



void process_write_data(void)
{
	ssize_t count = 0;
	ssize_t actual_write_size = 0;
	int repeat = (clopts.tx_bytes == 0);

	do
	{
		if (clopts.write_after_read == 0) {
			actual_write_size = stats._write_size;
		} else {
			actual_write_size = stats._read_count > stats._write_count ? stats._read_count - stats._write_count : 0;
			if (actual_write_size > stats._write_size) {
				actual_write_size = stats._write_size;
			}
		}
		if (actual_write_size == 0) {
			break;
		}

		ssize_t i;
		for (i = 0; i < actual_write_size; i++) {
			stats._write_data[i] = stats._write_count_value;
			stats._write_count_value = next_count_value(stats._write_count_value);
		}
                if ( clopts.no_tx )
                    break;
                
		ssize_t c = write(stats._fd, stats._write_data, actual_write_size);

		if (c < 0) {
			if (errno != EAGAIN) {
				printf("write failed - errno=%d (%s)\n", errno, strerror(errno));
			}
			c = 0;
		}

		count += c;

		if (c < actual_write_size) {
			stats._write_count_value = stats._write_data[c];
			repeat = 0;
		} else if ( c == actual_write_size ) {
                    repeat = 0;
                }
	} while (repeat);

	stats._write_count += count;

        if ( clopts.count > 0 && stats._write_count > clopts.count ) 
            clopts.no_tx = 1;

	if (clopts.tx_detailed)
		printf("wrote %zd bytes\n", count);
}


static void setup_serial_port()
{
	struct termios newtio;
	struct serial_rs485 rs485;
	int ret;

	stats._fd = open(clopts.port, O_RDWR | O_NONBLOCK);

	if (stats._fd < 0) {
		ret = -errno;
		perror("Error opening serial port");
		exit(ret);
	}

	/* Lock device file */
	if (flock(stats._fd, LOCK_EX | LOCK_NB) < 0) {
		ret = -errno;
		perror("Error failed to lock device file");
		exit(ret);
	}

	memset(&newtio, 0, sizeof(newtio)); /* clear struct for new port settings */

	/* man termios get more info on below settings */
	newtio.c_cflag = clopts.baud | CS8 | CLOCAL | CREAD;

	if (clopts.rts_cts) {
		newtio.c_cflag |= CRTSCTS;
	}

	if (clopts.two_stop_bit) {
		newtio.c_cflag |= CSTOPB;
	}

	if (clopts.parity) {
		newtio.c_cflag |= PARENB;
		if (clopts.odd_parity) {
			newtio.c_cflag |= PARODD;
		}
		if (clopts.stick_parity) {
			newtio.c_cflag |= CMSPAR;
		}
	}

	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	// block for up till 128 characters
	newtio.c_cc[VMIN] = 128;

	// 0.5 seconds read timeout
	newtio.c_cc[VTIME] = 5;

	/* now clean the modem line and activate the settings for the port */
	tcflush(stats._fd, TCIOFLUSH);
	tcsetattr(stats._fd,TCSANOW,&newtio);

	/* enable/disable rs485 direction control, first check if RS485 is supported */
	/* if(ioctl(stats._fd, TIOCGRS485, &rs485) < 0) { */
	/* 	if (clopts.rs485_after_delay >= 0) { */
	/* 		/\* error could be because hardware is missing rs485 support so only print when actually trying to activate it *\/ */
	/* 		perror("Error getting RS-485 mode"); */
	/* 	} */
	/* } else { */
	/* 	if (clopts.rs485_after_delay >= 0) { */
	/* 		/\* enable RS485 *\/ */
	/* 		rs485.flags |= SER_RS485_ENABLED | SER_RS485_RX_DURING_TX | */
	/* 			(clopts.rs485_rts_after_send ? SER_RS485_RTS_AFTER_SEND : SER_RS485_RTS_ON_SEND); */
	/* 		rs485.flags &= ~(clopts.rs485_rts_after_send ? SER_RS485_RTS_ON_SEND : SER_RS485_RTS_AFTER_SEND); */
	/* 		rs485.delay_rts_after_send = clopts.rs485_after_delay; */
	/* 		rs485.delay_rts_before_send = clopts.rs485_before_delay; */
	/* 		if(ioctl(stats._fd, TIOCSRS485, &rs485) < 0) { */
	/* 			perror("Error setting RS-485 mode"); */
	/* 		} */
	/* 	} else { */
	/* 		/\* disable RS485 *\/ */
	/* 		rs485.flags &= ~(SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND); */
	/* 		rs485.delay_rts_after_send = 0; */
	/* 		rs485.delay_rts_before_send = 0; */
	/* 		if(ioctl(stats._fd, TIOCSRS485, &rs485) < 0) { */
	/* 			perror("Error setting RS-232 mode"); */
	/* 		} */
	/* 	} */
	/* } */
}

static int diff_ms(const struct timespec *t1, const struct timespec *t2)
{
	struct timespec diff;

	diff.tv_sec = t1->tv_sec - t2->tv_sec;
	diff.tv_nsec = t1->tv_nsec - t2->tv_nsec;
	if (diff.tv_nsec < 0) {
		diff.tv_sec--;
		diff.tv_nsec += 1000000000;
	}
	return (diff.tv_sec * 1000 + diff.tv_nsec/1000000);
}

static int compute_error_count(void)
{
	long long int result;
	if (clopts.no_rx == 1 || clopts.no_tx == 1)
		result = stats._error_count;
	else
		result = llabs(stats._write_count - stats._read_count) + stats._error_count;

	return (result > 125) ? 125 : (int)result;
}

void initialize_stats()
{
    set_modem_lines(stats._fd, clopts.loopback ? TIOCM_LOOP : 0, TIOCM_LOOP);
    stats._write_size = (clopts.tx_bytes == 0) ? 1024 : clopts.tx_bytes;
    stats._write_data = malloc(stats._write_size);
    if (stats._write_data == NULL)
    {
        fprintf(stderr, "ERROR: Memory allocation failed\n");
        exit(-ENOMEM);
    }

    if (clopts.ascii_range)
    {
        stats._read_count_value = stats._write_count_value = 32;
    }

    stats._write_size = (clopts.tx_bytes == 0) ? 1024 : clopts.tx_bytes;
    stats._write_data = malloc(stats._write_size);
    if (stats._write_data == NULL) {
        fprintf(stderr, "ERROR: Memory allocation failed\n");
        exit(-ENOMEM);
    }
    if (clopts.ascii_range) {
		stats._read_count_value = stats._write_count_value = 32;
    }
}

/* void process_results(int retval, 	struct pollfd *serial_poll) */
void process_results(int retval, 	fdselect_t *fds)
{
    if (retval == -1)
    {
        if ( fds->type == USE_POLL ) { 
            perror("poll()");
        } else {
            perror("select()");
        }
    }
    else if (retval)
    {
        if( fds->type == USE_POLL ) 
        {
            if (fds->ps.poll.revents & POLLIN)
            {
                if (clopts.rx_delay)
                {
                    // only read if it has been rx-delay ms
                    // since the last read
                    if (diff_ms(&timing.current, &timing.last_read) > clopts.rx_delay)
                    {
                        process_read_data();
                        timing.last_read = timing.current;
                    }
                }
                else
                {
                    process_read_data();
                    timing.last_read = timing.current;
                }
            }

            if (fds->ps.poll.revents & POLLOUT)
            {
                if (clopts.tx_delay)
                {
                    // only write if it has been tx-delay ms
                    // since the last write
                    if (diff_ms(&timing.current, &timing.last_write) > clopts.tx_delay)
                    {
                        process_write_data();
                        timing.last_write = timing.current;
                    }
                }
                else
                {
                    process_write_data();
                    timing.last_write = timing.current;
                }
            }
        } 
        else {
            if (clopts.rx_delay)
            {
                // only read if it has been rx-delay ms
                // since the last read
                if (diff_ms(&timing.current, &timing.last_read) > clopts.rx_delay)
                {
                    process_read_data();
                    timing.last_read = timing.current;
                }
            }
            else
            {
                process_read_data();
                timing.last_read = timing.current;
            }
        }
    }
}

void calculate_timing(struct time_stats_t *timing)
{
    if (diff_ms(&timing->current, &timing->last_timeout) > 1000)
    {
        int rx_timeout, tx_timeout;

        // Has it been over two seconds since we transmitted or received data?
        rx_timeout = (!clopts.no_rx && diff_ms(&timing->current, &timing->last_read) > 2000);
        tx_timeout = (!clopts.no_tx && diff_ms(&timing->current, &timing->last_write) > 2000);
        // Special case - we don't want to warn about receive
        // timeouts at the end of a loopback test (where we are
        // no longer transmitting and the receive count equals
        // the transmit count).
        if (clopts.no_tx && stats._write_count != 0 && stats._write_count == stats._read_count)
        {
            rx_timeout = 0;
        }
        if (rx_timeout || tx_timeout)
        {
            const char* s;
            if (rx_timeout)
            {
                printf("%s: No data received for %.1fs.", clopts.port,
                       (double)diff_ms(&timing->current, &timing->last_read) / 1000);
                s = " ";
            }
            else
            {
                s = "";
            }
            if (tx_timeout)
            {
                printf("%sNo data transmitted for %.1fs.", s, (double)diff_ms(&timing->current, &timing->last_write) / 1000);
            }
            printf("\n");
            timing->last_timeout = timing->current;
        }
    }

    if (clopts.stats)
    {
        if (timing->current.tv_sec - timing->last_stat.tv_sec > 5)
        {
            dump_serial_port_stats();
            timing->last_stat = timing->current;
        }
    }
}


void start_timing(struct time_stats_t *timing)
{
    clock_gettime(CLOCK_MONOTONIC, &timing->start_time);
    timing->last_stat      = timing->start_time;
    timing->last_timeout   = timing->start_time;
    timing->last_read      = timing->start_time;
    timing->last_write     = timing->start_time;
}

void show_stats(fdselect_t *fds)
{
    if (fds->type == USE_POLL ) {
        if (clopts.tx_time)
        {
            if (timing.current.tv_sec - timing.start_time.tv_sec >= clopts.tx_time)
            {
                clopts.tx_time = 0;
                clopts.no_tx = 1;
                fds->ps.poll.events &= ~POLLOUT;
                printf("Stopped transmitting.\n");
            }
        }
        if (clopts.rx_time)
        {
            if (timing.current.tv_sec - timing.start_time.tv_sec >= clopts.rx_time)
            {
                clopts.rx_time = 0;
                clopts.no_rx = 1;
                fds->ps.poll.events &= ~POLLIN;
                printf("Stopped receiving.\n");
            }
        }
    } else {
        

    }
}
/* void * */
/* sensor_event_loop(__attribute__((unused)) void *arg) */
/* { */
/*     fd_set sensor_fds; */
/*     FD_ZERO(&sensor_fds); */
/*     /\* for (index = 0; index < sensor_ctl.count; index++) *\/ */
/*     /\* { *\/ */
/*     /\*     if (sensor_ctl.sensor[index].sensor_fd > 0) *\/ */
/*     /\*         FD_SET(sensor_ctl.sensor[index].sensor_fd, &sensor_fds); *\/ */
/*     /\* } *\/ */
/*     int fd; */
    
/*     FD_SET( fd, &sensor_fds ); */

/*     /\* Waiting. *\/ */
/*     /\* ready = select(sensor_ctl.max_fd + 1, &sensor_fds, NULL, NULL, NULL); *\/ */
/*     /\* if ((ready == -1) && (errno == EINTR)) *\/ */
/*     /\* { *\/ */
/*     /\*     /\\* Someone rudely interrupted us. Please continue. *\\/ *\/ */
/*     /\*     continue; *\/ */
/*     /\* } *\/ */
/*     /\* /\\* Update packet count for SubView logging. *\\/ *\/ */
/*     /\* packet_count++; *\/ */
/*     /\* for (index = 0; index < sensor_ctl.count; index++) *\/ */
/*     /\* { *\/ */
/*     /\*     if (FD_ISSET(sensor_ctl.sensor[index].sensor_fd, &sensor_fds)) *\/ */
/*     /\*     { *\/ */
/*     /\*         (*sensor_ctl.sensor[index].handle_data)( *\/ */
/*     /\*                                                 sensor_ctl.sensor[index].priv_state, *\/ */
/*     /\*                                                 packet_count); *\/ */
/*     /\*     } *\/ */
/*     /\* } *\/ */
/* } */

void handler(int sig)
{
    clopts.no_rx = 1;
    clopts.no_tx = 1;
    return ;
}

void setup_fdselect( fdselect_t *fs )
{
    fs->type = ( clopts.select ? USE_SELECT : USE_POLL );
    if ( fs->type == USE_POLL ) {
        fs->ps.poll.fd = stats._fd;
	if (!clopts.no_rx) {
		fs->ps.poll.events |= POLLIN;
	} else {
		fs->ps.poll.events &= ~POLLIN;
	}
	if (!clopts.no_tx) {
		fs->ps.poll.events |= POLLOUT;
	} else {
		fs->ps.poll.events &= ~POLLOUT;
	}
    } else {
        FD_ZERO(&fs->ps.select );
        FD_SET( stats._fd, &fs->ps.select );
    }
}

int wait_for_events( fdselect_t *fs , int timeout )
{
    int retval;
    if ( fs->type == USE_POLL ) {
	 retval = poll(&fs->ps.poll, 1, timeout);
    } else {
        struct timeval to = (struct timeval){.tv_sec = (timeout/1000 <= 0 ? 1 : timeout/1000)  , .tv_usec=0};
        FD_ZERO(&fs->ps.select);
        FD_SET(stats._fd,&fs->ps.select);
        retval = select(FD_SETSIZE,&fs->ps.select, NULL, NULL, &to );
    }
    return retval;
}


int main(int argc, char * argv[])
{
	printf("Linux serial test app\n");

	atexit(&exit_handler);
        signal(SIGINT, &handler);
	process_command_line(argc, argv);

        setup_serial_port(clopts.baud);
        if (clopts.divisor) 
            clear_custom_speed_flag();

        initialize_stats();
	set_modem_lines(stats._fd, clopts.loopback ? TIOCM_LOOP : 0, TIOCM_LOOP);
        setup_fdselect( &fdselect ) ;
        start_timing( &timing );

	while (!(clopts.no_rx && clopts.no_tx)) {
            int retval;
            if ( clopts.no_tx ) { /* Receiver */

                retval = wait_for_events( &fdselect, clopts.timeout );
		clock_gettime(CLOCK_MONOTONIC, &timing.current);
                process_results( retval, &fdselect );

            } else if( clopts.no_rx ) { /* Transmitter */

                retval = wait_for_events( &fdselect , clopts.timeout );
                process_results( retval, &fdselect );

            }
            // Has it been at least a second since we reported a timeout?
            calculate_timing(&timing);
            show_stats(&fdselect);
	}

	tcdrain(stats._fd);
	dump_serial_port_stats();
	set_modem_lines(stats._fd, 0, TIOCM_LOOP);
	tcflush(stats._fd, TCIOFLUSH);

	return compute_error_count();
}

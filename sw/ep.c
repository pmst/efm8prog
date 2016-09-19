#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>


#if defined(__linux__)
#include <termios.h>
#else
#include <windows.h>
#endif

void setCPUtype(char* cpu);
int parse_hex (char * filename, unsigned char * progmem);

char* COM = "";
//char* COM = "/dev/ttyS0";


#define	PROGMEM_LEN	260000
unsigned char progmem[PROGMEM_LEN];

int com;

int baudRate;
int verbose = 1;
int verify = 1;
int program = 1;
int flash_size = 8192;
int page_size = 64;
int sleep_time = 0;


void comErr(char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(stderr,"%s", buf);
	perror(COM);
	va_end(va);
	abort(); 
	}

void flsprintf(FILE* f, char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(f,"%s", buf);
	fflush(f);
	va_end(va);
	}
	
	
#if defined(__linux__)
	
void initSerialPort() {
	baudRate=B115200;
	if (verbose>2)
		printf("Opening: %s at %d\n",COM,baudRate);
	com =  open(COM, O_RDWR | O_NOCTTY | O_NDELAY);
	if (com <0) 
		comErr("Failed to open serial port");

	struct termios opts;
	memset (&opts,0,sizeof (opts));	
	
	fcntl(com, F_SETFL, 0);	

	if (tcgetattr(com, &opts)!=0)
		{
		printf("Err tcgetattr\n");
		}

	cfsetispeed(&opts, baudRate);   
	cfsetospeed(&opts, baudRate);  

	opts.c_lflag  &=  ~(ICANON | ECHO | ECHOE | ISIG);

	opts.c_cflag |=  (CLOCAL | CREAD);
	opts.c_cflag &=  ~PARENB;
	opts.c_cflag &= ~CSTOPB;
	opts.c_cflag &=  ~CSIZE;
	opts.c_cflag |=  CS8;
	
	opts.c_oflag &=  ~OPOST;
	
	opts.c_iflag &=  ~INPCK;
	opts.c_iflag &=  ~ICRNL;		//do NOT translate CR to NL
	opts.c_iflag &=  ~(IXON | IXOFF | IXANY);	
	opts.c_cc[ VMIN ] = 0;
	opts.c_cc[ VTIME ] = 100; //1 sec
	


	if (tcsetattr(com, TCSANOW, &opts) != 0) {
		perror(COM); 
		printf("set attr error");
		abort(); 
		}
		
	tcflush(com,TCIOFLUSH); // just in case some crap is the buffers
	/*	
	char buf = -2;
	while (read(com, &buf, 1)>0) {
		if (verbose)
			printf("Unexpected data from serial port: %02X\n",buf & 0xFF);
		}
*/
	}

	
	void putByte(int byte) {
	char buf = byte;
	if (verbose>3)
		flsprintf(stdout,"TX: 0x%02X\n", byte);
	int n = write(com, &buf, 1);
	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
	}
	
int getByte() {
	char buf;
	int n = read(com, &buf, 1);
	if (verbose>3)
		flsprintf(stdout,n<1?"RX: fail\n":"RX:  0x%02X\n", buf & 0xFF);
	if (n == 1)
		return buf & 0xFF;
	
	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
	}
#else

HANDLE port_handle;
	
void initSerialPort() 
{

char mode[40],portname[20];
COMMTIMEOUTS timeout_sets;
DCB port_sets;
strcpy(portname,"\\\\.\\");
strcat(portname,COM);
  port_handle = CreateFileA(portname,
                      GENERIC_READ|GENERIC_WRITE,
                      0,                          /* no share  */
                      NULL,                       /* no security */
                      OPEN_EXISTING,
                      0,                          /* no threads */
                      NULL);                      /* no templates */
  if(port_handle==INVALID_HANDLE_VALUE)
  {
    printf("unable to open port %s -> %s\n",COM, portname);
    exit(0);
  }
  strcpy (mode,"baud=57600 data=8 parity=n stop=1");
  memset(&port_sets, 0, sizeof(port_sets));  /* clear the new struct  */
  port_sets.DCBlength = sizeof(port_sets);
  
  if(!BuildCommDCBA(mode, &port_sets))
  {
	printf("dcb settings failed\n");
	CloseHandle(port_handle);
	exit(0);
  }

  if(!SetCommState(port_handle, &port_sets))
  {
    printf("cfg settings failed\n");
    CloseHandle(port_handle);
    exit(0);
  }


  timeout_sets.ReadIntervalTimeout         = 1;
  timeout_sets.ReadTotalTimeoutMultiplier  = 100;
  timeout_sets.ReadTotalTimeoutConstant    = 1;
  timeout_sets.WriteTotalTimeoutMultiplier = 100;
  timeout_sets.WriteTotalTimeoutConstant   = 1;

  if(!SetCommTimeouts(port_handle, &timeout_sets))
  {
    printf("timeout settings failed\n");
    CloseHandle(port_handle);
    exit(0);
  }
  
  
}
void putByte(int byte) 
{
  int n;
  	if (verbose>3)
		flsprintf(stdout,"TX: 0x%02X\n", byte);
  WriteFile(port_handle, &byte, 1, (LPDWORD)((void *)&n), NULL);
  	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
}
	
int getByte() 
{
unsigned char buf[2];
int n;
ReadFile(port_handle, buf, 1, (LPDWORD)((void *)&n), NULL);
	if (verbose>3)
		flsprintf(stdout,n<1?"RX: fail\n":"RX:  0x%02X\n", buf[0] & 0xFF);	
	if (n == 1)
		return buf[0] & 0xFF;
	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
	}
#endif

	

void sleep_ms (int num)
{
	struct timespec tspec;
	tspec.tv_sec=num/1000;
	tspec.tv_nsec=(num%1000)*1000000; 
	nanosleep(&tspec,0);
}
	
		
/*	
int getIntArg(char* arg) {
	if (strlen(arg)>=2 && memcmp(arg,"0x",2)==0) {
		unsigned int u;
		sscanf(arg+2,"%X",&u);
		return u;
		}
	else {
		int d;
		sscanf(arg,"%d",&d);
		return d;
		}
	}
	*/

void printHelp() {
		flsprintf(stdout,"pp programmer\n");
	exit(0);
	}
	

void parseArgs(int argc, char *argv[]) {	
	int c;
	while ((c = getopt (argc, argv, "c:nps:t:v:")) != -1) {
		switch (c) {
			case 'c' : 
				COM=optarg;
				break;
			case 'n':
		    	verify = 0;
			    break;
			case 'p':
		    	program = 0;
			    break;
			case 's' : 
				sscanf(optarg,"%d",&sleep_time);
				break;
			case 't' :
				setCPUtype(optarg);
				break;
			case 'v' :
				sscanf(optarg,"%d",&verbose);
				break;
			case '?' :
				if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr,"Unknown option character `\\x%x'.\n",optopt);
			  default:
				fprintf (stderr,"Bug, unhandled option '%c'\n",c);
				abort ();
			}
		}
	if (argc<=1) 
		printHelp();
	}


int prog_enter_progmode (void)
	{
	if (verbose>2)
		flsprintf(stdout,"Entering programming mode\n");
	putByte(0x01);
	putByte(0x00);
	getByte();
	return 0;
	}

int prog_exit_progmode (void)
	{
	if (verbose>2)
		flsprintf(stdout,"Exiting programming mode\n");
	putByte(0x02);
	putByte(0x00);
	getByte();
	return 0;
	}

int c2_mass_erase (void)
	{
	if (verbose>2)
		flsprintf(stdout,"Mass erase\n");

	putByte(0x04);
	putByte(0x00);
	getByte();
	return 0;
	}

int c2_write_page (unsigned char * data, int address, unsigned char num)
{
unsigned char i,empty;
empty = 0;
for (i=0;i<num;i++)
{
	if (data[i]!=0xFF) empty = 0;
}
if (empty==1)	
{
if (verbose>3)
		flsprintf(stdout,"~");
	return 0;
}
if (verbose>2)
	flsprintf(stdout,"Writing page of %d bytes at 0x%4.4x\n", num, address);
putByte(0x03);
putByte(4+num);
putByte(num);
putByte((address>>16)&0xFF);
putByte((address>>8)&0xFF);
putByte((address>>0)&0xFF);
for (i=0;i<num;i++)
		putByte(data[i]);
getByte();
return 0;
}

int c2_read_page (unsigned char * data, int address, unsigned char num)
{
unsigned char i;
	if (verbose>2)
		flsprintf(stdout,"Reading page of %d bytes at 0x%4.4x\n", num, address);
putByte(0x5);
putByte(0x04);
putByte(num);
putByte((address>>16)&0xFF);
putByte((address>>8)&0xFF);
putByte((address>>0)&0xFF);
getByte();
for (i=0;i<num;i++)
	{
	*data++ = getByte();
	}
return 0;
}

int c2_write_byte (unsigned char address, unsigned char data)
{
	if (verbose>2)
		flsprintf(stdout,"Writing byte %d bytes at 0x%2.2x\n", data, address);
putByte(0x6);
putByte(0x02);
putByte(address);
putByte(data);
getByte();
return 0;
}

int c2_read_byte (unsigned char address)
{
unsigned char i;
	if (verbose>2)
		flsprintf(stdout,"Reading byte at 0x%2.2x\n",address);
putByte(0x7);
putByte(0x01);
putByte(address);
i = getByte();
getByte();
return i;
}



size_t getlinex(char **lineptr, size_t *n, FILE *stream) {
    char *bufptr = NULL;
    char *p = bufptr;
    size_t size;
    int c;

    if (lineptr == NULL) {
    	return -1;
    }
    if (stream == NULL) {
    	return -1;
    }
    if (n == NULL) {
    	return -1;
    }
    bufptr = *lineptr;
    size = *n;

    c = fgetc(stream);
    if (c == EOF) {
    	return -1;
    }
    if (bufptr == NULL) {
    	bufptr = malloc(128);
    	if (bufptr == NULL) {
    		return -1;
    	}
    	size = 128;
    }
    p = bufptr;
    while(c != EOF) {
    	if ((p - bufptr) > (size - 1)) {
    		size = size + 128;
    		bufptr = realloc(bufptr, size);
    		if (bufptr == NULL) {
    			return -1;
    		}
    	}
    	*p++ = c;
    	if (c == '\n') {
    		break;
    	}
    	c = fgetc(stream);
    }

    *p++ = '\0';
    *lineptr = bufptr;
    *n = size;

    return p - bufptr - 1;
}


int parse_hex (char * filename, unsigned char * progmem)
{
	char * line = NULL;
	unsigned char line_content[128];
    size_t len = 0;
	int i,temp;
    int read;
	int line_len, line_type, line_address, line_address_offset=0;
	if (verbose>2) printf ("Opening filename %s \n", filename);
	FILE* sf = fopen(filename, "r");
	if (sf==0)
		return -1;
	
	if (verbose>2) printf ("File open\n");
	while ((read =  getlinex(&line, &len, sf)) != -1) 
		{
		if (verbose>2) printf("\nRead %d chars: %s",read,line);
		if (line[0]!=':') 
			{
			if (verbose>1) printf("--- : invalid\n");
			return -1;
			}
		sscanf(line+1,"%2X",&line_len);
		sscanf(line+3,"%4X",&line_address);
		sscanf(line+7,"%2X",&line_type);
		if (verbose>2) printf("Line len %d B, type %d, address 0x%4.4x offset 0x%4.4x\n",line_len,line_type,line_address,line_address_offset);
		if (line_type==0)
			{
			for (i=0;i<line_len;i++)
				{
				sscanf(line+9+i*2,"%2X",&temp);
				line_content[i] = temp;
				}
			if ((line_address_offset==0))
				{
				if (verbose>2) printf("PM ");
				for (i=0;i<line_len;i++) progmem[line_address+i] = line_content[i];
				}
			}
		if (line_type==4)
			{
			sscanf(line+9,"%4X",&line_address_offset);
			}
		if (verbose>2) for (i=0;i<line_len;i++) printf("%2.2X",line_content[i]);
		if (verbose>2) printf("\n");
		}
	fclose(sf);
	return 0;
}

int is_empty (unsigned char * buff, int len)
{
int i,empty;
empty = 1;
for (i=0;i<len;i++)
	if (buff[i]!=0xFF) empty = 0;
return empty;
}

int main(int argc, char *argv[]) 
	{
	int i,j,pages_performed=0,devid;
	unsigned char * pm_point;
	unsigned char tdat[200];
	parseArgs(argc,argv);
	if (verbose>0) printf ("\n\n -- EFM8 programmer -- \n\n");
	if (verbose>0) printf ("Opening serial port\n");
	initSerialPort();
	if (sleep_time>0)
		{
		printf ("Sleeping for %d ms before accessing serial port\n", sleep_time);
		fflush(stdout);
		sleep_ms (sleep_time);
		}
	for (i=0;i<PROGMEM_LEN;i++) progmem[i] = 0xFF;
	char* filename=argv[argc-1];
	pm_point = (unsigned char *)(&progmem);

	parse_hex(filename,pm_point);
	prog_enter_progmode();
	devid = c2_read_byte(0);
	if (verbose>0) printf ("Target found, ID: 0x%2.2X\n",devid);
	if (program==1)
		{
		c2_mass_erase();
		if (verbose>0) printf ("Programming FLASH (%d B in %d pages per %d bytes): \n",flash_size,flash_size/page_size,page_size);
		fflush(stdout); 
		for (i=0;i<flash_size;i=i+page_size)
			{
			if (is_empty(progmem+i,page_size)==0) 
				{
				c2_write_page(progmem+i,i,page_size);
				pages_performed++;
				if (verbose>1) 
					{
					printf ("#");
					fflush(stdout); 
					} 
				}
			else if (verbose>2) 
				{
				printf (".");
				fflush(stdout); 
				}
			}
		if (verbose>0) printf (" %d pages programmed\n",pages_performed);
		}	
	if (verify==1)
		{
		pages_performed = 0;
		if (verbose>0) printf ("Verifying FLASH (%d B in %d pages per %d bytes): \n",flash_size,flash_size/page_size,page_size);
		for (i=0;i<flash_size;i=i+page_size)
			{
			if (is_empty(progmem+i,page_size))
				{
				if (verbose>2) 
					{	
					printf (".");
					fflush(stdout); 
					}		
				}	
			else
				{
				if (verbose>3) printf ("Verifying page at 0x%4.4X\n",i);
				c2_read_page(tdat,i,page_size);
				pages_performed++;
				if (verbose>1) 
					{
					printf ("#");
					fflush(stdout); 
					}
				for (j=0;j<page_size;j++)
					{
					if (progmem[i+j] != tdat[j])
						{
						printf ("******************************Error at 0x%4.4X E:0x%2.2X R:0x%2.2X\n",i+j,progmem[i+j],tdat[j]);
						printf ("Exiting now\n");
						prog_exit_progmode();
						exit(1);
						}
					}
				}
			}
		if (verbose>0) printf (" %d pages verified\n",pages_performed);
		}

	prog_exit_progmode();
	return 0;
	}

	

void setCPUtype(char* cpu) 
{
}

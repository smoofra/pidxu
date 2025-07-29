// build with MinGW:
// gcc pidxu.c adpcm.c mongoose.c -lwsock32 -lportaudio -o pidxu.exe -Wall; strip pidxu.exe
// build with Linux:
// gcc pidxu.c adpcm.c mongoose.c -lrt -lm -ldl -lportaudio -lpthread -o pidxu -Wall; strip pidxu

// http://hardware.slashdot.org/story/12/08/24/2228251/serious-problems-with-usb-and-ethernet-on-the-raspberry-pi
// http://www.raspyfi.com/raspberry-pi-usb-audio-fix/
// http://wiki.linuxaudio.org/wiki/raspberrypi
// http://www.raspberrypi.org/phpBB3/viewtopic.php?f=28&t=39175

// Raspberry pi
// cpu temp:  cat /sys/class/thermal/thermal_zone0/temp
// GPIO -- http://elinux.org/RPi_Low-level_peripherals
//
// http://kb9mwr.blogspot.com/2013/01/raspberry-pi-and-sound-input.html
//		Add "dwc_otg.speed=1" to /boot/cmdline.txt
//		Edit /etc/modprobe.d/alsa-base.conf
//				comment out "options snd-usb-audio index=-2"
//				add a line: "options snd-usb-audio nrpacks=1"
//
//	also did these, but don't think required:
//			sudo apt-get update
//			sudo apt-get upgrade
//			sudo apt-get install sox alsa-oss alsa-utils
//		update the firmware - https://github.com/Hexxeh/rpi-update/ 
//			sudo apt-get install rpi-update
//			sudo rpi-update		-or-	sudo BRANCH=fiq_split rpi-update

// Raspberry pi -> install/compile portaudio:
//		sudo apt-get install libportaudio2
// - or -
//		sudo apt-get install libasound2-dev
//		download  pa_stable_v19_20111121.tgz  from  http://portaudio.com/download.html
//					or the file marked like this: "<== You probably want this!!"
//		tar -xvzf pa_stable_v19_20111121.tgz
//		cd portaudio
//		./configure
//		make
//		sudo make install

// sudo apt-get install libportaudio0 libportaudio2 libportaudiocpp0 portaudio19-dev

// open source SIP/SDP/RTP/STUN/etc. client -- http://www.creytiv.com/

// mongoose lightweight HTTP server -- https://code.google.com/p/mongoose/
// http://serverfault.com/questions/51597/how-to-fix-tcp-ip-has-reached-the-security-limit-event-message
// http://stackoverflow.com/questions/2306172/malloc-a-3-dimensional-array-in-c
// http://www.codeproject.com/Articles/11740/A-simple-UDP-time-server-and-client-for-beginners
// http://cs.baylor.edu/~donahoo/practical/CSockets/textcode.html
// http://cboard.cprogramming.com/networking-device-communication/56595-recvfrom-timeout.html

#define VERSION "0.1.8"

#define GPIO_COS 2		// Connector P1 - pin 3
#define GPIO_AUX0 3		// Connector P1 - pin 5
#define GPIO_PTT 4		// Connector P1 - pin 7
/* P1 connector pinout - Rev 2 ******************************************
*		P1-1		3.3v					P1-2		5.0v
*		P1-3	GPIO2		SDA1			P1-4		5.0v
*		P1-5	GPIO3		SCL1			P1-6		Ground
*		P1-7	GPIO4						P1-8	GPIO14		TXD0
*		P1-9		Ground					P1-10	GPIO15		RXD0
*		P1-11	GPIO17						P1-12	GPIO18
*		P1-13	GPIO27						P1-14		Ground
*		P1-15	GPIO22						P1-16	GPIO23
*		P1-17		3.3v					P1-18	GPIO24
*		P1-19	GPIO10		SPI_MOSI		P1-20		Ground
*		P1-21	GPIO9		SPI_MISO		P1-22	GPIO25
*		P1-23	GPIO11		SPI_SCLK		P1-24	GPIO8		SPI_CE0_n
*		P1-25								P1-26	GPIO7		SPI_CE1_n
*************************************************************************/


#ifdef WIN32
	#include <winsock.h>
	#define cleanexit mg_stop(mongoose_ctx); Pa_Terminate(); closesocket(nxu.stcp); closesocket(nxu.sudp); WSACleanup(); exit
	#define cleanexitptr mg_stop(mongoose_ctx); Pa_Terminate(); closesocket(nxu->stcp); closesocket(nxu->sudp); WSACleanup(); exit
	#define socklen_t int
	#define dxu_sleep Sleep(2000)
#else
	#include <sys/socket.h>
	#include <sys/un.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <unistd.h>
	#include <errno.h>
	#define closesocket close
	#define cleanexit mg_stop(mongoose_ctx); Pa_Terminate(); close(nxu.stcp); close(nxu.sudp); exit
	#define cleanexitptr mg_stop(mongoose_ctx); Pa_Terminate(); close(nxu->stcp); close(nxu->sudp); exit
	#define SOCKET int
	#define INVALID_SOCKET -1
	#define SOCKET_ERROR -1
	#define WSAGetLastError() errno
	#define WSAECONNRESET ECONNRESET
	#define dxu_sleep nanosleep((struct timespec[]){{2, 0}}, NULL);
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <limits.h>
#include <string.h>
#include <fnmatch.h>
#include <portaudio.h>
#include "adpcm.h"
#include "mongoose.h"

#define MAXPENDING 1			/* Maximum outstanding connection requests per port*/
#define NUMBUFFERS 4			/* Number of buffers for each NXU  - MUST BE POWER OF 2! */
#define BUFFER_DELAY 2			/* Number of buffers to delay TX from RX */
#define SELECT_TIMEOUT_USEC 1000
#define HTTP_BUF_SIZE 16384

#define SAMPLE_RATE        (48000)
#define FRAMES_PER_PACKET  (4800)
#define OUT_FRAMES_PER_BUFFER  (4800)
#define IN_FRAMES_PER_BUFFER  (paFramesPerBufferUnspecified )


#define MULAW_ZERO 0xff

/* bit definitions for byte[4] - 5th byte */ 
#define PIN_COS 1
#define PIN_AUX0 2
#define PIN_AUX1 4

// Raspberry PI - GPIO memory I/O locations
#ifdef __arm__
#include <fcntl.h>
#include <sys/mman.h>

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define BLOCK_SIZE (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET(g) (g<32 ? (*(gpio+7)=(1<<g)) : (*(gpio+8)=(1<<(g-32))))  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR(g) (g<32 ? (*(gpio+10)=(1<<g)) : (*(gpio+11)=(1<<(g-32)))) // clears bits which are 1 ignores bits which are 0
#define READ_GPIO(g) (g<32 ? ((*(gpio+13)&(1<<g))?1:0) : ((*(gpio+14)&(1<<(g-32)))?1:0)) // read pin values
#endif // __arm__

/* structure of data sent in each UDP packet */
struct nxu_udp_packet
{
	// format_recv and format_xmit may be swapped, I haven't tested to determine which is which!
	unsigned char sequence_number;		// incrementing packet sequence number
	unsigned char format_recv;			// ADPCM data format (5=64kbit)
	unsigned char unused2;
	unsigned char format_xmit;			// ADPCM data format (5=64kbit)
	unsigned char inputs;				// connector inputs -- bit 0: COS,  bit 1: AUX0, bit 2: AUX1
	unsigned char unused5;
	unsigned char unused6;
	unsigned char unused7;
	unsigned char pcm[800];				// 8kS/sec u-Law PCM compressed audio
	short raw[4800];					// uncompressed, interpolated to 48kS/sec
};

struct list_item
{
	struct list_item *prev;
	struct list_item *next;
	struct linklist *list;
	void *data;
};

struct linklist
{
	struct list_item *head;
	struct list_item *tail;
};

struct nxu_struct
{
	SOCKET sudp;						// socket handle for UDP
	SOCKET stcp;						// socket handle for TCP listen port
	struct sockaddr_in udp_server;		// UDP server address, port number
	struct sockaddr_in udp_client;		// UDP client address, port number
	struct sockaddr_in tcp_server;		// TCP server address, port number
	unsigned char connected;			// 0 = TCP not connected,  1 = TCP connected
	unsigned char rx_active_test;		// counts 0 to BUFFER_DELAY when UDP data begins to properly buffer audio
	unsigned char rx_active;			// 0 = not receiving audio,  1 = actively receiving audio and repeating
	unsigned char rx_buffer_number;		// incrementing buffer number of most recently received packet
	unsigned char tx_buffer_number;		// incrementing buffer number of next packet to transmit
	unsigned char tx_packet_count;		// incrementing 0-255 number in byte[0] for outgoing packets
	unsigned char rx_packet_count;		// incrementing 0-255 number in byte[0] of incoming packet to transmit next
	int tx_idle_timer;					// keep track of time between sending idle (no audio) packets - sent every 4 secs
	int rx_idle_timer;					// keep track of time since last received packet (timeout for disconnect detection)
	int buffer_error;					// counter to adjust buffering when it is too small or too big
	float phase;						// calibration phase (radians) to produce variable freq, continuous phase calibration output

	/* status only - persist N seconds by counting down from N to zero */
	int dropped_packets;				// count dropped packets and packets that were delayed too long to be repeated
	int buffer_adjust;					// count packet skips (- for insertions)
	unsigned char status_rx;			// 0 = not receiving audio,  >0 = actively receiving audio
	unsigned char status_tx;			// 0 = not sending audio,  >0 = actively sending audio
	unsigned char status_cal;			// 0 = normal operation,  >0 = calibrating (AUX0 pin grounded on NXU)
	time_t connect_time;				// date/time connection began

	/* user config options */
	char sitename[31];
	char server_ip[31];
	unsigned short server_port;
	unsigned short local_port;
	char http_port_str[10];
	unsigned char ptt;					// 0 = idle, 1 = send audio
	unsigned char cal;					// 0 = normal, 1 = set calibration mode (AUX0)
	unsigned char cor_polarity;			// 0 = active low,  1 = active high
	unsigned char cal_polarity;			// 0 = active low,  1 = active high
	unsigned char ptt_polarity;			// 0 = active low,  1 = active high
	/* audio in/out config */
	int audio_in_device;
	int audio_out_device;
	int restart_portaudio;
	unsigned char apply_deemph;
	unsigned char apply_preemph;
	unsigned char filter_pl;

//	unsigned char *buffer_in;			// incoming data buffer pointer
	struct linklist buffer;
};


/* structure of data to pass from main() to portaudio callback function */
typedef struct {
	struct nxu_struct *nxu;
    unsigned char *buffer_in;	// from UDP/IP to Speaker
	short *raw_out;				// from Line In to UDP/IP
	short *raw_out_partial;
	int *tx_callback_tick;
	int *rx_callback_tick;
	int *input_buffer_count;		// rx buffers are smaller than size of UDP packets, so must keep track
	const PaStreamCallbackTimeInfo* CallbackTime;
	struct iir_state *pl_state;
	char *pl_num_stages;
} paUserData;


/* structure of data to pass from main() to mongoose HTTP server callback function */
struct mg_user_data
{
	char *status_buffer;
	char *config_buffer;
	char *portaudio_buffer;
	char *cfgfname;
	struct nxu_struct *nxu;
};

struct iir_state
{
	/* polynomial coefficients are expected to be signed 16 bit integers scaled by 2^12 */
	int in[4];	// circular buffer for previous inputs
	int out[4];	// circular buffer for previous outputs
	short b[2];		// numerator polynomial coefficients b0=4096, b[0]=b1, b[1]=b2
	short a[2];		// denominator polynomial coefficients a0=4096, a[0]=a1, a[1]=a2
	short index;	// index for circular buffers
};

/* functions */
void fir_decimate(short *buffer, int offset);
void fir_interpolate(short *in, short *out, int length);

int deemphasis(int input);
int preemphasis(int input);
int iir_pd(int *last_in, int input, int lshift);
int iir_lpf(int *last_w, int input, int lshift);

int iir_biquad(struct iir_state *state, int input);
void pl_filt_init(struct iir_state **state, char *num_stages);
void pl_filt_free(struct iir_state **state);
short pl_filt(struct iir_state state[], char *num_stages, short input);

void init_nxu(struct nxu_struct *nxu);
void reset_nxu(struct nxu_struct *nxu);
int connect_nxu(struct nxu_struct *nxu);

void wwwstatus(char *buffer, struct nxu_struct *nxu);
void wwwconfig(char *buffer, struct nxu_struct *nxu);
void wwwportaudio(char *buffer, PaStream *instream, PaStream *outstream);
void parse_config(char *buffer, struct nxu_struct *nxu);
void parse_args(int argc, char *argv[], struct nxu_struct *nxu);

static int mongoose_request_handler(struct mg_connection *conn);

static void StreamFinished( void* userData );
PaError setup_portaudio(PaStream **instream, PaStream **outstream, paUserData *data, struct nxu_struct *nxu);
static int pa_output_callback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData );
static int pa_input_callback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData );
#ifdef __arm__
void setup_gpio();
#endif

/* globals vars */
time_t start_time;
char *myname;
const char *cor_polarity_strings[] = { "Active Low", "Active High" };
struct mg_context *mongoose_ctx;
// GP I/O access
#ifdef __arm__
volatile unsigned *gpio;
#endif

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IOLBF, 0);
	setvbuf(stderr, NULL, _IOLBF, 0);

	struct nxu_struct nxu;
	int dropped_printf_threshold = 0;
	char host_name[64];
	unsigned char tcp_buffer[1024];
	unsigned char buffer_in[NUMBUFFERS][1024];
	int buffer_error_calc;
	unsigned char rx_packet_count_last;
	unsigned char buffer_out[1024];
	short raw_out[2*FRAMES_PER_PACKET+800];
	struct hostent *hp;
	int bytes_received = 0;
	int tx_callback_tick = 0;
	int rx_callback_tick = 0;
	int input_buffer_count = 0;
	int file_timer = 0;
	int port_number = 1221;
	socklen_t client_length;
	int i, j, k;
	struct timeval recvtimeout;
	fd_set recv_fd;
	time_t current_time;
	FILE *cfgfile;
	char cfg_buffer[4096];
	char *mypath, *cfgfname, *pwfname;
	short *preemph_sample;

	float sample, average;
	double sum_sq, rms, db;
	double db_peak = -70;
	float db_pk_decay = 0.01;

	char pl_num_stages;
	struct iir_state *pl_state = 0;
	
	PaStream *instream;
	PaStream *outstream;
	paUserData data;
	PaError err;

	struct mg_callbacks mongoose_callbacks;
	struct mg_user_data mongoose_user_data;
	char status_buffer[HTTP_BUF_SIZE];
	char config_buffer[HTTP_BUF_SIZE];
	char portaudio_buffer[HTTP_BUF_SIZE];

	/* prevent unused variable warnings */
	(void) rx_packet_count_last;

	start_time = time(NULL);

	mypath = malloc(256);
#ifdef WIN32
	GetModuleFileName(NULL, mypath, 256);
#else
	readlink("/proc/self/exe", mypath, 256);
#endif
	// i=0; j=0;
	// do {
		// j+=i+1;
		// i=strcspn(mypath+j, "/\\");
	// } while (*(mypath+j+i)!=0);

	// find last / or \ in 
	for ( i=0, j=0;  *(mypath+j+i)!=0;  j+=i+1, i=strcspn(mypath+j, "/\\") ) {};

	myname = strdup(mypath+j);
	*(myname + strcspn(myname, ".")) = 0;	// truncate extension (everything after first '.')

	*(mypath+j) = 0;	// truncate after last \ or /

	cfgfname = malloc(strlen(mypath)+20);
	sprintf(cfgfname, "%s%s", mypath, myname);
#ifdef WIN32
	strcat(cfgfname, ".cfg");
#else
	strcat(cfgfname, ".conf");
#endif

	pwfname = malloc(strlen(mypath)+20);
	sprintf(pwfname, "/=%spassword", mypath);

	// printf("Config file %s\r\n", cfgfname);
	// printf("Password file %s\r\n", pwfname);

	for (i=0; i<strlen(myname); i++)
		myname[i] = toupper(myname[i]);

	/* Startup windows sockets */
#ifdef WIN32
	WSADATA w;
	if (WSAStartup(0x0101, &w) != 0)
	{
		fprintf(stderr, "Could not startup windows sockets.\n");
		exit(0);
	}
#endif

#ifdef __arm__
	setup_gpio();
	// Set GPIO 0 - GPIO 31 to input
	INP_GPIO(GPIO_PTT); // must use INP_GPIO before we can use OUT_GPIO
	INP_GPIO(GPIO_AUX0); // must use INP_GPIO before we can use OUT_GPIO
	INP_GPIO(GPIO_COS); // must use INP_GPIO before we can use OUT_GPIO

	OUT_GPIO(GPIO_PTT);	// set PTT pin as output
#endif

	/* initialize data passing structure for portaudio callback */
    data.buffer_in = &buffer_in[0][0];
	data.nxu = &nxu;
	data.raw_out = &raw_out[0];
	data.raw_out_partial = &raw_out[0];
	data.tx_callback_tick = &tx_callback_tick;
	data.rx_callback_tick = &rx_callback_tick;
	data.input_buffer_count = &input_buffer_count;
	data.pl_num_stages = &pl_num_stages;
	data.pl_state = pl_state;

	/* initialize data passing structure for mongoose HTTP server callback */
	mongoose_user_data.status_buffer = status_buffer;
	mongoose_user_data.config_buffer = config_buffer;
	mongoose_user_data.portaudio_buffer = portaudio_buffer;
	mongoose_user_data.cfgfname = cfgfname;
	mongoose_user_data.nxu = &nxu;

	// buffer_out = malloc(1024*sizeof(char));
	// raw_out = malloc(800*sizeof(float));
	// buffer_in = malloc(MAXCONNECTIONS*1024*sizeof(char));
	if (buffer_out==NULL || raw_out==NULL || buffer_in==NULL)
	{
		fprintf(stderr, "Memory Allocation failed.\r\n");
		cleanexit(0);
	}

	buffer_out[0] = 0;		// incrementing packet sequence number
	buffer_out[1] = 5;		// ADPCM data format (5=64kbit)
	buffer_out[2] = 0;
	buffer_out[3] = 5;		// ADPCM data format (5=64kbit)
	buffer_out[4] = 1;		// connector inputs -- bit 0: COS,  bit 1: AUX0, bit 2: AUX1
	buffer_out[5] = 0;
	buffer_out[6] = 0;
	buffer_out[7] = 0;

	client_length = (int)sizeof(struct sockaddr_in);

    err = Pa_Initialize();
    if( err != paNoError ) {
		fprintf( stderr, "Could not initialize PortAudio.\n");
		fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
		cleanexit(0);
	}

	printf("\r\nPC NXU v" VERSION "\r\nCtrl-C to exit\r\n\r\n");
	init_nxu(&nxu);

	/* load config from file */
	nxu.server_port = port_number;
	if ((cfgfile = fopen(cfgfname, "r")))
	{
		fscanf(cfgfile, "%s\n", cfg_buffer);
		fclose(cfgfile);
		parse_config(cfg_buffer, &nxu);
	}
	// nxu.ptt = 0;		// always start with PTT off

	parse_args(argc, argv, &nxu);

	fprintf(stderr, "Attempting to connect to %s:%d\r\n", nxu.server_ip, nxu.server_port);

	/* DNS lookup IP if needed */
	// use getaddrinfo() for ip6 compatibility
	if ((nxu.udp_server.sin_addr.s_addr = inet_addr(nxu.server_ip)) == htonl(INADDR_NONE)) {
		if ((hp = gethostbyname(nxu.server_ip)) == NULL) {
			fprintf(stderr, "Could not get server IP from host name \"%s\".\n", nxu.server_ip);
		} else {
			memcpy(&(nxu.udp_server.sin_addr.s_addr), hp->h_addr, hp->h_length);
			// printf("%s is at %s\r\n", nxu.server_ip, inet_ntoa(nxu.udp_server.sin_addr));
		}
	}

	sprintf(nxu.http_port_str, "%d", nxu.local_port%100+8000);
	/* Start Mongoose HTTP server */
	const char *mongoose_options[] = {	"listening_ports", nxu.http_port_str,
										"protect_uri", pwfname,				// password protect entire site
										"authentication_domain", "nxuhub",	// using realm "nxuhub"
										"document_root", "",				// don't serve any files (except from callback)
										NULL};
	// Prepare callbacks structure. We have only one callback, the rest are NULL.
	memset(&mongoose_callbacks, 0, sizeof(mongoose_callbacks));
	mongoose_callbacks.begin_request = mongoose_request_handler;
	// Start the web server.
	mongoose_ctx = mg_start(&mongoose_callbacks, &mongoose_user_data, mongoose_options);


	/* set up portaudio */
	if (setup_portaudio(&instream, &outstream, &data, &nxu) != paNoError) {
		cleanexit(0);
	}
	wwwportaudio(portaudio_buffer, instream, outstream);


	/* Get my own IP address */
	gethostname(host_name, sizeof(host_name));
	hp = gethostbyname(host_name);
	if (hp == NULL) {
		fprintf(stderr, "Could not get host name.\n");
		cleanexit(0);
	}

	/* Assign the address */
	memcpy(&nxu.udp_client.sin_addr.s_addr, hp->h_addr, hp->h_length);
	printf("My web browser interface is at http://%s:%s/\r\n", inet_ntoa(nxu.udp_client.sin_addr), nxu.http_port_str);
	memset(&nxu.udp_client.sin_addr.s_addr, 0, hp->h_length);

	while (1)
	{
		/* if configuration (server_ip or server_port) changed,
		 * shutdown and initiate new TCP connection */
		if (	(htons(nxu.server_port) != nxu.udp_server.sin_port)
			 || (nxu.udp_server.sin_addr.s_addr != nxu.tcp_server.sin_addr.s_addr)
//			 || (htons(nxu.local_port) != nxu.udp_client.sin_port)
// removed because of change to use any available udp port if local_port is unavailable
			 || (nxu.restart_portaudio) )
		{
			/* in case this is first time through or config just changed,
			 * set up WWW pages in memory */
			wwwportaudio(portaudio_buffer, instream, outstream);
			wwwstatus(status_buffer, &nxu);
			wwwconfig(config_buffer, &nxu);

			/* shutdown/close existing connections - must shutdown before Pa_StopStream() */
			closesocket(nxu.sudp);
			if (shutdown(nxu.stcp, 1)!=SOCKET_ERROR) {
				closesocket(nxu.stcp);
				fprintf(stderr, "\r\nDisconnected, attempting connection to %s:%d\r\n", nxu.server_ip, nxu.server_port);
			}

			/* if audio configuration changed,
			 * shutdown and restart portaudio stream */
			if (nxu.restart_portaudio)
			{
				err = Pa_StopStream( instream );
				if( err != paNoError ) {
					fprintf(stderr, "Could not stop PortAudio input stream\r\n");
					cleanexit(0);
				}

				err = Pa_StopStream( outstream );
				if( err != paNoError ) {
					fprintf(stderr, "Could not stop PortAudio output stream\r\n");
					cleanexit(0);
				}

				err = Pa_Terminate(); 
				if( err != paNoError ) {
					fprintf(stderr, "Could not terminate PortAudio\r\n");
					cleanexit(0);
				}

				err = Pa_Initialize();
				if( err != paNoError ) {
					fprintf( stderr, "Could not initialize PortAudio.\n");
					fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
					cleanexit(0);
				}

				/* load config from file */
				nxu.server_port = port_number;
				if ((cfgfile = fopen(cfgfname, "r")))
				{
					fscanf(cfgfile, "%s\n", cfg_buffer);
					fclose(cfgfile);
					parse_config(cfg_buffer, &nxu);
				}

				parse_args(argc, argv, &nxu);

				if (setup_portaudio(&instream, &outstream, &data, &nxu) != paNoError ) {
					cleanexit(0);
				}
			}

			/* connect to server */
			if (connect_nxu(&nxu) < 0) {
				dxu_sleep;	// connect failed, wait so we don't retry too often
				continue;
			}

			pl_filt_init(&pl_state, &pl_num_stages);
			data.pl_state = pl_state;
		}


		/* check for TCP server activity */
		recvtimeout.tv_sec = 0;
		recvtimeout.tv_usec = SELECT_TIMEOUT_USEC;
		FD_ZERO(&recv_fd);
		FD_SET(nxu.stcp, &recv_fd);
		int t = select(nxu.stcp+1, &recv_fd, 0, 0, &recvtimeout);
		if (t == SOCKET_ERROR) {
			fprintf(stderr, "Call to TCP select() returned error %d\r\n", WSAGetLastError());
			cleanexit(0);
		}
		if (t != 0) {
			if (FD_ISSET(nxu.stcp, &recv_fd)) {
				/* receive TCP data, check for disconnect, ignore other data */
				bytes_received = recv(nxu.stcp, (char *)tcp_buffer, sizeof(tcp_buffer), 0);
				if ( (bytes_received == 0) || ((bytes_received < 0) && (WSAGetLastError()==WSAECONNRESET)) ) {
					current_time = time(NULL);
					printf("Disconnected at %s", ctime(&current_time));
					nxu.tcp_server.sin_addr.s_addr = 0;		// force TCP reconnect attempt
					dxu_sleep;
				} else if (bytes_received < 0) {
					fprintf(stderr, "Error %d Receiving data from TCP.\r\n", WSAGetLastError());
					cleanexit(0);
				} else { // (bytes_received > 0)
					printf("Received %d bytes from TCP\r\n", bytes_received);
				}
			}
		}


		/* Process UDP Receive data */
		recvtimeout.tv_sec = 0;
		recvtimeout.tv_usec = SELECT_TIMEOUT_USEC;
		FD_ZERO(&recv_fd);
		FD_SET(nxu.sudp, &recv_fd);
		t = select(nxu.sudp+1, &recv_fd, 0, 0, &recvtimeout);
		if (t == SOCKET_ERROR) {
			fprintf(stderr, "Call to UDP select() returned error %d\r\n", WSAGetLastError());
			cleanexit(0);
		}
		if (t != 0) {
			if (FD_ISSET(nxu.sudp, &recv_fd)) {
				/* Receive bytes from udp server */
				nxu.rx_idle_timer = 0;
				nxu.rx_buffer_number = (nxu.rx_buffer_number+1)&(NUMBUFFERS-1);
				bytes_received = recvfrom(nxu.sudp, (char *)buffer_in[nxu.rx_buffer_number], sizeof(buffer_in[0]), 0, 0, 0);
				if (bytes_received < 0) {
					/* check for ICMP destination unreachable error */
					if (WSAGetLastError()==WSAECONNRESET) {
						current_time = time(NULL);
						printf("ICMP Port Unreachable, disconnect at %s", ctime(&current_time));
						nxu.tcp_server.sin_addr.s_addr = 0;		// force TCP reconnect attempt
						dxu_sleep;
					} else {
						fprintf(stderr, "Could not receive datagram, error %d\r\n", WSAGetLastError());
						cleanexit(0);
					}
				}

				/* while inactive, keep tx and rx buffer counters sync'ed */
				if (nxu.rx_active_test == 0) {
					nxu.rx_active = 0;
					nxu.rx_packet_count = buffer_in[nxu.rx_buffer_number][0];
				}

				/* increment nxu.rx_active_test until it equals BUFFER_DELAY to provide stream buffering */
				if (buffer_in[nxu.rx_buffer_number][4] & PIN_COS) {
					if (nxu.rx_active_test < BUFFER_DELAY)
						nxu.rx_active_test++;
				} else 	{
					nxu.rx_active_test = 0;
				}

				/* after BUFFER_DELAY set nxu.rx_active */
				if (nxu.rx_active_test == BUFFER_DELAY)
				{
					nxu.rx_active = 1;
					/* check packet buffering and if needed insert/skip a packet */
					buffer_error_calc = (int)nxu.rx_packet_count - ((buffer_in[nxu.rx_buffer_number][0] - (BUFFER_DELAY-1)) & 255);
					/* nxu.buffer_error = 2*sign(buffer_error_calc) - sign(nxu.buffer_error) */
					nxu.buffer_error += (buffer_error_calc > 0) ? 2 : 0;
					nxu.buffer_error -= (buffer_error_calc < -1) ? 2 : 0;
					nxu.buffer_error -= (nxu.buffer_error > 0);
					nxu.buffer_error += (nxu.buffer_error < 0);
					// if (nxu.buffer_error!=0)
						// printf("nxu.buffer_error[%d] = %d, calc=%d\r\n", i, nxu.buffer_error, buffer_error_calc);
					if (nxu.buffer_error > 600) {
						nxu.rx_packet_count--;
						nxu.buffer_adjust--;
						nxu.buffer_error = 0;
						// current_time = time(NULL);
						// printf("Inserting a packet at %s", ctime(&current_time));
					}
					if (nxu.buffer_error < -600) {
						nxu.rx_packet_count++;
						nxu.buffer_adjust++;
						nxu.buffer_error = 0;
						// current_time = time(NULL);
						// printf("Skipping a packet at %s", ctime(&current_time));
					}
				}

				 // printf("rx buffer: %d   nxu.rx_active: %d   nxu.rx_active_test: %d   packet %d   expected packet %d   dropped %d\r\n",
							// nxu.rx_buffer_number, nxu.rx_active, nxu.rx_active_test, buffer_in[nxu.rx_buffer_number][0], nxu.rx_packet_count, nxu.dropped_packets);
			}
		}

		/* just finished outputting audio */
		if (rx_callback_tick) {
			rx_callback_tick = 0;

			/* set/clear PTT output */
#ifdef __arm__
			if (nxu.rx_active==nxu.ptt_polarity) {
				GPIO_SET(GPIO_PTT);
			} else {
				GPIO_CLR(GPIO_PTT);
			}
#endif

			/* update RX status indicator */
			if (nxu.rx_active) {
				if (nxu.status_rx <= 0) nxu.status_rx = 70;
				if (nxu.status_rx > 1) --nxu.status_rx;
			} else {
				if (nxu.status_rx > 0) --nxu.status_rx;
			}

			/* if we haven't received a message in 20 seconds, disconnect */
			if (++nxu.rx_idle_timer > 200) {
				current_time = time(NULL);
				printf("Timeout, disconnect at %s", ctime(&current_time));
				nxu.tcp_server.sin_addr.s_addr = 0;		// force TCP reconnect attempt
			}

			/* Report dropped packets occasionally */
			if (nxu.dropped_packets > dropped_printf_threshold) {
				fprintf(stderr, "Dropped %d packets\r\n", nxu.dropped_packets);
				dropped_printf_threshold += 10;
			}
		}

		/* just finished inputting audio */
		if (tx_callback_tick) {
			int raw_offset = tx_callback_tick==2 ? FRAMES_PER_PACKET : 0;
			tx_callback_tick = 0;

#ifdef __arm__
//			if (READ_GPIO(GPIO_AUX0)==nxu.cal_polarity)
			if (READ_GPIO(GPIO_AUX0)==nxu.cal_polarity || nxu.cal)
#else
			if (nxu.cal)
#endif
				buffer_out[4] |= PIN_AUX0;
			else
				buffer_out[4] &= ~PIN_AUX0;

#ifdef __arm__
//			if (READ_GPIO(GPIO_COS)==nxu.cor_polarity) {
			if (READ_GPIO(GPIO_COS)==nxu.cor_polarity || nxu.ptt)
#else
			if (nxu.ptt)
#endif
			{
				/*	if input is limited to max/min short divided by 4
					audio distortion is drastically reduced but still happens */
				preemph_sample = &raw_out[raw_offset];
				for (k=0; k<FRAMES_PER_PACKET; k++, preemph_sample++) {
					*preemph_sample = *preemph_sample >> 2;
					// if (*preemph_sample > SHRT_MAX/4)
						// *preemph_sample = SHRT_MAX/4;
					// else if (*preemph_sample < SHRT_MIN/4)
						// *preemph_sample = SHRT_MIN/4;
				}

				/* compute RMS of each buffer for display only */
				average = 0.0;
				preemph_sample = &raw_out[raw_offset];
				for (k=0; k<FRAMES_PER_PACKET; k++)
				{
					average += *preemph_sample++;
				}
				average = average/FRAMES_PER_PACKET;
				sum_sq = 0.0;
				preemph_sample = &raw_out[raw_offset];
				for (k=0; k<FRAMES_PER_PACKET; k++)
				{
					sample = *preemph_sample++ - average;
					sum_sq += (double)sample*(double)sample;
				}
				rms = sqrt(sum_sq/800.0);
				if (rms>0)
				{
					db = 20.0*log10(rms) - 76.7;	// 76.7 so that 0.1 Vrms at test point = -10 dBm
					if (db>db_peak) {
						db_peak = db;
						db_pk_decay = 0.01;
					} else {
						db_peak -= db_pk_decay;
						db_pk_decay += 0.01;
					}

					printf("Audio input dB: %.1f   Peak dB: %.1f                 \r", db, db_peak);
				}


				if (nxu.apply_preemph) {
					preemph_sample = &raw_out[raw_offset];
					for (k=0; k<FRAMES_PER_PACKET; k++) {
						// bug: really need to check for overflow here!
						*preemph_sample = (short)preemphasis((int)*preemph_sample);
						preemph_sample++;
					}
				}
				fir_decimate(raw_out, raw_offset);

				if (nxu.status_tx <= 0) nxu.status_tx = 70;
				if (nxu.status_tx > 1) --nxu.status_tx;
				for (k=0; k<800; k++)
					buffer_out[k+8] = LinearToMuLawSample(raw_out[2*FRAMES_PER_PACKET + k]);

				nxu.tx_idle_timer = 0;
				buffer_out[0] = ++nxu.tx_packet_count;
				buffer_out[4] |= PIN_COS;
				if (sendto(nxu.sudp, (char *)&buffer_out, 808, 0, (struct sockaddr *)&nxu.udp_server, client_length) != 808)
				{
					printf ("Active sendto failed error %d\n", WSAGetLastError());
//					break;
					nxu.tcp_server.sin_addr.s_addr = 0;		// force TCP reconnect attempt
				}
			} else { /* otherwise send idle packet every 4 seconds */
				if (nxu.status_tx > 0) --nxu.status_tx;
				if (--nxu.tx_idle_timer <= 0) {
					nxu.tx_idle_timer = 40;
					buffer_out[0] = ++nxu.tx_packet_count;
					buffer_out[4] &= ~PIN_COS;
					if (sendto(nxu.sudp, (char *)&buffer_out, 8, 0, (struct sockaddr *)&nxu.udp_server, client_length) != 8)
					{
						printf ("Idle sendto failed error %d\n", WSAGetLastError());
//						break;
						nxu.tcp_server.sin_addr.s_addr = 0;		// force TCP reconnect attempt
					}
				}
			}

			if (--file_timer <= 0) {
				file_timer = 10;
				wwwstatus(status_buffer, &nxu);
				wwwconfig(config_buffer, &nxu);
				wwwportaudio(portaudio_buffer, instream, outstream);
			}
			// printf("Time: callback=%f  ", data.CallbackTime->currentTime);
			// printf("ADC in=%f\r\n", data.CallbackTime->inputBufferAdcTime);
			// printf("DAC out=%f\r\n", data.CallbackTime->outputBufferDacTime);
		}
	}

	/* shutdown/close existing connections - must shutdown before Pa_StopStream() */
	closesocket(nxu.sudp);
	if (shutdown(nxu.stcp, 1)!=SOCKET_ERROR) {
		closesocket(nxu.stcp);
	}

    if( Pa_StopStream( instream ) != paNoError ) {
		fprintf(stderr, "Could not stop PortAudio input stream\r\n");
		cleanexit(0);
	}

    if( Pa_StopStream( outstream ) != paNoError ) {
		fprintf(stderr, "Could not stop PortAudio output stream\r\n");
		cleanexit(0);
	}

	if (pl_state) pl_filt_free(&pl_state);
	cleanexit(0);
}


/* Decimation filter takes input buffer of 4800 samples @ 48000 samples/s
 * filters and stores decimated output in extra 800 samples at end of buffer
 */
void fir_decimate(short *buffer, int offset)
{
	/* FIR Filter coefficients from octave - nxu_decim.m
	 * order = 119, fs = 48000, fout = 8000, fpass = 3250, fstop = 4000 leftshift = 17
	 */
	short decim_coe[] = {
		    357,    151,    126,     55,    -60,   -206,   -362,   -499,
		   -589,   -607,   -539,   -389,   -178,     60,    279,    434,
		    488,    423,    245,    -17,   -308,   -567,   -728,   -746,
		   -601,   -307,     85,    499,    842,   1031,   1007,    751,
		    297,   -276,   -853,  -1302,  -1509,  -1397,   -953,   -236,
		    628,   1464,   2080,   2305,   2032,   1244,     34,  -1400,
		  -2779,  -3784,  -4110,  -3522,  -1908,    698,   4100,   7963,
		  11856,  15312,  17898,  19281,  19281,  17898,  15312,  11856,
		   7963,   4100,    698,  -1908,  -3522,  -4110,  -3784,  -2779,
		  -1400,     34,   1244,   2032,   2305,   2080,   1464,    628,
		   -236,   -953,  -1397,  -1509,  -1302,   -853,   -276,    297,
		    751,   1007,   1031,    842,    499,     85,   -307,   -601,
		   -746,   -728,   -567,   -308,    -17,    245,    423,    488,
		    434,    279,     60,   -178,   -389,   -539,   -607,   -589,
		   -499,   -362,   -206,    -60,     55,    126,    151,    357,
		};

	// /* FIR Filter coefficients from octave - nxu_decim.m
	 // * order = 191, fs = 48000, fout = 8000, fpass = 3250, fstop = 4000 leftshift = 17
	 // */
	// short decim_coe[] = {
		     // -2,     17,     31,     55,     85,    118,    151,    178,
		    // 194,    193,    173,    133,     77,     11,    -55,   -111,
		   // -147,   -155,   -132,    -80,     -9,     70,    140,    186,
		    // 197,    166,     98,      3,   -101,   -193,   -253,   -263,
		   // -218,   -123,      8,    149,    270,    344,    351,    283,
		    // 147,    -32,   -219,   -376,   -466,   -462,   -359,   -169,
		     // 74,    322,    521,    625,    602,    446,    180,   -148,
		   // -472,   -723,   -837,   -780,   -547,   -174,    270,    696,
		   // 1009,   1131,   1017,    668,    139,   -473,  -1044,  -1445,
		  // -1569,  -1360,   -827,    -48,    836,   1645,   2192,   2323,
		   // 1951,   1081,   -174,  -1606,  -2936,  -3855,  -4079,  -3396,
		  // -1715,    914,   4290,   8084,  11880,  15234,  17735,  19070,
		  // 19070,  17735,  15234,  11880,   8084,   4290,    914,  -1715,
		  // -3396,  -4079,  -3855,  -2936,  -1606,   -174,   1081,   1951,
		   // 2323,   2192,   1645,    836,    -48,   -827,  -1360,  -1569,
		  // -1445,  -1044,   -473,    139,    668,   1017,   1131,   1009,
		    // 696,    270,   -174,   -547,   -780,   -837,   -723,   -472,
		   // -148,    180,    446,    602,    625,    521,    322,     74,
		   // -169,   -359,   -462,   -466,   -376,   -219,    -32,    147,
		    // 283,    351,    344,    270,    149,      8,   -123,   -218,
		   // -263,   -253,   -193,   -101,      3,     98,    166,    197,
		    // 186,    140,     70,     -9,    -80,   -132,   -155,   -147,
		   // -111,    -55,     11,     77,    133,    173,    193,    194,
		    // 178,    151,    118,     85,     55,     31,     17,     -2,
		// };

	int num_coeff = sizeof(decim_coe)/sizeof(decim_coe[0]);
	int i, j, k, src;
	register int accum;

	accum = 0;
	src = offset - num_coeff;
	if (src<0)
		src += 2*FRAMES_PER_PACKET;
	for (i=0; i<800; i++) {
		for (j=0, k=src+6*i; j<num_coeff; j++, k++) {
			if (k >= 2*FRAMES_PER_PACKET)
				k -= 2*FRAMES_PER_PACKET;
			accum += (((int)decim_coe[j] * buffer[k]) >> 9);
		}
		buffer[2*FRAMES_PER_PACKET+i] = (short)(accum >> 8);
		accum = 0;
	}
}


/* Interpolation filter takes in buffer of length samples @ 8000 samples/sec
 * filters and stores interpolated output of 6*length samples @ 48 kS/sec in out buffer
 *
 * setting in to NULL will use zeros for all samples
 */
void fir_interpolate(short *in_buffer, short *out_buffer, int length)
{
	/* FIR Filter coefficients from octave - nxu_decim.m
	 * order = 119, fs = 48000, fout = 8000, fpass = 3250, fstop = 4000 leftshift = 15
	 */
	short interp_coe[] = {
		    535,    226,    189,     83,    -89,   -309,   -543,   -749,
		   -883,   -910,   -809,   -584,   -267,     90,    419,    651,
		    733,    635,    367,    -25,   -463,   -850,  -1092,  -1120,
		   -901,   -460,    128,    748,   1263,   1546,   1510,   1126,
		    445,   -415,  -1279,  -1953,  -2263,  -2095,  -1430,   -354,
		    942,   2196,   3119,   3458,   3048,   1866,     51,  -2101,
		  -4169,  -5676,  -6164,  -5284,  -2862,   1047,   6149,  11945,
		  17784,  22968,  26847,  28922,  28922,  26847,  22968,  17784,
		  11945,   6149,   1047,  -2862,  -5284,  -6164,  -5676,  -4169,
		  -2101,     51,   1866,   3048,   3458,   3119,   2196,    942,
		   -354,  -1430,  -2095,  -2263,  -1953,  -1279,   -415,    445,
		   1126,   1510,   1546,   1263,    748,    128,   -460,   -901,
		  -1120,  -1092,   -850,   -463,    -25,    367,    635,    733,
		    651,    419,     90,   -267,   -584,   -809,   -910,   -883,
		   -749,   -543,   -309,    -89,     83,    189,    226,    535,
		};
	#define INTERP_LOCAL_BUFF_SIZE 20

	// /* FIR Filter coefficients from octave - nxu_decim.m
	 // * order = 191, fs = 48000, fout = 8000, fpass = 3250, fstop = 4000 leftshift = 15
	 // */
	// const short interp_coe[] = {
		     // -4,     25,     47,     82,    127,    177,    226,    267,
		    // 290,    289,    259,    200,    116,     17,    -83,   -167,
		   // -221,   -232,   -198,   -120,    -13,    105,    209,    279,
		    // 295,    250,    148,      5,   -152,   -290,   -379,   -395,
		   // -327,   -184,     12,    223,    405,    516,    526,    424,
		    // 221,    -47,   -329,   -564,   -698,   -693,   -538,   -253,
		    // 111,    483,    782,    937,    903,    669,    270,   -222,
		   // -709,  -1084,  -1256,  -1170,   -821,   -262,    405,   1044,
		   // 1514,   1697,   1525,   1002,    209,   -710,  -1566,  -2167,
		  // -2354,  -2041,  -1241,    -72,   1254,   2467,   3288,   3485,
		   // 2926,   1622,   -261,  -2409,  -4404,  -5783,  -6118,  -5095,
		  // -2572,   1371,   6435,  12126,  17821,  22851,  26602,  28605,
		  // 28605,  26602,  22851,  17821,  12126,   6435,   1371,  -2572,
		  // -5095,  -6118,  -5783,  -4404,  -2409,   -261,   1622,   2926,
		   // 3485,   3288,   2467,   1254,    -72,  -1241,  -2041,  -2354,
		  // -2167,  -1566,   -710,    209,   1002,   1525,   1697,   1514,
		   // 1044,    405,   -262,   -821,  -1170,  -1256,  -1084,   -709,
		   // -222,    270,    669,    903,    937,    782,    483,    111,
		   // -253,   -538,   -693,   -698,   -564,   -329,    -47,    221,
		    // 424,    526,    516,    405,    223,     12,   -184,   -327,
		   // -395,   -379,   -290,   -152,      5,    148,    250,    295,
		    // 279,    209,    105,    -13,   -120,   -198,   -232,   -221,
		   // -167,    -83,     17,    116,    200,    259,    289,    290,
		    // 267,    226,    177,    127,     82,     47,     25,     -4,
		// };
	// #define INTERP_LOCAL_BUFF_SIZE 32

	const int num_coeff = sizeof(interp_coe)/sizeof(interp_coe[0]);
	int i, j;
	register int accum, k, m;
	short *out = out_buffer;
	short *in = in_buffer;
	int limit;

	static short local_buffer[INTERP_LOCAL_BUFF_SIZE] = { 0 };	// wrong if coeff size changes
	static int local_buff_index = 0;

	/* IMPORTANT: For this algorithm to function properly,
	 * num_coeff should be a multiple of the interpolation factor (6) */

	accum = 0;
	limit = INTERP_LOCAL_BUFF_SIZE;
	if (in) limit = length;

	for (i=0; i<limit; i++) {
		local_buffer[local_buff_index] = (in==NULL) ? 0 : *in++;
		if (++local_buff_index >= INTERP_LOCAL_BUFF_SIZE) local_buff_index = 0;
		for (j=5; j>=0; j--) {
			for (k=j, m=local_buff_index; k<num_coeff; k+=6, m++) {
				if (m >= INTERP_LOCAL_BUFF_SIZE) m = 0;
				accum += (((int)interp_coe[k] * local_buffer[m]) >> 9);
			}
			*out++ = (short)(accum >> 6);
			accum = 0;
		}
	}
	// to save CPU cycles when in==NULL:
	// only run filter on INTERP_LOCAL_BUFF_SIZE zeros (to flush local_buffer)
	// and fill remaining output buffer with zeros
	if (i<length)
		for ( ; i<length; i++)
			for (j=5; j>=0; j--)
				*out++ = 0;
}


PaError setup_portaudio(PaStream **instream, PaStream **outstream, paUserData *data, struct nxu_struct *nxu)
{
	nxu->restart_portaudio = 0;

	/* Set up audio output stream */
    PaStreamParameters inputParameters;
    PaStreamParameters outputParameters;
    PaError err = -1;

	outputParameters.device = nxu->audio_out_device;
	if ((outputParameters.device < 0) || (outputParameters.device >= Pa_GetDeviceCount()))
		outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters.device == paNoDevice) {
		fprintf(stderr,"Error: No default output device.\n");
		return err;
    }
    outputParameters.channelCount = 2;       /* stereo output */
    outputParameters.sampleFormat = paInt16;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(
              outstream,
              NULL,
              &outputParameters,
              SAMPLE_RATE,
              OUT_FRAMES_PER_BUFFER,
              paClipOff,      /* we won't output out of range samples so don't bother clipping them */
              pa_output_callback,
              data );
    if( err != paNoError ) {
		fprintf( stderr, "Could not open PortAudio output stream.\n");
		fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
		return err;
	}

    err = Pa_StartStream( *outstream );
    if( err != paNoError ) {
		fprintf( stderr, "Could not start PortAudio output stream.\n");
		fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
		return err;
	}

	/* set up input stream */
	inputParameters.device = nxu->audio_in_device;
	if ((inputParameters.device < 0) || (inputParameters.device >= Pa_GetDeviceCount()))
		inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
    if (inputParameters.device == paNoDevice) {
        fprintf(stderr,"Error: No default input device.\n");
		return err;
    }
    inputParameters.channelCount = 1;                    /* stereo input */
    inputParameters.sampleFormat = paInt16;
    // inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
    inputParameters.suggestedLatency = 0.02;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(
              instream,
              &inputParameters,
              NULL,
              SAMPLE_RATE,
              IN_FRAMES_PER_BUFFER,
              paClipOff,      /* we won't output out of range samples so don't bother clipping them */
              pa_input_callback,
              data );
    if( err != paNoError ) {
		fprintf( stderr, "Could not open PortAudio input stream.\n");
		fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
		return err;
	}

    err = Pa_StartStream( *instream );
    if( err != paNoError ) {
		fprintf( stderr, "Could not start PortAudio input stream.\n");
		fprintf( stderr, "Error %d: %s\n", err, Pa_GetErrorText( err ) );
		return err;
	}

	// printf("in_dev = %d, out_dev = %d\r\n", inputParameters.device, outputParameters.device);
	// printf("defaultLowInputLatency = %f, defaultLowOutputLatency = %f\r\n", 
			// Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency,
			// Pa_GetDeviceInfo( inputParameters.device )->defaultLowOutputLatency );
	// printf("defaultHighInputLatency = %f, defaultHighOutputLatency = %f\r\n", 
			// Pa_GetDeviceInfo( inputParameters.device )->defaultHighInputLatency,
			// Pa_GetDeviceInfo( inputParameters.device )->defaultHighOutputLatency );


	/* end PortAudio setup */
	return paNoError;
}

/* This routine will be called by the PortAudio engine when audio is needed.
** It may called at interrupt level on some machines so don't do anything
** that could mess up the system like calling malloc() or free().
*/
static int pa_output_callback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData )
{
    paUserData *data = (paUserData*)userData;
    short *out = (short*)outputBuffer;
    unsigned long i, j;

	int tx_buffer_number;
	int buffer_num;
//	short sample;
	unsigned char *interpolate_in;
	short *sample;

    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) inputBuffer;
    (void) j;
    (void) sample;

	*data->rx_callback_tick = 1;

	/* determine which buffer is next in sequence, check for drops */
	tx_buffer_number = -1;
	if (data->nxu->rx_active) {
		for (buffer_num=0; buffer_num<(NUMBUFFERS*1024); buffer_num += 1024)
			if (data->nxu->rx_packet_count == *(data->buffer_in + buffer_num + 0))
				tx_buffer_number = buffer_num;

		data->nxu->rx_packet_count++;
		if (tx_buffer_number<0) {
			data->nxu->dropped_packets++;
		} else {
			if ( !(*(data->buffer_in + tx_buffer_number + 4) & PIN_COS) ) {
				data->nxu->rx_active = 0;
				data->nxu->rx_active_test = 0;
			}
		}
	}

	if (data->nxu->rx_active_test==0)
		data->nxu->rx_active = 0;

	/* audio from server to output (speaker) */
	interpolate_in = NULL;		// NULL pointer indicates use all zeros as input data
	if (tx_buffer_number>=0)
		interpolate_in = data->buffer_in + tx_buffer_number + 8;

	sample = out;
	if (interpolate_in) {
		if (data->nxu->filter_pl) {
			for (i=0; i<800; i++)
				*sample++ = pl_filt(data->pl_state, data->pl_num_stages, MuLawDecompressTable[*interpolate_in++]);
		} else {
			for (i=0; i<800; i++)
				*sample++ = (MuLawDecompressTable[*interpolate_in++]);
		}
		sample = out;
	} else {
		pl_filt_init(&(data->pl_state), data->pl_num_stages);
		sample = NULL;
	}

	fir_interpolate(sample, &out[framesPerBuffer], framesPerBuffer/6);

	sample = &out[framesPerBuffer];
	if (interpolate_in && data->nxu->apply_deemph) {
		for (i=0; i<framesPerBuffer; i++, sample++) {
			// bug: really need to check for overflow here!
			*sample = (short)deemphasis((int)*sample);
		}
	}

	sample = &out[framesPerBuffer];
	for (i=0; i<framesPerBuffer; i++, sample++) {
		*out++ = *sample;
		*out++ = *sample;
	}

	/* audio from server to output (speaker) */
	/* cheap easy way to resample -- duplicate each sample 6 times */
	// if (tx_buffer_number<0) {
		// for( i=0; i<framesPerBuffer; i++ ) {
			// *out++ = 0;
			// *out++ = 0;
		// }
	// } else {
	// for( i=0, j=0; i<framesPerBuffer; i++ ) {
			// if (i%6 == 0) sample = MuLawDecompressTable[*(data->buffer_in + tx_buffer_number + (j++) + 8)];
			// *out++ = sample;
			// *out++ = sample;
		// }
    // }

	return paContinue;
}


static int pa_input_callback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData )
{
    paUserData *data = (paUserData*)userData;
    short *in = (short*)inputBuffer;
    unsigned long i, framesToEnd;

    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) outputBuffer;

	data->CallbackTime = timeInfo;		// debug

	/* audio from input (mic) to server */
	framesToEnd = FRAMES_PER_PACKET - *data->input_buffer_count;
	if ( (framesToEnd > 0) && (framesToEnd <= framesPerBuffer) )
		*data->tx_callback_tick = 1;

	framesToEnd = 2*FRAMES_PER_PACKET - *data->input_buffer_count;
	*data->input_buffer_count += framesPerBuffer;
	if (framesToEnd <= framesPerBuffer) {
		*data->input_buffer_count -= 2*FRAMES_PER_PACKET;
		*data->tx_callback_tick = 2;
		for( i=0; i<framesToEnd; i++ )
			*data->raw_out_partial++ = *in++;
		data->raw_out_partial = data->raw_out;
		for( ; i<framesPerBuffer; i++ )
			*data->raw_out_partial++ = *in++;
	} else {
		for( i=0; i<framesPerBuffer; i++ )
			*data->raw_out_partial++ = *in++;
	}

	return paContinue;
}


/*
 * This routine is called by portaudio when playback is done.
 */
static void StreamFinished( void* userData )
{
    paUserData *data = (paUserData*)userData;

	data->nxu->restart_portaudio = 1;
}

void wwwportaudio(char *buffer, PaStream *instream, PaStream *outstream)
{
	const PaStreamInfo* streaminfo = Pa_GetStreamInfo(instream);
    int numDevices = Pa_GetDeviceCount();
	int i;
	const   PaDeviceInfo *deviceInfo;
	sprintf(buffer,
			"<!DOCTYPE html>\n<html>\r\n"
			"<head>\n"
			"<title>%s PortAudio Status</title>\r\n"
			"</head>\r\n"
			"<body>\r\n", myname);
	sprintf(buffer + strlen(buffer),
			"<pre>PortAudio input stream info\r\n"
			"\tstructVersion %d\r\n"
			"\tinputLatency %f\r\n"
			"\toutputLatency %f\r\n"
			"\tsampleRate %f\r\n"
			"\r\n"
			"\r\n",
			streaminfo->structVersion,
			streaminfo->inputLatency,
			streaminfo->outputLatency,
			streaminfo->sampleRate );
	streaminfo = Pa_GetStreamInfo(outstream);
 	sprintf(buffer + strlen(buffer),
			"<pre>PortAudio output stream info\r\n"
			"\tstructVersion %d\r\n"
			"\tinputLatency %f\r\n"
			"\toutputLatency %f\r\n"
			"\tsampleRate %f\r\n"
			"\r\n"
			"\r\n",
			streaminfo->structVersion,
			streaminfo->inputLatency,
			streaminfo->outputLatency,
			streaminfo->sampleRate );
   for( i=0; i<numDevices; i++ ) {
		deviceInfo = Pa_GetDeviceInfo( i );
        if( i == Pa_GetDefaultInputDevice() )
			sprintf(buffer + strlen(buffer), "Default input:   ");
        else if( i == Pa_GetDefaultOutputDevice() )
			sprintf(buffer + strlen(buffer), "Default output:  ");
		else
			sprintf(buffer + strlen(buffer), "                 ");
		sprintf(buffer + strlen(buffer),
				"Device %d: %s\r\n", i, deviceInfo->name);
	}

	sprintf(buffer + strlen(buffer),
			"</pre></body>\r\n"
 			"</html>\r\n" );

	return;
}

void wwwstatus(char *buffer, struct nxu_struct *nxu)
{
	int cos_active, cal_active;

#ifdef __arm__
	cos_active = nxu->ptt || READ_GPIO(GPIO_COS)==nxu->cor_polarity;
	cal_active = nxu->cal || READ_GPIO(GPIO_AUX0)==nxu->cal_polarity;
#else
	cos_active = nxu->ptt;
	cal_active = nxu->cal;
#endif

	sprintf(buffer, 
			"<!DOCTYPE html>\n<html>\r\n"
			"<head>\n"
			"<meta http-equiv=\"refresh\" content=\"5\">\r\n"
			"<title>%s Status</title>\r\n"
			"</head>\r\n"
			"<body onLoad=\"document.getElementById('pttcheckbox').focus();\">\r\n"
			"<h1 style=\"text-align:center;\">%s v%s - %s</h1>\r\n"
			"<h3 style=\"text-align:center;font-size:22px;\">"
			"Status&nbsp;&nbsp;&nbsp;"
			"<a href=\"/config.html\">Configuration</a></h3>\r\n"
			, myname, myname, VERSION, nxu->sitename);

	sprintf(buffer + strlen(buffer),
			"<form name=\"pttcal\" action=\"index.html\" method=\"post\" style=\"text-align:center;\">\r\n"
			"PTT: <input type=\"checkbox\" id=\"pttcheckbox\" name=\"ptt\" value=\"on\" tabindex=\"1\" onclick=\"this.form.submit()\"" );
	if (nxu->ptt) sprintf(buffer + strlen(buffer), " checked");
	sprintf(buffer + strlen(buffer),
			">\r\nCal: <input type=\"checkbox\" name=\"cal\" value=\"on\" tabindex=\"2\" onclick=\"this.form.submit()\"" );
	if (nxu->cal) sprintf(buffer + strlen(buffer), " checked");
	sprintf(buffer + strlen(buffer),
			">\r\n<input type=\"hidden\" name=\"ptt\" value=\"off\">\r\n"
			"<input type=\"hidden\" name=\"cal\" value=\"off\">\r\n"
			"</form><br>\r\n" );

	sprintf(buffer + strlen(buffer),
			"<table align=\"center\" style=\"text-align:center;\">\r\n");

	sprintf(buffer + strlen(buffer),
			"<tr><td>Local UDP Port &nbsp; </td><td>%d</td></tr>\r\n", ntohs(nxu->udp_client.sin_port));
	sprintf(buffer + strlen(buffer),
			"<tr><td>Server IP:port &nbsp; </td>");
	if (inet_addr(nxu->server_ip) == htonl(INADDR_NONE)) {
		sprintf(buffer + strlen(buffer),
				"<td>%s</td></tr>\r\n<tr><td>&nbsp;</td>"
				, nxu->server_ip);
	}
	sprintf(buffer + strlen(buffer),
			"<td>%s:%d</td></tr>\r\n"
			, inet_ntoa(nxu->udp_server.sin_addr), ntohs(nxu->udp_server.sin_port));
	if (nxu->connected)
		sprintf(buffer + strlen(buffer), "<tr><td>Connected since &nbsp; </td><td>%s</td></tr>\r\n", ctime(&nxu->connect_time));
	else
		sprintf(buffer + strlen(buffer), "<tr><td>Not Connected</td></tr>\r\n");

	strcat(buffer,
			"<tr><td>&nbsp;</td></tr>"
			"<tr><td>Status &nbsp; </td><td>");
	if (nxu->status_rx)
		strcat(buffer, "<a style=\"color:green\">RX</a> ");
	if (cos_active || nxu->status_tx)
		strcat(buffer, "<a style=\"color:red\">TX</a> ");
	strcat(buffer, "</td></tr>\r\n");

	sprintf(buffer + strlen(buffer),
			"<tr><td>Drops &nbsp; </td><td>%d</td></tr>\r\n", nxu->dropped_packets);
	sprintf(buffer + strlen(buffer),
			"<tr><td>Skips &nbsp; </td><td>%d</td></tr>\r\n", nxu->buffer_adjust);


	sprintf(buffer + strlen(buffer),
			"<tr><td>COS in &nbsp; </td>");
#ifdef __arm__
	sprintf(buffer + strlen(buffer), "<td>%s</td>", cor_polarity_strings[nxu->cor_polarity]);
#endif
	if (cos_active)
		sprintf(buffer + strlen(buffer), "<th>Active</th>");
	else
		sprintf(buffer + strlen(buffer), "<td>Not active</td>");
	sprintf(buffer + strlen(buffer), "</tr>\r\n");


	sprintf(buffer + strlen(buffer),
			"<tr><td>Cal (AUX0) &nbsp; </td>");
#ifdef __arm__
	sprintf(buffer + strlen(buffer), "<td>%s</td>", cor_polarity_strings[nxu->cal_polarity]);
#endif
	if (cal_active)
		sprintf(buffer + strlen(buffer), "<th>Active</th>");
	else
		sprintf(buffer + strlen(buffer), "<td>Not active</td>");
	sprintf(buffer + strlen(buffer), "</tr>\r\n");


#ifdef __arm__
	sprintf(buffer + strlen(buffer),
			"<tr><td>PTT out &nbsp; </td>");
	sprintf(buffer + strlen(buffer), "<td>%s</td>", cor_polarity_strings[nxu->ptt_polarity]);
	if (nxu->rx_active)
		sprintf(buffer + strlen(buffer), "<th>Active</th>");
	else
		sprintf(buffer + strlen(buffer), "<td>Not active</td>");
	sprintf(buffer + strlen(buffer), "</tr>\r\n");
#endif


	sprintf(buffer + strlen(buffer), 
			"</table>\r\n"
			"<h2 style=\"text-align:center;font-size:22px;\">Up Since %s</h2>\r\n"
			"</body>\r\n"
			"</html>\r\n"
			, ctime(&start_time)
			);

	return;
}



void wwwconfig(char *buffer, struct nxu_struct *nxu)
{
    int numDevices = Pa_GetDeviceCount();
	const   PaDeviceInfo *deviceInfo;
	int i;

	sprintf(buffer, 
			"<!DOCTYPE html>\n<html>\r\n"
			"<head>\r\n"
			"<title>%s Config</title>\r\n"
			"</head>\r\n"
			"<body>\r\n"
			"<h1 style=\"text-align:center;\">%s v%s - %s</h1>\r\n"
			"<h3 style=\"text-align:center;font-size:22px;\">"
			"<a href=\"/index.html\">Status</a>&nbsp;&nbsp;&nbsp;"
			"Configuration</h3><br>\r\n"
			"<form id=\"frm1\" action=\"saveconfig.asp\" method=\"post\">\r\n"
			, myname, myname, VERSION, nxu->sitename);
	sprintf(buffer + strlen(buffer),
			"Site Name: <input type=\"text\" size=30 name=\"sitename\" value=\"%s\"><br>\r\n",
			nxu->sitename);
	sprintf(buffer + strlen(buffer),
			"Server IP address: <input type=\"text\" size=30 name=\"server_ip\" value=\"%s\"><br>\r\n",
			nxu->server_ip);
	sprintf(buffer + strlen(buffer),
			"Server VOIP port: <input type=\"text\" size=6 name=\"server_port\" value=\"%d\"><br>\r\n",
			nxu->server_port);
	sprintf(buffer + strlen(buffer),
			"My VOIP port: <input type=\"text\" size=6 name=\"local_port\" value=\"%d\"><br>\r\n",
			nxu->local_port);

#ifdef __arm__
	sprintf(buffer + strlen(buffer), "<br>COR Polarity: <select name=\"cor_polarity\">\r\n");
	for (i=0; i<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); i++)
		if (i==nxu->cor_polarity)
			sprintf(buffer + strlen(buffer), "<option selected>%s\r\n", cor_polarity_strings[i]);
		else
			sprintf(buffer + strlen(buffer), "<option>%s\r\n", cor_polarity_strings[i]);
	sprintf(buffer + strlen(buffer), "</select><br>\r\n");

	sprintf(buffer + strlen(buffer), "Cal (AUX0) Polarity: <select name=\"cal_polarity\">\r\n");
	for (i=0; i<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); i++)
		if (i==nxu->cal_polarity)
			sprintf(buffer + strlen(buffer), "<option selected>%s\r\n", cor_polarity_strings[i]);
		else
			sprintf(buffer + strlen(buffer), "<option>%s\r\n", cor_polarity_strings[i]);
	sprintf(buffer + strlen(buffer), "</select><br>\r\n");

	sprintf(buffer + strlen(buffer), "PTT Polarity: <select name=\"ptt_polarity\">\r\n");
	for (i=0; i<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); i++)
		if (i==nxu->ptt_polarity)
			sprintf(buffer + strlen(buffer), "<option selected>%s\r\n", cor_polarity_strings[i]);
		else
			sprintf(buffer + strlen(buffer), "<option>%s\r\n", cor_polarity_strings[i]);
	sprintf(buffer + strlen(buffer), "</select><br>\r\n");
#endif
	// sprintf(buffer + strlen(buffer), "PTT: <input type=\"checkbox\" name=\"ptt\" value=\"on\"");
	// if (nxu->ptt) sprintf(buffer + strlen(buffer), " checked");
	// sprintf(buffer + strlen(buffer), "><br>\r\n");


 	sprintf(buffer + strlen(buffer), "<br>Audio Input: <select name=\"audio_input\">\r\n");
	if (nxu->audio_in_device<0) /* negative = use default */
		sprintf(buffer + strlen(buffer), "<option selected>Default\r\n");
	else
		sprintf(buffer + strlen(buffer), "<option>Default\r\n");

	for( i=0; i<numDevices; i++ ) {
		deviceInfo = Pa_GetDeviceInfo( i );
		if (deviceInfo->maxInputChannels > 0) {
			if( i == nxu->audio_in_device )
				sprintf(buffer + strlen(buffer), "<option selected>%s", deviceInfo->name);
			else
				sprintf(buffer + strlen(buffer), "<option>%s", deviceInfo->name);
			// if ( i == Pa_GetDefaultInputDevice() )
				// sprintf(buffer + strlen(buffer), "        <-- Default\r\n");
			// else
				// sprintf(buffer + strlen(buffer), "\r\n");
		}
	}
	sprintf(buffer + strlen(buffer), "</select><br>\r\n");

	sprintf(buffer + strlen(buffer), "Audio Output: <select name=\"audio_output\">\r\n");
	if (nxu->audio_out_device<0) /* negative = use default */
		sprintf(buffer + strlen(buffer), "<option selected>Default\r\n");
	else
		sprintf(buffer + strlen(buffer), "<option>Default\r\n");

	for( i=0; i<numDevices; i++ ) {
		deviceInfo = Pa_GetDeviceInfo( i );
		if (deviceInfo->maxOutputChannels > 0) {
			if( i == nxu->audio_out_device )
				sprintf(buffer + strlen(buffer), "<option selected>%s", deviceInfo->name);
			else
				sprintf(buffer + strlen(buffer), "<option>%s", deviceInfo->name);
			// if ( i == Pa_GetDefaultOutputDevice() )
				// sprintf(buffer + strlen(buffer), "        <-- Default\r\n");
			// else
				// sprintf(buffer + strlen(buffer), "\r\n");
		}
	}
	sprintf(buffer + strlen(buffer), "</select><br>\r\n");

	sprintf(buffer + strlen(buffer), "<br><input type=\"hidden\" name=\"apply_preemph\" value=\"off\">\r\n");
	sprintf(buffer + strlen(buffer), "<input type=\"checkbox\" name=\"apply_preemph\" value=\"on\"");
	if (nxu->apply_preemph) sprintf(buffer + strlen(buffer), " checked");
	sprintf(buffer + strlen(buffer), ">Apply pre-emphasis<br>\r\n");

	sprintf(buffer + strlen(buffer), "<input type=\"hidden\" name=\"apply_deemph\" value=\"off\">\r\n");
	sprintf(buffer + strlen(buffer), "<input type=\"checkbox\" name=\"apply_deemph\" value=\"on\"");
	if (nxu->apply_deemph) sprintf(buffer + strlen(buffer), " checked");
	sprintf(buffer + strlen(buffer), ">Apply de-emphasis<br>\r\n");

	sprintf(buffer + strlen(buffer), "<input type=\"hidden\" name=\"filter_pl\" value=\"off\">\r\n");
	sprintf(buffer + strlen(buffer), "<input type=\"checkbox\" name=\"filter_pl\" value=\"on\"");
	if (nxu->filter_pl) sprintf(buffer + strlen(buffer), " checked");
	sprintf(buffer + strlen(buffer), ">Filter CTCSS tones<br>\r\n");



	strcat(buffer, 
			"<input type=\"submit\" value=\"Save Config\">\r\n"
			"</form>\r\n"
			"</body>\r\n"
			"</html>\r\n"
			);
	return;
}



void parse_config(char *buffer, struct nxu_struct *nxu)
{
#define HEXTOI(x) (isdigit(x) ? x - '0' : tolower(x) - 'W')
	char *form_name[128];
	char *form_value[128];
	int num_values=0;
	int i, j;
	int length = strlen(buffer);
	struct hostent *hp;
    int numDevices = Pa_GetDeviceCount();
	const   PaDeviceInfo *deviceInfo;
	int audio_in_device_last = nxu->audio_in_device;
	int audio_out_device_last = nxu->audio_out_device;

	/* parse submitted form into name/value arrays based on & and = delimiters*/
	form_name[num_values] = buffer;
	do {
		form_value[num_values] = form_name[num_values] + strcspn(form_name[num_values], "=");
		*form_value[num_values]++ = 0;
		form_name[num_values+1] = form_value[num_values] + strcspn(form_value[num_values], "&");
		if (*form_name[++num_values]!=0)
			*form_name[num_values]++ = 0;
	} while (*form_name[num_values]);

	/* convert %XX values back to ASCII and '+' to ' ' */
	for (i=j=0; i<=length; j++, i++)
	{
		if (buffer[i] == '%' && (i < length-2) && isxdigit(buffer[i+1]) && isxdigit(buffer[i+2])) {
			buffer[j] = (char) ((HEXTOI(buffer[i+1]) << 4) | HEXTOI(buffer[i+2]));
			i += 2;
		} else if (buffer[i] == '+') {
			buffer[j] = ' ';
		} else {
			buffer[j] = buffer[i];
		}
		/* pad with extra NULL chars so form_name, form_value pointers are still correct */
		if (buffer[i] == '\0' && j<i)
			i--;
		/* remove any double-quote characters (screws up html form) */
		if ((buffer[j] == '"') || (buffer[j] == '\r') || (buffer[j] == '\n'))
			j--;
	}

	nxu->audio_in_device = -1;	// Use default unless a match is found
	nxu->audio_out_device = -1;	// Use default unless a match is found

	/* use name/value arrays to set configuration */
//	nxu->ptt = 0;
	for (i=0; i<num_values; i++)
	{
		if (strcasecmp(form_name[i], "sitename")==0) {
			strncpy(nxu->sitename, form_value[i], 30);
			nxu->sitename[30] = '\0';
		}
		if (strcasecmp(form_name[i], "server_ip")==0) {
			strncpy(nxu->server_ip, form_value[i], 30);
			nxu->server_ip[30] = '\0';
		}
		if (strcasecmp(form_name[i], "server_port")==0) {
			nxu->server_port = atoi(form_value[i]);
		}
		if (strcasecmp(form_name[i], "local_port")==0) {
			nxu->local_port = atoi(form_value[i]);
		}
		// if (strcasecmp(form_name[i], "ptt")==0) {
			// nxu->ptt = 1;
		// }
		if (strcasecmp(form_name[i], "cor_polarity")==0) {
			for (j=0; j<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); j++)
				if (strcasecmp(form_value[i], cor_polarity_strings[j])==0)
					nxu->cor_polarity = j;
		}
		if (strcasecmp(form_name[i], "cal_polarity")==0) {
			for (j=0; j<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); j++)
				if (strcasecmp(form_value[i], cor_polarity_strings[j])==0)
					nxu->cal_polarity = j;
		}
		if (strcasecmp(form_name[i], "ptt_polarity")==0) {
			for (j=0; j<(sizeof(cor_polarity_strings)/sizeof(cor_polarity_strings[0])); j++)
				if (strcasecmp(form_value[i], cor_polarity_strings[j])==0)
					nxu->ptt_polarity = j;
		}

		if (strcasecmp(form_name[i], "audio_input")==0) {
			for( j=0; j<numDevices; j++ ) {
				deviceInfo = Pa_GetDeviceInfo( j );
				if (fnmatch(form_value[i], deviceInfo->name, 0)==0)
					if (deviceInfo->maxInputChannels > 0) {
						nxu->audio_in_device = j;
						printf("Found Audio Input %d %s\n", j, deviceInfo->name);
						break;
					}
			}
		}

		if (strcasecmp(form_name[i], "audio_output")==0) {
			for( j=0; j<numDevices; j++ ) {
				deviceInfo = Pa_GetDeviceInfo( j );
				if (fnmatch(form_value[i], deviceInfo->name, 0)==0)
					if (deviceInfo->maxOutputChannels > 0) {
						nxu->audio_out_device = j;
						printf("Found Audio Output %d %s\n", j, deviceInfo->name);
						break;
					}
			}
		}

		if (strcasecmp(form_name[i], "apply_preemph")==0) {
			if (strcasecmp(form_value[i], "on")==0)
				nxu->apply_preemph = 1;
			else
				nxu->apply_preemph = 0;
		}

		if (strcasecmp(form_name[i], "apply_deemph")==0) {
			if (strcasecmp(form_value[i], "on")==0)
				nxu->apply_deemph = 1;
			else
				nxu->apply_deemph = 0;
		}

		if (strcasecmp(form_name[i], "filter_pl")==0) {
			if (strcasecmp(form_value[i], "on")==0)
				nxu->filter_pl = 1;
			else
				nxu->filter_pl = 0;
		}

	}

	if ((audio_in_device_last != nxu->audio_in_device) || (audio_out_device_last != nxu->audio_out_device)) {
		nxu->restart_portaudio = 1;
		// printf("audio_in_device: %d     audio_out_device: %d\r\n", nxu->audio_in_device, nxu->audio_out_device);
	}

	/* DNS lookup IP if needed */
	// use getaddrinfo() for ip6 compatibility
	if ((nxu->udp_server.sin_addr.s_addr = inet_addr(nxu->server_ip)) == htonl(INADDR_NONE)) {
		if ((hp = gethostbyname(nxu->server_ip)) == NULL) {
			fprintf(stderr, "Could not get server IP from host name \"%s\".\n", nxu->server_ip);
		} else {
			memcpy(&(nxu->udp_server.sin_addr.s_addr), hp->h_addr, hp->h_length);
			// printf("%s is at %s\r\n", nxu->server_ip, inet_ntoa(nxu->udp_server.sin_addr));
		}
	}
}



// This function will be called by mongoose on every new HTTP request.
static int mongoose_request_handler(struct mg_connection *conn)
{
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	struct mg_user_data *user_data = request_info->user_data;
	int retval = 0;
	FILE *cfgfile;
	time_t current_time;
	char post_buf[4096];
	char var_string[32];
	int content_len, post_len;
	const char *content_len_str;

	char html_config_saved[] = 
	"<!DOCTYPE html>\n<html>\n"
	"<head>\n"
	"<title>PI DXU Config Saved</title>\n"		// use myname
	"</head>\n"
	"<body>\n"
	"<br><br><h1 style=\"text-align:center;\">Configuration successfully saved!</h1>\n"
	"<br><h2 style=\"text-align:center;\"><a href=/index.html>Status</a>  "
	"<br><a href=/config.html>Configuration</a></h2>\n"
	"</body>\n"
	"</html>\n";

	if(!strcasecmp(request_info->request_method, "GET"))
	{
		if(		!strcasecmp(request_info->uri, "/")
			 || !strcasecmp(request_info->uri, "/INDEX.HTM")
			 || !strcasecmp(request_info->uri, "/INDEX.HTML")
			 || !strcasecmp(request_info->uri, "/STATUS.HTM")
			 || !strcasecmp(request_info->uri, "/STATUS.HTML") )
		{
			mg_printf(conn,
					"HTTP/1.0 200 OK\r\n"
					"Content-Type: text/html\r\n"
					"Content-Length: %d\r\n"        // Always set Content-Length
					"\r\n"
					"%s",
					strlen(user_data->status_buffer), user_data->status_buffer);
			retval = 1;
		}
		else if(!strcasecmp(request_info->uri, "/CONFIG.HTM")
			 || !strcasecmp(request_info->uri, "/CONFIG.HTML") )
		{
			mg_printf(conn,
					"HTTP/1.0 200 OK\r\n"
					"Content-Type: text/html\r\n"
					"Content-Length: %d\r\n"        // Always set Content-Length
					"\r\n"
					"%s",
					strlen(user_data->config_buffer), user_data->config_buffer);
			retval = 1;
		}
		else if(!strcasecmp(request_info->uri, "/PORTAUDIO.HTM")
			 || !strcasecmp(request_info->uri, "/PORTAUDIO.HTML") )
		{
			mg_printf(conn,
					"HTTP/1.0 200 OK\r\n"
					"Content-Type: text/html\r\n"
					"Content-Length: %d\r\n"        // Always set Content-Length
					"\r\n"
					"%s",
					strlen(user_data->portaudio_buffer), user_data->portaudio_buffer);
			retval = 1;
		}
	}
	else if(!strcasecmp(request_info->request_method, "POST"))
	{
		if(		!strcasecmp(request_info->uri, "/")
			 || !strcasecmp(request_info->uri, "/INDEX.HTM")
			 || !strcasecmp(request_info->uri, "/INDEX.HTML")
			 || !strcasecmp(request_info->uri, "/STATUS.HTM")
			 || !strcasecmp(request_info->uri, "/STATUS.HTML") )
		{
			content_len = -1;
			if ((content_len_str = mg_get_header(conn, "Content-Length")) != NULL)
				content_len = atoi(content_len_str);

			if (content_len > 0) {
				if ((post_len = mg_read(conn, post_buf, sizeof(post_buf)-1)) == content_len) {
					if (mg_get_var(post_buf, post_len, "ptt", var_string, sizeof(var_string)) >= 0) {
						if (!strcasecmp("on", var_string))
							user_data->nxu->ptt = 1;
						else
							user_data->nxu->ptt = 0;
					}
					if (mg_get_var(post_buf, post_len, "cal", var_string, sizeof(var_string)) >= 0) {
						if (!strcasecmp("on", var_string))
							user_data->nxu->cal = 1;
						else
							user_data->nxu->cal = 0;
					}
					wwwstatus(user_data->status_buffer, user_data->nxu);
				} else {/* have incorrect data size */
					fprintf(stdout, "Incomplete POST data, unable to process!!!\n");
				}
			}
			mg_printf(conn,
					"HTTP/1.0 200 OK\r\n"
					"Content-Type: text/html\r\n"
					"Content-Length: %d\r\n"        // Always set Content-Length
					"\r\n"
					"%s",
					strlen(user_data->status_buffer), user_data->status_buffer);
			retval = 1;
		}
		if (strcasecmp(request_info->uri, "/SAVECONFIG.ASP")==0)
		{
			int fail_count = 0;

			content_len = -1;
			if ((content_len_str = mg_get_header(conn, "Content-Length")) != NULL)
				content_len = atoi(content_len_str);

			if ((post_len = mg_read(conn, post_buf, sizeof(post_buf)-1)) != content_len)
			{
				fprintf(stdout, "Incomplete POST data, unable to process!!!\n");
			}
			else /* have correct data size */
			{
				while (!(cfgfile = fopen(user_data->cfgfname, "w")) && fail_count++<3)
				{
					fprintf(stdout, "Error opening config file for writing (%d), retrying...\n", fail_count);
				}

				if (cfgfile)
				{
					current_time = time(NULL);
					unsigned long addr = htonl(request_info->remote_ip);
					fprintf(stdout, "Saved Config by %s from %s at %s", request_info->remote_user, inet_ntoa(*((struct in_addr*)&addr)), ctime(&current_time));
					fprintf(cfgfile, "%s\n", post_buf);
					fclose(cfgfile);
				}

				*(post_buf + post_len) = 0;
				parse_config(post_buf, user_data->nxu);
				wwwstatus(user_data->status_buffer, user_data->nxu);
				wwwconfig(user_data->config_buffer, user_data->nxu);
				mg_printf(conn,
						"HTTP/1.0 200 OK\r\n"
						"Content-Type: text/html\r\n"
						"Content-Length: %d\r\n"        // Always set Content-Length
						"\r\n"
						"%s",
						strlen(html_config_saved), html_config_saved);
				retval = 1;
			}
		}
	}

	// Returning non-zero tells mongoose that our function has replied to
	// the client, and mongoose should not send client any more data.
	return retval;
}


/* initialize structure to zero when program starts */
void init_nxu(struct nxu_struct *nxu)
{
	memset((void *)nxu, '\0', sizeof(struct nxu_struct));
	// nxu->buffer_in = malloc(NUMBUFFERS*1024*sizeof(char));
}


/* called when disconnecting */
/* clear out all client related data, save server info and buffers */
void reset_nxu(struct nxu_struct *nxu)
{
	/* store persistent server values */
	struct sockaddr_in udp_client;
	struct sockaddr_in udp_server;
	char *sitename = strdup(nxu->sitename);
	char *server_ip = strdup(nxu->server_ip);
	char *http_port_str = strdup(nxu->http_port_str);
	unsigned short server_port = nxu->server_port;
	unsigned short local_port = nxu->local_port;
	unsigned char cor_polarity = nxu->cor_polarity;
	unsigned char cal_polarity = nxu->cal_polarity;
	unsigned char ptt_polarity = nxu->ptt_polarity;
	unsigned char apply_preemph = nxu->apply_preemph;
	unsigned char apply_deemph = nxu->apply_deemph;
	unsigned char filter_pl = nxu->filter_pl;
	// unsigned char *buffer_in = nxu->buffer_in;
	int audio_in_device = nxu->audio_in_device;
	int audio_out_device = nxu->audio_out_device;
	int restart_portaudio = nxu->restart_portaudio;

	udp_client.sin_port = nxu->udp_client.sin_port;
	udp_client.sin_addr = nxu->udp_client.sin_addr;
	udp_server.sin_port = nxu->udp_server.sin_port;
	udp_server.sin_addr = nxu->udp_server.sin_addr;

//	flush_list(nxu->buffer);		// free any remaining memory buffers

	/* clear entire struct */
	memset((void *)nxu, '\0', sizeof(struct nxu_struct));
	
	/* Set family=internet */
	nxu->udp_server.sin_family = AF_INET;
	nxu->tcp_server.sin_family = AF_INET;
	nxu->udp_client.sin_family = AF_INET;

	/* restore persistent server values */
	nxu->udp_client.sin_port = udp_client.sin_port;
	nxu->udp_client.sin_addr = udp_client.sin_addr;
	nxu->udp_server.sin_port = udp_server.sin_port;
	nxu->udp_server.sin_addr = udp_server.sin_addr;
	strcpy(nxu->sitename, sitename); free(sitename);
	strcpy(nxu->server_ip, server_ip); free(server_ip);
	strcpy(nxu->http_port_str, http_port_str); free(http_port_str);
	nxu->server_port = server_port;
	nxu->local_port = local_port;
	nxu->cor_polarity = cor_polarity;
	nxu->cal_polarity = cal_polarity;
	nxu->ptt_polarity = ptt_polarity;
	nxu->apply_preemph = apply_preemph;
	nxu->apply_deemph = apply_deemph;
	nxu->filter_pl = filter_pl;
	// nxu->buffer_in = buffer_in;
	nxu->audio_in_device = audio_in_device;
	nxu->audio_out_device = audio_out_device;
	nxu->restart_portaudio = restart_portaudio;
}


int connect_nxu(struct nxu_struct *nxu)
{
	socklen_t client_length = sizeof(struct sockaddr_in);
	struct hostent *hp;

	/* re-initialize structure */
	reset_nxu(nxu);
	nxu->connected = 0;

	/* Set family=internet */
	nxu->udp_server.sin_family = AF_INET;
	nxu->tcp_server.sin_family = AF_INET;
	nxu->udp_client.sin_family = AF_INET;

	nxu->udp_server.sin_port = htons(nxu->server_port);
	nxu->tcp_server.sin_port = htons(nxu->server_port);
	nxu->udp_client.sin_port = htons(nxu->local_port);

	/* DNS lookup IP if needed */
	// use getaddrinfo() for ip6 compatibility
	if ((nxu->udp_server.sin_addr.s_addr = inet_addr(nxu->server_ip)) == htonl(INADDR_NONE)) {
		if ((hp = gethostbyname(nxu->server_ip)) == NULL) {
			fprintf(stderr, "Could not get server IP from host name %s\r\n", nxu->server_ip);
		} else {
			memcpy(&(nxu->udp_server.sin_addr.s_addr), hp->h_addr, hp->h_length);
			// printf("%s is at %s\r\n", nxu->server_ip, inet_ntoa(nxu->udp_server.sin_addr));
		}
	}
	nxu->tcp_server.sin_addr.s_addr = nxu->udp_server.sin_addr.s_addr;

	/* Open a TCP socket */
	nxu->stcp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (nxu->stcp == INVALID_SOCKET) {
		fprintf(stderr, "Could not create TCP socket.\n");
		cleanexitptr(0);
	}
	/* Connect TCP to the server address/port */
	if (connect(nxu->stcp, (struct sockaddr *) &nxu->tcp_server, client_length) < 0) {
		/* failed so force retry */
		nxu->tcp_server.sin_addr.s_addr = 0;
		return -1;
	} else {
		nxu->connected = 1;
		nxu->connect_time = time(NULL);
		printf("Connected to TCP/IP server %s:%d at %s",
				nxu->server_ip, nxu->server_port, ctime(&nxu->connect_time));
	}


	/* Open a UDP socket */
	nxu->sudp = socket(AF_INET, SOCK_DGRAM, 0);
	if (nxu->sudp == INVALID_SOCKET) {
		fprintf(stderr, "Could not create UDP socket.\n");
		cleanexitptr(0);
	}
	/* Bind UDP address to socket */
	if (bind(nxu->sudp, (struct sockaddr *)&nxu->udp_client, client_length) == -1) {
		nxu->udp_client.sin_port = 0;
		if (bind(nxu->sudp, (struct sockaddr *)&nxu->udp_client, client_length) == -1) {
			fprintf(stderr, "Could not bind socket to UDP port.  Error %d\n", WSAGetLastError());
			cleanexitptr(0);
		}
	}

	printf("My UDP port is %d\r\n", ntohs(nxu->udp_client.sin_port));
	return 0;
}

void parse_args(int argc, char *argv[], struct nxu_struct *nxu)
{
	/* command line options override config file */
	if (argc == 2)
	{
		nxu->server_port = atoi(argv[1]);
		nxu->local_port = nxu->server_port+1000;
	}
	if (argc == 3)
	{
		strncpy(nxu->server_ip, argv[1], 30);
		nxu->server_ip[30] = '\0';
		nxu->server_port = atoi(argv[2]);
		nxu->local_port = nxu->server_port+1000;
	}
	if (argc == 4)
	{
		strncpy(nxu->server_ip, argv[1], 30);
		nxu->server_ip[30] = '\0';
		nxu->server_port = atoi(argv[2]);
		nxu->local_port = nxu->server_port+1000;
	}
	if (argc > 4)
	{
		printf("\r\n\r\nIncorrect arguments\r\n");
		exit(0);
	}

}

/* ---------- de-emphasis ----------
 * 300-3000 Hz 6dB per octave +/- 1dB
 * de-emphasis lpf shift 6 bits,  hpf shift 13 bits
 * de-emphasis actual 3dB freq 119.371 Hz, gain 8
 * not re-entrant due to static vars */
int deemphasis(int input)
{
	static int last_w_hpf = 0;
	static int last_w_lpf = 0;
	int temp;

	temp = input - iir_lpf(&last_w_hpf, input, 13);		// dc block
//	temp = iir_lpf(&last_w_lpf, temp<<3, 6);			// de-emphasis curve, 120 Hz, gain 2^3
	temp = iir_lpf(&last_w_lpf, temp<<2, 5);			// de-emphasis curve, 240 Hz, gain 2^2
	return temp;
}

/* ---------- pre-emphasis ----------
 * 300-3000 Hz 6dB per octave +/- 1dB
 * pre-emphasis hpf shift 13 bits,  hpf2 shift 6 bits
 * pre-emphasis actual 3dB freq 119.371 Hz, gain 0.125
 * not re-entrant due to static vars */
int preemphasis(int input)
{
	static int last_w_hpf = 0;
	static int last_in_pd = 0;
	int temp;

	temp = input - iir_lpf(&last_w_hpf, input, 13);		// dc block
//	temp = iir_pd(&last_in_pd, temp, 6) >> 3;			// pre-emphasis curve, 120 Hz, gain 2^-3
	temp = iir_pd(&last_in_pd, temp, 5) >> 2;			// pre-emphasis curve, 240 Hz, gain 2^-2

	// clip to max/min short (16 bits)
	if (temp > SHRT_MAX)
		temp = SHRT_MAX;
	else if (temp < SHRT_MIN)
		temp = SHRT_MIN;

	return temp;
}

/* proportional + derivative
 * unity gain from 0 Hz to corner freq, increases 6dB/octave after corner freq */
int iir_pd(int *last_in, int input, int rshift)
{
	int out;
	/* y(n) = x(n) + x(n)<<rshift - x(n-1)<<rshift */
	out = input + (input<<rshift) - ((*last_in)<<rshift);
	*last_in = input;

	return out;
}

/* low pass filter
 * unity gain from 0 Hz to corner freq, decreases 6dB/octave after corner freq */
int iir_lpf(int *last_w, int input, int lshift)
{
	int temp;

	/* w(n) = w(n-1) - w(n-1)>>lshift + x(n)
	 * y(n) = x(n) - w(n)>>lshift */
	*last_w = *last_w - ((*last_w) >> lshift) + input;
	temp = *last_w >> lshift;

	// clip to max/min short (16 bits)
	if (temp > SHRT_MAX) {
		*last_w = SHRT_MAX << lshift;
		return SHRT_MAX;
	} else if (temp < SHRT_MIN) {
		*last_w = SHRT_MIN << lshift;
		return SHRT_MIN;
	} else {
		return temp;
	}
}


int iir_biquad(struct iir_state *state, int input)
{
	int temp;
	short out;

	temp = input << 12;
	temp += (int)state->b[0] * state->in[state->index];
	temp += (int)state->b[1] * state->in[(state->index-1)&3];
	temp -= (int)state->a[0] * state->out[state->index];
	temp -= (int)state->a[1] * state->out[(state->index-1)&3];

	out = temp >> 12;
	state->index = (state->index+1)&3;
	state->in[state->index] = input;
	state->out[state->index] = out;
	return(out);
}

/* Chebyshev Type II filter from iir_pl_cheby_cascade.m
 * [b a] = cheby2(10, 40, 250/4000, 'high');
 * [sos,g] = tf2sos(b,a);		*/
void pl_filt_init(struct iir_state **state, char *num_stages)
{
	*num_stages = 5;
	if (*state==NULL)
		*state = malloc(5*sizeof(struct iir_state));
	memset(*state, 0, 5*sizeof(struct iir_state));

	(*state)[0].b[0] = -8113;
	(*state)[0].b[1] = 4096;
	(*state)[0].a[0] = -7490;
	(*state)[0].a[1] = 3512;

	(*state)[1].b[0] = -8159;
	(*state)[1].b[1] = 4096;
	(*state)[1].a[0] = -7643;
	(*state)[1].a[1] = 3713;

	(*state)[2].b[0] = -8038;
	(*state)[2].b[1] = 4096;
	(*state)[2].a[0] = -7348;
	(*state)[2].a[1] = 3300;

	(*state)[3].b[0] = -8188;
	(*state)[3].b[1] = 4096;
	(*state)[3].a[0] = -7858;
	(*state)[3].a[1] = 3960;

	(*state)[4].b[0] = -8067;
	(*state)[4].b[1] = 4096;
	(*state)[4].a[0] = -7394;
	(*state)[4].a[1] = 3372;
}

void pl_filt_free(struct iir_state **state)
{
	free(*state);
	*state = NULL;
}

short pl_filt(struct iir_state state[], char *num_stages, short input)
{
	int temp[*num_stages];
	int i;
	
	i = (int)input;
	temp[0] = iir_biquad(&state[0], i);
	for (i=1; i<*num_stages; i++) {
		temp[i] = iir_biquad(&state[i], temp[i-1]);
	}

	// apply gain from filter design here:  0.705987348
	i = (2892 * temp[*num_stages-1]) >> 12;

	if (i > SHRT_MAX)
		i = SHRT_MAX;
	else if (i < SHRT_MIN)
		i = SHRT_MIN;

	return (short)i;
}



#ifdef __arm__
// Set up a memory regions to access GPIO
void setup_gpio()
{
	int  mem_fd;
	void *gpio_map;

	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = mmap(
			NULL,             //Any adddress in our space will do
			BLOCK_SIZE,       //Map length
			PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
			MAP_SHARED,       //Shared with other processes
			mem_fd,           //File to map
			GPIO_BASE         //Offset to GPIO peripheral
			);

	close(mem_fd); //No need to keep mem_fd open after mmap

	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (int)gpio_map);//errno also set!
		exit(-1);
	}

	// Always use volatile pointer!
	gpio = (volatile unsigned *)gpio_map;
} // setup_gpio
#endif // __arm__

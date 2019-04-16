#include "make_tls_server.h"
#include "tls_common_lib.h"
#include "netconstants.h"
#include "constants.h"
#include "packet.h"
#include "serial.h"
#include "serialize.h"

// Raspicam
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>

//RPi GPIO
#include <wiringPi.h>

/* TODO: Set PORT_NAME to the port name of your Arduino */
#define PORT_NAME			"/dev/ttyACM0"
/* END TODO */

#define BAUD_RATE			B57600

// TLS Port Number
#define SERVER_PORT			5000

/* TODO: #define constants for the  filenames for Alex's private key, certificate, CA certificate name,
   and the Common Name for your laptop */
#define serverPrivateKey "alex.key"
#define serverCertFname "alex.crt"
#define caCertFname "signing.pem"

//#define clientNameOnCert "322_laptop.lol.com" //Outdated
//#define clientNameOnCert "www.322_laptop.com" //CAA 080419
#define clientNameOnCert "www.laptoping.com" //CAA 090419

/* END TODO */

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN	129

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.

static volatile int networkActive;

// This variable is used by sendNetworkData to send back responses
// to the TLS connection.  It is sent by handleNetworkData

static void *tls_conn = NULL;

/*

   Alex Serial Routines to the Arduino

 */

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

typedef struct Pixel{
    unsigned int red;
    unsigned int blue;
    unsigned int green;
} Pixel;

Pixel image[960][1280];
raspicam::RaspiCam Camera; //Camera object
int res[2] = {0, 0};

void parse(unsigned char *data){
    for (int i=0;i<960;i++){
        for(int j=0; j<1280; j++){
            Pixel p;
            p.red = (unsigned int)data[1280*3*i+3*j];
            p.green = (unsigned int)data[1280*3*i+3*j+1];
            p.blue = (unsigned int)data[1280*3*i+3*j+2];
            image[i][j] = p;
        }
    }
}

bool check_green(Pixel p){
  //  return (p.green > 80 && p.red < 20); // && p.blue < 60);
  return (p.green > 100  && p.red <80&& p.blue < 80);
}

bool check_red(Pixel p){
    return (p.red > 110 && p.green < 60 && p.blue < 60);
}

void check_object(){
    for (int j=0; j<1280; j++){
        for (int i=0; i<960; i++){
            Pixel p = image[i][j];
//            std::cout << p.red << ", " << p.green << ", " << p.blue << std::endl;
            if (check_green(p)){
                res[0] = 1;
		return;
                //return objects;
            }
            else if (check_red(p)){
                res[1] = 1;
		return;
                //return objects;
            }    
        }
    }
}

void handleErrorResponse(TPacket *packet)
{
    printf("UART ERROR: %d\n", packet->command);
    char buffer[2];
    buffer[0] = NET_ERROR_PACKET;
    buffer[1] = packet->command;
    sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet)
{
    char data[33];
    printf("UART MESSAGE PACKET: %s\n", packet->data);
    data[0] = NET_MESSAGE_PACKET;
    memcpy(&data[1], packet->data, sizeof(packet->data));
    sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet)
{
    char data[65];
    printf("UART STATUS PACKET\n");
    data[0] = NET_STATUS_PACKET;
    memcpy(&data[1], packet->params, sizeof(packet->params));
    sendNetworkData(data, sizeof(data));
}

void handleResponse(TPacket *packet)
{
// The response code is stored in command
    switch(packet->command)
    {
        case RESP_OK:
            char resp[2];
            printf("Command OK\n");
            resp[0] = NET_ERROR_PACKET;
            resp[1] = RESP_OK;
            sendNetworkData(resp, sizeof(resp));
            break;

        case RESP_STATUS:
            handleStatus(packet);
            break;

        default:
            printf("Boo\n");
    }
}


void handleUARTPacket(TPacket *packet)
{
    switch(packet->packetType)
    {
        case PACKET_TYPE_COMMAND:
            // Only we send command packets, so ignore
            break;

        case PACKET_TYPE_RESPONSE:
            handleResponse(packet);
            break;

        case PACKET_TYPE_ERROR:
            handleErrorResponse(packet);
            break;

        case PACKET_TYPE_MESSAGE:
            handleMessage(packet);
            break;
    }
}


void uartSendPacket(TPacket *packet)
{
    char buffer[PACKET_SIZE];
    int len = serialize(buffer, packet, sizeof(TPacket));

    serialWrite(buffer, len);
}

void handleError(TResult error)
{
    switch(error)
    {
        case PACKET_BAD:
            printf("ERROR: Bad Magic Number\n");
            break;

        case PACKET_CHECKSUM_BAD:
            printf("ERROR: Bad checksum\n");
            break;

        default:
            printf("ERROR: UNKNOWN ERROR\n");
    }
}

void *uartReceiveThread(void *p)
{
    char buffer[PACKET_SIZE];
    int len;
    TPacket packet;
    TResult result;
    int counter=0;

    while(1)
    {
        len = serialRead(buffer);
        counter+=len;
        if(len > 0)
        {
            result = deserialize(buffer, len, &packet);

            if(result == PACKET_OK)
            {
                counter=0;
                handleUARTPacket(&packet);
            }
            else 
                if(result != PACKET_INCOMPLETE)
                {
                    printf("PACKET ERROR\n");
                    handleError(result);
                } // result
        } // len > 0
    } // while
}

/*

   Alex Network Routines

 */


void sendNetworkData(const char *data, int len)
{
    // Send only if network is active
    if(networkActive)
    {
        // Use this to store the number of bytes actually written to the TLS connection.
        int c;

        printf("WRITING TO CLIENT\n");

        if(tls_conn != NULL) {
            /* TODO: Implement SSL write here to write data to the network. Note that
               handleNetworkData should already have set tls_conn to point to the TLS
               connection we want to write to. */
               
               c = sslWrite(tls_conn, data, sizeof(data));
            /* END TODO */

        }

        // Network is still active if we can write more then 0 bytes.
        networkActive = (c > 0);
    }
}

void handleCommand(void *conn, const char *buffer)
{
    // The first byte contains the command
    char cmd = buffer[1];
    uint32_t cmdParam[2];

    // Copy over the parameters.
    memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

    TPacket commandPacket;

    commandPacket.packetType = PACKET_TYPE_COMMAND;
    commandPacket.params[0] = cmdParam[0];
    commandPacket.params[1] = cmdParam[1];

    printf("COMMAND RECEIVED: %c %d %d\n", cmd, cmdParam[0], cmdParam[1]);

    switch(cmd)
    {
        case 'f':
        case 'F':
        {
            commandPacket.command = COMMAND_FORWARD;
            uartSendPacket(&commandPacket);
            break;
        }
        case 'b':
        case 'B':
        {
            commandPacket.command = COMMAND_REVERSE;
            uartSendPacket(&commandPacket);
            break;
        }
        case 'l':
        case 'L':
        {
            commandPacket.command = COMMAND_TURN_LEFT;
            uartSendPacket(&commandPacket);
            break;
        }
        case 'r':
        case 'R':
        {
            commandPacket.command = COMMAND_TURN_RIGHT;
            uartSendPacket(&commandPacket);
            break;
        }
        case 's':
        case 'S':
        {
            commandPacket.command = COMMAND_STOP;
            uartSendPacket(&commandPacket);
            break;
        }
        case 'c':
        case 'C':
        {
            commandPacket.command = COMMAND_CLEAR_STATS;
            commandPacket.params[0] = 0;
            uartSendPacket(&commandPacket);
            break;
        }
        case 'g':
        case 'G':
        {
            commandPacket.command = COMMAND_GET_STATS;
            uartSendPacket(&commandPacket);
            break;
        }
	case 'M':
	case 'm':
	{
	    digitalWrite(2, LOW);
	    usleep(1000000);
	    digitalWrite(2, HIGH);
	    break;
	}
        case 'k':
        case 'K':
        {
	    Camera.setAWB_RB(1,0.5);
	    if ( !Camera.open()) {std::cerr<<"Error opening camera"<<std::endl;}
	    usleep(3*1000000); 
	    Camera.grab();
	    unsigned char *cam_data = new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)];
	    Camera.retrieve(cam_data,raspicam::RASPICAM_FORMAT_RGB);
	    for (int i=0; i<Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )/3; i++){
		    //BGR -> RGB
		    unsigned char temp = cam_data[i*3];
		    cam_data[i*3]=cam_data[i*3+2];
		    cam_data[i*3+2]=temp;
		}
	    parse(cam_data);
	    check_object();
	    
	    if (res[0]==1){
		printf("Green object identified.\n");
		char data[33];
		data[0] = NET_MESSAGE_PACKET;
		data[1] = 'g';
		data[2] = 'r';
		data[3] = 'e';
		data[4] = 'e';
		data[5] = 'n';
		sendNetworkData(data, sizeof(data));
	    }
	    else if (res[1]==1){
		printf("Red object identified.\n");
		char data[33];
		data[0] = NET_MESSAGE_PACKET;
		data[1] = 'r';
		data[2] = 'e';
		data[3] = 'd';
		sendNetworkData(data, sizeof(data));
	    }
            else{
                printf("Unknown color!\n");
                char data[33];
                data[0] = NET_MESSAGE_PACKET;
                data[1] = 'o';
                data[2] = 'o';
                data[3] = 'f';
                sendNetworkData(data, sizeof(data));
            }
	    std::ofstream outFile ( "raspicam_image.ppm",std::ios::binary );
	    outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
	    outFile.write ( ( char* ) cam_data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
	    std::cout<<"Image saved at raspicam_image.ppm"<<std::endl;
            res[0] = 0; //reset res
            res[1] = 0;
	    delete cam_data;
            break;
        }
        default:
            printf("Bad command\n");

    }
}

void handleNetworkData(void *conn, const char *buffer, int len)
{
    /* Note: A problem with our design is that we actually get data to be written
       to the SSL network from the serial port. I.e. we send a command to the Arduino,
       get back a status, then write to the TLS connection.  So we do a hack:
       we assume that whatever we get back from the Arduino is meant for the most
       recent client, so we just simply store conn, which contains the TLS
       connection, in a global variable called tls_conn */

    tls_conn = conn; // This is used by sendNetworkData

    if(buffer[0] == NET_COMMAND_PACKET)
        handleCommand(conn, buffer);
}

void *worker(void *conn)
{
    int len;

    char buffer[BUF_LEN];

    while(networkActive)
    {
        /* TODO: Implement SSL read into buffer */
        len = sslRead(conn, buffer, sizeof(buffer)); //CAA 090419

        /* END TODO */
        // As long as we are getting data, network is active
        networkActive=(len > 0);

        if(len > 0)
            handleNetworkData(conn, buffer, len);
        else
            if(len < 0)
                perror("ERROR READING NETWORK: ");
    }

    // Reset tls_conn to NULL.
    tls_conn = NULL;
    EXIT_THREAD(conn);
}


void sendHello()
{
    // Send a hello packet
    TPacket helloPacket;

    helloPacket.packetType = PACKET_TYPE_HELLO;
    uartSendPacket(&helloPacket);
}

int main()
{	
    usleep(1000000); //Sleep for one second;
	    
    wiringPiSetup(); //sets up GPIO for Arduino reset
    pinMode (2, OUTPUT);
    digitalWrite(2, HIGH); // set it up as HIGH; LOW will cause reset
    // Start the uartReceiveThread. The network thread is started by
    // createServer

    pthread_t serThread;

    printf("\nALEX REMOTE SUBSYSTEM\n\n");

    printf("Opening Serial Port\n");
    // Open the serial port
    startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
    printf("Done. Waiting 3 seconds for Arduino to reboot\n");
    sleep(3);

    printf("DONE. Starting Serial Listener\n");
    pthread_create(&serThread, NULL, uartReceiveThread, NULL);

    printf("Starting Alex Server\n");

    networkActive = 1;

    /* TODO: Call createServer with the necessary parameters to do client authentication and to send
       Alex's certificate. Use the #define names you defined earlier  */
    //createServer("alex.key", "alex.crt",SERVER_PORT, worker, "signing.pem", "322_laptop.lol.com",1);
    
    createServer(serverPrivateKey, serverCertFname, SERVER_PORT, &worker, caCertFname, clientNameOnCert, 1);

    /* TODO END */

    printf("DONE. Sending HELLO to Arduino\n");
    sendHello();
    printf("DONE.\n");


    // Loop while the server is active
    while(server_is_running());
}

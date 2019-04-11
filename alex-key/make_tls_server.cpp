#include <signal.h>
#include "make_tls_server.h"


// Exit flag.  We quit the listener loop when this is 1
static volatile int _exitFlag = 0;

// CTRL-C handler
void breakHandler(int dummy) {
    printf("\n\nEXITING. WHY DID YOU PRESS CTRL-C??\n\n");
    _exitFlag = 1;
}

// SIGTERM handler
void termHandler(int dummy) {
    printf("\n\nEXITING. WHY DID YOU TERMINATE ME??\n\n");
    _exitFlag = 1;
}

// SIGKILL handler
void killHandler(int dummy) {
    printf("\n\nEXITING. YOU KILLER!\n\n");
    _exitFlag = 1;
}
// Maximum length of a filename, including
// path.

#define MAX_FILENAME_LEN        128
#define MAX_DOMAIN_NAME_LEN     128

// Thread structure for the listener thread
static pthread_t _listener;

// This variable points to the worker thread, and _tls_listener
// uses it to spawn the new worker thread

static void *(*_worker_thread)(void *);

// Our port number
static int _port_num;

// Our private key and certificate filenames
static char _key_filename[MAX_FILENAME_LEN];
static char _cert_filename[MAX_FILENAME_LEN];

// Set whether or not to do verification
static int _verify_peer = 0;

// Our CA's cert name. 
static char _ca_cert_filename[MAX_FILENAME_LEN];

// Client's  name, as entered into the FQDN field
// of their certificate
static char _client_name[MAX_DOMAIN_NAME_LEN];

// This function is the internal server listener loop. It
// creates a new socket, then listens to it, then spawns
// a worker thread if there is a new connection.

static void *_tls_listener(void *dummy) {

	// Declare two integer variables that will
	// point to sockets.
	int listenfd, connfd;

	// serv_addr will be used to configure the port number
	// of our server.
	struct sockaddr_in serv_addr;

	// Set every element in serv_addr to 0.
	memset(&serv_addr, 0, sizeof(serv_addr));

	// Open up a TCP/IP (AF_INET) socket, using the reliable TCP
	// protocol (SOCK_STREAM). You can also create a best effort UDP socket
	// by specifying SOCK_DGRAM instead of SOCK_STREAM. The "0" means
	// use the first protocol in the AF_INET family. The AF_INET family
	// has only one protocol so this is always 0.
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	// We use perror to print out error messages
	if(listenfd < 0)
	{
		perror("Unable to create socket: ");
		exit(-1);
	}

	// Configure our server to bind to all interfaces (INADDR_ANY)
	// including all network cards and WiFi ports. 
    // Our port is indicated in _port_num

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(_port_num);
    
    // We set REUSEADDR so that we can reuse port numbers
    // and avoid "Address in use" errors.
    int one = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	// Now actually bind our socket to port 5000
	if(bind(listenfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Unable to bind: ");
		exit(-1);
	}

	// And start listening for connections. We maintain a FIFO
	// queue of 10 entries of "unaccepted" connections. I.e.
	// connections pending acceptance using the accept() function below.

	printf("Now listening..\n");
	if(listen(listenfd, 10) < 0)
	{
		perror("Unable to listen to port: ");
		exit(-1);
	}

	int c = sizeof(struct sockaddr_in);

	// NEW: Initialize the SSL library
	init_openssl();

	// NEW: Create SSL Context.
	SSL_CTX *ctx = create_context(_ca_cert_filename, _verify_peer, 1);

	// NEW: Configure context to load certificate and private keys
	configure_context(ctx, _cert_filename, _key_filename);

	// NEW: Configure multithreading in OpenSSL
	CRYPTO_thread_setup();

    // This exit variable is declared globally and is set
    // by the CTRL-C handler.
	while(!_exitFlag)
	{
		// Accept a new connection from the queue in listen. We will
		// build an echo server
		struct sockaddr_in client;
		connfd = accept(listenfd, (struct sockaddr *) &client, (socklen_t *) &c);

		char clientAddress[32];

		// Use inet_ntop to extract client's IP address.
		inet_ntop(AF_INET, &client.sin_addr, clientAddress, 32);

		printf("Received connection from %s\n", clientAddress);

        SSL *ssl;

        if(_verify_peer) {
		    ssl = connectSSL(ctx, connfd, _client_name);
        } else {
		    ssl = connectSSL(ctx, connfd, NULL);
        }

		if(ssl != NULL)
		{
            int spawn = 1;

            if(_verify_peer) {
                printCertificate(ssl);

                if(!verifyCertificate(ssl)) {
                    printf("Certificate error for %s\n", clientAddress);
                    spawn = 0;
                }
                else
                    printf("SSL CLIENT CERTIFICATE IS VALID.\n");
            }

            if(spawn)
            {
                pthread_t worker;

                // NEW: We pass in ssl instead of connfd
                pthread_create(&worker, NULL, _worker_thread, (void *) ssl);
                pthread_detach(worker);
            }

		}
	}

	// We reach here if our program exits
	close(listenfd);
	SSL_CTX_free(ctx);
	thread_cleanup();
	cleanup_openssl();
}

// This function creates a new TLS server. It spawns a new
// thread to run the server listener loop, and thus
// it must be run together with (but not within) some form
// of infinite loopto prevent the creation of orphaned
// or zombie threads.
// keyFilename = Server's private key filename
// certFilename = Server's certificate filename
// portNum = Port number to listen to
// workerThread = Pointer to worker thread
// caCertFilename = CA's certificate filename. Needed only if verifyPeer is true.
// verifyPeer = Set to true to force verification of client's identity

void createServer(const char *keyFilename, const char *certFilename, int portNum, void *(*workerThread)(void *), const char *caCertFilename, const char *clientName, int verifyPeer) {

    // Save the pointer to the worker thread and the
    // port number.  These will be used by _tls_listener.

    _worker_thread = workerThread;
    _port_num = portNum;

    // Save the private key and certificate filenames, also used by
    // _tls_listener.

    strncpy(_key_filename, keyFilename, MAX_FILENAME_LEN);
    strncpy(_cert_filename, certFilename, MAX_FILENAME_LEN);

    // If we want to verify the client, copy over the CA's certificate filename,
    // which must be present in the current directory. Used by _tls_listener.

    _verify_peer = verifyPeer;
    if(verifyPeer) {
        strncpy(_ca_cert_filename, caCertFilename, MAX_FILENAME_LEN);
        strncpy(_client_name, clientName, MAX_DOMAIN_NAME_LEN);
    }

    // Set the handlers
    signal(SIGINT, breakHandler);
    signal(SIGTERM, termHandler);
    signal(SIGKILL, killHandler);

    // Now spawn the listener thread

    printf("\n** Spawning TLS Server **\n\n");
    pthread_create(&_listener, NULL, _tls_listener, NULL);
    pthread_detach(_listener);
}

// Returns TRUE if the TLS listener loop is still running.
int server_is_running() {
    return !_exitFlag;
}

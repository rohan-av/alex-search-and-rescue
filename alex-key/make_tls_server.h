#ifndef __MAKE_TLS_SERVER__
#define __MAKE_TLS_SERVER__

#include  <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include "tls_server_lib.h"
#include "tls_common_lib.h"
#include "tls_pthread.h"

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
// peerName = FQDN name as used in the peer's certificate.
// verifyPeer = Set to true to force verification of client's identity

void createServer(const char *keyFilename, const char *certFilename, int portNum, void *(*workerThread)(void *), const char *caCertFilename, const char *peerName, int verifyPeer);

// Returns TRUE if the TLS listener loop is still running.
int server_is_running();

#endif

#ifndef __TLS_COMMON_LIB__
#define __TLS_COMMON_LIB__

#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/crypto.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>
#include <openssl/pem.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>


/** These routines are not generally called from within reader/writer
    threads.  In the event a function that requires an SSL structure
    to be passed in, the reader/writer thread should cast its conn
    argument to SSL * before calling the function **/

// Initialize OpenSSL by loading all error strings and algorithms.
void init_openssl();

// Clean up OpenSSL
void cleanup_openssl();

// Create an OpenSSL Context
// CACertName is the filename of the CA's certificate. Needed only if
// verifyPeer is set to true.
// Set verifyPeer to true to verify the other side
// isServer = 0 to create context for a client, and 1 to create context for a server
SSL_CTX *create_context(const char *CACertName, int verifyPeer, int isServer);

//
// This function prints out the server's certificate details of the SSL session.
// ssl = SSL session

void printCertificate(SSL *ssl);

// This function verifies the certificate. Returns TRUE if 
// the certificate is valid
int verifyCertificate(SSL *ssl);

// Enables host verification
long setHostVerification(SSL *ssl, const char *hostname);

// Configure the OpenSSL Context.
// ctx - Context createde in create_context
// cert_name - Filename of your certificate (.crt or .pem) file
// pkey_name - Filename of your private key (.key) file
void configure_context(SSL_CTX *ctx, const char *cert_name, const char *pkey_name);

/** Routines below are meant to be called from within reader and writer
    threads and for that reason take in arguments in the form of void *
    instead of SSL * **/

// Write to the SSL connection. Called from within
// the worker thread.  Returns the number of
// bytes actually written.  If <0 an error has occurred.
// Call perror to print the error
// If 0 the connection has been closed.

int sslWrite(void *conn, const char *buffer, int len);

// Read from SSL connection. Called from within
// the worker thread.  Returns the number of
// bytes actually read.  If <0 an error has occurred.
// Call perror to print the error
// If 0 the connection has been closed.
// Read data is written into buffer.

int sslRead(void *conn, char *buffer, int len);

// Call this when exiting from a worker, reader or writer thread. This cleans
// up the thread environment.
#define EXIT_THREAD(conn) SSL_free((SSL *) conn); pthread_exit(NULL)
#endif

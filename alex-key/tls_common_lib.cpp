#include "tls_common_lib.h"

// Verify callback.
static int _verify_callback(int preverify, X509_STORE_CTX *x509_ctx){
    return preverify;

}

// Initialize OpenSSL by loading all error strings and algorithms.
void init_openssl()
{
	SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
}

// Clean up OpenSSL
void cleanup_openssl()
{
	EVP_cleanup();
}

// Create an OpenSSL Context

// Create an OpenSSL Context
// CACertName is the filename of the CA's certificate. Needed only if
// verifyPeer is set to true.
// Set verifyPeer to true to verify the other side

// isServer = 0 to create context for a client, and 1 to create context for a server
SSL_CTX *create_context(const char *CACertName, int verifyPeer, int isServer)
{
	// Create an SSL context
	const SSL_METHOD *method;
	SSL_CTX *ctx;

	// To maintain compatibility we will continue to
	// use SSLv23, but will restrict the use
	// of SSLv2 and SSLv3 which are known to have
	// security flaws.

    if(!isServer){ 
	    method = SSLv23_client_method();
    }
    else {
        method = SSLv23_server_method();
    }

	ctx = SSL_CTX_new(method);

	if(!ctx)
	{
		perror("Unable to create SSL context: ");
		ERR_print_errors_fp(stderr);
		exit(-1);
	}

	// Here is where we restrict the use of SSLv2 and SSLv3
	SSL_CTX_set_options(ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3);

    if(verifyPeer) {

        // Turn on verification of certificate
        SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, _verify_callback);

        // Set verification depth
        SSL_CTX_set_verify_depth(ctx, 4);

        // We load in the CA certificate to verify the server's certificate
        SSL_CTX_load_verify_locations(ctx, CACertName, NULL);

    }
	return ctx;
}

// Configure the OpenSSL Context.
// ctx - Context createde in create_context
// cert_name - Filename of your certificate (.crt or .pem) file
// pkey_name - Filename of your private key (.key) file
void configure_context(SSL_CTX *ctx, const char *cert_name, const char *pkey_name)
{
	// Use elliptic curve diffie-helman to exchange keys
	SSL_CTX_set_ecdh_auto(ctx, 1);

	// Load our server certificate
	if(SSL_CTX_use_certificate_file(ctx, cert_name, SSL_FILETYPE_PEM) <= 0)
	{
		ERR_print_errors_fp(stderr);
		exit(-1);
	}

	// Load our server private key
	if(SSL_CTX_use_PrivateKey_file(ctx, pkey_name, SSL_FILETYPE_PEM) <= 0)
	{
		ERR_print_errors_fp(stderr);
		exit(-1);
	}
}

//
// This function prints out the server's certificate details of the SSL session.
// ssl = SSL session

void printCertificate(SSL *ssl)
{
	// NEW: Getting certificates
	X509 *cert=NULL;
	X509_NAME *certname = NULL;

	cert = SSL_get_peer_certificate(ssl);

	if(cert == NULL)
	{
		printf( "Cannot get peer certificate\n");
		return;
	}

	certname = X509_NAME_new();
	certname = X509_get_subject_name(cert);
	X509_NAME_print_ex_fp(stdout, certname, 0, 0);
	printf( "\n\n");
}

// This function verifies the certificate. Returns TRUE if 
// the certificate is valid
int verifyCertificate(SSL *ssl)
{
	long res = SSL_get_verify_result(ssl);
	return (res == X509_V_OK);
}
// Enables host verification
long setHostVerification(SSL *ssl, const char *hostname)
{
	X509_VERIFY_PARAM *param = SSL_get0_param(ssl);
	X509_VERIFY_PARAM_set_hostflags(param, X509_CHECK_FLAG_NO_PARTIAL_WILDCARDS);

	long res = X509_VERIFY_PARAM_set1_host(param, hostname, strlen(hostname));
	return res;
}

// Write to the SSL connection. Called from within
// the worker thread.  Returns the number of
// bytes actually written.  If <0 an error has occurred.
// Call perror to print the error
// If 0 the connection has been closed.

int sslWrite(void *conn, const char *buffer, int len) {
    SSL *ssl = (SSL *) conn;
    
    return SSL_write(ssl, buffer, len);
}

// Read from SSL connection. Called from within
// the worker thread.  Returns the number of
// bytes actually read.  If <0 an error has occurred.
// Call perror to print the error
// If 0 the connection has been closed.
// Read data is written into buffer.

int sslRead(void *conn, char *buffer, int len) {
    SSL *ssl = (SSL *) conn;
    return SSL_read(ssl, buffer, len);
}


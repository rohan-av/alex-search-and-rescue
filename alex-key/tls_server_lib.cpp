#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "tls_common_lib.h"
#include "tls_server_lib.h"



// Creates a new SSL session, attaches the client file
// descriptor to it, initiates the connection and
// returns the SSL session.
// ctx = SSL context
// fd = File descriptor for client connection.
//

SSL *connectSSL(SSL_CTX *ctx, int fd, const char *common_name)
{
	SSL *ssl = SSL_new(ctx);
	SSL_set_fd(ssl, fd);
    
    if(common_name != NULL)
    {
        setHostVerification(ssl, common_name);
        
    }

	if(SSL_accept(ssl) <= 0)
	{
		ERR_print_errors_fp(stderr);
		SSL_free(ssl);
		return NULL;
	}

    X509 *cert;
    cert = SSL_get_peer_certificate(ssl);

    if(cert == NULL) {
        printf("Unable to get certificate\n");
        SSL_free(ssl);
        return NULL;
    }

	return ssl;
}

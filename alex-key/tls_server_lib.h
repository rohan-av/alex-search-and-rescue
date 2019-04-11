#ifndef __TLS_SERVER_LIB__
#define __TLS_SERVER_LIB__

#include <openssl/ssl.h>
#include <openssl/err.h>

// Creates a new SSL session, attaches the client file
// descriptor to it, initiates the connection and
// returns the SSL session.
// ctx = SSL context
// fd = File descriptor for client connection.
// common_name= Common name of peer, for verification
//

SSL *connectSSL(SSL_CTX *ctx, int fd, const char *common_name);
#endif

#include "make_tls_server.h"
#include "tls_common_lib.h"

#include <stdio.h>

#define PORTNUM 5000
#define KEY_FNAME   "alex.key"
#define CERT_FNAME  "alex.crt"
#define CA_CERT_FNAME   "signing.pem"
#define CLIENT_NAME     "322_laptop.lol.com"


// We are making an echo server. So we
// just echo back whatever we read.

void *worker(void *conn) {
    int exit = 0;

    while(!exit) {

        int count;
        char buffer[128];

        count = sslRead(conn, buffer, sizeof(buffer));

        if(count > 0) {
            printf("Read %s. Echoing.\n", buffer);
            count = sslWrite(conn, buffer, sizeof(buffer));

            if(count < 0) {
                perror("Error writing to network: " );
            }
        }
        else if(count < 0) {
            perror("Error reading from network: ");
        }
    
        // Exit of we have an error or the connection has closed.
        exit = (count <= 0);
    }

    printf("\nConnection closed. Exiting.\n\n");
    EXIT_THREAD(conn);
}

int main() {
    createServer(KEY_FNAME, CERT_FNAME, PORTNUM, &worker, CA_CERT_FNAME, CLIENT_NAME, 1);

    while(server_is_running());
}




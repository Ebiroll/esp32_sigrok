
#ifndef NETCOMM_H
#define NETCOMM_H

#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include <sys/wait.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/fcntl.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>

#include <sys/time.h>

#include "simple_except.h"



//******************************************************************************
class Host;
class Client;
struct Socket {
    struct sockaddr_storage other_addr;
    int sockfd;
    std::string other;
    
    Socket(): sockfd(-1) {}
    ~Socket() {if(sockfd != -1) Close();}
    friend class Host;
    friend class Client;
    
    int Close() {return close(sockfd);}
    int Shutdown(int how) {return shutdown(sockfd, how);}
    int ShutdownRx(int how) {return shutdown(sockfd, 0);}
    int ShutdownTx(int how) {return shutdown(sockfd, 1);}
    int ShutdownRxTx(int how) {return shutdown(sockfd, 2);}
    
    // flags: MSG_OOB, MSG_PEEK, MSG_WAITALL
    int Recv(std::vector<uint8_t> & buf, size_t maxdata, int flags = 0) {
        buf.resize(maxdata);
        ssize_t status = recv(sockfd, &buf[0], maxdata, 0);
        if(status >= 0)
            buf.resize(status);
        return status;
    }
    // flags: MSG_OOB, MSG_DONTROUTE
    int Send(const std::vector<uint8_t> & buf, int flags = 0) {
        return send(sockfd, &buf[0], buf.size(), 0);
    }
};
//******************************************************************************

// This class wraps SLIP-style message framing around a socket. Received messages
// are queued for handling.
class FramedConnection {
    int sockfd;
    std::queue<std::vector<uint8_t> *> buffers;
    std::vector<uint8_t> * currentBuffer;
    bool escaped;
    bool good;
    
  public:
    FramedConnection(int s);
    
    void SetNonblocking();
    void SetBlocking();
    
    // Returns true if connection is usable, otherwise false.
    bool Good() {return good;}
    
    // Handles any pending incoming data.
    // Throws FormattedError if connection is lost.
    bool Poll();
    
    // Pop a message from the queue if any complete message has been received,
    // else return NULL. Does not poll socket.
    // Message must be deleted by caller.
    std::vector<uint8_t> * PopMessage();
    
    // Pop a message from the queue if any complete message has been received,
    // else block and poll until one is received.
    // Message must be deleted by caller.
    std::vector<uint8_t> * WaitPopMessage();
    
    // Send a message.
    // Throws FormattedError if connection is lost.
    void SendMessage(uint8_t bfr[], size_t len);
};

//******************************************************************************

class Host {
    int listenfd;
    
  public:
    Host(const std::string & port, int backlog = 10);
    ~Host();
    
    void Close();
    
    Socket * Accept();
};

//******************************************************************************

Socket * ClientConnect(const std::string & host, const std::string & port);


//******************************************************************************

// This class manages a server discovery mechanism listening for queries broadcast to
// a given address on a query port, and responding to the sender on a separate response port.
class DiscoverableServer {
    int reqSock;
    std::string addr;
    int qport;
    int rport;
    
  public:
    DiscoverableServer(const std::string & a, uint16_t qp, uint16_t rp);
    
    bool Poll(double secs = 0.001);
    
    void SendResponse(uint8_t * buffer, size_t msgLen, sockaddr_in & srcAddr, socklen_t srcAddrLen);
    
    // Override this to send a response with any needed server info
    virtual bool HandleRequest(uint8_t * buffer, size_t msgLen, sockaddr_in & srcAddr, socklen_t srcAddrLen) = 0;
};

//******************************************************************************
class ServerFinder {
    int respSock;
    std::string addr;
    int qport;
    int rport;
    
  public:
    ServerFinder(const std::string & a, uint16_t qp, uint16_t rp);
    ~ServerFinder();
    
    void SendQuery(uint8_t * buffer, size_t msgLen);
    
    bool Poll(double secs = 0.001);
    
    bool PollFor(double secs);
    
    // Override this to handle responding servers
    virtual bool HandleResponse(uint8_t * buffer, size_t msgLen, sockaddr_in & srcAddr, socklen_t srcAddrLen) = 0;
};

//******************************************************************************
#endif // NETCOMM_H


#include "netcomm.h"

//******************************************************************************
// Return real time in seconds
static inline double real_seconds(void)
{
    struct timeval newTime;
    gettimeofday(&newTime, NULL);
    return newTime.tv_sec + newTime.tv_usec / 1e6;
}

//******************************************************************************
// FramedConnection
//******************************************************************************

FramedConnection::FramedConnection(int s):
    sockfd(s),
    currentBuffer(NULL),
    good(true)
{
    SetNonblocking();
}

void FramedConnection::SetNonblocking() {
    int flags = fcntl(sockfd, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(sockfd, F_SETFL, flags);
}

void FramedConnection::SetBlocking() {
    int flags = fcntl(sockfd, F_GETFL, 0);
    flags &= O_NONBLOCK;
    fcntl(sockfd, F_SETFL, flags);
}

bool FramedConnection::Poll()
{
    if(!currentBuffer) {
        escaped = false;
        currentBuffer = new std::vector<uint8_t>;
    }
    
    uint8_t tmpbuf[1024];
    ssize_t status;
    do {
        // Read block of data into tmpbuf, copy to accumulation buffer,
        // breaking into frames.
        // Framing is done SLIP-style:
        // 0xFF is an escape character
        // 0xFF 0xFE escapes the 0xFF character
        // 0xFF 0xFD terminates the current frame
        status = recv(sockfd, tmpbuf, 1024, 0);
        
        // Ignore these errors
        if(status == -1 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        
        if(status == -1)
        {
            good = false;
            throw FormattedError("recv() failed (error %d): %s", errno, strerror(errno));
        }
        
        for(ssize_t j = 0; j < status; ++j)
        {
            if(escaped)
            {
                if(tmpbuf[j] == 0xFE) {// escaped 0xFF
                    currentBuffer->push_back(0xFF);
                }
                else if(tmpbuf[j] == 0xFD) {// break frame
                    buffers.push(currentBuffer);
                    // printf("received: \n");
                    // for(ssize_t j = 0; j < currentBuffer->size(); ++j)
                    //     printf(" %02X", (*currentBuffer)[j]);
                    // printf("\n");
                    currentBuffer = new std::vector<uint8_t>;
                }
                escaped = false;
            }
            else
            {
                if(tmpbuf[j] == 0xFF)
                    escaped = true;
                else
                    currentBuffer->push_back(tmpbuf[j]);
            }
        }
    } while(status > 0);
    
    return !buffers.empty();
}

std::vector<uint8_t> * FramedConnection::PopMessage() {
    if(buffers.empty())
        return NULL;
    std::vector<uint8_t> * bfr = buffers.front();
    buffers.pop();
    return bfr;
}

std::vector<uint8_t> * FramedConnection::WaitPopMessage() {
    while(buffers.empty())
        Poll();
    return PopMessage();
}

void FramedConnection::SendMessage(uint8_t bfr[], size_t len)
{
    std::vector<uint8_t> outbfr(len*2 + 2);
    // printf("sending: \n");
    // for(ssize_t j = 0; j < len; ++j)
    //     printf(" %02X", bfr[j]);
    // printf("\n");
    int k = 0;
    for(int j = 0; j < len; ++j)
    {
        if(bfr[j] == 0xFF)
        {
            outbfr[k++] = 0xFF;
            outbfr[k++] = 0xFE;
        }
        else
        {
            outbfr[k++] = bfr[j];
        }
    }
    //outbfr[k++] = 0xFF;
    //outbfr[k++] = 0xFD;


    // End with newline
    outbfr[k++] = 0x0a;
    
    ssize_t status = send(sockfd, &outbfr[0], k, 0);
    if(status == -1)
    {
        good = false;
        throw FormattedError("send() failed (error %d): %s", errno, strerror(errno));
    }
}

//******************************************************************************
// Host
//******************************************************************************
Host::Host(const std::string & port, int backlog):
    listenfd(-1)
{
    int status;
    struct addrinfo hints, * servinfos;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    status = getaddrinfo(NULL, port.c_str(), &hints, &servinfos);
    if(status != 0) throw FormattedError("getaddrinfo() failed with status: %d", status);
    
    listenfd = -1;
    struct addrinfo * servinfo = servinfos;
    while(servinfo != NULL && listenfd == -1)
    {
        listenfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol);
        if(listenfd == -1) {
            perror("listen socket error");
            servinfo = servinfo->ai_next;
            continue;
        }
        
        int reuse = 1;
        status = setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int));
        if(status == -1)
            throw FormattedError("setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
        
        fcntl(listenfd, F_SETFL, O_NONBLOCK);
        
        status = bind(listenfd, servinfo->ai_addr, servinfo->ai_addrlen);
        if(status == -1) {
            close(listenfd);
            listenfd = -1;
            perror("bind() failed");
            servinfo = servinfo->ai_next;
            continue;
        }
    }
    
    freeaddrinfo(servinfos);
    
    if(listenfd == -1)
        throw FormattedError("failed to bind");
    
    if(listen(listenfd, backlog) == -1) {
        perror("listen");
        throw FormattedError("listen() failed");
    }
}
Host::~Host() {if(listenfd != -1) close(listenfd);}

void Host::Close() {if(listenfd != -1) close(listenfd); listenfd = -1;}

Socket * Host::Accept()
{
    struct sockaddr_storage other_addr;
    int sockfd;
    socklen_t sin_size = sizeof(struct sockaddr_storage);
    sockfd = accept(listenfd, (struct sockaddr *)&other_addr, &sin_size);
    if(sockfd == -1)
    {
        if(errno == EWOULDBLOCK)
            return NULL;
        else
            throw FormattedError("accept() failed: %s", strerror(errno));
    }
    
    char s[INET6_ADDRSTRLEN];
    struct sockaddr * sa = (struct sockaddr *)&other_addr;
    if(sa->sa_family == AF_INET)
        inet_ntop(other_addr.ss_family, &(((struct sockaddr_in *)sa)->sin_addr), s, INET6_ADDRSTRLEN);
    else
        inet_ntop(other_addr.ss_family, &(((struct sockaddr_in6 *)sa)->sin6_addr), s, INET6_ADDRSTRLEN);
    
    Socket * sock = new Socket;
    sock->other_addr = other_addr;
    sock->sockfd = sockfd;
    sock->other = s;
    return sock;
}

//******************************************************************************
// Client
//******************************************************************************

Socket * ClientConnect(const std::string & host, const std::string & port)
{
    int status;
    struct addrinfo hints, * servinfos;
    memset(&hints, 0, sizeof(addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    status = getaddrinfo(host.c_str(), port.c_str(), &hints, &servinfos);
    if(status != 0) throw FormattedError("getaddrinfo() failed with status: %d", status);
    
    int sockfd = -1;
    struct addrinfo * servinfo = servinfos;
    while(servinfo != NULL && sockfd == -1)
    {
        sockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol);
        if(sockfd == -1) {
            perror("socket error");
            servinfo = servinfo->ai_next;
            continue;
        }
        
        status = connect(sockfd, servinfo->ai_addr, servinfo->ai_addrlen);
        if(status == -1) {
            close(sockfd);
            sockfd = -1;
            perror("connect error");
            servinfo = servinfo->ai_next;
            continue;
        }
    }
    
    if(sockfd == -1)
        throw FormattedError("failed to connect");
    
    char s[INET6_ADDRSTRLEN];
    struct sockaddr * sa = (struct sockaddr *)&(servinfo->ai_addr);
    if(sa->sa_family == AF_INET)
        inet_ntop(servinfo->ai_family, &(((struct sockaddr_in *)sa)->sin_addr), s, INET6_ADDRSTRLEN);
    else
        inet_ntop(servinfo->ai_family, &(((struct sockaddr_in6 *)sa)->sin6_addr), s, INET6_ADDRSTRLEN);
    
    Socket * sock = new Socket;
    // sock->other_addr = *sa;
    sock->sockfd = sockfd;
    sock->other = s;
    
    freeaddrinfo(servinfos);
    return sock;
}

//******************************************************************************
// DiscoverableServer
//******************************************************************************

DiscoverableServer::DiscoverableServer(const std::string & a, uint16_t qp, uint16_t rp):
        addr(a),
        qport(qp),
        rport(rp)
{
    // Setup request socket
    reqSock = socket(AF_INET, SOCK_DGRAM, 0);
    if(reqSock < 0)
    {
        perror("Error while creating client socket: ");
        abort();
    }
    
    struct sockaddr_in serverAddr;
    bzero((uint8_t*)&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(qport);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    if(bind(reqSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        perror("Error while binding client socket: ");
        abort();
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(addr.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if(setsockopt(reqSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        perror("Error while reusing client socket: ");
        abort();
    }
}

bool DiscoverableServer::Poll(double secs)
{
    struct timeval timeout;
    timeout.tv_sec = floor(secs);
    timeout.tv_usec = (secs - floor(secs))*1e6;
    
    fd_set reqSocks;
    FD_ZERO(&reqSocks);
    FD_SET(reqSock, &reqSocks);
    int sel = select(FD_SETSIZE, &reqSocks, NULL, NULL, &timeout);
    if(sel < 0)
    {
        perror("select() returned error: ");
        abort();
    }
    else if(sel == 0)
    {
        return false;
    }
    // std::cerr << "select() returned: " << sel << std::endl;
    
    struct sockaddr_in srcAddr;
    socklen_t srcAddrLen;
    uint8_t buffer[1024];// TODO: make max message length configurable
    size_t msgLen = recvfrom(reqSock, buffer, 1024, 0, (struct sockaddr *)&srcAddr, &srcAddrLen);
    
    if(msgLen > 0)
        return HandleRequest(buffer, msgLen, srcAddr, srcAddrLen);
    else
        return false;
}

void DiscoverableServer::SendResponse(uint8_t * buffer, size_t msgLen, sockaddr_in & srcAddr, socklen_t srcAddrLen)
{
    // struct sockaddr_storage ss;
    sockaddr_in respAddr;
    memcpy(&respAddr, &srcAddr, srcAddrLen);
    respAddr.sin_port = htons(rport);
    
    // char s[INET6_ADDRSTRLEN];
    // struct sockaddr * sa = (struct sockaddr *)&respAddr;
    // if(sa->sa_family == AF_INET)
    //     inet_ntop(sa->sa_family, &(((struct sockaddr_in *)sa)->sin_addr), s, INET6_ADDRSTRLEN);
    // else
    //     inet_ntop(sa->sa_family, &(((struct sockaddr_in6 *)sa)->sin6_addr), s, INET6_ADDRSTRLEN);
    // std::cerr << "Sending response to: " << s << std::endl;
    sendto(reqSock, buffer, msgLen, 0, (struct sockaddr *)&respAddr, srcAddrLen);
}


//******************************************************************************
// ServerFinder
//******************************************************************************
ServerFinder::ServerFinder(const std::string & a, uint16_t qp, uint16_t rp):
    addr(a),
    qport(qp),
    rport(rp)
{
}
ServerFinder::~ServerFinder() {
    close(respSock);
}

void ServerFinder::SendQuery(uint8_t * buffer, size_t msgLen)
{
    // Setup query response socket
    respSock = socket(AF_INET, SOCK_DGRAM, 0);
    if(respSock < 0)
    {
        perror("Error while creating client socket: ");
        abort();
    }
    
    struct sockaddr_in serverAddr;
    bzero((uint8_t*)&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(rport);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    if(bind(respSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        perror("Error while binding client socket: ");
        abort();
    }
    
    
    // Setup query broadcast socket
    int bcastSock = socket(AF_INET, SOCK_DGRAM, 0);
    if(bcastSock < 0)
    {
        perror("Error while creating broadcast socket: ");
        abort();
    }
    
    struct sockaddr_in bcastAddr;
    bzero((uint8_t*)&bcastAddr, sizeof(bcastAddr));
    bcastAddr.sin_family = AF_INET;
    bcastAddr.sin_port = htons(qport);
    bcastAddr.sin_addr.s_addr = inet_addr(addr.c_str());
    
    // Send query
    sendto(bcastSock, buffer, msgLen, 0,
           (struct sockaddr *)&bcastAddr, sizeof(bcastAddr));
    
    close(bcastSock);
}

bool ServerFinder::Poll(double secs)
{
    struct timeval timeout;
    timeout.tv_sec = floor(secs);
    timeout.tv_usec = (secs - floor(secs))*1e6;
    
    fd_set respSocks;
    FD_ZERO(&respSocks);
    FD_SET(respSock, &respSocks);
    int sel = select(FD_SETSIZE, &respSocks, NULL, NULL, &timeout);
    if(sel < 0)
    {
        perror("select() returned error: ");
        abort();
    }
    else if(sel == 0)
    {
        return false;
    }
    std::cerr << "select() returned: " << sel << std::endl;
    
    struct sockaddr_in srcAddr;
    socklen_t srcAddrLen;
    uint8_t buffer[1024];// TODO: make max message length configurable
    size_t msgLen = recvfrom(respSock, buffer, 1024, 0,
                             (struct sockaddr *)&srcAddr, &srcAddrLen);
    
    std::cerr << "selectmsgLen: " << msgLen << std::endl;
    if(msgLen > 0)
        return HandleResponse(buffer, msgLen, srcAddr, srcAddrLen);
    else
        return false;
}

bool ServerFinder::PollFor(double secs)
{
    double start = real_seconds(), now = start;
    while((now - start) < secs) {
        Poll(secs - (now - start));
        now = real_seconds();
    }
}

//******************************************************************************


#ifndef REMOTEDEVICE_H
#define REMOTEDEVICE_H

#include "netcomm.h"
#include "freetmc.h"
#include "protocol.h"

#include <cstdlib>
#include <vector>
#include <string>
#include <exception>

#include <cstdarg>

//******************************************************************************

class TMC_RemoteDevice: public TMC_Device {
    FramedConnection conn;
    uint8_t seqNum;
  public:
    TMC_RemoteDevice(uint16_t vendID, uint16_t prodID, const std::string & sernum, int sockfd);
    virtual ~TMC_RemoteDevice();
    
    virtual size_t Write(const uint8_t * msg, size_t len);
    
    virtual void StartRead(uint8_t * msg, size_t len);
    virtual ssize_t FinishRead(uint8_t * msg, size_t len);
};


//******************************************************************************
#endif // REMOTEDEVICE_H

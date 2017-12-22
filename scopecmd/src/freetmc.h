
#ifndef FREETMC_H
#define FREETMC_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <exception>

#include <cstdarg>
//#include <cstdint>
#include <stdint.h>

#include "simple_except.h"

//******************************************************************************

struct IDN_Response {
    std::string manufacturer;
    std::string model;
    std::string serial;
    std::string version;
};



class TMC_Device {
  public:
    TMC_Device() {}
    virtual ~TMC_Device() {}
    
    virtual size_t Write(const uint8_t * msg, size_t len) = 0;
    virtual size_t Write(const std::vector<uint8_t> & msg) {return Write(&msg[0], msg.size());}
    virtual size_t Write(const std::string & msg) {return Write((const uint8_t*)&msg[0], msg.size());}
    
    virtual void StartRead(uint8_t * msg, size_t nbytes) = 0;
    virtual ssize_t FinishRead(uint8_t * msg, size_t nbytes) = 0;
    virtual size_t Read(uint8_t * msg, size_t nbytes) {
        StartRead(msg, nbytes);
        return FinishRead(msg, nbytes);
    }
    virtual size_t Read(std::vector<uint8_t> & msg, size_t nbytes) {
        msg.resize(nbytes);
        size_t bytesRead = Read(&msg[0], nbytes);
        msg.resize(bytesRead);
        return bytesRead;
    }
    
    // Direct queries
    void Cmd(const std::string & opt) {Write(opt);}
    void CmdVA(const std::string & format, va_list ap) {
        char * fstr;
        vasprintf(&fstr, format.c_str(), ap);
        if(fstr != NULL) {
            Write(fstr);
            free(fstr);
        }
    }
    void CmdF(const std::string & format, ...) {
        va_list ap;
        va_start(ap, format);
        CmdVA(format, ap);
        va_end(ap);
    }
    
    // Basic query, returns response as string
    std::string Query(const std::string & str, size_t maxSize = 1024) {
        Write(str);
        std::vector<uint8_t> resp;
        Read(resp, maxSize);
        return std::string(resp.begin(), resp.end());
    }
    std::string QueryF(size_t maxSize, const std::string & format, ...) {
        va_list ap;
        va_start(ap, format);
        CmdVA(format, ap);
        va_end(ap);
        std::vector<uint8_t> resp;
        Read(resp, maxSize);
        return std::string(resp.begin(), resp.end());
    }
    std::string QueryF(const std::string & format, ...) {
        va_list ap;
        va_start(ap, format);
        CmdVA(format, ap);
        va_end(ap);
        std::vector<uint8_t> resp;
        Read(resp, 1024);
        return std::string(resp.begin(), resp.end());
    }
    
    // Query returning binary data
    void BinQuery(std::vector<uint8_t> & resp, const std::string & str, size_t maxSize = 1024) {
        Write(str);
        Read(resp, maxSize);
    }
    
    // Query returning floating point parameter
    double FloatQuery(const std::string & format, ...) {
        va_list ap;
        va_start(ap, format);
        CmdVA(format, ap);
        va_end(ap);
        std::vector<uint8_t> resp;
        Read(resp, 1024);
        return strtod((char *)&resp[0], NULL);
    }
    
    // Query returning integer parameter
    int64_t IntQuery(const std::string & format, ...) {
        va_list ap;
        va_start(ap, format);
        CmdVA(format, ap);
        va_end(ap);
        std::vector<uint8_t> resp;
        Read(resp, 1024);
        return strtol((char *)&resp[0], NULL, 0);
    }
    
    // Standard queries
    IDN_Response Identify() {
        std::string resp = Query("*IDN?");
        // std::cout << "IDN: " << resp << std::endl;
        IDN_Response idn;
        size_t start = 0, end = resp.find_first_of(',');
        idn.manufacturer = resp.substr(start, end - start);
        start = end + 1;
        end = resp.find_first_of(',', start);
        idn.model = resp.substr(start, end - start);
        start = end + 1;
        end = resp.find_first_of(',', start);
        idn.serial = resp.substr(start, end - start);
        start = end + 1;
        end = resp.find_first_of(',', start);
        idn.version = resp.substr(start, end - start);
        return idn;
    }
};


//******************************************************************************
#endif // FREETMC_H

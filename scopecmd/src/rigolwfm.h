
#ifndef RIGOLWFM_H
#define RIGOLWFM_H

// Based on code from http://www.mathworks.com/matlabcentral/fileexchange/18999-read-binary-rigol-waveforms
// and file format information derived from http://meteleskublesku.cz/wfm_view/file_wfm.zip

#include <iostream>
#include <string>
#include <exception>
#include <vector>
#include <cstdlib>
#include <algorithm>

#include "units.h"
#include "sigproc.h"

namespace rigwfm {

class Rigol_Exception: public std::exception {
  protected:
    std::string msg;
  public:
    Rigol_Exception(const std::string & m): msg(m) {}
    virtual ~Rigol_Exception() throw() {}
    virtual const char* what() const throw() {return msg.c_str();}
};


enum trig_t {
    kEdgeTrigger = 0x0000,
    kPulseTrigger = 0x0101,
    kSlopeTrigger = 0x0202,
    kVideoTrigger = 0x0303,
    kAltTrigger = 0x0004,// Not 0404?
    kPatternTrigger = 0x0505,
    kDurationTrigger = 0x0606
};
enum trigsrc_t {
// if triggersource == 0
//     trigger.source = 'ch1';
// elseif triggersource == 1
//     trigger.source = 'ch2';
// elseif triggersource == 2
//     trigger.source = 'ext';
// elseif triggersource == 3
//     trigger.source = 'ext/5';
// elseif triggersource == 5
//     trigger.source = 'acLine';
// elseif triggersource == 7
//     trigger.source = 'dig.ch';
// else
};

struct RigolData {
    double scale;// Volts/div
    double offset;// Volts
    double scale2;// Volts/div, apparently identical to scale
    double offset2;// Volts, apparently identical to offset
    double attenuation;// Dimensionless
    bool inverted;
    bool inverted2;
    std::vector<float> data;// Volts, with scaling, offset, and attenuation accounted for
    // WARNING: first 4 samples appear to be garbage
    
    RigolData():
        scale(1),
        offset(0),
        attenuation(1),
        inverted(false)
    {}
    
    friend std::ostream & operator<<(std::ostream & ostrm, const RigolData & data);
};

// 
struct RigolWaveform {
    uint32_t npoints; // Number of data points
    double tscale;// time per div, seconds
    double tdelay;// time delay, seconds
    double fs;// sample frequency, Hz
    double duration;// Total waveform time, seconds
    
    double scaleM;
    double delayM;
    
    trig_t trigger;
    trigsrc_t triggerSource;
    
    RigolData channels[2];
    
    RigolWaveform(const std::string & fpath);
    void ReadChannelInfo(int ch, size_t dataLoc, FILE * fin);
    
    friend std::ostream & operator<<(std::ostream & ostrm, const RigolWaveform & waveform);
};

template<typename T>
void freadpp(T & val, size_t offset, FILE * fin) {
    fseek(fin, offset, SEEK_SET);
    fread(&val, sizeof(T), 1, fin);
}
template<typename T>
void freadpp(T & val, size_t offset, size_t num, FILE * fin) {
    fseek(fin, offset, SEEK_SET);
    fread(&val, sizeof(T), num, fin);
}

inline RigolWaveform::RigolWaveform(const std::string & fpath):
    tscale(1),
    tdelay(0),
    duration(0)
{
    FILE * fin = fopen(fpath.c_str(), "r");
    if(!fin)
        throw Rigol_Exception("File doesn't exist");
    
    uint8_t checkbytes[2];
    freadpp(checkbytes, 0, 2, fin);
    if(checkbytes[0] != 165 ||checkbytes[1] != 165)
    {
        fclose(fin);
        throw Rigol_Exception("Not Rigol waveform file");
    }
    
    uint8_t ch1enab, ch2enab;
    uint64_t tscaleI;
    int64_t tdelayI;
    int64_t scaleMI;
    int64_t delayMI;
    float fsF;
    uint16_t triggerI;
    uint16_t triggerSrcI;
    
    freadpp(npoints, 28, fin);
    freadpp(ch1enab, 49, fin);
    freadpp(ch2enab, 73, fin);
    
    freadpp(tscaleI, 84, fin);
    freadpp(tdelayI, 92, fin);
    freadpp(fsF, 100, fin);
    freadpp(scaleM, 104, fin);
    freadpp(delayM, 112, fin);
    
    freadpp(triggerI, 142, fin);
    freadpp(triggerSrcI, 144, fin);
    
    tscale = tscaleI*1e-12;
    tdelay = tdelayI*1e-12;
    duration = tscale*12;// time per div*12 divs
    fs = fsF;
    scaleM = scaleMI*1e-12;
    delayM = delayMI*1e-12;
    
    trigger = (trig_t)triggerI;
    triggerSource = (trigsrc_t)triggerSrcI;
    
    size_t dataLoc = 272;
    if(ch1enab == 1) {
        ReadChannelInfo(0, dataLoc, fin);
        dataLoc += npoints;
    }
    if(ch2enab == 1) {
        ReadChannelInfo(1, dataLoc, fin);
    }
    
    fclose(fin);
}

inline void RigolWaveform::ReadChannelInfo(int ch, size_t dataLoc, FILE * fin)
{
    RigolData & channel = channels[ch];
    size_t base = (ch == 0)? 36 : 60;
    
    int32_t scale;
    int16_t offset;
    uint16_t attenuation;
    uint8_t inverted;
    uint8_t inverted2;
    int32_t scale2;
    int16_t offset2;
    freadpp(scale, base, fin);
    freadpp(offset, base + 4, fin);
    freadpp(attenuation, base + 10, fin);
    freadpp(inverted, base + 12, fin);
    freadpp(inverted2, base + 14, fin);
    freadpp(scale2, base + 16, fin);
    freadpp(offset2, base + 20, fin);
    
    if(attenuation == 0x3F80)
        channel.attenuation = 1;
    else if(attenuation == 0x4120)
        channel.attenuation = 10;
    else if(attenuation == 0x42C8)
        channel.attenuation = 100;
    else if(attenuation == 0x447A)
        channel.attenuation = 1000;
    else {
        fclose(fin);
        throw Rigol_Exception("Bad attenuation value");
    }
    
    channel.scale = scale*1e-6*channel.attenuation;
    channel.offset = offset/250.0*channel.attenuation;
    channel.scale2 = scale2*1e-6*channel.attenuation;
    channel.offset2 = offset2/250.0*channel.attenuation;
    
    channel.inverted = inverted;
    channel.inverted2 = inverted2;
    
    std::vector<uint8_t> data(npoints, 255);
    freadpp(data[0], dataLoc, npoints, fin);
    
    channel.data.resize(npoints);
    for(int j = 0; j < npoints; ++j)
        channel.data[j] = ((125.0 - (float)data[j])/250.0f*channel.scale)*10.0 - channel.offset;
}

inline std::ostream & operator<<(std::ostream & ostrm, const RigolWaveform & waveform)
{
    ostrm << "tscale: " << FmtSec(waveform.tscale)
          << ", tdelay: " << FmtSec(waveform.tdelay)
          << ", fs: " << FmtHz(waveform.fs) << std::endl;
    ostrm << "duration: " << FmtSec(waveform.duration) << std::endl;
    ostrm << "npoints: " << waveform.npoints << std::endl;
    ostrm << "scaleM: " << waveform.scaleM << std::endl;
    ostrm << "delayM: " << waveform.delayM << std::endl;
    ostrm << "trigger: " << waveform.trigger << ", trigger source: " << waveform.triggerSource << std::endl;
    
    if(waveform.channels[0].data.empty() && waveform.channels[1].data.empty())
        ostrm << "no channels enabled" << std::endl;
    if(!waveform.channels[0].data.empty() && waveform.channels[1].data.empty())
        ostrm << "channel 1" << std::endl;
    if(waveform.channels[0].data.empty() && !waveform.channels[1].data.empty())
        ostrm << "channel 2" << std::endl;
    if(!waveform.channels[0].data.empty() && !waveform.channels[1].data.empty())
        ostrm << "channels 1 and 2" << std::endl;
    
    ostrm << std::endl << "channel 1:" << std::endl << waveform.channels[0];
    ostrm << std::endl << "channel 2:" << std::endl << waveform.channels[1];
    return ostrm;
}

inline std::ostream & operator<<(std::ostream & ostrm, const RigolData & data)
{
    ostrm << "scale: " << data.scale << std::endl;
    ostrm << "offset: " << data.offset << std::endl;
    ostrm << "scale2: " << data.scale2 << std::endl;
    ostrm << "offset2: " << data.offset2 << std::endl;
    ostrm << "attenuation: " << data.attenuation << std::endl;
    ostrm << "inverted: " << data.inverted << std::endl;
    ostrm << "inverted2: " << data.inverted2 << std::endl;
    return ostrm;
}


} // namespace rigwfm
#endif // RIGOLWFM_H

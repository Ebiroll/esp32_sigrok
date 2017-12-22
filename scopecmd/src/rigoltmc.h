
#ifndef RIGOLTMC_H
#define RIGOLTMC_H

#include "freetmc.h"

class DS1000E: public TMC_Device {
    TMC_Device * tmcDev;
  public:
    DS1000E(TMC_Device * tmc): tmcDev(tmc) {}
    
    virtual size_t Write(const uint8_t * msg, size_t len) {tmcDev->Write(msg, len);}
    virtual void StartRead(uint8_t * msg, size_t nbytes) {return tmcDev->StartRead(msg, nbytes);}
    virtual ssize_t FinishRead(uint8_t * msg, size_t nbytes) {return tmcDev->FinishRead(msg, nbytes);}
    
    // Main API
    
    void Reset()   {Cmd("*RST");}
    
    // "RAW" mode requires STOP state (?)
    // "MAX" mode: same as NOR when in RUN state, same as RAW in STOP state.
    //
    //               (short mem)  (long mem)
    //          NOR     RAW         RAW
    //
    //    MATH  600     600         600
    //     FFT  512     512         512
    //     CHx  600     8192        524288
    //    HCHx  600     16384       1048576
    // DIGITAL  600     600         600
    std::string CurrentMode() {return Query(":WAV:POIN:MODE?");}
    void NormalMode() {Cmd(":WAV:POIN:MODE NOR");}
    void RawMode()    {Cmd(":WAV:POIN:MODE RAW");}
    void MaxMode()    {Cmd(":WAV:POIN:MODE MAX");}
    
    void WaveData(std::vector<uint8_t> & resp, int channel, size_t maxSize = 1024*1024) {
        CmdF(":WAV:DATA? CHAN%c", '1' + channel);
        TMC_Device::Read(resp, maxSize);
    }
    
    
    double SampleRateDigital() {return FloatQuery(":ACQ:SAMP? DIGITAL");}
    double SampleRate(int channel) {return FloatQuery(":ACQ:SAMP? CHAN%c", '1' + channel);}
    
    double ChanScale(int channel) {return FloatQuery(":CHAN%c:SCAL?", '1' + channel);}
    double ChanOffset(int channel) {return FloatQuery(":CHAN%c:OFFS?", '1' + channel);}
    
    double TimeScale() {return FloatQuery(":TIM:SCAL?");}
    double TimeOffset() {return FloatQuery(":TIM:OFFS?");}
    
    void Run()   {Cmd(":RUN");}
    void Stop()  {Cmd(":STOP");}
    void Auto()  {Cmd(":AUTO");}
    void HardCopy()  {Cmd(":HARD");}
    void Force() {Cmd(":FORC");}
    
    void Beep()   {Cmd(":BEEP:ACT");}
    
    void KeyLock(bool locked) {Cmd((locked) ? ":KEY:LOCK ENAB" : ":KEY:LOCK DIS");}
    void KeyRun() {Cmd(":KEY:RUN");}
};

#endif // RIGOLTMC_H

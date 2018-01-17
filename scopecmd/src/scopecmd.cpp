
#include "freetmc.h"
#include "rigoltmc.h"
//#include "freetmc_local.h"
#include "freetmc_remote.h"

#include <iostream>
#include <map>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>

using namespace std;

DS1000E * scope = NULL;

string serialNum = "";
string serverURL = "";
//string serverPortNum = "9393";
string serverPortNum = "5555";
bool verbose = false;
bool dryRun = false;

static const char * optString = "s:u:v";
static const struct option longOpts[] = {
    // {"plot", no_argument, NULL, 'p'},
    {"url",  required_argument, NULL, 'u'},
    {"port",  required_argument, NULL, 0},
    {"dry-run",  no_argument, NULL, 0},
    {NULL, 0, NULL, 0}
};

inline bool streq(const char * s1, const char * s2) {return strcmp(s1, s2) == 0;}

// Handlers must run if scope == NULL, to perform dry-run
struct Command;
typedef void (*CmdHandler)(int & argc, char *** argv, Command &);

struct Command {
    string shortCmd;
    string cmdStr;
    int respLen;
    CmdHandler handler;
    string description;
};

void WRITE(int & argc, char *** argv, Command & cmd);
void READ(int & argc, char *** argv, Command & cmd);
void QUERY(int & argc, char *** argv, Command & cmd);
void ECHO(int & argc, char *** argv, Command & cmd);
void NEWLINE(int & argc, char *** argv, Command & cmd);
void ACQ_AVER(int & argc, char *** argv, Command & cmd);
void WAV_DATA(int & argc, char *** argv, Command & cmd);
void GET_SCOPE_PARAMS(int & argc, char *** argv, Command & cmd);

Command commands[] = {
    // System commands
    {"idn", "*IDN?", 256, NULL, "return identity string, of the form \"RIGOL TECHNOLOGIES,DS1102E,DS1EB104702974,00.02.01.01.00\""},
    {"rst", "*RST", 0, NULL, "reset parameters to defaults"},
    {"run", ":RUN", 0, NULL, "Start acquiring data"},
    {"stop", ":STOP", 0, NULL, "Stop acquiring data"},
    {"auto", ":AUTO", 0, NULL, "Automatically set capture parameters"},
    {"hard", ":HARD", 0, NULL, "Save a screen capture to USB as sequentially named file with name \"HardCopy###.bmp\""},
    
    // TODO: Any point to implementing all these commands? Might as well use raw commands for most.
    // Acquisition commands
/*    {"acq-type-norm", ":ACQ:TYPE NORM", 0, NULL, "Set normal acquisition type"},
    {"acq-type-aver", ":ACQ:TYPE AVER", 0, NULL, "Set averaging acquisition type"},
    {"acq-type-peak", ":ACQ:TYPE PEAK", 0, NULL, "Set peak detection acquisition type"},
    {"get-acq-type", ":ACQ:TYPE?", 256, NULL, "Get acquisition type"},
    
    {"acq-mode-rtim", ":ACQ:MODE RTIM", 0, NULL, "Set real-time acquisition mode"},
    {"acq-mode-etim", ":ACQ:MODE ETIM", 0, NULL, "Set equal-time acquisition mode"},
    {"get-acq-mode", ":ACQ:MODE?", 256, NULL, "Get acquisition mode"},
    
    {"acq-aver", ":ACQ:AVER N", 0, ACQ_AVER, "Set number of points to average"},
    {"get-acq-aver", ":ACQ:AVER?", 256, NULL, "Get number of points being averaged"},
    
    {"get-acq-samp-1", ":ACQ:SAMP? CHAN1", 256, NULL, "Get sampling rate for channel 1"},
    {"get-acq-samp-2", ":ACQ:SAMP? CHAN2", 256, NULL, "Get sampling rate for channel 2"},
    {"get-acq-samp-digital", ":ACQ:SAMP? DIGITAL", 256, NULL, "Get sampling rate for logic analyzer (DS1000D series only)"},
    
    {"acq-memd-norm", ":ACQ:MEMD NORM", 0, NULL, "Set normal memory depth (8192-16384 points)"},
    {"acq-memd-long", ":ACQ:MEMD LONG", 0, NULL, "Set long memory depth (524288-1048576 points)"},
    {"get-acq-memd", ":ACQ:MEMD?", 256, NULL, "Get memory depth"},*/
    
    // Display commands
    
    // Timebase commands
    // ***
/*    {"tim-mode-main", ":TIM:MODE MAIN", 0, NULL, "Use main timebase"},
    {"tim-mode-del", ":TIM:MODE DEL", 0, NULL, "Use delayed timebase"},
    {"get-tim-mode", ":TIM:MODE?", 256, NULL, "Get timebase mode"},*/
    
    // {"tim-off", ":TIM:OFFS N", 0, TIM_OFF, "Set timebase offset"},
    // {"get-tim-off", ":TIM:OFFS?", 256, NULL, "Get timebase offset"},
    
    // {"tim-del-off", ":TIM:DEL:OFFS N", 0, TIM_DEL_OFF, "Set delayed timebase offset"},
    // {"get-tim-del-off", ":TIM:DEL:OFFS?", 256, NULL, "Get delayed timebase offset"},
    
    // Trigger commands
    // ***
    
    // Restore to factory settings
    // {"stor-fact-load", ":STOR:FACT:LOAD", 256, NULL, "Restore to factory settings"},
    
    // Math commands
    
    // Channel commands
    // ***
    
    // Measurement commands
    
    // Waveform commands
    // {"wav-points-norm", ":WAV:POIN:MODE NORM", 0, NULL, "Get normal waveform data (600 points)"},
    // {"wav-points-raw", ":WAV:POIN:MODE RAW", 0, NULL, "Get raw waveform data (8192-16384 points in stop state)"},
    // {"wav-points-max", ":WAV:POIN:MODE MAX", 0, NULL, "Get maximum waveform data (depends on run/stop state)"},
    {"wav-data", ":WAV:DATA?", 1024*1024, WAV_DATA, "Read waveform data (with specified output options)"},
    
    // Logic analyzer commands
    
    // Key commands
    
    // Other commands
    
    // scopecmd commands
    
    // Contrary to the documention, the enable/disable is for the lock, not the buttons
    {"lock", ":KEY:LOCK ENAB", 0, NULL, "Lock panel buttons"},
    {"unlock", ":KEY:LOCK DIS", 0, NULL, "Unlock panel buttons"},
    
    {"get-scope-params", "", 0, GET_SCOPE_PARAMS, "Get scope parameters in human readable, easily parsable format"},
    {"w", "<VARIES>", 0, WRITE, "Write raw command to scope"},
    {"r", "<NONE>", 0, READ, "Read raw data from scope"},
    {"q", "<NONE>", 0, QUERY, "Send command and read short response"},
    {"echo", "<NONE>", 0, ECHO, "Echo string to output (no effect on scope)"},
    {"nl", "<NONE>", 0, NEWLINE, "Write newline to output (no effect on scope)"}
};
map<string, Command> commandMap;


int DoCommands(int argc, char ** argv);

void ScopeWrite(const string & cmdStr)
{
    if(scope) {
        scope->Write((uint8_t*)cmdStr.c_str(), cmdStr.length());
    }
    else if(verbose)
    {
        cerr << "dry run write: " << cmdStr << endl;
    }
}
void ScopeRead(size_t size)
{
    if(scope)
    {
        vector<uint8_t> bfr(size);
        size_t respLen = scope->Read(&bfr[0], size);
        std::cout << string(bfr.begin(), bfr.begin() + respLen) << endl;
    }
    else if(verbose)
    {
        cerr << "dry run read: " << size << " B" << endl;
    }
}

void ExitCleanup()
{
  //libusb_exit(NULL);
}

int main(int argc, char ** argv)
{
    if(argc < 2)
    {
        cerr << "Usage: scopecmd [OPTIONS] [COMMANDS]" << endl;
        cerr << "OPTIONS:" << endl;
        // cerr << "\t-t SCOPE_TYPE: Oscilloscope type. ds1k" << endl;
        cerr << "\t-s SERNUM: device serial number. Will take first device found if not specified." << endl;
        cerr << "\t-u URL: URL of remote device. Will assume local USB if not specified." << endl;
        cerr << "\t--port PORTNUM: port number of remote device server. Default is 5555." << endl;
        // cerr << "\t--wav-mode=WMODE: Waveform output mode: ascii, csv, hex, fixpoint (binary), float (binary)" << endl;
        // cerr << "\t--wav-path=PATH: Waveform output path and base name." << endl;
        cerr << "\t-v: Verbose output" << endl;
        cerr << "\t--dry-run: Dry run only (no scope comms)" << endl;
        // cerr << "\t--list: List recognized local scopes" << endl;
        // cerr << "\t--list-remote: List recognized remote scopes" << endl;
        cerr << "" << endl;
        cerr << "\tWaveform length: normal (600 pts), raw (8K-16K pts), long (512K-1M pts)." << endl;
        cerr << "COMMANDS:" << endl;
        size_t longest = 0;
        for(int j = 0; j < (sizeof(commands)/sizeof(Command)); ++j)
            longest = max(longest, commands[j].shortCmd.length());
        
        for(int j = 0; j < (sizeof(commands)/sizeof(Command)); ++j)
            cerr << string(longest - commands[j].shortCmd.length(), ' ')
                << commands[j].shortCmd
                << ": " << commands[j].description << endl;
        
        return EXIT_FAILURE;
    }
    int r;
    //usb_init();
    //if(r < 0) {
    //    cerr << "libusb_init() failed" << endl;
    //    return EXIT_FAILURE;
    //}
    atexit(ExitCleanup);
    
    for(int j = 0; j < (sizeof(commands)/sizeof(Command)); ++j)
        commandMap[commands[j].shortCmd] = commands[j];
    
    int opt, optInd = 0;
    do {
        opt = getopt_long(argc, argv, optString, longOpts, &optInd);
        // optarg, optind, optopt
        switch(opt) {
            case 's': serialNum = optarg; break;
            case 'u': serverURL = optarg; break;
            case 'v': verbose = true; break;
            
            case 0:
                if(streq("port", longOpts[optInd].name))
                    serverPortNum = optarg;
                else if(streq("url", longOpts[optInd].name))
                    serverURL = optarg;
                else if(streq("dry-run", longOpts[optInd].name))
                    dryRun = true;
            break;
            
            default:
            break;
        }
    } while(opt != -1);
    
    argc -= optind;
    argv += optind;
    
    if(verbose)
    {
        if(serverURL == "")
            cerr << "Using local USB" << endl;
        else
            cerr << "Using remote device: " << serverURL << ":" << serverPortNum << endl;
        
        if(serialNum == "")
            cerr << "Using first device found" << endl;
        else
            cerr << "Looking for device: " << serialNum << endl;
    }
    
    try {
        // Check commands

        r = DoCommands(argc, argv);
        if(r != EXIT_SUCCESS)
            return r;
        
        // Open scope
        if(!dryRun)
        {
            Socket * sock = NULL;
            if(serverURL == "")
            {
                if(verbose) cerr << "Attempting to connect via USB" << endl;
                //scope = new DS1000E(new TMC_LocalDevice(0x1AB1, 0x0588, serialNum));
            }
            else
            {
                if(verbose) cerr << "Attempting to connect to remote device" << endl;
                while(!sock) {
                    usleep(10000);
                    sock = ClientConnect(serverURL, serverPortNum);
                }
                // if(verbose) cerr << "Connected to server: " << sock->other << endl;
                if(verbose) cerr << "Connected to server: " << endl;
                scope = new DS1000E(new TMC_RemoteDevice(0x1AB1, 0x0588, serialNum, sock->sockfd));
                sock->sockfd = -1;// TMC_RemoteDevice takes ownership of fd
                delete sock;
            }
        
            r = DoCommands(argc, argv);
        }
    }
    catch(exception & err)
    {
        cout << "Caught exception: " << err.what() << endl;
        return EXIT_FAILURE;
    }
    
    return r;
}



int DoCommands(int argc, char ** argv)
{
    // Perform commands, or do dry run if scope is NULL
    while(argc)
    {
        map<string, Command>::iterator cmd = commandMap.find(*argv);
        
        if(cmd == commandMap.end())
        {
            cerr << "Command not recognized: " << *argv;
            return EXIT_FAILURE;
        }
        
        if(cmd->second.handler)
        {
            cmd->second.handler(argc, &argv, cmd->second);
        }
        else
        {
            if(cmd->second.cmdStr != "")
                ScopeWrite(cmd->second.cmdStr);
            
            if(cmd->second.respLen != 0)
                ScopeRead(cmd->second.respLen);
            --argc;
            ++argv;
        }
    }
    return EXIT_SUCCESS;
}


void WRITE(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    if(argc)
    {
        ScopeWrite(**argv);
        --argc;
        ++(*argv);
    }
    else
    {
        throw FormattedError("w requires a parameter!");
    }
}

void READ(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    if(argc)
    {
        int respLen = strtol(**argv, NULL, 0);
        ScopeRead(respLen);
        --argc;
        ++(*argv);
    }
    else
    {
        throw FormattedError("r requires a parameter!");
    }
}

void QUERY(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    if(argc)
    {
        ScopeWrite(**argv);
        ScopeRead(256);
        --argc;
        ++(*argv);
    }
    else
    {
        throw FormattedError("w requires a parameter!");
    }
}

void ECHO(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    if(argc)
    {
        cout << **argv;
        --argc;
        ++(*argv);
    }
}

void NEWLINE(int & argc, char *** argv, Command & cmd)
{
    cout << endl;
    --argc;
    ++(*argv);
}

void ACQ_AVER(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    if(argc)
    {
        int avrPts = strtol(**argv, NULL, 0);
        char bfr[1024];
        snprintf(bfr, 1024, ":ACQ:AVER %d", avrPts);
        ScopeWrite(bfr);
        --argc;
        ++(*argv);
    }
    else
    {
        throw FormattedError("acq-aver requires a parameter!");
    }
}

void WAV_DATA(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    // Read waveform data
    // TODO: optionally rescale/reformat data, redirect to a file, etc
    ScopeWrite(cmd.cmdStr);
    ScopeRead(cmd.respLen);
}

string PrintParam(const std::string & cmd, const std::string & cmdName) {
    ScopeWrite(cmd);
    vector<uint8_t> bfr(1024);
    size_t respLen = scope->Read(&bfr[0], bfr.size());
    string resp(bfr.begin(), bfr.begin() + respLen);
    std::cout << cmdName << ": " << resp << endl;
    return resp;
}

void PrintChannelParams(const std::string & chan)
{
    PrintParam(":CHAN" + chan + ":BWL?", "Ch" + chan + "_BandwidthLimit");
    PrintParam(":CHAN" + chan + ":COUP?", "Ch" + chan + "_Coupling");
    PrintParam(":CHAN" + chan + ":DISP?", "Ch" + chan + "_Display");
    PrintParam(":CHAN" + chan + ":INV?", "Ch" + chan + "_Invert");
    PrintParam(":CHAN" + chan + ":OFFS?", "Ch" + chan + "_Offset");
    PrintParam(":CHAN" + chan + ":PROB?", "Ch" + chan + "_Probe");
    PrintParam(":CHAN" + chan + ":SCAL?", "Ch" + chan + "_Scale");
    PrintParam(":CHAN" + chan + ":FILT?", "Ch" + chan + "_Filter");
    PrintParam(":CHAN" + chan + ":MEMD?", "Ch" + chan + "_MemoryDepth");
    PrintParam(":CHAN" + chan + ":VERN?", "Ch" + chan + "_Vernier");
}

void GET_SCOPE_PARAMS(int & argc, char *** argv, Command & cmd)
{
    --argc;
    ++(*argv);
    
    if(!scope && verbose)
        cerr << "dry run get-scope-params" << endl;
    if(!scope)
        return;
    
    size_t respLen;
    IDN_Response idn = scope->Identify();
    cout << "Manufacturer: " << idn.manufacturer << endl;
    cout << "Model: " << idn.model << endl;
    cout << "Serial: " << idn.serial << endl;
    cout << "Version: " << idn.version << endl;
    
    bool hasLA = (idn.model == "DS1052D" || idn.model == "DS1102D");
    
    PrintParam(":ACQ:TYPE?", "AcquireType");
    PrintParam(":ACQ:MODE?", "AcquireMode");
    PrintParam(":ACQ:AVER?", "AcquireAverages");
    PrintParam(":ACQ:SAMP? CHAN1", "Ch1_SamplingRate");
    PrintParam(":ACQ:SAMP? CHAN2", "Ch2_SamplingRate");
    if(hasLA)
        PrintParam(":ACQ:SAMP? DIGITAL", "DigitalSamplingRate");
    PrintParam(":ACQ:MEMD?", "MemoryDepth");
    
    PrintParam(":DISP:TYPE?", "DisplayType");
    PrintParam(":DISP:GRID?", "DisplayGrid");
    PrintParam(":DISP:PERS?", "DisplayPersist");
    PrintParam(":DISP:MNUD?", "DisplayMenuDisplay");
    PrintParam(":DISP:MNUS?", "DisplayMenuStatus");
    PrintParam(":DISP:BRIG?", "DisplayBrightness");
    PrintParam(":DISP:INT?", "DisplayIntensity");
    
    PrintParam(":TIM:MODE?", "TimebaseMode");
    PrintParam(":TIM:OFFS?", "TimebaseOffset");
    PrintParam(":TIM:DEL:OFFS?", "DelayedTimebaseOffset");
    PrintParam(":TIM:SCAL?", "TimebaseScale");
    PrintParam(":TIM:DEL:SCAL?", "DelayedTimebaseScale");
    PrintParam(":TIM:FORM?", "TimebaseFormat");
    
    string trigMode = PrintParam(":TRIG:MODE?", "TriggerMode");
    PrintParam(":TRIG:HOLD?", "TriggerHoldoff");
    PrintParam(":TRIG:STAT?", "TriggerStatus");
    
    // Trigger modes: EDGE, PULS, VIDEO, SLOP, PATT, ALT
    if(trigMode == "EDGE")
    {
        PrintParam(":TRIG:EDGE:SOUR?", "TriggerSource");
        PrintParam(":TRIG:EDGE:LEV?", "TriggerLevel");
        PrintParam(":TRIG:EDGE:SWE?", "TriggerSweep");
        PrintParam(":TRIG:EDGE:COUP?", "TriggerCoupling");
        
        PrintParam(":TRIG:EDGE:SLOP?", "TriggerEdgeSlope");
        PrintParam(":TRIG:EDGE:SENS?", "TriggerEdgeSensitivity");
    }
    else if(trigMode == "PULS")
    {
        PrintParam(":TRIG:PULS:SOUR?", "TriggerSource");
        PrintParam(":TRIG:PULS:LEV?", "TriggerLevel");
        PrintParam(":TRIG:PULS:SWE?", "TriggerSweep");
        PrintParam(":TRIG:PULS:COUP?", "TriggerCoupling");
        
        PrintParam(":TRIG:PULS:MODE?", "TriggerPulseMode");
        PrintParam(":TRIG:PULS:SENS?", "TriggerPulseSensitivity");
        PrintParam(":TRIG:PULS:WIDT?", "TriggerPulseWidth");
    }
    else if(trigMode == "VIDEO")
    {
        PrintParam(":TRIG:VIDEO:SOUR?", "TriggerSource");
        PrintParam(":TRIG:VIDEO:LEV?", "TriggerLevel");
        PrintParam(":TRIG:VIDEO:SWE?", "TriggerSweep");
        PrintParam(":TRIG:VIDEO:COUP?", "TriggerCoupling");
        // TODO
    }
    else if(trigMode == "SLOP")
    {
        PrintParam(":TRIG:SLOP:SOUR?", "TriggerSource");
        PrintParam(":TRIG:SLOP:LEV?", "TriggerLevel");
        PrintParam(":TRIG:SLOP:SWE?", "TriggerSweep");
        PrintParam(":TRIG:SLOP:COUP?", "TriggerCoupling");
        // TODO
    }
    else if(trigMode == "PATT")
    {
        PrintParam(":TRIG:PATT:SOUR?", "TriggerSource");
        PrintParam(":TRIG:PATT:LEV?", "TriggerLevel");
        PrintParam(":TRIG:PATT:SWE?", "TriggerSweep");
        PrintParam(":TRIG:PATT:COUP?", "TriggerCoupling");
        // TODO
    }
    else if(trigMode == "ALT")
    {
        PrintParam(":TRIG:ALT:SOUR?", "TriggerSource");
        PrintParam(":TRIG:ALT:LEV?", "TriggerLevel");
        PrintParam(":TRIG:ALT:SWE?", "TriggerSweep");
        PrintParam(":TRIG:ALT:COUP?", "TriggerCoupling");
        // TODO
        
    }
    
    PrintParam(":MATH:DISP?", "MathDisplay");
    PrintParam(":MATH:OPER?", "MathOperation");
    PrintParam(":FFT:DISP?", "FFT_Display");
    
    PrintChannelParams("1");
    PrintChannelParams("2");
    
    // PrintParam(":MEAS:?", "Measure");
    
    PrintParam(":WAV:POIN:MODE?", "WaveformPointsMode");
    
    if(hasLA)
    {
        char bfr[1024];
        char bfr2[1024];
        PrintParam(":LA:DISP?", "LogicDisplay");
        for(int j = 0; j < 16; ++j)
        {
            snprintf(bfr, 1024, ":DIG%d:TURN?", j);
            snprintf(bfr2, 1024, "Digital%d_Enabled", j);
            PrintParam(bfr, bfr2);
            snprintf(bfr, 1024, ":DIG%d:POS?", j);
            snprintf(bfr2, 1024, "Digital%d_Position", j);
            PrintParam(bfr, bfr2);
        }
        PrintParam(":LA:THR?", "LogicDisplay");
        PrintParam(":LA:GROU1?", "LogicGroup1");
        PrintParam(":LA:GROU1:SIZ?", "LogicGroup1_Size");
        PrintParam(":LA:GROU2?", "LogicGroup2");
        PrintParam(":LA:GROU2:SIZ?", "LogicGroup2_Size");
    }
    
    PrintParam(":KEY:LOCK?", "KeysLocked");
    PrintParam(":INFO:LANG?", "Language");
    PrintParam(":COUN:ENAB?", "FreqCounterEnabled");
    PrintParam(":BEEP:ENAB?", "BeepEnabled");
}

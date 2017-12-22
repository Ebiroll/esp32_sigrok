// g++  `GraphicsMagick++-config --cxxflags --cppflags  --ldflags --libs` rigwfm.cpp -o rigwfm
// for i in *.wfm; do ./rigwfm ${i} ${i}.png; done

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <vector>

#include <cstdlib>
#include <cmath>
#include <cfloat>

#include <unistd.h>
#include <Magick++.h>

#include "rigolwfm.h"

using namespace std;
using namespace rigwfm;
using namespace Magick;

// Ternary value type
// 0: false, 1: true, -1: neither (undefined boolean value)
typedef int8_t tern_t;

const tern_t unknown = -1;


struct PlotOpts {
    int smoothing;
    double startT, endT;// start and end time referenced from start, in sample periods
    tern_t ch1enab;
    tern_t ch2enab;
    
    PlotOpts():
        smoothing(1),
        startT(DBL_MIN),
        endT(DBL_MAX),
        ch1enab(unknown),
        ch2enab(unknown)
    {}
};


// Plot waveform as DrawablePolyline
// smoothing: n consecutive points are averaged together to do a simple low pass filter
void PlotChannel(Image & image, std::list<Magick::Drawable> & drawList, const PlotOpts & opts, const RigolData & channel);
void PlotWaveform(const std::string foutPath, const PlotOpts & opts, const RigolWaveform & wfm);
void PlotGrid(Image & image, std::list<Magick::Drawable> & drawList, const PlotOpts & opts, const RigolWaveform & wfm);

void ParseOpts(PlotOpts & opts, int argc, const char * argv[]);

int main(int argc, const char * argv[])
{
    if(argc < 3)
    {
        cerr << "Usage: rigwfm WAVEFORM_FILE IMAGE_FILE";
        return EXIT_FAILURE;
    }
    
    InitializeMagick(*argv);
    
    try {
        PlotOpts opts;
        opts.smoothing = 4;
        
        ParseOpts(opts, argc, argv);
        
        RigolWaveform waveform(argv[1]);
        cout << waveform << endl;
        
        if(opts.ch1enab == unknown) opts.ch1enab = !waveform.channels[0].data.empty();
        if(opts.ch2enab == unknown) opts.ch1enab = !waveform.channels[1].data.empty();
        
        PlotWaveform(argv[2], opts, waveform);
    }
    catch(exception & err)
    {
        cout << "Caught exception: " << err.what() << endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

static const char * optString = "Il:o:vh?";
static const struct option longOpts[] = {
    {"plot", no_argument, NULL, 'p'},
    {"smoothing",  required_argument, NULL, 0},
    {"tstart",  required_argument, NULL, 0},
    {"tend",    required_argument, NULL, 0},
    {"tcenter", required_argument, NULL, 0},
    {"file", required_argument, NULL, 'f'},
    {NULL, 0, NULL, 0}
};
void ParseOpts(PlotOpts & opts, int argc, const char * argv[])
{
    int renderPlot = 0;
    int opt, optInd = 0;
    do {
        opt = getopt_long(argc, argv, optString, longOpts, &optInd);
        // optarg, optind, optopt
        switch(opt) {
            case p:
            break;
            
            case 0:
                if(strcmp("randomize", longOpts[longIndex].name) == 0) {
                    globalArgs.randomized = 1;
                }
            break
            
            case -1:
            break;
        }
    } while(opt != -1);
}


void PlotWaveform(const std::string foutPath, const PlotOpts & opts, const RigolWaveform & wfm)
{
    // Image image = Image("1024x512", "white");
    Image image = Image("2048x512", "white");
    
    std::list<Magick::Drawable> drawList;
    drawList.push_back(DrawablePushGraphicContext());
    drawList.push_back(DrawableViewbox(0, 0, image.columns(), image.rows()));
    
    drawList.push_back(DrawableFillColor(Color()));
    
    PlotGrid(image, drawList, opts, wfm);
    
    drawList.push_back(DrawableStrokeWidth(1.0));
    if(opts.ch1enab) {
        drawList.push_back(DrawableStrokeColor("#F00"));
        PlotChannel(image, drawList, opts, wfm.channels[0]);
    }
    if(opts.ch1enab) {
        drawList.push_back(DrawableStrokeColor("#00F"));
        PlotChannel(image, drawList, opts, wfm.channels[1]);
    }
    
    // opts.startT = wfm.npoints/2.0 - 6*50e-6*wfm.fs;
    // opts.endT = wfm.npoints/2.0 - 5*50e-6*wfm.fs;;
    // // opts.startT = wfm.npoints/2.0 - 6*5e-3*wfm.fs;
    // // opts.endT = wfm.npoints/2.0 + 0;
    // // opts.endT = wfm.npoints/2.0 + 6*5e-3*wfm.fs;
    // drawList.push_back(DrawableStrokeColor("#0B0"));
    // PlotChannel(image, drawList, opts, wfm.channels[0]);
    
    drawList.push_back(DrawablePopGraphicContext());
    
    image.draw(drawList);
    image.write(std::string(foutPath));
}

void PlotGrid(Image & image, std::list<Magick::Drawable> & drawList, const PlotOpts & opts, const RigolWaveform & wfm)
{
    // Vertical rules
    // Trigger delay, etc are referenced from the midpoint of the captured waveform
    double midT = wfm.npoints/2.0/wfm.fs;
    double vrule;
    drawList.push_back(DrawableStrokeColor("#444"));
    for(int j = 1; j < 6; ++j) {
        vrule = wfm.fs*(midT - wfm.tscale*j)*image.columns()/wfm.npoints;
        drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
        vrule = wfm.fs*(midT + wfm.tscale*j)*image.columns()/wfm.npoints;
        drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
    }
    drawList.push_back(DrawableStrokeWidth(0.5));
    drawList.push_back(DrawableStrokeColor("#AAA"));
    for(int j = 1; j < 7; ++j) {
        for(int k = 1; k < 5; ++k) {
            vrule = wfm.fs*(midT - wfm.tscale*(j - k/5.0))*image.columns()/wfm.npoints;
            drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
            vrule = wfm.fs*(midT + wfm.tscale*(j - k/5.0))*image.columns()/wfm.npoints;
            drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
        }
    }
    
    drawList.push_back(DrawableStrokeWidth(2.0));
    drawList.push_back(DrawableStrokeColor("#000"));
    vrule = wfm.fs*(midT - wfm.tscale*6)*image.columns()/wfm.npoints;
    drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
    vrule = wfm.fs*midT*image.columns()/wfm.npoints;
    drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
    vrule = wfm.fs*(midT + wfm.tscale*6)*image.columns()/wfm.npoints;
    drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
    
    // Trigger position
    drawList.push_back(DrawableStrokeWidth(1.0));
    drawList.push_back(DrawableStrokeColor("#990"));
    vrule = wfm.fs*(midT - wfm.tdelay)*image.columns()/wfm.npoints;
    drawList.push_back(DrawableLine(vrule, 0, vrule, image.rows()));
    
    // Horizontal rules
    double hrule;
    drawList.push_back(DrawableStrokeColor("#444"));
    for(int j = 1; j < 4; ++j) {
        hrule = image.rows() - (4 - j)*(image.rows()/8.0);
        drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
        hrule = image.rows() - (4 + j)*(image.rows()/8.0);
        drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
    }
    drawList.push_back(DrawableStrokeWidth(0.5));
    drawList.push_back(DrawableStrokeColor("#AAA"));
    for(int j = 1; j < 5; ++j) {
        for(int k = 1; k < 5; ++k) {
            hrule = image.rows() - (4 + k/5.0 - j)*(image.rows()/8.0);
            drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
            hrule = image.rows() - (4 - k/5.0 + j)*(image.rows()/8.0);
            drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
        }
    }
    
    drawList.push_back(DrawableStrokeWidth(2.0));
    drawList.push_back(DrawableStrokeColor("#000"));
    hrule = image.rows() - (4)*(image.rows()/8.0);
    drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
    hrule = image.rows() - (0)*(image.rows()/8.0);
    drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
    hrule = image.rows() - (8)*(image.rows()/8.0);
    drawList.push_back(DrawableLine(0, hrule, image.columns(), hrule));
}

void PlotChannel(Image & image, std::list<Magick::Drawable> & drawList, const PlotOpts & opts, const RigolData & channel)
{
    double width = image.columns(), height = image.rows();
    
    double startT = (opts.startT != DBL_MIN)? opts.startT : 0.0;
    double endT = (opts.endT != DBL_MAX)? opts.endT : channel.data.size();
    
    std::list<Coordinate> vertices;
    double yscl = height/8.0/channel.scale;
    double yoff = channel.offset;
    size_t npts = endT - startT;
    if(width >= npts)
    {
        double dx = (double)npts/width;
        for(int j = 0; j < width; ++j) {
            double y = SincReconstruct(channel.data, (float)j*dx + startT, 64);
            vertices.push_back(Coordinate(j, height/2 - (y + yoff)*yscl));
        }
    }
    else
    {
        for(int j = std::max(0, (int)floor(startT)); (j + opts.smoothing) <= channel.data.size(); j += opts.smoothing) {
            double y = 0;
            for(int k = 0; k < opts.smoothing; ++k)
                y += channel.data[j + k];
            y /= opts.smoothing;
            vertices.push_back(Coordinate(((double)j - startT)/npts*width, height/2 - (y + yoff)*yscl));
        }
    }
    drawList.push_back(DrawablePolyline(vertices));
}


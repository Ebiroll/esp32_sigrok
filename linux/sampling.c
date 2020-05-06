// This works as a proxy, receiving udp packets from 
#include <stdbool.h>
#include <stdio.h>
int trig_pin;

#define NUM_SAMPLES 1024*1024

static int maxSamples=NUM_SAMPLES;

void set_mem_depth(int depth) {
    if (depth<NUM_SAMPLES) {
        maxSamples=depth;
    }
}


int stop_aq=false;
void stop_aquisition() {
    stop_aq=true;
}

void setTimescale(float scale){

  printf("setTimescale=%.3f\n",scale);

}

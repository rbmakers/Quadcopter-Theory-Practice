#pragma once
#include <stdint.h>
#include <math.h>
struct TimingStats {
    uint32_t count=0; double mean=0,m2=0,min=0,max=0;
    void add(double v){
        if(!count){min=max=v;} else {if(v<min)min=v;if(v>max)max=v;}
        ++count;double d=v-mean;mean+=d/count;m2+=d*(v-mean);
    }
    double sd() const {return count>1?sqrt(m2/(count-1)):0;}
};
// Shift-right IN 2 + autopush32: oldest pair ends in bits 1:0.
inline uint8_t packedPair(const uint32_t *words,uint32_t sample){
    return uint8_t((words[sample/16] >> (2*(sample%16)))&3u);
}
inline uint32_t extractEdges(const uint32_t *words,uint32_t samples,uint32_t channel,
                            uint32_t *edges,uint32_t capacity,bool &overflow){
    overflow=false;if(!samples)return 0;
    uint32_t n=0;uint8_t prev=(packedPair(words,0)>>channel)&1u;
    for(uint32_t i=1;i<samples;++i){
        uint8_t now=(packedPair(words,i)>>channel)&1u;
        if(now&&!prev){if(n<capacity)edges[n++]=i;else overflow=true;}
        prev=now;
    }
    return n;
}

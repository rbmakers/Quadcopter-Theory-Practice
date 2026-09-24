#include <initializer_list>
#include "../Lab02_PIO_INT/TimingAnalysis.h"
#include <cassert>
#include <cstdio>
int main(){
 uint32_t words[4]={},edges[16]={};bool overflow;
 // Simultaneous highs at sample 4,5 then 20,21. Earliest pair is LSB.
 for(uint32_t i=0;i<64;++i){uint32_t level=((i>=4&&i<6)||(i>=20&&i<22))?3:0;
   words[i/16]|=level<<(2*(i%16));}
 for(uint32_t ch=0;ch<2;++ch){auto n=extractEdges(words,64,ch,edges,16,overflow);
   assert(n==2&&!overflow&&edges[0]==4&&edges[1]==20);}
 assert(extractEdges(words,64,0,edges,1,overflow)==1&&overflow);
 uint32_t high[1]={0xFFFFFFFF};assert(extractEdges(high,16,0,edges,16,overflow)==0);
 TimingStats s;for(double t:{999.,1000.,1001.})s.add(t);
 assert(s.count==3&&s.mean==1000&&s.min==999&&s.max==1001&&fabs(s.sd()-1)<1e-12);
 puts("PASS: packing, simultaneous edges, finite-capacity overflow, initial-high handling, sample standard deviation");
}

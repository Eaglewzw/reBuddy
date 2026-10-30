#include "rb_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    int16_t from[6]={-3000,1000,0,0,0,0},to[6]={3000,-1000,0,0,0,0};
    rb_motion_limit_t limits[6];
    for(unsigned i=0;i<6;++i)limits[i]=(rb_motion_limit_t){-9000,9000,6000,12000};
    uint32_t duration=0;
    assert(rb_motion_bridge_duration(from,to,limits,300,&duration));
    assert(duration>=1875); /* At least 1.875*6000/6000 seconds. */
    float pose[6],previous[6],before[6];
    assert(rb_motion_bridge_sample(from,to,0,duration,previous));
    for(unsigned i=0;i<6;++i)assert(previous[i]==from[i]);
    for(uint32_t t=1;t<=duration;++t) {
        assert(rb_motion_bridge_sample(from,to,t,duration,pose));
        for(unsigned i=0;i<6;++i) {
            assert(pose[i]>=limits[i].minimum && pose[i]<=limits[i].maximum);
            assert(fabsf(pose[i]-previous[i])*1000<=limits[i].velocity+20);
            /* Float output quantization limits numeric second differences;
               test acceleration with a wider, independent finite difference. */
        }
        for(unsigned i=0;i<6;++i)previous[i]=pose[i];
    }
    for(unsigned i=0;i<6;++i)assert(pose[i]==to[i]);
    for(uint32_t t=20;t+20<=duration;t+=20) {
        rb_motion_bridge_sample(from,to,t-20,duration,before);
        rb_motion_bridge_sample(from,to,t,duration,pose);
        rb_motion_bridge_sample(from,to,t+20,duration,previous);
        for(unsigned i=0;i<6;++i)
            assert(fabsf(previous[i]-2*pose[i]+before[i])/.0004f<limits[i].acceleration+80);
    }
    assert(rb_motion_ease(0)==0 && rb_motion_ease(1)==1);
    assert(rb_motion_ease(.001f)<.00000002f);
    limits[0].velocity=0;assert(!rb_motion_bridge_duration(from,to,limits,0,&duration));
    limits[0].velocity=6000;to[0]=10000;
    assert(!rb_motion_bridge_duration(from,to,limits,0,&duration));
    assert(!rb_motion_bridge_sample(from,to,0,0,pose));
    assert(!rb_motion_bridge_sample(NULL,to,0,100,pose));
    puts("PASS compact motion bridge bounds, speed/acceleration, endpoints and invalid input; no motor backend");
}

#include "rb_dance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static rb_dance_plan_t plan,slow;
static void sample(const rb_dance_plan_t *p,float t,float out[3][7])
{assert(rb_dance_sample(p,t,out[0],out[1],out[2],NULL,NULL));}
int main(void)
{
    uint8_t all[32];for(unsigned i=0;i<32;++i)all[i]=(uint8_t)(i%10+1);
    assert(!rb_dance_compile(&plan,all,0,1));assert(!rb_dance_compile(&plan,all,33,1));
    assert(!rb_dance_compile(&plan,all,10,1.25f));assert(!rb_dance_compile(&plan,all,10,NAN));
    uint8_t bad[]={0,11};assert(!rb_dance_compile(&plan,bad,2,1));
    assert(rb_dance_compile(&plan,all,32,1));assert(plan.part_count==63);
    for(unsigned left=1;left<=10;++left)for(unsigned right=1;right<=10;++right) {
        uint8_t numbers[]={(uint8_t)left,(uint8_t)right};
        assert(rb_dance_compile(&plan,numbers,2,1));assert(rb_dance_compile(&slow,numbers,2,.5f));
        assert(plan.part_count==3);assert(fabsf(slow.duration_s-2*plan.duration_s)<1e-4f);
        for(unsigned i=0;i<=100;++i) {
            float t=plan.duration_s*i/100.f,q[3][7],r[3][7];sample(&plan,t,q);sample(&slow,2*t,r);
            for(unsigned j=0;j<7;++j) {
                assert(fabsf(q[0][j]-r[0][j])<1e-5f);
                assert(fabsf(q[1][j]*.5f-r[1][j])<1e-5f);
                assert(fabsf(q[2][j]*.25f-r[2][j])<1e-5f);
                assert(fabsf(q[1][j])<(j==6?2.81f:1.01f));
                assert(fabsf(q[2][j])<(j==6?10.01f:2.01f));
            }
        }
        for(unsigned i=1;i<plan.part_count;++i) {
            float t=plan.parts[i].begin_s,q[3][7],r[3][7];sample(&plan,t-1e-5f,q);sample(&plan,t,r);
            for(unsigned j=0;j<7;++j) {
                assert(fabsf(q[0][j]-r[0][j])<1e-4f);
                assert(fabsf(q[1][j]-r[1][j])<2e-4f);
                assert(fabsf(q[2][j]-r[2][j])<.01f);
            }
        }
    }
    unsigned seen=0;
    for(uint32_t seed=0;seed<100;++seed) {
        uint8_t a[10],b[10];assert(rb_dance_random(a,10,seed));assert(rb_dance_random(b,10,seed));
        assert(!memcmp(a,b,10));unsigned mask=0;
        for(unsigned j=0;j<10;++j) {assert(a[j]>=1 && a[j]<=10);mask|=1u<<(a[j]-1);}
        assert(mask==1023);seen|=1u<<(a[0]-1);
    }
    assert(seen==1023);assert(!rb_dance_random(all,11,1));assert(!rb_dance_random(all,0,1));
    printf("PASS native 100 joins, 32-action capacity, scaling, boundaries and seeded random; plan RAM %zu bytes\n",sizeof(plan));
    return 0;
}

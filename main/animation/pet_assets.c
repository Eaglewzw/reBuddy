#include "pet_assets.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "miniz.h"
#include "esp_timer.h"
static tinfl_decompressor *inflater;
#else
#include <zlib.h>
#include <time.h>
#endif
#define PIXEL_BYTES RB_FRAME_BYTES
static const uint8_t *blob;
static size_t blob_size;
static struct { unsigned index; uint8_t *pixels; lv_image_dsc_t image; } slots[2];
static unsigned slot_count,next_slot;
static uint8_t *base_pixels;
static uint8_t *dance_planes;
static unsigned cached_base=UINT32_MAX;
static bool degraded;
static rb_asset_stats_t stats;
static uint64_t decode_clock(void)
{
#ifdef ESP_PLATFORM
    return (uint64_t)esp_timer_get_time();
#else
    struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;
#endif
}
static lv_image_dsc_t direct,room_image;
#define DANCE_PIXELS (RB_FRAME_WIDTH*RB_FRAME_HEIGHT)
static void *asset_alloc(size_t bytes)
{
#ifdef ESP_PLATFORM
    return heap_caps_malloc(bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
#else
    return malloc(bytes);
#endif
}
static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0]|((uint16_t)p[1]<<8); }
static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
bool rb_assets_set_room(const uint8_t *data,size_t bytes)
{
    memset(&room_image,0,sizeof(room_image));
    for (unsigned i=0;i<slot_count;++i) slots[i].index=UINT32_MAX;
    if (!data || bytes!=153612u || data[0]!=LV_IMAGE_HEADER_MAGIC ||
        data[1]!=LV_COLOR_FORMAT_RGB565 || le16(data+2)!=0 || le16(data+4)!=320 ||
        le16(data+6)!=240 || le16(data+8)!=640) return false;
    room_image=(lv_image_dsc_t){.header={.magic=LV_IMAGE_HEADER_MAGIC,.cf=LV_COLOR_FORMAT_RGB565,
        .w=320,.h=240,.stride=640},.data_size=153600,.data=data+12};
    return true;
}
const lv_image_dsc_t *rb_assets_room(void) { return room_image.data?&room_image:NULL; }
static const uint8_t *frame_record(unsigned index)
{
    if (!blob || index>=RB_FRAME_COUNT) return NULL;
    size_t offset=rb_frames[index].offset,bytes=rb_frames[index].bytes;
    if (offset>blob_size || bytes<=16 || bytes>blob_size-offset) return NULL;
    const uint8_t *h=blob+offset;
    if (memcmp(h,"RBG2",4) || le16(h+4)!=RB_FRAME_WIDTH || le16(h+6)!=RB_FRAME_HEIGHT ||
        le32(h+8)!=PIXEL_BYTES || le32(h+12)>7) return NULL;
    if (le32(h+12)==0 && bytes!=16+PIXEL_BYTES) return NULL;
    if (le32(h+12)==2 && bytes<=20) return NULL;
    if (le32(h+12)==5 && bytes<=20) return NULL;
    if (le32(h+12)==4 && bytes<=20) return NULL;
    if (le32(h+12)==6 && bytes<=24) return NULL;
    if (le32(h+12)==7 && bytes<=20) return NULL;
    return h;
}
static lv_image_dsc_t descriptor(const uint8_t *pixels)
{
    return (lv_image_dsc_t){.header={.magic=LV_IMAGE_HEADER_MAGIC,.cf=LV_COLOR_FORMAT_RGB565,
        .w=RB_FRAME_WIDTH,.h=RB_FRAME_HEIGHT,.stride=RB_FRAME_WIDTH*2},
        .data_size=PIXEL_BYTES,.data=pixels};
}
void rb_assets_close(void)
{
    for (unsigned i=0;i<slot_count;++i) { lv_image_cache_drop(&slots[i].image); free(slots[i].pixels); }
    lv_image_cache_drop(&direct);
    free(dance_planes);dance_planes=NULL;
    free(base_pixels); base_pixels=NULL; cached_base=UINT32_MAX;
#ifdef ESP_PLATFORM
    free(inflater); inflater=NULL;
#endif
    slot_count=next_slot=0; blob=NULL; blob_size=0;
    memset(&direct,0,sizeof(direct));
}
bool rb_assets_init(const uint8_t *data,size_t bytes,unsigned cache_slots)
{
    rb_assets_close(); blob=data; blob_size=bytes; degraded=false;
    memset(&stats,0,sizeof(stats));
    bool has_delta=false;
    for (unsigned i=0;i<RB_FRAME_COUNT;++i) {
        const uint8_t *record=frame_record(i);
        if (!record || (i==0 && le32(record+12)!=0)) {
            blob=NULL; blob_size=0; degraded=true; return false;
        }
        unsigned codec=le32(record+12);
        if(codec>=4)has_delta=true;
        if (codec==2 || codec==5 || codec==7) {
            has_delta=true;
            unsigned base=le32(record+16);
            const uint8_t *base_record=frame_record(base);
            /* A delta may reference only a complete raw, zlib or palette frame. This
               rules out self-references, cycles and unbounded decode chains. */
            if (!base_record || (codec>=5?(base>=i || (le32(base_record+12)!=codec-1 && le32(base_record+12)!=codec)):
                (le32(base_record+12)==2 || le32(base_record+12)>3))) {
                blob=NULL; blob_size=0; degraded=true; return false;
            }
        }
        if(codec==5 || codec==7) {
            unsigned node=i,depth=0;
            const uint8_t *r=record;
            while(le32(r+12)==codec) {
                unsigned next=le32(r+16);
                if(++depth>7 || next>=node || !(r=frame_record(next)) || (le32(r+12)!=codec-1 && le32(r+12)!=codec)) {
                    blob=NULL;blob_size=0;degraded=true;return false;
                }
                node=next;
            }
        }
    }
#ifdef ESP_PLATFORM
    /* ROM inflate state is too large for the LVGL task stack. */
    if (cache_slots) inflater=calloc(1,sizeof(*inflater));
    if (!inflater) cache_slots=0;
#endif
    for (unsigned i=0;i<cache_slots && i<2;++i) {
#ifdef ESP_PLATFORM
        uint8_t *pixels=heap_caps_malloc(PIXEL_BYTES,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
#else
        uint8_t *pixels=malloc(PIXEL_BYTES);
#endif
        if (!pixels) break;
        slots[i].pixels=pixels; slots[i].index=UINT32_MAX;
        slots[i].image=descriptor(pixels); ++slot_count;
    }
    if (has_delta && cache_slots) {
#ifdef ESP_PLATFORM
        base_pixels=heap_caps_malloc(PIXEL_BYTES,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
#else
        base_pixels=malloc(PIXEL_BYTES);
#endif
        if (!base_pixels) degraded=true;
    }
    if (slot_count<2) degraded=true;
    return true;
}
static bool inflate_buffer(const uint8_t *source,size_t length,uint8_t *pixels,size_t expected)
{
#ifdef ESP_PLATFORM
    if (!inflater) return false;
    tinfl_init(inflater);
    size_t input=length,output=expected;
    tinfl_status status=tinfl_decompress(inflater,source,&input,pixels,pixels,&output,
        TINFL_FLAG_PARSE_ZLIB_HEADER|TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    return status==TINFL_STATUS_DONE && input==length && output==expected;
#else
    uLongf output=expected;
    return uncompress(pixels,&output,source,length)==Z_OK && output==expected;
#endif
}
static bool inflate_pixels(const uint8_t *source,size_t length,uint8_t *pixels)
{ return inflate_buffer(source,length,pixels,PIXEL_BYTES); }
static bool palette_planes(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    size_t bytes=rb_frames[frame].bytes;
    unsigned count=le16(record+16),size=le32(record+12)==6?le16(record+20):0;
    if(le32(record+12)==6 && (size<2 || size>240 || le16(record+22)!=size))return false;
    size_t header=(size?24:20)+count*2,expected=size?(size_t)size*size*2:PIXEL_BYTES;
    return count && count<=256 && bytes>header &&
        inflate_buffer(record+header,bytes-header,pixels,expected);
}
/* The stored colors have a white matte. Recover premultiplied foreground
 * before interpolation; room pixels are sampled only at final screen size. */
static bool composite_native(const uint8_t *palette,const uint8_t *planes,uint8_t *pixels)
{
    unsigned count=le16(palette+16),size=le16(palette+20),n=size*size;
    if(!count || count>256 || size<2 || size>240 || le16(palette+22)!=size)return false;
    uint16_t key=le16(palette+18);
    int matte[3]={(key>>11)*255/31,((key>>5)&63)*255/63,(key&31)*255/31};
    static int colors[256][3];
    for(unsigned i=0;i<count;++i) {
        uint16_t c=le16(palette+24+2*i);
        colors[i][0]=(c>>11)*255/31;colors[i][1]=((c>>5)&63)*255/63;colors[i][2]=(c&31)*255/31;
    }
    for(unsigned i=0;i<n;++i)if(planes[i]>=count)return false;
    static unsigned low[240],high[240],fraction[240];
    for(unsigned i=0;i<240;++i) {
        int coord=(int)(((2*i+1)*size*128)/240)-128;
        if(coord<0)coord=0;
        if(coord>(int)(size-1)*256)coord=(int)(size-1)*256;
        low[i]=(unsigned)coord/256;high[i]=low[i]+1<size?low[i]+1:low[i];fraction[i]=(unsigned)coord%256;
    }
    memcpy(pixels,room_image.data,PIXEL_BYTES);
    for(unsigned y=0;y<240;++y)for(unsigned x=0;x<240;++x) {
        unsigned sx[4]={low[x],high[x],low[x],high[x]},sy[4]={low[y],low[y],high[y],high[y]};
        if(!planes[n+sy[0]*size+sx[0]] && !planes[n+sy[1]*size+sx[1]] &&
           !planes[n+sy[2]*size+sx[2]] && !planes[n+sy[3]*size+sx[3]])continue;
        unsigned wx=fraction[x],wy=fraction[y];
        unsigned weights[4]={(256-wx)*(256-wy),wx*(256-wy),(256-wx)*wy,wx*wy};
        unsigned total[4]={0};
        for(unsigned k=0;k<4;++k) {
            unsigned i=sy[k]*size+sx[k],alpha=planes[n+i],index=planes[i];
            if(!alpha)continue;
            total[3]+=alpha*weights[k];
            for(unsigned j=0;j<3;++j) {
                int value=colors[index][j]-(int)((255-alpha)*(unsigned)matte[j]/255);
                if(value>(int)alpha)value=(int)alpha;
                total[j]+=(unsigned)(value>0?value:0)*weights[k];
            }
        }
        unsigned alpha=(total[3]+32768)/65536;
        if(!alpha)continue;
        unsigned at=y*320+x+40;uint16_t bg=le16(room_image.data+2*at);
        unsigned background[3]={(bg>>11)*255/31,((bg>>5)&63)*255/63,(bg&31)*255/31},c[3];
        for(unsigned j=0;j<3;++j) {
            c[j]=(total[j]+32768)/65536+(255-alpha)*background[j]/255;
            if(c[j]>255)c[j]=255;
        }
        uint16_t value=(uint16_t)(((c[0]>>3)<<11)|((c[1]>>2)<<5)|(c[2]>>3));
        pixels[2*at]=(uint8_t)value;pixels[2*at+1]=(uint8_t)(value>>8);
    }
    return true;
}
static bool decode_dance(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    if (!room_image.data) return false;
    if (!dance_planes) dance_planes=asset_alloc(PIXEL_BYTES);
    if (!dance_planes) return false;
    if(!base_pixels)return false;
    unsigned chain[8],length=0,current=frame;
    const uint8_t *palette=record;
    for(;;) {
        if(length>=8 || !palette)return false;
        chain[length++]=current;
        unsigned codec=le32(palette+12);
        if(codec==4 || codec==6)break;
        if(codec!=5 && codec!=7)return false;
        unsigned next=le32(palette+16);
        if(next>=current)return false;
        current=next;palette=frame_record(current);
    }
    unsigned size=le32(palette+12)==6?le16(palette+20):0;
    if(le32(palette+12)==6 && (size<2 || size>240 || le16(palette+22)!=size))return false;
    unsigned plane_bytes=size?size*size*2:PIXEL_BYTES;
    int reuse=-1;
    for(unsigned n=0;n<length;++n)if(chain[n]==cached_base)reuse=(int)n;
    if(reuse<0) {
        cached_base=UINT32_MAX;
        if(!palette_planes(current,palette,base_pixels))return false;
        cached_base=current;reuse=(int)length-1;
    }
    while(reuse>0) {
        unsigned next=chain[--reuse];const uint8_t *delta=frame_record(next);
        cached_base=UINT32_MAX;
        if(!delta || !inflate_buffer(delta+20,rb_frames[next].bytes-20,dance_planes,plane_bytes))return false;
        for(unsigned i=0;i<plane_bytes;++i)base_pixels[i]^=dance_planes[i];
        cached_base=next;
    }
    if(le32(palette+12)==6) {
        if(composite_native(palette,base_pixels,pixels))return true;
        cached_base=UINT32_MAX;return false;
    }
    unsigned count=le16(palette+16);
    uint16_t key=le16(palette+18);
    int matte[3]={(key>>11)*255/31,((key>>5)&63)*255/63,(key&31)*255/31};
    for (unsigned i=0;i<PIXEL_BYTES/2;++i) {
        unsigned index=base_pixels[i],alpha=base_pixels[PIXEL_BYTES/2+i];
        if (index>=count) { cached_base=UINT32_MAX;return false; }
        uint16_t pixel=le16(palette+20+index*2),room=le16(room_image.data+i*2);
        if (!alpha) pixel=room;
        else if (alpha<255) {
            int c[3]={(pixel>>11)*255/31,((pixel>>5)&63)*255/63,(pixel&31)*255/31};
            int bg[3]={(room>>11)*255/31,((room>>5)&63)*255/63,(room&31)*255/31};
            for(unsigned j=0;j<3;++j) {
                c[j]+=(int)(255-alpha)*(bg[j]-matte[j])/255;
                if(c[j]<0)c[j]=0;
                if(c[j]>255)c[j]=255;
            }
            pixel=(uint16_t)(((c[0]>>3)<<11)|((c[1]>>2)<<5)|(c[2]>>3));
        }
        pixels[2*i]=(uint8_t)pixel;pixels[2*i+1]=(uint8_t)(pixel>>8);
    }
    return true;
}
static bool decode_complete(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    unsigned codec=le32(record+12);
    if (codec==0) { memcpy(pixels,record+16,PIXEL_BYTES); return true; }
    if(codec==3) {
        size_t bytes=rb_frames[frame].bytes;
        if(bytes<20) return false;
        unsigned count=le16(record+16);size_t header=18+count*2;
        if(!count || count>256 || bytes<=header) return false;
        if(!inflate_buffer(record+header,bytes-header,pixels,PIXEL_BYTES/2)) return false;
        /* Expand backwards in the same buffer; future indices stay intact. */
        for(size_t i=PIXEL_BYTES/2;i--;) {
            unsigned index=pixels[i];if(index>=count)return false;
            pixels[2*i]=record[18+2*index];pixels[2*i+1]=record[19+2*index];
        }
        return true;
    }
    return codec==1 && inflate_pixels(record+16,rb_frames[frame].bytes-16,pixels);
}
static bool decode(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    if (le32(record+12)>=4) return decode_dance(frame,record,pixels);
    if (le32(record+12)!=2) return decode_complete(frame,record,pixels);
    if (!base_pixels) return false;
    unsigned base=le32(record+16);
    const uint8_t *base_record=frame_record(base);
    if (!base_record || le32(base_record+12)==2) return false;
    if (!inflate_pixels(record+20,rb_frames[frame].bytes-20,pixels)) return false;
    if (cached_base!=base) {
        /* A failed inflate may partially overwrite the buffer. Never retain
           its previous cache identity or advertise the new base prematurely. */
        cached_base=UINT32_MAX;
        if (!decode_complete(base,base_record,base_pixels)) return false;
        cached_base=base;
    }
    for (size_t i=0;i<PIXEL_BYTES;++i) pixels[i]^=base_pixels[i];
    return true;
}
static const lv_image_dsc_t *fallback(const rb_frame_meta_t **meta)
{
    degraded=true;
    const uint8_t *record=frame_record(0);
    if (!record || le32(record+12)!=0) return NULL;
    *meta=&rb_frames[0]; direct=descriptor(record+16); return &direct;
}
const lv_image_dsc_t *rb_assets_frame(unsigned frame,const rb_frame_meta_t **meta)
{
    if (!meta) return NULL;
    const uint8_t *record=frame_record(frame);
    if (!record || slot_count<2) return fallback(meta);
    for (unsigned i=0;i<slot_count;++i) if (slots[i].index==frame) {
        *meta=&rb_frames[frame]; return &slots[i].image;
    }
    unsigned i=next_slot++%slot_count;
    lv_image_cache_drop(&slots[i].image);
    slots[i].index=UINT32_MAX;
    uint64_t began=decode_clock();
    bool ok=decode(frame,record,slots[i].pixels);
    uint32_t cost=(uint32_t)(decode_clock()-began);
    ++stats.frames;stats.total_us+=cost;if(cost>stats.max_us)stats.max_us=cost;
    if (!ok) { ++stats.failures;return fallback(meta); }
    slots[i].index=frame; *meta=&rb_frames[frame]; return &slots[i].image;
}
/* Dance frames already carry offline coverage; decode directly into the regular
   double cache. No flood fill, temporary image, or per-frame work queue. */
const lv_image_dsc_t *rb_assets_dance_frame(unsigned frame,const rb_frame_meta_t **meta)
{
    if (!meta || !room_image.data) return NULL;
    const uint8_t *record=frame_record(frame);
    if (!record || le32(record+12)<4) return NULL;
    const lv_image_dsc_t *image=rb_assets_frame(frame,meta);
    return image && *meta==&rb_frames[frame]?image:NULL;
}
size_t rb_assets_cache_bytes(void)
{ return (slot_count+(base_pixels?1u:0u)+(dance_planes?1u:0u))*PIXEL_BYTES; }
bool rb_assets_degraded(void) { return degraded; }
rb_asset_stats_t rb_assets_stats(void) { return stats; }

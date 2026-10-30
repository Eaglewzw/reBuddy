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
        le32(h+8)!=PIXEL_BYTES || le32(h+12)>5) return NULL;
    if (le32(h+12)==0 && bytes!=16+PIXEL_BYTES) return NULL;
    if (le32(h+12)==2 && bytes<=20) return NULL;
    if (le32(h+12)==5 && bytes<=20) return NULL;
    if (le32(h+12)==4 && bytes<=20) return NULL;
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
        if (codec==2 || codec==5) {
            has_delta=true;
            unsigned base=le32(record+16);
            const uint8_t *base_record=frame_record(base);
            /* A delta may reference only a complete raw, zlib or palette frame. This
               rules out self-references, cycles and unbounded decode chains. */
            if (!base_record || (codec==5?(base>=i || le32(base_record+12)<4):
                (le32(base_record+12)==2 || le32(base_record+12)>3))) {
                blob=NULL; blob_size=0; degraded=true; return false;
            }
        }
        if(codec==5) {
            unsigned node=i,depth=0;
            const uint8_t *r=record;
            while(le32(r+12)==5) {
                unsigned next=le32(r+16);
                if(++depth>7 || next>=node || !(r=frame_record(next)) || le32(r+12)<4) {
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
    unsigned count=le16(record+16);
    size_t header=20+count*2;
    return count && count<=256 && bytes>header &&
        inflate_pixels(record+header,bytes-header,pixels);
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
        if(codec==4)break;
        if(codec!=5)return false;
        unsigned next=le32(palette+16);
        if(next>=current)return false;
        current=next;palette=frame_record(current);
    }
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
        if(!delta || !inflate_pixels(delta+20,rb_frames[next].bytes-20,dance_planes))return false;
        for(unsigned i=0;i<PIXEL_BYTES;++i)base_pixels[i]^=dance_planes[i];
        cached_base=next;
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

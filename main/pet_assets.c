#include "pet_assets.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "miniz.h"
static tinfl_decompressor *inflater;
#else
#include <zlib.h>
#endif
#define PIXEL_BYTES RB_FRAME_BYTES
static const uint8_t *blob;
static size_t blob_size;
static struct { unsigned index; uint8_t *pixels; lv_image_dsc_t image; } slots[2];
static unsigned slot_count,next_slot;
static uint8_t *base_pixels;
static unsigned cached_base=UINT32_MAX;
static bool degraded;
static lv_image_dsc_t direct,room_image,dance_image;
static uint16_t *dance_pixels;
static uint32_t *dance_queue;
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
        le32(h+8)!=PIXEL_BYTES || le32(h+12)>3) return NULL;
    if (le32(h+12)==0 && bytes!=16+PIXEL_BYTES) return NULL;
    if (le32(h+12)==2 && bytes<=20) return NULL;
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
    lv_image_cache_drop(&dance_image);
    free(dance_pixels);dance_pixels=NULL;free(dance_queue);dance_queue=NULL;
    memset(&dance_image,0,sizeof(dance_image));
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
    bool has_delta=false;
    for (unsigned i=0;i<RB_FRAME_COUNT;++i) {
        const uint8_t *record=frame_record(i);
        if (!record || (i==0 && le32(record+12)!=0)) {
            blob=NULL; blob_size=0; degraded=true; return false;
        }
        if (le32(record+12)==2) {
            has_delta=true;
            unsigned base=le32(record+16);
            const uint8_t *base_record=frame_record(base);
            /* A delta may reference only a complete raw, zlib or palette frame. This
               rules out self-references, cycles and unbounded decode chains. */
            if (!base_record || le32(base_record+12)==2) {
                blob=NULL; blob_size=0; degraded=true; return false;
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
    if (!decode(frame,record,slots[i].pixels)) return fallback(meta);
    slots[i].index=frame; *meta=&rb_frames[frame]; return &slots[i].image;
}
/* Render the source's exterior cream backdrop as the existing room.
   Flood only border-connected colors: eye/gloss highlights inside the dark
   robot outline remain opaque. The original frame/cache/anchor never changes. */
static bool backdrop(uint16_t pixel,uint16_t key)
{
    return abs((int)(pixel>>11)-(int)(key>>11))<=1 &&
           abs((int)((pixel>>5)&63)-(int)((key>>5)&63))<=2 &&
           abs((int)(pixel&31)-(int)(key&31))<=2;
}
static void rgb_channels(uint16_t value,int rgb[3])
{
    rgb[0]=((value>>11)&31)*255/31;
    rgb[1]=((value>>5)&63)*255/63;
    rgb[2]=(value&31)*255/31;
}
/* Remove the old cream matte from a one-pixel antialiased silhouette band.
   Estimate coverage from a neighboring dark outline, then replace only the
   background contribution: C_new = C_old + (1-alpha)*(room-key).
   Interior highlights are never candidates; geometry and source pixels stay
   unchanged. Read neighbors from source so processing order cannot affect it. */
static uint16_t blend_dance_edge(unsigned pos,const uint8_t *source,uint16_t key,uint16_t room,uint16_t marker)
{
    uint16_t pixel=le16(source+2*pos);
    if((pixel>>11)<20 || ((pixel>>5)&63)<45 || (pixel&31)<20) return pixel;
    unsigned x=pos%RB_FRAME_WIDTH,y=pos/RB_FRAME_WIDTH;
    if(!x || !y || x+1==RB_FRAME_WIDTH || y+1==RB_FRAME_HEIGHT) return pixel;
    bool exterior=false;int darkest=766;uint16_t foreground=pixel;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
        if(!dx && !dy) continue;
        unsigned n=(unsigned)((int)pos+dy*RB_FRAME_WIDTH+dx);
        if(dance_pixels[n]==marker) { exterior=true;continue; }
        uint16_t neighbor=le16(source+2*n);int rgb[3];rgb_channels(neighbor,rgb);
        int light=rgb[0]+rgb[1]+rgb[2];
        if(light<darkest) { darkest=light;foreground=neighbor; }
    }
    if(!exterior || darkest>300) return pixel;
    int c[3],f[3],k[3],b[3];rgb_channels(pixel,c);rgb_channels(foreground,f);
    rgb_channels(key,k);rgb_channels(room,b);
    int numerator=0,denominator=0;
    for(unsigned j=0;j<3;++j) { int delta=k[j]-f[j];numerator+=(k[j]-c[j])*delta;denominator+=delta*delta; }
    if(!denominator) return pixel;
    int alpha=numerator*256/denominator;if(alpha<0)alpha=0;if(alpha>256)alpha=256;
    for(unsigned j=0;j<3;++j) { c[j]+=(256-alpha)*(b[j]-k[j])/256;if(c[j]<0)c[j]=0;if(c[j]>255)c[j]=255; }
    return (uint16_t)(((c[0]>>3)<<11)|((c[1]>>2)<<5)|(c[2]>>3));
}
const lv_image_dsc_t *rb_assets_dance_frame(unsigned frame,const rb_frame_meta_t **meta)
{
    if (!meta || frame>=RB_FRAME_COUNT) return NULL;
    const lv_image_dsc_t *source=rb_assets_frame(frame,meta);
    if (!source || !room_image.data || *meta!=&rb_frames[frame]) return NULL;
    if (!dance_pixels) dance_pixels=asset_alloc(PIXEL_BYTES);
    if (!dance_queue) dance_queue=asset_alloc(DANCE_PIXELS*sizeof(*dance_queue));
    if (!dance_pixels || !dance_queue) { degraded=true; return NULL; }
    lv_image_cache_drop(&dance_image);
    memcpy(dance_pixels,source->data,PIXEL_BYTES);
    uint16_t key=dance_pixels[0];
    /* Magenta is a temporary flood marker, absent from the reviewed cream,
       lime and charcoal palette. Refuse it instead of damaging source pixels. */
    const uint16_t marker=0xf81f;
    for (unsigned i=0;i<DANCE_PIXELS;++i) if(dance_pixels[i]==marker) return NULL;
    unsigned head=0,tail=0;
    for(unsigned y=0;y<RB_FRAME_HEIGHT;++y) for(unsigned x=0;x<RB_FRAME_WIDTH;++x) {
        if (x && y && x+1<RB_FRAME_WIDTH && y+1<RB_FRAME_HEIGHT) continue;
        unsigned pos=y*RB_FRAME_WIDTH+x;
        if(backdrop(dance_pixels[pos],key)) { dance_pixels[pos]=marker;dance_queue[tail++]=pos; }
    }
    while(head<tail) {
        unsigned pos=dance_queue[head++],x=pos%RB_FRAME_WIDTH,y=pos/RB_FRAME_WIDTH;
        unsigned neighbors[4]={x?pos-1:pos,x+1<RB_FRAME_WIDTH?pos+1:pos,
            y?pos-RB_FRAME_WIDTH:pos,y+1<RB_FRAME_HEIGHT?pos+RB_FRAME_WIDTH:pos};
        for(unsigned k=0;k<4;++k) {
            unsigned next=neighbors[k];
            if(backdrop(dance_pixels[next],key)) { dance_pixels[next]=marker;dance_queue[tail++]=next; }
        }
    }
    const uint8_t *room=room_image.data;
    /* Keep flood markers until every edge has been evaluated. */
    for(unsigned i=0;i<DANCE_PIXELS;++i) if(dance_pixels[i]!=marker)
        dance_pixels[i]=blend_dance_edge(i,source->data,key,le16(room+i*2),marker);
    for(unsigned i=0;i<DANCE_PIXELS;++i) if(dance_pixels[i]==marker) dance_pixels[i]=le16(room+i*2);
    dance_image=descriptor((const uint8_t *)dance_pixels);
    return &dance_image;
}
size_t rb_assets_cache_bytes(void)
{ return (slot_count+(base_pixels?1u:0u)+(dance_pixels?1u:0u))*PIXEL_BYTES+
    (dance_queue?DANCE_PIXELS*sizeof(*dance_queue):0); }
bool rb_assets_degraded(void) { return degraded; }

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
static lv_image_dsc_t direct,room_image;
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
        le32(h+8)!=PIXEL_BYTES || le32(h+12)>2) return NULL;
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
            /* A delta may reference only a complete raw or zlib frame. This
               rules out self-references, cycles and unbounded decode chains. */
            if (!base_record || le32(base_record+12)>1) {
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
static bool inflate_pixels(const uint8_t *source,size_t length,uint8_t *pixels)
{
#ifdef ESP_PLATFORM
    if (!inflater) return false;
    tinfl_init(inflater);
    size_t input=length,output=PIXEL_BYTES;
    tinfl_status status=tinfl_decompress(inflater,source,&input,pixels,pixels,&output,
        TINFL_FLAG_PARSE_ZLIB_HEADER|TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    return status==TINFL_STATUS_DONE && input==length && output==PIXEL_BYTES;
#else
    uLongf output=PIXEL_BYTES;
    return uncompress(pixels,&output,source,length)==Z_OK && output==PIXEL_BYTES;
#endif
}
static bool decode_complete(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    unsigned codec=le32(record+12);
    if (codec==0) { memcpy(pixels,record+16,PIXEL_BYTES); return true; }
    return codec==1 && inflate_pixels(record+16,rb_frames[frame].bytes-16,pixels);
}
static bool decode(unsigned frame,const uint8_t *record,uint8_t *pixels)
{
    if (le32(record+12)!=2) return decode_complete(frame,record,pixels);
    if (!base_pixels) return false;
    unsigned base=le32(record+16);
    const uint8_t *base_record=frame_record(base);
    if (!base_record || le32(base_record+12)>1) return false;
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
size_t rb_assets_cache_bytes(void)
{ return (slot_count+(base_pixels?1u:0u))*PIXEL_BYTES; }
bool rb_assets_degraded(void) { return degraded; }

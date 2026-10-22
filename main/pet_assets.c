#include "pet_assets.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
#define PIXEL_BYTES (160u*160u*2u)
static const uint8_t *blob;
static size_t blob_size;
static struct { unsigned index; uint8_t *pixels; lv_image_dsc_t image; } slots[2];
static unsigned slot_count,next_slot;
static bool degraded;
static lv_image_dsc_t direct;
static lv_image_dsc_t room_image;
static uint16_t le16(const uint8_t *p) { return (uint16_t)p[0]|((uint16_t)p[1]<<8); }
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
static const uint8_t *frame_data(unsigned index)
{
    if (!blob || index>=RB_FRAME_COUNT) return NULL;
    size_t offset=rb_frames[index].offset;
    if (offset>blob_size || blob_size-offset<RB_FRAME_BYTES) return NULL;
    const uint8_t *h=blob+offset;
    if (h[0]!=LV_IMAGE_HEADER_MAGIC || h[1]!=LV_COLOR_FORMAT_RGB565 || le16(h+2)!=0 ||
        le16(h+4)!=160 || le16(h+6)!=160 || le16(h+8)!=320) return NULL;
    return h+12;
}
static lv_image_dsc_t descriptor(const uint8_t *pixels)
{
    return (lv_image_dsc_t){.header={.magic=LV_IMAGE_HEADER_MAGIC,.cf=LV_COLOR_FORMAT_RGB565,
        .w=160,.h=160,.stride=320},.data_size=PIXEL_BYTES,.data=pixels};
}
void rb_assets_close(void)
{
    for (unsigned i=0;i<slot_count;++i) { lv_image_cache_drop(&slots[i].image); free(slots[i].pixels); }
    slot_count=next_slot=0; blob=NULL; blob_size=0;
}
bool rb_assets_init(const uint8_t *data,size_t bytes,unsigned cache_slots)
{
    rb_assets_close(); blob=data; blob_size=bytes; degraded=false;
    if (!frame_data(0)) { degraded=true; return false; }
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
    if (slot_count<2) degraded=true;
    return true;
}
const lv_image_dsc_t *rb_assets_frame(unsigned frame,const rb_frame_meta_t **meta)
{
    const uint8_t *pixels=frame_data(frame);
    if (!pixels || slot_count<2) { degraded=true; frame=0; pixels=frame_data(0); }
    if (!pixels) return NULL;
    *meta=&rb_frames[frame];
    if (!slot_count) { direct=descriptor(pixels); return &direct; }
    for (unsigned i=0;i<slot_count;++i) if (slots[i].index==frame) return &slots[i].image;
    unsigned i=next_slot++%slot_count;
    lv_image_cache_drop(&slots[i].image);
    memcpy(slots[i].pixels,pixels,PIXEL_BYTES); slots[i].index=frame;
    return &slots[i].image;
}
size_t rb_assets_cache_bytes(void) { return slot_count*PIXEL_BYTES; }
bool rb_assets_degraded(void) { return degraded; }

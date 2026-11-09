#include "choreo_store.h"

#include <stdbool.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "nvs.h"
#include "nvs_flash.h"
#endif

/* Stable wire envelope: magic:u32, version:u16, header length:u16,
 * generation:u32, payload length:u32, CRC32:u32. All integers little-endian.
 * CRC includes bytes 0..15 and the entire payload, excluding its own field. */
#define RECORD_MAGIC UINT32_C(0x48434252)
#define RECORD_VERSION 1u
#define HEADER_SIZE 20u
#define MAX_RECORD_SIZE 512u
/* Keep the original s0r0..s2r1 keys and extend through s9r1. Both the
 * envelope and payload remain version 1; existing works need no migration. */
_Static_assert(RB_CHOREO_STORE_SLOTS<=10u,"Single-digit NVS slot keys support at most ten works");

static rb_choreo_store_backend_t storage;
static bool injected;
static bool ready;

static uint16_t get16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8); }
static uint32_t get32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void put16(uint8_t *p, uint16_t value)
{ p[0]=(uint8_t)value; p[1]=(uint8_t)(value>>8); }
static void put32(uint8_t *p, uint32_t value)
{ for (unsigned i=0;i<4;++i) p[i]=(uint8_t)(value>>(8*i)); }

static uint32_t crc_update(uint32_t crc, const uint8_t *data, size_t length)
{
    for (size_t i=0;i<length;++i) {
        crc^=data[i];
        for (unsigned bit=0;bit<8;++bit)
            crc=(crc>>1)^(UINT32_C(0xedb88320)&(0u-(crc&1u)));
    }
    return crc;
}
static uint32_t record_crc(const uint8_t *data, size_t length)
{
    uint32_t crc=crc_update(UINT32_MAX,data,16);
    return ~crc_update(crc,data+HEADER_SIZE,length-HEADER_SIZE);
}

#ifdef ESP_PLATFORM
static void key_name(unsigned key, char name[5])
{
    name[0]='s'; name[1]=(char)('0'+key/2);
    name[2]='r'; name[3]=(char)('0'+key%2); name[4]='\0';
}
static rb_choreo_store_result_t nvs_read_record(void *context, unsigned key,
                                              uint8_t *data, size_t capacity, size_t *length)
{
    (void)context;
    if (key>=RB_CHOREO_STORE_SLOTS*2u) return RB_CHOREO_STORE_INVALID;
    nvs_handle_t handle;
    esp_err_t error=nvs_open("rb_choreo",NVS_READONLY,&handle);
    if (error==ESP_ERR_NVS_NOT_FOUND) return RB_CHOREO_STORE_EMPTY;
    if (error!=ESP_OK) return RB_CHOREO_STORE_IO;
    char name[5]; key_name(key,name);
    size_t size=0;
    error=nvs_get_blob(handle,name,NULL,&size);
    if (error==ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle); return RB_CHOREO_STORE_EMPTY;
    }
    if (error!=ESP_OK) {
        nvs_close(handle);
        return error==ESP_ERR_NVS_TYPE_MISMATCH?RB_CHOREO_STORE_CORRUPT:RB_CHOREO_STORE_IO;
    }
    if (size>capacity) {
        nvs_close(handle); return RB_CHOREO_STORE_CORRUPT;
    }
    error=nvs_get_blob(handle,name,data,&size);
    nvs_close(handle);
    if (error!=ESP_OK) return RB_CHOREO_STORE_IO;
    *length=size;
    return RB_CHOREO_STORE_OK;
}
static rb_choreo_store_result_t nvs_write_record(void *context, unsigned key,
                                               const uint8_t *data, size_t length)
{
    (void)context;
    if (key>=RB_CHOREO_STORE_SLOTS*2u) return RB_CHOREO_STORE_INVALID;
    nvs_handle_t handle;
    if (nvs_open("rb_choreo",NVS_READWRITE,&handle)!=ESP_OK) return RB_CHOREO_STORE_IO;
    char name[5]; key_name(key,name);
    esp_err_t error=nvs_set_blob(handle,name,data,length);
    if (error==ESP_OK) error=nvs_commit(handle);
    nvs_close(handle);
    return error==ESP_OK?RB_CHOREO_STORE_OK:RB_CHOREO_STORE_IO;
}
#endif

rb_choreo_store_result_t rb_choreo_store_set_backend(const rb_choreo_store_backend_t *backend)
{
    if (backend && (!backend->read || !backend->write)) return RB_CHOREO_STORE_INVALID;
    memset(&storage,0,sizeof(storage));
    injected=backend!=NULL;
    if (backend) storage=*backend;
    ready=false;
    return RB_CHOREO_STORE_OK;
}

rb_choreo_store_result_t rb_choreo_store_init(void)
{
    ready=false;
    if (!injected) {
#ifdef ESP_PLATFORM
        /* In particular, NO_FREE_PAGES and NEW_VERSION_FOUND are reported;
         * never erase NVS, which can also contain unrelated application data. */
        if (nvs_flash_init()!=ESP_OK) return RB_CHOREO_STORE_IO;
        storage=(rb_choreo_store_backend_t){NULL,nvs_read_record,nvs_write_record};
#else
        return RB_CHOREO_STORE_IO;
#endif
    }
    ready=true;
    return RB_CHOREO_STORE_OK;
}

static rb_choreo_store_result_t decode_payload(const uint8_t *data, size_t length,
                                               rb_choreo_work_t *work);
static size_t encode_payload(uint8_t *data, const rb_choreo_work_t *work);

/* Payload v1: schema:u16, work id:u8, name[32] (zero padded), tempo:u8,
 * count:u8, then count records of action:u8/repeat:u8/hold:u16/wait:u16.
 * No compiler layout, padding, enum representation or pointers are stored. */
#define PAYLOAD_BASE_SIZE (2u+1u+RB_CHOREO_NAME_BYTES+1u+1u)
#define STEP_WIRE_SIZE 6u
_Static_assert(HEADER_SIZE+PAYLOAD_BASE_SIZE+RB_CHOREO_MAX_STEPS*STEP_WIRE_SIZE<=MAX_RECORD_SIZE,
               "Storage record buffer is too small for the work schema");
_Static_assert(RB_CHOREO_SLOTS==RB_CHOREO_STORE_SLOTS,"Storage slot count mismatch");

static size_t encode_payload(uint8_t *data, const rb_choreo_work_t *work)
{
    if (rb_choreo_validate(work,NULL)!=RB_CHOREO_OK) return 0;
    put16(data,work->schema_version);
    data[2]=work->work_id;
    memset(data+3,0,RB_CHOREO_NAME_BYTES);
    size_t name_length=0;
    while (name_length<RB_CHOREO_NAME_BYTES && work->name[name_length]) ++name_length;
    if (name_length==RB_CHOREO_NAME_BYTES) return 0;
    memcpy(data+3,work->name,name_length);
    data[3+RB_CHOREO_NAME_BYTES]=work->tempo;
    data[4+RB_CHOREO_NAME_BYTES]=work->count;
    for (unsigned i=0;i<work->count;++i) {
        uint8_t *step=data+PAYLOAD_BASE_SIZE+i*STEP_WIRE_SIZE;
        step[0]=work->steps[i].action_id; step[1]=work->steps[i].repeat;
        put16(step+2,work->steps[i].hold_ms); put16(step+4,work->steps[i].duration_ms);
    }
    return PAYLOAD_BASE_SIZE+work->count*STEP_WIRE_SIZE;
}

static rb_choreo_store_result_t decode_payload(const uint8_t *data, size_t length,
                                               rb_choreo_work_t *work)
{
    if (length<PAYLOAD_BASE_SIZE) return RB_CHOREO_STORE_CORRUPT;
    if (get16(data)!=RB_CHOREO_SCHEMA_VERSION) return RB_CHOREO_STORE_VERSION;
    unsigned count=data[4+RB_CHOREO_NAME_BYTES];
    if (count>RB_CHOREO_MAX_STEPS || length!=PAYLOAD_BASE_SIZE+count*STEP_WIRE_SIZE ||
        !memchr(data+3,0,RB_CHOREO_NAME_BYTES)) return RB_CHOREO_STORE_CORRUPT;
    rb_choreo_work_t decoded={0};
    decoded.schema_version=get16(data); decoded.work_id=data[2];
    memcpy(decoded.name,data+3,RB_CHOREO_NAME_BYTES);
    decoded.tempo=data[3+RB_CHOREO_NAME_BYTES]; decoded.count=(uint8_t)count;
    for (unsigned i=0;i<count;++i) {
        const uint8_t *step=data+PAYLOAD_BASE_SIZE+i*STEP_WIRE_SIZE;
        decoded.steps[i].action_id=step[0]; decoded.steps[i].repeat=step[1];
        decoded.steps[i].hold_ms=get16(step+2); decoded.steps[i].duration_ms=get16(step+4);
    }
    if (rb_choreo_validate(&decoded,NULL)!=RB_CHOREO_OK) return RB_CHOREO_STORE_CORRUPT;
    *work=decoded;
    return RB_CHOREO_STORE_OK;
}

typedef struct {
    rb_choreo_store_result_t result;
    uint32_t generation;
    rb_choreo_work_t work;
} record_t;

static record_t read_record(unsigned key)
{
    record_t record={0};
    uint8_t data[MAX_RECORD_SIZE];
    size_t length=0;
    record.result=storage.read(storage.context,key,data,sizeof(data),&length);
    if (record.result!=RB_CHOREO_STORE_OK) return record;
    record.result=RB_CHOREO_STORE_CORRUPT;
    if (length<HEADER_SIZE || length>sizeof(data) || get32(data)!=RECORD_MAGIC ||
        get16(data+6)!=HEADER_SIZE || get32(data+12)!=length-HEADER_SIZE ||
        get32(data+16)!=record_crc(data,length)) return record;
    record.generation=get32(data+8);
    if (get16(data+4)!=RECORD_VERSION) {
        record.result=RB_CHOREO_STORE_VERSION;
        return record;
    }
    record.result=decode_payload(data+HEADER_SIZE,length-HEADER_SIZE,&record.work);
    if (record.result==RB_CHOREO_STORE_OK && record.work.work_id!=key/2+1)
        record.result=RB_CHOREO_STORE_CORRUPT;
    return record;
}

/* Half-range sequence comparison also handles UINT32_MAX -> 0 rollover. */
static bool newer(uint32_t a, uint32_t b)
{ return a!=b && (uint32_t)(a-b)<UINT32_C(0x80000000); }

static int newest_valid(const record_t records[2])
{
    bool a=records[0].result==RB_CHOREO_STORE_OK;
    bool b=records[1].result==RB_CHOREO_STORE_OK;
    if (a && b) return newer(records[1].generation,records[0].generation)?1:0;
    return a?0:b?1:-1;
}
static rb_choreo_store_result_t absent_result(const record_t records[2])
{
    if (records[0].result==RB_CHOREO_STORE_IO || records[1].result==RB_CHOREO_STORE_IO)
        return RB_CHOREO_STORE_IO;
    if (records[0].result==RB_CHOREO_STORE_VERSION || records[1].result==RB_CHOREO_STORE_VERSION)
        return RB_CHOREO_STORE_VERSION;
    if (records[0].result==RB_CHOREO_STORE_EMPTY && records[1].result==RB_CHOREO_STORE_EMPTY)
        return RB_CHOREO_STORE_EMPTY;
    return RB_CHOREO_STORE_CORRUPT;
}

rb_choreo_store_result_t rb_choreo_store_load(unsigned slot, rb_choreo_work_t *work)
{
    if (slot>=RB_CHOREO_STORE_SLOTS || !work) return RB_CHOREO_STORE_INVALID;
    if (!ready) return RB_CHOREO_STORE_IO;
    record_t records[2]={read_record(slot*2),read_record(slot*2+1)};
    /* Do not silently downgrade or substitute an incompatible saved work. */
    if (records[0].result==RB_CHOREO_STORE_VERSION || records[1].result==RB_CHOREO_STORE_VERSION)
        return RB_CHOREO_STORE_VERSION;
    int index=newest_valid(records);
    if (index<0) return absent_result(records);
    *work=records[index].work;
    return RB_CHOREO_STORE_OK;
}

rb_choreo_store_result_t rb_choreo_store_save(unsigned slot, const rb_choreo_work_t *work)
{
    if (slot>=RB_CHOREO_STORE_SLOTS || !work || work->work_id!=slot+1)
        return RB_CHOREO_STORE_INVALID;
    if (!ready) return RB_CHOREO_STORE_IO;
    uint8_t data[MAX_RECORD_SIZE]={0};
    size_t payload_length=encode_payload(data+HEADER_SIZE,work);
    if (!payload_length) return RB_CHOREO_STORE_INVALID;
    record_t records[2]={read_record(slot*2),read_record(slot*2+1)};
    rb_choreo_store_result_t state=absent_result(records);
    if (state==RB_CHOREO_STORE_IO || state==RB_CHOREO_STORE_VERSION) return state;
    int latest=newest_valid(records);
    unsigned target=latest<0?0u:(unsigned)(1-latest);
    uint32_t generation=latest<0?1u:records[latest].generation+1u;
    put32(data,RECORD_MAGIC); put16(data+4,RECORD_VERSION); put16(data+6,HEADER_SIZE);
    put32(data+8,generation); put32(data+12,(uint32_t)payload_length);
    put32(data+16,record_crc(data,HEADER_SIZE+payload_length));
    unsigned key=slot*2+target;
    if (storage.write(storage.context,key,data,HEADER_SIZE+payload_length)!=RB_CHOREO_STORE_OK)
        return RB_CHOREO_STORE_IO;
    /* Detect a backend reporting success for a truncated or wrong write. */
    uint8_t check[MAX_RECORD_SIZE];
    size_t check_length=0;
    if (storage.read(storage.context,key,check,sizeof(check),&check_length)!=RB_CHOREO_STORE_OK ||
        check_length!=HEADER_SIZE+payload_length || memcmp(data,check,check_length)!=0)
        return RB_CHOREO_STORE_IO;
    return RB_CHOREO_STORE_OK;
}

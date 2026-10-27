#include "choreo_store.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t records[RB_CHOREO_STORE_SLOTS*2][512];
    size_t lengths[RB_CHOREO_STORE_SLOTS*2];
    bool fail_write;
    bool tear_write;
    bool acknowledge_torn_write;
    int fail_read_key;
    unsigned writes;
} memory_store_t;

static rb_choreo_store_result_t read_record(void *context, unsigned key,
                                          uint8_t *data, size_t capacity, size_t *length)
{
    memory_store_t *memory=context;
    assert(key<RB_CHOREO_STORE_SLOTS*2);
    if ((int)key==memory->fail_read_key) return RB_CHOREO_STORE_IO;
    if (!memory->lengths[key]) return RB_CHOREO_STORE_EMPTY;
    if (memory->lengths[key]>capacity) return RB_CHOREO_STORE_CORRUPT;
    memcpy(data,memory->records[key],memory->lengths[key]);
    *length=memory->lengths[key];
    return RB_CHOREO_STORE_OK;
}
static rb_choreo_store_result_t write_record(void *context, unsigned key,
                                           const uint8_t *data, size_t length)
{
    memory_store_t *memory=context;
    assert(key<RB_CHOREO_STORE_SLOTS*2 && length<=sizeof(memory->records[key]));
    ++memory->writes;
    if (memory->tear_write) {
        memory->lengths[key]=length/2;
        memcpy(memory->records[key],data,length/2);
        return memory->acknowledge_torn_write?RB_CHOREO_STORE_OK:RB_CHOREO_STORE_IO;
    }
    if (memory->fail_write) return RB_CHOREO_STORE_IO;
    memory->lengths[key]=length;
    memcpy(memory->records[key],data,length);
    return RB_CHOREO_STORE_OK;
}
static void restart(memory_store_t *memory)
{
    rb_choreo_store_backend_t backend={memory,read_record,write_record};
    assert(rb_choreo_store_set_backend(&backend)==RB_CHOREO_STORE_OK);
    assert(rb_choreo_store_init()==RB_CHOREO_STORE_OK);
}
static uint32_t crc_bytes(uint32_t crc, const uint8_t *data, size_t size)
{
    for (size_t i=0;i<size;++i) {
        crc^=data[i];
        for (unsigned bit=0;bit<8;++bit)
            crc=(crc>>1)^(UINT32_C(0xedb88320)&(0u-(crc&1u)));
    }
    return crc;
}
static void reseal(memory_store_t *memory, unsigned key)
{
    uint8_t *bytes=memory->records[key];
    uint32_t crc=crc_bytes(UINT32_MAX,bytes,16);
    crc=~crc_bytes(crc,bytes+20,memory->lengths[key]-20);
    for (unsigned i=0;i<4;++i) bytes[16+i]=(uint8_t)(crc>>(8*i));
}

static rb_choreo_work_t make_work(unsigned id, unsigned wait_ms)
{
    rb_choreo_work_t work={0};
    work.schema_version=RB_CHOREO_SCHEMA_VERSION;
    work.work_id=(uint8_t)id;
    memcpy(work.name,"舞蹈",sizeof("舞蹈"));
    work.tempo=1;
    work.count=1;
    work.steps[0].action_id=15; /* stable WAIT wire id */
    work.steps[0].repeat=1;
    work.steps[0].duration_ms=(uint16_t)wait_ms;
    return work;
}
static void equal_work(const rb_choreo_work_t *a, const rb_choreo_work_t *b)
{
    assert(a->schema_version==b->schema_version && a->work_id==b->work_id);
    assert(!strcmp(a->name,b->name) && a->tempo==b->tempo && a->count==b->count);
    for (unsigned i=0;i<a->count;++i) {
        assert(a->steps[i].action_id==b->steps[i].action_id);
        assert(a->steps[i].repeat==b->steps[i].repeat);
        assert(a->steps[i].hold_ms==b->steps[i].hold_ms);
        assert(a->steps[i].duration_ms==b->steps[i].duration_ms);
    }
}
static void load_equal(unsigned slot, const rb_choreo_work_t *expected)
{
    rb_choreo_work_t loaded={0};
    assert(rb_choreo_store_load(slot,&loaded)==RB_CHOREO_STORE_OK);
    equal_work(&loaded,expected);
}
static void set_generation(memory_store_t *memory, unsigned key, uint32_t generation)
{
    for (unsigned i=0;i<4;++i) memory->records[key][8+i]=(uint8_t)(generation>>(8*i));
    reseal(memory,key);
}

static void legacy_wire_compatibility(void)
{
    /* Fixed version-1 bytes, independent of the current encoder: work 1,
       standard tempo, a 500ms WAIT. The first three slots used this format. */
    static const uint8_t legacy_record[]={
        0x52,0x42,0x43,0x48,0x01,0x00,0x14,0x00,0x01,0x00,0x00,0x00,
        0x2b,0x00,0x00,0x00,0x3d,0x8d,0x59,0xfd,0x01,0x00,0x01,0xe8,
        0x88,0x9e,0xe8,0xb9,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x0f,0x01,0x00,
        0x00,0xf4,0x01
    };
    memory_store_t memory={.fail_read_key=-1};
    for (unsigned slot=0;slot<3;++slot) {
        unsigned key=slot*2;
        memcpy(memory.records[key],legacy_record,sizeof(legacy_record));
        memory.lengths[key]=sizeof(legacy_record);
        if (slot) { memory.records[key][22]=(uint8_t)(slot+1); reseal(&memory,key); }
    }
    restart(&memory);
    for (unsigned slot=0;slot<3;++slot) {
        rb_choreo_work_t expected=make_work(slot+1,500); load_equal(slot,&expected);
    }
    for (unsigned slot=3;slot<RB_CHOREO_STORE_SLOTS;++slot) {
        rb_choreo_work_t loaded;
        assert(rb_choreo_store_load(slot,&loaded)==RB_CHOREO_STORE_EMPTY);
    }
    assert(memory.writes==0); /* init/load perform no migration writes */
    assert(rb_choreo_store_set_backend(NULL)==RB_CHOREO_STORE_OK);
}

int main(void)
{
    legacy_wire_compatibility();
    memory_store_t memory={.fail_read_key=-1};
    rb_choreo_work_t first=make_work(1,500),second=make_work(1,1000),third=make_work(1,1500);
    rb_choreo_work_t loaded=third;
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_IO);
    restart(&memory);
    for (unsigned slot=0;slot<3;++slot) {
        assert(rb_choreo_store_load(slot,&loaded)==RB_CHOREO_STORE_EMPTY);
        equal_work(&loaded,&third); /* failures never partly overwrite output */
        rb_choreo_work_t work=make_work(slot+1,500);
        assert(rb_choreo_store_save(slot,&work)==RB_CHOREO_STORE_OK);
        load_equal(slot,&work);
    }
    assert(memory.writes==3);
    /* Capacity expansion must preserve the original three works byte-for-byte,
       with the same schema and key indices, while allocating seven new slots. */
    memory_store_t legacy_three=memory;
    restart(&memory);
    for (unsigned slot=3;slot<RB_CHOREO_STORE_SLOTS;++slot) {
        assert(rb_choreo_store_load(slot,&loaded)==RB_CHOREO_STORE_EMPTY);
        rb_choreo_work_t work;
        rb_choreo_work_init(&work,slot);
        assert(work.work_id==slot+1 && work.schema_version==1);
        work=make_work(slot+1,500);
        assert(rb_choreo_store_save(slot,&work)==RB_CHOREO_STORE_OK);
        restart(&memory); load_equal(slot,&work);
    }
    assert(RB_CHOREO_STORE_SLOTS==10 && RB_CHOREO_MAX_STEPS==8);
    assert(memory.writes==RB_CHOREO_STORE_SLOTS);
    assert(!memcmp(memory.records,legacy_three.records,sizeof(memory.records[0])*6));
    assert(!memcmp(memory.lengths,legacy_three.lengths,sizeof(memory.lengths[0])*6));
    for (unsigned slot=0;slot<3;++slot) {
        rb_choreo_work_t work=make_work(slot+1,500);
        load_equal(slot,&work);
    }
    assert(rb_choreo_store_save(0,&second)==RB_CHOREO_STORE_OK);
    restart(&memory); load_equal(0,&second);
    memory_store_t two_good=memory;

    memory.fail_write=true;
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_IO);
    restart(&memory); load_equal(0,&second);
    memory=two_good; memory.tear_write=true; restart(&memory);
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_IO);
    restart(&memory); load_equal(0,&second);
    memory=two_good; memory.tear_write=true; memory.acknowledge_torn_write=true; restart(&memory);
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_IO);
    restart(&memory); load_equal(0,&second);

    memory=two_good; memory.records[1][25]^=0x20; restart(&memory);
    load_equal(0,&first); /* damaged newest generation recovers earlier work */
    memory.records[0][25]^=0x20;
    loaded=third;
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    equal_work(&loaded,&third);
    memory=two_good; memory.lengths[1]=10; restart(&memory); load_equal(0,&first);
    memory=two_good; memory.lengths[1]=513; restart(&memory); load_equal(0,&first);

    memory=two_good; memory.fail_read_key=1; restart(&memory);
    load_equal(0,&first);
    unsigned writes=memory.writes;
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_IO);
    assert(memory.writes==writes);

    memory=two_good; memory.records[1][4]=2; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_VERSION);
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_VERSION);
    memory=two_good; memory.records[1][20]=2; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_VERSION);

    /* Valid checksum cannot bypass semantic validation. Only one record exists. */
    memory=two_good; memory.lengths[0]=0; memory.records[1][57]=254; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    for (unsigned id=9;id<=22;++id) {
        if (id==RB_CHOREO_WAIT) continue;
        memory=two_good; memory.lengths[0]=0; memory.records[1][57]=(uint8_t)id;
        reseal(&memory,1); restart(&memory);
        assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    }
    memory=two_good; memory.lengths[0]=0; memset(memory.records[1]+23,'x',32); reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    memory=two_good; memory.lengths[0]=0; memory.records[1][56]=9; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    memory=two_good; memory.lengths[0]=0; memory.records[1][22]=2; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);
    memory=two_good; memory.lengths[0]=0; memory.records[1][12]++; reseal(&memory,1); restart(&memory);
    assert(rb_choreo_store_load(0,&loaded)==RB_CHOREO_STORE_CORRUPT);

    memory=two_good; set_generation(&memory,0,UINT32_MAX); set_generation(&memory,1,0); restart(&memory);
    load_equal(0,&second);
    assert(rb_choreo_store_save(0,&third)==RB_CHOREO_STORE_OK);
    assert(memory.records[0][8]==1); restart(&memory); load_equal(0,&third);

    memory=two_good; restart(&memory); writes=memory.writes;
    rb_choreo_work_t invalid=first;
    invalid.steps[0].action_id=254;
    assert(rb_choreo_store_save(0,&invalid)==RB_CHOREO_STORE_INVALID);
    invalid=first; invalid.count=9;
    assert(rb_choreo_store_save(0,&invalid)==RB_CHOREO_STORE_INVALID);
    invalid=first; invalid.work_id=2;
    assert(rb_choreo_store_save(0,&invalid)==RB_CHOREO_STORE_INVALID);
    assert(rb_choreo_store_load(RB_CHOREO_STORE_SLOTS,&loaded)==RB_CHOREO_STORE_INVALID);
    assert(rb_choreo_store_save(RB_CHOREO_STORE_SLOTS,&first)==RB_CHOREO_STORE_INVALID);
    assert(rb_choreo_store_save(0,NULL)==RB_CHOREO_STORE_INVALID);
    assert(rb_choreo_store_load(0,NULL)==RB_CHOREO_STORE_INVALID);
    assert(memory.writes==writes);

    rb_choreo_work_t empty=first; empty.count=0;
    assert(rb_choreo_store_save(0,&empty)==RB_CHOREO_STORE_OK);
    restart(&memory); load_equal(0,&empty);
    rb_choreo_work_t maximum=first; maximum.count=RB_CHOREO_MAX_STEPS;
    for (unsigned i=1;i<maximum.count;++i) maximum.steps[i]=maximum.steps[0];
    assert(rb_choreo_store_save(0,&maximum)==RB_CHOREO_STORE_OK);
    restart(&memory); load_equal(0,&maximum);
    assert(memory.lengths[1]==20+37+8*6); /* explicit layout, no native padding */

    /* Unused native-struct bytes never leak to the persistent payload. */
    rb_choreo_work_t same=maximum;
    memset(same.name+strlen(same.name)+1,0xa5,sizeof(same.name)-strlen(same.name)-1);
    assert(rb_choreo_store_save(0,&same)==RB_CHOREO_STORE_OK);
    assert(memory.lengths[0]==memory.lengths[1]);
    assert(!memcmp(memory.records[0]+20,memory.records[1]+20,memory.lengths[0]-20));

    /* No implicit writes on init or read, and no erased unrelated slots. */
    writes=memory.writes; restart(&memory); load_equal(0,&maximum);
    assert(memory.writes==writes);
    for (unsigned slot=1;slot<RB_CHOREO_STORE_SLOTS;++slot) {
        rb_choreo_work_t other=make_work(slot+1,500); load_equal(slot,&other);
    }
    /* Both records of the tenth slot are usable, isolated and recoverable. */
    rb_choreo_work_t tenth=make_work(10,1000);
    assert(rb_choreo_store_save(9,&tenth)==RB_CHOREO_STORE_OK);
    assert(memory.lengths[18] && memory.lengths[19]);
    restart(&memory); load_equal(9,&tenth);
    memory.records[19][25]^=0x20;
    tenth.steps[0].duration_ms=500;
    restart(&memory); load_equal(9,&tenth);
    load_equal(0,&maximum);
    /* New dance IDs retain the v1 wire layout and coexist with the old work.
       Reload every new action after a simulated restart, then compile it. */
    rb_choreo_work_t dance;
    rb_choreo_work_demo(&dance,8);
    assert(dance.schema_version==1 && dance.count==6);
    assert(rb_choreo_store_save(8,&dance)==RB_CHOREO_STORE_OK);
    restart(&memory); load_equal(8,&dance); load_equal(0,&maximum);
    rb_choreo_plan_t plan;
    assert(rb_choreo_store_load(8,&loaded)==RB_CHOREO_STORE_OK);
    assert(rb_choreo_compile(&loaded,&plan,NULL)==RB_CHOREO_OK && plan.total_ms==30000);
    for (unsigned i=0;i<dance.count;++i) assert(loaded.steps[i].action_id==dance.steps[i].action_id);
    /* Each of the ten new stable IDs survives persistence independently. */
    for (unsigned id=RB_CHOREO_GROOVE_BOUNCE;id<=RB_CHOREO_DIME_STOP;++id) {
        rb_choreo_work_init(&dance,8);
        dance.count=1;
        dance.steps[0]=(rb_choreo_step_t){(uint8_t)id,3,500,0};
        assert(rb_choreo_store_save(8,&dance)==RB_CHOREO_STORE_OK);
        restart(&memory); load_equal(8,&dance);
        assert(rb_choreo_store_load(8,&loaded)==RB_CHOREO_STORE_OK);
        assert(rb_choreo_compile(&loaded,&plan,NULL)==RB_CHOREO_OK);
        assert(plan.total_ms==rb_choreo_action(id)->duration_ms*3u+500u);
    }
    for (unsigned id=9;id<=22;++id) {
        if (id==RB_CHOREO_WAIT) continue;
        dance.steps[0].action_id=(uint8_t)id; writes=memory.writes;
        assert(rb_choreo_store_save(8,&dance)==RB_CHOREO_STORE_INVALID && memory.writes==writes);
    }
    rb_choreo_store_backend_t incomplete={0};
    assert(rb_choreo_store_set_backend(&incomplete)==RB_CHOREO_STORE_INVALID);
    assert(rb_choreo_store_set_backend(NULL)==RB_CHOREO_STORE_OK);
    assert(rb_choreo_store_init()==RB_CHOREO_STORE_IO); /* host has no accidental persistence */
    puts("choreo_store: ten slots, legacy three preserved, explicit wire format, restart, CRC, rollback, versions and write failures passed");
    return 0;
}

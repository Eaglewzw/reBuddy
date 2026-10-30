#include "rb_choreo.h"
#include "pet_assets.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static uint8_t *load(const char *path,size_t *bytes)
{
    FILE *f=fopen(path,"rb");assert(f);
    assert(!fseek(f,0,SEEK_END));long size=ftell(f);assert(size>0);rewind(f);
    uint8_t *data=malloc((size_t)size);assert(data);
    assert(fread(data,1,(size_t)size,f)==(size_t)size);fclose(f);*bytes=(size_t)size;return data;
}
int main(int argc,char **argv)
{
    assert(argc==2);lv_init();
    size_t bytes,room_bytes;uint8_t *blob=load(RB_ASSET_FILE,&bytes),*room=load(RB_ROOM_FILE,&room_bytes);
    assert(rb_assets_init(blob,bytes,2) && rb_assets_set_room(room,room_bytes));
    rb_choreo_work_t work;rb_choreo_work_init(&work,0);
    const unsigned actions[]={RB_CHOREO_2D_NOD,RB_CHOREO_2D_SWAY,RB_CHOREO_2D_NOD,RB_CHOREO_2D_WAVE};
    for(unsigned i=0;i<4;++i) {
        rb_choreo_step_t step={(uint8_t)actions[i],1,0,0};
        assert(rb_choreo_add_after(&work,-1,&step)==RB_CHOREO_OK);
    }
    rb_choreo_plan_t plan;assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_OK);
    FILE *f=fopen(argv[1],"wb");assert(f);
    for(uint32_t t=0;t<plan.total_ms;t+=50) {
        rb_choreo_sample_t sample;assert(rb_choreo_sample(&plan,t,&sample));
        const rb_clip_meta_t *clip=&rb_clips[rb_choreo_clip_map[sample.action_id]];
        unsigned tick=0;while(tick+1<clip->count && clip->at[tick+1]<=sample.clip_time_ms)++tick;
        const rb_frame_meta_t *meta;
        const lv_image_dsc_t *image=rb_assets_dance_frame(clip->frames[tick],&meta);assert(image);
        assert(fwrite(image->data,1,image->data_size,f)==image->data_size);
    }
    fclose(f);rb_assets_close();free(room);free(blob);lv_deinit();
    printf("Rendered %ums using the firmware timeline/decoder; output RGB565LE 320x240 at 20 FPS\n",plan.total_ms);
}

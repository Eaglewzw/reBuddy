#include "can_console_arm_teach_file.h"
#include <string.h>
static void put16(uint8_t *p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void put32(uint8_t *p,uint32_t v){put16(p,v);put16(p+2,v>>16);}
static uint16_t get16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static uint32_t get32(const uint8_t *p){return get16(p)|((uint32_t)get16(p+2)<<16);}
uint32_t arm_teach_crc32(uint32_t state,const void *data,size_t size)
{
    const uint8_t *p=data;
    while(size--){state^=*p++;for(unsigned b=0;b<8;++b)state=(state>>1)^((state&1)?0xedb88320U:0);}
    return state;
}
static bool valid(const arm_teach_metadata_t *m)
{
    if(m->channel_mask!=ARM_TEACH_CHANNELS_GRIPPER || m->sample_format!=ARM_TEACH_SAMPLE_FORMAT_GRIPPER || (m->model_id!=1 && m->model_id!=2) || m->arm_dof!=6 ||
       m->sample_period_ms!=20 || m->coordinate_version==0 ||
       m->sample_count==0 || m->sample_count>30001 || m->duration_ms>600000 ||
       m->end_reason==ARM_TEACH_END_NONE || m->end_reason>ARM_TEACH_END_TIME)return false;
    for(unsigned j=0;j<6;++j){if(m->motor_ids[j]==0)return false;
        for(unsigned k=0;k<j;++k)if(m->motor_ids[j]==m->motor_ids[k])return false;}
    if(!m->gripper_motor_id || !m->gripper_calibration_id)return false;
    for(unsigned j=0;j<ARM_TEACH_DOF;++j)if(m->motor_ids[j]==m->gripper_motor_id)return false;
    return true;
}
esp_err_t arm_teach_file_header(const arm_teach_metadata_t *m,uint8_t out[64])
{
    if(m==NULL || out==NULL || !valid(m))return ESP_ERR_INVALID_ARG;
    memset(out,0,64);memcpy(out,"TCH1",4);put16(out+4,2);put16(out+6,64);
    out[8]=m->sample_format;out[9]=m->model_id;out[10]=m->arm_dof;out[11]=m->end_reason;
    put16(out+12,m->channel_mask);put16(out+14,ARM_TEACH_FILE_SAMPLE_SIZE);put16(out+16,m->sample_period_ms);
    put32(out+20,m->coordinate_version);put32(out+24,m->calibration_id);
    put32(out+28,m->sample_count);put32(out+32,m->duration_ms);memcpy(out+36,m->motor_ids,6);
    out[42]=m->gripper_motor_id;put32(out+44,m->gripper_calibration_id);
    return ESP_OK;
}
esp_err_t arm_teach_file_parse_header(const uint8_t in[64],arm_teach_metadata_t *m,uint32_t *crc)
{
    if(in==NULL || m==NULL || crc==NULL || memcmp(in,"TCH1",4)!=0 || get16(in+4)!=2 ||
       get16(in+6)!=64 || get16(in+18)!=0)return ESP_ERR_INVALID_ARG;
    if(get16(in+14)!=ARM_TEACH_FILE_SAMPLE_SIZE || in[43]!=0)return ESP_ERR_INVALID_ARG;
    for(unsigned i=48;i<60;++i)if(in[i]!=0)return ESP_ERR_INVALID_ARG;
    arm_teach_metadata_t value={.sample_format=in[8],.model_id=in[9],.arm_dof=in[10],.end_reason=in[11],
        .gripper_motor_id=in[42],.gripper_calibration_id=get32(in+44),
        .channel_mask=get16(in+12),.sample_period_ms=get16(in+16),.coordinate_version=get32(in+20),
        .calibration_id=get32(in+24),.sample_count=get32(in+28),.duration_ms=get32(in+32)};
    memcpy(value.motor_ids,in+36,6);if(!valid(&value))return ESP_ERR_INVALID_ARG;
    *m=value;*crc=get32(in+60);return ESP_OK;
}
void arm_teach_file_encode_sample(const arm_teach_sample_t *s,uint8_t out[ARM_TEACH_FILE_SAMPLE_SIZE])
{put32(out,s->time_ms);for(unsigned j=0;j<6;++j)put16(out+4+2*j,(uint16_t)s->position_q[j]);put16(out+16,s->gripper_q);}
void arm_teach_file_decode_sample(const uint8_t in[ARM_TEACH_FILE_SAMPLE_SIZE],arm_teach_sample_t *s)
{*s=(arm_teach_sample_t){.time_ms=get32(in)};for(unsigned j=0;j<6;++j){uint16_t v=get16(in+4+2*j);s->position_q[j]=(int16_t)(v<=32767?v:(int32_t)v-65536);}s->gripper_q=get16(in+16);}
esp_err_t arm_teach_file_check_sample(const arm_teach_metadata_t *m,uint32_t index,
    const arm_teach_sample_t *s,uint32_t previous)
{
    if(index>=m->sample_count || s->time_ms>600000 ||
       (index==0?s->time_ms!=0:s->time_ms<=previous || s->time_ms-previous>100) ||
       (index+1==m->sample_count && s->time_ms!=m->duration_ms))return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

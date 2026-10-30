#include <assert.h>
#include <float.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include "src/can_console_arm_motion.c"

static atomic_bool prohibit_allocations;
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void *__wrap_malloc(size_t n) { assert(!atomic_load(&prohibit_allocations)); return __real_malloc(n); }
void *__wrap_calloc(size_t n, size_t size) { assert(!atomic_load(&prohibit_allocations)); return __real_calloc(n, size); }
void *__wrap_realloc(void *p, size_t n) { assert(!atomic_load(&prohibit_allocations)); return __real_realloc(p, n); }

bool __wrap_can_console_ik_solve(const float p[3], const float r[9], const float q[6],
    const can_console_ik_options_t *options, float out[6], can_console_ik_result_t *result)
{ (void)p; (void)r; (void)q; (void)options; (void)out; (void)result; assert(!"joint path called IK"); return false; }
void __wrap_can_console_ik_forward(const float q[6], float p[3], float r[9])
{ (void)q; (void)p; (void)r; assert(!"joint path called FK"); }

static arm_joint_path_limits_t limits(float velocity, float acceleration)
{
    arm_joint_path_limits_t out;
    for (unsigned j = 0; j < 6; ++j) { out.velocity[j] = velocity; out.acceleration[j] = acceleration; }
    return out;
}
static void *worker(void *unused)
{
    (void)unused; motion_request_t request;
    assert(take_request(&request)); run_request(&request); return NULL;
}
static can_console_arm_motion_status_t status(void)
{
    can_console_arm_motion_status_t out;
    assert(can_console_arm_motion_get_status(&out) == ESP_OK); return out;
}
static void wait_buffer(unsigned depth)
{
    const int64_t deadline = esp_timer_get_time() + 2000000;
    while (status().buffered_points < depth) {
        assert(esp_timer_get_time() < deadline); vTaskDelay(1);
    }
}
static void finish(pthread_t thread)
{
    pthread_join(thread, NULL);
    assert(!status().planner_busy);
}
static void model_ranges(void)
{
    float lower[6], upper[6];
    assert(!can_console_ik_get_joint_limits(NULL, upper));
    assert(!can_console_ik_get_joint_limits(lower, NULL));
    for (int m = 0; m < 2; ++m) {
        assert(can_console_ik_select_model((can_console_ik_model_t)m));
        assert(can_console_ik_get_joint_limits(lower, upper));
        assert(lower[0] == -2.8f && upper[0] == 2.8f);
        assert(m ? (lower[1] == -3.14f && upper[1] == 0) : (lower[1] == 0 && upper[1] == 3.14f));
        assert(m ? (lower[3] == -1.87f) : (lower[3] == -1.57f));
    }
}
static unsigned checks_left;
static bool allow_check(void *unused) { (void)unused; return checks_left-- != 0; }
static void math_checks(void)
{
    const float lower[6] = {-3.2f,-3.2f,-3.2f,-3.2f,-3.2f,-3.2f};
    const float upper[6] = {3.2f,3.2f,3.2f,3.2f,3.2f,3.2f};
    arm_teach_sample_t raw[6] = {{.time_ms=0}, {.time_ms=20, .position_q={1250}}};
    arm_joint_path_view_t view = {.samples=raw, .count=2};
    arm_joint_path_limits_t cap = limits(.5f, 1e6f);
    arm_joint_path_plan_t plan;
    assert(arm_joint_path_prepare(&view,NULL,lower,upper,20,1,NULL,NULL,&plan)==ESP_OK);
    assert(plan.total_points==2 && plan.effective_speed==1 && plan.duration_s==.02);
    assert(arm_joint_path_prepare(&view,NULL,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_OK);
    assert(plan.total_points==3 && plan.effective_speed==.5 && plan.duration_s==.04);
    assert(arm_joint_path_prepare(&view, &cap, lower, upper, 20, .5f, NULL, NULL, &plan) == ESP_OK);
    /* Two stopped endpoints: q=.125*(3u^2-2u^3), Vmax=9.375 rad/s.
     * The velocity cap needs 18.75 periods, rounded to 19. */
    assert(plan.total_points == 20 && fabs(plan.effective_speed - 1.0/19) < 1e-12);
    cap = limits(100, 1);
    assert(arm_joint_path_prepare(&view, &cap, lower, upper, 20, 1, NULL, NULL, &plan) == ESP_OK);
    /* Amax=1875 rad/s^2 -> sqrt(1/1875), aligned to 44 periods. */
    assert(plan.total_points == 45 && fabs(plan.effective_speed - 1.0/44) < 1e-12);
    arm_joint_path_segment_t segment;
    arm_joint_path_make_segment(&view, 0, &segment);
    for (unsigned i=0; i<=1000; ++i) {
        const double u=i/1000.0;
        can_console_arm_motion_sample_t sample={0};
        arm_joint_path_evaluate(&segment, u, plan.effective_speed, &sample);
        assert(fabs(sample.position[0] - .125*(3*u*u-2*u*u*u)) < 2e-8);
        assert(fabs(sample.reference_velocity[0] - .125*6*u*(1-u)/.02*plan.effective_speed) < 1e-7);
    }
    /* Nonuniform samples, a plateau, reversal and exact hard-bound values. */
    const uint32_t times[6] = {0,7,35,50,90,100};
    const int16_t positions[6] = {0,1250,2500,2500,1000,0};
    for (unsigned i=0; i<6; ++i) {
        raw[i].time_ms=times[i];
        for (unsigned j=0; j<6; ++j) raw[i].position_q[j] = (j & 1) ? -positions[i] : positions[i];
    }
    view.count=6; cap=limits(.4f, 2);
    cap.velocity[4]=.03f; cap.acceleration[5]=.2f;
    assert(arm_joint_path_prepare(&view, &cap, lower, upper, 20, .5f, NULL, NULL, &plan) == ESP_OK);
    for (unsigned i=0; i+1<6; ++i) {
        arm_joint_path_make_segment(&view,i,&segment);
        for (unsigned k=0; k<=1000; ++k) {
            can_console_arm_motion_sample_t q={0};
            const double u=k/1000.0;
            arm_joint_path_evaluate(&segment,u,plan.effective_speed,&q);
            for (unsigned j=0; j<6; ++j) {
                const float p0=raw[i].position_q[j]/10000.f, p1=raw[i+1].position_q[j]/10000.f;
                assert(q.position[j] >= fminf(p0,p1)-1e-7 && q.position[j] <= fmaxf(p0,p1)+1e-7);
                assert(fabs(q.reference_velocity[j]) <= cap.velocity[j]+1e-6);
                /* Independent numerical acceleration from neighbouring velocity evaluations. */
                if (k>0 && k<1000) {
                    can_console_arm_motion_sample_t before={0},after={0};
                    arm_joint_path_evaluate(&segment,u-.0001,plan.effective_speed,&before);
                    arm_joint_path_evaluate(&segment,u+.0001,plan.effective_speed,&after);
                    const double dt=.0002*segment.duration_s/plan.effective_speed;
                    assert(fabs((after.reference_velocity[j]-before.reference_velocity[j])/dt) < cap.acceleration[j]+.003);
                }
            }
            if (i==2) assert(q.position[0]==.25f && q.reference_velocity[0]==0);
        }
        if (i+2<6) {
            can_console_arm_motion_sample_t left={0}, right={0};
            arm_joint_path_evaluate(&segment,1,plan.effective_speed,&left);
            arm_joint_path_make_segment(&view,i+1,&segment);
            arm_joint_path_evaluate(&segment,0,plan.effective_speed,&right);
            for (unsigned j=0;j<6;++j) {
                assert(fabs(left.position[j]-right.position[j])<1e-7);
                assert(fabs(left.reference_velocity[j]-right.reference_velocity[j])<1e-7);
            }
        }
    }
    checks_left=2;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,allow_check,NULL,&plan)==ESP_ERR_INVALID_STATE);
    checks_left=view.count+1;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,allow_check,NULL,&plan)==ESP_ERR_INVALID_STATE);
    raw[5].time_ms=raw[4].time_ms;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_ERR_INVALID_ARG);
    raw[5].time_ms=191;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_ERR_INVALID_ARG);
    raw[5].time_ms=100;raw[0].time_ms=1;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_ERR_INVALID_ARG);
    raw[0].time_ms=0;raw[5].position_q[0]=INT16_MAX;
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_ERR_INVALID_ARG);
    raw[5].position_q[0]=0;cap=limits(FLT_MIN,FLT_MIN);
    assert(arm_joint_path_prepare(&view,&cap,lower,upper,20,.5f,NULL,NULL,&plan)==ESP_ERR_INVALID_ARG);
}

typedef struct {
    uint32_t accepted;
    float last_time;
    bool fail;
    bool defer;
    float expected_velocity;
    bool analytic_two_point;
    can_console_arm_motion_sample_t last;
} consumer_t;
static can_console_arm_motion_sample_result_t consume(const uint8_t ids[6],
    const can_console_arm_motion_sample_t *sample, const float cap[6], void *context)
{
    consumer_t *c=context;
    assert(ids[0]==1 && cap[0]>0);
    if (c->expected_velocity > 0) assert(cap[0] == c->expected_velocity);
    if (c->fail) return CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR;
    if (c->defer) {
        c->last=*sample;return CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER;
    }
    if (c->analytic_two_point) {
        const double u=c->accepted/19.0;
        assert(fabs(sample->position[0]-.125*(3*u*u-2*u*u*u))<2e-8);
        assert(fabs(sample->reference_velocity[0]-.125*6*u*(1-u)/.02/19)<1e-7);
        assert(fabs(sample->reference_time_s-.02*u)<1e-8);
    }
    assert(sample->reference_time_s >= c->last_time);
    for(unsigned j=0;j<6;++j) assert(isfinite(sample->position[j]) && isfinite(sample->reference_velocity[j]));
    c->last_time=sample->reference_time_s;c->last=*sample;++c->accepted;
    return CAN_CONSOLE_ARM_MOTION_SAMPLE_APPLIED;
}
static void drain(consumer_t *consumer)
{
    const int64_t deadline=esp_timer_get_time()+25000000;
    while(status().state!=CAN_CONSOLE_ARM_MOTION_COMPLETE) {
        assert(esp_timer_get_time()<deadline);
        assert(status().state!=CAN_CONSOLE_ARM_MOTION_ERROR);
        if(!can_console_arm_motion_tick_reference(consume,consumer)) vTaskDelay(1);
    }
}
static can_console_arm_motion_sample_result_t consume_legacy(const uint8_t ids[6],
    const float q[6], const float cap[6], void *context)
{
    consumer_t *c=context;
    assert(ids[0]==1 && cap[0]==1);
    if(c->defer) { memcpy(c->last.position,q,sizeof(c->last.position));return CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER; }
    memcpy(c->last.position,q,sizeof(c->last.position));++c->accepted;
    return CAN_CONSOLE_ARM_MOTION_SAMPLE_APPLIED;
}
static void integration_checks(void)
{
    can_console_arm_motion_config_t config;can_console_arm_motion_config_set_defaults(&config);
    assert(can_console_arm_motion_init(&config)==ESP_OK);s_motion.started=true;
    arm_joint_path_limits_t cap=limits(10,100000);
    arm_teach_sample_t short_raw[2]={{.time_ms=0},{.time_ms=20,.position_q={1250}}};
    arm_joint_path_view_t view={.samples=short_raw,.count=2};
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,NAN)==ESP_ERR_INVALID_ARG);
    view.count=1;assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_ERR_INVALID_ARG);view.count=2;
    cap.acceleration[0]=0;assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_ERR_INVALID_ARG);cap.acceleration[0]=100000;
    assert(status().state==CAN_CONSOLE_ARM_MOTION_IDLE);
    /* vlim cannot loosen the separately supplied calibrated per-axis cap. */
    assert(can_console_arm_motion_set_velocity_limit(4)==ESP_OK);
    cap.velocity[0]=.5f;
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_OK);
    assert(s_motion.request.path.samples==short_raw);
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_ERR_INVALID_STATE);
    atomic_store(&prohibit_allocations,true);
    pthread_t thread;assert(pthread_create(&thread,NULL,worker,NULL)==0);
    wait_buffer(20);
    assert(status().effective_speed<.1f && status().requested_speed==.5f);
    assert(status().ik_solves==0 && status().total_points==20);
    consumer_t c={.defer=true,.expected_velocity=.5f,.analytic_two_point=true};
    can_console_arm_motion_sample_t head={0};
    for(unsigned i=0;i<10;++i) {
        assert(!can_console_arm_motion_tick_reference(consume,&c));
        if(i==0)head=c.last;else assert(memcmp(&head,&c.last,sizeof(head))==0);
        assert(status().consumed_points==0 && status().consumed_reference_time_s==0);
    }
    c.defer=false;drain(&c);finish(thread);
    assert(c.accepted==20 && c.last.position[0]==.125f && c.last.reference_velocity[0]==0 && c.last.reference_time_s==.02f);
    atomic_store(&prohibit_allocations,false);
    /* NULL limits bypasses global motion vlim as well as path v/a retiming,
     * while preserving model-bound preflight and exact endpoints. */
    assert(can_console_arm_motion_set_velocity_limit(.0001f)==ESP_OK);
    for(unsigned run=0;run<2;++run) {
        assert(can_console_arm_motion_plan_joint_path(&view,NULL,run?.5f:1)==ESP_OK);
        assert(pthread_create(&thread,NULL,worker,NULL)==0);
        c=(consumer_t){0};drain(&c);finish(thread);
        assert(status().total_points==(run?3:2));
        assert(status().effective_speed==(run?.5f:1));
        assert(c.last.position[0]==.125f && c.last.reference_time_s==.02f);
    }
    short_raw[1].position_q[0]=32767;
    assert(can_console_arm_motion_plan_joint_path(&view,NULL,1)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);finish(thread);
    assert(status().state==CAN_CONSOLE_ARM_MOTION_ERROR && status().produced_points==0);
    short_raw[1].position_q[0]=1250;
    assert(can_console_arm_motion_set_velocity_limit(4)==ESP_OK);
    /* Late corrupt points cause planning errors before any point is emitted. */
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);
    short_raw[1].position_q[0]=30000;
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);finish(thread);
    assert(status().state==CAN_CONSOLE_ARM_MOTION_ERROR && status().error_source==CAN_CONSOLE_ARM_MOTION_ERROR_PLANNING);
    assert(status().produced_points==0 && status().buffered_points==0);
    short_raw[1].position_q[0]=1250;
    /* Execution rejection has a different source and clears prefetched points. */
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);wait_buffer(20);
    c=(consumer_t){.fail=true};assert(!can_console_arm_motion_tick_reference(consume,&c));finish(thread);
    assert(status().state==CAN_CONSOLE_ARM_MOTION_ERROR && status().error_source==CAN_CONSOLE_ARM_MOTION_ERROR_EXECUTION);
    assert(status().buffered_points==0);
    /* Defensive invariant (artificial state, not a reproduced defect): with
     * ERROR, plan_complete and a queued tail point, consuming that last point
     * must leave the state ERROR rather than COMPLETE. */
    {
        motion_point_t tail={0};tail.position[0]=.125f;
        xSemaphoreTake(s_motion.lock,portMAX_DELAY);
        s_motion.state=CAN_CONSOLE_ARM_MOTION_ERROR;s_motion.plan_complete=true;
        (void)xQueueSend(s_motion.buffer,&tail,0);
        xSemaphoreGive(s_motion.lock);
        c=(consumer_t){0};
        assert(can_console_arm_motion_tick_reference(consume,&c));
        assert(status().state==CAN_CONSOLE_ARM_MOTION_ERROR);
        assert(status().buffered_points==0);
    }
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);

    assert(can_console_arm_motion_set_velocity_limit(.1f)==ESP_OK);
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.5f)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);
    c=(consumer_t){.expected_velocity=.1f};drain(&c);finish(thread);
    assert(status().effective_speed < .02f);
    assert(can_console_arm_motion_set_velocity_limit(1)==ESP_OK);

    arm_teach_sample_t *long_raw=calloc(ARM_TEACH_MAX_SAMPLES,sizeof(*long_raw));assert(long_raw);
    for(unsigned i=0;i<ARM_TEACH_MAX_SAMPLES;++i)long_raw[i].time_ms=i*20;
    view=(arm_joint_path_view_t){.samples=long_raw,.count=ARM_TEACH_MAX_SAMPLES};cap=limits(1,1);
    /* Check both actual model ranges, including an int16 point on 2.8 rad. */
    for(int model=0;model<2;++model) {
        assert(can_console_ik_select_model((can_console_ik_model_t)model));
        short_raw[0].position_q[0]=short_raw[1].position_q[0]=28000;
        arm_joint_path_view_t boundary={.samples=short_raw,.count=2};
        assert(can_console_arm_motion_plan_joint_path(&boundary,&cap,.5f)==ESP_OK);
        assert(pthread_create(&thread,NULL,worker,NULL)==0);c=(consumer_t){0};drain(&c);finish(thread);
        short_raw[0].position_q[0]=short_raw[1].position_q[0]=0;
    }
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.1f)==ESP_OK);
    atomic_store(&prohibit_allocations,true);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);wait_buffer(64);
    assert(status().total_points>=300000 && status().duration_s>=5999 && status().ik_solves==0);
    assert(status().planner_busy && status().buffered_points==64);
    assert(can_console_arm_motion_set_velocity_limit(.1f)==ESP_ERR_INVALID_STATE);
    c=(consumer_t){.defer=true};assert(!can_console_arm_motion_tick_reference(consume,&c));
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);finish(thread);
    const uint32_t underruns=status().underruns;
    assert(status().buffered_points==0 && !can_console_arm_motion_tick_reference(consume,&c));
    assert(status().underruns==underruns);
    /* A new request cannot see any old point; fully stream a ten-minute path
     * at 0.1 speed (>300000 output points) with every allocator prohibited. */
    long_raw[0].position_q[0]=100;
    assert(can_console_arm_motion_plan_joint_path(&view,&cap,.1f)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);wait_buffer(64);
    c=(consumer_t){0};assert(can_console_arm_motion_tick_reference(consume,&c));assert(c.last.position[0]==.01f);
    drain(&c);finish(thread);
    assert(c.accepted==status().total_points && c.last.reference_time_s==600 && c.last.position[0]==0);
    assert(status().consumed_reference_time_s==600 && status().buffered_points==0);
    atomic_store(&prohibit_allocations,false);
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);free(long_raw);
    /* Keep the existing position-only callback and ordinary joint planner
     * working through the same queue, including DEFER and caller context. */
    const float start[6]={0},target[6]={.01f};
    assert(sizeof(joint_position_t)==24 && sizeof(motion_point_t)==52);
    assert(can_console_arm_motion_plan_joints(start,target,.1f)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);wait_buffer(6);
    c=(consumer_t){.defer=true};assert(!can_console_arm_motion_tick(consume_legacy,&c));
    assert(status().consumed_points==0 && c.last.position[0]==0);
    c.defer=false;
    while(status().state!=CAN_CONSOLE_ARM_MOTION_COMPLETE) {
        if(!can_console_arm_motion_tick(consume_legacy,&c))vTaskDelay(1);
    }
    finish(thread);assert(c.accepted==6 && c.last.position[0]==.01f);
    /* Seek uses the existing joint solver but must obey acceleration as well
     * as velocity, even when the requested duration is much too short. */
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);
    const float seek_target[6]={.2f};cap=limits(.2f,.5f);
    assert(can_console_arm_motion_plan_joints_limited(start,seek_target,.1f,&cap)==ESP_OK);
    assert(pthread_create(&thread,NULL,worker,NULL)==0);wait_buffer(64);
    assert(status().duration_s>=1.875f);
    c=(consumer_t){0};float previous_velocity=0;
    while(status().state!=CAN_CONSOLE_ARM_MOTION_COMPLETE) {
        if(!can_console_arm_motion_tick_reference(consume,&c)){vTaskDelay(1);continue;}
        assert(fabsf(c.last.reference_velocity[0])<=.200001f);
        assert(fabsf(c.last.reference_velocity[0]-previous_velocity)/.02f<=.50001f);
        previous_velocity=c.last.reference_velocity[0];
    }
    finish(thread);assert(c.last.position[0]==.2f && previous_velocity==0);
    /* Stopping streams through the same queue, retains a DEFERred head, and
     * invalid requests cannot destroy the stop already in progress. */
    assert(can_console_arm_motion_cancel_and_wait(1000)==ESP_OK);
    const float brake_velocity[6]={.25f,-.125f};cap=limits(1,1);
    atomic_store(&prohibit_allocations,true);
    assert(can_console_arm_motion_plan_stop(start,brake_velocity,&cap,20)==ESP_OK);
    const uint32_t stop_points=status().total_points;assert(stop_points==13);
    c=(consumer_t){.defer=true};assert(!can_console_arm_motion_tick_reference(consume,&c));head=c.last;
    cap.velocity[0]=NAN;
    assert(can_console_arm_motion_plan_stop(start,brake_velocity,&cap,20)==ESP_ERR_INVALID_ARG);
    cap.velocity[0]=1;
    const float outside_stop[6]={2.799f};
    assert(can_console_arm_motion_plan_stop(outside_stop,brake_velocity,&cap,20)==ESP_ERR_INVALID_ARG);
    assert(status().total_points==stop_points && status().consumed_points==0);
    assert(!can_console_arm_motion_tick_reference(consume,&c) && memcmp(&head,&c.last,sizeof(head))==0);
    c.defer=false;previous_velocity=.25f;float previous_position=0;
    while(status().state!=CAN_CONSOLE_ARM_MOTION_COMPLETE) {
        assert(can_console_arm_motion_tick_reference(consume,&c));
        assert(c.last.position[0]>=previous_position && c.last.reference_velocity[0]>=0);
        assert(c.last.reference_velocity[0]<=previous_velocity);
        assert((previous_velocity-c.last.reference_velocity[0])/.02f<=1.00001f);
        previous_velocity=c.last.reference_velocity[0];previous_position=c.last.position[0];
    }
    assert(c.accepted==13 && previous_velocity==0);
    assert(fabsf(c.last.position[0]-.0325f)<1e-7f && fabsf(c.last.position[1]+.01625f)<1e-7f);
    atomic_store(&prohibit_allocations,false);
    vQueueDelete(s_motion.buffer);vSemaphoreDelete(s_motion.lock);
}
int main(void)
{
    model_ranges();math_checks();integration_checks();
    puts("joint path passed: analytic retiming, nonuniform PCHIP, zero allocations/IK, >300000 streamed points, DEFER/cancel/error boundaries");
    return 0;
}

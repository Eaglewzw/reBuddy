#include "vendors/motor_vendor.h"

#include <string.h>

/* Compile-time exclusion: builds that only drive RobStride motors can define
 * MOTORBRIDGE_NO_DAMIAO to keep damiao's per-vendor static tables (~12 KB of
 * BSS: reg_cache / write-ack stamps / reg-verify state, all sized [256] by
 * register id, not by MOTORBRIDGE_MAX_MOTOR_ID) out of the linked image.  With
 * no reference surviving, the static library never pulls damiao_vendor.o in.
 * Default (macro undefined) keeps full damiao support, so shared-demo builds
 * are unaffected. */
#ifndef MOTORBRIDGE_NO_DAMIAO
extern const motor_vendor_ops_t *motor_vendor_get_damiao(void);
#endif
extern const motor_vendor_ops_t *motor_vendor_get_robstride(void);

/* 运行时注册表：新增厂商只需在 ensure_registry() 登记一行。
 * 用运行时表而非 static const 数组，是因为厂商 ops 由各 vendor.c 提供的
 * 取值函数返回（避免跨编译单元静态初始化顺序问题）。 */
static const motor_vendor_ops_t *s_vendors[2];
static int s_vendor_count = 0;

static void ensure_registry(void)
{
    static bool inited = false;
    if (inited) {
        return;
    }
    inited = true;
#ifndef MOTORBRIDGE_NO_DAMIAO
    s_vendors[0] = motor_vendor_get_damiao();
    s_vendors[1] = motor_vendor_get_robstride();
    s_vendor_count = 2;
#else
    s_vendors[0] = motor_vendor_get_robstride();
    s_vendor_count = 1;
#endif
}

const motor_vendor_ops_t *motor_vendor_get(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    ensure_registry();
    for (int i = 0; i < s_vendor_count; ++i) {
        if (s_vendors[i] != NULL && s_vendors[i]->name != NULL &&
            strcmp(s_vendors[i]->name, name) == 0) {
            return s_vendors[i];
        }
    }
    return NULL;
}

int motor_vendor_count(void)
{
    ensure_registry();
    return s_vendor_count;
}

const motor_vendor_ops_t *motor_vendor_get_by_index(int index)
{
    ensure_registry();
    if (index < 0 || index >= s_vendor_count) {
        return NULL;
    }
    return s_vendors[index];
}



#ifndef RB_CHOREO_STORE_H
#define RB_CHOREO_STORE_H

#include <stddef.h>
#include <stdint.h>
#include "rb_choreo.h"

#define RB_CHOREO_STORE_SLOTS RB_CHOREO_SLOTS

typedef enum {
    RB_CHOREO_STORE_OK = 0,
    RB_CHOREO_STORE_EMPTY,
    RB_CHOREO_STORE_CORRUPT,
    RB_CHOREO_STORE_IO,
    RB_CHOREO_STORE_VERSION,
    RB_CHOREO_STORE_INVALID
} rb_choreo_store_result_t;

/* Keys are 0..19: two independently committed records for each of ten slots.
 * EMPTY means no record. A read larger than capacity must return CORRUPT.
 * write must commit before returning OK; an error may leave a partial record.
 * The store never erases the other record, including when a write fails. */
typedef struct {
    void *context;
    rb_choreo_store_result_t (*read)(void *context, unsigned key,
                                   uint8_t *data, size_t capacity, size_t *length);
    rb_choreo_store_result_t (*write)(void *context, unsigned key,
                                    const uint8_t *data, size_t length);
} rb_choreo_store_backend_t;

/* Call before init to inject a backend; NULL restores the platform backend.
 * The callback context must remain alive. Neither call saves a work. */
rb_choreo_store_result_t rb_choreo_store_set_backend(const rb_choreo_store_backend_t *backend);
rb_choreo_store_result_t rb_choreo_store_init(void);
rb_choreo_store_result_t rb_choreo_store_load(unsigned slot, rb_choreo_work_t *work);
rb_choreo_store_result_t rb_choreo_store_save(unsigned slot, const rb_choreo_work_t *work);

#endif

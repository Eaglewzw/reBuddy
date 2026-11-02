#pragma once
#include "rb_daily_control.h"
#include "rb_pet.h"

/* Follow one copied controller snapshot. True requests the separate SLEEP
 * disable step after verified REST arrival; it does not send motor commands.
 * REST stays on screen until fresh hardware disable is confirmed. */
bool rb_daily_sync_screen(rb_pet_t *pet,const rb_dc_status_t *status);

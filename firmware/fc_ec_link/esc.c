#include "esc.h"

#include <math.h>
#include <string.h>

bool fcec_esc_config_ok(const fcec_esc_config_t *c) {
    if (c == NULL || !c->approved) {
        return false;
    }
    if (c->min_pulse_us == 0 || c->max_pulse_us <= c->min_pulse_us) {
        return false;
    }
    if (c->frame_period_us <= c->max_pulse_us) {
        return false; /* the pulse has to end inside its frame */
    }
    /* Written so NaN fails too. */
    if (!(c->max_throttle > 0.0f && c->max_throttle <= 1.0f)) {
        return false;
    }
    return c->max_rise_per_s >= 0.0f && isfinite(c->max_rise_per_s); /* 0 = no limit */
}

static uint16_t pulse_for(const fcec_esc_config_t *c, float throttle) {
    float span = (float)(c->max_pulse_us - c->min_pulse_us);
    return (uint16_t)((float)c->min_pulse_us + throttle * span + 0.5f);
}

void fcec_esc_init(fcec_esc_t *esc, const fcec_esc_config_t *config, uint64_t now_us) {
    memset(esc, 0, sizeof *esc);
    if (config != NULL) {
        esc->config = *config;
    }
    esc->started_us = now_us;
    if (fcec_esc_config_ok(config)) {
        esc->state = FCEC_ESC_ARMING;
        esc->pulse_us = esc->config.min_pulse_us;
    } else {
        esc->state = FCEC_ESC_UNCONFIGURED;
        esc->pulse_us = 0; /* no signal at all */
    }
}

uint16_t fcec_esc_update(fcec_esc_t *esc, float target, bool drive, uint64_t now_us) {
    float dt_s = 0.0f;
    if (esc->have_update && now_us > esc->last_update_us) {
        dt_s = (float)(now_us - esc->last_update_us) / 1e6f;
    }
    esc->last_update_us = now_us;
    esc->have_update = true;

    if (esc->state == FCEC_ESC_UNCONFIGURED) {
        esc->throttle = 0.0f;
        esc->pulse_us = 0;
        return 0;
    }
    if (esc->state == FCEC_ESC_ARMING && now_us >= esc->started_us &&
        now_us - esc->started_us >= (uint64_t)esc->config.arm_hold_ms * 1000u) {
        esc->state = FCEC_ESC_READY;
    }

    float want = 0.0f;
    if (esc->state == FCEC_ESC_READY && drive && isfinite(target) && target > 0.0f) {
        want = target < esc->config.max_throttle ? target : esc->config.max_throttle;
    }
    if (want <= esc->throttle || esc->config.max_rise_per_s == 0.0f) {
        esc->throttle = want; /* cuts are immediate; so is any change without a limit */
    } else {
        float rise = esc->config.max_rise_per_s * dt_s;
        esc->throttle = want - esc->throttle > rise ? esc->throttle + rise : want;
    }

    esc->pulse_us = pulse_for(&esc->config, esc->throttle);
    return esc->pulse_us;
}

void fcec_esc_fault(fcec_esc_t *esc) {
    if (esc->state == FCEC_ESC_UNCONFIGURED) {
        return;
    }
    esc->state = FCEC_ESC_FAULT;
    esc->throttle = 0.0f;
    esc->pulse_us = esc->config.min_pulse_us;
}

void fcec_esc_clear_fault(fcec_esc_t *esc, uint64_t now_us) {
    if (esc->state == FCEC_ESC_FAULT) {
        esc->state = FCEC_ESC_ARMING;
        esc->started_us = now_us;
    }
}

bool fcec_esc_ready(const fcec_esc_t *esc) {
    return esc->state == FCEC_ESC_READY;
}

uint16_t fcec_esc_service(fcec_esc_t *esc, fcec_ec_t *ec, uint64_t now_us) {
    fcec_ec_command_t cmd = fcec_ec_command(ec);
    bool drive = (cmd.drive_mask & FCEC_CH_MAIN_ESC) != 0;

    /* The throttle loaded on an earlier call is on the wire once a full frame
     * has passed since this command started: confirm it before loading the
     * next one. */
    if (drive && esc->state != FCEC_ESC_UNCONFIGURED) {
        if (!esc->have_gen || esc->gen != cmd.gen) {
            esc->gen = cmd.gen;
            esc->gen_since_us = now_us;
            esc->have_gen = true;
        } else if (now_us >= esc->gen_since_us &&
                   now_us - esc->gen_since_us >= esc->config.frame_period_us) {
            fcec_outputs_t actual = *cmd.outputs;
            actual.main_throttle = esc->throttle;
            fcec_ec_confirm_applied(ec, FCEC_CH_MAIN_ESC, cmd.gen, &actual, now_us);
        }
    }

    uint16_t pulse = fcec_esc_update(esc, cmd.outputs->main_throttle, drive, now_us);

    bool ready = fcec_esc_ready(esc);
    if (ready != ((ec->ready_mask & FCEC_CH_MAIN_ESC) != 0)) {
        fcec_ec_set_ready(ec, FCEC_CH_MAIN_ESC, ready);
    }
    return pulse;
}

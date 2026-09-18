// Stepper pulse schedule compression
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// The goal of this code is to take a series of scheduled stepper
// pulse times and compress them into a handful of commands that can
// be efficiently transmitted and executed on a microcontroller (mcu).
// The mcu accepts step pulse commands that take interval, count, and
// add parameters such that 'count' pulses occur, with each step event
// calculating the next step event time using:
//  next_wake_time = last_wake_time + interval; interval += add
// This code is written in C (instead of python) for processing
// efficiency - the repetitive integer math is vastly faster in C.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdint.h> // uint32_t
#include <stdio.h> // fprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // DIV_ROUND_UP
#include "pyhelper.h" // errorf
#include "serialqueue.h" // struct queue_message
#include "stepcompress.h" // stepcompress_alloc

#define CHECK_LINES 1
#define QUEUE_START_SIZE 1024

// Fixed per-stepper rings avoid allocation in the step-generation path.
#define DEBUG_MOTION_VERSION "motion-v3"
#define DEBUG_APPEND_COUNT 24
#define DEBUG_EVENT_COUNT 8
#define DEBUG_QUEUE_COUNT 12

struct debug_append {
    double print_time, step_time, offset, rel_sc;
    double time_offset, freq, last_print_time;
    uint64_t clock, last_clock, pending_clock;
    int sdir, pending_dir, tag;
};

struct debug_event {
    const char *kind;
    uint64_t before, after;
    double time_offset, freq, last_print_time;
    uint32_t interval;
    int count, add, sdir;
};

// Storage for queuing steps (only lower 32 bits of step clock are stored as
// optimization to reduce memory, improve cache usage, and reduce 64 bit ops)
struct qstep {
    uint32_t clock32;
};

// Main stepcompress object storage
struct stepcompress {
    // Buffer management
    struct qstep *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq, last_step_print_time;
    // Message generation
    uint64_t last_step_clock;
    struct list_head *msg_queue;
    uint32_t oid;
    int32_t queue_step_msgtag, set_next_step_dir_msgtag;
    int sdir, invert_sdir;
    // Step+dir+step filter
    uint64_t next_step_clock;
    int next_step_dir;
    // History tracking
    int64_t last_position;
    struct list_head history_list;
    // Fixed rings: no allocation or logging in the step-generation path.
    struct debug_append dbg_append[DEBUG_APPEND_COUNT];
    struct debug_event dbg_event[DEBUG_EVENT_COUNT];
    unsigned int dbg_pos, dbg_count, dbg_event_pos, dbg_event_count;
    int dbg_reported;
};

// Parameters of a single queue_step command
struct step_move {
    uint32_t interval;
    uint16_t count;
    int16_t add;
};

// Storage for internal history of recently sent queue_step commands
struct history_steps {
    struct list_node node;
    uint64_t first_clock, last_clock;
    int64_t start_position;
    int step_count, interval, add;
};

// Events share the preserved flush ring, including clock and direction changes.
static void
debug_record_event(struct stepcompress *sc, const char *kind, uint64_t after
                   , uint32_t interval, int count, int add)
{
    struct debug_event *d = &sc->dbg_event[sc->dbg_event_pos];
    *d = (struct debug_event){
        kind, sc->last_step_clock, after, sc->mcu_time_offset, sc->mcu_freq,
        sc->last_step_print_time, interval, count, add, sc->sdir
    };
    sc->dbg_event_pos = (sc->dbg_event_pos + 1) % DEBUG_EVENT_COUNT;
    if (sc->dbg_event_count < DEBUG_EVENT_COUNT)
        sc->dbg_event_count++;
}


/****************************************************************
 * Step compression
 ****************************************************************/

static inline int32_t
idiv_up(int32_t n, int32_t d)
{
    return (n>=0) ? DIV_ROUND_UP(n,d) : (n/d);
}

static inline int32_t
idiv_down(int32_t n, int32_t d)
{
    return (n>=0) ? (n/d) : (n - d + 1) / d;
}

struct points {
    int32_t minp, maxp;
};

// Given a requested step time, return the minimum and maximum
// acceptable times
static inline struct points
minmax_point(struct stepcompress *sc, struct qstep *pos)
{
    uint32_t lsc = sc->last_step_clock, point = pos->clock32 - lsc;
    uint32_t prevpoint = pos > sc->queue_pos ? (pos-1)->clock32 - lsc : 0;
    uint32_t max_error = (point - prevpoint) / 2;
    if (max_error > sc->max_error)
        max_error = sc->max_error;
    return (struct points){ point - max_error, point };
}

// The maximum add delta between two valid quadratic sequences of the
// form "add*count*(count-1)/2 + interval*count" is "(6 + 4*sqrt(2)) *
// maxerror / (count*count)".  The "6 + 4*sqrt(2)" is 11.65685, but
// using 11 works well in practice.
#define QUADRATIC_DEV 11

// Find a 'step_move' that covers a series of step times
static struct step_move
compress_bisect_add(struct stepcompress *sc)
{
    struct qstep *qlast = sc->queue_next;
    if (qlast > sc->queue_pos + 65535)
        qlast = sc->queue_pos + 65535;
    struct points point = minmax_point(sc, sc->queue_pos);
    int32_t outer_mininterval = point.minp, outer_maxinterval = point.maxp;
    int32_t add = 0, minadd = -0x8000, maxadd = 0x7fff;
    int32_t bestinterval = 0, bestcount = 1, bestadd = 1, bestreach = INT32_MIN;
    int32_t zerointerval = 0, zerocount = 0;

    for (;;) {
        // Find longest valid sequence with the given 'add'
        struct points nextpoint;
        int32_t nextmininterval = outer_mininterval;
        int32_t nextmaxinterval = outer_maxinterval, interval = nextmaxinterval;
        int32_t nextcount = 1;
        for (;;) {
            nextcount++;
            if (&sc->queue_pos[nextcount-1] >= qlast) {
                int32_t count = nextcount - 1;
                return (struct step_move){ interval, count, add };
            }
            nextpoint = minmax_point(sc, sc->queue_pos + nextcount - 1);
            int32_t nextaddfactor = nextcount*(nextcount-1)/2;
            int32_t c = add*nextaddfactor;
            if (nextmininterval*nextcount < nextpoint.minp - c)
                nextmininterval = idiv_up(nextpoint.minp - c, nextcount);
            if (nextmaxinterval*nextcount > nextpoint.maxp - c)
                nextmaxinterval = idiv_down(nextpoint.maxp - c, nextcount);
            if (nextmininterval > nextmaxinterval)
                break;
            interval = nextmaxinterval;
        }

        // Check if this is the best sequence found so far
        int32_t count = nextcount - 1, addfactor = count*(count-1)/2;
        int32_t reach = add*addfactor + interval*count;
        if (reach > bestreach
            || (reach == bestreach && interval > bestinterval)) {
            bestinterval = interval;
            bestcount = count;
            bestadd = add;
            bestreach = reach;
            if (!add) {
                zerointerval = interval;
                zerocount = count;
            }
            if (count > 0x200)
                // No 'add' will improve sequence; avoid integer overflow
                break;
        }

        // Check if a greater or lesser add could extend the sequence
        int32_t nextaddfactor = nextcount*(nextcount-1)/2;
        int32_t nextreach = add*nextaddfactor + interval*nextcount;
        if (nextreach < nextpoint.minp) {
            minadd = add + 1;
            outer_maxinterval = nextmaxinterval;
        } else {
            maxadd = add - 1;
            outer_mininterval = nextmininterval;
        }

        // The maximum valid deviation between two quadratic sequences
        // can be calculated and used to further limit the add range.
        if (count > 1) {
            int32_t errdelta = sc->max_error*QUADRATIC_DEV / (count*count);
            if (minadd < add - errdelta)
                minadd = add - errdelta;
            if (maxadd > add + errdelta)
                maxadd = add + errdelta;
        }

        // See if next point would further limit the add range
        int32_t c = outer_maxinterval * nextcount;
        if (minadd*nextaddfactor < nextpoint.minp - c)
            minadd = idiv_up(nextpoint.minp - c, nextaddfactor);
        c = outer_mininterval * nextcount;
        if (maxadd*nextaddfactor > nextpoint.maxp - c)
            maxadd = idiv_down(nextpoint.maxp - c, nextaddfactor);

        // Bisect valid add range and try again with new 'add'
        if (minadd > maxadd)
            break;
        add = maxadd - (maxadd - minadd) / 4;
    }
    if (zerocount + zerocount/16 >= bestcount)
        // Prefer add=0 if it's similar to the best found sequence
        return (struct step_move){ zerointerval, zerocount, 0 };
    return (struct step_move){ bestinterval, bestcount, bestadd };
}


/****************************************************************
 * Step compress checking
 ****************************************************************/

// One dump per object lifetime: at most 80 lines, each below errorf's 512 bytes.
// Queue clocks are raw low words; forward_clock is a modular reconstruction,
// not evidence that the original full clock actually advanced.
static void
debug_dump(struct stepcompress *sc, struct step_move move, int point_index
           , const char *reason)
{
    if (sc->dbg_reported)
        return;
    sc->dbg_reported = 1;
    int queued = sc->queue ? sc->queue_next - sc->queue_pos : 0;
    errorf("[DEBUG-motion] version=" DEBUG_MOTION_VERSION
           " o=%u sc=%p reason=%s i=%u c=%u a=%d point=%d"
           " queued=%d max_error=%u sdir=%d pending_dir=%d invert=%d"
           , sc->oid, (void *)sc, reason, move.interval, move.count, move.add
           , point_index, queued, sc->max_error, sc->sdir, sc->next_step_dir
           , sc->invert_sdir);
    errorf("[DEBUG-motion] o=%u state last_clock=%llu pending_clock=%llu"
           " last_print_time=%.17g time_offset=%.17g freq=%.17g"
           , sc->oid, (unsigned long long)sc->last_step_clock
           , (unsigned long long)sc->next_step_clock, sc->last_step_print_time
           , sc->mcu_time_offset, sc->mcu_freq);
    int i;
    // Include a failing point outside the head window without an unbounded scan.
    for (i = 0; i < DEBUG_QUEUE_COUNT + 1; i++) {
        int qi = i < DEBUG_QUEUE_COUNT ? i : point_index;
        if (qi < 0 || qi >= queued
            || (i == DEBUG_QUEUE_COUNT && qi < DEBUG_QUEUE_COUNT))
            continue;
        uint32_t clock = sc->queue_pos[qi].clock32;
        uint32_t delta = clock - (uint32_t)sc->last_step_clock;
        uint32_t prev = qi ? sc->queue_pos[qi-1].clock32 : sc->last_step_clock;
        struct points p = minmax_point(sc, sc->queue_pos + qi);
        errorf("[DEBUG-motion] o=%u queue[%d] clock32=%u delta_u32=%u"
               " delta_i32=%d previous_delta_i32=%d forward_clock=%llu"
               " min=%d max=%d"
               , sc->oid, qi, clock, delta, (int32_t)delta
               , (int32_t)(clock - prev)
               , (unsigned long long)(sc->last_step_clock + delta)
               , p.minp, p.maxp);
    }
    unsigned int k;
    for (k = 0; k < sc->dbg_event_count; k++) {
        unsigned int idx = (sc->dbg_event_pos + DEBUG_EVENT_COUNT
                            - sc->dbg_event_count + k) % DEBUG_EVENT_COUNT;
        struct debug_event *d = &sc->dbg_event[idx];
        errorf("[DEBUG-motion] o=%u event[%u] kind=%s before=%llu after=%llu"
               " i=%u c=%d a=%d sdir=%d time_offset=%.17g freq=%.17g"
               " last_print_time=%.17g"
               , sc->oid, k, d->kind, (unsigned long long)d->before
               , (unsigned long long)d->after, d->interval, d->count, d->add
               , d->sdir, d->time_offset, d->freq, d->last_print_time);
    }
    for (k = 0; k < sc->dbg_count; k++) {
        unsigned int idx = (sc->dbg_pos + DEBUG_APPEND_COUNT
                            - sc->dbg_count + k) % DEBUG_APPEND_COUNT;
        struct debug_append *d = &sc->dbg_append[idx];
        errorf("[DEBUG-motion] o=%u append[%u] move_pt=%.17g step_t=%.17g"
               " abs=%.17g clock=%llu last_clock=%llu pending_clock=%llu"
               " sdir=%d pending_dir=%d tag=%d"
               , sc->oid, k, d->print_time, d->step_time
               , d->print_time + d->step_time, (unsigned long long)d->clock
               , (unsigned long long)d->last_clock
               , (unsigned long long)d->pending_clock
               , d->sdir, d->pending_dir, d->tag);
        errorf("[DEBUG-motion] o=%u conversion[%u] offset=%.17g rel_sc=%.17g"
               " time_offset=%.17g freq=%.17g last_print_time=%.17g"
               , sc->oid, k, d->offset, d->rel_sc, d->time_offset, d->freq
               , d->last_print_time);
    }
}

// Verify that a given 'step_move' matches the actual step times
static int
check_line(struct stepcompress *sc, struct step_move move)
{
    if (!CHECK_LINES)
        return 0;
    if (!move.count || (!move.interval && !move.add && move.count > 1)
        || move.interval >= 0x80000000) {
        errorf("stepcompress o=%d i=%d c=%d a=%d: Invalid sequence"
               , sc->oid, move.interval, move.count, move.add);
        debug_dump(sc, move, -1, "invalid-sequence");
        return ERROR_RET;
    }
    uint32_t interval = move.interval, p = 0;
    uint16_t i;
    for (i=0; i<move.count; i++) {
        struct points point = minmax_point(sc, sc->queue_pos + i);
        p += interval;
        if (p < point.minp || p > point.maxp) {
            errorf("stepcompress o=%d i=%d c=%d a=%d: Point %d: %d not in %d:%d"
                   , sc->oid, move.interval, move.count, move.add
                   , i+1, p, point.minp, point.maxp);
            debug_dump(sc, move, i, "point-out-of-range");
            return ERROR_RET;
        }
        if (interval >= 0x80000000) {
            errorf("stepcompress o=%d i=%d c=%d a=%d:"
                   " Point %d: interval overflow %d"
                   , sc->oid, move.interval, move.count, move.add
                   , i+1, interval);
            debug_dump(sc, move, i, "interval-overflow");
            return ERROR_RET;
        }
        interval += move.add;
    }
    return 0;
}


/****************************************************************
 * Step compress interface
 ****************************************************************/

// Allocate a new 'stepcompress' object
struct stepcompress *
stepcompress_alloc(struct list_head *msg_queue)
{
    struct stepcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->history_list);
    sc->sdir = -1;
    sc->msg_queue = msg_queue;
    return sc;
}

// Fill message id information
void __visible
stepcompress_fill(struct stepcompress *sc, uint32_t oid, uint32_t max_error
                  , int32_t queue_step_msgtag, int32_t set_next_step_dir_msgtag)
{
    sc->oid = oid;
    sc->max_error = max_error;
    sc->queue_step_msgtag = queue_step_msgtag;
    sc->set_next_step_dir_msgtag = set_next_step_dir_msgtag;
}

// Set the inverted stepper direction flag
void __visible
stepcompress_set_invert_sdir(struct stepcompress *sc, uint32_t invert_sdir)
{
    invert_sdir = !!invert_sdir;
    if (invert_sdir != sc->invert_sdir) {
        sc->invert_sdir = invert_sdir;
        if (sc->sdir >= 0)
            sc->sdir ^= 1;
    }
}

// Expire the stepcompress history older than the given clock
void
stepcompress_history_expire(struct stepcompress *sc, uint64_t end_clock)
{
    while (!list_empty(&sc->history_list)) {
        struct history_steps *hs = list_last_entry(
            &sc->history_list, struct history_steps, node);
        if (hs->last_clock > end_clock)
            break;
        list_del(&hs->node);
        free(hs);
    }
}

// Free memory associated with a 'stepcompress' object
void
stepcompress_free(struct stepcompress *sc)
{
    if (!sc)
        return;
    free(sc->queue);
    stepcompress_history_expire(sc, UINT64_MAX);
    free(sc);
}

uint32_t
stepcompress_get_oid(struct stepcompress *sc)
{
    return sc->oid;
}

int
stepcompress_get_step_dir(struct stepcompress *sc)
{
    return sc->next_step_dir;
}

// Determine the "print time" of the last_step_clock
static void
calc_last_step_print_time(struct stepcompress *sc)
{
    double lsc = sc->last_step_clock;
    sc->last_step_print_time = sc->mcu_time_offset + (lsc - .5) / sc->mcu_freq;
}

// Set the conversion rate of 'print_time' to mcu clock
void
stepcompress_set_time(struct stepcompress *sc
                      , double time_offset, double mcu_freq)
{
    debug_record_event(sc, "set-time-before", sc->last_step_clock, 0, 0, 0);
    sc->mcu_time_offset = time_offset;
    sc->mcu_freq = mcu_freq;
    calc_last_step_print_time(sc);
    debug_record_event(sc, "set-time-after", sc->last_step_clock, 0, 0, 0);
}

// Maximium clock delta between messages in the queue
#define CLOCK_DIFF_MAX (3<<28)

// Helper to create a queue_step command from a 'struct step_move'
static void
add_move(struct stepcompress *sc, uint64_t first_clock, struct step_move *move)
{
    int32_t addfactor = move->count*(move->count-1)/2;
    uint32_t ticks = move->add*addfactor + move->interval*(move->count-1);
    uint64_t last_clock = first_clock + ticks;
    debug_record_event(sc, "flush", last_clock
                       , move->interval, move->count, move->add);

    // Create and queue a queue_step command
    uint32_t msg[5] = {
        sc->queue_step_msgtag, sc->oid, move->interval, move->count, move->add
    };
    struct queue_message *qm = message_alloc_and_encode(msg, 5);
    qm->min_clock = qm->req_clock = sc->last_step_clock;
    if (move->count == 1 && first_clock >= sc->last_step_clock + CLOCK_DIFF_MAX)
        qm->req_clock = first_clock;
    list_add_tail(&qm->node, sc->msg_queue);
    sc->last_step_clock = last_clock;

    // Create and store move in history tracking
    struct history_steps *hs = malloc(sizeof(*hs));
    hs->first_clock = first_clock;
    hs->last_clock = last_clock;
    hs->start_position = sc->last_position;
    hs->interval = move->interval;
    hs->add = move->add;
    hs->step_count = sc->sdir ? move->count : -move->count;
    sc->last_position += hs->step_count;
    list_add_head(&hs->node, &sc->history_list);
}

// Convert previously scheduled steps into commands for the mcu
static int
queue_flush(struct stepcompress *sc, uint64_t move_clock)
{
    if (sc->queue_pos >= sc->queue_next)
        return 0;
    while (sc->last_step_clock < move_clock) {
        struct step_move move = compress_bisect_add(sc);
        // The target is useful when a partial flush detects an older bad step.
        // Record only on failure so normal compression retains its event window.
        int ret = check_line(sc, move);
        if (ret) {
            if (sc->dbg_reported == 1) {
                sc->dbg_reported = 2;
                errorf("[DEBUG-motion] o=%u flush_target=%llu"
                       , sc->oid, (unsigned long long)move_clock);
            }
            return ret;
        }

        add_move(sc, sc->last_step_clock + move.interval, &move);

        if (sc->queue_pos + move.count >= sc->queue_next) {
            sc->queue_pos = sc->queue_next = sc->queue;
            break;
        }
        sc->queue_pos += move.count;
    }
    calc_last_step_print_time(sc);
    return 0;
}

// Generate a queue_step for a step far in the future from the last step
static int
stepcompress_flush_far(struct stepcompress *sc, uint64_t abs_step_clock)
{
    struct step_move move = { abs_step_clock - sc->last_step_clock, 1, 0 };
    add_move(sc, abs_step_clock, &move);
    calc_last_step_print_time(sc);
    return 0;
}

// Send the set_next_step_dir command
static int
set_next_step_dir(struct stepcompress *sc, int sdir)
{
    if (sc->sdir == sdir)
        return 0;
    int ret = queue_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->sdir = sdir;
    debug_record_event(sc, "direction", sc->last_step_clock, 0, 0, 0);
    uint32_t msg[3] = {
        sc->set_next_step_dir_msgtag, sc->oid, sdir ^ sc->invert_sdir
    };
    struct queue_message *qm = message_alloc_and_encode(msg, 3);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, sc->msg_queue);
    return 0;
}

// Slow path for queue_append() - handle next step far in future
static int
queue_append_far(struct stepcompress *sc)
{
    uint64_t step_clock = sc->next_step_clock;
    sc->next_step_clock = 0;
    int ret = queue_flush(sc, step_clock - CLOCK_DIFF_MAX + 1);
    if (ret)
        return ret;
    if (step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX)
        return stepcompress_flush_far(sc, step_clock);
    sc->queue_next->clock32 = step_clock;
    sc->queue_next++;
    return 0;
}

// Slow path for queue_append() - expand the internal queue storage
static int
queue_append_extend(struct stepcompress *sc)
{
    if (sc->queue_next - sc->queue_pos > 65535 + 2000) {
        // No point in keeping more than 64K steps in memory
        uint32_t flush = ((sc->queue_next-65535)->clock32
                          - (uint32_t)sc->last_step_clock);
        int ret = queue_flush(sc, sc->last_step_clock + flush);
        if (ret)
            return ret;
    }

    if (sc->queue_next >= sc->queue_end) {
        // Make room in the queue
        int in_use = sc->queue_next - sc->queue_pos;
        if (sc->queue_pos > sc->queue) {
            // Shuffle the internal queue to avoid having to allocate more ram
            memmove(sc->queue, sc->queue_pos, in_use * sizeof(*sc->queue));
        } else {
            // Expand the internal queue of step times
            int alloc = sc->queue_end - sc->queue;
            if (!alloc)
                alloc = QUEUE_START_SIZE;
            while (in_use >= alloc)
                alloc *= 2;
            sc->queue = realloc(sc->queue, alloc * sizeof(*sc->queue));
            sc->queue_end = sc->queue + alloc;
        }
        sc->queue_pos = sc->queue;
        sc->queue_next = sc->queue + in_use;
    }

    sc->queue_next->clock32 = sc->next_step_clock;
    sc->queue_next++;
    sc->next_step_clock = 0;
    return 0;
}

// Add a step time to the queue (flushing the queue if needed)
static int
queue_append(struct stepcompress *sc)
{
    if (unlikely(sc->next_step_dir != sc->sdir)) {
        int ret = set_next_step_dir(sc, sc->next_step_dir);
        if (ret)
            return ret;
    }
    if (unlikely(sc->next_step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX))
        return queue_append_far(sc);
    if (unlikely(sc->queue_next >= sc->queue_end))
        return queue_append_extend(sc);
    sc->queue_next->clock32 = sc->next_step_clock;
    sc->queue_next++;
    sc->next_step_clock = 0;
    return 0;
}

#define SDS_FILTER_TIME .000750

// Add next step time
int
stepcompress_append(struct stepcompress *sc, int sdir
                    , double print_time, double step_time)
{
    // Calculate step clock
    double offset = print_time - sc->last_step_print_time;
    double rel_sc = (step_time + offset) * sc->mcu_freq;
    uint64_t step_clock = sc->last_step_clock + (uint64_t)rel_sc;
    struct debug_append *d = &sc->dbg_append[sc->dbg_pos];
    *d = (struct debug_append){
        print_time, step_time, offset, rel_sc, sc->mcu_time_offset,
        sc->mcu_freq, sc->last_step_print_time, step_clock,
        sc->last_step_clock, sc->next_step_clock, sdir, sc->next_step_dir, 0
    };
    sc->dbg_pos = (sc->dbg_pos + 1) % DEBUG_APPEND_COUNT;
    if (sc->dbg_count < DEBUG_APPEND_COUNT)
        sc->dbg_count++;
    // tag: 0 pending, 1 direction-filter rollback, 2 flushing previous pending.
    // Flush previous pending step (if any)
    if (sc->next_step_clock) {
        if (unlikely(sdir != sc->next_step_dir)) {
            double diff = (int64_t)(step_clock - sc->next_step_clock);
            if (diff < SDS_FILTER_TIME * sc->mcu_freq) {
                // Rollback last step to avoid rapid step+dir+step
                d->tag = 1;
                sc->next_step_clock = 0;
                sc->next_step_dir = sdir;
                return 0;
            }
        }
        d->tag = 2;
        int ret = queue_append(sc);
        if (ret)
            return ret;
    }
    // Store this step as the next pending step
    sc->next_step_clock = step_clock;
    sc->next_step_dir = sdir;
    return 0;
}

// Commit next pending step (ie, do not allow a rollback)
int
stepcompress_commit(struct stepcompress *sc)
{
    if (sc->next_step_clock)
        return queue_append(sc);
    return 0;
}

// Flush pending steps
int
stepcompress_flush(struct stepcompress *sc, uint64_t move_clock)
{
    if (sc->next_step_clock && move_clock >= sc->next_step_clock) {
        int ret = queue_append(sc);
        if (ret)
            return ret;
    }
    return queue_flush(sc, move_clock);
}

// Reset the internal state of the stepcompress object
int __visible
stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    debug_record_event(sc, "reset", last_step_clock, 0, 0, 0);
    sc->last_step_clock = last_step_clock;
    sc->sdir = -1;
    calc_last_step_print_time(sc);
    return 0;
}

// Set last_position in the stepcompress object
int __visible
stepcompress_set_last_position(struct stepcompress *sc, uint64_t clock
                               , int64_t last_position)
{
    int ret = stepcompress_flush(sc, UINT64_MAX);
    if (ret)
        return ret;
    sc->last_position = last_position;

    // Add a marker to the history list
    struct history_steps *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->first_clock = hs->last_clock = clock;
    hs->start_position = last_position;
    list_add_head(&hs->node, &sc->history_list);
    return 0;
}

// Search history of moves to find a past position at a given clock
int64_t __visible
stepcompress_find_past_position(struct stepcompress *sc, uint64_t clock)
{
    int64_t last_position = sc->last_position;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (clock < hs->first_clock) {
            last_position = hs->start_position;
            continue;
        }
        if (clock >= hs->last_clock)
            return hs->start_position + hs->step_count;
        int32_t interval = hs->interval, add = hs->add;
        int32_t ticks = (int32_t)(clock - hs->first_clock) + interval, offset;
        if (!add) {
            offset = ticks / interval;
        } else {
            // Solve for "count" using quadratic formula
            double a = .5 * add, b = interval - .5 * add, c = -ticks;
            offset = (sqrt(b*b - 4*a*c) - b) / (2. * a);
        }
        if (hs->step_count < 0)
            return hs->start_position - offset;
        return hs->start_position + offset;
    }
    return last_position;
}

// Return history of queue_step commands
int __visible
stepcompress_extract_old(struct stepcompress *sc, struct pull_history_steps *p
                         , int max, uint64_t start_clock, uint64_t end_clock)
{
    int res = 0;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (start_clock >= hs->last_clock || res >= max)
            break;
        if (end_clock <= hs->first_clock)
            continue;
        p->first_clock = hs->first_clock;
        p->last_clock = hs->last_clock;
        p->start_position = hs->start_position;
        p->step_count = hs->step_count;
        p->interval = hs->interval;
        p->add = hs->add;
        p++;
        res++;
    }
    return res;
}

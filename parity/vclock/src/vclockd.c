// vclockd: deterministic scheduler and virtual clock for processes running with libvclock.so.
//
// Exactly one registered thread (the baton holder) runs at a time. When the holder blocks,
// threads are offered the baton in registration order starting after the holder; each one
// re-checks its own wait condition and answers RUNNING or BUSY. When a full round finds
// nobody runnable (repeated once after a short real settle for asynchronous kernel I/O),
// virtual time jumps to the earliest deadline. No deadline left means deadlock.
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include "vclock.h"

#define MAX_THREADS 1024

enum { T_FREE, T_PENDING, T_NEW, T_BLOCKED, T_RUNNING, T_EXEC };

struct thr {
    int state;
    int fd;
    uint64_t id;
    uint32_t pid, tid;
    uint64_t deadline;
    char reason[VC_REASON_LEN];
};

static struct thr threads[MAX_THREADS];
static int nthreads_seen;
static uint64_t next_id = 1;
static uint64_t vnow;
static int holder = -1;
static int last_holder = -1;
static int expect_procs = 1, procs_registered, started;
static FILE *trace;
static uint32_t proc_pids[MAX_THREADS];

static void tracef(const char *fmt, ...) {
    if (!trace) return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(trace, "%llu.%09llu ", (unsigned long long)(vnow / 1000000000ULL),
            (unsigned long long)(vnow % 1000000000ULL));
    vfprintf(trace, fmt, ap);
    fputc('\n', trace);
    va_end(ap);
    fflush(trace);
}

static void event(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
    fflush(stdout);
}

static int read_full(int fd, void *buf, size_t len) {
    char *p = buf;
    while (len) {
        ssize_t n = read(fd, p, len);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return -1;
        p += n; len -= n;
    }
    return 0;
}

static int write_full(int fd, const void *buf, size_t len) {
    const char *p = buf;
    while (len) {
        ssize_t n = write(fd, p, len);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return -1;
        p += n; len -= n;
    }
    return 0;
}

static void send_msg(int idx, uint32_t type, uint64_t a, uint64_t b) {
    struct vc_msg m = { .type = type, .a = a, .b = b };
    if (write_full(threads[idx].fd, &m, sizeof(m)) < 0)
        tracef("send failed to slot %d", idx);
}

static unsigned long long epoch;

// WELCOME carries the slot id, virtual time, wall-clock epoch, and whether the thread
// already holds the baton (an exec'd image taking over its pre-exec slot).
static void send_welcome(int idx, int running) {
    struct vc_msg m = { .type = VC_WELCOME, .pad = (uint32_t)epoch, .a = threads[idx].id,
                        .b = vnow };
    m.reason[0] = running ? 'R' : 'N';
    if (write_full(threads[idx].fd, &m, sizeof(m)) < 0)
        tracef("send failed to slot %d", idx);
}

static int proc_known(uint32_t pid) {
    for (int i = 0; i < procs_registered; i++)
        if (proc_pids[i] == pid) return 1;
    return 0;
}

static void drop_thread(int idx, const char *why) {
    struct thr *t = &threads[idx];
    tracef("drop slot=%d id=%llu pid=%u tid=%u (%s)", idx, (unsigned long long)t->id,
           t->pid, t->tid, why);
    close(t->fd);
    int pid = t->pid, alive = 0;
    t->state = T_FREE;
    t->fd = -1;
    for (int i = 0; i < MAX_THREADS; i++)
        if (threads[i].state != T_FREE && threads[i].pid == (uint32_t)pid) alive = 1;
    if (!alive) event("{\"event\":\"exit\",\"pid\":%d,\"status\":0}", pid);
    if (holder == idx) holder = -1;
}

static int live_threads(void) {
    int n = 0;
    for (int i = 0; i < MAX_THREADS; i++)
        if (threads[i].state != T_FREE) n++;
    return n;
}

// Order of candidates: registration id, starting after the last holder.
static int order_cmp_start;
static int cmp_slots(const void *a, const void *b) {
    const struct thr *x = &threads[*(const int *)a], *y = &threads[*(const int *)b];
    return x->id < y->id ? -1 : x->id > y->id;
}

static int collect_candidates(int *out) {
    int n = 0;
    for (int i = 0; i < MAX_THREADS; i++)
        if (threads[i].state == T_NEW || threads[i].state == T_BLOCKED) out[n++] = i;
    qsort(out, n, sizeof(int), cmp_slots);
    // rotate so the first candidate has id greater than the last holder's id
    uint64_t after = last_holder >= 0 ? threads[last_holder].id : 0;
    (void)order_cmp_start;
    int rot = 0;
    while (rot < n && threads[out[rot]].id <= after) rot++;
    if (rot == n) rot = 0;
    int tmp[MAX_THREADS];
    for (int i = 0; i < n; i++) tmp[i] = out[(rot + i) % n];
    memcpy(out, tmp, n * sizeof(int));
    return n;
}

// Offer the baton to one candidate. Returns 1 if it took it.
static int offer(int idx) {
    struct thr *t = &threads[idx];
    send_msg(idx, VC_TRY, 0, vnow);
    struct vc_msg m;
    if (read_full(t->fd, &m, sizeof(m)) < 0) {
        drop_thread(idx, "eof during try");
        return 0;
    }
    if (m.type == VC_RUNNING) {
        t->state = T_RUNNING;
        holder = idx;
        tracef("run slot=%d id=%llu pid=%u", idx, (unsigned long long)t->id, t->pid);
        return 1;
    }
    if (m.type == VC_BUSY) return 0;
    if (m.type == VC_EXIT) { drop_thread(idx, "exit during try"); return 0; }
    tracef("protocol error slot=%d type=%u", idx, m.type);
    drop_thread(idx, "protocol error");
    return 0;
}

static void deadlock(void) {
    event("{\"event\":\"deadlock\",\"vtime_ns\":%llu,\"threads\":[", (unsigned long long)vnow);
    int first = 1;
    for (int i = 0; i < MAX_THREADS; i++) {
        struct thr *t = &threads[i];
        if (t->state == T_FREE) continue;
        printf("%s{\"pid\":%u,\"tid\":%u,\"reason\":\"%s\"}", first ? "" : ",", t->pid, t->tid,
               t->reason);
        first = 0;
    }
    event("]}");
    for (int i = 0; i < MAX_THREADS; i++)
        if (threads[i].state != T_FREE) kill(threads[i].pid, SIGKILL);
    exit(3);
}

static void schedule(void) {
    if (holder >= 0 || !started) return;
    // an exec'd image inherits the baton; nobody else may run until it registers
    for (int i = 0; i < MAX_THREADS; i++)
        if (threads[i].state == T_EXEC) return;
    for (;;) {
        int cand[MAX_THREADS];
        int empty_rounds = 0;
        while (empty_rounds < 2) {
            int n = collect_candidates(cand);
            if (!n) return;
            for (int i = 0; i < n; i++)
                if (threads[cand[i]].state != T_FREE && offer(cand[i])) return;
            empty_rounds++;
            struct timespec ts = { 0, 1000000 };
            nanosleep(&ts, NULL); // settle asynchronous kernel delivery (ptys, sockets)
        }
        uint64_t next = VC_INF;
        for (int i = 0; i < MAX_THREADS; i++)
            if (threads[i].state == T_BLOCKED && threads[i].deadline < next)
                next = threads[i].deadline;
        if (next == VC_INF) {
            if (!live_threads()) return;
            deadlock();
        }
        if (next > vnow) {
            vnow = next;
            tracef("advance");
        }
    }
}

static void handle_msg(int idx) {
    struct thr *t = &threads[idx];
    struct vc_msg m;
    if (read_full(t->fd, &m, sizeof(m)) < 0) {
        drop_thread(idx, "eof");
        return;
    }
    switch (m.type) {
    case VC_REGISTER: {
        t->pid = (uint32_t)m.a;
        t->tid = (uint32_t)m.b;
        // an exec'd image inherits the slot (and baton) of its pre-exec thread
        for (int i = 0; i < MAX_THREADS; i++) {
            if (threads[i].state == T_EXEC && threads[i].pid == t->pid) {
                t->id = threads[i].id;
                t->state = T_RUNNING;
                holder = idx;
                last_holder = idx;
                threads[i].state = T_FREE;
                tracef("exec-register slot=%d pid=%u", idx, t->pid);
                send_welcome(idx, 1);
                return;
            }
        }
        t->id = next_id++;
        t->state = T_NEW;
        strcpy(t->reason, "new");
        nthreads_seen++;
        if (!proc_known(t->pid)) {
            proc_pids[procs_registered] = t->pid;
            event("{\"event\":\"register\",\"pid\":%u,\"proc\":%d}", t->pid, procs_registered);
            procs_registered++;
            if (procs_registered >= expect_procs) started = 1;
        }
        tracef("register slot=%d id=%llu pid=%u tid=%u", idx, (unsigned long long)t->id,
               t->pid, t->tid);
        send_welcome(idx, 0);
        return;
    }
    case VC_BLOCK:
        if (holder != idx) tracef("block from non-holder slot=%d", idx);
        t->state = T_BLOCKED;
        t->deadline = m.a;
        memcpy(t->reason, m.reason, VC_REASON_LEN);
        t->reason[VC_REASON_LEN - 1] = 0;
        if (holder == idx) holder = -1;
        last_holder = idx;
        return;
    case VC_EXIT:
        last_holder = idx;
        drop_thread(idx, "exit");
        return;
    case VC_EXEC:
        tracef("exec slot=%d pid=%u", idx, t->pid);
        close(t->fd);
        t->fd = -1;
        t->state = T_EXEC;
        if (holder == idx) holder = -1;
        return;
    default:
        tracef("unexpected msg %u from slot %d", m.type, idx);
        drop_thread(idx, "unexpected");
    }
}

int main(int argc, char **argv) {
    const char *sock = NULL;
    static struct option opts[] = {
        { "sock", required_argument, 0, 's' }, { "epoch", required_argument, 0, 'e' },
        { "trace", required_argument, 0, 't' }, { "expect", required_argument, 0, 'x' },
        { 0, 0, 0, 0 } };
    int c;
    while ((c = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        if (c == 's') sock = optarg;
        else if (c == 'e') epoch = strtoull(optarg, NULL, 10);
        else if (c == 't') trace = fopen(optarg, "w");
        else if (c == 'x') expect_procs = atoi(optarg);
        else return 2;
    }
    if (!sock) {
        fprintf(stderr, "usage: vclockd --sock PATH --epoch SECONDS [--trace FILE] [--expect N]\n");
        return 2;
    }
    if (!epoch) {
        fprintf(stderr, "vclockd: --epoch is required\n");
        return 2;
    }
    signal(SIGPIPE, SIG_IGN);
    unlink(sock);
    int lfd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    struct sockaddr_un a = { .sun_family = AF_UNIX };
    strncpy(a.sun_path, sock, sizeof(a.sun_path) - 1);
    if (lfd < 0 || bind(lfd, (struct sockaddr *)&a, sizeof(a)) < 0 || listen(lfd, 128) < 0) {
        perror("vclockd: listen");
        return 1;
    }
    for (int i = 0; i < MAX_THREADS; i++) threads[i].fd = -1;
    for (;;) {
        struct pollfd pfd[MAX_THREADS + 1];
        int map[MAX_THREADS + 1], n = 0;
        pfd[n].fd = lfd; pfd[n].events = POLLIN; map[n++] = -1;
        for (int i = 0; i < MAX_THREADS; i++) {
            if (threads[i].fd < 0 || threads[i].state == T_FREE) continue;
            pfd[n].fd = threads[i].fd; pfd[n].events = POLLIN; map[n++] = i;
        }
        if (poll(pfd, n, -1) < 0) {
            if (errno == EINTR) continue;
            perror("vclockd: poll");
            return 1;
        }
        if (pfd[0].revents & POLLIN) {
            int cfd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
            if (cfd >= 0) {
                int i = 0;
                while (i < MAX_THREADS && threads[i].state != T_FREE) i++;
                if (i == MAX_THREADS) {
                    close(cfd);
                } else {
                    memset(&threads[i], 0, sizeof(threads[i]));
                    threads[i].fd = cfd;
                    threads[i].state = T_PENDING;
                    threads[i].id = UINT64_MAX;
                    // the new thread sends REGISTER right after connecting and waits for WELCOME
                    handle_msg(i);
                }
            }
        }
        for (int k = 1; k < n; k++) {
            int idx = map[k];
            if (threads[idx].state == T_FREE || threads[idx].fd != pfd[k].fd) continue;
            if (pfd[k].revents & (POLLIN | POLLHUP | POLLERR)) handle_msg(idx);
        }
        schedule();
        if (nthreads_seen && !live_threads()) {
            event("{\"event\":\"done\"}");
            unlink(sock);
            return 0;
        }
    }
}

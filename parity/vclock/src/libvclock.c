// libvclock.so: LD_PRELOAD shim that puts every registered thread under vclockd's baton and
// replaces time with the coordinator's virtual clock.
//
// A registered thread runs only while it holds the baton. Every interposed blocking call first
// tries the operation without blocking; if it would block, the thread sends BLOCK (with its
// virtual deadline) and then answers each TRY from vclockd by re-trying the operation without
// blocking: RUNNING when it can proceed, BUSY otherwise. Virtual time only changes while every
// thread is blocked, so the running thread's cached virtual time is always current.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <spawn.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include "vclock.h"

#define EXPORT __attribute__((visibility("default")))
#define NS_PER_S 1000000000ULL

// ---------------------------------------------------------------- real functions

static int (*r_clock_gettime)(clockid_t, struct timespec *);
static int (*r_poll)(struct pollfd *, nfds_t, int);
static int (*r_ppoll)(struct pollfd *, nfds_t, const struct timespec *, const sigset_t *);
static int (*r_select)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
static int (*r_pselect)(int, fd_set *, fd_set *, fd_set *, const struct timespec *,
                        const sigset_t *);
static int (*r_epoll_wait)(int, struct epoll_event *, int, int);
static int (*r_epoll_pwait)(int, struct epoll_event *, int, int, const sigset_t *);
static ssize_t (*r_read)(int, void *, size_t);
static ssize_t (*r_write)(int, const void *, size_t);
static ssize_t (*r_recv)(int, void *, size_t, int);
static ssize_t (*r_recvfrom)(int, void *, size_t, int, struct sockaddr *, socklen_t *);
static ssize_t (*r_send)(int, const void *, size_t, int);
static int (*r_accept)(int, struct sockaddr *, socklen_t *);
static int (*r_accept4)(int, struct sockaddr *, socklen_t *, int);
static int (*r_pthread_create)(pthread_t *, const pthread_attr_t *, void *(*)(void *), void *);
static int (*r_pthread_join)(pthread_t, void **);
static int (*r_pthread_tryjoin_np)(pthread_t, void **);
static void (*r_pthread_exit)(void *) __attribute__((noreturn));
static int (*r_mutex_trylock)(pthread_mutex_t *);
static int (*r_mutex_lock)(pthread_mutex_t *);
static int (*r_mutex_unlock)(pthread_mutex_t *);
static int (*r_mutex_timedlock)(pthread_mutex_t *, const struct timespec *);
static int (*r_cond_init)(pthread_cond_t *, const pthread_condattr_t *);
static int (*r_cond_wait)(pthread_cond_t *, pthread_mutex_t *);
static int (*r_cond_timedwait)(pthread_cond_t *, pthread_mutex_t *, const struct timespec *);
static int (*r_cond_signal)(pthread_cond_t *);
static int (*r_cond_broadcast)(pthread_cond_t *);
static int (*r_sem_wait)(sem_t *);
static int (*r_sem_trywait)(sem_t *);
static int (*r_sem_timedwait)(sem_t *, const struct timespec *);
static pid_t (*r_fork)(void);
static int (*r_execve)(const char *, char *const[], char *const[]);
static int (*r_nanosleep)(const struct timespec *, struct timespec *);

#define RESOLVE(var, name) do { if (!var) var = dlsym(RTLD_NEXT, name); } while (0)
// The condvar entry points exist in two versions; plain dlsym can return the pre-2.3.2 ABI,
// which rejects clock attributes and uses a different layout.
#define RESOLVE_COND(var, name) do { \
        if (!var) var = dlvsym(RTLD_NEXT, name, "GLIBC_2.3.2"); \
        if (!var) var = dlsym(RTLD_NEXT, name); } while (0)

static void resolve_all(void) {
    RESOLVE(r_clock_gettime, "clock_gettime");
    RESOLVE(r_poll, "poll");
    RESOLVE(r_ppoll, "ppoll");
    RESOLVE(r_select, "select");
    RESOLVE(r_pselect, "pselect");
    RESOLVE(r_epoll_wait, "epoll_wait");
    RESOLVE(r_epoll_pwait, "epoll_pwait");
    RESOLVE(r_read, "read");
    RESOLVE(r_write, "write");
    RESOLVE(r_recv, "recv");
    RESOLVE(r_recvfrom, "recvfrom");
    RESOLVE(r_send, "send");
    RESOLVE(r_accept, "accept");
    RESOLVE(r_accept4, "accept4");
    RESOLVE(r_pthread_create, "pthread_create");
    RESOLVE(r_pthread_join, "pthread_join");
    RESOLVE(r_pthread_tryjoin_np, "pthread_tryjoin_np");
    RESOLVE(r_pthread_exit, "pthread_exit");
    RESOLVE(r_mutex_trylock, "pthread_mutex_trylock");
    RESOLVE(r_mutex_lock, "pthread_mutex_lock");
    RESOLVE(r_mutex_unlock, "pthread_mutex_unlock");
    RESOLVE(r_mutex_timedlock, "pthread_mutex_timedlock");
    RESOLVE_COND(r_cond_init, "pthread_cond_init");
    RESOLVE_COND(r_cond_wait, "pthread_cond_wait");
    RESOLVE_COND(r_cond_timedwait, "pthread_cond_timedwait");
    RESOLVE_COND(r_cond_signal, "pthread_cond_signal");
    RESOLVE_COND(r_cond_broadcast, "pthread_cond_broadcast");
    RESOLVE(r_sem_wait, "sem_wait");
    RESOLVE(r_sem_trywait, "sem_trywait");
    RESOLVE(r_sem_timedwait, "sem_timedwait");
    RESOLVE(r_fork, "fork");
    RESOLVE(r_execve, "execve");
    RESOLVE(r_nanosleep, "nanosleep");
}

// ---------------------------------------------------------------- state

static char g_sock[108];
static int g_enabled;
static uint64_t g_vnow;          // virtual monotonic ns, valid while this process runs
static uint64_t g_epoch_ns;      // CLOCK_REALTIME = epoch + virtual monotonic
static __thread int t_fd = -1;   // this thread's connection to vclockd

__attribute__((noreturn)) static void die(const char *msg) {
    syscall(SYS_write, 2, "libvclock: ", 11);
    syscall(SYS_write, 2, msg, strlen(msg));
    syscall(SYS_write, 2, "\n", 1);
    syscall(SYS_exit_group, 99);
    __builtin_unreachable();
}
static void msg_send(int fd, uint32_t type, uint64_t a, const char *reason) {
    struct vc_msg m;
    memset(&m, 0, sizeof(m));
    m.type = type;
    m.a = a;
    if (reason) strncpy(m.reason, reason, VC_REASON_LEN - 1);
    const char *p = (const char *)&m;
    size_t len = sizeof(m);
    while (len) {
        long n = syscall(SYS_write, fd, p, len);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) die("lost connection to vclockd (write)");
        p += n; len -= (size_t)n;
    }
}

static void msg_recv(int fd, struct vc_msg *m) {
    char *p = (char *)m;
    size_t len = sizeof(*m);
    while (len) {
        long n = syscall(SYS_read, fd, p, len);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) die("lost connection to vclockd (read)");
        p += n; len -= (size_t)n;
    }
}

static int registered(void) { return g_enabled && t_fd >= 0; }

// Connect this thread and register it. Returns 1 if vclockd says it already holds the baton.
static int vc_register(void) {
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) die("socket");
    struct sockaddr_un a;
    memset(&a, 0, sizeof(a));
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, g_sock, sizeof(a.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&a, sizeof(a)) != 0) die("connect to vclockd failed");
    struct vc_msg m;
    memset(&m, 0, sizeof(m));
    m.type = VC_REGISTER;
    m.a = (uint64_t)getpid();
    m.b = (uint64_t)syscall(SYS_gettid);
    const char *p = (const char *)&m;
    size_t len = sizeof(m);
    while (len) {
        long n = syscall(SYS_write, fd, p, len);
        if (n <= 0) die("register write");
        p += n; len -= (size_t)n;
    }
    msg_recv(fd, &m);
    if (m.type != VC_WELCOME) die("expected WELCOME");
    g_vnow = m.b;
    g_epoch_ns = (uint64_t)m.pad * NS_PER_S;
    t_fd = fd;
    return m.reason[0] == 'R';
}

typedef int (*check_fn)(void *ctx);

// Wait for TRY messages until check() succeeds; on return this thread holds the baton.
static void wait_turn(check_fn check, void *ctx) {
    for (;;) {
        struct vc_msg m;
        msg_recv(t_fd, &m);
        if (m.type != VC_TRY) die("expected TRY");
        g_vnow = m.b;
        if (!check || check(ctx)) {
            msg_send(t_fd, VC_RUNNING, 0, NULL);
            return;
        }
        msg_send(t_fd, VC_BUSY, 0, NULL);
    }
}

static void block(uint64_t deadline, const char *reason, check_fn check, void *ctx) {
    msg_send(t_fd, VC_BLOCK, deadline, reason);
    wait_turn(check, ctx);
}

static uint64_t ts_ns(const struct timespec *ts) {
    return (uint64_t)ts->tv_sec * NS_PER_S + (uint64_t)ts->tv_nsec;
}

static void ns_ts(uint64_t ns, struct timespec *ts) {
    ts->tv_sec = (time_t)(ns / NS_PER_S);
    ts->tv_nsec = (long)(ns % NS_PER_S);
}

static uint64_t add_sat(uint64_t a, uint64_t b) { return a > VC_INF - b ? VC_INF : a + b; }

// Absolute deadline on clock `clk` -> virtual monotonic deadline.
static uint64_t abs_deadline(clockid_t clk, const struct timespec *abst) {
    uint64_t t = ts_ns(abst);
    if (clk == CLOCK_REALTIME || clk == CLOCK_REALTIME_COARSE)
        return t > g_epoch_ns ? t - g_epoch_ns : 0;
    return t;
}

static uint64_t ms_deadline(int timeout_ms) {
    if (timeout_ms < 0) return VC_INF;
    return add_sat(g_vnow, (uint64_t)timeout_ms * 1000000ULL);
}

static int deadline_reached(uint64_t d) { return d != VC_INF && g_vnow >= d; }

__attribute__((constructor)) static void vc_init(void) {
    resolve_all();
    const char *s = getenv("VCLOCK_SOCK");
    if (!s || !*s) return;
    strncpy(g_sock, s, sizeof(g_sock) - 1);
    g_enabled = 1;
    if (!vc_register()) wait_turn(NULL, NULL);
}

// ---------------------------------------------------------------- time

EXPORT int clock_gettime(clockid_t clk, struct timespec *ts) {
    if (!registered()) {
        resolve_all();
        return r_clock_gettime(clk, ts);
    }
    switch (clk) {
    case CLOCK_REALTIME:
    case CLOCK_REALTIME_COARSE:
        ns_ts(g_epoch_ns + g_vnow, ts);
        return 0;
    default:
        // monotonic, raw, boottime, coarse, and cpu-time clocks all read virtual time
        ns_ts(g_vnow, ts);
        return 0;
    }
}

EXPORT int gettimeofday(struct timeval *restrict tv, struct timezone *restrict tz) {
    (void)tz;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    if (tv) {
        tv->tv_sec = ts.tv_sec;
        tv->tv_usec = ts.tv_nsec / 1000;
    }
    return 0;
}

EXPORT time_t time(time_t *t) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    if (t) *t = ts.tv_sec;
    return ts.tv_sec;
}

struct sleep_ctx { uint64_t deadline; };
static int sleep_check(void *c) { return deadline_reached(((struct sleep_ctx *)c)->deadline); }

static int vsleep_until(uint64_t deadline) {
    struct sleep_ctx c = { deadline };
    if (!deadline_reached(deadline)) block(deadline, "sleep", sleep_check, &c);
    return 0;
}

EXPORT int nanosleep(const struct timespec *req, struct timespec *rem) {
    if (!registered()) {
        resolve_all();
        return r_nanosleep(req, rem);
    }
    if (rem) rem->tv_sec = rem->tv_nsec = 0;
    return vsleep_until(add_sat(g_vnow, ts_ns(req)));
}

EXPORT int clock_nanosleep(clockid_t clk, int flags, const struct timespec *req,
                           struct timespec *rem) {
    if (!registered()) {
        resolve_all();
        return r_nanosleep(req, rem);
    }
    if (rem) rem->tv_sec = rem->tv_nsec = 0;
    if (flags & TIMER_ABSTIME) return vsleep_until(abs_deadline(clk, req));
    return vsleep_until(add_sat(g_vnow, ts_ns(req)));
}

EXPORT int usleep(useconds_t us) {
    struct timespec ts = { us / 1000000, (long)(us % 1000000) * 1000 };
    return nanosleep(&ts, NULL);
}

EXPORT unsigned int sleep(unsigned int s) {
    struct timespec ts = { s, 0 };
    nanosleep(&ts, NULL);
    return 0;
}

// ---------------------------------------------------------------- fd waits

struct poll_ctx { struct pollfd *fds; nfds_t n; int res; uint64_t deadline; };
static int poll_check(void *p) {
    struct poll_ctx *c = p;
    c->res = r_poll(c->fds, c->n, 0);
    return c->res != 0 || deadline_reached(c->deadline);
}

static int vpoll(struct pollfd *fds, nfds_t n, uint64_t deadline) {
    int r = r_poll(fds, n, 0);
    if (r != 0 || deadline == g_vnow) return r;
    struct poll_ctx c = { fds, n, 0, deadline };
    block(deadline, "poll", poll_check, &c);
    return c.res;
}

EXPORT int poll(struct pollfd *fds, nfds_t n, int timeout) {
    resolve_all();
    if (!registered()) return r_poll(fds, n, timeout);
    return vpoll(fds, n, ms_deadline(timeout));
}

EXPORT int ppoll(struct pollfd *fds, nfds_t n, const struct timespec *tmo, const sigset_t *sig) {
    resolve_all();
    if (!registered()) return r_ppoll(fds, n, tmo, sig);
    return vpoll(fds, n, tmo ? add_sat(g_vnow, ts_ns(tmo)) : VC_INF);
}

struct select_ctx {
    int nfds; fd_set *r, *w, *e; fd_set r0, w0, e0; int res; uint64_t deadline;
};
static int select_try(struct select_ctx *c) {
    if (c->r) *c->r = c->r0;
    if (c->w) *c->w = c->w0;
    if (c->e) *c->e = c->e0;
    struct timeval zero = { 0, 0 };
    c->res = r_select(c->nfds, c->r, c->w, c->e, &zero);
    return c->res;
}
static int select_check(void *p) {
    struct select_ctx *c = p;
    return select_try(c) != 0 || deadline_reached(c->deadline);
}

static int vselect(int nfds, fd_set *r, fd_set *w, fd_set *e, uint64_t deadline) {
    struct select_ctx c;
    memset(&c, 0, sizeof(c));
    c.nfds = nfds; c.r = r; c.w = w; c.e = e; c.deadline = deadline;
    if (r) c.r0 = *r;
    if (w) c.w0 = *w;
    if (e) c.e0 = *e;
    if (select_try(&c) != 0 || deadline == g_vnow) return c.res;
    block(deadline, "select", select_check, &c);
    return c.res;
}

EXPORT int select(int nfds, fd_set *r, fd_set *w, fd_set *e, struct timeval *tv) {
    resolve_all();
    if (!registered()) return r_select(nfds, r, w, e, tv);
    uint64_t d = tv ? add_sat(g_vnow, (uint64_t)tv->tv_sec * NS_PER_S
                                          + (uint64_t)tv->tv_usec * 1000ULL) : VC_INF;
    int res = vselect(nfds, r, w, e, d);
    if (tv) tv->tv_sec = tv->tv_usec = 0;
    return res;
}

EXPORT int pselect(int nfds, fd_set *r, fd_set *w, fd_set *e, const struct timespec *tmo,
                   const sigset_t *sig) {
    resolve_all();
    if (!registered()) return r_pselect(nfds, r, w, e, tmo, sig);
    return vselect(nfds, r, w, e, tmo ? add_sat(g_vnow, ts_ns(tmo)) : VC_INF);
}

struct epoll_ctx { int epfd; struct epoll_event *ev; int max; int res; uint64_t deadline; };
static int epoll_check(void *p) {
    struct epoll_ctx *c = p;
    c->res = r_epoll_wait(c->epfd, c->ev, c->max, 0);
    return c->res != 0 || deadline_reached(c->deadline);
}

static int vepoll(int epfd, struct epoll_event *ev, int max, int timeout) {
    int r = r_epoll_wait(epfd, ev, max, 0);
    uint64_t d = ms_deadline(timeout);
    if (r != 0 || d == g_vnow) return r;
    struct epoll_ctx c = { epfd, ev, max, 0, d };
    block(d, "epoll", epoll_check, &c);
    return c.res;
}

EXPORT int epoll_wait(int epfd, struct epoll_event *ev, int max, int timeout) {
    resolve_all();
    if (!registered()) return r_epoll_wait(epfd, ev, max, timeout);
    return vepoll(epfd, ev, max, timeout);
}

EXPORT int epoll_pwait(int epfd, struct epoll_event *ev, int max, int timeout,
                       const sigset_t *sig) {
    resolve_all();
    if (!registered()) return r_epoll_pwait(epfd, ev, max, timeout, sig);
    return vepoll(epfd, ev, max, timeout);
}

// Wait until a blocking fd is ready; nonblocking fds return immediately.
static void wait_fd(int fd, short events) {
    int fl = fcntl(fd, F_GETFL);
    if (fl < 0 || (fl & O_NONBLOCK)) return;
    struct pollfd p = { fd, events, 0 };
    vpoll(&p, 1, VC_INF);
}

EXPORT ssize_t read(int fd, void *buf, size_t n) {
    resolve_all();
    if (registered()) wait_fd(fd, POLLIN);
    return r_read(fd, buf, n);
}

EXPORT ssize_t write(int fd, const void *buf, size_t n) {
    resolve_all();
    if (registered()) wait_fd(fd, POLLOUT);
    return r_write(fd, buf, n);
}

EXPORT ssize_t recv(int fd, void *buf, size_t n, int flags) {
    resolve_all();
    if (registered() && !(flags & MSG_DONTWAIT)) wait_fd(fd, POLLIN);
    return r_recv(fd, buf, n, flags);
}

EXPORT ssize_t recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr *a,
                        socklen_t *al) {
    resolve_all();
    if (registered() && !(flags & MSG_DONTWAIT)) wait_fd(fd, POLLIN);
    return r_recvfrom(fd, buf, n, flags, a, al);
}

EXPORT ssize_t send(int fd, const void *buf, size_t n, int flags) {
    resolve_all();
    if (registered() && !(flags & MSG_DONTWAIT)) wait_fd(fd, POLLOUT);
    return r_send(fd, buf, n, flags);
}

EXPORT int accept(int fd, struct sockaddr *a, socklen_t *al) {
    resolve_all();
    if (registered()) wait_fd(fd, POLLIN);
    return r_accept(fd, a, al);
}

EXPORT int accept4(int fd, struct sockaddr *a, socklen_t *al, int flags) {
    resolve_all();
    if (registered()) wait_fd(fd, POLLIN);
    return r_accept4(fd, a, al, flags);
}

// ---------------------------------------------------------------- mutexes, condvars, semaphores

struct mutex_ctx { pthread_mutex_t *m; uint64_t deadline; int got; };
static int mutex_check(void *p) {
    struct mutex_ctx *c = p;
    c->got = r_mutex_trylock(c->m) == 0;
    return c->got || deadline_reached(c->deadline);
}

EXPORT int pthread_mutex_lock(pthread_mutex_t *m) {
    resolve_all();
    if (!registered()) return r_mutex_lock(m);
    int r = r_mutex_trylock(m);
    if (r != EBUSY) return r;
    struct mutex_ctx c = { m, VC_INF, 0 };
    block(VC_INF, "mutex", mutex_check, &c);
    return 0;
}

EXPORT int pthread_mutex_timedlock(pthread_mutex_t *m, const struct timespec *abst) {
    resolve_all();
    if (!registered()) return r_mutex_timedlock(m, abst);
    int r = r_mutex_trylock(m);
    if (r != EBUSY) return r;
    struct mutex_ctx c = { m, abs_deadline(CLOCK_REALTIME, abst), 0 };
    block(c.deadline, "mutex", mutex_check, &c);
    return c.got ? 0 : ETIMEDOUT;
}

// Per-condvar signal sequence and clock, keyed by address (open addressing, no malloc).
#define COND_SLOTS 8192
struct cond_ent { pthread_cond_t *c; uint64_t seq; clockid_t clk; };
static struct cond_ent cond_tab[COND_SLOTS];
static pthread_mutex_t cond_tab_lock = PTHREAD_MUTEX_INITIALIZER;

static struct cond_ent *cond_lookup(pthread_cond_t *c) {
    uintptr_t h = ((uintptr_t)c >> 3) * 2654435761u;
    r_mutex_lock(&cond_tab_lock);
    for (unsigned i = 0; i < COND_SLOTS; i++) {
        struct cond_ent *e = &cond_tab[(h + i) % COND_SLOTS];
        if (e->c == c || !e->c) {
            if (!e->c) { e->c = c; e->seq = 0; e->clk = CLOCK_REALTIME; }
            r_mutex_unlock(&cond_tab_lock);
            return e;
        }
    }
    r_mutex_unlock(&cond_tab_lock);
    die("condvar table full");
}

EXPORT int pthread_cond_init(pthread_cond_t *c, const pthread_condattr_t *attr) {
    resolve_all();
    int r = r_cond_init(c, attr);
    clockid_t clk = CLOCK_REALTIME;
    if (attr) pthread_condattr_getclock(attr, &clk);
    struct cond_ent *e = cond_lookup(c);
    e->clk = clk;
    return r;
}

EXPORT int pthread_cond_signal(pthread_cond_t *c) {
    resolve_all();
    __atomic_add_fetch(&cond_lookup(c)->seq, 1, __ATOMIC_SEQ_CST);
    return r_cond_signal(c);
}

EXPORT int pthread_cond_broadcast(pthread_cond_t *c) {
    resolve_all();
    __atomic_add_fetch(&cond_lookup(c)->seq, 1, __ATOMIC_SEQ_CST);
    return r_cond_broadcast(c);
}

struct cond_ctx {
    struct cond_ent *e; pthread_mutex_t *m; uint64_t seq0; uint64_t deadline; int woke; int timed_out;
};
static int cond_check(void *p) {
    struct cond_ctx *c = p;
    if (!c->woke && !c->timed_out) {
        if (__atomic_load_n(&c->e->seq, __ATOMIC_SEQ_CST) != c->seq0) c->woke = 1;
        else if (deadline_reached(c->deadline)) c->timed_out = 1;
        else return 0;
    }
    return r_mutex_trylock(c->m) == 0;
}

static int vcond_wait(pthread_cond_t *cv, pthread_mutex_t *m, uint64_t deadline) {
    struct cond_ctx c = { cond_lookup(cv), m, 0, deadline, 0, 0 };
    c.seq0 = __atomic_load_n(&c.e->seq, __ATOMIC_SEQ_CST);
    r_mutex_unlock(m);
    block(deadline, "cond", cond_check, &c);
    return c.timed_out ? ETIMEDOUT : 0;
}

EXPORT int pthread_cond_wait(pthread_cond_t *cv, pthread_mutex_t *m) {
    resolve_all();
    if (!registered()) return r_cond_wait(cv, m);
    return vcond_wait(cv, m, VC_INF);
}

EXPORT int pthread_cond_timedwait(pthread_cond_t *cv, pthread_mutex_t *m,
                                  const struct timespec *abst) {
    resolve_all();
    if (!registered()) return r_cond_timedwait(cv, m, abst);
    return vcond_wait(cv, m, abs_deadline(cond_lookup(cv)->clk, abst));
}

struct sem_ctx { sem_t *s; uint64_t deadline; int got; };
static int sem_check(void *p) {
    struct sem_ctx *c = p;
    c->got = r_sem_trywait(c->s) == 0;
    return c->got || deadline_reached(c->deadline);
}

EXPORT int sem_wait(sem_t *s) {
    resolve_all();
    if (!registered()) return r_sem_wait(s);
    if (r_sem_trywait(s) == 0) return 0;
    struct sem_ctx c = { s, VC_INF, 0 };
    block(VC_INF, "sem", sem_check, &c);
    return 0;
}

EXPORT int sem_timedwait(sem_t *s, const struct timespec *abst) {
    resolve_all();
    if (!registered()) return r_sem_timedwait(s, abst);
    if (r_sem_trywait(s) == 0) return 0;
    struct sem_ctx c = { s, abs_deadline(CLOCK_REALTIME, abst), 0 };
    if (!deadline_reached(c.deadline)) block(c.deadline, "sem", sem_check, &c);
    if (c.got) return 0;
    errno = ETIMEDOUT;
    return -1;
}

// ---------------------------------------------------------------- threads and processes

struct start { void *(*fn)(void *); void *arg; int ack[2]; };

static void *trampoline(void *p) {
    struct start st = *(struct start *)p;
    free(p);
    vc_register();
    char b = 1;
    syscall(SYS_write, st.ack[1], &b, 1);
    syscall(SYS_close, st.ack[1]);
    wait_turn(NULL, NULL);
    void *ret = st.fn(st.arg);
    msg_send(t_fd, VC_EXIT, 0, NULL);
    syscall(SYS_close, t_fd);
    t_fd = -1;
    return ret;
}

EXPORT int pthread_create(pthread_t *th, const pthread_attr_t *attr, void *(*fn)(void *),
                          void *arg) {
    resolve_all();
    if (!registered()) return r_pthread_create(th, attr, fn, arg);
    struct start *st = malloc(sizeof(*st));
    if (!st) return EAGAIN;
    st->fn = fn;
    st->arg = arg;
    if (pipe2(st->ack, O_CLOEXEC) != 0) { free(st); return EAGAIN; }
    int ackr = st->ack[0];
    int r = r_pthread_create(th, attr, trampoline, st);
    if (r != 0) {
        syscall(SYS_close, st->ack[0]);
        syscall(SYS_close, st->ack[1]);
        free(st);
        return r;
    }
    // keep the baton until the child is registered so registration order is deterministic
    char b;
    while (syscall(SYS_read, ackr, &b, 1) < 0 && errno == EINTR) {}
    syscall(SYS_close, ackr);
    return 0;
}

EXPORT void pthread_exit(void *ret) {
    resolve_all();
    if (registered()) {
        msg_send(t_fd, VC_EXIT, 0, NULL);
        syscall(SYS_close, t_fd);
        t_fd = -1;
    }
    r_pthread_exit(ret);
}

struct join_ctx { pthread_t t; void **ret; int res; };
static int join_check(void *p) {
    struct join_ctx *c = p;
    c->res = r_pthread_tryjoin_np(c->t, c->ret);
    return c->res != EBUSY;
}

EXPORT int pthread_join(pthread_t t, void **ret) {
    resolve_all();
    if (!registered()) return r_pthread_join(t, ret);
    struct join_ctx c = { t, ret, 0 };
    if (join_check(&c)) return c.res;
    block(VC_INF, "join", join_check, &c);
    return c.res;
}

EXPORT pid_t fork(void) {
    resolve_all();
    if (!registered()) return r_fork();
    int ack[2];
    if (pipe2(ack, O_CLOEXEC) != 0) return -1;
    pid_t pid = r_fork();
    if (pid == 0) {
        // the child is a new participant; drop the parent's connection first
        syscall(SYS_close, t_fd);
        t_fd = -1;
        syscall(SYS_close, ack[0]);
        vc_register();
        char b = 1;
        syscall(SYS_write, ack[1], &b, 1);
        syscall(SYS_close, ack[1]);
        wait_turn(NULL, NULL);
        return 0;
    }
    syscall(SYS_close, ack[1]);
    if (pid > 0) {
        char b;
        while (syscall(SYS_read, ack[0], &b, 1) < 0 && errno == EINTR) {}
    }
    syscall(SYS_close, ack[0]);
    return pid;
}

EXPORT pid_t vfork(void) { return fork(); }

struct wait_ctx { pid_t pid; int *status; int options; pid_t res; };
static int wait_check(void *p) {
    struct wait_ctx *c = p;
    c->res = (pid_t)syscall(SYS_wait4, c->pid, c->status, c->options | WNOHANG, NULL);
    return c->res != 0;
}

EXPORT pid_t waitpid(pid_t pid, int *status, int options) {
    if (!registered() || (options & WNOHANG))
        return (pid_t)syscall(SYS_wait4, pid, status, options, NULL);
    struct wait_ctx c = { pid, status, options, 0 };
    if (!wait_check(&c)) block(VC_INF, "waitpid", wait_check, &c);
    return c.res;
}

EXPORT pid_t wait(int *status) { return waitpid(-1, status, 0); }

EXPORT pid_t wait4(pid_t pid, int *status, int options, struct rusage *ru) {
    if (ru) memset(ru, 0, sizeof(*ru));
    return waitpid(pid, status, options);
}

EXPORT pid_t wait3(int *status, int options, struct rusage *ru) {
    return wait4(-1, status, options, ru);
}

struct waitid_ctx { idtype_t idt; id_t id; siginfo_t *info; int options; int res; };
static int waitid_check(void *p) {
    struct waitid_ctx *c = p;
    c->info->si_pid = 0;
    c->res = (int)syscall(SYS_waitid, c->idt, c->id, c->info, c->options | WNOHANG, NULL);
    return c->res != 0 || c->info->si_pid != 0;
}

EXPORT int waitid(idtype_t idt, id_t id, siginfo_t *info, int options) {
    if (!registered() || (options & WNOHANG))
        return (int)syscall(SYS_waitid, idt, id, info, options, NULL);
    struct waitid_ctx c = { idt, id, info, options, 0 };
    if (!waitid_check(&c)) block(VC_INF, "waitid", waitid_check, &c);
    return c.res;
}

// glibc's system() and posix_spawn() create children with clone() internally, bypassing the
// fork() interposer; reimplement them on top of the interposed fork/execve/waitpid.
EXPORT int system(const char *cmd) {
    if (!cmd) return 1;
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        extern char **environ;
        char *const argv[] = { "sh", "-c", (char *)cmd, NULL };
        execve("/bin/sh", argv, environ);
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;
    return status;
}

static int vspawn(pid_t *pidp, const char *path, int search, char *const argv[],
                  char *const envp[]) {
    pid_t pid = fork();
    if (pid < 0) return errno;
    if (pid == 0) {
        if (search && !strchr(path, '/')) {
            const char *pathenv = getenv("PATH");
            char buf[4096];
            for (const char *p = pathenv ? pathenv : "/bin:/usr/bin"; ; ) {
                const char *e = strchr(p, ':');
                size_t n = e ? (size_t)(e - p) : strlen(p);
                if (n + strlen(path) + 2 < sizeof(buf)) {
                    memcpy(buf, p, n);
                    buf[n] = '/';
                    strcpy(buf + n + 1, path);
                    execve(buf, argv, envp);
                }
                if (!e) break;
                p = e + 1;
            }
        } else {
            execve(path, argv, envp);
        }
        _exit(127);
    }
    if (pidp) *pidp = pid;
    return 0;
}

EXPORT int posix_spawn(pid_t *pid, const char *path, const posix_spawn_file_actions_t *fa,
                       const posix_spawnattr_t *attr, char *const argv[], char *const envp[]) {
    (void)attr;
    if (fa) return ENOSYS; // file actions are not used by the reference processes
    return vspawn(pid, path, 0, argv, envp);
}

EXPORT int posix_spawnp(pid_t *pid, const char *file, const posix_spawn_file_actions_t *fa,
                        const posix_spawnattr_t *attr, char *const argv[], char *const envp[]) {
    (void)attr;
    if (fa) return ENOSYS;
    return vspawn(pid, file, 1, argv, envp);
}

EXPORT int execve(const char *path, char *const argv[], char *const envp[]) {
    resolve_all();
    if (!registered()) return r_execve(path, argv, envp);
    int has_sock = 0;
    for (char *const *e = envp; e && *e; e++)
        if (!strncmp(*e, "VCLOCK_SOCK=", 12)) has_sock = 1;
    if (!has_sock) {
        // the new image will not participate; it just leaves
        return r_execve(path, argv, envp);
    }
    msg_send(t_fd, VC_EXEC, (uint64_t)getpid(), NULL);
    int fd = t_fd;
    t_fd = -1;
    int r = r_execve(path, argv, envp);
    // exec failed: rejoin, inheriting the slot that VC_EXEC parked
    int save = errno;
    syscall(SYS_close, fd);
    if (vc_register() == 0) wait_turn(NULL, NULL);
    errno = save;
    return r;
}

// glibc's exec* helpers call the internal __execve directly, so route them through execve().
extern char **environ;

EXPORT int execv(const char *path, char *const argv[]) { return execve(path, argv, environ); }

EXPORT int execvpe(const char *file, char *const argv[], char *const envp[]) {
    if (strchr(file, '/')) return execve(file, argv, envp);
    const char *pathenv = getenv("PATH");
    char buf[4096];
    int err = ENOENT;
    for (const char *p = pathenv ? pathenv : "/bin:/usr/bin"; ; ) {
        const char *e = strchr(p, ':');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n + strlen(file) + 2 < sizeof(buf)) {
            memcpy(buf, p, n);
            buf[n] = '/';
            strcpy(buf + n + 1, file);
            execve(buf, argv, envp);
            if (errno != ENOENT) err = errno;
        }
        if (!e) break;
        p = e + 1;
    }
    errno = err;
    return -1;
}

EXPORT int execvp(const char *file, char *const argv[]) { return execvpe(file, argv, environ); }

# Deterministic virtual clock and thread scheduler

`vclockd` coordinates, `libvclock.so` (LD_PRELOAD) interposes.

## Build

```
make x86_64      # build/x86_64/{vclockd,libvclock.so}
make mips CROSS_PREFIX=mipsel-linux-gnu- SYSROOT_LIB=../../pad_sysroot/lib
# uses scripts/build-mips.sh with -mnan=2008 -mfp64 -D_TIME_BITS=32
# like scripts/build-mips-nan2008-klipper.sh
```

## Run

```
parity/vclock/build/x86_64/vclockd --sock /tmp/vc.sock --epoch 1700000000 --trace /tmp/vc.trace &
LD_PRELOAD=parity/vclock/build/x86_64/libvclock.so VCLOCK_SOCK=/tmp/vc.sock <participant> ...
```

Wire (line based, unix stream socket at --sock, shared memory at --sock.shm):
shim -> daemon: `REGISTER pid=P`, `EXIT pid=P status=S`, `DEADLOCK pid=P tid=T reason=R;...`
daemon stdout JSON: `{"event":"register","pid":P,"proc":K}` in registration order,
`{"event":"exit","pid":P,"status":S}`, `{"event":"deadlock",...}` with every
thread wait reason, `{"event":"done"}` when all participants exited.
`--trace FILE` logs every wire line. Wall clock = --epoch + virtual monotonic;
virtual monotonic starts at 0. CLOCK_REALTIME = epoch + monotonic.

## How it works

One global baton in `<sock>.shm` (struct vc_shm in src/vclock.h): exactly one
registered thread runs. A thread gives up the baton only inside an interposed
blocking call: it marks its slot BLOCKED with an absolute virtual deadline
(or INF) and a wait reason, clears the baton, and spins (real 50us nanosleep)
until a signaller flips it REGISTERED or its deadline passes and it wins the
baton back. After each block the blocking loop calls maybe_advance_time: if
no REGISTERED thread exists and some BLOCKED thread has a finite deadline,
after two settle rounds (0.5ms real sleeps; pty/socket delivery can be
asynchronous) virtual time jumps to the earliest deadline. If no REGISTERED
and no deadline, the shim sends DEADLOCK with every thread wait reason and
aborts all participants. Round-robin order is registration order (process
order, then thread creation order); creation is serialized because the
child waits for the baton before running user code.

Deviation from the task text: scheduling lives in the shared-memory file
created by vclockd, not in per-operation socket RPCs. The wire socket carries
only REGISTER/EXIT/DEADLOCK lines. This keeps the hot path fork-safe (no
malloc in the shim, raw syscall(SYS_write) for wire I/O, flock on the shm fd
for short critical sections) and works for MIPS guests under qemu-user
(the shm file is host-visible). Cross-process atomicity uses flock, not a
shared pthread mutex clock.

## Interposed symbols and semantics

Time: clock_gettime (MONOTONIC/MONOTONIC_RAW/BOOTTIME/MONOTONIC_COARSE/
PROCESS_CPUTIME_ID/THREAD_CPUTIME_ID -> vtime; REALTIME -> epoch+vtime;
others passed through), gettimeofday, time (epoch+vtime).
Sleep: nanosleep, clock_nanosleep (TIMER_ABSTIME honoured, REALTIME converted),
usleep, sleep -> block until virtual deadline.
Fd readiness: poll, ppoll, select, pselect, epoll_wait, epoll_pwait -> zero
timeout probe, then block to deadline, then real call with 0 timeout.
Blocking I/O: read, write (O_NONBLOCK passes through), recv, send, recvfrom,
accept, accept4 -> readiness wait then real call.
Threads: pthread_create (reserves slot under baton, child waits for baton),
pthread_join (polls pthread_tryjoin_np with 1ms virtual quanta),
pthread_exit, pthread_mutex_lock (trylock, block, retry), pthread_mutex_trylock
(pass), pthread_mutex_timedlock (virtual deadline, ETIMEDOUT),
pthread_mutex_unlock (wakes one mutex waiter),
pthread_cond_wait / pthread_cond_timedwait / pthread_cond_clockwait
(release real mutex, block on per-cond seq counter, re-lock with trylock),
pthread_cond_signal / pthread_cond_broadcast (bump seq, wake waiters),
pthread_condattr_setclock (pass through),
sem_wait / sem_timedwait / sem_clockwait (trywait, block, retry; timedwait
converts REALTIME abs deadline), sem_trywait (pass), sem_post (wake waiter).
Process: fork, vfork (child re-registers, waits for baton), execve
(deregisters, new image re-registers via constructor), _exit, exit
(deregister + EXIT line).

Python 3.8: GIL cond_timedwait on CLOCK_MONOTONIC condvar and
threading.Lock sem_timedwait with CLOCK_REALTIME deadlines are both handled
by the cond-seq and sem entries above; greenlets never block; reactor
epoll is covered by epoll_wait; multiprocessing fork children register as
new participants.

Real functions via dlsym(RTLD_NEXT), resolved once.

## Tests (write-only; integration owner runs)

```
gcc -std=gnu11 -Wall -Wextra -fsyntax-only -fPIC src/vclockd.c
gcc -std=gnu11 -Wall -Wextra -fsyntax-only -fPIC src/libvclock.c -Isrc
gcc -std=gnu11 -Wall -Wextra -fsyntax-only -fPIC tests/threaded_trace.c
python3 -m py_compile tests/py_determinism.py
sh tests/run.sh   # builds threaded_trace, runs each 20x under vclockd,
                  # asserts byte-identical output + exact elapsed_ms
```

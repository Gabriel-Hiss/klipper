#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int
openpty(int *amaster, int *aslave, char *name, const struct termios *termp,
        const struct winsize *winp)
{
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0)
        return -1;
    if (grantpt(master) || unlockpt(master))
        goto fail;
    char *slave_name = ptsname(master);
    if (!slave_name)
        goto fail;
    int slave = open(slave_name, O_RDWR | O_NOCTTY);
    if (slave < 0)
        goto fail;
    if (termp)
        tcsetattr(slave, TCSAFLUSH, termp);
    if (winp)
        ioctl(slave, TIOCSWINSZ, winp);
    if (name)
        strcpy(name, slave_name);
    *amaster = master;
    *aslave = slave;
    return 0;

fail:
    close(master);
    return -1;
}

int
timer_create(clockid_t clockid, struct sigevent *sevp, timer_t *timerid)
{
    *timerid = (timer_t)1;
    return 0;
}

int
timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value,
              struct itimerspec *old_value)
{
    struct itimerval it;
    memset(&it, 0, sizeof(it));
    struct timespec value = new_value->it_value;
    if (flags & TIMER_ABSTIME) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        value.tv_sec -= now.tv_sec;
        value.tv_nsec -= now.tv_nsec;
        if (value.tv_nsec < 0) {
            value.tv_sec--;
            value.tv_nsec += 1000000000L;
        }
        if (value.tv_sec < 0)
            value = (struct timespec){0, 1};
    }
    it.it_value.tv_sec = value.tv_sec;
    it.it_value.tv_usec = value.tv_nsec / 1000;
    if (!it.it_value.tv_sec && !it.it_value.tv_usec
        && (value.tv_sec || value.tv_nsec))
        it.it_value.tv_usec = 1;
    it.it_interval.tv_sec = new_value->it_interval.tv_sec;
    it.it_interval.tv_usec = new_value->it_interval.tv_nsec / 1000;
    return setitimer(ITIMER_REAL, &it, NULL);
}

int
pthread_create(pthread_t *thread, const pthread_attr_t *attr,
               void *(*start_routine)(void *), void *arg)
{
    errno = ENOSYS;
    return ENOSYS;
}

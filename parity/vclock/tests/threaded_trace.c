// Determinism probe: condvars, mutexes, semaphores, pipes and timed sleeps across threads.
// Every line carries the virtual time, so 20 runs under vclockd must print identical output.
// Build: gcc -O1 -pthread -o threaded_trace tests/threaded_trace.c
#define _GNU_SOURCE
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond;
static sem_t sem;
static int counter, ready;
static int fds[2];

static double now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void *producer(void *arg) {
    (void)arg;
    for (int i = 0; i < 5; i++) {
        struct timespec dl;
        clock_gettime(CLOCK_MONOTONIC, &dl);
        dl.tv_nsec += 10000000;
        if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
        pthread_mutex_lock(&mu);
        int r = 0;
        while (!ready && r == 0) r = pthread_cond_timedwait(&cond, &mu, &dl);
        ready = 0;
        counter++;
        printf("%.6f P %d counter=%d timedout=%d\n", now(), i, counter, r != 0);
        fflush(stdout);
        pthread_mutex_unlock(&mu);
        sem_post(&sem);
    }
    return NULL;
}

static void *signaller(void *arg) {
    (void)arg;
    for (int i = 0; i < 3; i++) {
        usleep(3000);
        pthread_mutex_lock(&mu);
        ready = 1;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mu);
        printf("%.6f S %d\n", now(), i);
        fflush(stdout);
    }
    return NULL;
}

static void *writer(void *arg) {
    (void)arg;
    for (int i = 0; i < 3; i++) {
        struct timespec req = { 0, 7000000 };
        nanosleep(&req, NULL);
        char buf[8];
        int n = snprintf(buf, sizeof(buf), "q%d", i);
        if (write(fds[1], buf, (size_t)n) < 0) perror("write");
    }
    return NULL;
}

static void *reader(void *arg) {
    (void)arg;
    for (int i = 0; i < 3; i++) {
        char rb[8] = { 0 };
        if (read(fds[0], rb, 2) < 0) perror("read");
        printf("%.6f R %d got=%s\n", now(), i, rb);
        fflush(stdout);
    }
    return NULL;
}

int main(void) {
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&cond, &ca);
    sem_init(&sem, 0, 0);
    if (pipe(fds) != 0) return 1;
    pthread_t t[4];
    pthread_create(&t[0], NULL, producer, NULL);
    pthread_create(&t[1], NULL, signaller, NULL);
    pthread_create(&t[2], NULL, writer, NULL);
    pthread_create(&t[3], NULL, reader, NULL);
    for (int i = 0; i < 5; i++) sem_wait(&sem);
    for (int i = 0; i < 4; i++) pthread_join(t[i], NULL);
    double a = now();
    usleep(20000);
    printf("%.6f elapsed_ms=%.3f counter=%d\n", now(), (now() - a) * 1000.0, counter);
    return 0;
}

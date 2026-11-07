// Checks that pthread_cond_init with a CLOCK_MONOTONIC attribute succeeds under the shim.
#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <time.h>

int main(void) {
    pthread_condattr_t a;
    pthread_cond_t c;
    int r1 = pthread_condattr_init(&a);
    int r2 = pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
    int r3 = pthread_cond_init(&c, &a);
    printf("condattr_init=%d setclock=%d cond_init=%d\n", r1, r2, r3);
    return r3 != 0;
}

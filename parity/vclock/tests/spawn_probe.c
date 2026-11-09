// posix_spawn probe: spawns children through posix_spawnp with file actions
// (dup2 of pipe ends onto stdout/stderr, an addopen of /dev/null onto stdin, a
// close) and attributes, the way Rust's std::process::Command does, and prints
// what the children wrote plus each spawn result. Its output under vclock must
// equal its output run natively (glibc is the oracle). PATH starts with a
// missing directory, so every spawn first takes the failed-exec rejoin path.
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

static void drain(const char *label, int fd) {
    char buf[256];
    size_t len = 0;
    for (;;) {
        ssize_t n = read(fd, buf + len, sizeof(buf) - 1 - len);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        len += (size_t)n;
        if (len == sizeof(buf) - 1) break;
    }
    buf[len] = 0;
    printf("%s=[%s]\n", label, buf);
    close(fd);
}

static void spawn_one(const char *prog, char *const argv[]) {
    int out[2], err[2];
    if (pipe2(out, O_CLOEXEC) || pipe2(err, O_CLOEXEC)) {
        printf("pipe failed\n");
        return;
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&fa, out[1], 1);
    posix_spawn_file_actions_adddup2(&fa, err[1], 2);
    posix_spawn_file_actions_addclose(&fa, 9);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    sigset_t set;
    sigemptyset(&set);
    posix_spawnattr_setsigmask(&attr, &set);
    sigaddset(&set, SIGPIPE);
    posix_spawnattr_setsigdefault(&attr, &set);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);
    pid_t pid = 0;
    int ret = posix_spawnp(&pid, prog, &fa, &attr, argv, environ);
    posix_spawnattr_destroy(&attr);
    posix_spawn_file_actions_destroy(&fa);
    close(out[1]);
    close(err[1]);
    printf("spawn %s ret=%s\n", prog, ret ? strerror(ret) : "0");
    drain("stdout", out[0]);
    drain("stderr", err[0]);
    if (!ret) {
        int status = 0;
        waitpid(pid, &status, 0);
        printf("exit=%d\n", WIFEXITED(status) ? WEXITSTATUS(status) : -1);
    }
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("PATH", "/nonexistent-vclock-probe:/usr/bin:/bin", 1);
    char *sh_argv[] = { "sh", "-c", "read x || echo stdin-eof; echo out; echo err >&2; exit 3",
                        NULL };
    spawn_one("sh", sh_argv);
    char *missing_argv[] = { "vclock-no-such-program", NULL };
    spawn_one("vclock-no-such-program", missing_argv);
    for (int i = 0; i < 5; i++) {
        char arg[32];
        snprintf(arg, sizeof(arg), "echo round %d", i);
        char *argv[] = { "sh", "-c", arg, NULL };
        spawn_one("sh", argv);
    }
    return 0;
}

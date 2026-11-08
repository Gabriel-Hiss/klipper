// Symbols Rust's std expects as plain exports but glibc 2.29 only provides through
// libc_nonshared.a (the stat family) on the printer. Each forwards to the exported
// versioned implementation, exactly as glibc's own inline wrappers do.
#define _GNU_SOURCE
#define _LARGEFILE64_SOURCE
#include <sys/stat.h>

extern int __xstat64(int ver, const char *path, struct stat64 *buf);
extern int __fxstat64(int ver, int fd, struct stat64 *buf);
extern int __lxstat64(int ver, const char *path, struct stat64 *buf);
extern int __fxstatat64(int ver, int fd, const char *path, struct stat64 *buf, int flag);

#define STAT_VER_LINUX 3

int stat64(const char *path, struct stat64 *buf) { return __xstat64(STAT_VER_LINUX, path, buf); }
int fstat64(int fd, struct stat64 *buf) { return __fxstat64(STAT_VER_LINUX, fd, buf); }
int lstat64(const char *path, struct stat64 *buf) { return __lxstat64(STAT_VER_LINUX, path, buf); }
int fstatat64(int fd, const char *path, struct stat64 *buf, int flag)
{
    return __fxstatat64(STAT_VER_LINUX, fd, path, buf, flag);
}

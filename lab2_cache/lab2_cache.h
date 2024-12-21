#ifndef LAB2_CACHE_H
#define LAB2_CACHE_H

#include <cstddef>
#include <sys/types.h>

int lab2_open(const char* path, int flags, mode_t mode);
int lab2_close(int fd);
ssize_t lab2_read(int fd, void* buf, size_t count);
ssize_t lab2_write(int fd, const void* buf, size_t count);
off_t lab2_lseek(int fd, off_t offset, int whence);
int lab2_fsync(int fd_to_sync);

// Stats
unsigned long lab2_get_cache_hits();
unsigned long lab2_get_cache_misses();
void lab2_reset_cache_counters();

#endif
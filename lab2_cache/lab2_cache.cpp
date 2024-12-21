#include "lab2_cache.h"
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <list>
#include <vector>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <climits>

#define BLOCK_SIZE 4096
#define MAX_CACHE_SIZE 32

static unsigned long cache_hits = 0;
static unsigned long cache_misses = 0;

struct CacheBlock {
    char* data;
    bool is_dirty;
    bool was_accessed;
    access_hint_t next_access_time;
};

struct FileDescriptor {
    int fd;
    off_t offset;
};

typedef std::pair<int, off_t> CacheKey;

std::map<int, FileDescriptor> fd_table;
std::map<CacheKey, CacheBlock> cache_table;
std::list<CacheKey> cache_queue;

FileDescriptor& get_file_descriptor(int fd);
void free_cache_block() {
    CacheKey victim_key;
    access_hint_t latest_access_time = cache_table.begin()->second.next_access_time;

    for (auto it = cache_table.begin(); it != cache_table.end(); ++it) {
        CacheBlock& block = it->second;

        if (block.next_access_time > latest_access_time) {
            latest_access_time = block.next_access_time;
            victim_key = it->first;
        }
    }

    CacheBlock& victim_block = cache_table[victim_key];
    if (victim_block.is_dirty) {
        const int fd = victim_key.first;
        const off_t block_id = victim_key.second;
        ssize_t ret = pwrite(fd, victim_block.data, BLOCK_SIZE, block_id * BLOCK_SIZE);
        if (ret != BLOCK_SIZE) {
            perror("pwrite");
        }
    }

    free(victim_block.data);
    cache_table.erase(victim_key);
}

char* allocate_aligned_buffer() {
    void* buf = nullptr;
    if (posix_memalign(&buf, BLOCK_SIZE, BLOCK_SIZE) != 0) {
        perror("posix_memalign");
        return nullptr;
    }
    return static_cast<char*>(buf);
}

int lab2_open(const char* path, const int flags, const mode_t mode) {
    const int fd = open(path, flags | O_DIRECT, mode);
    if (fd < 0) {
        perror("open");
        return -1;
    }
    fd_table[fd] = {fd, 0};
    return fd;
}

int lab2_close(const int fd) {
    const auto iterator = fd_table.find(fd);
    if (iterator == fd_table.end()) {
        errno = EBADF;
        return -1;
    }
    lab2_fsync(fd);
    const int result = close(iterator->second.fd);
    fd_table.erase(iterator);
    return result;
}

ssize_t lab2_read(const int fd, void* buf, const size_t count, access_hint_t hint) {
    auto& [found_fd, offset] = get_file_descriptor(fd);
    if (found_fd < 0 || offset < 0) {
        errno = EBADF;
        return -1;
    }
    size_t bytes_read = 0;
    const auto buffer = static_cast<char*>(buf);

    while (bytes_read < count) {
        off_t block_id = offset / BLOCK_SIZE;
        const size_t block_offset = offset % BLOCK_SIZE;
        const size_t bytes_to_read = std::min(BLOCK_SIZE - block_offset, count - bytes_read);

        CacheKey key = {found_fd, block_id};
        auto cache_iterator = cache_table.find(key);
        if (cache_iterator != cache_table.end()) {
            cache_hits++;

            CacheBlock& found_block = cache_iterator->second;
            found_block.was_accessed = true;
            found_block.next_access_time = hint;
            size_t available_bytes = BLOCK_SIZE - block_offset;
            const size_t bytes_from_block = std::min(bytes_to_read, available_bytes);
            std::memcpy(buffer + bytes_read, found_block.data + block_offset, bytes_from_block);
            offset += bytes_from_block;
            bytes_read += bytes_from_block;
        } else {
            cache_misses++;

            if (cache_table.size() >= MAX_CACHE_SIZE) {
                free_cache_block();
            }

            char* aligned_buf = allocate_aligned_buffer();
            const ssize_t ret = pread(found_fd, aligned_buf, BLOCK_SIZE, block_id * BLOCK_SIZE);
            if (ret < 0) {
                perror("pread");
                free(aligned_buf);
                return -1;
            }
            if (ret == 0) {
                free(aligned_buf);
                break;
            }
            const auto valid_data_size = static_cast<size_t>(ret);
            if (ret < BLOCK_SIZE) {
                std::memset(aligned_buf + ret, 0, BLOCK_SIZE - ret);
            }
            const CacheBlock new_block = {aligned_buf, false, true};
            cache_table[key] = new_block;
            cache_queue.push_back(key);

            size_t available_bytes = valid_data_size - block_offset;
            if (available_bytes <= 0) {
                break;
            }
            const size_t bytes_from_block = std::min(bytes_to_read, available_bytes);
            std::memcpy(buffer + bytes_read, aligned_buf + block_offset, bytes_from_block);
            offset += bytes_from_block;
            bytes_read += bytes_from_block;
        }
    }
    return bytes_read;
}

ssize_t lab2_write(const int fd, const void* buf, const size_t count, access_hint_t hint) {
    auto& [found_fd, offset] = get_file_descriptor(fd);
    if (found_fd < 0 || offset < 0) {
        errno = EBADF;
        return -1;
    }
    size_t bytes_written = 0;
    const auto buffer = static_cast<const char*>(buf);

    while (bytes_written < count) {
        off_t block_id = offset / BLOCK_SIZE;
        const size_t block_offset = offset % BLOCK_SIZE;
        const size_t to_write = std::min(BLOCK_SIZE - block_offset, count - bytes_written);

        CacheKey key = {found_fd, block_id};
        auto cache_it = cache_table.find(key);
        CacheBlock* block_ptr = nullptr;

        if (cache_it == cache_table.end()) {
            cache_misses++;

            if (cache_table.size() >= MAX_CACHE_SIZE) {
                free_cache_block();
            }
            char* aligned_buf = allocate_aligned_buffer();
            ssize_t ret = pread(found_fd, aligned_buf, BLOCK_SIZE, block_id * BLOCK_SIZE);
            if (ret < 0) {
                perror("pread");
                free(aligned_buf);
                return -1;
            } else if (ret == 0) {
                std::memset(aligned_buf, 0, BLOCK_SIZE);
            } else if (ret < BLOCK_SIZE) {
                std::memset(aligned_buf + ret, 0, BLOCK_SIZE - ret);
            }
            CacheBlock& block = cache_table[key] = {aligned_buf, false, true, hint};
            cache_queue.push_back(key);
            block_ptr = &block;
        } else {
            cache_hits++;
            block_ptr = &cache_it->second;
            block_ptr->was_accessed = true;
        }
        std::memcpy(block_ptr->data + block_offset, buffer + bytes_written, to_write);
        block_ptr->is_dirty = true;

        offset += to_write;
        bytes_written += to_write;
    }
    return bytes_written;
}

off_t lab2_lseek(const int fd, const off_t offset, const int whence) {
    auto& [found_fd, file_offset] = get_file_descriptor(fd);
    if (found_fd < 0 || file_offset < 0) {
        errno = EBADF;
        return -1;
    }
    if (whence != SEEK_SET) {
        errno = EINVAL;
        return -1;
    }
    if (offset < 0) {
        errno = EINVAL;
        return -1;
    }
    file_offset = offset;
    return file_offset;
}

int lab2_fsync(const int fd_to_sync) {
    const int found_fd = get_file_descriptor(fd_to_sync).fd;
    if (found_fd < 0) {
        errno = EBADF;
        return -1;
    }
    for (auto& [key, block] : cache_table) {
        if (key.first == found_fd && block.is_dirty) {
            ssize_t ret = pwrite(found_fd, block.data, BLOCK_SIZE, key.second * BLOCK_SIZE);
            if (ret != BLOCK_SIZE) {
                perror("pwrite");
                return -1;
            }
            block.is_dirty = false;
        }
    }
    return fsync(found_fd);
}

FileDescriptor& get_file_descriptor(const int fd) {
    const auto iterator = fd_table.find(fd);
    if (iterator == fd_table.end()) {
        static FileDescriptor invalid_fd = {-1, -1};
        return invalid_fd;
    }
    return iterator->second;
}

unsigned long lab2_get_cache_hits() {
    return cache_hits;
}

unsigned long lab2_get_cache_misses() {
    return cache_misses;
}

void lab2_reset_cache_counters() {
    cache_hits = 0;
    cache_misses = 0;
}

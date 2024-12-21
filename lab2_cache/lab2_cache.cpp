#include "lab2_cache.h"
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <list>
#include <vector>
#include <cstring>
#include <iostream>
#include <cerrno>

#define BLOCK_SIZE 4096     // Block size in bytes
#define MAX_CACHE_SIZE 32 // Max blocks in cache
// 4096 * 32 = 128KB

// Global counters for cache hits and misses
static unsigned long cache_hits = 0;
static unsigned long cache_misses = 0;

struct CacheBlock {
    char* data;           // Pointer to block data
    bool is_dirty;        // Dirty flag (modified since read)
    bool was_accessed;    // Reference bit for Second-Chance
};

struct FileDescriptor {
    int fd;
    off_t offset;
};

typedef std::pair<int, off_t> CacheKey; // (file descriptor, block id corresponding to offset)

std::map<int, FileDescriptor> fd_table;     // Table for file descriptors
std::map<CacheKey, CacheBlock> cache_table;       // Table for cache blocks
std::list<CacheKey> cache_queue;            // Queue for Second-Chance

// Helper functions
FileDescriptor& get_file_descriptor(int fd);

/**
 * Frees a single cache block from memory.
 * Retrieves LChecks if a page has was_accessed bit, and if not,
 */
void free_cache_block() {
    while (!cache_queue.empty()) {
        CacheKey key = cache_queue.front();
        cache_queue.pop_front();
        CacheBlock& block = cache_table[key];
        if (block.was_accessed) {
            // Has been referenced, so reset reference bit and move to back
            block.was_accessed = false;
            cache_queue.push_back(key);
        } else {
            if (block.is_dirty) { // Save to disk if dirty before removing
                const int fd = key.first;
                const off_t block_id = key.second;
                ssize_t ret = pwrite(fd, block.data, BLOCK_SIZE, block_id * BLOCK_SIZE);
                if (ret != BLOCK_SIZE) {
                    perror("pwrite");
                }
            }
            // Remove from cache
            free(block.data);
            cache_table.erase(key);
            break;
        }
    }
}

/**
 * Allocates an aligned buffer into memory.
 * @return Pointer to buffer
 */
char* allocate_aligned_buffer() {
    void* buf = nullptr;
    if (posix_memalign(&buf, BLOCK_SIZE, BLOCK_SIZE) != 0) {
        perror("posix_memalign");
        return nullptr;
    }
    return static_cast<char*>(buf);
}

/**
 * Opens a file with direct I/O.
 * @param path Path to file
 * @param flags File flags, same as in open()
 * @param mode File mode, same as in open()
 * @return Integer file descriptor
 */
int lab2_open(const char* path, const int flags, const mode_t mode) {
    const int fd = open(path, flags | O_DIRECT, mode);  // Read and write, and bypass OS cache
    if (fd < 0) {
        perror("open");
        return -1;
    }
    // For simplicity, use actual fd
    fd_table[fd] = {fd, 0};
    return fd;
}

/**
 * Closes a file.
 * @param fd File descriptor
 * @return 0 on success, -1 on failure
 */
int lab2_close(const int fd) {
    const auto iterator = fd_table.find(fd);
    if (iterator == fd_table.end()) {
        errno = EBADF;                           // Bad file descriptor
        return -1;
    }
    lab2_fsync(fd);                              // Sync data before closing
    const int result = close(iterator->second.fd);
    fd_table.erase(iterator);
    return result;
}

/**
 * Reads data from a file into a buffer.
 * @param fd File descriptor
 * @param buf Buffer to read into
 * @param count Number of bytes to read
 * @return Number of bytes read on success, -1 on failure
 */
ssize_t lab2_read(const int fd, void* buf, const size_t count) {
    auto& [found_fd, offset] = get_file_descriptor(fd);
    if (found_fd < 0 || offset < 0) {
        errno = EBADF;
        return -1;
    }
    size_t bytes_read = 0;
    const auto buffer = static_cast<char*>(buf);

    while (bytes_read < count) {
        off_t block_id = offset / BLOCK_SIZE;
        const size_t block_offset = offset % BLOCK_SIZE; // Offset within block
        const size_t bytes_to_read = std::min(BLOCK_SIZE - block_offset, count - bytes_read); // To read from block

        // Check if block is in cache
        CacheKey key = {found_fd, block_id};
        auto cache_iterator = cache_table.find(key);
        if (cache_iterator != cache_table.end()) {
            // HIT: Read from cache
            cache_hits++;

            CacheBlock& found_block = cache_iterator->second;
            found_block.was_accessed = true;
            size_t available_bytes = BLOCK_SIZE - block_offset; // What's available in block
            const size_t bytes_from_block = std::min(bytes_to_read, available_bytes); // Compare what we need with what's available
            std::memcpy(buffer + bytes_read, found_block.data + block_offset, bytes_from_block);
            offset += bytes_from_block;
            bytes_read += bytes_from_block;
        } else {
            // MISS: Read block from disk
            cache_misses++;

            if (cache_table.size() >= MAX_CACHE_SIZE) { // Cache is full, evict a block
                free_cache_block();
            }

            char* aligned_buf = allocate_aligned_buffer();
            const ssize_t ret = pread(found_fd, aligned_buf, BLOCK_SIZE, block_id * BLOCK_SIZE);
            if (ret < 0) {
                // Failed to read
                perror("pread");
                free(aligned_buf);
                return -1;
            }
            if (ret == 0) {
                // EOF reached
                free(aligned_buf);
                break; // Exit the loop as there's no more data to read
            }
            // Partial read or full block read
            const auto valid_data_size = static_cast<size_t>(ret);
            // Fill the rest of the block with zeros if partial read
            if (ret < BLOCK_SIZE) {
                std::memset(aligned_buf + ret, 0, BLOCK_SIZE - ret);
            }
            // Add new block to cache
            const CacheBlock new_block = {aligned_buf, false, true};
            cache_table[key] = new_block;
            cache_queue.push_back(key);

            size_t available_bytes = valid_data_size - block_offset; // What's available in block
            if (available_bytes <= 0) {
                // No valid data available after the offset
                break;
            }
            const size_t bytes_from_block = std::min(bytes_to_read, available_bytes); // Compare what we need with what's available
            std::memcpy(buffer + bytes_read, aligned_buf + block_offset, bytes_from_block);
            offset += bytes_from_block;
            bytes_read += bytes_from_block;
        }
    }
    return bytes_read;
}

/**
 * Writes data from a buffer to a file.
 * @param fd File descriptor
 * @param buf Buffer to write from
 * @param count Number of bytes to write
 * @return Number of bytes written on success, -1 on failure
 */
ssize_t lab2_write(const int fd, const void* buf, const size_t count) {
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

        // Check if block is in cache
        CacheKey key = {found_fd, block_id};
        auto cache_it = cache_table.find(key);
        CacheBlock* block_ptr = nullptr;

        if (cache_it == cache_table.end()) { // MISS
            cache_misses++;

            // If cache is full, evict a block
            if (cache_table.size() >= MAX_CACHE_SIZE) {
                free_cache_block();
            }
            // Read the block from disk to cache
            char* aligned_buf = allocate_aligned_buffer();
            ssize_t ret = pread(found_fd, aligned_buf, BLOCK_SIZE, block_id * BLOCK_SIZE);
            if (ret < 0) {
                // Read failed
                perror("pread");
                free(aligned_buf);
                return -1;
            } else if (ret == 0) {
                // If read fails (e.g., beyond EOF), fill with zeros
                std::memset(aligned_buf, 0, BLOCK_SIZE);
            } else if (ret < BLOCK_SIZE) {
                // Partial read, fill rest with zeros
                std::memset(aligned_buf + ret, 0, BLOCK_SIZE - ret);
            }
            // Add read block to cache
            CacheBlock& block = cache_table[key] = {aligned_buf, false, true};
            cache_queue.push_back(key);
            block_ptr = &block;
        } else { // HIT
            cache_hits++;
            block_ptr = &cache_it->second;
            block_ptr->was_accessed = true;
        }
        // Write to cache
        std::memcpy(block_ptr->data + block_offset, buffer + bytes_written, to_write);
        block_ptr->is_dirty = true;

        offset += to_write;
        bytes_written += to_write;
    }
    return bytes_written;
}

/**
 * Sets file's offset. Only supports SEEK_SET (absolute positioning).
 * @param fd File descriptor
 * @param offset Absolute offset in bytes
 * @param whence Positioning mode (only SEEK_SET supported)
 * @return New file's offset on success, -1 on failure
 */
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

/**
 * Flushes dirty blocks to disk for a file descriptor.
 * @param fd_to_sync File descriptor to sync
 * @return 0 on success, -1 on failure
 */
int lab2_fsync(const int fd_to_sync) {
    const int found_fd = get_file_descriptor(fd_to_sync).fd;
    if (found_fd < 0) {
        errno = EBADF;
        return -1;
    }
    // Write back dirty blocks for this file descriptor
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
    // Ensure all data and metadata are flushed to the physical storage
    return fsync(found_fd);
}

/**
 * Gets the file descriptor struct for a given file descriptor id.
 * @param fd File descriptor
 * @return FileDescriptor struct
 */
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
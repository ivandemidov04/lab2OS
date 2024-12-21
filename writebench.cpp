#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include "../lab2_cache/lab2_cache.h"

const size_t BLOCK_SIZE = 512;
const size_t FILE_SIZE = 256 * 1024 * 1024;

void writebench(const int iterations, const bool use_cache) {
    bool verbose = true;
    const std::vector buffer(BLOCK_SIZE, 'a');
    std::vector<double> durations;
    durations.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        remove("write.txt");

        auto start = std::chrono::high_resolution_clock::now();

        if (use_cache) {
            const int fd = lab2_open("write.txt", O_CREAT | O_RDWR | O_TRUNC, 0644);
            if (fd < 0) {
                std::cerr << "Error opening file for benchmark!" << std::endl;
                return;
            }

            for (size_t written = 0; written < FILE_SIZE; written += BLOCK_SIZE) {
                ssize_t ret = lab2_write(fd, buffer.data(), BLOCK_SIZE, std::chrono::high_resolution_clock::now());
                if (ret != BLOCK_SIZE) {
                    std::cerr << "Error writing to file during benchmark!" << std::endl;
                    lab2_close(fd);
                    return;
                }
            }
            lab2_fsync(fd);
            lab2_close(fd);
        } else {
            const int fd = open("write.txt", O_CREAT | O_RDWR | O_TRUNC, 0644);
            if (fd < 0) {
                std::cerr << "Error opening file for benchmark!" << std::endl;
                return;
            }

            void* aligned_buffer = nullptr;
            if (posix_memalign(&aligned_buffer, 512, BLOCK_SIZE) != 0) {
                std::cerr << "Error allocating aligned buffer!" << std::endl;
                close(fd);
                return;
            }
            std::memset(aligned_buffer, 'a', BLOCK_SIZE);

            for (size_t written = 0; written < FILE_SIZE; written += BLOCK_SIZE) {
                ssize_t ret = write(fd, aligned_buffer, BLOCK_SIZE);
                if (ret != static_cast<ssize_t>(BLOCK_SIZE)) {
                    std::cerr << "Error writing to file during benchmark!" << std::endl;
                    free(aligned_buffer);
                    close(fd);
                    return;
                }
            }
            fsync(fd);
            free(aligned_buffer);
            close(fd);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        durations.push_back(duration.count());
    }

    const double avg_duration = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();

    std::cout << "Average write: " << avg_duration << " seconds\n";
}

int main(int argc, char* argv[]) {
    std::cout << "Starting " << argv[0] << " benchmark\n";

    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <iterations> [--use-cache]\n";
        return 1;
    }
    
    bool use_cache = false;
    if (argc  == 3 && strcmp(argv[2], "--use-cache") == 0) {
        use_cache = true;
    } else if (argc == 3) {
        std::cerr << "Usage: " << argv[0] << " <iterations> [--use-cache]\n";
        return 1;
    }

    writebench(std::stoi(argv[1]), use_cache);

    return 0;
}

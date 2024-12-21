#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include "../lab2_cache/lab2_cache.h"
using namespace std;

const size_t BLOCK_SIZE = 512;

void emasearchbench(const std::string& file_path, const int iterations, const bool use_cache) {
    bool verbose = true;
    std::vector<char> buffer(BLOCK_SIZE);
    std::vector<double> durations;
    durations.reserve(iterations);

    for (int i = 0; i < iterations; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        if (use_cache) {
            const int fd = lab2_open(file_path.c_str(), O_RDONLY, 0);
            if (fd < 0) {
                std::cerr << "Error opening file for benchmark!" << std::endl;
                return;
            }

            ssize_t bytes_read = 0;
            do {
                bytes_read = lab2_read(fd, buffer.data(), BLOCK_SIZE, std::chrono::high_resolution_clock::now());
                if (bytes_read < 0) {
                    std::cerr << "Error reading from file during benchmark!" << std::endl;
                    lab2_close(fd);
                    return;
                }
            } while (bytes_read > 0);

            lab2_close(fd);
        } else {
            const int fd = open(file_path.c_str(), O_RDONLY);
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

            ssize_t bytes_read = 0;
            do {
                bytes_read = read(fd, aligned_buffer, BLOCK_SIZE);
                if (bytes_read < 0) {
                    std::cerr << "Error reading from file during benchmark!" << std::endl;
                    free(aligned_buffer);
                    close(fd);
                    return;
                }
            } while (bytes_read > 0);

            free(aligned_buffer);
            close(fd);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        durations.push_back(duration.count());
    }

    const double avg_duration = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();

    std::cout << "Average read: " << avg_duration << " seconds\n";
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

    emasearchbench("random.txt", std::stoi(argv[1]), use_cache);

    return 0;
}
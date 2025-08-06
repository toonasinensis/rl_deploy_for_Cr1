#pragma once

#include <fcntl.h>     // shm_open
#include <sys/mman.h>  // mmap, munmap
#include <unistd.h>    // close
#include <string>
#include <vector>
#include <cstring>     // strerror
#include <cerrno>
#include <iostream>

class ShmFloatReader {
public:
    ShmFloatReader(const std::string& shm_name, size_t float_count)
        : shm_name_(shm_name), float_count_(float_count), shm_fd_(-1), data_ptr_(nullptr), valid_(false)
    {
        openSharedMemory();
    }

    ~ShmFloatReader() {
        if (data_ptr_) {
            munmap(data_ptr_, float_count_ * sizeof(float));
        }
        if (shm_fd_ != -1) {
            close(shm_fd_);
        }
    }

    std::vector<float> read() const {
        if (!valid_ || !data_ptr_) {
            std::cerr << "Warning: Attempted to read from invalid shared memory: " << shm_name_ << std::endl;
            return std::vector<float>(float_count_, 0.0f);  // 或者返回空 vector
        }
        const float* raw = static_cast<float*>(data_ptr_);
        return std::vector<float>(raw, raw + float_count_);
    }

    bool is_valid() const {
        return valid_;
    }

private:
    std::string shm_name_;
    size_t float_count_;
    int shm_fd_;
    void* data_ptr_;
    bool valid_;

    void openSharedMemory() {
        shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
        if (shm_fd_ == -1) {
            std::cerr << "shm_open failed for " << shm_name_ << ": " << strerror(errno) << std::endl;
            return;
        }

        size_t size = float_count_ * sizeof(float);
        data_ptr_ = mmap(nullptr, size, PROT_READ, MAP_SHARED, shm_fd_, 0);
        if (data_ptr_ == MAP_FAILED) {
            std::cerr << "mmap failed for " << shm_name_ << ": " << strerror(errno) << std::endl;
            close(shm_fd_);
            shm_fd_ = -1;
            return;
        }

        valid_ = true;
    }
};

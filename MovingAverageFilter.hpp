#pragma once

// #include <cstddef>
// #include <stdexcept>
#include <stddef.h>

namespace mtrn3100 {

template <typename T, size_t N>
class MovingAverageFilter {
public:
    void sample(T data) {
        samples[head] = data;
        head = (head + 1) % N;
        if (head == tail) {
            tail = (tail + 1) % N;
        }
    }

    template <typename U = T>
    U average() const {
        if (isEmpty()) {
            return U{}; // Return default value for empty buffer
        }

        U sum = 0;
        size_t count = size();
        for (size_t i = 0; i < count; i++) {
            sum += samples[(tail + i) % N];
        }
        return sum / count;
    }

    bool isEmpty() const { return head == tail; }

    bool isFull() const { return (head + 1) % N == tail; }

    size_t size() const {
        if (isEmpty()) {
            return 0;
        } else if (isFull()) {
            return N;
        } else if (head > tail) {
            return head - tail;
        } else {
            return N - tail + head;
        }
    }

    size_t capacity() const { return N; }

    void clear() {
        head = tail;
    }

private:
    T samples[N];
    size_t head = 0;
    size_t tail = 0;
};

}  // namespace mtrn3100

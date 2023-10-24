namespace mtrn3100 {

const int ROW = 5;
const int COL = 9;
const int MAX_SIZE = ROW * COL + 1; // This assumes that the size will never exceed this

// basic queue
class SimpleQueue {
    int data[MAX_SIZE];
    int frontIdx, backIdx;
public:
    SimpleQueue() : frontIdx(0), backIdx(0) {}

    void push(int val) {
        data[backIdx++] = val;
    }

    int front() {
        return data[frontIdx];
    }

    void pop() {
        frontIdx++;
    }

    bool empty() {
        return frontIdx == backIdx;
    }
};

}  // namespace mtrn3100

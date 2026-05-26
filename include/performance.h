#include <chrono>

class Performance final
{
public:
    Performance();
    ~Performance();

    void Reset();
    void IncreaseFrameCount();
    void PrintEverySecond();

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    TimePoint start_time_;
    uint64_t frame_count_ = 0;

    TimePoint last_print_time_;
    uint64_t last_frame_count_ = 0;
};
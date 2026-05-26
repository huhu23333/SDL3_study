#include <performance.h>
#include <cassert>
#include <stdio.h>

Performance::Performance()
{
    Reset();
}

Performance::~Performance() {}

void Performance::Reset()
{
    start_time_ = Clock::now();
    last_print_time_ = start_time_;
    frame_count_ = 0;
}

void Performance::IncreaseFrameCount()
{
    frame_count_++;
}

void Performance::PrintEverySecond()
{
    assert(start_time_.time_since_epoch().count() > 0);
    assert(last_print_time_ >= start_time_);
    auto now = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_);
    if (duration.count() >= 1000) {
        double elapsed_seconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() /
            1000.0;
        double average_fps = frame_count_ / elapsed_seconds;
        double realtime_fps = (frame_count_ - last_frame_count_) / (duration.count() / 1000.0);
        last_print_time_ = now;
        last_frame_count_ = frame_count_;

        fprintf(stderr, "Performance: FPS(AVR|RT): %.2f|%.2f      \r", average_fps, realtime_fps);
    }
}
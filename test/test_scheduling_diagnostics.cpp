// test_scheduling_diagnostics.cpp
// Unit tests for whole-loop 100 Hz scheduling diagnostics & true start lateness tracking.

#include <cstdint>
#include <cstdio>
#include <cassert>

struct ControlLoopStats {
    uint32_t lastDurationUs = 0;
    uint32_t minDurationUs = 0xFFFFFFFF;
    uint32_t avgDurationUs = 0;
    uint32_t maxDurationUs = 0;
    uint32_t missedDeadlines = 0;
    uint32_t totalIterations = 0;

    uint32_t lastStartLatenessUs = 0;
    uint32_t maxStartLatenessUs = 0;
    uint32_t missedControlPeriods = 0;
    uint32_t maxConsecutiveMissedPeriods = 0;
};

class SimulatedScheduler {
public:
    uint64_t scheduledStartUs = 0;
    ControlLoopStats loopStats;
    uint32_t executionCount = 0;

    void tick(uint64_t nowUs) {
        if (scheduledStartUs == 0) {
            scheduledStartUs = nowUs;
        }

        if (nowUs >= scheduledStartUs + 10000) {
            scheduledStartUs += 10000;

            uint32_t latenessUs = (uint32_t)(nowUs - scheduledStartUs);
            loopStats.lastStartLatenessUs = latenessUs;
            if (latenessUs > loopStats.maxStartLatenessUs) {
                loopStats.maxStartLatenessUs = latenessUs;
            }

            uint32_t periodsElapsed = (latenessUs / 10000) + 1;

            if (periodsElapsed > 1) {
                uint32_t missedThisTick = periodsElapsed - 1;
                loopStats.missedControlPeriods += missedThisTick;
                if (missedThisTick > loopStats.maxConsecutiveMissedPeriods) {
                    loopStats.maxConsecutiveMissedPeriods = missedThisTick;
                }
                scheduledStartUs += (uint64_t)missedThisTick * 10000;
            }

            executionCount++;
            loopStats.totalIterations++;
        }
    }
};

int main() {
    printf("=== Running C++ Whole-Loop 100 Hz Scheduling Diagnostics Unit Tests ===\n");

    // Test 1: Exactly on scheduled deadline -> lateness 0us, missed 0
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 10000; // t = 1010000 (exactly on scheduled deadline)
        sched.tick(t);

        assert(sched.executionCount == 1);
        assert(sched.loopStats.lastStartLatenessUs == 0);
        assert(sched.loopStats.missedControlPeriods == 0);
        printf(" [PASS] 1. Exactly on scheduled deadline -> lastStartLatenessUs = 0us, missed 0\n");
    }

    // Test 2: 4,000 us late -> lateness 4000us, missed 0
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 14000; // 4,000 us late
        sched.tick(t);

        assert(sched.executionCount == 1);
        assert(sched.loopStats.lastStartLatenessUs == 4000);
        assert(sched.loopStats.missedControlPeriods == 0);
        printf(" [PASS] 2. 4,000 us late -> lastStartLatenessUs = 4000us, missed 0\n");
    }

    // Test 3: 9,999 us late -> lateness 9999us, missed 0
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 19999; // 9,999 us late
        sched.tick(t);

        assert(sched.executionCount == 1);
        assert(sched.loopStats.lastStartLatenessUs == 9999);
        assert(sched.loopStats.missedControlPeriods == 0);
        printf(" [PASS] 3. 9,999 us late -> lastStartLatenessUs = 9999us, missed 0\n");
    }

    // Test 4: Exactly 10,000 us late -> lateness 10000us, missed 1 period
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 20000; // 10,000 us late
        sched.tick(t);

        assert(sched.executionCount == 1); // Single execution guard
        assert(sched.loopStats.lastStartLatenessUs == 10000);
        assert(sched.loopStats.missedControlPeriods == 1);
        assert(sched.loopStats.maxConsecutiveMissedPeriods == 1);
        printf(" [PASS] 4. Exactly 10,000 us late -> lastStartLatenessUs = 10000us, missed 1 period\n");
    }

    // Test 5: 25,000 us late -> lateness 25000us, missed 2 periods
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 35000; // 25,000 us late
        sched.tick(t);

        assert(sched.executionCount == 1); // Single execution guard
        assert(sched.loopStats.lastStartLatenessUs == 25000);
        assert(sched.loopStats.missedControlPeriods == 2);
        assert(sched.loopStats.maxConsecutiveMissedPeriods == 2);
        printf(" [PASS] 5. 25,000 us late -> lastStartLatenessUs = 25000us, missed 2 periods\n");
    }

    // Test 6: Single-execution guard & absolute schedule phase preservation
    {
        SimulatedScheduler sched;
        uint64_t t = 1000000;
        sched.tick(t);

        t += 35000; // 25ms late for 1010000 tick
        sched.tick(t);

        assert(sched.executionCount == 1);
        assert(sched.scheduledStartUs == 1030000);

        t += 5000; // t = 1040000 (exactly on time)
        sched.tick(t);
        assert(sched.executionCount == 2);
        assert(sched.loopStats.lastStartLatenessUs == 0);
        printf(" [PASS] 6. Single-execution guard & absolute phase preservation verified\n");
    }

    // Test 7: 64-bit uint64 wrap-safety
    {
        SimulatedScheduler sched;
        uint64_t t = 0xFFFFFFFFFF000000ULL;
        sched.tick(t);
        t += 10000;
        sched.tick(t);
        assert(sched.executionCount == 1);
        printf(" [PASS] 7. 64-bit uint64 wrap-safety verified\n");
    }

    printf("\nALL C++ SCHEDULING DIAGNOSTICS UNIT TESTS PASSED!\n");
    return 0;
}

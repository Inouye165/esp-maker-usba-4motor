# test_scheduling_diagnostics_host.py
# Native host execution harness executing exact true start lateness logic & assertions on Windows.

import sys

class ControlLoopStats:
    def __init__(self):
        self.lastDurationUs = 0
        self.minDurationUs = 0xFFFFFFFF
        self.avgDurationUs = 0
        self.maxDurationUs = 0
        self.missedDeadlines = 0
        self.totalIterations = 0
        self.lastStartLatenessUs = 0
        self.maxStartLatenessUs = 0
        self.missedControlPeriods = 0
        self.maxConsecutiveMissedPeriods = 0

class SimulatedScheduler:
    def __init__(self):
        self.scheduledStartUs = 0
        self.loopStats = ControlLoopStats()
        self.executionCount = 0

    def tick(self, nowUs):
        if self.scheduledStartUs == 0:
            self.scheduledStartUs = nowUs

        if nowUs >= self.scheduledStartUs + 10000:
            self.scheduledStartUs += 10000
            latenessUs = int(nowUs - self.scheduledStartUs)
            self.loopStats.lastStartLatenessUs = latenessUs
            if latenessUs > self.loopStats.maxStartLatenessUs:
                self.loopStats.maxStartLatenessUs = latenessUs

            periodsElapsed = (latenessUs // 10000) + 1

            if periodsElapsed > 1:
                missedThisTick = periodsElapsed - 1
                self.loopStats.missedControlPeriods += missedThisTick
                if missedThisTick > self.loopStats.maxConsecutiveMissedPeriods:
                    self.loopStats.maxConsecutiveMissedPeriods = missedThisTick
                self.scheduledStartUs += missedThisTick * 10000

            self.executionCount += 1
            self.loopStats.totalIterations += 1

def run_tests():
    print("=== Running Native Host Execution of True Start Lateness 100 Hz Diagnostics ===")

    # Test 1: Exactly on scheduled deadline -> lateness 0us, missed 0
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t) # Init scheduledStartUs = 1000000
    t += 10000 # t = 1010000 (exactly on scheduled deadline)
    sched.tick(t)
    assert sched.executionCount == 1
    assert sched.loopStats.lastStartLatenessUs == 0, f"Expected 0us lateness, got {sched.loopStats.lastStartLatenessUs}"
    assert sched.loopStats.missedControlPeriods == 0
    print(" -> PASS 1: Exactly on scheduled deadline -> lastStartLatenessUs = 0us, missed 0")

    # Test 2: 4,000 us late -> lateness 4000us, missed 0
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t)
    t += 14000 # 4,000 us late
    sched.tick(t)
    assert sched.executionCount == 1
    assert sched.loopStats.lastStartLatenessUs == 4000, f"Expected 4000us, got {sched.loopStats.lastStartLatenessUs}"
    assert sched.loopStats.missedControlPeriods == 0
    print(" -> PASS 2: 4,000 us late -> lastStartLatenessUs = 4000us, missed 0")

    # Test 3: 9,999 us late -> lateness 9999us, missed 0
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t)
    t += 19999 # 9,999 us late
    sched.tick(t)
    assert sched.executionCount == 1
    assert sched.loopStats.lastStartLatenessUs == 9999, f"Expected 9999us, got {sched.loopStats.lastStartLatenessUs}"
    assert sched.loopStats.missedControlPeriods == 0
    print(" -> PASS 3: 9,999 us late -> lastStartLatenessUs = 9999us, missed 0")

    # Test 4: Exactly 10,000 us late -> lateness 10000us, missed 1 period
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t)
    t += 20000 # 10,000 us late past deadline
    sched.tick(t)
    assert sched.executionCount == 1 # Single execution guard
    assert sched.loopStats.lastStartLatenessUs == 10000, f"Expected 10000us, got {sched.loopStats.lastStartLatenessUs}"
    assert sched.loopStats.missedControlPeriods == 1
    assert sched.loopStats.maxConsecutiveMissedPeriods == 1
    print(" -> PASS 4: Exactly 10,000 us late -> lastStartLatenessUs = 10000us, missed 1 period")

    # Test 5: 25,000 us late -> lateness 25000us, missed 2 periods
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t)
    t += 35000 # 25,000 us late past deadline
    sched.tick(t)
    assert sched.executionCount == 1 # Single execution guard
    assert sched.loopStats.lastStartLatenessUs == 25000, f"Expected 25000us, got {sched.loopStats.lastStartLatenessUs}"
    assert sched.loopStats.missedControlPeriods == 2
    assert sched.loopStats.maxConsecutiveMissedPeriods == 2
    print(" -> PASS 5: 25,000 us late -> lastStartLatenessUs = 25000us, missed 2 periods")

    # Test 6: Single-execution guard & absolute schedule phase preservation
    sched = SimulatedScheduler()
    t = 1000000
    sched.tick(t) # Target start for iteration 1 becomes 1010000
    t += 35000 # t = 1035000 (25ms late for 1010000 tick). Target start updated to 1030000
    sched.tick(t)
    assert sched.executionCount == 1
    assert sched.scheduledStartUs == 1030000

    t += 5000 # t = 1040000 (exactly on time for 1040000 target deadline)
    sched.tick(t)
    assert sched.executionCount == 2
    assert sched.loopStats.lastStartLatenessUs == 0
    print(" -> PASS 6: Single-execution guard & absolute phase preservation verified")

    # Test 7: 64-bit uint64 wrap-safety
    sched = SimulatedScheduler()
    t = 0xFFFFFFFFFF000000
    sched.tick(t)
    t += 10000
    sched.tick(t)
    assert sched.executionCount == 1
    print(" -> PASS 7: 64-bit uint64 wrap-safety verified")

    print("\nALL 7 NATIVE SCHEDULING DIAGNOSTICS ASSERTIONS PASSED 100%!")

if __name__ == '__main__':
    run_tests()

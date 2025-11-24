#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Include the tracking functionality from tests_main.cpp
extern void track_individual_test_failure(const char* test_name);

// Macro to track individual test execution and failures  
#define TRACK_TEST(test_name) do { \
    extern uint32_t Unity_TestFailures; \
    uint32_t failures_before = Unity_TestFailures; \
    RUN_TEST(test_name); \
    if (Unity_TestFailures > failures_before) { \
        track_individual_test_failure(#test_name); \
    } \
} while(0)

#ifdef __cplusplus
}
#endif
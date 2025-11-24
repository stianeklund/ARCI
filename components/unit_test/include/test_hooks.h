// Lightweight hooks to capture failing test names across suites
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Registration API implemented in tests_main.cpp
void register_failing_test(const char* test_name);

// Override Unity's RUN_TEST to track failures per test
#ifdef RUN_TEST
#undef RUN_TEST
#endif

#define RUN_TEST(TestFunc)                                                                     \
    do {                                                                                       \
        unsigned int _failures_before = Unity.TestFailures;                                    \
        UnityDefaultTestRun(TestFunc, #TestFunc, __LINE__);                                    \
        if (Unity.TestFailures > _failures_before) {                                           \
            register_failing_test(#TestFunc);                                                  \
        }                                                                                      \
    } while (0)

#ifdef __cplusplus
}
#endif


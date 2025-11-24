#ifndef TESTS_MAIN_H
#define TESTS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations for test runners
void run_button_handler_tests(void);

// Main test function to call from app_main
void run_all_tests(void);

#ifdef __cplusplus
}
#endif

#endif // TESTS_MAIN_H
#ifndef UNITY_H
#define UNITY_H

#include <stdio.h>

extern int UnityFailCount;

#define UNITY_BEGIN() do { UnityFailCount = 0; } while (0)
#define UNITY_END() (UnityFailCount)
#define RUN_TEST(func_) do { setUp(); func_(); tearDown(); } while (0)

#define TEST_ASSERT_EQUAL_INT(expected_, actual_) \
    do { if ((expected_) != (actual_)) { ++UnityFailCount; printf("Assertion failed: %s != %s\n", #expected_, #actual_); } } while (0)

#define TEST_ASSERT_TRUE(condition_) \
    do { if (!(condition_)) { ++UnityFailCount; printf("Assertion failed: %s is false\n", #condition_); } } while (0)

#endif /* UNITY_H */

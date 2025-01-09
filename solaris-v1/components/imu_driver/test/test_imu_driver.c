#include "unity.h"
#include "imu_driver.h"

TEST_CASE("Sum of positive numbers", "[imu_driver]"){
    TEST_ASSERT_EQUAL(5, sum(2,3));
}
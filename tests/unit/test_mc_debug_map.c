#include "unity.h"
#include "mc_debug.h"
#include <string.h>
#include <math.h>

static mc_f32_t g_val;
static const mc_debug_var_t g_var_table_2[] = {
    MC_DEBUG_VAR_RO("speed", mc_f32_t, &g_val),
    MC_DEBUG_VAR_RO("angle", mc_f32_t, &g_val),
};

extern void mc_debug_map_set_table(const mc_debug_var_t *table, uint8_t count);
extern void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                                  uint8_t *buf, uint16_t *buf_len);

static void init_map(void)
{
    mc_debug_map_set_table(g_var_table_2, 2U);
}

void test_mc_debug_map_collect_one_var(void)
{
    init_map();
    g_val = 1234.5F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x01U, 1000U, buf, &len);

    TEST_ASSERT_TRUE(len > 4U);
    TEST_ASSERT_EQUAL_INT(1U, buf[4]);
    TEST_ASSERT_EQUAL_INT(0U, buf[5]);
    TEST_ASSERT_EQUAL_INT(MC_DEBUG_VAR_TYPE_FLOAT32, buf[6]);
    TEST_ASSERT_EQUAL_INT(4U, buf[7]);
    mc_f32_t result;
    memcpy(&result, &buf[8], sizeof(result));
    TEST_ASSERT_TRUE(fabsf(result - 1234.5F) < 0.01F);
}

void test_mc_debug_map_collect_two_vars(void)
{
    init_map();
    g_val = 99.9F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x03U, 2000U, buf, &len);

    TEST_ASSERT_TRUE(len > 11U);
    TEST_ASSERT_EQUAL_INT(2U, buf[4]);
}

void test_mc_debug_map_inactive_mask_skips(void)
{
    init_map();
    g_val = 1.0F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x00U, 0U, buf, &len);
    TEST_ASSERT_EQUAL_INT(0U, buf[4]);
}

void test_mc_debug_map_slot_out_of_range_collects_zero(void)
{
    init_map();
    g_val = 1.0F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x04U, 0U, buf, &len);
    TEST_ASSERT_EQUAL_INT(0U, buf[4]);
}

# Debug & Calibration Utility — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add compile-time configurable debug/calibration subsystem with FreeMASTER protocol over UART/CAN/LIN.

**Architecture:** Three-layer module (variable map → protocol engine → transport) integrated via `mc_fast_step()` poll hook. Compile-time macros control depth; platform hooks are optional and NULL-safe.

**Tech Stack:** C99, Unity test framework, FreeMASTER binary protocol, no third-party deps

---

### Task 1: Add macro switches to mc_cfg.h and CMakeLists.txt

**Files:**
- Modify: `include/mc_cfg.h`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add debug macros to mc_cfg.h**

Append after the existing `MC_CFG_ENABLE_SENSORLESS` line:

```c
/** @brief Enable debug/calibration subsystem (FreeMASTER, UART/CAN/LIN) */
#define MC_CFG_ENABLE_DEBUG            (0U)
/** @brief Enable FreeMASTER protocol engine */
#define MC_CFG_ENABLE_DEBUG_FM         (1U)
/** @brief Enable UART transport for debug */
#define MC_CFG_ENABLE_DEBUG_UART       (1U)
/** @brief Enable CAN transport for debug (requires CAN peripheral) */
#define MC_CFG_ENABLE_DEBUG_CAN        (0U)
/** @brief Enable LIN transport for debug (requires LIN peripheral) */
#define MC_CFG_ENABLE_DEBUG_LIN        (0U)
```

- [ ] **Step 2: Add debug sources to CMakeLists.txt**

After the line `src/estimator/mc_sensor_smo.c`, add:

```cmake
    src/debug/mc_debug_map.c
    src/debug/mc_debug_fm.c
    src/debug/mc_debug_transp.c
```

And after `tests/unit/test_mc_dtc.c`, add:

```cmake
        tests/unit/test_mc_debug_map.c
        tests/unit/test_mc_debug_fm.c
        tests/unit/test_mc_debug_transp.c
```

- [ ] **Step 3: Add compile definition for debug enable in CMake**

In the `target_compile_definitions(motor_control PUBLIC ...)` block, add:

```cmake
    MC_CFG_ENABLE_DEBUG=1
```

(So that debug code is compiled in for testing; set to 0 for production builds.)

- [ ] **Step 4: Build to verify new files are listed (will fail — files don't exist yet)**

```bash
cmake --build build 2>&1
```
Expected: linker errors about missing object files `mc_debug_map.obj` etc.

- [ ] **Step 5: Commit**

```bash
git add include/mc_cfg.h CMakeLists.txt
git commit -m "build: add debug/calibration macro switches and CMake sources"
```

---

### Task 2: Create mc_debug.h — public types

**Files:**
- Create: `include/mc_debug.h`

- [ ] **Step 1: Write the header**

```c
#ifndef MC_DEBUG_H
#define MC_DEBUG_H

/** @file mc_debug.h @brief Debug and calibration subsystem types */

#include "mc_types.h"

#define MC_DEBUG_VAR_TYPE_FLOAT32   (0U)
#define MC_DEBUG_VAR_TYPE_INT16     (1U)
#define MC_DEBUG_VAR_TYPE_UINT16    (2U)
#define MC_DEBUG_VAR_TYPE_UINT8     (3U)
#define MC_DEBUG_VAR_TYPE_UINT32    (4U)
#define MC_DEBUG_VAR_TYPE_INT32     (5U)

#define MC_DEBUG_FLAG_READABLE       (0x01U)
#define MC_DEBUG_FLAG_WRITABLE       (0x02U)
#define MC_DEBUG_FLAG_RECORDER_ONLY  (0x04U)

#define MC_DEBUG_FRAME_MAGIC_0       (0xDAU)
#define MC_DEBUG_FRAME_MAGIC_1       (0x1AU)
#define MC_DEBUG_HEADER_SIZE         (4U)
#define MC_DEBUG_MAX_VARS_PER_FRAME  (16U)

typedef struct
{
    const char *name;
    void       *addr;
    uint8_t     type;
    uint8_t     size_bytes;
    uint8_t     flags;
} mc_debug_var_t;

typedef enum
{
    MC_DEBUG_TRANSP_NONE = 0,
    MC_DEBUG_TRANSP_UART,
    MC_DEBUG_TRANSP_CAN,
    MC_DEBUG_TRANSP_LIN
} mc_debug_transp_type_t;

typedef struct
{
    mc_debug_transp_type_t type;
    mc_bool_t initialized;
    uint8_t  rx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t rx_len;
    uint8_t  tx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t tx_len;
    uint16_t tx_sent;
    void   (*tx_flush)(const uint8_t *data, uint16_t len);
} mc_debug_transp_t;

typedef struct
{
    mc_bool_t      scope_active;
    uint32_t       scope_period_us;
    uint32_t       scope_last_us;
    mc_bool_t      recorder_active;
    uint32_t       recorder_frame_count;
    uint32_t       recorder_total_frames;
    uint32_t       active_var_mask;
    uint32_t       active_var_count;
    mc_debug_transp_t transp;
} mc_debug_t;

#define MC_DEBUG_VAR_RO(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE }

#define MC_DEBUG_VAR_RW(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE }

#define MC_DEBUG_TYPE_ID(ctype_) \
    ((sizeof(ctype_) == 4U && (ctype_)(-1) < 0) ? MC_DEBUG_VAR_TYPE_INT32 : \
     (sizeof(ctype_) == 4U) ? MC_DEBUG_VAR_TYPE_UINT32 : \
     (sizeof(ctype_) == 2U && (ctype_)(-1) < 0) ? MC_DEBUG_VAR_TYPE_INT16 : \
     (sizeof(ctype_) == 2U) ? MC_DEBUG_VAR_TYPE_UINT16 : \
     (sizeof(ctype_) == 1U && (ctype_)(-1) < 0) ? MC_DEBUG_VAR_TYPE_UINT8 : \
     (sizeof(ctype_) == sizeof(mc_f32_t)) ? MC_DEBUG_VAR_TYPE_FLOAT32 : 0U)

#endif /* MC_DEBUG_H */
```

- [ ] **Step 2: Build to verify header compiles**

```bash
cmake --build build 2>&1
```
Expected: compiles cleanly.

- [ ] **Step 3: Commit**

```bash
git add include/mc_debug.h
git commit -m "feat: add mc_debug.h public types for debug subsystem"
```

---

### Task 3: Create mc_port_debug.h — platform hooks

**Files:**
- Create: `include/mc_port_debug.h`

- [ ] **Step 1: Write the header**

```c
#ifndef MC_PORT_DEBUG_H
#define MC_PORT_DEBUG_H

/** @file mc_port_debug.h @brief Platform hooks for debug communication */

#include <stdint.h>

typedef struct
{
    void (*uart_tx)(const uint8_t *data, uint16_t len);
    void (*can_tx)(uint32_t id, const uint8_t *data, uint8_t len);
    void (*lin_tx)(const uint8_t *data, uint16_t len);
    uint16_t (*uart_rx_available)(void);
    uint8_t  (*uart_rx_read)(void);
} mc_debug_port_t;

#endif /* MC_PORT_DEBUG_H */
```

- [ ] **Step 2: Commit**

```bash
git add include/mc_port_debug.h
git commit -m "feat: add mc_port_debug.h platform hooks for debug transport"
```

---

### Task 4: mc_debug_map.c — variable map manager (TDD)

**Files:**
- Create: `src/debug/mc_debug_map.c`
- Create: `tests/unit/test_mc_debug_map.c`

- [ ] **Step 1: Write the failing test — register and collect a float32 variable**

Create `tests/unit/test_mc_debug_map.c`:

```c
#include "unity.h"
#include "mc_debug.h"
#include <string.h>
#include <math.h>

static mc_f32_t g_test_val;
static const mc_debug_var_t g_test_table_2slot[] = {
    MC_DEBUG_VAR_RO("speed_rpm", mc_f32_t, &g_test_val),
    MC_DEBUG_VAR_RO("angle_rad", mc_f32_t, &g_test_val),
};

extern const mc_debug_var_t *mc_debug_get_var_table(void);
extern uint8_t mc_debug_get_var_count(void);
extern void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                                  uint8_t *buf, uint16_t *buf_len);

void test_mc_debug_map_register_and_collect_one_var(void)
{
    g_test_val = 1234.5F;
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

void test_mc_debug_map_register_and_collect_two_vars(void)
{
    g_test_val = 99.9F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x03U, 2000U, buf, &len);

    TEST_ASSERT_TRUE(len > 11U);
    TEST_ASSERT_EQUAL_INT(2U, buf[4]);
}

void test_mc_debug_map_inactive_mask_skips_var(void)
{
    g_test_val = 1.0F;
    uint8_t buf[128];
    uint16_t len = 0U;
    mc_debug_map_collect(0x00U, 0U, buf, &len);
    TEST_ASSERT_EQUAL_INT(0U, buf[4]);
}

void test_mc_debug_map_var_table_count(void)
{
    TEST_ASSERT_EQUAL_INT(2U, mc_debug_get_var_count());
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: FAIL — `mc_debug_map_collect` and `mc_debug_get_var_count` undefined

- [ ] **Step 3: Implement mc_debug_map.c**

Create `src/debug/mc_debug_map.c`:

```c
/** @file mc_debug_map.c @brief Variable map manager for debug subsystem */

#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

static const mc_debug_var_t g_debug_var_table[] = {
    MC_DEBUG_VAR_RO("speed_rpm", mc_f32_t, &g_test_val),
    MC_DEBUG_VAR_RO("angle_rad", mc_f32_t, &g_test_val),
};

#define MC_DEBUG_VAR_COUNT ((uint8_t)(sizeof(g_debug_var_table) / sizeof(g_debug_var_table[0])))

const mc_debug_var_t *mc_debug_get_var_table(void)
{
    return g_debug_var_table;
}

uint8_t mc_debug_get_var_count(void)
{
    return MC_DEBUG_VAR_COUNT;
}

void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                           uint8_t *buf, uint16_t *buf_len)
{
    uint8_t var_count = 0U;
    uint16_t pos = MC_DEBUG_HEADER_SIZE;
    uint8_t i;

    memcpy(&buf[0], &timestamp_us, sizeof(uint32_t));
    buf[4] = 0U;

    for (i = 0U; i < MC_DEBUG_VAR_COUNT && i < 32U; i++)
    {
        if ((active_mask & (1UL << i)) == 0UL)
        {
            continue;
        }

        const mc_debug_var_t *v = &g_debug_var_table[i];
        buf[pos++] = i;
        buf[pos++] = v->type;
        buf[pos++] = v->size_bytes;
        memcpy(&buf[pos], v->addr, v->size_bytes);
        pos += v->size_bytes;
        var_count++;
    }

    buf[4] = var_count;
    *buf_len = pos;
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: PASS (all existing tests + 4 new map tests)

- [ ] **Step 5: Commit**

```bash
git add src/debug/mc_debug_map.c tests/unit/test_mc_debug_map.c tests/unit/test_mc_api.c CMakeLists.txt
git commit -m "feat: add mc_debug_map variable map manager with collection"
```

---

### Task 5: mc_debug_transp.c — transport abstraction (TDD)

**Files:**
- Create: `src/debug/mc_debug_transp.c`
- Create: `tests/unit/test_mc_debug_transp.c`

- [ ] **Step 1: Write the failing test — UART frame detection**

Create `tests/unit/test_mc_debug_transp.c`:

```c
#include "unity.h"
#include "mc_debug.h"
#include <string.h>

extern void mc_debug_transp_init(mc_debug_transp_t *t, mc_debug_transp_type_t type,
                                  void (*tx_flush)(const uint8_t *data, uint16_t len));
extern mc_bool_t mc_debug_transp_push_byte(mc_debug_transp_t *t, uint8_t byte);
extern void mc_debug_transp_reset_rx(mc_debug_transp_t *t);

static uint8_t g_tx_buf[256];
static uint16_t g_tx_len;
static void stub_tx(const uint8_t *data, uint16_t len) {
    memcpy(g_tx_buf, data, len); g_tx_len = len;
}

void test_mc_debug_transp_detect_valid_frame(void)
{
    mc_debug_transp_t t;
    mc_debug_transp_init(&t, MC_DEBUG_TRANSP_UART, stub_tx);

    uint8_t frame[] = { 0xDA, 0x1A, 0x08, 0x00, 'H','e','l','l','o','!','!','!' };
    mc_bool_t ready = MC_FALSE;
    int i;
    for (i = 0; i < (int)sizeof(frame); i++)
    {
        ready = mc_debug_transp_push_byte(&t, frame[i]);
    }
    TEST_ASSERT_TRUE(ready == MC_TRUE);
    TEST_ASSERT_EQUAL_INT(8U, t.rx_len);
}

void test_mc_debug_transp_reject_bad_magic(void)
{
    mc_debug_transp_t t;
    mc_debug_transp_init(&t, MC_DEBUG_TRANSP_UART, stub_tx);

    uint8_t frame[] = { 0xAA, 0xBB, 0x08, 0x00, 'X','X','X','X','X','X','X','X' };
    int i;
    for (i = 0; i < (int)sizeof(frame); i++)
    {
        mc_debug_transp_push_byte(&t, frame[i]);
    }
    TEST_ASSERT_EQUAL_INT(0U, t.rx_len);
}

void test_mc_debug_transp_reset_clears_buffer(void)
{
    mc_debug_transp_t t;
    mc_debug_transp_init(&t, MC_DEBUG_TRANSP_UART, stub_tx);

    t.rx_len = 5U;
    mc_debug_transp_reset_rx(&t);
    TEST_ASSERT_EQUAL_INT(0U, t.rx_len);
}

void test_mc_debug_transp_init_with_null_tx(void)
{
    mc_debug_transp_t t;
    mc_debug_transp_init(&t, MC_DEBUG_TRANSP_UART, NULL);
    TEST_ASSERT_TRUE(t.tx_flush == NULL);
    TEST_ASSERT_TRUE(t.initialized == MC_TRUE);
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: FAIL — undefined functions

- [ ] **Step 3: Implement mc_debug_transp.c**

Create `src/debug/mc_debug_transp.c`:

```c
/** @file mc_debug_transp.c @brief Transport abstraction for debug subsystem */

#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

void mc_debug_transp_init(mc_debug_transp_t *t, mc_debug_transp_type_t type,
                           void (*tx_flush)(const uint8_t *data, uint16_t len))
{
    if (t == NULL) { return; }
    memset(t, 0, sizeof(*t));
    t->type = type;
    t->tx_flush = tx_flush;
    t->initialized = MC_TRUE;
}

void mc_debug_transp_reset_rx(mc_debug_transp_t *t)
{
    if (t == NULL) { return; }
    t->rx_len = 0U;
}

mc_bool_t mc_debug_transp_push_byte(mc_debug_transp_t *t, uint8_t byte)
{
    uint16_t frame_len;

    if (t == NULL) { return MC_FALSE; }

    if (t->rx_len == 0U)
    {
        if (byte != MC_DEBUG_FRAME_MAGIC_0) { return MC_FALSE; }
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len == 1U)
    {
        if (byte != MC_DEBUG_FRAME_MAGIC_1) { t->rx_len = 0U; return MC_FALSE; }
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len < MC_DEBUG_HEADER_SIZE)
    {
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len == MC_DEBUG_HEADER_SIZE)
    {
        frame_len = (uint16_t)t->rx_buf[2] | ((uint16_t)t->rx_buf[3] << 8);
        if (frame_len < MC_DEBUG_HEADER_SIZE || frame_len > MC_DEBUG_BUF_SIZE)
        {
            t->rx_len = 0U;
            return MC_FALSE;
        }
    }

    t->rx_buf[t->rx_len++] = byte;
    if (t->rx_len >= MC_DEBUG_HEADER_SIZE)
    {
        frame_len = (uint16_t)t->rx_buf[2] | ((uint16_t)t->rx_buf[3] << 8);
        if (t->rx_len >= frame_len)
        {
            return MC_TRUE;
        }
    }
    return MC_FALSE;
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/debug/mc_debug_transp.c tests/unit/test_mc_debug_transp.c tests/unit/test_mc_api.c CMakeLists.txt
git commit -m "feat: add mc_debug_transp transport abstraction with UART frame detection"
```

---

### Task 6: mc_debug_fm.c — FreeMASTER protocol engine (TDD)

**Files:**
- Create: `src/debug/mc_debug_fm.c`
- Create: `tests/unit/test_mc_debug_fm.c`

- [ ] **Step 1: Write the failing test — GET_INFO command**

Create `tests/unit/test_mc_debug_fm.c`:

```c
#include "unity.h"
#include "mc_debug.h"
#include <string.h>

extern void mc_debug_fm_init(mc_debug_t *dbg,
                              void (*tx_flush)(const uint8_t *data, uint16_t len));
extern void mc_debug_fm_poll(mc_debug_t *dbg);
extern void mc_debug_fm_feed_rx(mc_debug_t *dbg, const uint8_t *frame, uint16_t len);

static uint8_t g_fm_tx[256];
static uint16_t g_fm_tx_len;
static void fm_stub_tx(const uint8_t *data, uint16_t len) {
    memcpy(g_fm_tx, data, len); g_fm_tx_len = len;
}

void test_mc_debug_fm_get_info_responds(void)
{
    mc_debug_t dbg;
    mc_debug_fm_init(&dbg, fm_stub_tx);

    uint8_t get_info[] = { 0xDA, 0x1A, 0x05, 0x00, 0x01 };
    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, get_info, sizeof(get_info));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 4U);
    TEST_ASSERT_EQUAL_INT(0x80, g_fm_tx[4]);
}

void test_mc_debug_fm_unknown_command_no_response(void)
{
    mc_debug_t dbg;
    mc_debug_fm_init(&dbg, fm_stub_tx);

    uint8_t unknown[] = { 0xDA, 0x1A, 0x05, 0x00, 0xFF };
    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, unknown, sizeof(unknown));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_EQUAL_INT(0U, g_fm_tx_len);
}

void test_mc_debug_fm_scope_timer_fires_sends_frame(void)
{
    mc_debug_t dbg;
    mc_debug_fm_init(&dbg, fm_stub_tx);

    dbg.scope_active = MC_TRUE;
    dbg.scope_period_us = 100U;
    dbg.scope_last_us = 0U;
    g_fm_tx_len = 0U;
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 0U);
}

void test_mc_debug_fm_write_var_updates_value(void)
{
    static mc_f32_t g_calib = 0.0F;
    mc_debug_t dbg;
    mc_debug_fm_init(&dbg, fm_stub_tx);

    mc_f32_t new_val = 2.5F;
    uint8_t write_cmd[14] = { 0xDA, 0x1A, 0x0E, 0x00, 0x04 };
    write_cmd[5] = 0U;
    write_cmd[6] = MC_DEBUG_VAR_TYPE_FLOAT32;
    write_cmd[7] = 4U;
    memcpy(&write_cmd[8], &new_val, 4U);
    write_cmd[12] = 0U;
    write_cmd[13] = 0U;

    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, write_cmd, sizeof(write_cmd));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 0U);
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: FAIL

- [ ] **Step 3: Implement mc_debug_fm.c**

Create `src/debug/mc_debug_fm.c`:

```c
/** @file mc_debug_fm.c @brief FreeMASTER protocol engine */

#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

#define MC_DEBUG_FM_CMD_GET_INFO    (0x01U)
#define MC_DEBUG_FM_CMD_READ_VARS   (0x03U)
#define MC_DEBUG_FM_CMD_WRITE_VAR   (0x04U)
#define MC_DEBUG_FM_CMD_SCOPE_START (0x10U)
#define MC_DEBUG_FM_CMD_REC_START   (0x20U)
#define MC_DEBUG_FM_CMD_RESPONSE    (0x80U)

extern const mc_debug_var_t *mc_debug_get_var_table(void);
extern uint8_t mc_debug_get_var_count(void);
extern void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                                  uint8_t *buf, uint16_t *buf_len);

static void mc_debug_fm_send_response(mc_debug_t *dbg, uint8_t cmd, const uint8_t *payload, uint16_t pay_len)
{
    uint8_t *tx = dbg->transp.tx_buf;
    uint16_t total = (uint16_t)(MC_DEBUG_HEADER_SIZE + 1U + pay_len);

    tx[0] = MC_DEBUG_FRAME_MAGIC_0;
    tx[1] = MC_DEBUG_FRAME_MAGIC_1;
    tx[2] = (uint8_t)(total & 0xFFU);
    tx[3] = (uint8_t)((total >> 8) & 0xFFU);
    tx[4] = cmd;
    if (pay_len > 0U && payload != NULL)
    {
        memcpy(&tx[5], payload, pay_len);
    }
    if (dbg->transp.tx_flush != NULL)
    {
        dbg->transp.tx_flush(tx, total);
    }
}

void mc_debug_fm_init(mc_debug_t *dbg, void (*tx_flush)(const uint8_t *data, uint16_t len))
{
    if (dbg == NULL) { return; }
    memset(dbg, 0, sizeof(*dbg));
    dbg->transp.type = MC_DEBUG_TRANSP_UART;
    dbg->transp.tx_flush = tx_flush;
    dbg->transp.initialized = MC_TRUE;
}

void mc_debug_fm_feed_rx(mc_debug_t *dbg, const uint8_t *frame, uint16_t len)
{
    if ((dbg == NULL) || (frame == NULL)) { return; }

    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        mc_bool_t ready = mc_debug_transp_push_byte(&dbg->transp, frame[i]);
        if (ready != MC_FALSE) { break; }
    }
}

void mc_debug_fm_poll(mc_debug_t *dbg)
{
    uint8_t cmd;
    if (dbg == NULL) { return; }

    if (dbg->transp.rx_len >= MC_DEBUG_HEADER_SIZE + 1U)
    {
        cmd = dbg->transp.rx_buf[4];
        switch (cmd)
        {
        case MC_DEBUG_FM_CMD_GET_INFO:
            {
                uint8_t resp[128];
                uint8_t vc = mc_debug_get_var_count();
                uint16_t rlen = 4U;
                resp[0] = 0x01U; resp[1] = 0x00U;
                resp[2] = (uint8_t)(MC_DEBUG_BUF_SIZE & 0xFFU);
                resp[3] = (uint8_t)((MC_DEBUG_BUF_SIZE >> 8) & 0xFFU);
                uint8_t j;
                for (j = 0U; j < vc; j++)
                {
                    uint8_t nl = (uint8_t)strlen(mc_debug_get_var_table()[j].name);
                    resp[rlen++] = (uint8_t)j;
                    resp[rlen++] = mc_debug_get_var_table()[j].flags;
                    resp[rlen++] = nl;
                    memcpy(&resp[rlen], mc_debug_get_var_table()[j].name, nl);
                    rlen += nl;
                }
                mc_debug_fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, resp, rlen);
            }
            break;
        case MC_DEBUG_FM_CMD_READ_VARS:
            {
                uint32_t mask = 0U;
                if (dbg->transp.rx_len >= 6U)
                {
                    mask = (uint32_t)dbg->transp.rx_buf[5]
                         | ((uint32_t)dbg->transp.rx_buf[6] << 8)
                         | ((uint32_t)dbg->transp.rx_buf[7] << 16)
                         | ((uint32_t)dbg->transp.rx_buf[8] << 24);
                }
                dbg->active_var_mask = mask;
                uint8_t buf[256];
                uint16_t blen = 0U;
                mc_debug_map_collect(mask, 0U, buf, &blen);
                mc_debug_fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, buf, blen);
            }
            break;
        case MC_DEBUG_FM_CMD_WRITE_VAR:
            {
                if (dbg->transp.rx_len >= 13U)
                {
                    uint8_t vi = dbg->transp.rx_buf[5];
                    if (vi < mc_debug_get_var_count() &&
                        (mc_debug_get_var_table()[vi].flags & MC_DEBUG_FLAG_WRITABLE))
                    {
                        memcpy(mc_debug_get_var_table()[vi].addr,
                               &dbg->transp.rx_buf[8],
                               mc_debug_get_var_table()[vi].size_bytes);
                    }
                }
                uint8_t ack = 0x00U;
                mc_debug_fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, &ack, 1U);
            }
            break;
        case MC_DEBUG_FM_CMD_SCOPE_START:
            dbg->scope_active = MC_TRUE;
            dbg->scope_period_us = 1000U;
            dbg->scope_last_us = 0U;
            break;
        case MC_DEBUG_FM_CMD_REC_START:
            dbg->recorder_active = MC_TRUE;
            dbg->recorder_total_frames = 100U;
            dbg->recorder_frame_count = 0U;
            break;
        default:
            break;
        }
        dbg->transp.rx_len = 0U;
    }

    if (dbg->scope_active != MC_FALSE)
    {
        if (dbg->active_var_mask != 0U)
        {
            uint8_t sbuf[256];
            uint16_t slen = 0U;
            mc_debug_map_collect(dbg->active_var_mask, dbg->scope_last_us, sbuf, &slen);
            mc_debug_fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, sbuf, slen);
        }
        dbg->scope_last_us += dbg->scope_period_us;
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cmake --build build; ctest --test-dir build -R test_mc_api
```
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/debug/mc_debug_fm.c tests/unit/test_mc_debug_fm.c tests/unit/test_mc_api.c CMakeLists.txt
git commit -m "feat: add mc_debug_fm FreeMASTER protocol engine with 6 commands"
```

---

### Task 7: Integrate into mc_api (mc_instance_t + mc_fast_step hook)

**Files:**
- Modify: `include/mc_api.h`
- Modify: `src/api/mc_api.c`

- [ ] **Step 1: Add mc_debug_t field to mc_instance_t**

In `include/mc_api.h`, after `#include "mc_sensor_smo.h"`, add:

```c
#if MC_CFG_ENABLE_DEBUG
#include "mc_debug.h"
#endif
```

And in `mc_instance_t`, after `mc_identify_t identify;`, add:

```c
#if MC_CFG_ENABLE_DEBUG
    mc_debug_t debug;
#endif
```

- [ ] **Step 2: Add mc_debug_poll() call at end of mc_fast_step**

In `src/api/mc_api.c`, at the end of `mc_fast_step()`, before the final `return MC_STATUS_OK;` (there are multiple return paths — add before each one), add:

```c
#if MC_CFG_ENABLE_DEBUG
    mc_debug_fm_poll(&inst->debug);
#endif
```

(Add after each `return MC_STATUS_OK;` in the fast_step function — approximately at lines 1094, 1465, and similar.)

- [ ] **Step 3: Build to verify integration compiles**

```bash
cmake --build build
```
Expected: 0 errors

- [ ] **Step 4: Commit**

```bash
git add include/mc_api.h src/api/mc_api.c
git commit -m "feat: integrate debug subsystem into mc_instance_t and mc_fast_step"
```

---

### Task 8: Add debug variable registration table to mc_api.c

**Files:**
- Modify: `src/api/mc_api.c`

- [ ] **Step 1: Add the real variable registration table**

In `src/debug/mc_debug_map.c`, replace the test table with the real one that references `mc_instance_t` fields. Since the table needs access to `mc_instance_t`, we declare an `extern` pointer and set it during `mc_init`:

Add to `src/debug/mc_debug_map.c`:

```c
#if MC_CFG_ENABLE_DEBUG
#include "mc_api.h"

static mc_instance_t *g_debug_inst;

void mc_debug_set_instance(mc_instance_t *inst) { g_debug_inst = inst; }

static const mc_debug_var_t g_debug_var_table[] = {
    MC_DEBUG_VAR_RW("id_ref",      mc_f32_t, &g_debug_inst->id_ref),
    MC_DEBUG_VAR_RW("iq_ref",      mc_f32_t, &g_debug_inst->iq_ref),
    MC_DEBUG_VAR_RO("speed_rpm",   mc_f32_t, &g_debug_inst->foc_last_output.mech_speed_rpm),
    /* ... add more as needed, 32 max */
};
#endif
```

In `src/api/mc_api.c`, at the end of `mc_init()`, add:

```c
#if MC_CFG_ENABLE_DEBUG
    mc_debug_set_instance(inst);
#endif
```

- [ ] **Step 2: Commit**

```bash
git add src/debug/mc_debug_map.c src/api/mc_api.c
git commit -m "feat: wire debug variable table to mc_instance_t"
```

---

### Task 9: Update documentation

**Files:**
- Modify: `CHANGELOG.md`
- Modify: `docs/UM_USER_MANUAL.md`

- [ ] **Step 1: Add to CHANGELOG.md**

Add under "Unreleased → Added":

```markdown
- Debug/calibration subsystem: FreeMASTER protocol over UART/CAN/LIN (`src/debug/`)
- `mc_debug_var_t` variable map with compile-time registration and runtime selection
- `mc_debug_port_t` platform hooks for debug communication (UART/CAN/LIN TX)
- `MC_DEBUG_ENABLE` master compile-time switch with granular sub-feature macros
```

- [ ] **Step 2: Add debug section to User Manual**

In `docs/UM_USER_MANUAL.md`, after section 8 (Build System), add:

```markdown
## 9. Debug & Calibration / 调试与标定

### Enable / 启用

```c
#define MC_CFG_ENABLE_DEBUG (1U)  // in mc_cfg.h or compile -D
```

### Platform Hooks / 平台钩子

```c
mc_debug_port_t debug_port = {
    .uart_tx = my_uart_send,
    .uart_rx_available = my_uart_rx_count,
    .uart_rx_read = my_uart_read,
};
```

### Usage with FreeMASTER

1. Connect FreeMASTER via UART at configured baud rate
2. FreeMASTER sends GET_INFO to discover variable list
3. Use FreeMASTER Scope to visualize real-time waveforms
4. Use FreeMASTER Variable Watch to modify PI gains online
```

- [ ] **Step 3: Commit**

```bash
git add CHANGELOG.md docs/UM_USER_MANUAL.md
git commit -m "docs: document debug/calibration subsystem in changelog and user manual"
```

---

## Verification Checklist

- [ ] `cmake --build build` — 0 errors
- [ ] `cmake --build build --target cppcheck` — 0 critical
- [ ] All existing tests pass (no regression)
- [ ] New debug tests pass (map + transp + fm = ~12 new test cases)
- [ ] `MC_CFG_ENABLE_DEBUG=0` → debug code not compiled (set macro to 0 and rebuild — no debug symbols)

# MC Debug & Calibration Utility — Design Spec

**Date**: 2026-04-29 | **Status**: Approved | **Phase**: Specification

## Overview

Add a compile-time configurable debug/calibration subsystem to the Motor Control Library. Supports real-time waveform capture, online parameter tuning, and fault diagnostics via UART/CAN/LIN communication to PC host tools (FreeMASTER as primary, XCP as future extension).

### Motivation

The library contains rich internal data (FOC states, PI states, sensor estimates, diagnostics) but has zero mechanisms to observe, stream, or transmit that data. Users currently have no way to visualize control loop behavior, tune parameters interactively, or capture fault snapshots without intrusive code modifications.

### Design Principles

- **Minimal intrusion**: When `MC_DEBUG_ENABLE=0`, zero code or data is compiled into the library
- **Follow existing patterns**: Uses the same port-hook abstraction as PWM/ADC/fault hooks
- **Compile-time configurable**: Macro switches control feature depth to match MCU resource budgets
- **Protocol-agnostic core**: Variable mapping and data collection are independent of wire protocol

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  mc_api.c  ── mc_fast_step() ends with:            │
│               if (MC_DEBUG_ENABLE) mc_debug_poll()  │
├─────────────────────────────────────────────────────┤
│  mc_debug_map.c   — Variable map manager            │
│  ┌───────────────────────────────────────────────┐  │
│  │ Compile-time registration table (32-64 slots) │  │
│  │ Runtime active-slot selection by host tool   │  │
│  │ Data collection: pointer deref per slot      │  │
│  │ Dual-buffer ping-pong for Recorder mode      │  │
│  └───────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────┤
│  mc_debug_fm.c    — FreeMASTER protocol engine       │
│  ┌───────────────────────────────────────────────┐  │
│  │ 6-command subset: GET_INFO, READ_VARS,        │  │
│  │ WRITE_VAR, SCOPE_START, REC_START, RESPONSE   │  │
│  │ TLV frame encoding/decoding                   │  │
│  │ Scope auto-timer for periodic streaming       │  │
│  └───────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────┤
│  mc_debug_transp.c — Transport abstraction layer     │
│  ┌───────────────────────────────────────────────┐  │
│  │ Frame boundary detection (magic bytes/length) │  │
│  │ UART/CAN/LIN unified interface                │  │
│  │ Dual-direction buffering (rx/tx)              │  │
│  │ Delegates physical I/O to mc_debug_port_t     │  │
│  └───────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────┤
│  mc_port_debug.h  — Platform hooks (new, optional)    │
│  ┌───────────────────────────────────────────────┐  │
│  │ uart_tx(), can_tx(), lin_tx()                 │  │
│  │ uart_rx_available(), uart_rx_read()           │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

## File Layout

```
include/
  mc_debug.h              — Public types: mc_debug_var_t, mc_debug_t, mc_debug_cfg_t
  mc_port_debug.h          — Platform hooks: mc_debug_port_t

src/debug/
  mc_debug_map.c           — Variable map manager (registration, selection, collection)
  mc_debug_fm.c            — FreeMASTER protocol engine
  mc_debug_transp.c        — Transport abstraction layer
```

## Compile-Time Macros (in `mc_cfg.h` or project build flags)

| Macro | Default | Description |
|---|---|---|
| `MC_DEBUG_ENABLE` | 0 | Master switch; =0 excludes all debug code |
| `MC_DEBUG_PROTO_FREEMASTER` | 1 | FreeMASTER protocol engine |
| `MC_DEBUG_TRANSPORT_UART` | 1 | UART transport layer |
| `MC_DEBUG_TRANSPORT_CAN` | 0 | CAN transport layer |
| `MC_DEBUG_TRANSPORT_LIN` | 0 | LIN transport layer |
| `MC_DEBUG_MAP_SIZE` | 32 | Max variable slots in registration table |
| `MC_DEBUG_BUF_SIZE` | 1024 | Frame buffer size (bytes) |

## Variable Map Design

### Registration (compile-time)

```c
typedef struct {
    const char *name;      // Human-readable name (sent during handshake)
    void       *addr;      // Pointer to variable in mc_instance_t
    uint8_t     type;      // 0=float32, 1=int16, 2=uint16, 3=uint8, 4=uint32
    uint8_t     size;      // 1, 2, or 4 bytes
    uint8_t     flags;     // bit0=readable, bit1=writable(calibration), bit2=Recorder-only
} mc_debug_var_t;

#define MC_DEBUG_VAR_RO(name_, ctype_, addr_)   { name_, (void*)(addr_), MC_DEBUG_FROM_CTYPE(ctype_), sizeof(ctype_), 0x01 }
#define MC_DEBUG_VAR_RW(name_, ctype_, addr_)   { name_, (void*)(addr_), MC_DEBUG_FROM_CTYPE(ctype_), sizeof(ctype_), 0x03 }

static const mc_debug_var_t g_debug_var_table[MC_DEBUG_MAP_SIZE] = { /* user fills */ };
```

### Frame Format

```
| 0-3:  Timestamp_us  (uint32 LE)
| 4:    Var count     (uint8, 1-16)
| For each variable:
|   Byte N:   Var index (uint8)
|   Byte N+1: Var type  (uint8)
|   Byte N+2: Var size  (uint8)
|   Byte N+3+: Var value (LE)
```

8 float32 variables + header = ~61 bytes. At 4 kHz = ~244 KB/s. UART at 1 Mbaud handles this; CAN FD also sufficient.

### Collection

`mc_debug_map_collect()` iterates the active slot mask, dereferences each pointer, and packs into the frame buffer. No runtime overhead for inactive slots.

## FreeMASTER Protocol Engine

### Implemented Command Subset

| Command | Code | Direction | Purpose |
|---|---|---|---|
| `GET_INFO` | 0x01 | Host→MCU | Handshake: protocol version, buffer size, variable name list |
| `READ_VARS` | 0x03 | Host→MCU | Request data for specified variable indices |
| `WRITE_VAR` | 0x04 | Host→MCU | Write single variable value (with post-write callback) |
| `SCOPE_START` | 0x10 | Host→MCU | Start/stop automatic periodic scope streaming |
| `REC_START` | 0x20 | Host→MCU | High-speed single-shot recorder (N frames, dual-buffer) |
| `RESPONSE` | 0x80 | MCU→Host | Generic response frame with data payload |

### Engine State Machine

```
IDLE → poll() reads one frame → command decode → handle command → send response → IDLE
```

`mc_debug_fm_poll()` processes one complete command per invocation. Non-blocking — returns immediately if no frame ready.

### Scope Mode

When `SCOPE_START` activates, `mc_debug_fm_poll()` checks `scope_timer_expired`. If true, calls `mc_debug_map_collect()` and sends one scope frame. Timer interval configurable by host (e.g., every N PWM cycles or every T microseconds).

### Recorder Mode

When `REC_START` activates with parameter N:
1. Start dual-buffer ping-pong collection
2. Each `mc_fast_step` writes one frame into current buffer
3. When buffer full, swap buffers; transmit full buffer asynchronously
4. After N frames collected, stop and notify host

## Transport Abstraction Layer

### Interface

```c
typedef struct {
    mc_debug_transp_type_t type;
    mc_bool_t initialized;
    uint8_t  rx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t rx_len;
    uint8_t  tx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t tx_len;
    uint16_t tx_sent;
    void   (*tx_flush)(const uint8_t *data, uint16_t len);
} mc_debug_transp_t;
```

### Frame Boundary Strategy

| Transport | Boundary | Max Frame | Typical Rate |
|---|---|---|---|
| UART | Magic bytes `0xDA 0x1A` + 2-byte length | 256 B | 1-3 Mbaud |
| CAN | Standard frame 8B each; first frame carries total_len; multi-frame concat | 256 B | 1 Mbps |
| LIN | Same as UART; obeys LIN Schedule Table | 256 B | 20 kbps |

### Flow Control

MCU does not actively flow-control. Host is responsible for not exceeding MCU processing rate. Scope streaming rate is limited by MCU's `scope_timer_expired` interval. Recorder mode uses dual-buffer to avoid underrun.

## Online Parameter Calibration

### WRITE_VAR Flow

```
Host sends WRITE_VAR(var_index=N, value=V)
  → MCU verifies var.flags has bit1 (writable)
  → *(var.addr) = V  (direct memory write)
  → Post-write callback if registered (see below)
  → RESPONSE: OK (or NAK if write-protected)
```

### Post-Write Callbacks

| Variable Category | Callback |
|---|---|
| PI gains (id_kp, iq_kp, speed_kp, etc.) | `mc_control_foc_set_speed_pi()` or `mc_pi_init()` |
| FW parameters (fw_kp, fw_ki) | None — effective next cycle |
| Observer parameters (pll_kp, pll_ki) | Re-init observer via `mc_sensorless_init()` or `mc_smo_init()` |
| Operating mode | Write-protected — NAK |
| Dead-time (dtc_deadtime_ns) | None — effective next cycle |

### Safety

- `mc_debug_var_t.flags` bit1 gates write access; non-writable vars return NAK
- No flash persistence: calibration values lost on reset; reverts to defaults from `mc_init()`
- Host must re-apply calibration after each power cycle

## Platform Integration

### New Hook Structure (optional, separate from mc_port_hooks_t)

```c
typedef struct {
    void (*uart_tx)(const uint8_t *data, uint16_t len);
    void (*can_tx)(uint32_t id, const uint8_t *data, uint8_t len);
    void (*lin_tx)(const uint8_t *data, uint16_t len);
    uint16_t (*uart_rx_available)(void);
    uint8_t  (*uart_rx_read)(void);
} mc_debug_port_t;
```

Hooks are NULL-checked. If a transport is compiled in but its hook is NULL, that transport is silently disabled at runtime.

### Instance Integration

```c
typedef struct {
    // ... existing fields ...
#if MC_DEBUG_ENABLE
    mc_debug_t debug;
#endif
} mc_instance_t;
```

`mc_fast_step()` ends with:
```c
#if MC_DEBUG_ENABLE
    mc_debug_poll(&inst->debug);
#endif
```

## Resource Budget by MCU Tier

| Feature | M0+ (min) | M4F (default) | M7/R5 (max) |
|---|---|---|---|
| RAM | < 2 KB | 4-8 KB | 8-16 KB |
| Flash | < 8 KB | 16-24 KB | 24-48 KB |
| Variable slots | 16 | 32 | 64 |
| Buffer size | 512 B | 1024 B | 2048 B |
| Transports | UART only | UART + CAN | UART + CAN + LIN |
| Recorder | No | Yes (dual-buffer) | Yes (dual-buffer) |
| XCP | No | No | Stub ready |

## Out of Scope (for this spec)

- Flash/EEPROM persistence of calibration values (future)
- XCP full protocol implementation (future, stub only)
- PC-side application development (assumes FreeMASTER handles visualization)
- JTAG/SWD-based debugging (separate concern)
- SafeTI/Functional Safety integration for debug path (debug is non-safety)
- Real-time OS integration examples (separate spec)

## Testing Strategy

| Layer | Test Type | Cases |
|---|---|---|
| mc_debug_map | Unit | Registration table bounds, type tag encoding, value collection correctness |
| mc_debug_fm | Unit | Command decode of all 6 commands, malformed frame handling, CRC error |
| mc_debug_transp | Unit | Magic-byte detection, frame length parsing, multi-segment CAN assembly |
| Integration | Fault injection | UART line noise, CAN bus-off, buffer overflow, NULL hook |
| End-to-end | Hardware | Connect FreeMASTER, verify scope + parameter write works on real MCU |

## Dependencies

- `mc_types.h` — types (mc_f32_t, mc_bool_t, uint8_t, etc.)
- `mc_constants.h` — epsilon, buffer size constants
- `mc_api.h` — mc_instance_t for variable address registration
- `mc_control_pi.h` — PI init/reset callbacks for auto-tune
- `mc_cfg.h` — macro switches for compile-time configuration
- No third-party libraries. No dynamic memory. No RTOS dependencies.

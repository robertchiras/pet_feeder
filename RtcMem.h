#include "Logger.h"

/* RTC memory defines */
#define MAGIC_ID 0xfcec
#define RUN_MAGIC_ID 0xccec
#define OTA_MAGIC_ID 0xdcec
/*
 * Structure of MAGIC_BLK:
 * 31-16: MAGIC_ID
 * 15-3: Reserved
 *    5: i2c reset bit
 *    4: HX711 present bit
 *    3: RTC present bit
 *    2: Daylight Saving Time
 *    1: Deep Sleep Enabled
 *    0: Initialized bit
 */
/*
 * Structure of RUN_BLK:
 * 15-0: lastRun
 */
#define RTC_BLK 186 // 186-187 - time obtained from NTP/RTC
#define UPTIME_BLK 184 //184-185 - time elapsed since we got NTP/RTC
#define MAGIC_BLK 183
#define RTCU_BLK 182 // not used
#define RTCC_BLK 181 // not used
#define RUN_BLK 180
#define STATS_BLK 179
#define WARN_BLK 178
/* END RTC memory defines */

#define BIT(nr) (1 << nr)
#define GENMASK(h, l) (((1 << ((h) - (l) + 1)) - 1) << (l))
#define REG_PUT(x, h, l) (((x) << (l)) & GENMASK(h, l))
#define REG_GET(x, h, l) (((x) & GENMASK(h, l)) >> (l))
#define REG_UPDATE(reg, val, h, l) ((reg & ~(REG_PUT(~0, h, l))) | REG_PUT(val, h, l))
#define REG_GET_BIT(reg, b)     REG_GET(reg, b, b)
#define REG_SET_BIT(reg, val, b)  ((reg & ~(REG_PUT(~0, b, b)))   | REG_PUT(val, b, b))

/* MAGIC_BLK */
#define MAGIC_GET_ID(x)         REG_GET(x, 31, 16)
#define MAGIC_GET_RESETS(x)     REG_GET(x, 15, 4)
#define MAGIC_GET_I2C_RESET(x)  REG_GET_BIT(x, 5)
#define MAGIC_GET_SCALE(x)      REG_GET_BIT(x, 4)
#define MAGIC_GET_RTC(x)        REG_GET_BIT(x, 3)
#define MAGIC_GET_DST(x)        REG_GET_BIT(x, 2)
#define MAGIC_GET_SLEEP(x)      REG_GET_BIT(x, 1)
#define MAGIC_GET_INIT(x)       REG_GET_BIT(x, 0)

#define MAGIC_SET_ID(x, v)        REG_UPDATE(x, v, 31, 16)
#define MAGIC_SET_RESETS(x, v)    REG_UPDATE(x, v, 15, 4)
#define MAGIC_SET_I2C_RESET(x, v) REG_SET_BIT(x, v, 5)
#define MAGIC_SET_SCALE(x, v)     REG_SET_BIT(x, v, 4)
#define MAGIC_SET_RTC(x, v)       REG_SET_BIT(x, v, 3)
#define MAGIC_SET_DST(x, v)       REG_SET_BIT(x, v, 2)
#define MAGIC_SET_SLEEP(x, v)     REG_SET_BIT(x, v, 1)
#define MAGIC_SET_INIT(x, v)      REG_SET_BIT(x, v, 0)

/* Generic BYTE_BLK */
#define BYTE_GET_3(x)     REG_GET(x, 31, 24)
#define BYTE_GET_2(x)     REG_GET(x, 23, 16)
#define BYTE_GET_1(x)     REG_GET(x, 15, 8)
#define BYTE_GET_0(x)     REG_GET(x, 7, 0)

#define BYTE_SET_3(x, v)  ((x & ~(REG_PUT(~0, 31, 24))) | REG_PUT(v, 31, 24))
#define BYTE_SET_2(x, v)  ((x & ~(REG_PUT(~0, 23, 16))) | REG_PUT(v, 23, 16))
#define BYTE_SET_1(x, v)  ((x & ~(REG_PUT(~0, 15, 8)))  | REG_PUT(v, 15, 8))
#define BYTE_SET_0(x, v)  ((x & ~(REG_PUT(~0, 7, 0)))   | REG_PUT(v, 7, 0))

u8 rtcmem_get_byte(u8 blk, u8 id) {
  u32 w_data = 0;
  u8 b = 0;
  
  system_rtc_mem_read(blk, &w_data, sizeof(w_data));

  switch (id) {
    case 0:
      b = BYTE_GET_0(w_data);
      break;
    case 1:
      b = BYTE_GET_1(w_data);
      break;
    case 2:
      b = BYTE_GET_2(w_data);
      break;
    case 3:
      b = BYTE_GET_3(w_data);
      break;
  }

  LOG(4, "[BLK:%u] rtcmem_get_byte[%u]: %u(0x%02X) -> w_data: 0x%08X\n", blk, id, b, b, w_data);
  return b;
}
#define rtcmem_get_u16(blk, pos) (rtcmem_get_byte(blk, pos + 1) << 8 | rtcmem_get_byte(blk, pos))

void rtcmem_set_byte(u8 blk, u8 id, u8 b) {
  u32 w_data = 0;
  
  system_rtc_mem_read(blk, &w_data, sizeof(w_data));
  switch (id) {
    case 0:
      w_data = BYTE_SET_0(w_data, b);
      break;
    case 1:
      w_data = BYTE_SET_1(w_data, b);
      break;
    case 2:
      w_data = BYTE_SET_2(w_data, b);
      break;
    case 3:
      w_data = BYTE_SET_3(w_data, b);
      break;
  }
  
  LOG(4, "[BLK:%u] rtcmem_set_byte[%u]: %u(0x%02X) -> w_data: 0x%08X\n", blk, id, b, b, w_data);
  system_rtc_mem_write(blk, &w_data, sizeof(w_data));  
}
#define rtcmem_set_u16(blk, v, pos) \
{\
    rtcmem_set_byte(blk, pos, v & 0xff);\
    rtcmem_set_byte(blk, pos + 1, v >> 8);\
}

// MAGIC_BLK
#define rtcmem_get_magic() rtcmem_get_u16(MAGIC_BLK, 2)
#define rtcmem_set_magic(v) rtcmem_set_u16(MAGIC_BLK, v, 2)

// WARN_BLK
#define rtcmem_get_lpsleep_id() rtcmem_get_u16(WARN_BLK, 2)
#define rtcmem_set_lpsleep_id(v) rtcmem_set_u16(WARN_BLK, v, 2)
#define rtcmem_get_lpsleep_secs() rtcmem_get_u16(WARN_BLK, 0)
#define rtcmem_set_lpsleep_secs(v) rtcmem_set_u16(WARN_BLK, v, 0)

/* GET/SET a bit inside the MAGIC_ID block */
bool magic_get_bit(u8 b) {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  LOG(4, "magic_get_bit @%u -> w_data: 0x%08X\n", b, w_data);
   
  return REG_GET_BIT(w_data, b);
}

void magic_set_bit(bool v, u8 b) {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  w_data = REG_SET_BIT(w_data, !!v, b);
  LOG(4, "magic_set_bit(%d@%u) -> w_data: 0x%08X\n", !!v, b, w_data);
  system_rtc_mem_write(MAGIC_BLK, &w_data, sizeof(w_data));
}

#include "Logger.h"

#define BIT(nr) (1 << nr)
#define GENMASK(h, l) (((1 << ((h) - (l) + 1)) - 1) << (l))
#define REG_PUT(x, h, l) (((x) << (l)) & GENMASK(h, l))
#define REG_GET(x, h, l) (((x) & GENMASK(h, l)) >> (l))

/* MAGIC_BLK */
#define MAGIC_GET_ID(x)     REG_GET(x, 31, 16)
#define MAGIC_GET_RESETS(x) REG_GET(x, 15, 4)
#define MAGIC_GET_DST(x)  REG_GET(x, 2, 2)
#define MAGIC_GET_SLEEP(x)  REG_GET(x, 1, 1)
#define MAGIC_GET_INIT(x)   (x & BIT(0))

#define MAGIC_SET_ID(x, v)      ((x & ~(REG_PUT(~0, 31, 16))) | REG_PUT(v, 31, 16))
#define MAGIC_SET_RESETS(x, v)  ((x & ~(REG_PUT(~0, 15, 4)))  | REG_PUT(v, 15, 4))
#define MAGIC_SET_DST(x, v)   ((x & ~(REG_PUT(~0, 2, 2)))   | REG_PUT(v, 2, 2))
#define MAGIC_SET_SLEEP(x, v)   ((x & ~(REG_PUT(~0, 1, 1)))   | REG_PUT(v, 1, 1))
#define MAGIC_SET_INIT(x, v)    ((x & ~BIT(0)) | (v & BIT(0)))

/* Generic BYTE_BLK */
#define BYTE_GET_3(x)     REG_GET(x, 31, 24)
#define BYTE_GET_2(x)     REG_GET(x, 23, 16)
#define BYTE_GET_1(x)     REG_GET(x, 15, 8)
#define BYTE_GET_0(x)     REG_GET(x, 7, 0)

#define BYTE_SET_3(x, v)  ((x & ~(REG_PUT(~0, 31, 24))) | REG_PUT(v, 31, 24))
#define BYTE_SET_2(x, v)  ((x & ~(REG_PUT(~0, 23, 16))) | REG_PUT(v, 23, 16))
#define BYTE_SET_1(x, v)  ((x & ~(REG_PUT(~0, 15, 8)))  | REG_PUT(v, 15, 8))
#define BYTE_SET_0(x, v)  ((x & ~(REG_PUT(~0, 7, 0)))   | REG_PUT(v, 7, 0))

u8 rtc_get_byte(u8 blk, u8 id) {
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

  LOG(4, "[BLK:%u] rtc_get_byte[%u]: %u(0x%02X) -> w_data: 0x%08X\n", blk, id, b, b, w_data);
  return b;
}
#define rtc_get_u16(blk, pos) (rtc_get_byte(blk, pos + 1) << 8 | rtc_get_byte(blk, pos))

void rtc_set_byte(u8 blk, u8 id, u8 b) {
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
  
  LOG(4, "[BLK:%u] rtc_set_byte[%u]: %u(0x%02X) -> w_data: 0x%08X\n", blk, id, b, b, w_data);
  system_rtc_mem_write(blk, &w_data, sizeof(w_data));  
}
#define rtc_set_u16(blk, v, pos) \
{\
    rtc_set_byte(blk, pos, v & 0xff);\
    rtc_set_byte(blk, pos + 1, v >> 8);\
}

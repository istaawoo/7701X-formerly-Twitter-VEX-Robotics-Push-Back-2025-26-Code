/* Minimal lv_conf.h for PROS build: only minimal macros to let lvgl headers compile.
   This file intentionally keeps features off. You can expand later with full lv_conf.h settings.
*/

#ifndef LV_CONF_H
#define LV_CONF_H

/* Basic display color depth and resolution defaults (safe choices) */
#define LV_COLOR_DEPTH 16
#define LV_HOR_RES_MAX 480
#define LV_VER_RES_MAX 320

/* Memory options */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (32U * 1024U)

/* Logging and debugging */
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_USE_PERF_MONITOR 0

/* Disable most optional features to minimize build footprint */
#define LV_USE_USER_DATA 0
#define LV_USE_ASYNC 0
#define LV_USE_GPU 0
#define LV_USE_FILESYSTEM 0
#define LV_USE_FS_STD 0

/* Input device options */
#define LV_USE_GROUP 0
#define LV_USE_INDEV 0

/* Widgets / features - turn off large subsystems to keep compile light.
   Enable what you need later. */
#define LV_USE_LABEL 1
#define LV_USE_BTN 0
#define LV_USE_IMG 0
#define LV_USE_CANVAS 0
#define LV_USE_DRAW_MASKS 0

#endif /* LV_CONF_H */

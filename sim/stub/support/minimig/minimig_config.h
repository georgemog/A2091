#pragma once
#include <stdint.h>
typedef struct { unsigned char cfg; unsigned char reserved; char filename[1024]; } mm_hardfileTYPE;
typedef struct {
	uint8_t cpu;
	uint8_t scsi_cfg;
	mm_hardfileTYPE scsi[6];
	uint8_t a2091_cfg;
	mm_hardfileTYPE a2091_hdd[7];
} mm_configTYPE;
extern mm_configTYPE minimig_config;

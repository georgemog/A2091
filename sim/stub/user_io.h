#pragma once
void diskled_on();
const char *user_io_get_core_path(const char *suffix = 0, int recheck = 0);
#define HomeDir(x) user_io_get_core_path(x)

#pragma once

// Runs one of the APP_* bench modes from config_defaults.h. Never returns.
// Which one to use when, and what the output should look like: TESTING.md.
void bench_run(int app_mode);

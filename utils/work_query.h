#pragma once
#include <zephyr/kernel.h>
struct k_work_q *work_query_get_workq(void);

int work_query_init(void);

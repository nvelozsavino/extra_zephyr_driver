
#include "work_query.h"

#include <zephyr/zephyr.h>

#define QUERY_WORKQ_STACK_SIZE      (1024)
#define QUERY_WORKQ_THREAD_PRIORITY 6

K_THREAD_STACK_DEFINE(workq_stack_area, QUERY_WORKQ_STACK_SIZE);

static struct k_work_q workq;

struct k_work_q *work_query_get_workq(void) {
    return &workq;
}

int work_query_init(void) {
    static bool initialized = false;

    if (!initialized) {
        k_work_queue_init(&workq);
        k_work_queue_start(&workq, workq_stack_area, K_THREAD_STACK_SIZEOF(workq_stack_area),
                           QUERY_WORKQ_THREAD_PRIORITY, NULL);
        k_thread_name_set(&workq.thread, "query");
        initialized = true;
    }

    return 0;
}

#include "board.h"

void inv_set_duty(abc_dq_t * voltage)
{
}

void inv_get_current(abc_dq_t * current)
{
    current->a = 1.0;
    current->b = -0.5;
    current->c = -0.5;
}



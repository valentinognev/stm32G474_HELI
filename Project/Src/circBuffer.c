
#include "circBuffer.h"
#include <stdint.h>

void circ_buf_init(circ_buf_t *q)
{
    q->first = q->last = 0;
    q->circBufferLen = MAXCIRCBUFFERLEN;
}

bool circ_buf_set_len(circ_buf_t *q, int len)
{
    if (len > MAXCIRCBUFFERLEN)
    {
        return false;
    }
    q->circBufferLen = len;
    q->first = q->last = 0;
    return true;
}

bool circ_buf_is_empty(const circ_buf_t *q)
{
    return q->first == q->last;
}

bool circ_buf_is_full(const circ_buf_t *q)
{
    return (q->last + 1) % q->circBufferLen == q->first;
}

bool circ_buf_enqueue(circ_buf_t *q, const scalarMeasurement_t *data)
{
    float len = q->circBufferLen;
    if (circ_buf_is_full(q))
    {
         q->first = (q->first + 1) % q->circBufferLen;
    }
    else
    {
        len=q->last - q->first+1;
    }
    q->data[q->last] = *data;
    q->last = (q->last + 1) % q->circBufferLen;
 
    return !circ_buf_is_full(q);
}

bool circ_buf_dequeue(circ_buf_t *q, scalarMeasurement_t *data)
{
    if (circ_buf_is_empty(q))
    {
        return false;
    }

    *data = q->data[q->first];
    q->first = (q->first + 1) % q->circBufferLen;
    return true;
}

bool circ_buf_last(const circ_buf_t *q, scalarMeasurement_t *data)
{
    if (circ_buf_is_empty(q))
    {
        return false;
    }

    *data = q->data[(q->circBufferLen + q->last - 1)% q->circBufferLen];
    return true;
}

bool circ_buf_first(const circ_buf_t *q, scalarMeasurement_t *data)
{
    if (circ_buf_is_empty(q))
    {
        return false;
    }

    *data = q->data[q->first];
    return true;
}


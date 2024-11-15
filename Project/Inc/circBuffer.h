#ifndef CIRCVZBUFFER_H_
#define CIRCVZBUFFER_H_

#include <stdbool.h>
#include "main.h"

#define MAXCIRCBUFFERLEN 3

typedef int32_t scalarMeasurement_t;

typedef struct
{
    uint32_t data[MAXCIRCBUFFERLEN];
    uint8_t first;
    uint8_t last;
    uint8_t circBufferLen;
} circ_buf_t;

void circ_buf_init(circ_buf_t *q);
bool circ_buf_is_empty(const circ_buf_t *q);
bool circ_buf_is_full(const circ_buf_t *q);
bool circ_buf_set_len(circ_buf_t *q, int len);
bool circ_buf_enqueue(circ_buf_t *q, const scalarMeasurement_t* data);
bool circ_buf_dequeue(circ_buf_t *q, scalarMeasurement_t *data);
bool circ_buf_last(const circ_buf_t *q, scalarMeasurement_t *data);
bool circ_buf_first(const circ_buf_t *q, scalarMeasurement_t *data);

bool circ_buf_mean(const circ_buf_t *q, float *mean);
bool circ_buf_mean_stddev(const circ_buf_t *q, float *mean, float *stdDev);
bool circ_buf_is_outlier(const circ_buf_t *q, float mean, float stdDev, float measVz);

#endif /* CIRCVZBUFFER_H_ */
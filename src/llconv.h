#ifndef LLCONV_H
#define LLCONV_H

void lltot(double lat, double lng, int* tx, int* tz, float *x, float *z);
void ttoll(int tx, int tz, double x, double z, double* lat, double* lng);

#endif

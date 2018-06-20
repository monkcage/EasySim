#ifndef READPAR_H
#define READPAR_H

#ifdef __cplusplus
extern "C" {
#endif

extern void readpar ( char *sw, char *ifn,
                      int *intValue[], char *intName[], int intNum,
                      float *floatValue[], char *floatName[], int floatNum,
                      char *stringValue[], char *stringName[], int stringNum );

#ifdef __cplusplus
}
#endif

#endif // READPAR_H

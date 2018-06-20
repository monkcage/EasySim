#ifndef NRUTIL_H
#define NRUTIL_H

#ifdef __cplusplus
extern "C" {
#endif


extern void free_ivector(int *v, long nl, long nh);
extern void free_vector(float *v, long nl, long nh);
extern int *ivector(long nl, long nh);


#ifdef __cplusplus
}
#endif

#endif // NRUTIL_H

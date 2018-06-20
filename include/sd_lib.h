#ifndef SD_LIB_H
#define SD_LIB_H



#ifdef __cplusplus
extern "C" {
#endif
#include <pthread.h>
#include <unistd.h>

extern pthread_mutex_t mutex_xs;
extern pthread_mutex_t mutex_ys;



extern void Init_Env();

extern void Init_Bare(char *init_switch);
extern void Init_Demo();
extern void ReInit();
extern void Save_Demo();
extern void Upd();
extern float *get_xs();
extern float *get_ys();
extern float *get_diameter();
extern int get_updnum();
extern int get_maxupdnum();
extern float get_maxsimtime();
extern float get_smoketime();
extern float get_vsmoke();
extern float get_simtime();
extern float get_fire_dis();
extern int *get_injured();
extern int get_injured_num();
extern int get_ninroom();
extern int get_n();

extern void set_particle_num(int num);
extern void set_room_size(float x, float y);
extern void set_door_width(float val);
extern void set_wall_width(float val);
extern void set_v0(float val);
extern void enable_fire(int val);
extern void set_fire_v0(float val);

//extern float *D;
//extern float *X;
//extern float *Y;

#ifdef __cplusplus
}
#endif


#endif // SD_LIB_H



#include "sd_lib.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <unistd.h>
#include "base.c"



#define SD_LIB_EXIT {_E("sd_lib.c: Exiting to system.\n");exit(-1);}
#define MY_STRLEN 200
/* number of walls and wpoints */
#define NW 9
#define NWP 4

static float EPSILON = (float)1.0e-5;
pthread_mutex_t mutex_xs;


typedef struct wall { float x1, y1, x2, y2; } wall;
typedef struct wpoint { float x, y; } wpoint;

/************** constants needed for sd.c only ***************/
static int MainSwitch_DEFAULT = 0;
static char *IFN_DEFAULT = "./sd.par";
static char *OFN_DEFAULT = "./sd.dat";
static char *OF2N_DEFAULT = "./sd.dat2";


extern void readpar ( char *sw, char *ifn,
                      int *intValue[], char *intName[], int intNum,
                      float *floatValue[], char *floatName[], int floatNum,
                      char *stringValue[], char *stringName[], int stringNum );
extern float *vector(long nl, long nh);
extern int *ivector(long nl, long nh);
extern void free_vector(float *v, long nl, long nh);

/********* global parameters -- to be read from parameter file **********/

static int N0, FrictionSwitch, InjurySwitch, ColumnSwitch,
   X11_RightRim, RndSeed, MaxUpdNum, AyS;
static float RoomXSize, RoomYSize, DoorWidth, WallWidth, Dmean,
  deltaD, A, B, A_fire, B_fire, Kappa, Gamma, C_Young, R, R_fire, V0, Tau,
  GaMe, GaTh, GaCM,
  SmokeStartTime, VSmoke, FCrush_over_1m,
  ColumnCenterX, ColumnCenterY, ColumnD,
  X11_Magn, MaxSimTime, Vmax, H, DefaultDeltaT, C_NS, V_ChangeLimit;


static float gtstep = 0.01f;

int *IPar[]={&N0, &FrictionSwitch, &InjurySwitch, &ColumnSwitch,
         &X11_RightRim, &RndSeed, &MaxUpdNum, &AyS};
char *IParName[]={"N0", "FrictionSwitch", "InjurySwitch", "ColumnSwitch",
          "X11_RightRim", "RndSeed", "MaxUpdNum", "AyS"};
float *FPar[]={&RoomXSize, &RoomYSize, &DoorWidth, &WallWidth, &Dmean,
           &deltaD, &A, &B, &A_fire, &B_fire, &Kappa, &Gamma, &C_Young,
           &R, &R_fire, &V0, &Tau, &GaMe, &GaTh, &GaCM, &SmokeStartTime,
           &VSmoke, &FCrush_over_1m, &ColumnCenterX, &ColumnCenterY,
           &ColumnD, &X11_Magn, &MaxSimTime, &Vmax, &H, &DefaultDeltaT,
           &C_NS, &V_ChangeLimit};
char *FParName[]={"RoomXSize", "RoomYSize", "DoorWidth", "WallWidth", "Dmean",
          "deltaD", "A", "B", "A_fire", "B_fire","Kappa", "Gamma", "C_Young",
          "R", "R_fire", "V0", "Tau", "GaMe", "GaTh", "GaCM", "SmokeStartTime",
          "VSmoke", "FCrush_over_1m", "ColumnCenterX", "ColumnCenterY",
          "ColumnD", "X11_Magn",  "MaxSimTime", "Vmax", "H", "DefaultDeltaT",
          "C_NS", "V_ChangeLimit"};
//char *SPar[]={BackGroundColorName, InfoColorName};
//char *SParName[]={"BackGroundColorName", "InfoColorName"};


static int IParNum = sizeof(IPar)/sizeof(int*),
    FParNum = sizeof(FPar)/sizeof(float*);
//    SParNum = sizeof(SPar)/sizeof(char*);

/******* global variables needed both for sd.c and sd_crunch.c ************/

static int MainSwitch, UpdNum, N, GX, GY, G, Mb, Me, *BIndBd, *BInd,
  GaussFlag, NInRoom, *Injured, NInjured;
static float *SimTime, XS, YS, *D, *Phi, *X, *Y, *Xprev, *Yprev, *V, *VX,
  *VY, *E, *Vdir, GaussSet1, GaussSet2, *V0of, sim_time;

//float *tmp_x, *tmp_y;

char IFN[MY_STRLEN], OFN[MY_STRLEN], OF2N[MY_STRLEN];
FILE *OFP,*OF2P;
wall *W;
wpoint *WP;

struct stat IFStatBuf;
long int IFModTime;

/* MainSwitch (see in sd.c)
   UpdNum     index of present update
   N          number of particles
   IFN, IFP   input file name, input file pointer
   IFStatBuf  input file status buffer
   IFModTime  modification time of input file
   X11_WWi/X11_WHe
              window width/height
   SimTime[n]    time after the nth update (SimTime[0] = 0.0)

   X[i]       current coordinate of particle i
   Xprev[i]   previous coordinate
   VX[i]      current velocity component of particle i
   V[i]       magnitude of current velocity of particle i
   Phi[i]     preferred direction of particle i
   Vdir[i]    direction of particle i's velocity
   V0of[i]    preferred magnitude of velocity of particle i

   E[n]       is the efficiency at update step no. n
   Mb/Me      mem.alloc. begin/end
              ie. limits of current time window
          (see explanation of AyS in sd.par)
   XS,YS      size of field
   GaussFlag,GaussSet1,GaussSet2
              are used by the random number generator
   NInRoom    number of particles in room
   Injured[i] =1, if pedestrian i is injured
              =0, if not
   NInjured   number of injured

BOOK-KEEPING OF PARTICLES:
  GX*GY blocks (all same size)
  the size of a block should be: not less than R, i.e.
  GX = (int)floor(XS/R) i.e. R <= XS/GX
  indexed with 0, 1, ... , G^2 - 1, where G = max(GX,GY)
  block indices of particles stored in BIndBd & BInd

  BIndBd[n] (Block + Index + Board)
             = 0,1,...,N-1: the number of the nth block's first particle
         = -1: there's no particle in the nth block
  BInd[i]   = 0,1,...,N-1: the next particle in the block of particle i
            = -1: there are no more particles in this block
        */



/* FW_x x component of force exerted on walls left and right from exit */
static float FW_x;


/***************** prototypes **********************/
/* functions needed in all cases */

void WallParticleRelation(int iw, int i, float *tr, int *can_see);
void WallPsychForce(int iw, int i, float r, float *fx, float *fy);
void WallYoungForce(int iw, int i, float r, float *fx, float *fy);
void WallTangForce_FS0(int iw, int i, float *fx, float *fy);
void WallTangForce_FS1(int iw, int i, float r, float *fx, float *fy);

void WPointParticleRelation(int iwp, int i, float *r, int *can_see);
void WPointPsychForce(int iwp, int i, float r, float *fx, float *fy);
//static void WPointYoungForce(int iwp, int i, float r, float *fx, float *fy);
//void WPointTangForce_FS0(int iwp, int i, float r, float *fx, float *fy);
//void WPointTangForce_FS1(int iwp, int i, float r, float *fx, float *fy);

//void PP_PsychForce(int i1, int i2, float r, float *fx, float *fy);
//void PP_YoungForce(int i1, int i2, float r, float *fx, float *fy);
//void PP_TangForce_FS0(int i1, int i2, float r, float *fx, float *fy);
void PP_TangForce_FS1(int i1, int i2, float r, float *fx, float *fy);

float DirectionOfExit( int i );
void RemoveParticle( int *n, int i );
void EulTStep( float *tstep, float f );


void Start_Bare( int narg, char *argstr[] );
void Init_Bare( char *init_switch );
void Upd();
float GaussRand( float gmean, float gtheta, float gcutmult );
float EMean( char* sw, int cnfreq, float stfreq );



/* functions needed only for sd.c */

void Start_Demo( int narg, char *argstr[] );
void Init_Demo();


/********************* definitions *****************/

/*
** can_see : Whether particle i is within the range of wall iw
** r : distance
*/
void WallParticleRelation(int iw, int i, float *r, int *can_see){
    switch(iw){
    case 0:{ *r = Y[i];           break; }
    case 1:{ *r = WP[0].x-X[i];   break; }
    case 2:{ *r = Y[i]-WP[0].y;   break; }
    case 3:{ *r = X[i]-WP[1].x;   break; }
    case 4:{ *r = RoomYSize-Y[i]; break; }
    case 5:{ *r = X[i]-WP[2].x;   break; }
    case 6:{ *r = WP[2].y-Y[i];   break; }
    case 7:{ *r = WP[3].x-X[i];   break; }
    case 8:{ *r = X[i];           break; }
    }

    int flag = 0;
    switch(iw){
    case 0:{ flag = Y[i] <= R;  break; }
    case 1:{ flag = (X[i]>=WP[0].x-R)&&(X[i]<=WP[0].x)&&(Y[i]<=WP[0].y); break; }
    case 2:{ flag = (X[i]>=WP[0].x)&&(X[i]<=WP[1].x)&&(Y[i]<=WP[0].y+R); break; }
    case 3:{ flag = (X[i]>=WP[1].x)&&(X[i]<=WP[1].x+R)&&(Y[i]<=WP[1].y); break; }
    case 4:{ flag = Y[i]>=RoomYSize-R; break; }
    case 5:{ flag = (X[i]>=WP[2].x)&&(X[i]<=WP[2].x+R)&&(Y[i]>=WP[2].y); break; }
    case 6:{ flag = (X[i]>=WP[3].x)&&(X[i]<=WP[2].x)&&(Y[i]>=WP[2].y-R); break; }
    case 7:{ flag = (X[i]<=WP[3].x)&&(X[i]>=WP[3].x-R)&&(Y[i]>=WP[3].y); break; }
    case 8:{ flag = X[i]<=R; break; }
    }
    *can_see = flag ? 1 : 0;
}


void WallPsychForce(int iw, int i, float r, float *fx, float *fy){
#define tmp_f (A*exp(-(r-0.5*D[i])/B))
    switch(iw){
    case 0:{ *fx = 0.0;     *fy = tmp_f;   break; }
    case 1:{ *fx = - tmp_f; *fy = 0.0;     break; }
    case 2:{ *fx = 0.0;     *fy = tmp_f;   break; }
    case 3:{ *fx = tmp_f;   *fy = 0.0;     break; }
    case 4:{ *fx = 0.0;     *fy = - tmp_f; break; }
    case 5:{ *fx = tmp_f;   *fy = 0.0;     break; }
    case 6:{ *fx = 0.0;     *fy = - tmp_f; break; }
    case 7:{ *fx = - tmp_f; *fy = 0.0;     break; }
    case 8:{ *fx = tmp_f;   *fy = 0.0;     break; }
    }
#undef tmp_f
}


void WallYoungForce(int iw, int i, float r, float *fx, float *fy){
#define tmp_f (2.0*C_Young*(0.5*D[i]-r))
    switch(iw){
    case 0:{ *fx = 0.0;     *fy = tmp_f;   break; }
    case 1:{ *fx = - tmp_f; *fy = 0.0;     break; }
    case 2:{ *fx = 0.0;     *fy = tmp_f;   break; }
    case 3:{ *fx = tmp_f;   *fy = 0.0;     break; }
    case 4:{ *fx = 0.0;     *fy = - tmp_f; break; }
    case 5:{ *fx = tmp_f;   *fy = 0.0;     break; }
    case 6:{ *fx = 0.0;     *fy = - tmp_f; break; }
    case 7:{ *fx = - tmp_f; *fy = 0.0;     break; }
    case 8:{ *fx = tmp_f;   *fy = 0.0;     break; }
    }
#undef tmp_f
}


void WallTangForce_FS0(int iw, int i, float *fx, float *fy){
    switch(iw){
    case 0: case 2: case 4: case 6: {
        *fx = -Gamma*VX[i];
        *fy = 0.0;
        break;
    }
    case 1: case 3: case 5: case 7: case 8: {
        *fx = 0.0;
        *fy = -Gamma*VY[i];
        break;
    }
    }
}


void WallTangForce_FS1( int iw, int i, float r, float *fx, float *fy ){
#define tmp_delta_r (0.5*D[i]-r)
  /* friction forces */
    switch(iw){
    case 0: case 2: case 4: case 6: {
        *fx = -Kappa*tmp_delta_r*VX[i];
        *fy = 0.0;
        break;
    }
    case 1: case 3: case 5: case 7: case 8: {
        *fx = 0.0;
        *fy = -Kappa*tmp_delta_r*VY[i];
        break;
    }
    }
#undef tmp_delta_r
}


/*
** can_see : Whether particle i is within the range of wpoint iwp
** r : distance
*/
void WPointParticleRelation(int iwp, int i, float *r, int *can_see)
{
    *r = sqrt(SQR(WP[iwp].x-X[i])+SQR(WP[iwp].y-Y[i]));
    int flag=0;
    switch(iwp){
    case 0:{ flag = (X[i]<=WP[0].x)&&(Y[i]>=WP[0].y); break; }
    case 1:{ flag = (X[i]>=WP[1].x)&&(Y[i]>=WP[1].y); break; }
    case 2:{ flag = (X[i]>=WP[2].x)&&(Y[i]<=WP[2].y); break; }
    case 3:{ flag = (X[i]<=WP[3].x)&&(Y[i]<=WP[3].y); break; }
    }
    *can_see = flag ? 1 : 0;
}

/*
** exerted by wpoint iwp on particle i
*/
void WPointPsychForce(int iwp, int i, float r, float *fx, float *fy)
{
#define tmp_f_over_r (A*exp(-(r-0.5*D[i])/B)/r)
    *fx = (X[i]-WP[iwp].x) * tmp_f_over_r;
    *fy = (Y[i]-WP[iwp].y) * tmp_f_over_r;
#undef tmp_f_over_r
}

/*
** exerted by wpoint iwp on particle i
*/
static inline
void WPointYoungForce(int iwp, int i, float r, float *fx, float *fy)
{
    float rx,ry;
#define tmp_f_over_r ( 2.0*C_Young*(0.5*D[i]-r) / r)
    rx=WP[iwp].x-X[i];
    ry=WP[iwp].y-Y[i];
    *fx = - rx * tmp_f_over_r;
    *fy = - ry * tmp_f_over_r;
#undef tmp_f_over_r
}

/*
** exerted by wpoint iwp on particle i
*/

static inline
void WPointTangForce_FS0(int iwp, int i, float r, float *fx, float *fy)
{
    float rx,ry,scal_prod_over_rsqr;
    rx = X[i]-WP[iwp].x;
    ry = Y[i]-WP[iwp].y;
    scal_prod_over_rsqr = (ry*VX[i] - rx*VY[i]) / SQR(r);
    *fx = -Gamma * (   ry * scal_prod_over_rsqr );
    *fy = -Gamma * ( - rx * scal_prod_over_rsqr );
}

/*
** exerted by wpoint iwp on particle i
*/
static inline
void WPointTangForce_FS1(int iwp, int i, float r, float *fx, float *fy)
{
    float rx,ry,scal_prod_over_rsqr;
    rx = X[i]-WP[iwp].x;
    ry = Y[i]-WP[iwp].y;
    scal_prod_over_rsqr = (ry*VX[i] - rx*VY[i]) / SQR(r);
    *fx = -Kappa * (0.5*D[i]-r) * (   ry * scal_prod_over_rsqr );
    *fy = -Kappa * (0.5*D[i]-r) * ( - rx * scal_prod_over_rsqr );
}

static inline
void PP_PsychForce(int i1, int i2, float r, float *fx, float *fy)
{
    float f_over_r;
    f_over_r = A*exp(-(r-0.5*(D[i1]+D[i2]))/B) / r;
    *fx = (X[i1]-X[i2]) * f_over_r;
    *fy = (Y[i1]-Y[i2]) * f_over_r;
}


static inline
void PP_YoungForce(int i1, int i2, float r, float *fx, float *fy)
{
    float f_over_r;
    f_over_r = 2.0*C_Young*(0.5*(D[i1]+D[i2])-r) / r;
    *fx = (X[i1]-X[i2]) * f_over_r;
    *fy = (Y[i1]-Y[i2]) * f_over_r;
}


/*
** exerted by particle i2 on particle i1
*/
static inline
void PP_TangForce_FS0(int i1, int i2, float r, float *fx, float *fy)
{
    float rx,ry,vx,vy,scal_prod_over_rsqr;
    rx = X[i1]-X[i2];
    ry = Y[i1]-Y[i2];
    vx = VX[i1]-VX[i2];
    vy = VY[i1]-VY[i2];
    scal_prod_over_rsqr = (ry*vx - rx*vy) / SQR(r);
    *fx = -Gamma * (   ry * scal_prod_over_rsqr );
    *fy = -Gamma * ( - rx * scal_prod_over_rsqr );
}

/*
** exerted by particle i2 on particle i1
*/
void PP_TangForce_FS1(int i1, int i2, float r, float *fx, float *fy)
{
    float rx,ry,vx,vy,scal_prod_over_rsqr;

    rx = X[i1]-X[i2];
    ry = Y[i1]-Y[i2];
    vx = VX[i1]-VX[i2];
    vy = VY[i1]-VY[i2];
    scal_prod_over_rsqr = (ry*vx - rx*vy) / SQR(r);
    *fx = -Kappa * (0.5*(D[i1]+D[i2])-r) * (   ry * scal_prod_over_rsqr );
    *fy = -Kappa * (0.5*(D[i1]+D[i2])-r) * ( - rx * scal_prod_over_rsqr );
}

/*
** direction of exit for particle i
*/
float DirectionOfExit( int i )
{
    float dsqr, /* sqr of particle center - door-post distance */
    rsqr; /* sqr of particle's radius */

  /* behind the upper door-post */
    if((Y[i]<=0.5*YS-0.5*DoorWidth+0.5*D[i]+EPSILON)&&(X[i]<=RoomXSize)){
        dsqr = SQR(W[1].x2-X[i]) + SQR(W[1].y2-Y[i]);
        rsqr = SQR(0.5*D[i])+EPSILON;
        if(dsqr<=rsqr){
            /* very close to the door-post */
            if(Y[i]<=0.5*YS-0.5*DoorWidth){
                return( 0.5*PI );
            } else {
                  return(   0.5*PI
                  + atan2( W[1].y2-Y[i],W[1].x2-X[i] ));
            }
        } else {
              /* well apart from the door-post */
            return( atan2( 1.0, sqrt(dsqr/rsqr-1.0) )
                     + atan2( W[1].y2-Y[i],W[1].x2-X[i] ));
        }
    }
    /* behind the lower door-post */
    else if((Y[i]>=0.5*YS+0.5*DoorWidth-0.5*D[i]-EPSILON)&&(X[i]<=RoomXSize)){
        dsqr = SQR(W[6].x2-X[i]) + SQR(W[6].y2-Y[i]);
        rsqr = SQR(0.5*D[i])+EPSILON;
        if(dsqr<=rsqr){
            /* very close to the door-post */
            if(Y[i]>=0.5*YS+0.5*DoorWidth){
                return( -0.5*PI );
            } else {
                return( - 0.5*PI
                  + atan2( W[6].y2-Y[i],W[6].x2-X[i] ));
            }
        } else {
              /* well apart from the door-post */
              return( - atan2( 1.0, sqrt(dsqr/rsqr-1.0) )
                          + atan2( W[6].y2-Y[i],W[6].x2-X[i] ) );
        }
    }
    /* in the center or outside */
    else { return 0.0; }
}

/*
** (a) : particle i (which is off board now) is removed
**       from the book-keeping (block determined by previous coordinates)
** (b) : if i != *n-1
**     (b1) particle *n - 1 is removed from the book-keeping
**	      (block determined by previous coordinates)
**     (b2) copying all values of particle *n-1 into i's place
**     (b3) inserting particle i (that used to be indexed *n-1) into the
**           book-keeping, into the block given by the previous
**           coordinates (Xprev[i],Yprev[i]), and not into the block
**           given by (X[i],Y[i])
**          . reason: after this substitution (*n-1 -> i)
**          particle i will be looked for in the block of
**         (Xprev[i],Yprev[i]) in Upd
**          because no one tells the main cycle (located in Upd),
**          whether this particle is the result of a substitution
**         or not
** (c) decrement particle number  ( *n to *n - 1 )
**
** *n : number of particles now
**  i : index of particles to be removed
*/
void RemoveParticle( int *n, int i )
{
    int j;

    /* a */
    j = (int)floor(Xprev[i]*GX/XS) + G*(int)floor(Yprev[i]*GY/YS);
    if(BIndBd[j]==i) {
        BIndBd[j] = BInd[i];
    } else {
        j = BIndBd[j];
        while(BInd[j]!=i) { j = BInd[j]; }
        BInd[j] = BInd[i];
    }



    /* b */
    if(i!=*n-1){
        /* b1 */
        j = (int)floor(Xprev[*n-1]*GX/XS) + G*(int)floor(Yprev[*n-1]*GY/YS);
        if(BIndBd[j]==*n-1) {
            BIndBd[j] = BInd[*n-1];
        } else {
            j = BIndBd[j];
            while(BInd[j]!=*n-1) {
                j = BInd[j];
            }
            BInd[j] = BInd[*n-1];
        }

        /* b2 */
        D[i] = D[*n-1];
        Phi[i] = Phi[*n-1];
        X[i] = X[*n-1];
        Y[i] = Y[*n-1];
        V[i] = V[*n-1];
        VX[i] = VX[*n-1];
        VY[i] = VY[*n-1];
        Xprev[i] = Xprev[*n-1];
        Yprev[i] = Yprev[*n-1];
        Vdir[i]=Vdir[*n-1];
        Injured[i]=Injured[*n-1];
        V0of[i]=V0of[*n-1];

        /* b3 */
        j = (int)floor(Xprev[i]*GX/XS) + G*(int)floor(Yprev[i]*GY/YS);
        if(BIndBd[j]==-1) {
            BIndBd[j] = i;
            BInd[i] = -1;
        } else {
            j = BIndBd[j];
            while(BInd[j]!=-1) { j = BInd[j]; }
            BInd[j] = i;
            BInd[i] = -1;
        }
    }

    /* c */
    (*n)--;
}

/*-----------------------------------------------------------------------*/

void EulTStep( float *tstep, float f )
{
  /* adjusts the time step in a way that the force (fx,fy) doesn't
     change the velocity of particle i by more than V_ChangeLimit */
  while( f*(*tstep) >= V_ChangeLimit ){ *tstep *= C_NS; }
}

/********************************/

void Init_Env()
{
    /* Dummy copy */
    MainSwitch = MainSwitch_DEFAULT;
    strcpy(IFN,IFN_DEFAULT);
    strcpy(OFN,OFN_DEFAULT);
    strcpy(OF2N,OF2N_DEFAULT);

    fprintf(stderr,"(default values are:\n MainSwitch = %d, input = %s, output = %s, %s)\n", MainSwitch,IFN,OFN,OF2N);
    fflush(stderr);

    /* reading parameters */
    readpar ( "start", IFN_DEFAULT, IPar, IParName, IParNum,
         FPar, FParName, FParNum, /*SPar*/NULL, /*SParName*/NULL, /*SParNum*/0 );
}

int get_n() {return N;}
int get_updnum() { return UpdNum; }
int get_maxupdnum() { return MaxUpdNum; }
float get_maxsimtime() {return MaxSimTime;}
float get_simtime() { return sim_time; }
float get_smoketime() {return SmokeStartTime;}
float get_vsmoke() {return VSmoke;}
float *get_xs() { return X; }
float *get_ys() { return Y; }
float *get_diameter() { return D; }
int *get_injured() { return Injured;}

//float get_injured(int idx) { return Injured[idx]; }
float get_fire_dis() { return (sim_time - SmokeStartTime)*VSmoke; }
int get_injured_num() { return NInjured; }
int get_ninroom() { return NInRoom; }

void set_room_size(float x, float y)
{
    RoomXSize = x;
    RoomYSize = y;
}


void set_door_width(float val) { DoorWidth = val; }
void set_wall_width(float val) { WallWidth = val; }
void set_v0(float val) { V0 = val; }
void enable_fire(int val) { InjurySwitch = val; }
void set_fire_v0(float val) { VSmoke = val; }
void set_particle_num(int num) { N0 = num; }



void init_walls()
{
    W = (wall*)calloc(NW,sizeof(wall));
    WP = (wpoint*)calloc(NWP,sizeof(wpoint));

    /* every wall rotated by PI/2 points towards the inside of the room */

    /* upper part */
    W[0].x1 = 0.0;
    W[0].y1 = 0.0;
    W[0].x2 = XS;
    W[0].y2 = 0.0;

    W[1].x1 = RoomXSize;
    W[1].y1 = 0.0;
    W[1].x2 = W[2].x1 = WP[0].x = RoomXSize;
    W[1].y2 = W[2].y1 = WP[0].y = 0.5*RoomYSize-0.5*DoorWidth;
    W[2].x2 = W[3].x1 = WP[1].x = RoomXSize+WallWidth;
    W[2].y2 = W[3].y1 = WP[1].y = 0.5*RoomYSize-0.5*DoorWidth;
    W[3].x2 = RoomXSize+WallWidth;
    W[3].y2 = 0.0;


    /* lower part */
    W[4].x1 = XS;
    W[4].y1 = RoomYSize;
    W[4].x2 = 0.0;
    W[4].y2 = RoomYSize;

    W[5].x1 = RoomXSize+WallWidth;
    W[5].y1 = RoomYSize;

    W[5].x2 = W[6].x1 = WP[2].x = RoomXSize+WallWidth;
    W[5].y2 = W[6].y1 = WP[2].y = 0.5*RoomYSize+0.5*DoorWidth;
    W[6].x2 = W[7].x1 = WP[3].x = RoomXSize;
    W[6].y2 = W[7].y1 = WP[3].y = 0.5*RoomYSize+0.5*DoorWidth;
    W[7].x2 = RoomXSize;
    W[7].y2 = RoomYSize;

    /* left wall of the room */
    W[8].x1 = 0.0;
    W[8].y1 = RoomYSize;
    W[8].x2 = 0.0;
    W[8].y2 = 0.0;
}

/*----------------------------------------*/

void Init_Bare( char *init_switch ){
  /* 1 global vars, mem.alloc.
     2 walls, wpoints
     3 particles
   */

  int i,j,ok_flag;

  /* 1 */
  stat(IFN, &IFStatBuf);
  IFModTime = IFStatBuf.st_mtime;
  UpdNum = 0;
  Mb = 0;
  Me = AyS-1;
  SimTime = vector( Mb, Me );
  SimTime[0] = 0.0;

  sim_time = 0.0;
  srand(RndSeed);

  XS = RoomXSize+WallWidth+X11_RightRim+EPSILON;
  YS = RoomYSize;
  N = N0;
  NInRoom = N0;

  GX = (int)MAX(1.0,floor(XS/R));
  GY = (int)MAX(1.0,floor(YS/R));
  G = (int)MAX(GX,GY);

  BIndBd = ivector( 0, SQR(G)-1 );
  BInd = ivector( 0, N0-1 );
  D = vector( 0, N0-1 );
  Phi = vector( 0, N0-1 );
  X = vector( 0, N0-1 );
  Y = vector( 0, N0-1 );

  Xprev = vector( 0, N0-1 );
  Yprev = vector( 0, N0-1 );
  V = vector( 0, N0-1 );
  VX = vector( 0, N0-1 );
  Vdir = vector(0,N0-1);
  VY = vector( 0, N0-1 );
  V0of = vector( 0, N0-1 );
  E = vector( Mb, Me );
  E[0] = 1.0;
  Injured = ivector(0,N0-1);


  /* 2 walls, wpoints */
  /* allocating memory:
   * if there's a column at the door,
   * the four faces and corners of the column have to be initialized,
   * too
   */
  init_walls();


  /* 3 */
  /* diameters and coordinates */
  for(i=0; i<N; i++) {
      D[i] =   (Dmean + deltaD)
         - 2.0*deltaD * rand()/(RAND_MAX+1.0);
      X[i] =   0.5*H*D[i]+EPSILON
         + (RoomXSize-H*D[i]-2.0*EPSILON)*rand()/(RAND_MAX+1.0);
      Y[i] =   0.5*H*D[i]+EPSILON
         + (RoomYSize-H*D[i]-2.0*EPSILON)*rand()/(RAND_MAX+1.0);

      /* checking whether far enough from the column */
      ok_flag = 1;
      switch(ColumnSwitch){
      default: case 0:{ break; }
      case 1:{
          if(   SQR(X[i]-ColumnCenterX)+SQR(Y[i]-ColumnCenterY)
         <= SQR(0.5*(D[i]+ColumnD))+EPSILON
        ){
              ok_flag = 0;
              i--;
          }
          break;
      }
      }


      /* checking distances to already existing particles */
      if(ok_flag==1){
          for(j=0;j<i;j++) {
              if(     SQR(X[j] - X[i])
                + SQR(Y[j] - Y[i])
             <= SQR( 0.5*H*(D[i]+D[j]) ) + EPSILON
            ) {
                  i = i - 1;
                  j = i - 1;
              }
          }
      }
  }


  /* book-keeping */
  for(i=0;i<SQR(G);i++){ BIndBd[i] = -1; }
  for(i=0;i<N;i++){ BInd[i] = -1; }

  for(i=0;i<N;i++) {
          j = (int)floor(X[i]*GX/XS) + G * (int)floor(Y[i]*GY/YS);
//      j = (int)floor(X[i]/R) + G * (int)floor(Y[i]/R);
         // j = (int)floor(X[i]/R) + G * (int)floor(Y[i]/R);

      if(BIndBd[j]==-1) {
              BIndBd[j] = i;
      }
      else {
              j = BIndBd[j];
          while(BInd[j]!=-1) {
                  j = BInd[j];
                  }
          BInd[j] = i;
      }
  }



  /* injuries, velocities and preferred directions */
  NInjured = 0;
//  for(i=0;i<N;i++){ Injured[i] = 0; }

  for(i=0;i<N;i++){
      Phi[i] = DirectionOfExit( i );
      Vdir[i] = Phi[i];
//      V[i]=0.0;
      V0of[i]=V0;
//      VX[i]=0.0;
//      VY[i]=0.0;
  }
}



void calc_wall_force(float *fsumx, float *fsumy, float *ftmagsum)
{
    float tmpr, tmp_fpsx, tmp_fpsy, tmp_fyox, tmp_fyoy,
          tmp_ftax, tmp_ftay;
    int can_see;
    int i=0, iw=0;
    for(i=0;i<N;i++){
        for(iw=0;iw<NW;iw++){
            WallParticleRelation(iw,i,&tmpr,&can_see);
            if((can_see==1)&&(tmpr<=R)){

                /* init */
                tmp_fpsx = tmp_fpsy = 0.0;
                tmp_fyox = tmp_fyoy = 0.0;
                tmp_ftax = tmp_ftay = 0.0;

                /* psychological force */
                WallPsychForce(iw,i,tmpr,&tmp_fpsx,&tmp_fpsy);
                /* Young and tangential forces */
                if(tmpr<=(float)(0.5)*D[i]){
                WallYoungForce(iw,i,tmpr,&tmp_fyox,&tmp_fyoy);
                switch(FrictionSwitch){
                case 0:{
                WallTangForce_FS0(iw,i,&tmp_ftax,&tmp_ftay);
                break;
            }
            case 1:{
                WallTangForce_FS1(iw,i,tmpr,&tmp_ftax,&tmp_ftay);
                break; }
            }
        }
          /* summing wall forces */
        if(Injured[i]==0){
            *(fsumx+i) += tmp_fpsx + tmp_fyox + tmp_ftax;
            *(fsumy+i) += tmp_fpsy + tmp_fyoy + tmp_ftay;
        }
        else /* ie. if Injured[i]=1 */ {
            *(fsumx+i) += tmp_fyox + tmp_ftax;
            *(fsumy+i) += tmp_fyoy + tmp_ftay;
        }

        /* sum of magnitude of touching forces */
        if(InjurySwitch==1){
            *(ftmagsum+i) += sqrt(SQR(tmp_fyox)+SQR(tmp_fyoy));
        }

          /* measuring x component of touching force exerted
         on walls left and right from exit
         -- only in demo mode */
          if((iw==1)||(iw==7)){
          FW_x -= tmp_fyox + tmp_ftax;
          }
      }
      }
  }
}


void calc_wall_point_force(float *fsumx, float *fsumy, float *ftmagsum)
{
    float tmpr, tmp_fpsx, tmp_fpsy, tmp_fyox, tmp_fyoy,
          tmp_ftax, tmp_ftay;
    int i, iwp, can_see;
    for(i=0;i<N;i++){
        for(iwp=0;iwp<NWP;iwp++){

            WPointParticleRelation(iwp,i,&tmpr,&can_see);
            if((can_see==1)&&(tmpr<=R)){

                /* init */
                tmp_fpsx = tmp_fpsy = 0.0;
                tmp_fyox = tmp_fyoy = 0.0;
                tmp_ftax = tmp_ftay = 0.0;

                /* computing forces */
                WPointPsychForce(iwp,i,tmpr,&tmp_fpsx,&tmp_fpsy);
                if(tmpr<=(float)0.5*D[i]){
                    WPointYoungForce(iwp,i,tmpr,&tmp_fyox,&tmp_fyoy);
                    switch(FrictionSwitch){
                    case 0:{
                        WPointTangForce_FS0(iwp,i,tmpr,&tmp_ftax,&tmp_ftay);
                        break;
                    }
                    case 1:{
                        WPointTangForce_FS1(iwp,i,tmpr,&tmp_ftax,&tmp_ftay);
                        break;
                    }
                    }
                }

                /* summing forces */
                if(Injured[i]==0){
                    fsumx[i] += tmp_fpsx + tmp_fyox + tmp_ftax;
                    fsumy[i] += tmp_fpsy + tmp_fyoy + tmp_ftay;
                }
                else /* ie. if Injured[i]=1 */ {
                    fsumx[i] += tmp_fyox + tmp_ftax;
                    fsumy[i] += tmp_fyoy + tmp_ftay;
                }

                /* sum of magnitude of touching forces */
                if(InjurySwitch==1){
                    ftmagsum[i] += sqrt(SQR(tmp_fyox)+SQR(tmp_fyoy));
                }

                /* measuring x component of touching force exerted
                    on walls left and right from exit
                    -- only in demo mode */

                if((iwp==0)||(iwp==3)){
                    FW_x -= tmp_fyox + tmp_ftax;
                }
            }
        }
    }
}


void calc_pair_force(float *fsumx, float *fsumy, float *ftmagsum)
{
    int i, j, k, l, mx, my, m;
    float tmpr, tmp_fpsx, tmp_fpsy, tmp_fyox, tmp_fyoy,
          tmp_ftax, tmp_ftay, tmprsqr;
    for(i=0; i<N; i++) {

        j = (int)floor(X[i]*GX/XS) + G * (int)floor(Y[i]*GY/YS);
        for(k=-1;k<=1;k++){
            for(l=-1;l<=1;l++){
                mx = j%G+k;
                my = j/G+l;
                if((mx>=0)&&(mx<GX)&&(my>=0)&&(my<GY)){
                    m = BIndBd[ (mx+GX)%GX + G * (my%GY) ];
                    /* checking each pair of particles only once */
                    while(m>=i) { m = BInd[m]; }
                    if(m!=-1) {
                        do {
                            tmprsqr = SQR(X[i]-X[m]) + SQR(Y[i]-Y[m]);
                            if( tmprsqr <= SQR(R) ){
                                tmpr = sqrt(tmprsqr);

                                /* init */
                                tmp_fpsx = tmp_fpsy = 0.0;
                                tmp_fyox = tmp_fyoy = 0.0;
                                tmp_ftax = tmp_ftay = 0.0;

                                /* pair forces */
                                /* Force(i,m,...) gives the force exerted by m
                                    on i, all forces are symmetric now */
                                PP_PsychForce(i,m,tmpr,&tmp_fpsx,&tmp_fpsy);
                                if(tmpr<=(float)(0.5)*(D[i]+D[m])){
                                    PP_YoungForce(i,m,tmpr,&tmp_fyox,&tmp_fyoy);
                                    switch(FrictionSwitch){
                                    case 0:{
                                        PP_TangForce_FS0(i,m,tmpr,&tmp_ftax,&tmp_ftay);
                                        break;
                                    }
                                    case 1:{
                                        PP_TangForce_FS1(i,m,tmpr,&tmp_ftax,&tmp_ftay);
                                        break;
                                    }
                                    }
                                }

                                /* summing forces */
                                if(Injured[i]==0){
                                    *(fsumx+i) += tmp_fpsx + tmp_fyox + tmp_ftax;
                                    *(fsumy+i) += tmp_fpsy + tmp_fyoy + tmp_ftay;
                                } else { /* ie. if Injured[i]=1 */
                                    *(fsumx+i) += tmp_fyox + tmp_ftax;
                                    *(fsumy+i) += tmp_fyoy + tmp_ftay;
                                }
                                if(Injured[m]==0){
                                    *(fsumx+m) -= tmp_fpsx + tmp_fyox + tmp_ftax;
                                    *(fsumy+m) -= tmp_fpsy + tmp_fyoy + tmp_ftay;
                                } else { /* ie. if Injured[m]=1 */
                                    *(fsumx+m) -= tmp_fyox + tmp_ftax;
                                    *(fsumy+m) -= tmp_fyoy + tmp_ftay;
                                }

                                /* sum of magnitude of touching forces */
                                if(InjurySwitch==1){
                                    ftmagsum[i] += sqrt(SQR(tmp_fyox)+SQR(tmp_fyoy));
                                    ftmagsum[m] += sqrt(SQR(tmp_fyox)+SQR(tmp_fyoy));
                                }
                            }

                            m = BInd[m];
                            while(m>=i) { m = BInd[m]; }
                        }while(m!=-1);
                    }
                }
            }
        }
    }
}

void Upd()
{

  /* one parallel update step
     using Euler's method with adaptive stepsize

     0, allocating memory to local arrays

     1, forces
        1, walls
    2, wpoints
    3, pairs
    4, column
        5, smoke force & injuries

     2,
        1. preparing update of the eq. of motion computing vxnew[i], vynew[i]
        2. if falling down is allowed, checking injuries

     3,
        1, current coordinates of particle i stored
        2, updating coordinates of particle i
            (X[i],Y[i]) = (X[i],Y[i]) + (VX[i],VY[i]) * time step
      (if any particles have left, storing leaving times)
        3, removing particles that have dropped off (with X[i]>XS)
    4, updating book-keeping for remainig particles

     4, 1 efficiency in this update
        2 for(i=0..N-1)
          (VX[i],VY[i]) = (VXNew[i],VYNew[i])
    3 updating preferred directions
    4 time step added to present time value

     5, freeing mem. allocated to local arrays
  */


    int allocN,i,j,j_old,j_new;
    float *fspx,*fspy,*fsumx,*fsumy,*vxnew,*vynew, tmpr,
        tmp_fpsx,tmp_fpsy,tmp_fyox,tmp_fyoy,tmp_ftax,tmp_ftay,
        tmprsqr,sqrt_fact,ksi,eta,vnew,*ftmagsum,*fsmokex,*fsmokey,
        x_smokefront,tmpf,f_over_r,scal_prod_over_rsqr,rx,ry,
        *fcolx,*fcoly;

    gtstep = 0;

    /* 0 */
    allocN=N;
    fsmokex=vector(0,allocN-1);
    fsmokey=vector(0,allocN-1);

    fspx=vector(0,allocN-1);
    fspy=vector(0,allocN-1);
    fsumx=vector(0,allocN-1);
    fsumy=vector(0,allocN-1);
    vxnew=vector(0,allocN-1);
    vynew=vector(0,allocN-1);
    ftmagsum=vector(0,allocN-1);
    fcolx=vector(0,allocN-1);
    fcoly=vector(0,allocN-1);

    gtstep = DefaultDeltaT;

    FW_x=0.0;

    calc_wall_force(fsumx, fsumy, ftmagsum);   /* wall force */
    calc_wall_point_force(fsumx, fsumy, ftmagsum); /* wpoint force */
    calc_pair_force(fsumx, fsumy, ftmagsum); /* particle-particle forces */

  /* 1.4
   * column
   */
    switch(ColumnSwitch){
    default: case 0:{
        for(i=0;i<N;i++){
            fcolx[i] = fcoly[i] = 0.0;
        }
        break;
    }
    case 1:{
        for(i=0;i<N;i++){
            tmprsqr = SQR(X[i]-ColumnCenterX)+SQR(Y[i]-ColumnCenterY);
            if(tmprsqr<=SQR(R)){
                tmpr=sqrt(tmprsqr);

            /* init */
            tmp_fpsx = tmp_fpsy = 0.0;
            tmp_fyox = tmp_fyoy = 0.0;
            tmp_ftax = tmp_ftay = 0.0;

            /* computing forces */
            /* psychological */
            f_over_r = A * exp(-(tmpr-0.5*(D[i]+ColumnD))/B) / tmpr;
            tmp_fpsx = (X[i]-ColumnCenterX) * f_over_r;
            tmp_fpsy = (Y[i]-ColumnCenterY) * f_over_r;
            /* touching */
            if(tmpr<=0.5*(D[i]+ColumnD)){
            /* Young */
            f_over_r = 2.0*C_Young*(0.5*(D[i]+ColumnD)-tmpr) / tmpr;
            tmp_fyox = (X[i]-ColumnCenterX) * f_over_r;
            tmp_fyoy = (Y[i]-ColumnCenterY) * f_over_r;
            /* friction */
            rx = X[i]-ColumnCenterX;
            ry = Y[i]-ColumnCenterY;
            scal_prod_over_rsqr = (ry*VX[i] - rx*VY[i]) / SQR(tmpr);
            switch(FrictionSwitch){
            case 0:{
                tmp_ftax = -Gamma * (   ry * scal_prod_over_rsqr );
                tmp_ftay = -Gamma * ( - rx * scal_prod_over_rsqr );
                break;
            }
            case 1:{
                tmp_ftax =   -Kappa * (0.5*(D[i]+ColumnD)-tmpr)
                     * (   ry * scal_prod_over_rsqr );
                tmp_ftay =   -Kappa * (0.5*(D[i]+ColumnD)-tmpr)
                     * ( - rx * scal_prod_over_rsqr );
                break;
            }
            }
            }


          /* summing forces */
          if(Injured[i]==0){
              fcolx[i] = tmp_fpsx + tmp_fyox + tmp_ftax;
              fcoly[i] = tmp_fpsy + tmp_fyoy + tmp_ftay;
          }
          else /* ie. if Injured[i]==1 */ {
              fcolx[i] = tmp_fyox + tmp_ftax;
              fcoly[i] = tmp_fyoy + tmp_ftay;
          }


          /* sum of magnitude of touching forces */
          if(InjurySwitch==1){
              ftmagsum[i] += sqrt(SQR(tmp_fyox)+SQR(tmp_fyoy));
          }
      }
      }
      break;
  }
  }



  /* 1.5 */
  /* injuries */

  switch(InjurySwitch){
  case 0: { break; }
  case 1:{

      /* case: people crushed */
      for(i=0;i<N;i++){

      /* newly injured */
      if((ftmagsum[i]>FCrush_over_1m*PI*D[i])&&(Injured[i]==0)){
          Injured[i] = 1;
          NInjured++;
          V0of[i] = 0.0;
      }
      }
      break;
  }
  case 2: case 3: {

      /* case: smoke front */
      if(sim_time>=SmokeStartTime){
      x_smokefront = (sim_time-SmokeStartTime)*VSmoke;

      for(i=0;i<N;i++){
          /* checking position compared to smoke front */
          tmpr = X[i] - x_smokefront;

          /* center of particle behind smoke front: injured */
          if( tmpr < 0.5*D[i] ){
          if(Injured[i]==0){
              Injured[i] = 1;
              NInjured++;
              V0of[i] = 0.0;
              VX[i] = VY[i] = 0.0;
          }
          }
          /* ahead of front but within its interaction range:
         trying to escape */
          if( (tmpr>=0.5*D[i])&&(tmpr<=R) ){
          tmpf = A_fire*exp(-(tmpr-0.5*D[i])/B_fire);
          fsmokex[i] += cos(Phi[i])*tmpf;
          fsmokey[i] += sin(Phi[i])*tmpf;
          }
      }
      }
      break;
  }
  }



  /* 2 */

  /* 2.1 preparing update of the eq. of motion */

  sqrt_fact = sqrt(gtstep/DefaultDeltaT);
  for(i=0;i<N;i++) {

          /* self-propelling */
          fspx[i] = 1/Tau * (V0of[i]*cos(Phi[i]) - VX[i]);
      fspy[i] = 1/Tau * (V0of[i]*sin(Phi[i]) - VY[i]);

      /* noise */
      if(GaTh!=0.0){
              ksi = GaussRand(GaMe, GaTh, GaCM);
          eta = 2.0*PI * rand() / (RAND_MAX+1.0);
      }
      else{ ksi=0.0; eta=0.0; }


      /* sum of forces */
      fsumx[i] +=   fspx[i] + sqrt_fact * ksi * cos(eta);
      fsumy[i] +=   fspy[i] + sqrt_fact * ksi * sin(eta);


      /* adding smoke force */
      if((InjurySwitch==2)||(InjurySwitch==3)){
          fsumx[i] += fsmokex[i];
          fsumy[i] += fsmokey[i];
      }
      /* adding force of column */
      switch(ColumnSwitch){
      default: case 0:{ break; }
      case 1:{
          fsumx[i] += fcolx[i];
          fsumy[i] += fcoly[i];
          break;
      }
      }


      /* time step adjustment for velocity change */
      EulTStep( &gtstep, sqrt(SQR(fsumx[i])+SQR(fsumy[i])) );


      /* new velocity */
      if(  (Injured[i]==1)
         &&((InjurySwitch==1)||(InjurySwitch==3))
        ){
              vxnew[i] = 0.0;
          vynew[i] = 0.0;
      }
      else{
              vxnew[i] = VX[i] + fsumx[i] * gtstep;
          vynew[i] = VY[i] + fsumy[i] * gtstep;
      }


      /* checking new velocity */
      vnew = sqrt( SQR(vxnew[i]) + SQR(vynew[i]) );
      if(vnew > Vmax) {
              vxnew[i] = vxnew[i]/vnew * Vmax;
          vynew[i] = vynew[i]/vnew * Vmax;
      }
  }




  /* 3 */

  for(i=0; i<N; i++) {

          /* .1 */
      Xprev[i] = X[i];
      Yprev[i] = Y[i];

          /* .2 */
      X[i] += VX[i] * gtstep;
      Y[i] += VY[i] * gtstep;

//      if((Xprev[i]>RoomXSize)&&(X[i]<=RoomXSize)){ NInRoom++; }
      if((Xprev[i]<=RoomXSize)&&(X[i]>RoomXSize)){
          NInRoom--;
      }
  }


  /* .3 and .4 */
  for(i=0;i<N;i++){

      /* (a) if the particle is on the board, its book-keeping
         arrays are modified only if its block has changed during
         the last update
         (b) if the particle is off-board, it will be removed */


          /* a */
      if(X[i]<XS){
              j_old =   (int)floor(Xprev[i]*GX/XS)
                  + G*(int)floor(Yprev[i]*GY/YS);
              j_new = (int)floor(X[i]*GX/XS) + G*(int)floor(Y[i]*GY/YS);
          if( j_new != j_old ) {

              /* deleting particle i from its old block */
              j = j_old;
              if(BIndBd[j]==i) {
                  BIndBd[j] = BInd[i];
              }
              else {
                  j = BIndBd[j];
              while(BInd[j]!=i) {
                      j = BInd[j];
              }
              BInd[j] = BInd[i];
              }


              /* inserting particle i into its new block */
              j = j_new;
              if(BIndBd[j]==-1) {
                  BIndBd[j] = i;
              BInd[i] = -1;
              }
              else {
                  j = BIndBd[j];
              while(BInd[j]!=-1) {
                      j = BInd[j];
              }
              BInd[j] = i;
              BInd[i] = -1;
              }
          }
      }
      else{
              RemoveParticle( &N, i );
          i--;
      }
  }



  /* 4 */

  /* 4.1 */
//  E[UpdNum+1] = 0.0;
//  for(i=0;i<N;i++) {
//          E[UpdNum+1] += VX[i] * cos(Phi[i]) + VY[i] * sin(Phi[i]);
//  }
//  if(N>0){ E[UpdNum+1] /= N; }


  /* 4.2 */
    for(i=0;i<N;i++){
        VX[i] = vxnew[i];
        VY[i] = vynew[i];
        V[i] = sqrt(SQR(VX[i])+SQR(VY[i]));
        Vdir[i] = atan2(VY[i],VX[i]);
        Phi[i] = DirectionOfExit( i );
    }


    sim_time += gtstep;
    UpdNum++;


    /* 5 */
    free_vector(fsmokex,0,allocN-1);
    free_vector(fsmokey,0,allocN-1);

    free_vector(fspx,0,allocN-1);
    free_vector(fspy,0,allocN-1);
    free_vector(fsumx,0,allocN-1);
    free_vector(fsumy,0,allocN-1);
    free_vector(vxnew,0,allocN-1);
    free_vector(vynew,0,allocN-1);
    free_vector(ftmagsum,0,allocN-1);
    free_vector(fcolx,0,allocN-1);
    free_vector(fcoly,0,allocN-1);
    /* 6 */
}

/*
** Generates a random number (x) with
** P(x) = exp[- (x-gmean)^2 / (2*gtheta)], if x is in
**        [gmean - gcutmult*sqrt(gtheta), gmean + gcutmult*sqrt(gtheta)]
**      = 0                             , if not
*/
float GaussRand( float gmean, float gtheta, float gcutmult )
{
    if( (GaussFlag==1) && (fabs((GaussSet2-gmean)) <= gcutmult*sqrt(gtheta)) ) {
        GaussFlag = 0;
        return GaussSet2;
    } else {
        float v1,v2,rsq,fac;
        GaussFlag = 0;
        do {
            do {
                v1 = (float)(1.0 - 2.0*(rand()/(RAND_MAX+1.0)));
                v2 = (float)(1.0 - 2.0*(rand()/(RAND_MAX+1.0)));
            } while((rsq=v1*v1+v2*v2) >= (float)1.0);
            fac = (float)sqrt(-2.0*(double)gtheta*log((double)rsq)/(double)rsq);
            GaussSet1 = v1*fac;
            GaussSet2 = v2*fac;
        } while( (fabs(GaussSet1-gmean) > gcutmult*sqrt(gtheta))
           && (fabs(GaussSet2-gmean) > gcutmult*sqrt(gtheta)) );

        if(fabs(GaussSet1-gmean) <= gcutmult*sqrt(gtheta)) {
            GaussFlag = 1;
            return GaussSet1;
        } else {
            GaussFlag = 0;
            return GaussSet2;
        }
    }
}

/*
** Descript: Calculates the mean value of the efficiency of the system for the last
**           few update steps -- NOTE : use this function only when UpdNum > 0
*/
float EMean( char* sw, int unfreq, float stfreq ) {
  /* if unfreq != 0, the average will be calculated for the last unfreq
     updates (the present one included)
     if unfreq == 0, the average will be calculated for the shortest
     possible time interval exceeding stfreq */

  int i, start;
  float e_mean, f;

  if(strcmp(sw,"un")==0) { start = UpdNum - unfreq; }
  else /* i.e. if(strcmp(sw,"st")==0) */ {
          start = Mb; /* start from beginning of present time window */
//      f = floor( SimTime[UpdNum] / stfreq );
          f = floor(sim_time/stfreq);
      while( f - floor( /*SimTime[start]*/0.0 / stfreq ) > 1.0 ) { start++; }
      if( start==UpdNum ) { start--; }
  }
  e_mean = 0.0;
  for(i=start+1; i<=UpdNum; i++) {
          e_mean += E[i] * ( gtstep/*SimTime[i] - SimTime[i-1] */);
  }
  e_mean /= sim_time/*SimTime[UpdNum] - SimTime[start]*/;
  e_mean /= V0;
  return e_mean;
}


/*==============================*/


void Init_Demo(){
  /* 1 general
     2 special
     */

  _E("Initializing, please wait... \n");
  /* 1 */
  Init_Bare("demo");

  /* 2 */
  /* opening files */
  if(!(OFP=fopen(OFN,"w"))){
          fprintf(stderr,"sd_lib.c: Couldn't open %s for writing.\n",OFN);
      SD_LIB_EXIT;
  }
  fprintf(OFP,"UpdNum, SimTime, N, <E>\n");
  fflush(OFP);

  if(!(OF2P=fopen(OF2N,"w"))){
          fprintf(stderr,"sd_lib.c: Couldn't open %s for writing.\n",OF2N);
      SD_LIB_EXIT;
  }
  fprintf(OF2P,"\n");
  fflush(OF2P);

  _E("... finished.\n");
}

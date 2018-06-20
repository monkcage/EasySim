#include "plot.h"

#include <QPainter>
#include <QPen>
#include <QColor>
#include <QBrush>

#include "sd_lib.h"


#define MDiameter this->_diameter
#define MXS       this->_xs
#define MYS       this->_ys
#define MWalls    this->_walls
#define MOffsetX  this->_offset_x
#define MOffsetY  this->_offset_y
#define MHomo     this->_homo
#define MEnFire   this->_enfire

extern pthread_mutex_t mutex;
extern void *shared_mem;


Plot::Plot(QWidget *parent):QWidget (parent)
{
    this->setStyleSheet("background-color:white;");
    this->setUpdatesEnabled(true);
    MHomo = 20.0;
    MXS = nullptr;
    MYS = nullptr;
    MWalls = nullptr;
    MDiameter = nullptr;
    MEnFire = false;
}

Plot::~Plot() { }

#include <QDebug>

void Plot::paintEvent(QPaintEvent *e)
{

    if (MWalls == nullptr) return;
    QPainter *painter = new QPainter(this);
    painter->begin(this);
    // draw walls
    for(int i=0; i<9; ++i){
        painter->drawLine(MWalls[i].x1 * MHomo + MOffsetX,
                          MWalls[i].y1 * MHomo + MOffsetY,
                          MWalls[i].x2 * MHomo + MOffsetX,
                          MWalls[i].y2 * MHomo + MOffsetY);
    }
    // draw door
    painter->setPen(QColor(255, 0, 0));
    painter->drawLine(MWalls[9].x1 * MHomo + MOffsetX,
            MWalls[9].y1 * MHomo + MOffsetY,
            MWalls[9].x2 * MHomo + MOffsetX,
            MWalls[9].y2 * MHomo + MOffsetY);
    // draw particle

    painter->setPen(QColor(0, 255, 0));
    painter->setBrush(QBrush(QColor(0, 255, 0)));

    int nin = get_ninroom();
    float sim_time = get_simtime();
    int inj_num = get_injured_num();

    painter->drawText(QPointF(5, 15), QString("房间内人数：%1").arg(nin));
    painter->drawText(QPointF(5, 30), QString("疏散时间：%1").arg(sim_time));
    painter->drawText(QPointF(5, 45), QString("受伤人数：%1").arg(inj_num));

    int *inj = get_injured();
    for(int i=0; i<get_n(); ++i){
        if(inj[i] == 0){
            painter->setPen(QColor(0, 255, 0, 255));
        }else{
            painter->setPen(QColor(255, 0, 0, 255));
        }
//        if (MXS[i] < 0.0001 && MYS[i] < 0.0001){
//            continue;
//        }
        float x = (MXS[i] - MDiameter[i] / 2) * MHomo + MOffsetX;
        float y = (MYS[i] - MDiameter[i] / 2) * MHomo + MOffsetY;
        float d = (MDiameter[i] * MHomo);
        painter->drawEllipse(x, y, d, d);
    }

    if(this->_enfire){
//        qDebug() << "enable fire ..." << this->_fire * MHomo + MOffsetX;
        painter->setPen(QColor(255, 165, 0));
        painter->drawLine(this->_fire * MHomo + MOffsetX, MOffsetY,
                          this->_fire * MHomo + MOffsetX, this->_room_y * MHomo + MOffsetY);
    }

    painter->end();
    delete painter;
}

void Plot::set_diameter(float *diameter) { MDiameter = diameter; }

void Plot::set_homo(float homo) { MHomo = homo; }

void Plot::set_xs(float *xs) { MXS = xs; }

void Plot::set_ys(float *ys) { MYS = ys; }

void Plot::set_walls(Wall *walls) { MWalls = walls; }

void Plot::set_fire_dis(float val) { this->_fire = val; }

void Plot::set_offset(float off_x, float off_y)
{
    MOffsetX = off_x;
    MOffsetY = off_y;
}

void Plot::set_room_size(float x, float y)
{
    this->_room_x = x;
    this->_room_y = y;
}


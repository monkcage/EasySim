#include "widget.h"
#include "ui_widget.h"

#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QThread>
#include <QDebug>

#include <sys/mman.h>
#include <pthread.h>

#include "sd_lib.h"



#define MPlot       this->_plot
#define MRoomX      this->_room_x
#define MRoomY      this->_room_y
#define MWallWidth  this->_wall_width
#define MDoorWidth  this->_door_width
#define MNodeNum    this->_nodes_num
#define MNodeV0     this->_nodes_v0
#define MEnFire     this->_en_fire
#define MFireV0     this->_fire_v0
#define MTimer      this->_timer

#define new_array(var, type, size) {   \
    if (var == NULL){                  \
        var = new type[size];        \
    }else{                             \
        free(var);                     \
        var = new type[size];        \
    }                                  \
}

#define del_array(var)  {           \
    if(var != NULL) delete []var;   \
}


pthread_mutex_t mutex;
float *xs;
float *ys;
unsigned int nodes_num = 0;

static void get_walls(float x, float y, float door,
                      float wall_width, Wall *walls)
{
    walls[0].x1 = .0;
    walls[0].y1 = .0;
    walls[0].x2 = x * 1.2;
    walls[0].y2 = .0;

    walls[1].x1 = x;
    walls[1].y1 = .0;
    walls[1].x2 = x;
    walls[1].y2 = .5 * (y - door);

    walls[2].x1 = x;
    walls[2].y1 = .5 * (y - door);
    walls[2].x2 = x + wall_width;
    walls[2].y2 = .5 * (y - door);

    walls[3].x1 = x + wall_width;
    walls[3].y1 = .5 * (y - door);
    walls[3].x2 = x + wall_width;
    walls[3].y2 = .0;

    walls[4].x1 = x * 1.2;
    walls[4].y1 = y;
    walls[4].x2 = .0;
    walls[4].y2 = y;

    walls[5].x1 = x + wall_width;
    walls[5].y1 = y;
    walls[5].x2 = x + wall_width;
    walls[5].y2 = .5 * (y + door);

    walls[6].x1 = x + wall_width;
    walls[6].y1 = .5 * (y + door);
    walls[6].x2 = x;
    walls[6].y2 = .5 * (y + door);

    walls[7].x1 = x;
    walls[7].y1 = .5 * (y + door);
    walls[7].x2 = x;
    walls[7].y2 = y;

    walls[8].x1 = .0;
    walls[8].y1 = y;
    walls[8].x2 = .0;
    walls[8].y2 = .0;

    walls[9].x1 = x;
    walls[9].y1 = .5 * (y - door);
    walls[9].x2 = x;
    walls[9].y2 = .5 * (y + door);
}

static void *run_upd(void *)
{
   while(1){
        Upd();
        pthread_mutex_lock(&mutex);
        memcpy(xs, get_xs(), sizeof(*xs)*nodes_num);
        memcpy(ys, get_ys(), sizeof(*ys)*nodes_num);
        int ninjured = get_injured_num();
        int ninroom = get_ninroom();
        pthread_mutex_unlock(&mutex);
        if (ninjured == ninroom){
            // TODO : log
            break;
        }
    }
   return NULL;
}


Widget::Widget(QWidget *parent) :
    QWidget(parent)
{

    this->showMaximized();
    this->showUi();
    xs = nullptr;
    ys = nullptr;
}

Widget::~Widget()
{
    del_array(xs);
    del_array(ys);
}

void Widget::showUi()
{
    MPlot = new Plot(this);
    MPlot->setFixedSize(900, 700);

    this->_timer = new QTimer(this);
    this->connect(this->_timer, &QTimer::timeout, this, &Widget::slot_step);

    QLabel *lbl_room = new QLabel("房间大小:");
    QLabel *lbl_x = new QLabel("米");
    QLabel *lbl_y = new QLabel("米");
    MRoomX = new QLineEdit("15");
    MRoomY = new QLineEdit("15");

    QGridLayout *gridlayout = new QGridLayout();
    gridlayout->addWidget(lbl_room, 0, 0);
    gridlayout->addWidget(MRoomX, 0, 1);
    gridlayout->addWidget(lbl_x, 0, 2);
    gridlayout->addWidget(MRoomY, 0, 3);
    gridlayout->addWidget(lbl_y, 0, 4);

    QLabel *lbl_wall = new QLabel("墙厚度:");
    QLabel *lbl_wall_x = new QLabel("米");
    QLabel *lbl_door = new QLabel("门宽度:");
    QLabel *lbl_door_x = new QLabel("米");
    MWallWidth = new QLineEdit("1");
    MDoorWidth = new QLineEdit("1.2");

    gridlayout->addWidget(lbl_wall, 1, 0);
    gridlayout->addWidget(MWallWidth, 1, 1);
    gridlayout->addWidget(lbl_wall_x, 1, 2);
    gridlayout->addWidget(lbl_door, 1, 3);
    gridlayout->addWidget(MDoorWidth, 1, 4);
    gridlayout->addWidget(lbl_door_x, 1, 5);

    QLabel *lbl_node = new QLabel("容纳人数:");
    QLabel *lbl_nv0 = new QLabel("人初始速度:");
    QLabel *lbl_nv0_x = new QLabel("米/秒");
    MNodeNum = new QLineEdit("200");
    MNodeV0 = new QLineEdit("5");

    gridlayout->addWidget(lbl_node, 2, 0);
    gridlayout->addWidget(MNodeNum, 2, 1);
    gridlayout->addWidget(lbl_nv0, 2, 3);
    gridlayout->addWidget(MNodeV0, 2, 4);
    gridlayout->addWidget(lbl_nv0_x, 2, 5);

    QLabel *lbl_fire = new QLabel("火灾蔓延速度:");
    QLabel *lbl_fire_v = new QLabel("米/秒");
    MEnFire = new QCheckBox("使用火灾场景");
    MFireV0 = new QLineEdit("0.2");

    gridlayout->addWidget(MEnFire, 3, 1);
    gridlayout->addWidget(lbl_fire, 3, 3);
    gridlayout->addWidget(MFireV0, 3, 4);
    gridlayout->addWidget(lbl_fire_v, 3, 5);

    this->_btn_sim = new QPushButton("开始仿真");
    this->connect(_btn_sim, &QPushButton::clicked, this, &Widget::slot_sim);

    gridlayout->addWidget(_btn_sim, 4, 3);

    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(MPlot);
    layout->addLayout(gridlayout);

    this->setLayout(layout);
}

void Widget::slot_sim()
{
    this->_btn_sim->setEnabled(false);
    float room_x = MRoomX->text().toFloat();
    float room_y = MRoomY->text().toFloat();
    float wall_width = MWallWidth->text().toFloat();
    float door_width = MDoorWidth->text().toFloat();
    nodes_num = MNodeNum->text().toUInt();
    float nodes_v0 = MNodeV0->text().toFloat();
    float fire_v0 = MFireV0->text().toFloat();
    bool  en_fire = MEnFire->isChecked();


    Init_Env();
    // TODO: modify parameter here;
    set_room_size(room_x, room_y);
    set_door_width(door_width);
    set_wall_width(wall_width);
    set_v0(nodes_v0);
    set_fire_v0(fire_v0);
    set_particle_num(int(nodes_num));
    if (en_fire) {
        enable_fire(3);
        MPlot->enable_fire(true);
    }
    Init_Demo();

//    Wall walls[10];
    Wall *walls = new Wall[10];
    get_walls(room_x, room_y, door_width, wall_width, walls);

    float homo = 20;
    float off_x = (MPlot->size().width() - room_x * homo) / float(2.0);
    float off_y = (MPlot->size().height() - room_y * homo) / float(2.0);
    MPlot->set_homo(homo);
    MPlot->set_offset(off_x, off_y);
    MPlot->set_walls(walls);
    MPlot->set_room_size(room_x, room_y);
    MPlot->set_diameter(get_diameter());

    pthread_t pid;
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutex_init(&mutex, &attr);

    size_t shared_size = sizeof(float) * nodes_num;
    new_array(xs, float, shared_size);
    new_array(ys, float, shared_size);


    pthread_create(&pid, nullptr, run_upd, nullptr);
    pthread_detach(pid);
    this->_timer->start(100);
}

void Widget::slot_step()
{
    pthread_mutex_lock(&mutex);
    int ninroom = get_ninroom();
    int ninjured = get_injured_num();
    pthread_mutex_unlock(&mutex);
    if (ninjured == ninroom){
        this->_timer->stop();
        this->_btn_sim->setEnabled(true);
    }
    MPlot->set_xs(xs);
    MPlot->set_ys(ys);
    MPlot->set_fire_dis(get_fire_dis());
    MPlot->update();
}

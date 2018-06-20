#ifndef PLOT_H
#define PLOT_H

#include <QWidget>
#include <QPaintEvent>

struct Wall{
    float x1;
    float y1;
    float x2;
    float y2;
};

class Plot : public QWidget
{
public:
    Plot(QWidget *parent=nullptr);
    ~Plot();

    virtual void paintEvent(QPaintEvent *e);
    void set_homo(float homo);
    void set_offset(float off_x, float off_y);
    void set_diameter(float *diameter);
    void set_xs(float *xs);
    void set_ys(float *ys);
    void set_walls(Wall *walls);
    void enable_fire(bool val) { this->_enfire = val; }
    void set_fire_dis(float val);
    void set_room_size(float x, float y);

    float *get_xs() {return this->_xs;}
    float *get_ys() {return this->_ys;}

private:
    float *_diameter;
    float *_xs;
    float *_ys;
    Wall  *_walls;
    float  _room_x;
    float  _room_y;
    float  _offset_x;
    float  _offset_y;
    float  _homo;
    bool   _enfire;
    float  _fire;
};

#endif // PLOT_H

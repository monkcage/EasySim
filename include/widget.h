#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLineEdit>
#include <QCheckBox>
#include <QTimer>
#include <QPushButton>

#include "plot.h"


class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();
    
    void showUi();
private slots:
    void slot_sim();
    void slot_step();
//    void run_upd();

private:
    QPushButton*_btn_sim;
    Plot       *_plot;
    QLineEdit  *_room_x;
    QLineEdit  *_room_y;
    QLineEdit  *_wall_width;
    QLineEdit  *_door_width;
    QLineEdit  *_nodes_num;
    QLineEdit  *_nodes_v0;
    QCheckBox  *_en_fire;
    QLineEdit  *_fire_v0;
    QTimer     *_timer;
};

#endif // WIDGET_H

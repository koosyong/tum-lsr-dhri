#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qnode.hpp"
#include <QMainWindow>
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void run();

private:
    void readParams();
    void initGui();

private:
    Ui::MainWindow *ui;


    double ws_origin_x;
    double ws_origin_y;
    double ws_origin_z;
    double ws_size_x;
    double ws_size_y;
    double ws_size_z;
    vector<double> ws;
    int numSubTopics;
    vector<string> subTopics;
    string pubTopic;
};

#endif // MAINWINDOW_H

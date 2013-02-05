/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Feb 4 17:53:56 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      33,   24,   11,   11, 0x08,
      54,   24,   11,   11, 0x08,
      91,   75,   11,   11, 0x08,
     116,   75,   11,   11, 0x08,
     147,  141,   11,   11, 0x08,
     195,  141,   11,   11, 0x08,
     243,  141,   11,   11, 0x08,
     291,  141,   11,   11, 0x08,
     341,  141,   11,   11, 0x08,
     393,  141,   11,   11, 0x08,
     444,  141,   11,   11, 0x08,
     492,  141,   11,   11, 0x08,
     540,  141,   11,   11, 0x08,
     588,  141,   11,   11, 0x08,
     638,  141,   11,   11, 0x08,
     690,  141,   11,   11, 0x08,
     749,  741,   11,   11, 0x08,
     786,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0update_tf()\0_irImage\0"
    "updateIR1(IplImage*)\0updateIR2(IplImage*)\0"
    "rotMat,transMat\0updateTF1(CvMat*,CvMat*)\0"
    "updateTF2(CvMat*,CvMat*)\0value\0"
    "on_horizontalSlider_camera1_x_valueChanged(int)\0"
    "on_horizontalSlider_camera1_y_valueChanged(int)\0"
    "on_horizontalSlider_camera1_z_valueChanged(int)\0"
    "on_horizontalSlider_camera1_yaw_valueChanged(int)\0"
    "on_horizontalSlider_camera1_pitch_valueChanged(int)\0"
    "on_horizontalSlider_camera1_roll_valueChanged(int)\0"
    "on_horizontalSlider_camera2_x_valueChanged(int)\0"
    "on_horizontalSlider_camera2_y_valueChanged(int)\0"
    "on_horizontalSlider_camera2_z_valueChanged(int)\0"
    "on_horizontalSlider_camera2_yaw_valueChanged(int)\0"
    "on_horizontalSlider_camera2_pitch_valueChanged(int)\0"
    "on_horizontalSlider_camera2_roll_valueChanged(int)\0"
    "checked\0on_actionStereo_Auto_triggered(bool)\0"
    "on_pushButton_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->update_tf(); break;
        case 1: _t->updateIR1((*reinterpret_cast< IplImage*(*)>(_a[1]))); break;
        case 2: _t->updateIR2((*reinterpret_cast< IplImage*(*)>(_a[1]))); break;
        case 3: _t->updateTF1((*reinterpret_cast< CvMat*(*)>(_a[1])),(*reinterpret_cast< CvMat*(*)>(_a[2]))); break;
        case 4: _t->updateTF2((*reinterpret_cast< CvMat*(*)>(_a[1])),(*reinterpret_cast< CvMat*(*)>(_a[2]))); break;
        case 5: _t->on_horizontalSlider_camera1_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_horizontalSlider_camera1_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_horizontalSlider_camera1_z_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_horizontalSlider_camera1_yaw_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_horizontalSlider_camera1_pitch_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_horizontalSlider_camera1_roll_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_horizontalSlider_camera2_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_horizontalSlider_camera2_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_horizontalSlider_camera2_z_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->on_horizontalSlider_camera2_yaw_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->on_horizontalSlider_camera2_pitch_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->on_horizontalSlider_camera2_roll_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_actionStereo_Auto_triggered((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_pushButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

/****************************************************************************
** Meta object code from reading C++ file 'dialogcalibration.h'
**
** Created: Tue Feb 12 17:23:03 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../dhri_calibrationOneCam/src/dialogcalibration.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dialogcalibration.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DialogCalibration[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      32,   19,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
      64,   55,   18,   18, 0x08,
     100,   84,   18,   18, 0x08,
     130,  124,   18,   18, 0x08,
     177,  124,   18,   18, 0x08,
     224,  124,   18,   18, 0x08,
     271,  124,   18,   18, 0x08,
     320,  124,   18,   18, 0x08,
     371,  124,   18,   18, 0x08,
     421,   18,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_DialogCalibration[] = {
    "DialogCalibration\0\0camera_param\0"
    "emitTF(vector<double>)\0_irImage\0"
    "updateIR(IplImage*)\0rotMat,transMat\0"
    "updateTF(CvMat*,CvMat*)\0value\0"
    "on_horizontalSlider_camera_x_valueChanged(int)\0"
    "on_horizontalSlider_camera_y_valueChanged(int)\0"
    "on_horizontalSlider_camera_z_valueChanged(int)\0"
    "on_horizontalSlider_camera_yaw_valueChanged(int)\0"
    "on_horizontalSlider_camera_pitch_valueChanged(int)\0"
    "on_horizontalSlider_camera_roll_valueChanged(int)\0"
    "on_pushButton_start_clicked()\0"
};

void DialogCalibration::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        DialogCalibration *_t = static_cast<DialogCalibration *>(_o);
        switch (_id) {
        case 0: _t->emitTF((*reinterpret_cast< vector<double>(*)>(_a[1]))); break;
        case 1: _t->updateIR((*reinterpret_cast< IplImage*(*)>(_a[1]))); break;
        case 2: _t->updateTF((*reinterpret_cast< CvMat*(*)>(_a[1])),(*reinterpret_cast< CvMat*(*)>(_a[2]))); break;
        case 3: _t->on_horizontalSlider_camera_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_horizontalSlider_camera_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_horizontalSlider_camera_z_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_horizontalSlider_camera_yaw_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_horizontalSlider_camera_pitch_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_horizontalSlider_camera_roll_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_pushButton_start_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData DialogCalibration::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject DialogCalibration::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_DialogCalibration,
      qt_meta_data_DialogCalibration, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DialogCalibration::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DialogCalibration::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DialogCalibration::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DialogCalibration))
        return static_cast<void*>(const_cast< DialogCalibration*>(this));
    return QDialog::qt_metacast(_clname);
}

int DialogCalibration::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void DialogCalibration::emitTF(vector<double> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE

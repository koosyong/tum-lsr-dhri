/****************************************************************************
** Meta object code from reading C++ file 'ckinectcalibrationstereo.h'
**
** Created: Tue Feb 5 18:36:24 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ckinectcalibrationstereo.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ckinectcalibrationstereo.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CKinectCalibrationStereo[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      34,   26,   25,   25, 0x05,
      73,   57,   25,   25, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_CKinectCalibrationStereo[] = {
    "CKinectCalibrationStereo\0\0irImage\0"
    "emitIRImage(IplImage*)\0rotMat,transMat\0"
    "emitTF(CvMat*,CvMat*)\0"
};

void CKinectCalibrationStereo::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CKinectCalibrationStereo *_t = static_cast<CKinectCalibrationStereo *>(_o);
        switch (_id) {
        case 0: _t->emitIRImage((*reinterpret_cast< IplImage*(*)>(_a[1]))); break;
        case 1: _t->emitTF((*reinterpret_cast< CvMat*(*)>(_a[1])),(*reinterpret_cast< CvMat*(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CKinectCalibrationStereo::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CKinectCalibrationStereo::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_CKinectCalibrationStereo,
      qt_meta_data_CKinectCalibrationStereo, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CKinectCalibrationStereo::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CKinectCalibrationStereo::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CKinectCalibrationStereo::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CKinectCalibrationStereo))
        return static_cast<void*>(const_cast< CKinectCalibrationStereo*>(this));
    return QThread::qt_metacast(_clname);
}

int CKinectCalibrationStereo::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void CKinectCalibrationStereo::emitIRImage(IplImage * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CKinectCalibrationStereo::emitTF(CvMat * _t1, CvMat * _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE

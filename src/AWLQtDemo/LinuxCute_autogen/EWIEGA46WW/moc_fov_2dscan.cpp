/****************************************************************************
** Meta object code from reading C++ file 'fov_2dscan.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../fov_2dscan.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'fov_2dscan.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_awl__FOV_2DScan_t {
    QByteArrayData data[15];
    char stringdata0[249];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_awl__FOV_2DScan_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_awl__FOV_2DScan_t qt_meta_stringdata_awl__FOV_2DScan = {
    {
QT_MOC_LITERAL(0, 0, 15), // "awl::FOV_2DScan"
QT_MOC_LITERAL(1, 16, 6), // "closed"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 17), // "slotConfigChanged"
QT_MOC_LITERAL(4, 42, 24), // "slotDetectionDataChanged"
QT_MOC_LITERAL(5, 67, 17), // "Detection::Vector"
QT_MOC_LITERAL(6, 85, 4), // "data"
QT_MOC_LITERAL(7, 90, 15), // "ShowContextMenu"
QT_MOC_LITERAL(8, 106, 3), // "pos"
QT_MOC_LITERAL(9, 110, 17), // "slotPaletteAction"
QT_MOC_LITERAL(10, 128, 22), // "slotMergeDisplayAction"
QT_MOC_LITERAL(11, 151, 21), // "slotMeasureModeAction"
QT_MOC_LITERAL(12, 173, 19), // "slotColorCodeAction"
QT_MOC_LITERAL(13, 193, 29), // "slotDisplayDistanceModeAction"
QT_MOC_LITERAL(14, 223, 25) // "slotDisplayZoomModeAction"

    },
    "awl::FOV_2DScan\0closed\0\0slotConfigChanged\0"
    "slotDetectionDataChanged\0Detection::Vector\0"
    "data\0ShowContextMenu\0pos\0slotPaletteAction\0"
    "slotMergeDisplayAction\0slotMeasureModeAction\0"
    "slotColorCodeAction\0slotDisplayDistanceModeAction\0"
    "slotDisplayZoomModeAction"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_awl__FOV_2DScan[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   65,    2, 0x0a /* Public */,
       4,    1,   66,    2, 0x0a /* Public */,
       7,    1,   69,    2, 0x0a /* Public */,
       9,    0,   72,    2, 0x0a /* Public */,
      10,    0,   73,    2, 0x0a /* Public */,
      11,    0,   74,    2, 0x0a /* Public */,
      12,    0,   75,    2, 0x0a /* Public */,
      13,    0,   76,    2, 0x0a /* Public */,
      14,    0,   77,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, QMetaType::QPoint,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void awl::FOV_2DScan::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FOV_2DScan *_t = static_cast<FOV_2DScan *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->closed(); break;
        case 1: _t->slotConfigChanged(); break;
        case 2: _t->slotDetectionDataChanged((*reinterpret_cast< const Detection::Vector(*)>(_a[1]))); break;
        case 3: _t->ShowContextMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 4: _t->slotPaletteAction(); break;
        case 5: _t->slotMergeDisplayAction(); break;
        case 6: _t->slotMeasureModeAction(); break;
        case 7: _t->slotColorCodeAction(); break;
        case 8: _t->slotDisplayDistanceModeAction(); break;
        case 9: _t->slotDisplayZoomModeAction(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (FOV_2DScan::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FOV_2DScan::closed)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject awl::FOV_2DScan::staticMetaObject = {
    { &QFrame::staticMetaObject, qt_meta_stringdata_awl__FOV_2DScan.data,
      qt_meta_data_awl__FOV_2DScan,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *awl::FOV_2DScan::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *awl::FOV_2DScan::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_awl__FOV_2DScan.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int awl::FOV_2DScan::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void awl::FOV_2DScan::closed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

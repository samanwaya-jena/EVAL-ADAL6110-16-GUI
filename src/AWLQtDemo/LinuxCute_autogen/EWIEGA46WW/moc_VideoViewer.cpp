/****************************************************************************
** Meta object code from reading C++ file 'VideoViewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../VideoViewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'VideoViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_awl__VideoViewer_t {
    QByteArrayData data[9];
    char stringdata0[133];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_awl__VideoViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_awl__VideoViewer_t qt_meta_stringdata_awl__VideoViewer = {
    {
QT_MOC_LITERAL(0, 0, 16), // "awl::VideoViewer"
QT_MOC_LITERAL(1, 17, 15), // "ShowContextMenu"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 3), // "pos"
QT_MOC_LITERAL(4, 38, 24), // "slotDetectionDataChanged"
QT_MOC_LITERAL(5, 63, 17), // "Detection::Vector"
QT_MOC_LITERAL(6, 81, 4), // "data"
QT_MOC_LITERAL(7, 86, 16), // "slotImageChanged"
QT_MOC_LITERAL(8, 103, 29) // "slotVideoOptionsChangedAction"

    },
    "awl::VideoViewer\0ShowContextMenu\0\0pos\0"
    "slotDetectionDataChanged\0Detection::Vector\0"
    "data\0slotImageChanged\0"
    "slotVideoOptionsChangedAction"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_awl__VideoViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x0a /* Public */,
       4,    1,   37,    2, 0x0a /* Public */,
       7,    0,   40,    2, 0x0a /* Public */,
       8,    0,   41,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QPoint,    3,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void awl::VideoViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        VideoViewer *_t = static_cast<VideoViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ShowContextMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 1: _t->slotDetectionDataChanged((*reinterpret_cast< const Detection::Vector(*)>(_a[1]))); break;
        case 2: _t->slotImageChanged(); break;
        case 3: _t->slotVideoOptionsChangedAction(); break;
        default: ;
        }
    }
}

const QMetaObject awl::VideoViewer::staticMetaObject = {
    { &QFrame::staticMetaObject, qt_meta_stringdata_awl__VideoViewer.data,
      qt_meta_data_awl__VideoViewer,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *awl::VideoViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *awl::VideoViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_awl__VideoViewer.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int awl::VideoViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

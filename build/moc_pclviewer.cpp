/****************************************************************************
** Meta object code from reading C++ file 'pclviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/pclviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PCLViewer_t {
    QByteArrayData data[16];
    char stringdata0[224];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PCLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PCLViewer_t qt_meta_stringdata_PCLViewer = {
    {
QT_MOC_LITERAL(0, 0, 9), // "PCLViewer"
QT_MOC_LITERAL(1, 10, 18), // "resetButtonPressed"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 17), // "saveButtonPressed"
QT_MOC_LITERAL(4, 48, 17), // "polyButtonPressed"
QT_MOC_LITERAL(5, 66, 16), // "regButtonPressed"
QT_MOC_LITERAL(6, 83, 19), // "pSliderValueChanged"
QT_MOC_LITERAL(7, 103, 5), // "value"
QT_MOC_LITERAL(8, 109, 7), // "closing"
QT_MOC_LITERAL(9, 117, 9), // "drawFrame"
QT_MOC_LITERAL(10, 127, 7), // "toggled"
QT_MOC_LITERAL(11, 135, 17), // "loadActionPressed"
QT_MOC_LITERAL(12, 153, 16), // "lastFrameToggled"
QT_MOC_LITERAL(13, 170, 20), // "actionClearTriggered"
QT_MOC_LITERAL(14, 191, 15), // "tabChangedEvent"
QT_MOC_LITERAL(15, 207, 16) // "keypointsToggled"

    },
    "PCLViewer\0resetButtonPressed\0\0"
    "saveButtonPressed\0polyButtonPressed\0"
    "regButtonPressed\0pSliderValueChanged\0"
    "value\0closing\0drawFrame\0toggled\0"
    "loadActionPressed\0lastFrameToggled\0"
    "actionClearTriggered\0tabChangedEvent\0"
    "keypointsToggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PCLViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x0a /* Public */,
       3,    0,   80,    2, 0x0a /* Public */,
       4,    0,   81,    2, 0x0a /* Public */,
       5,    0,   82,    2, 0x0a /* Public */,
       6,    1,   83,    2, 0x0a /* Public */,
       8,    0,   86,    2, 0x0a /* Public */,
       9,    0,   87,    2, 0x0a /* Public */,
      10,    1,   88,    2, 0x0a /* Public */,
      11,    0,   91,    2, 0x0a /* Public */,
      12,    0,   92,    2, 0x0a /* Public */,
      13,    0,   93,    2, 0x0a /* Public */,
      14,    1,   94,    2, 0x0a /* Public */,
      15,    0,   97,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void,

       0        // eod
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->resetButtonPressed(); break;
        case 1: _t->saveButtonPressed(); break;
        case 2: _t->polyButtonPressed(); break;
        case 3: _t->regButtonPressed(); break;
        case 4: _t->pSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->closing(); break;
        case 6: _t->drawFrame(); break;
        case 7: _t->toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->loadActionPressed(); break;
        case 9: _t->lastFrameToggled(); break;
        case 10: _t->actionClearTriggered(); break;
        case 11: _t->tabChangedEvent((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->keypointsToggled(); break;
        default: ;
        }
    }
}

const QMetaObject PCLViewer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PCLViewer.data,
      qt_meta_data_PCLViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PCLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PCLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PCLViewer.stringdata0))
        return static_cast<void*>(const_cast< PCLViewer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int PCLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

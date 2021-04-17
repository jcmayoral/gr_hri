/****************************************************************************
** Meta object code from reading C++ file 'online.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../online.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'online.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gr_control_gui__MyViz_t {
    QByteArrayData data[20];
    char stringdata0[224];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gr_control_gui__MyViz_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gr_control_gui__MyViz_t qt_meta_stringdata_gr_control_gui__MyViz = {
    {
QT_MOC_LITERAL(0, 0, 21), // "gr_control_gui::MyViz"
QT_MOC_LITERAL(1, 22, 11), // "setTerrainY"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 5), // "value"
QT_MOC_LITERAL(4, 41, 11), // "setTerrainX"
QT_MOC_LITERAL(5, 53, 13), // "setDesiredRow"
QT_MOC_LITERAL(6, 67, 3), // "row"
QT_MOC_LITERAL(7, 71, 14), // "executeTopoMap"
QT_MOC_LITERAL(8, 86, 12), // "visualizeMap"
QT_MOC_LITERAL(9, 99, 7), // "saveMap"
QT_MOC_LITERAL(10, 107, 13), // "deleteTopoMap"
QT_MOC_LITERAL(11, 121, 8), // "setFrame"
QT_MOC_LITERAL(12, 130, 5), // "frame"
QT_MOC_LITERAL(13, 136, 13), // "publishRegion"
QT_MOC_LITERAL(14, 150, 10), // "timetogoCB"
QT_MOC_LITERAL(15, 161, 25), // "std_msgs::Float32ConstPtr"
QT_MOC_LITERAL(16, 187, 7), // "time2go"
QT_MOC_LITERAL(17, 195, 12), // "executeCycle"
QT_MOC_LITERAL(18, 208, 5), // "cycle"
QT_MOC_LITERAL(19, 214, 9) // "existsMap"

    },
    "gr_control_gui::MyViz\0setTerrainY\0\0"
    "value\0setTerrainX\0setDesiredRow\0row\0"
    "executeTopoMap\0visualizeMap\0saveMap\0"
    "deleteTopoMap\0setFrame\0frame\0publishRegion\0"
    "timetogoCB\0std_msgs::Float32ConstPtr\0"
    "time2go\0executeCycle\0cycle\0existsMap"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gr_control_gui__MyViz[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   74,    2, 0x08 /* Private */,
       4,    1,   77,    2, 0x08 /* Private */,
       5,    1,   80,    2, 0x08 /* Private */,
       7,    0,   83,    2, 0x08 /* Private */,
       8,    0,   84,    2, 0x08 /* Private */,
       9,    0,   85,    2, 0x08 /* Private */,
      10,    0,   86,    2, 0x08 /* Private */,
      11,    1,   87,    2, 0x08 /* Private */,
      13,    0,   90,    2, 0x08 /* Private */,
      14,    1,   91,    2, 0x08 /* Private */,
      17,    1,   94,    2, 0x08 /* Private */,
      19,    0,   97,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Bool,

       0        // eod
};

void gr_control_gui::MyViz::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MyViz *_t = static_cast<MyViz *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setTerrainY((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->setTerrainX((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->setDesiredRow((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->executeTopoMap(); break;
        case 4: _t->visualizeMap(); break;
        case 5: _t->saveMap(); break;
        case 6: _t->deleteTopoMap(); break;
        case 7: _t->setFrame((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->publishRegion(); break;
        case 9: _t->timetogoCB((*reinterpret_cast< const std_msgs::Float32ConstPtr(*)>(_a[1]))); break;
        case 10: _t->executeCycle((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: { bool _r = _t->existsMap();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject gr_control_gui::MyViz::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_gr_control_gui__MyViz.data,
      qt_meta_data_gr_control_gui__MyViz,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *gr_control_gui::MyViz::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gr_control_gui::MyViz::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gr_control_gui__MyViz.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int gr_control_gui::MyViz::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

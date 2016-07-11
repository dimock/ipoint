/****************************************************************************
** Meta object code from reading C++ file 'ipoint.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ipoint.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ipoint.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_IntrusionPointWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x08,
      30,   21,   21,   21, 0x08,
      39,   21,   21,   21, 0x08,
      48,   21,   21,   21, 0x08,
      70,   21,   21,   21, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_IntrusionPointWindow[] = {
    "IntrusionPointWindow\0\0onNew()\0onLoad()\0"
    "onSave()\0updateMousePt(QPoint)\0"
    "updateTrianglesCount(size_t)\0"
};

void IntrusionPointWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        IntrusionPointWindow *_t = static_cast<IntrusionPointWindow *>(_o);
        switch (_id) {
        case 0: _t->onNew(); break;
        case 1: _t->onLoad(); break;
        case 2: _t->onSave(); break;
        case 3: _t->updateMousePt((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 4: _t->updateTrianglesCount((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData IntrusionPointWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject IntrusionPointWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_IntrusionPointWindow,
      qt_meta_data_IntrusionPointWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &IntrusionPointWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *IntrusionPointWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *IntrusionPointWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_IntrusionPointWindow))
        return static_cast<void*>(const_cast< IntrusionPointWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int IntrusionPointWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

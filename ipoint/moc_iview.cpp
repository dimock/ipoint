/****************************************************************************
** Meta object code from reading C++ file 'iview.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "iview.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'iview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ViewWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x05,
      35,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   11,   11,   11, 0x08,
      81,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ViewWindow[] = {
    "ViewWindow\0\0pos\0mouseMoved(QPoint)\0"
    "trianglesChanged(size_t)\0onPosChanged(QPoint)\0"
    "onTrianglesChanged(size_t)\0"
};

void ViewWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ViewWindow *_t = static_cast<ViewWindow *>(_o);
        switch (_id) {
        case 0: _t->mouseMoved((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 1: _t->trianglesChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 2: _t->onPosChanged((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 3: _t->onTrianglesChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ViewWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ViewWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_ViewWindow,
      qt_meta_data_ViewWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ViewWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ViewWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ViewWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ViewWindow))
        return static_cast<void*>(const_cast< ViewWindow*>(this));
    return QWidget::qt_metacast(_clname);
}

int ViewWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ViewWindow::mouseMoved(const QPoint & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ViewWindow::trianglesChanged(size_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE

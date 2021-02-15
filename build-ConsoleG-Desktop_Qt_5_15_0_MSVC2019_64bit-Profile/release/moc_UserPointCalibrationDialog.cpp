/****************************************************************************
** Meta object code from reading C++ file 'UserPointCalibrationDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../ConsoleG/UserPointCalibrationDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'UserPointCalibrationDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_UserPointCalibrationDialog_t {
    QByteArrayData data[35];
    char stringdata0[725];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_UserPointCalibrationDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_UserPointCalibrationDialog_t qt_meta_stringdata_UserPointCalibrationDialog = {
    {
QT_MOC_LITERAL(0, 0, 26), // "UserPointCalibrationDialog"
QT_MOC_LITERAL(1, 27, 22), // "on_pbSelectImg_clicked"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 27), // "on_pbImgInteractive_clicked"
QT_MOC_LITERAL(4, 79, 23), // "on_pbImshow1600_clicked"
QT_MOC_LITERAL(5, 103, 31), // "getValidObjImgPointsAndMappings"
QT_MOC_LITERAL(6, 135, 7), // "cv::Mat"
QT_MOC_LITERAL(7, 143, 9), // "objPoints"
QT_MOC_LITERAL(8, 153, 9), // "imgPoints"
QT_MOC_LITERAL(9, 163, 8), // "cv::Mat&"
QT_MOC_LITERAL(10, 172, 14), // "validObjPoints"
QT_MOC_LITERAL(11, 187, 14), // "validImgPoints"
QT_MOC_LITERAL(12, 202, 17), // "std::vector<int>&"
QT_MOC_LITERAL(13, 220, 10), // "validToAll"
QT_MOC_LITERAL(14, 231, 10), // "allToValid"
QT_MOC_LITERAL(15, 242, 29), // "on_edGlobalPoints_textChanged"
QT_MOC_LITERAL(16, 272, 26), // "on_edImgPoints_textChanged"
QT_MOC_LITERAL(17, 299, 34), // "on_edColinearImgPoints_textCh..."
QT_MOC_LITERAL(18, 334, 22), // "on_pbCalibrate_clicked"
QT_MOC_LITERAL(19, 357, 24), // "on_edImgSize_textChanged"
QT_MOC_LITERAL(20, 382, 35), // "on_tbIntrinsic_itemSelectionC..."
QT_MOC_LITERAL(21, 418, 35), // "on_tbExtrinsic_itemSelectionC..."
QT_MOC_LITERAL(22, 454, 36), // "on_tbProjection_itemSelection..."
QT_MOC_LITERAL(23, 491, 22), // "on_pbUndistort_clicked"
QT_MOC_LITERAL(24, 514, 18), // "getIntrinsicFromUi"
QT_MOC_LITERAL(25, 533, 18), // "getExtrinsicFromUi"
QT_MOC_LITERAL(26, 552, 25), // "getCalibrationFlagsFromUi"
QT_MOC_LITERAL(27, 578, 20), // "displayIntrinsicToUi"
QT_MOC_LITERAL(28, 599, 20), // "displayExtrinsicToUi"
QT_MOC_LITERAL(29, 620, 21), // "displayProjectionToUi"
QT_MOC_LITERAL(30, 642, 27), // "displayUndistortedImageToUi"
QT_MOC_LITERAL(31, 670, 14), // "displayMessage"
QT_MOC_LITERAL(32, 685, 3), // "msg"
QT_MOC_LITERAL(33, 689, 17), // "on_pbSave_clicked"
QT_MOC_LITERAL(34, 707, 17) // "on_pbLoad_clicked"

    },
    "UserPointCalibrationDialog\0"
    "on_pbSelectImg_clicked\0\0"
    "on_pbImgInteractive_clicked\0"
    "on_pbImshow1600_clicked\0"
    "getValidObjImgPointsAndMappings\0cv::Mat\0"
    "objPoints\0imgPoints\0cv::Mat&\0"
    "validObjPoints\0validImgPoints\0"
    "std::vector<int>&\0validToAll\0allToValid\0"
    "on_edGlobalPoints_textChanged\0"
    "on_edImgPoints_textChanged\0"
    "on_edColinearImgPoints_textChanged\0"
    "on_pbCalibrate_clicked\0on_edImgSize_textChanged\0"
    "on_tbIntrinsic_itemSelectionChanged\0"
    "on_tbExtrinsic_itemSelectionChanged\0"
    "on_tbProjection_itemSelectionChanged\0"
    "on_pbUndistort_clicked\0getIntrinsicFromUi\0"
    "getExtrinsicFromUi\0getCalibrationFlagsFromUi\0"
    "displayIntrinsicToUi\0displayExtrinsicToUi\0"
    "displayProjectionToUi\0displayUndistortedImageToUi\0"
    "displayMessage\0msg\0on_pbSave_clicked\0"
    "on_pbLoad_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_UserPointCalibrationDialog[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  129,    2, 0x08 /* Private */,
       3,    0,  130,    2, 0x08 /* Private */,
       4,    0,  131,    2, 0x08 /* Private */,
       5,    6,  132,    2, 0x08 /* Private */,
      15,    0,  145,    2, 0x08 /* Private */,
      16,    0,  146,    2, 0x08 /* Private */,
      17,    0,  147,    2, 0x08 /* Private */,
      18,    0,  148,    2, 0x08 /* Private */,
      19,    0,  149,    2, 0x08 /* Private */,
      20,    0,  150,    2, 0x08 /* Private */,
      21,    0,  151,    2, 0x08 /* Private */,
      22,    0,  152,    2, 0x08 /* Private */,
      23,    0,  153,    2, 0x08 /* Private */,
      24,    0,  154,    2, 0x08 /* Private */,
      25,    0,  155,    2, 0x08 /* Private */,
      26,    0,  156,    2, 0x08 /* Private */,
      27,    0,  157,    2, 0x08 /* Private */,
      28,    0,  158,    2, 0x08 /* Private */,
      29,    0,  159,    2, 0x08 /* Private */,
      30,    0,  160,    2, 0x08 /* Private */,
      31,    1,  161,    2, 0x08 /* Private */,
      33,    0,  164,    2, 0x08 /* Private */,
      34,    0,  165,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 6, 0x80000000 | 9, 0x80000000 | 9, 0x80000000 | 12, 0x80000000 | 12,    7,    8,   10,   11,   13,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   32,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void UserPointCalibrationDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<UserPointCalibrationDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_pbSelectImg_clicked(); break;
        case 1: _t->on_pbImgInteractive_clicked(); break;
        case 2: _t->on_pbImshow1600_clicked(); break;
        case 3: _t->getValidObjImgPointsAndMappings((*reinterpret_cast< cv::Mat(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4])),(*reinterpret_cast< std::vector<int>(*)>(_a[5])),(*reinterpret_cast< std::vector<int>(*)>(_a[6]))); break;
        case 4: _t->on_edGlobalPoints_textChanged(); break;
        case 5: _t->on_edImgPoints_textChanged(); break;
        case 6: _t->on_edColinearImgPoints_textChanged(); break;
        case 7: _t->on_pbCalibrate_clicked(); break;
        case 8: _t->on_edImgSize_textChanged(); break;
        case 9: _t->on_tbIntrinsic_itemSelectionChanged(); break;
        case 10: _t->on_tbExtrinsic_itemSelectionChanged(); break;
        case 11: _t->on_tbProjection_itemSelectionChanged(); break;
        case 12: _t->on_pbUndistort_clicked(); break;
        case 13: _t->getIntrinsicFromUi(); break;
        case 14: _t->getExtrinsicFromUi(); break;
        case 15: _t->getCalibrationFlagsFromUi(); break;
        case 16: _t->displayIntrinsicToUi(); break;
        case 17: _t->displayExtrinsicToUi(); break;
        case 18: _t->displayProjectionToUi(); break;
        case 19: _t->displayUndistortedImageToUi(); break;
        case 20: _t->displayMessage((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 21: _t->on_pbSave_clicked(); break;
        case 22: _t->on_pbLoad_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject UserPointCalibrationDialog::staticMetaObject = { {
    QMetaObject::SuperData::link<QDialog::staticMetaObject>(),
    qt_meta_stringdata_UserPointCalibrationDialog.data,
    qt_meta_data_UserPointCalibrationDialog,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *UserPointCalibrationDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *UserPointCalibrationDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_UserPointCalibrationDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int UserPointCalibrationDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 23)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 23;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

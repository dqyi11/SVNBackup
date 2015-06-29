#ifndef CONFIGOBJDIALOG_H
#define CONFIGOBJDIALOG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>

class mainwindow;

class ConfigObjDialog : public QDialog
{
    Q_OBJECT
public:
    ConfigObjDialog(mainwindow * parent);

private:
    QListWidget * mpListWidget;
    QPushButton * mpBtnAdd;
    QPushButton * mpBtnRemove;
    QPushButton * mpBtnOK;
    QPushButton * mpBtnCancel;

    mainwindow * mpParentWindow;
};

#endif // CONFIGOBJDIALOG_H

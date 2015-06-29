#ifndef CONFIGOBJDIALOG_H
#define CONFIGOBJDIALOG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>

class MainWindow;

class ConfigObjDialog : public QDialog
{
    Q_OBJECT
public:
    ConfigObjDialog(MainWindow * parent);

private:
    QListWidget * mpListWidget;
    QPushButton * mpBtnAdd;
    QPushButton * mpBtnRemove;
    QPushButton * mpBtnOK;
    QPushButton * mpBtnCancel;

    QCheckBox * mpCheckMinDist;
    QLabel    * mpLabelMinDist;

    MainWindow * mpParentWindow;

public slots:
    void checkBoxStateChanged(int state);
};

#endif // CONFIGOBJDIALOG_H

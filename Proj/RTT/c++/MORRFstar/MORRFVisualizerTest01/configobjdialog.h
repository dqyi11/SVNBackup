#ifndef CONFIGOBJDIALOG_H
#define CONFIGOBJDIALOG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>

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
    QLabel    * mpLabelSubProb;
    QLineEdit * mpLineEditSubProb;
    QLabel    * mpLabelIterationNum;
    QLineEdit * mpLineEditIterationNum;

    MainWindow * mpParentWindow;

    void updateConfiguration();

public slots:
    //void checkBoxStateChanged(int state);
    void onBtnOKClicked();
    void onBtnCancelClicked();
    void onBtnAddClicked();
    void onBtnRemoveClicked();
};

#endif // CONFIGOBJDIALOG_H

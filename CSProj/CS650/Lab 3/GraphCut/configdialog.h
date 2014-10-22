#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>

class QLineEdit;
class QLabel;
class QPushButton;
class QRadioButton;
class QGridLayout;
class ParameterManager;

class ConfigDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ConfigDialog(ParameterManager * paramMgr=NULL, QWidget *parent = 0);
    ~ConfigDialog();

signals:

public slots:
    void on_ok_clicked();
    void on_cancel_clicked();

private:
    QLineEdit * mpEditSmoothnessRatio;
    QLabel    * mpLabelSmoothnessRatio;
    QLineEdit * mpEditKDEBandwidth;
    QLabel    * mpLabelKDEBandwidth;
    QLineEdit * mpEditIterationNumber;
    QLabel    * mpLabelIterationNumber;

    QRadioButton * mpBtnKDE;
    QRadioButton * mpBtnGMM;

    QPushButton * mpOKBtn;
    QPushButton * mpCancelBtn;
    QGridLayout * mpLayout;

    ParameterManager * mpParamMgr;

};

#endif // CONFIGDIALOG_H

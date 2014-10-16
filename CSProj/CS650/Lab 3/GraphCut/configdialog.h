#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>

class QLineEdit;
class QLabel;
class QPushButton;
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
    QLineEdit * mpEditRegionImportance;
    QLabel    * mpLabelRegionImportance;
    QLineEdit * mpEditNeighborhoodSigma;
    QLabel    * mpLabelNeighborhoodSimga;
    QLineEdit * mpEditKDESigma;
    QLabel    * mpLabelKDESigma;

    QLineEdit * mpEditIterationNumber;
    QLabel    * mpLabelIterationNumber;

    QPushButton * mpOKBtn;
    QPushButton * mpCancelBtn;
    QGridLayout * mpLayout;

    ParameterManager * mpParamMgr;

};

#endif // CONFIGDIALOG_H

#include "configdialog.h"
#include "parametermanager.h"
#include <QtGui>

ConfigDialog::ConfigDialog(ParameterManager * paramMgr, QWidget *parent) :
    QDialog(parent)
{
    mpParamMgr = paramMgr;

    mpEditSmoothnessRatio = new QLineEdit(this);
    mpLabelSmoothnessRatio = new QLabel(tr("Smoothness Ratio"), this);
    mpEditSmoothnessRatio->setText(QString::number(mpParamMgr->mSmoothnessRatio, 'g', 4));

    mpEditKDEBandwidth = new QLineEdit(this);
    mpLabelKDEBandwidth = new QLabel(tr("Kernel Density Estimator Bandwidth"), this);
    mpEditKDEBandwidth->setText(QString::number(mpParamMgr->mKDEBandWidth, 'g', 4));

    mpEditIterationNumber = new QLineEdit(this);
    mpLabelIterationNumber = new QLabel(tr("Iteration Number"), this);
    mpEditIterationNumber->setText(QString::number(mpParamMgr->mIterationNumber));

    mpOKBtn = new QPushButton(tr("OK"), this);
    connect(mpOKBtn, SIGNAL(released()), this, SLOT(on_ok_clicked()));
    mpCancelBtn = new QPushButton(tr("Cancel"), this);
    connect(mpCancelBtn, SIGNAL(released()), this, SLOT(on_cancel_clicked()));
    mpLayout = new QGridLayout(this);

    mpBtnKDE = new QRadioButton(tr("&KDE"));
    mpBtnGMM = new QRadioButton(tr("&GMM"));
    if(mpParamMgr->mDEType==KERNEL)
    {
        mpBtnKDE->setChecked(true);
    }
    else if(mpParamMgr->mDEType==GMM)
    {
        mpBtnGMM->setChecked(true);
    }

    mpLayout->addWidget(mpBtnKDE, 0, 0);
    mpLayout->addWidget(mpBtnGMM, 0, 1);
    mpLayout->addWidget(mpLabelSmoothnessRatio, 1, 0);
    mpLayout->addWidget(mpEditSmoothnessRatio, 1, 1);
    mpLayout->addWidget(mpLabelKDEBandwidth, 2, 0);
    mpLayout->addWidget(mpEditKDEBandwidth, 2, 1);
    mpLayout->addWidget(mpLabelIterationNumber, 3, 0);
    mpLayout->addWidget(mpEditIterationNumber, 3, 1);

    mpLayout->addWidget(mpOKBtn, 4, 0);
    mpLayout->addWidget(mpCancelBtn, 4, 1);

    setLayout(mpLayout);
}

ConfigDialog::~ConfigDialog()
{
    if(mpEditSmoothnessRatio)
    {
        delete mpEditSmoothnessRatio;
        mpEditSmoothnessRatio = NULL;
    }
    if(mpLabelSmoothnessRatio)
    {
        delete mpLabelSmoothnessRatio;
        mpLabelSmoothnessRatio = NULL;
    }
    if(mpEditKDEBandwidth)
    {
        delete mpEditKDEBandwidth;
        mpEditKDEBandwidth = NULL;
    }
    if(mpLabelKDEBandwidth)
    {
        delete mpLabelKDEBandwidth;
        mpLabelKDEBandwidth = NULL;
    }
    if(mpEditIterationNumber)
    {
        delete mpEditIterationNumber;
        mpEditIterationNumber = NULL;
    }
    if(mpLabelIterationNumber)
    {
        delete mpLabelIterationNumber;
        mpLabelIterationNumber = NULL;
    }
    if(mpOKBtn)
    {
        delete mpOKBtn;
        mpOKBtn = NULL;
    }
    if(mpCancelBtn)
    {
        delete mpCancelBtn;
        mpCancelBtn = NULL;
    }
    if(mpBtnKDE)
    {
        delete mpBtnKDE;
        mpBtnKDE = NULL;
    }
    if(mpBtnGMM)
    {
        delete mpBtnGMM;
        mpBtnGMM = NULL;
    }
    if(mpLayout)
    {
        delete mpLayout;
        mpLayout = NULL;
    }
}

void ConfigDialog::on_ok_clicked()
{
    mpParamMgr->mIterationNumber = mpEditIterationNumber->text().toInt();
    mpParamMgr->mSmoothnessRatio = mpEditSmoothnessRatio->text().toFloat();
    mpParamMgr->mKDEBandWidth = mpEditKDEBandwidth->text().toFloat();
    if(mpBtnKDE->isChecked()==true)
    {
        mpParamMgr->mDEType == KERNEL;
    }
    else if(mpBtnGMM->isChecked()==true)
    {
        mpParamMgr->mDEType == GMM;
    }

    close();
}

void ConfigDialog::on_cancel_clicked()
{
    close();
}

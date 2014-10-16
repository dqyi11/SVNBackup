#include "configdialog.h"
#include "parametermanager.h"
#include <QtGui>

ConfigDialog::ConfigDialog(ParameterManager * paramMgr, QWidget *parent) :
    QDialog(parent)
{
    mpParamMgr = paramMgr;

    mpEditRegionImportance = new QLineEdit(this);
    mpLabelRegionImportance = new QLabel(tr("Region Importance"), this);
    mpEditRegionImportance->setText(QString::number(mpParamMgr->mRegionImportance, 'g', 4));

    mpEditNeighborhoodSigma = new QLineEdit(this);
    mpLabelNeighborhoodSimga = new QLabel(tr("Neighborhood Sigma"), this);
    mpEditNeighborhoodSigma->setText(QString::number(mpParamMgr->mNeighborhoodSigma, 'g', 4));

    mpEditKDESigma = new QLineEdit(this);
    mpLabelKDESigma = new QLabel(tr("Kernel Density Estimator Sigma"), this);
    mpEditKDESigma->setText(QString::number(mpParamMgr->mKDESigma, 'g', 4));

    mpEditIterationNumber = new QLineEdit(this);
    mpLabelIterationNumber = new QLabel(tr("Iteration Number"), this);
    mpEditIterationNumber->setText(QString::number(mpParamMgr->mIterationNumber));

    mpOKBtn = new QPushButton(tr("OK"), this);
    connect(mpOKBtn, SIGNAL(released()), this, SLOT(on_ok_clicked()));
    mpCancelBtn = new QPushButton(tr("Cancel"), this);
    connect(mpCancelBtn, SIGNAL(released()), this, SLOT(on_cancel_clicked()));
    mpLayout = new QGridLayout(this);

    mpLayout->addWidget(mpLabelRegionImportance, 0, 0);
    mpLayout->addWidget(mpEditRegionImportance, 0, 1);
    mpLayout->addWidget(mpLabelNeighborhoodSimga, 1, 0);
    mpLayout->addWidget(mpEditNeighborhoodSigma, 1, 1);
    mpLayout->addWidget(mpLabelKDESigma, 2, 0);
    mpLayout->addWidget(mpEditKDESigma, 2, 1);
    mpLayout->addWidget(mpLabelIterationNumber, 3, 0);
    mpLayout->addWidget(mpEditIterationNumber, 3, 1);

    mpLayout->addWidget(mpOKBtn, 4, 0);
    mpLayout->addWidget(mpCancelBtn, 4, 1);

    setLayout(mpLayout);


}

ConfigDialog::~ConfigDialog()
{
    if(mpEditRegionImportance)
    {
        delete mpEditRegionImportance;
        mpEditRegionImportance = NULL;
    }
    if(mpLabelRegionImportance)
    {
        delete mpLabelRegionImportance;
        mpLabelRegionImportance = NULL;
    }
    if(mpEditNeighborhoodSigma)
    {
        delete mpEditNeighborhoodSigma;
        mpEditNeighborhoodSigma = NULL;
    }
    if(mpLabelNeighborhoodSimga)
    {
        delete mpLabelNeighborhoodSimga;
        mpLabelNeighborhoodSimga = NULL;
    }
    if(mpEditKDESigma)
    {
        delete mpEditKDESigma;
        mpEditKDESigma = NULL;
    }
    if(mpLabelKDESigma)
    {
        delete mpLabelKDESigma;
        mpLabelKDESigma = NULL;
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
    if(mpLayout)
    {
        delete mpLayout;
        mpLayout = NULL;
    }
}

void ConfigDialog::on_ok_clicked()
{
    mpParamMgr->mIterationNumber = mpEditIterationNumber->text().toInt();
    mpParamMgr->mRegionImportance = mpEditRegionImportance->text().toFloat();
    mpParamMgr->mNeighborhoodSigma = mpEditIterationNumber->text().toFloat();
    mpParamMgr->mKDESigma = mpEditKDESigma->text().toFloat();
    close();
}

void ConfigDialog::on_cancel_clicked()
{
    close();
}

#include "configobjdialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "mainwindow.h"
#include <QMessageBox>
#include <QFileDialog>

#define WEIGHTED_SUM_STR           "Weighted-sum"
#define TCHEBYCHEFF_STR           "Tchebycheff"
#define BOUNDARY_INTERSECTION_STR "Boundary-intersection"

ConfigObjDialog::ConfigObjDialog(MainWindow * parent)
{
    mpParentWindow = parent;

    mpCheckMinDist = new QCheckBox();
    if (mpParentWindow->mpViz->mMOPPInfo.mMinDistEnabled==true)
    {
        mpCheckMinDist->setChecked(true);
    }
    else
    {
        mpCheckMinDist->setChecked(false);
    }
    //connect(mpCheckMinDist , SIGNAL(stateChanged(int)),this,SLOT(checkBoxStateChanged(int)));
    mpLabelMinDist = new QLabel("Minimize distance");
    mpLabelSubProb = new QLabel("Subproblem Num: ");
    mpLineEditSubProb = new QLineEdit();
    mpLineEditSubProb->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mSubproblemNum));
    mpLineEditSubProb->setMaximumWidth(40);
    mpLabelIterationNum = new QLabel("Iteration Num: ");
    mpLineEditIterationNum = new QLineEdit();
    mpLineEditIterationNum->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mMaxIterationNum));
    mpLineEditIterationNum->setMaximumWidth(40);
    mpLabelSegmentLength = new QLabel("Segment Len: ");
    mpLineEditSegmentLength = new QLineEdit();
    mpLineEditSegmentLength->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mSegmentLength));
    mpLineEditSegmentLength->setMaximumWidth(40);

    QHBoxLayout * minDistLayout = new QHBoxLayout();
    minDistLayout->addWidget(mpCheckMinDist);
    minDistLayout->addWidget(mpLabelMinDist);
    minDistLayout->addWidget(mpLabelSubProb);
    minDistLayout->addWidget(mpLineEditSubProb);
    minDistLayout->addWidget(mpLabelIterationNum);
    minDistLayout->addWidget(mpLineEditIterationNum);
    minDistLayout->addWidget(mpLabelSegmentLength);
    minDistLayout->addWidget(mpLineEditSegmentLength);

    QHBoxLayout * typeLayout = new QHBoxLayout();
    mpLabelType = new QLabel("Type: ");
    mpComboType = new QComboBox();
    mpComboType->addItem(WEIGHTED_SUM_STR);
    mpComboType->addItem(TCHEBYCHEFF_STR);
    mpComboType->addItem(BOUNDARY_INTERSECTION_STR);
    typeLayout->addWidget(mpLabelType);
    typeLayout->addWidget(mpComboType);


    mpListWidget = new QListWidget();
    mpListWidget->setViewMode(QListView::IconMode);
    for(std::vector<QString>::iterator it=mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.begin();it!=mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.end();it++)
    {
        QString filename = (*it);
        mpListWidget->addItem(filename);
    }
    mpListWidget->show();

    mpBtnAdd = new QPushButton(tr("Add"));
    mpBtnRemove = new QPushButton(tr("Remove"));
    mpBtnOK = new QPushButton(tr("OK"));
    mpBtnCancel = new QPushButton(tr("Cancel"));

    connect(mpBtnAdd, SIGNAL(clicked()), this, SLOT(onBtnAddClicked()));
    connect(mpBtnRemove, SIGNAL(clicked()), this, SLOT(onBtnRemoveClicked()));
    connect(mpBtnOK, SIGNAL(clicked()), this, SLOT(onBtnOKClicked()));
    connect(mpBtnCancel, SIGNAL(clicked()), this, SLOT(onBtnCancelClicked()));

    QHBoxLayout * buttonsLayout = new QHBoxLayout();
    buttonsLayout->addWidget(mpBtnAdd);
    buttonsLayout->addWidget(mpBtnRemove);
    buttonsLayout->addWidget(mpBtnOK);
    buttonsLayout->addWidget(mpBtnCancel);

    QVBoxLayout * mainLayout = new QVBoxLayout();
    mainLayout->addLayout(minDistLayout);
    mainLayout->addLayout(typeLayout);
    mainLayout->addWidget(mpListWidget);
    mainLayout->addLayout(buttonsLayout);

    setWindowTitle("Config Objectives");

    setLayout(mainLayout);
}

void ConfigObjDialog::onBtnOKClicked()
{
    updateConfiguration();
    close();
}

void ConfigObjDialog::onBtnCancelClicked()
{
    close();
}

void ConfigObjDialog::onBtnAddClicked()
{
   QString objFilename = QFileDialog::getOpenFileName(this,
                 tr("Open Objective File"), "./", tr("Objective Files (*.*)"));
   if (objFilename!="")
   {
       if(true==isCompatible(objFilename))
       {
           mpListWidget->addItem(objFilename);
           repaint();
       }
   }
   else
   {
       QMessageBox msg;
       msg.setText("Fitness distribution file is not compatible!");
       msg.exec();
   }
}

void ConfigObjDialog::onBtnRemoveClicked()
{
    qDeleteAll(mpListWidget->selectedItems());
}

void ConfigObjDialog::updateDisplay()
{
    if(mpParentWindow)
    {
        if(mpParentWindow->mpViz)
        {
            if(mpParentWindow->mpViz->mMOPPInfo.mMinDistEnabled==true)
            {
                mpCheckMinDist->setChecked(true);
            }
            else
            {
                mpCheckMinDist->setChecked(false);
            }
            mpLineEditSubProb->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mSubproblemNum));
            mpLineEditSegmentLength->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mSegmentLength));
            mpLineEditIterationNum->setText(QString::number(mpParentWindow->mpViz->mMOPPInfo.mMaxIterationNum));
            if(mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.size()>0)
            {
                mpListWidget->clear();
                for(std::vector<QString>::iterator it= mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.begin();
                    it!=mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.end();it++)
                {
                    QString objFilename = (*it);
                    mpListWidget->addItem(objFilename);
                }
            }

            if(MORRF::WEIGHTED_SUM == mpParentWindow->mpViz->mMOPPInfo.mMethodType)
            {
                mpComboType->setCurrentIndex(0);
            }
            else if(MORRF::WEIGHTED_SUM == MORRF::TCHEBYCHEFF)
            {
                mpComboType->setCurrentIndex(1);
            }
            else if(MORRF::WEIGHTED_SUM == MORRF::BOUNDARY_INTERSACTION)
            {
                mpComboType->setCurrentIndex(2);
            }
        }
    }

}

void ConfigObjDialog::updateConfiguration()
{
    int numObj = 0;
    if (mpCheckMinDist->isChecked()==true)
    {
        numObj += 1;
        mpParentWindow->mpViz->mMOPPInfo.mMinDistEnabled=true;
    }
    else
    {
        mpParentWindow->mpViz->mMOPPInfo.mMinDistEnabled=false;
    }

    mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.clear();
    int count = mpListWidget->count();
    for(int i=0;i<count;i++)
    {
        QListWidgetItem * pItem = mpListWidget->item(i);
        mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.push_back(pItem->text());
        numObj +=1;
    }

    mpParentWindow->mpViz->mMOPPInfo.mObjectiveNum = numObj;

    mpParentWindow->mpViz->mMOPPInfo.mMaxIterationNum = mpLineEditIterationNum->text().toInt();
    mpParentWindow->mpViz->mMOPPInfo.mSubproblemNum = mpLineEditSubProb->text().toInt();
    mpParentWindow->mpViz->mMOPPInfo.mSegmentLength = mpLineEditSegmentLength->text().toDouble();

    int type = mpComboType->currentIndex();
    mpParentWindow->mpViz->mMOPPInfo.mMethodType = (MORRF::MORRF_TYPE) type;

}

bool ConfigObjDialog::isCompatible(QString fitnessFile)
{
    QPixmap pixmap(fitnessFile);
    if (pixmap.width()==mpParentWindow->mpViz->mMOPPInfo.mMapWidth
            && pixmap.height()==mpParentWindow->mpViz->mMOPPInfo.mMapHeight)
    {
        return true;
    }
    return false;
}

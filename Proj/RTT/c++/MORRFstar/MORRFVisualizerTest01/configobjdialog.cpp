#include "configobjdialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "mainwindow.h"
#include <QMessageBox>
#include <QFileDialog>

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

    QHBoxLayout * minDistLayout = new QHBoxLayout();
    minDistLayout->addWidget(mpCheckMinDist);
    minDistLayout->addWidget(mpLabelMinDist);
    minDistLayout->addWidget(mpLabelSubProb);
    minDistLayout->addWidget(mpLineEditSubProb);
    minDistLayout->addWidget(mpLabelIterationNum);
    minDistLayout->addWidget(mpLineEditIterationNum);

    mpListWidget = new QListWidget();
    mpListWidget->setViewMode(QListView::IconMode);
    for(std::list<QString>::iterator it=mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.begin();it!=mpParentWindow->mpViz->mMOPPInfo.mObjectiveFiles.end();it++)
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
       mpListWidget->addItem(objFilename);
       repaint();
   }
}

void ConfigObjDialog::onBtnRemoveClicked()
{
    qDeleteAll(mpListWidget->selectedItems());
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

}

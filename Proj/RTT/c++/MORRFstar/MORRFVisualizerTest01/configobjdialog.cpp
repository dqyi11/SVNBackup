#include "configobjdialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "mainwindow.h"
#include <QMessageBox>

ConfigObjDialog::ConfigObjDialog(MainWindow * parent)
{
    mpParentWindow = parent;

    mpCheckMinDist = new QCheckBox();
    mpCheckMinDist->setChecked(true);
    connect(mpCheckMinDist , SIGNAL(stateChanged(int)),this,SLOT(checkBoxStateChanged(int)));
    mpLabelMinDist = new QLabel("Minimize distance");
    QHBoxLayout * minDistLayout = new QHBoxLayout();
    minDistLayout->addWidget(mpCheckMinDist);
    minDistLayout->addWidget(mpLabelMinDist);

    mpListWidget = new QListWidget();
    mpListWidget->setViewMode(QListView::IconMode);

    mpBtnAdd = new QPushButton(tr("Add"));
    mpBtnRemove = new QPushButton(tr("Remove"));
    mpBtnOK = new QPushButton(tr("OK"));
    mpBtnCancel = new QPushButton(tr("Cancel"));
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

void ConfigObjDialog::checkBoxStateChanged(int state)
{
    QMessageBox* msg = new QMessageBox(this->parentWidget());
    msg->setWindowTitle("Hello !");

    if(state)
    {
      msg->setText("CheckBox is Checked !");
    }
    else
    {
      msg->setText("CheckBox is Unchecked !");
    }
    msg->show();
}


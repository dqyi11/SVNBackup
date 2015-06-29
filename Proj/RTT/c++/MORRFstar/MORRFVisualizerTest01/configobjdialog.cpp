#include "configobjdialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>

ConfigObjDialog::ConfigObjDialog(mainwindow * parent)
{
    mpParentWindow = parent;

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
    mainLayout->addWidget(mpListWidget);
    mainLayout->addLayout(buttonsLayout);

    setLayout(mainLayout);
}


#include "interactivewindow.h"
#include <QApplication>

#include "maxflow/graph.h"

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    InteractiveWindow w;
    w.show();

    return a.exec();
}

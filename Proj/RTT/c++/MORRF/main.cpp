#include <list>
#include "KDTree2D.h"

int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    KDTree2D src(std::ptr_fun(tac));

    KDNode2D c0(5, 4); src.insert(c0);
    KDNode2D c1(4, 2); src.insert(c1);
    KDNode2D c2(7, 6); src.insert(c2);
    KDNode2D c3(2, 2); src.insert(c3);
    KDNode2D c4(8, 0); src.insert(c4);
    KDNode2D c5(5, 7); src.insert(c5);
    KDNode2D c6(3, 3); src.insert(c6);
    KDNode2D c7(9, 7); src.insert(c7);
    KDNode2D c8(2, 2); src.insert(c8);
    KDNode2D c9(2, 0); src.insert(c9);

    KDNode2D target(3,5);
    std::pair<KDTree2D::const_iterator,double> found = src.find_nearest(target);
    KDNode2D found_node = *found.first;
    std::cout << "Nearest found: " <<  found_node << std::endl;

    KDNode2D newTarget(3,5);
    std::cout << "Compare result is " << (target==newTarget) << std::endl;

    std::list<KDNode2D> v;
    double RANGE = 3.0;
    src.find_within_range(target, RANGE, std::back_inserter(v));
    //src.find_within_range(target, RANGE, v);

    std::cout << "found " << v.size() << " nodes within range " << RANGE << "\n";
    std::list<KDNode2D>::const_iterator ci = v.begin();
    for (; ci != v.end(); ++ci)
    std::cout << *ci << " ";
    std::cout << "\n" << std::endl;

    //std::cout << src << std::endl;

    return 0;
    //return a.exec();
}

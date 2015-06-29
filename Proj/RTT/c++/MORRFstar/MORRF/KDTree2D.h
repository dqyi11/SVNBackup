#ifndef KDTREE2D_H
#define KDTREE2D_H

#define KDTREE_DEFINE_OSTREAM_OPERATORS

#include "kdtree++/kdtree.hpp"
#include <functional>
#include <iostream>

class RRTNode;

class POS2D
{
public:
    typedef double value_type;


    POS2D()
    {
        d[0] = 0;
        d[1] = 1;
    }

    POS2D(value_type x, value_type y)
    {
        d[0] = x;
        d[1] = y;
    }

    POS2D(const POS2D & x)
    {
        d[0] = x.d[0];
        d[1] = x.d[1];
    }

    ~POS2D(){}

    double distance_to(POS2D const& x) const
    {
        double dist = 0;
        for (int i = 0; i != 2; ++i)
            dist += (d[i]-x.d[i])*(d[i]-x.d[i]);
        return std::sqrt(dist);
    }

    value_type operator[](size_t const N) { return d[N]; }

    bool operator==(const POS2D &other)
    {
        return d[0] == other.d[0] && d[1] == other.d[1];
    }

    value_type d[2];
};

class KDNode2D : public POS2D
{
public:
    KDNode2D(value_type x, value_type y) : POS2D(x, y) { mNodeList.clear(); }
    KDNode2D(POS2D & pos) : POS2D(pos.d[0], pos.d[1]) { mNodeList.clear(); }

    std::vector<RRTNode*> mNodeList;
};


inline std::ostream& operator<<(std::ostream& out, POS2D const& T)
{
    return out << '(' << T.d[0] << ',' << T.d[1] << ')';
}


inline std::ostream& operator<<(std::ostream& out, KDNode2D const& T)
{
    return out << '(' << T.d[0] << ',' << T.d[1] << ')';
}

inline double tac( KDNode2D t, size_t k ) { return t[k]; }

typedef KDTree::KDTree<2, KDNode2D, std::pointer_to_binary_function<KDNode2D,size_t,double> > KDTree2D;

#endif // KDTREE2D_H

#ifndef SUBTREE_H
#define SUBTREE_H

#include "KDTree2D.h"
#include <vector>
#include <list>

class MORRF;

class RRTNode
{
public:
    RRTNode(POS2D pos, int objective_num);

    bool operator==(const RRTNode &other);

    int mObjectiveNum;
    double * mpCost;
    double mFitness;
    RRTNode * mpParent;
    POS2D mPos;

    std::vector<RRTNode*> mChildNodes;
};

class RRTree
{
public:
    enum TREE_TYPE{ SUBPROBLEM, REFERENCE };
    RRTree(MORRF* parent, int objective_num, double * p_weight);

    RRTNode* init(POS2D start, POS2D goal);
    RRTNode* createNewNode(POS2D pos);
    bool removeEdge(RRTNode* pNode_p, RRTNode* pNode_c);
    bool hasEdge(RRTNode* pNode_p, RRTNode* pNode_c);
    bool addEdge(RRTNode* pNode_p, RRTNode* pNode_c);

    virtual void attachNewNode(RRTNode* pNode_new, KDNode2D node_nearest, std::list<KDNode2D> near_nodes) = 0;
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<KDNode2D> near_nodes) = 0;
    virtual double calcFitness(double * pCost) = 0;
    virtual void updateCostToChildren(RRTNode* pNode, double* pDelta_cost) = 0;

    TREE_TYPE mType;
    int mIndex;
    int mObjectiveNum;

    POS2D mStart;
    POS2D mGoal;

    MORRF* mpParent;
    RRTNode * mpRoot;
    double * mpWeight;

    std::list<RRTNode*> mNodes;
};

class SubproblemTree : public RRTree
{
public:
    SubproblemTree(MORRF* parent, int objective_num, double * p_weight, int index);
    ~SubproblemTree();

    virtual void attachNewNode(RRTNode* pNode_new, KDNode2D node_nearest, std::list<KDNode2D> near_nodes);
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<KDNode2D> near_nodes);
    virtual double calcFitness(double * pCost);
    virtual void updateCostToChildren(RRTNode* pNode, double* pDelta_cost);

};

class ReferenceTree : public RRTree
{
public:
    ReferenceTree(MORRF* parent, int objective_num, int index);
    ~ReferenceTree();

    virtual void attachNewNode(RRTNode* pNode_new, KDNode2D node_nearest, std::list<KDNode2D> near_nodes);
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<KDNode2D> near_nodes);
    virtual double calcFitness(double * pCost);
    virtual void updateCostToChildren(RRTNode* pNode, double* pDelta_cost);
};

#endif // SUBTREE_H

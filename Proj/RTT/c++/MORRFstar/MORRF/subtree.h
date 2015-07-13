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

    std::list<RRTNode*> mChildNodes;
};

class Path
{
public:
    Path(POS2D start, POS2D goal, int objectiveNum);
    ~Path();

    int mObjectiveNum;
    double * mpCost;
    double mFitness;
    POS2D mStart;
    POS2D mGoal;
    std::vector<POS2D> mWaypoints;
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

    std::list<RRTNode*> findAllChildren(RRTNode* pNode);

    virtual void attachNewNode(RRTNode* pNode_new, RRTNode* pNearestNode, std::list<RRTNode*> near_nodes) = 0;
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<RRTNode*> near_nodes) = 0;
    virtual RRTNode * getClosetToGoal(double * deltaCost, double& deltaFitness) = 0;

    bool isStructureCorrect();
    bool areAllNodesTractable();
    bool areAllNodesFitnessPositive();
    RRTNode* findAncestor(RRTNode *pNode);

    Path* findPath();


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

class ReferenceTree : public RRTree
{
public:
    ReferenceTree(MORRF* parent, int objective_num, int index);
    ~ReferenceTree();

    virtual void attachNewNode(RRTNode* pNode_new, RRTNode* pNearestNode, std::list<RRTNode*> near_nodes);
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<RRTNode*> near_nodes);
    virtual RRTNode * getClosetToGoal(double * deltaCost, double& deltaFitness);
protected:
    void updateFitnessToChildren(RRTNode* pNode, double delta_fitness);
};

class SubproblemTree : public RRTree
{
public:
    SubproblemTree(MORRF* parent, int objective_num, double * p_weight, int index);
    ~SubproblemTree();

    virtual void attachNewNode(RRTNode* pNode_new, RRTNode* pNearestNode, std::list<RRTNode*> near_nodes);
    virtual void rewireNearNodes(RRTNode* pNode_new, std::list<RRTNode*> near_nodes);
    virtual RRTNode * getClosetToGoal(double * deltaCost, double& deltaFitness);
protected:
    void updateCostToChildren(RRTNode* pNode, double* pDelta_cost);

};

inline RRTNode* getAncestor(RRTNode * pNode)
{
    if(NULL == pNode)
    {
        return NULL;
    }
    if(NULL == pNode->mpParent)
    {
        return pNode;
    }
    else
    {
        return getAncestor(pNode->mpParent);
    }
}

inline void getParentNodeList(RRTNode * pNode, std::list<RRTNode*>& mPath)
{
    if(pNode==NULL)
    {
        return;
    }
    mPath.push_back(pNode);
    getParentNodeList(pNode->mpParent, mPath);
    return;
}

#endif // SUBTREE_H

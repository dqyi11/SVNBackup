#include "subtree.h"
#include "morrf.h"

RRTNode::RRTNode(POS2D pos, int objective_num)
{
    mPos = pos;
    mObjectiveNum = objective_num;

    mpCost = new double[mObjectiveNum];
    mFitness = 0.0;
    mpParent = NULL;
}

bool RRTNode::operator==(const RRTNode &other)
{
    return mPos==other.mPos;
}

RRTree::RRTree(MORRF* parent, int objective_num)
{
    mpParent = parent;
    mType = REFERENCE;
    mObjectiveNum = objective_num;
    mIndex = -1;

    mpRoot = NULL;
    mNodes.clear();
}

void RRTree::init(POS2D start, POS2D goal)
{
    if(mpRoot)
    {
        delete mpRoot;
        mpRoot = NULL;
    }
    mStart = start;
    mGoal = goal;
    mpRoot = new RRTNode(start, mObjectiveNum);
}

RRTNode*  RRTree::createNewNode(POS2D pos)
{
    RRTNode * pNode = new RRTNode(pos, mObjectiveNum);
    mNodes.push_back(pNode);

    return pNode;
}

bool RRTree::removeEdge(RRTNode* pNode_p, RRTNode*  pNode_c)
{
    if(pNode_p==NULL)
    {
        return false;
    }

    pNode_c->mpParent = NULL;
    bool removed = false;
    for(std::list<RRTNode*>::iterator it=pNode_p->mChildNodes.begin();it!=pNode_p->mChildNodes.end();it++)
    {
        RRTNode* pCurrent = (RRTNode*)(*it);
        if (pCurrent==pNode_c)
        {
            pCurrent->mpParent = NULL;
            it = pNode_p->mChildNodes.erase(it);
            removed = true;
        }
    }
    return removed;
}

bool RRTree::hasEdge(RRTNode* pNode_p, RRTNode* pNode_c)
{
    if (pNode_p==NULL || pNode_c==NULL)
        return false;
    if (pNode_p == pNode_c->mpParent)
        return true;
    return false;
}

bool RRTree::addEdge(RRTNode* pNode_p, RRTNode* pNode_c)
{
    if (pNode_p == pNode_c)
    {
        return false;
    }
    if (hasEdge(pNode_p, pNode_c))
    {
        pNode_c->mpParent = pNode_p;
    }

    pNode_p->mChildNodes.push_back(pNode_c);
    pNode_c->mpParent = pNode_p;

    return true;
}

ReferenceTree::ReferenceTree(MORRF* parent, int objective_num, int index)
    : RRTree(parent, objective_num)
{
    mIndex = index;
}

ReferenceTree::~ReferenceTree()
{

}

double ReferenceTree::calcFitness(double * pCost)
{
    double fitness = 0.0;
    return fitness;
}

void ReferenceTree::attachNewNode(RRTNode* pNode_new, KDNode2D node_nearest, std::list<KDNode2D> near_nodes)
{


}

void ReferenceTree::rewireNearNodes(RRTNode* pNode_new, std::list<KDNode2D> near_nodes)
{

}

void ReferenceTree::updateCostToChildren(RRTNode* pNode, double* pDelta_cost)
{

}

SubproblemTree::SubproblemTree(MORRF* parent, int objective_num, int index)
    : RRTree(parent, objective_num)
{
    mIndex = index;
}

SubproblemTree::~SubproblemTree()
{

}

double SubproblemTree::calcFitness(double * pCost)
{
    double fitness = 0.0;
    return fitness;
}

void SubproblemTree::attachNewNode(RRTNode* pNode_new, KDNode2D node_nearest, std::list<KDNode2D> near_nodes)
{
    double * pMinEdgeCost = new double[mObjectiveNum];
    for(int k=0;k<mObjectiveNum;k++)
    {
        pMinEdgeCost[k] = node_nearest.mNodeList[mObjectiveNum+mIndex]->mpCost[k] + mpParent->calcCost(node_nearest, pNode_new->mPos, k);
    }




}

void SubproblemTree::rewireNearNodes(RRTNode* pNode_new, std::list<KDNode2D> near_nodes)
{

}

void SubproblemTree::updateCostToChildren(RRTNode* pNode, double* pDelta_cost)
{

}

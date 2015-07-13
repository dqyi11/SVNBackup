#include "subtree.h"
#include "morrf.h"
#include <limits>

RRTNode::RRTNode(POS2D pos, int objective_num)
{
    mPos = pos;
    mObjectiveNum = objective_num;

    mpCost = new double[mObjectiveNum];
    for(int k=0;k<mObjectiveNum;k++)
    {
        mpCost[k] = 0.0;
    }
    mFitness = 0.0;
    mpParent = NULL;
}

bool RRTNode::operator==(const RRTNode &other)
{
    return mPos==other.mPos;
}

Path::Path(POS2D start, POS2D goal, int objectiveNum)
{
    mStart = start;
    mGoal = goal;
    mObjectiveNum = objectiveNum;
    mpCost = new double[mObjectiveNum];
    mFitness = 0.0;
}

Path::~Path()
{
    if(mpCost)
    {
        delete mpCost;
        mpCost = NULL;
    }
}

RRTree::RRTree(MORRF* parent, int objective_num, double * p_weight)
{
    mpParent = parent;
    mType = REFERENCE;
    mObjectiveNum = objective_num;
    mIndex = -1;
    mpWeight = new double[mObjectiveNum];
    if(p_weight)
    {
        for(int k=0;k<mObjectiveNum;k++)
        {
            mpWeight[k] = p_weight[k];
        }
    }

    mpRoot = NULL;
    mNodes.clear();
}

RRTNode* RRTree::init(POS2D start, POS2D goal)
{
    if(mpRoot)
    {
        delete mpRoot;
        mpRoot = NULL;
    }
    mStart = start;
    mGoal = goal;
    mpRoot = new RRTNode(start, mObjectiveNum);
    mNodes.push_back(mpRoot);

    return mpRoot;
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


std::list<RRTNode*> RRTree::findAllChildren(RRTNode* pNode)
{
    int level = 0;
    bool finished = false;
    std::list<RRTNode*> child_list;

    std::list<RRTNode*> current_level_nodes;
    current_level_nodes.push_back(pNode);
    while(false==finished)
    {
        std::list<RRTNode*> current_level_children;
        int child_list_num = child_list.size();

        for(std::list<RRTNode*>::iterator it=current_level_nodes.begin(); it!=current_level_nodes.end(); it++)
        {
            RRTNode* pCurrentNode = (*it);
            for(std::list<RRTNode*>::iterator itc=pCurrentNode->mChildNodes.begin(); itc!=pCurrentNode->mChildNodes.end();itc++)
            {
                RRTNode *pChildNode= (*itc);
                if(pChildNode)
                {
                    current_level_children.push_back(pChildNode);
                    child_list.push_back(pChildNode);
                }
            }
        }

        child_list.unique();
        current_level_children.unique();

        if (current_level_children.size()==0)
        {
            finished = true;
        }
        else if (child_list.size()==child_list_num)
        {
            finished = true;
        }
        else
        {
            current_level_nodes.clear();
            for(std::list<RRTNode*>::iterator itt=current_level_children.begin();itt!=current_level_children.end();itt++)
            {
                RRTNode * pTempNode = (*itt);
                if(pTempNode)
                {
                    current_level_nodes.push_back(pTempNode);
                }
            }
            level +=1;
        }

        if(level>100)
        {
            break;
        }
    }

    child_list.unique();
    return child_list;
}

bool RRTree::isStructureCorrect()
{
    for(std::list<RRTNode*>::iterator it=mNodes.begin();it!=mNodes.end();it++)
    {
        RRTNode * pNode = (*it);
        if(pNode)
        {
            for(std::list<RRTNode*>::iterator itc=pNode->mChildNodes.begin();itc!=pNode->mChildNodes.end();itc++)
            {
                RRTNode * pChildNode = (*itc);
                if(pChildNode)
                {
                    if(pChildNode->mFitness < pNode->mFitness)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

RRTNode* RRTree::findAncestor(RRTNode *pNode)
{
    return getAncestor(pNode);
}

Path* RRTree::findPath()
{
    Path* pNewPath = new Path(mStart, mGoal, mObjectiveNum);

    std::list<RRTNode*> node_list;

    RRTNode * pFirstNode = NULL;
    double deltaCost[mObjectiveNum];
    double deltaFitness = 0.0;
    pFirstNode = getClosetToGoal(deltaCost, deltaFitness);

    if(pFirstNode!=NULL)
    {
        getParentNodeList(pFirstNode, node_list);
        for(std::list<RRTNode*>::reverse_iterator rit=node_list.rbegin();
            rit!=node_list.rend(); ++rit)
        {
            RRTNode* pNode = (*rit);
            pNewPath->mWaypoints.push_back(pNode->mPos);
        }
        pNewPath->mWaypoints.push_back(mGoal);

        for(int k=0;k<mObjectiveNum;k++)
        {
            pNewPath->mpCost[k] = pFirstNode->mpCost[k] + deltaCost[k];
        }
        pNewPath->mFitness = pFirstNode->mFitness + deltaFitness;
    }

    return pNewPath;
}

bool RRTree::areAllNodesTractable()
{
    for(std::list<RRTNode*>::iterator it=mNodes.begin();it!=mNodes.end();it++)
    {
        RRTNode * pNode = (*it);
        if(pNode)
        {
            if(mpRoot != findAncestor(pNode))
            {
                return false;
            }
        }
    }
    return true;
}

bool RRTree::areAllNodesFitnessPositive()
{
    for(std::list<RRTNode*>::iterator it=mNodes.begin();it!=mNodes.end();it++)
    {
        RRTNode * pNode = (*it);
        if(pNode)
        {
            if(pNode->mFitness < 0.0)
            {
                return false;
            }
        }
    }
    return true;
}


ReferenceTree::ReferenceTree(MORRF* parent, int objective_num, int index)
    : RRTree(parent, objective_num, NULL)
{
    mIndex = index;
}

ReferenceTree::~ReferenceTree()
{

}

void ReferenceTree::attachNewNode(RRTNode* pNode_new, RRTNode* pNearestNode, std::list<RRTNode*> near_nodes)
{
    double min_new_node_fitness = pNearestNode->mFitness + mpParent->calcCost(pNearestNode->mPos, pNode_new->mPos, mIndex);
    RRTNode* pMinNode = pNearestNode;

    for(std::list<RRTNode*>::iterator it=near_nodes.begin();it!=near_nodes.end();it++)
    {
        RRTNode* pNearNode = *it;
        if (true == mpParent->isObstacleFree(pNearNode->mPos, pNode_new->mPos))
        {
            double fitness = pNearNode->mFitness + mpParent->calcCost(pNearNode->mPos, pNode_new->mPos, mIndex);
            if (fitness < min_new_node_fitness)
            {
                pMinNode = pNearNode;
                min_new_node_fitness = fitness;
            }
        }
    }

    bool added = addEdge(pMinNode, pNode_new);
    if(added)
    {
        pNode_new->mFitness = min_new_node_fitness;
        pNode_new->mpCost[mIndex] = pNode_new->mFitness;
    }

}

void ReferenceTree::rewireNearNodes(RRTNode* pNode_new, std::list<RRTNode*> near_nodes)
{
    for(std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++)
    {
        RRTNode * pNearNode = (*it);

        if(pNearNode->mPos ==pNode_new->mPos ||  pNearNode->mPos==mpRoot->mPos || pNode_new->mpParent->mPos==pNearNode->mPos)
        {
            continue;
        }

        if(true==mpParent->isObstacleFree(pNode_new->mPos, pNearNode->mPos))
        {
            double temp_fitness_from_new_node = pNode_new->mFitness + mpParent->calcCost(pNode_new->mPos, pNearNode->mPos, mIndex);
            if(temp_fitness_from_new_node < pNearNode->mFitness)
            {
                double delta_fitness = pNearNode->mFitness - temp_fitness_from_new_node;
                RRTNode * pParentNode = pNearNode->mpParent;
                bool removed = removeEdge(pParentNode, pNearNode);
                if(removed)
                {
                    bool added = addEdge(pNode_new, pNearNode);
                    if(added)
                    {
                        pNearNode->mFitness = temp_fitness_from_new_node;
                        pNearNode->mpCost[mIndex] = pNearNode->mFitness;
                        updateFitnessToChildren(pNearNode, delta_fitness);
                    }
                }
            }
        }
    }
}

void ReferenceTree::updateFitnessToChildren(RRTNode* pNode, double delta_fitness)
{
    std::list<RRTNode*> child_list = findAllChildren(pNode);
    for(std::list<RRTNode*>::iterator it=child_list.begin();it!=child_list.end();it++)
    {
        RRTNode* pChildNode = (*it);
        if(pChildNode)
        {
            pChildNode->mFitness -= delta_fitness;
            pChildNode->mpCost[mIndex] = pChildNode->mFitness;
        }
    }
}

RRTNode * ReferenceTree::getClosetToGoal(double * deltaCost, double& deltaFitness)
{
    RRTNode* pClosestNode = NULL;
    if(mpParent)
    {
        std::list<KDNode2D> near_nodes = mpParent->findNear(mGoal);
        double min_total_fitness = std::numeric_limits<double>::max();
        double min_delta_fitness = 0.0;
        RRTNode * pMinPrevNode = NULL;
        for(std::list<KDNode2D>::iterator it=near_nodes.begin();
            it!=near_nodes.end();it++)
        {
            KDNode2D kd_node = (*it);
            RRTNode* pNode = kd_node.mNodeList[mIndex];
            double delta_fitness = mpParent->calcCost(pNode->mPos, mGoal, mIndex);
            double new_total_fitness = pNode->mFitness + delta_fitness;
            if (new_total_fitness < min_total_fitness)
            {
                pMinPrevNode = pNode;
                min_delta_fitness = delta_fitness;
                min_total_fitness = new_total_fitness;
            }
        }
        pClosestNode = pMinPrevNode;
        deltaFitness = min_delta_fitness;
        for(int k=0;k<mObjectiveNum;k++)
        {
            if(k==mIndex)
            {
                deltaCost[k] = deltaFitness;
            }
            else
            {
                deltaCost[k] = 0.0;
            }
        }

    }
    return pClosestNode;
}


SubproblemTree::SubproblemTree(MORRF* parent, int objective_num, double * p_weight, int index)
    : RRTree(parent, objective_num, p_weight)
{
    mIndex = index;
}

SubproblemTree::~SubproblemTree()
{

}


void SubproblemTree::attachNewNode(RRTNode* pNode_new, RRTNode* pNearestNode, std::list<RRTNode*> near_nodes)
{
    double p_min_new_node_cost[mObjectiveNum];
    double p_min_new_node_cost_delta[mObjectiveNum];
    mpParent->calcCost(pNearestNode->mPos, pNode_new->mPos, p_min_new_node_cost_delta);
    for(int k=0;k<mObjectiveNum;k++)
    {
        p_min_new_node_cost[k] = pNearestNode->mpCost[k] + p_min_new_node_cost_delta[k];
    }
    double min_new_node_fitness = mpParent->calcFitness(p_min_new_node_cost, mpWeight, pNode_new->mPos);

    RRTNode* pMinNode = pNearestNode;

    for(std::list<RRTNode*>::iterator it=near_nodes.begin();it!=near_nodes.end();it++)
    {
        RRTNode* pNearNode = (*it);
        if (true == mpParent->isObstacleFree(pNearNode->mPos, pNode_new->mPos))
        {
            double p_cost_temp[mObjectiveNum];
            double p_cost_delta[mObjectiveNum];
            mpParent->calcCost(pNearNode->mPos, pNode_new->mPos, p_cost_delta);
            for(int k=0;k<mObjectiveNum;k++)
            {
                p_cost_temp[k] = pNearNode->mpCost[k] + p_cost_delta[k];
            }
            double fitness = mpParent->calcFitness(p_cost_temp, mpWeight, pNode_new->mPos);
            if (fitness < min_new_node_fitness)
            {
                pMinNode = pNearNode;
                min_new_node_fitness = fitness;
                for(int k=0;k<mObjectiveNum;k++)
                {
                    p_min_new_node_cost[k] = p_cost_temp[k];
                }
            }
        }
    }

    bool added = addEdge(pMinNode, pNode_new);
    if(added)
    {
        pNode_new->mFitness = min_new_node_fitness;
        for(int k=0;k<mObjectiveNum;k++)
        {
           pNode_new->mpCost[k] = p_min_new_node_cost[k];
        }
    }
}

void SubproblemTree::rewireNearNodes(RRTNode* pNode_new, std::list<RRTNode*> near_nodes)
{
    for(std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++)
    {
        RRTNode * pNearNode = (*it);

        if(pNearNode->mPos == pNode_new->mPos ||  pNearNode->mPos == mpRoot->mPos || pNode_new->mpParent->mPos == pNearNode->mPos)
        {
            continue;
        }

        if(true==mpParent->isObstacleFree(pNode_new->mPos, pNearNode->mPos))
        {
            double temp_cost_from_new_node[mObjectiveNum];
            double temp_delta_cost_from_new_node[mObjectiveNum];

            mpParent->calcCost(pNode_new->mPos, pNearNode->mPos, temp_delta_cost_from_new_node);
            for(int k=0;k<mObjectiveNum;k++)
            {
                temp_cost_from_new_node[k] = pNode_new->mpCost[k] + temp_delta_cost_from_new_node[k];
            }
            double temp_fitness_from_new_node = mpParent->calcFitness(temp_cost_from_new_node, mpWeight, pNearNode->mPos);

            if(temp_fitness_from_new_node < pNearNode->mFitness)
            {
                RRTNode * pParentNode = pNearNode->mpParent;
                bool removed = removeEdge(pParentNode, pNearNode);
                if(removed)
                {
                    bool added = addEdge(pNode_new, pNearNode);
                    if(added)
                    {
                        double delta_cost[mObjectiveNum];
                        for(int k=0;k<mObjectiveNum;k++)
                        {
                            delta_cost[k] = pNearNode->mpCost[k] - temp_cost_from_new_node[k];
                            pNearNode->mpCost[k] = temp_cost_from_new_node[k];
                            pNearNode->mFitness = temp_fitness_from_new_node;
                        }
                        updateCostToChildren(pNearNode, delta_cost);
                    }
                }
            }
        }

    }

}

void SubproblemTree::updateCostToChildren(RRTNode* pNode, double* pDelta_cost)
{
    std::list<RRTNode*> child_list = findAllChildren(pNode);
    for(std::list<RRTNode*>::iterator it=child_list.begin();it!=child_list.end();it++)
    {
        RRTNode* pChildNode = (*it);
        if(pChildNode)
        {
            for(int k=0;k<mObjectiveNum;k++)
            {
                pChildNode->mpCost[k] -= pDelta_cost[k];
            }
            pChildNode->mFitness = mpParent->calcFitness(pChildNode->mpCost, mpWeight, pChildNode->mPos);
        }
    }
}

RRTNode * SubproblemTree::getClosetToGoal(double * deltaCost, double& deltaFitness)
{
    RRTNode * pClosestNode = NULL;
    if(mpParent)
    {
        std::list<KDNode2D> near_nodes = mpParent->findNear(mGoal);
        double min_total_fitness = std::numeric_limits<double>::max();
        double min_delta_fitness = 0.0;
        double min_delta_cost[mObjectiveNum];
        RRTNode * pMinPrevNode = NULL;
        for(std::list<KDNode2D>::iterator it=near_nodes.begin();
            it!=near_nodes.end();it++)
        {
            KDNode2D kd_node = (*it);
            int index = mIndex + mObjectiveNum;
            RRTNode* pNode = kd_node.mNodeList[index];
            double new_delta_cost[mObjectiveNum];
            mpParent->calcCost(pNode->mPos, mGoal, new_delta_cost);
            double new_delta_fitness = mpParent->calcFitness(new_delta_cost, mpWeight, mGoal);
            double new_total_fitness = pNode->mFitness + new_delta_fitness;
            if (new_total_fitness < min_total_fitness)
            {
                pMinPrevNode = pNode;
                min_delta_fitness = new_delta_fitness;
                min_total_fitness = new_total_fitness;
                for(int k=0;k<mObjectiveNum;k++)
                {
                    min_delta_cost[k] = new_delta_cost[k];
                }
            }
        }
        pClosestNode = pMinPrevNode;
        deltaFitness = min_delta_fitness;
        for(int k=0;k<mObjectiveNum;k++)
        {
            deltaCost[k] = min_delta_cost[k];
        }

    }
    return pClosestNode;
}


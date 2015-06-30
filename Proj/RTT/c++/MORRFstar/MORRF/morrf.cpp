
#include "morrf.h"
#include <cstdlib>

MORRF::MORRF(int width, int height, int objective_num, int subproblem_num, MORRF_TYPE type)
{
    mSamplingWidth = width;
    mSamplingHeight = height;
    mObjectiveNum = objective_num;
    mSubproblemNum = subproblem_num;
    mType = type;

    mpKDTree = new KDTree2D(std::ptr_fun(tac));

    mRange = 100.0;
    mObsCheckResolution = 1;
}

MORRF::~MORRF()
{
    deinitWeights();

    if(mpKDTree)
    {
        delete mpKDTree;
        mpKDTree = NULL;
    }
}

void MORRF::addFuncs( std::vector<COST_FUNC_PTR> funcs, std::vector<int**> fitnessDistributions)
{
    mFuncs = funcs;
    mFitnessDistributions = fitnessDistributions;
}

void MORRF::initWeights()
{
    deinitWeights();

    if (mObjectiveNum == 2)
    {
        for(int i=0;i<mSubproblemNum;i++)
        {
            double * weight = new double[mObjectiveNum];
            weight[0] = (double)(i+1) / (double)(mSubproblemNum+2);
            weight[1] = (double)(mSubproblemNum-i+1) / (double)(mSubproblemNum+2);
        }
    }
    else
    {
        for(int i=0;i<mSubproblemNum;i++)
        {
            double * weight = new double[mObjectiveNum];
            for(int j=0;j<mObjectiveNum;j++)
            {
                weight[j] = (double)rand()/RAND_MAX;
            }
            mWeights.push_back(weight);
        }
    }
}

void MORRF::deinitWeights()
{
    for(std::vector<double*>::iterator it=mWeights.begin();it!=mWeights.end();it++)
    {
        double * weight = (double*)(*it);
        delete weight;
        weight = NULL;
    }
    mWeights.clear();
}

void MORRF::init(POS2D start, POS2D goal)
{
    initWeights();

    for(int k=0;k<mObjectiveNum;k++)
    {
        ReferenceTree * pRefTree = new ReferenceTree(this, mObjectiveNum, k);
        pRefTree->init(start, goal);
        mReferences.push_back(pRefTree);
    }

    for(int m=0;m<mSubproblemNum;m++)
    {
        SubproblemTree * pSubTree = new SubproblemTree(this, mObjectiveNum, m+mObjectiveNum);
        pSubTree->init(start, goal);
        mSubproblems.push_back(pSubTree);
    }
}

void MORRF::loadMap(int **map)
{

}

POS2D MORRF::sampling()
{
    double x = rand();
    double y = rand();
    x = x * ((double)(mSamplingWidth)/RAND_MAX);
    y = y * ((double)(mSamplingHeight)/RAND_MAX);

    POS2D m(x,y);
    return m;
}

POS2D MORRF::steer(POS2D pos_a, POS2D pos_b)
{
    double delta[2];
    delta[0] = pos_a[0] - pos_b[0];
    delta[1] = pos_a[1] - pos_b[1];
    double delta_len = std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]);
    double scale = mSegmentLength / delta_len;
    delta[0] *= scale;
    delta[1] *= scale;

    POS2D new_pos(pos_b[0]+delta[0], pos_b[1]+delta[1]);

    return new_pos;
}

bool MORRF::isInObstacle(POS2D pos)
{
    int x = (int)pos[0];
    int y = (int)pos[1];
    if( mpObstacle[x][y] < 255)
        return true;
    return false;
}

bool MORRF::isObstacleFree(POS2D pos_a, POS2D pos_b)
{
    if (pos_a == pos_b)
        return true;
    int x_dist = std::abs(pos_a[0] - pos_b[0]);
    int y_dist = std::abs(pos_a[1] - pos_b[1]);
    if (x_dist > y_dist)
    {
        double k = (double)y_dist/ x_dist;
        int startX, endX, startY;
        if (pos_a[0] < pos_b[0])
        {
            startX = pos_a[0];
            endX = pos_b[0];
            startY = pos_a[1];
        }
        else
        {
            startX = pos_b[0];
            endX = pos_a[0];
            startY = pos_b[1];
        }
        for (int coordX = startX; coordX < endX + mObsCheckResolution ; coordX+=mObsCheckResolution)
        {
            int coordY = (int)(k*(coordX-startX)+startY);
            if (coordY > mSamplingHeight || coordX <= mSamplingWidth) break;
            if (mpObstacle[coordX][coordY] < 255)
                return false;
        }
    }
    else
    {
        double k = (double)x_dist/ y_dist;
        int startY, endY, startX;
        if (pos_a[1] < pos_b[1])
        {
            startY = pos_a[1];
            endY = pos_b[1];
            startX = pos_a[0];
        }
        else
        {
            startY = pos_b[1];
            endY = pos_a[1];
            startX = pos_b[0];
        }
        for (int coordY = startY; coordY < endY + mObsCheckResolution ; coordY+=mObsCheckResolution)
        {
            int coordX = (int)(k*(coordY-startY)+startX);
            if (coordY > mSamplingHeight || coordX <= mSamplingWidth) break;
            if (mpObstacle[coordX][coordY] < 255)
                return false;
        }
    }
    return true;
}

void MORRF::extend()
{
    KDNode2D * pNewNode = NULL;
    while(pNewNode==NULL)
    {
        POS2D rndPos = sampling();
        KDNode2D nearest_node = findNearest(rndPos);

        POS2D new_pos = steer(rndPos, nearest_node);

        if( true==isInObstacle(new_pos) )
        {
            continue;
        }

        if(true==isObstacleFree(nearest_node, new_pos))
        {
            std::list<KDNode2D> near_nodes = findNear(new_pos);

            KDNode2D new_node(new_pos);

            // create new nodes of reference trees
            for(int k=0;k<mObjectiveNum;k++)
            {
                RRTNode * pNewRefNode = mReferences[k]->createNewNode(new_pos);
                new_node.mNodeList.push_back(pNewRefNode);
            }

            // create new nodes of subproblem trees
            for (int m=0;m<mSubproblemNum;m++)
            {
                RRTNode * pNewSubNode = mSubproblems[m]->createNewNode(new_pos);
                new_node.mNodeList.push_back(pNewSubNode);
            }

            // attach new node to reference trees
            // rewire near nodes of reference trees
            for (int k=0;k<mObjectiveNum;k++)
            {
                mReferences[k]->attachNewNode(new_node.mNodeList[k], nearest_node, near_nodes);
                mReferences[k]->rewireNearNodes(new_node.mNodeList[k], near_nodes);
            }


            // attach new nodes to subproblem trees
            // rewire near nodes of subproblem trees
            for(int m=0;m<mSubproblemNum;m++)
            {
                mSubproblems[m]->attachNewNode(new_node.mNodeList[m+mObjectiveNum], nearest_node, near_nodes);
                mSubproblems[m]->rewireNearNodes(new_node.mNodeList[m+mObjectiveNum], near_nodes);
            }
        }
    }
}

KDNode2D MORRF::findNearest(POS2D pos)
{
    KDNode2D node(pos);

    std::pair<KDTree2D::const_iterator,double> found = mpKDTree->find_nearest(node);
    KDNode2D near_node = *found.first;
    return near_node;
}

std::list<KDNode2D> MORRF::findNear(POS2D pos)
{
    std::list<KDNode2D> near_list;
    KDNode2D node(pos);

    mpKDTree->find_within_range(node, mRange, std::back_inserter(near_list));

    return near_list;
}

double * MORRF::calcCost(POS2D& pos_a, POS2D& pos_b)
{
    double * pCost = new double[mObjectiveNum];
    for(int k=0;k<mObjectiveNum;k++)
    {
        pCost[k] = calcCost(pos_a, pos_b, k);
    }

    return pCost;
}

double MORRF::calcCost(POS2D& pos_a, POS2D& pos_b, int k)
{
    return mFuncs[k](pos_a, pos_b, mFitnessDistributions[k]);
}

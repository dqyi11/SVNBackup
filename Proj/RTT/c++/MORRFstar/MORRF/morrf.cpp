
#include "morrf.h"
#include <cstdlib>
#include <iostream>
#include <fstream>

#define OBSTACLE_THRESHOLD 200

MORRF::MORRF(int width, int height, int objective_num, int subproblem_num, int segmentLength, MORRF_TYPE type)
{
    mSamplingWidth = width;
    mSamplingHeight = height;
    mObjectiveNum = objective_num;
    mSubproblemNum = subproblem_num;
    mType = type;

    mpKDTree = new KDTree2D(std::ptr_fun(tac));

    mRange = 100.0;
    mObsCheckResolution = 1;
    mCurrentIteration = 0;
    mSegmentLength = segmentLength;

    mpWeights = NULL;

    mpMapInfo = new int*[mSamplingWidth];
    for(int i=0;i<mSamplingWidth;i++)
    {
        mpMapInfo[i] = new int[mSamplingHeight];
    }
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

    mpWeights = new double*[mSubproblemNum];

    if (mObjectiveNum == 2)
    {
        for(int i=0;i<mSubproblemNum;i++)
        {
            mpWeights[i] = new double[mObjectiveNum];
            mpWeights[i][0] = (double)(i+1) / (double)(mSubproblemNum+2);
            mpWeights[i][1] = (double)(mSubproblemNum-i+1) / (double)(mSubproblemNum+2);
        }
    }
    else
    {
        for(int i=0;i<mSubproblemNum;i++)
        {
            mpWeights[i] = new double[mObjectiveNum];
            for(int j=0;j<mObjectiveNum;j++)
            {
                mpWeights[i][j] = (double)rand()/RAND_MAX;
            }
        }
    }
}

void MORRF::deinitWeights()
{
    if(mpWeights)
    {
        int size = sizeof(mpWeights)/sizeof(double*);
        for(int i=0;i<size;i++)
        {
            if(mpWeights[i])
            {
                delete mpWeights[i];
                mpWeights[i] = NULL;
            }
        }
    }
}

void MORRF::init(POS2D start, POS2D goal)
{
    initWeights();

    KDNode2D root(start);
    for(int k=0;k<mObjectiveNum;k++)
    {
        ReferenceTree * pRefTree = new ReferenceTree(this, mObjectiveNum, k);
        RRTNode * pRootNode = pRefTree->init(start, goal);
        root.mNodeList.push_back(pRootNode);
        mReferences.push_back(pRefTree);
    }

    for(int m=0;m<mSubproblemNum;m++)
    {
        SubproblemTree * pSubTree = new SubproblemTree(this, mObjectiveNum, mpWeights[m], m);
        RRTNode * pRootNode = pSubTree->init(start, goal);
        root.mNodeList.push_back(pRootNode);
        mSubproblems.push_back(pSubTree);
    }
    mpKDTree->insert(root);
    mCurrentIteration = 0;
}

void MORRF::loadMap(int **map)
{
    for(int i=0;i<mSamplingWidth;i++)
    {
        for(int j=0;j<mSamplingHeight;j++)
        {
            mpMapInfo[i][j] = map[i][j];
        }
    }
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
    POS2D new_pos(pos_a[0], pos_a[1]);
    double delta[2];
    delta[0] = pos_a[0] - pos_b[0];
    delta[1] = pos_a[1] - pos_b[1];
    double delta_len = std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]);

    if (delta_len > mSegmentLength)
    {
        double scale = mSegmentLength / delta_len;
        delta[0] = delta[0] * scale;
        delta[1] = delta[1] * scale;

        new_pos.setX( pos_b[0]+delta[0] );
        new_pos.setY( pos_b[1]+delta[1] );
    }
    return new_pos;
}

bool MORRF::isInObstacle(POS2D pos)
{
    int x = (int)pos[0];
    int y = (int)pos[1];
    if( mpMapInfo[x][y] < 255)
        return true;
    return false;
}

bool MORRF::isObstacleFree(POS2D pos_a, POS2D pos_b)
{
    if (pos_a == pos_b)
        return true;
    int x_dist = pos_a[0] - pos_b[0];
    int y_dist = pos_a[1] - pos_b[1];
    if (std::abs(x_dist) > std::abs(y_dist))
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
            if (coordY >= mSamplingHeight || coordX >= mSamplingWidth) break;
            if (mpMapInfo[coordX][coordY] < OBSTACLE_THRESHOLD)
            {
                return false;
            }
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
            if (coordY >= mSamplingHeight || coordX >= mSamplingWidth) break;
            if (mpMapInfo[coordX][coordY] < OBSTACLE_THRESHOLD)
            {
                return false;
            }
        }
    }
    return true;
}

void MORRF::extend()
{
    bool node_inserted = false;
    while(false==node_inserted)
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

            mpKDTree->insert(new_node);
            node_inserted = true;

            // attach new node to reference trees
            // rewire near nodes of reference trees
            for (int k=0;k<mObjectiveNum;k++)
            {
                // std::cout << "@ " << k << std::endl;
                mReferences[k]->attachNewNode(new_node.mNodeList[k], nearest_node, near_nodes);
                mReferences[k]->rewireNearNodes(new_node.mNodeList[k], near_nodes);
            }

            // attach new nodes to subproblem trees
            // rewire near nodes of subproblem trees
            for(int m=0;m<mSubproblemNum;m++)
            {
                // std::cout << "@ " << m+mObjectiveNum << std::endl;
                mSubproblems[m]->attachNewNode(new_node.mNodeList[m+mObjectiveNum], nearest_node, near_nodes);
                //mSubproblems[m]->rewireNearNodes(new_node.mNodeList[m+mObjectiveNum], near_nodes);
            }
        }

        mCurrentIteration++;
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

bool MORRF::calcCost(POS2D& pos_a, POS2D& pos_b, double * p_cost)
{
    if (p_cost==NULL)
    {
        return false;
    }
    for(int k=0;k<mObjectiveNum;k++)
    {
        p_cost[k] = calcCost(pos_a, pos_b, k);
    }
    return true;
}

double MORRF::calcCost(POS2D& pos_a, POS2D& pos_b, int k)
{
    return mFuncs[k](pos_a, pos_b, mFitnessDistributions[k]);
}

double MORRF::calcFitness(double * p_cost, double * p_weight, POS2D& pos)
{
    double fitness = 0.0;
    if(p_cost == NULL || p_weight==NULL)
    {
        return fitness;
    }
    if(mType==MORRF::WEIGHTED_SUM)
    {
        for(int k=0;k<mObjectiveNum;k++)
        {
            fitness += p_cost[k] * p_weight[k];
        }
    }
    else if(mType==MORRF::TCHEBYCHEFF)
    {
        double p_utopia[mObjectiveNum];
        if(true == getUtopiaReferenceVector(pos, p_utopia))
        {
            for(int k=0;k<mObjectiveNum;k++)
            {
               double weighted_dist = p_weight[k] * std::abs(p_cost[k] - p_utopia[k]);
               if (weighted_dist > fitness)
               {
                   fitness = weighted_dist;
               }
            }
        }
    }
    else
    {

    }

    return fitness;
}

bool MORRF::getUtopiaReferenceVector(POS2D& pos, double * p_utopia)
{
    if (p_utopia==NULL)
    {
        return false;
    }
    KDNode2D ref_node = findNearest(pos);
    if(ref_node.mNodeList.size()<mObjectiveNum)
    {
        return false;
    }

    for(int k=0;k<mObjectiveNum;k++)
    {
        RRTNode* pRRTNode = ref_node.mNodeList[k];
        p_utopia[k] = pRRTNode->mFitness;
    }
    return true;
}

ReferenceTree* MORRF::getReferenceTree(int k)
{
    if(k<0 || k>=mObjectiveNum)
    {
        return NULL;
    }
    return mReferences[k];
}

SubproblemTree* MORRF::getSubproblemTree(int m)
{
    if(m<0 || m>=mSubproblemNum)
    {
        return NULL;
    }
    return mSubproblems[m];
}

void MORRF::dumpMapInfo( std::string filename )
{
    std::ofstream mapInfoFile;
    mapInfoFile.open(filename.c_str());
    if(mpMapInfo)
    {

        for(int j=0;j<mSamplingHeight;j++)
        {
            for(int i=0;i<mSamplingWidth;i++)
            {
                mapInfoFile << mpMapInfo[i][j] << " ";
            }
            mapInfoFile << std::endl;
        }
    }
    mapInfoFile.close();
}

bool MORRF::isStructureCorrect()
{
    for(std::vector<ReferenceTree*>::iterator it=mReferences.begin();it!=mReferences.end();it++)
    {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree)
        {
            if(false==pRefTree->isStructureCorrect())
            {
                return false;
            }
        }
    }

    /*
    for(std::vector<SubproblemTree*>::iterator it=mSubproblems.begin();it!=mSubproblems.end();it++)
    {
        SubproblemTree* pRefTree = (*it);
        if(pRefTree)
        {
            if(false==pRefTree->isStructureCorrect())
            {
                return false;
            }
        }
    }
    */
    return true;
}

bool MORRF::areAllNodesTractable()
{
    for(std::vector<ReferenceTree*>::iterator it=mReferences.begin();it!=mReferences.end();it++)
    {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree)
        {
            if(false==pRefTree->areAllNodesTractable())
            {
                return false;
            }
        }
    }

    /*
    for(std::vector<SubproblemTree*>::iterator it=mSubproblems.begin();it!=mSubproblems.end();it++)
    {
        SubproblemTree* pRefTree = (*it);
        if(pRefTree)
        {
            if(false==pRefTree->areAllNodesTractable())
            {
                return false;
            }
        }
    }
    */
    return true;
}

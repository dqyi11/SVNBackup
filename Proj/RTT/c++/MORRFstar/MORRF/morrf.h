#ifndef MORRF_H
#define MORRF_H

#include "KDTree2D.h"
#include "subtree.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D,int**);


class MORRF
{
public:
    enum MORRF_TYPE{ WEIGHTED_SUM, TCHEBYCHEFF, BOUNDARY_INTERSACTION };
    MORRF(int width, int height, int objective_num, int subproblem_num, MORRF_TYPE type=WEIGHTED_SUM);
    ~MORRF();

    void addFuncs( std::vector<COST_FUNC_PTR> funcs, std::vector<int**> fitnessDistributions);

    void init(POS2D start, POS2D goal, std::vector<COST_FUNC_PTR> funcs);

    void loadMap(int **map);
    POS2D sampling();
    POS2D steer(POS2D pos_a, POS2D pos_b);
    void extend();

    KDNode2D findNearest(POS2D pos);
    std::list<KDNode2D> findNear(POS2D pos);

    double* getReferenceCost(POS2D pos);
    bool isObstacleFree(POS2D pos_a, POS2D pos_b);
    bool isInObstacle(POS2D pos);

    double * calcCost(POS2D& pos_a, POS2D& pos_b);
    double calcCost(POS2D& pos_a, POS2D& pos_b, int k);

    int getSamplingWidth() { return mSamplingWidth; }
    int getSamplingHeight() { return mSamplingHeight; }

    void setObstacleInfo(int ** pObstacle) { mpObstacle = pObstacle; }

protected:
    void initWeights();
    void deinitWeights();

private:
    int ** mpObstacle;

    MORRF_TYPE mType;
    int mSamplingWidth;
    int mSamplingHeight;

    int mObjectiveNum;
    int mSubproblemNum;

    KDTree2D * mpKDTree;

    std::vector<COST_FUNC_PTR> mFuncs;
    std::vector<int**> mFitnessDistributions;

    std::vector<double *> mWeights;

    std::vector<SubproblemTree> mSubproblems;
    std::vector<ReferenceTree> mReferences;

    double mRange;
    double mSegmentLength;
    int mObsCheckResolution;
};

#endif // MORRF_H

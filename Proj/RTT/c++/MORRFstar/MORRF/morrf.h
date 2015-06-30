#ifndef MORRF_H
#define MORRF_H

#include "KDTree2D.h"
#include "subtree.h"




class MORRF
{
public:
    enum MORRF_TYPE{ WEIGHTED_SUM, TCHEBYCHEFF, BOUNDARY_INTERSACTION };
    MORRF(int width, int height, int objective_num, int subproblem_num, MORRF_TYPE type=WEIGHTED_SUM);
    ~MORRF();

    void addFunc( COST_FUNC_PTR func, int objective_idx );

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

    COST_FUNC_PTR* mpFuncs;

    std::vector<double *> mWeights;

    std::vector<SubproblemTree> mSubproblems;
    std::vector<ReferenceTree> mReferences;

    double mRange;
    double mSegmentLength;
    int mObsCheckResolution;
};

#endif // MORRF_H

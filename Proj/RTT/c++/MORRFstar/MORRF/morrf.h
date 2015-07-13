#ifndef MORRF_H
#define MORRF_H

#include "KDTree2D.h"
#include "subtree.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D,int**);


class MORRF
{
public:
    enum MORRF_TYPE{ WEIGHTED_SUM, TCHEBYCHEFF, BOUNDARY_INTERSACTION };
    MORRF(int width, int height, int objective_num, int subproblem_num, int segmentLength, MORRF_TYPE type=WEIGHTED_SUM);
    ~MORRF();

    void addFuncs( std::vector<COST_FUNC_PTR> funcs, std::vector<int**> fitnessDistributions );

    void init(POS2D start, POS2D goal);

    void loadMap(int **map);
    POS2D sampling();
    POS2D steer(POS2D pos_a, POS2D pos_b);
    void extend();

    KDNode2D findNearest(POS2D pos);
    std::list<KDNode2D> findNear(POS2D pos);

    bool isObstacleFree(POS2D pos_a, POS2D pos_b);
    bool isInObstacle(POS2D pos);
    bool contains(POS2D pos);

    bool calcCost(POS2D& pos_a, POS2D& pos_b, double * p_cost);
    double calcCost(POS2D& pos_a, POS2D& pos_b, int k);
    double calcFitness(double * p_cost, double * p_weight, POS2D& pos);

    bool getUtopiaReferenceVector(POS2D& pos, double * p_utopia);

    int getSamplingWidth() { return mSamplingWidth; }
    int getSamplingHeight() { return mSamplingHeight; }

    void setObstacleInfo(int ** pObstacle) { mpMapInfo = pObstacle; }

    int getCurrentIteration() { return mCurrentIteration; }

    ReferenceTree* getReferenceTree(int k);
    SubproblemTree* getSubproblemTree(int m);

    std::vector<Path*> getPaths();

    int** getMapInfo() { return mpMapInfo; };

    void dumpMapInfo( std::string filename );

    bool areReferenceStructuresCorrect();
    bool areSubproblemStructuresCorrect();
    bool areAllReferenceNodesTractable();
    bool areAllSubproblemNodesTractable();
    bool areAllReferenceNodesFitnessPositive();
    bool areAllSubproblemNodesFitnessPositive();
    bool isNodeNumberIdentical();
    double getBallRadius() { return mBallRadius; };

protected:
    void initWeights();
    void deinitWeights();

private:
    int ** mpMapInfo;

    MORRF_TYPE mType;
    int mSamplingWidth;
    int mSamplingHeight;

    int mObjectiveNum;
    int mSubproblemNum;

    KDTree2D * mpKDTree;

    std::vector<COST_FUNC_PTR> mFuncs;
    std::vector<int**> mFitnessDistributions;

    double** mpWeights;

    std::vector<SubproblemTree*> mSubproblems;
    std::vector<ReferenceTree*> mReferences;

    double mRange;
    double mBallRadius;
    double mSegmentLength;
    int mObsCheckResolution;

    int mCurrentIteration;
};

#endif // MORRF_H

#include "morrfvisualizer.h"
#include <QtGui>

MORRFVisualizer::MORRFVisualizer(QWidget *parent) :
    QLabel(parent)
{
    mpMORRF = NULL;
    mCurrentTreeIdx = 0;
}

void MORRFVisualizer::setMORRF(MORRF* pMorrf)
{
    mpMORRF = pMorrf;
}

void MORRFVisualizer::prevTree()
{
    if(mpMORRF)
    {
        if(mCurrentTreeIdx > 0)
        {
            mCurrentTreeIdx --;
        }
        else
        {
            mCurrentTreeIdx = mMOPPInfo.mSubproblemNum+mMOPPInfo.mObjectiveNum-1;
        }
    }
}

void MORRFVisualizer::nextTree()
{
    if(mpMORRF)
    {
        if(mCurrentTreeIdx < mMOPPInfo.mSubproblemNum+mMOPPInfo.mObjectiveNum-1)
        {
            mCurrentTreeIdx ++;
        }
        else
        {
            mCurrentTreeIdx = 0;
        }
    }
}

void MORRFVisualizer::paintEvent(QPaintEvent * e)
{
    QLabel::paintEvent(e);

    if(mpMORRF)
    {
        RRTree * pTree = NULL;
        if(mCurrentTreeIdx < mMOPPInfo.mObjectiveNum)
        {
            pTree = mpMORRF->getReferenceTree(mCurrentTreeIdx);
        }
        else
        {
            pTree = mpMORRF->getSubproblemTree(mCurrentTreeIdx - mMOPPInfo.mObjectiveNum);
        }

        if(pTree)
        {
            QPainter painter(this);
            QPen paintpen(QColor(0,255,0));
            paintpen.setWidth(1);
            painter.setPen(paintpen);

            for(std::list<RRTNode*>::iterator it= pTree->mNodes.begin(); it!=pTree->mNodes.end();it++)
            {
                RRTNode* pNode = (*it);
                if(pNode)
                {
                    for(std::vector<RRTNode*>::iterator itc= pNode->mChildNodes.begin(); itc!=pNode->mChildNodes.end();itc++)
                    {
                        RRTNode* pChildNode = (*itc);
                        if(pChildNode)
                        {
                            painter.drawLine(QPoint(pNode->mPos[0], pNode->mPos[1]), QPoint(pChildNode->mPos[0], pChildNode->mPos[1]));
                        }
                    }
                }
            }
        }
    }

    if(mMOPPInfo.mStart.x() >= 0 && mMOPPInfo.mStart.y() >= 0)
    {
        QPainter painter(this);
        QPen paintpen(QColor(255,0,0));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mStart);
    }

    if(mMOPPInfo.mGoal.x() >= 0 && mMOPPInfo.mGoal.y() >= 0)
    {
        QPainter painter(this);
        QPen paintpen(QColor(0,0,255));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mGoal);
    }
}

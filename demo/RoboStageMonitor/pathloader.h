#ifndef PATHLOADER_H
#define PATHLOADER_H

#include "discretepath.h"
#include <QDir>

class PathLoader
{
public:
    PathLoader();

    bool loadFile(const QString &filename);

    QString getMapFilepath();
    DiscretePath * getPath();

    DiscretePath * mpPath;
    QString * mpMapFile;
    QDir * mpMapDir;
};

#endif // PATHLOADER_H

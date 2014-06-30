#include "pathloader.h"
#include <QXmlStreamReader>
#include <QFile>
#include <QFileInfo>

PathLoader::PathLoader()
{
    mpPath = NULL;
    mpMapFile = new QString();
    mpMapDir = new QDir();
}


bool PathLoader::loadFile(const QString &filename)
{
    QFile file(filename);
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return false;
    }

    QFileInfo info(filename);
    *mpMapDir = info.absoluteDir();

    QXmlStreamReader reader;
    reader.setDevice(&file);
    mpPath = new DiscretePath();

    while(!reader.atEnd())
    {
        reader.readNext();
        if(reader.isStartElement())
        {
            if(reader.name()=="path")
            {
                mpMapFile->append(reader.attributes().value("map"));
            }
            else if(reader.name() == "position")
            {
                QString posX = "";
                QString posY = "";
                posX.append(reader.attributes().value("pos_x"));
                posY.append(reader.attributes().value("pos_y"));

                mpPath->addPoint(posX.toFloat(), posY.toFloat());
            }
        }
    }

    file.close();

    return true;
}

QString PathLoader::getMapFilepath()
{
    QString * mapFilepath = new QString();
    mapFilepath->append(mpMapDir->absolutePath());
    mapFilepath->append("/");
    mapFilepath->append(mpMapFile);

    return *mapFilepath;
}

DiscretePath * PathLoader::getPath()
{
    return mpPath;
}

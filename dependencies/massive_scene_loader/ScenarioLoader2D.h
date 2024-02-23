//
// Created by yaozhuo on 2022/7/4.
//

#ifndef FREENAV_MASSIVE_SCENE_LOADER_H
#define FREENAV_MASSIVE_SCENE_LOADER_H

/*
 * scenarioLoader.h
 * hog
 *
 * Created by Renee Jansen on 5/2/2006
 *
 */


#include <vector>
#include <cstring>
#include <string>
using std::string;

static const int kNoScaling = -1;

/**
 * Experiments stored by the ScenarioLoader class.
 */
class ScenarioLoader2D;

class Experiment {
public:
    Experiment(int sx,int sy,int gx,int gy,int b, double d, string m)
            :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(kNoScaling),scaleY(kNoScaling),bucket(b),distance(d),map(m){}
    Experiment(int sx,int sy,int gx,int gy,int sizeX, int sizeY,int b, double d, string m)
            :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(sizeX),scaleY(sizeY),bucket(b),distance(d),map(m){}
    int GetStartX() const {return startx;}
    int GetStartY() const {return starty;}
    int GetGoalX() const {return goalx;}
    int GetGoalY() const {return goaly;}
    int GetBucket() const {return bucket;}
    double GetDistance() const {return distance;}
    void GetMapName(char* mymap) const {strcpy(mymap,map.c_str());}
    const char *GetMapName() const { return map.c_str(); }
    int GetXScale() const {return scaleX;}
    int GetYScale() const {return scaleY;}

private:
    friend class ScenarioLoader2D;
    int startx, starty, goalx, goaly;
    int scaleX;
    int scaleY;
    int bucket;
    double distance;
    string map;
};

/** A class which loads and stores scenarios from files.
 * Versions currently handled: 0.0 and 1.0 (includes scale).
 */


// for test scene from https://www.movingai.com/benchmarks/grids.html
class ScenarioLoader2D{
public:
    ScenarioLoader2D() { scenName[0] = 0; }
    ScenarioLoader2D(const char *);
    void Save(const char *);
    int GetNumExperiments(){return experiments.size();}
    const char *GetScenarioName() { return scenName; }
    Experiment GetNthExperiment(int which)
    {return experiments[which];}
    void AddExperiment(Experiment which);
private:
    char scenName[1024];
    std::vector<Experiment> experiments;
};

#endif //FREENAV_MASSIVE_SCENE_LOADER_H

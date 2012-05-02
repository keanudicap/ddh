/*
 * scenarioLoader.h
 * hog
 *
 * Created by Renee Jansen on 5/2/2006
 *
 */ 

#ifndef SCENARIOLOADER_H
#define SCENARIOLOADER_H

#include <vector>
#include <string>
#include <cstring>

using std::string;

static const int kNoScaling = -1;

/** 
 * Experiments stored by the ScenarioLoader class. 
 */

class Experiment{
public:
  Experiment(int sx,int sy,int gx,int gy,int b, double d, string m)
    :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(kNoScaling),scaleY(kNoScaling),bucket(b),distance(d),map(m),precision(4)
  {}

  Experiment(int sx,int sy,int gx,int gy,int sizeX, int sizeY,int b, double d, string m)
    :startx(sx),starty(sy),goalx(gx),goaly(gy),scaleX(sizeX),scaleY(sizeY),bucket(b),distance(d),map(m),precision(4)
  {}

  virtual ~Experiment() { }
  int getStartX(){return startx;}
  int getStartY(){return starty;}
  int getGoalX(){return goalx;}
  int getGoalY(){return goaly;}
  int getBucket(){return bucket;}
  double getDistance(){return distance;}
  void getMapName(char* mymap){strcpy(mymap,map.c_str());}
	const char *getMapName() { return map.c_str(); }
  int getXScale(){return scaleX;}
  int getYScale(){return scaleY;}
  void setPrecision(int prec) { precision = prec; }
  int getPrecision() { return precision; }

  virtual  void print(std::ostream& out);

private:
  int startx, starty, goalx, goaly;
  int scaleX;
  int scaleY;
  int bucket;
  double distance;
  string map;
  int precision;
};

/** A class which loads and stores scenarios from files.  
 * Versions currently handled: 0.0 and 1.0 (includes scale). 
 */

class ScenarioLoader{
public:
  ScenarioLoader(const char *);
  int getNumExperiments(){return experiments.size();}
	const char *getScenarioName() { return scenName; }
  Experiment getNthExperiment(int which){return experiments[which];}

private:
		char scenName[1024];
  std::vector<Experiment> experiments;
};

#endif

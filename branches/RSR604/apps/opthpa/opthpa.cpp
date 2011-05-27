/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "ClusterAStar.h"
#include "ClusterAStar.h"
#include "ClusterAStarFactory.h"
#include "common.h"
#include "opthpa.h"
#include "HPAClusterAbstraction.h"
#include "HPAClusterFactory.h"
#include "EdgeFactory.h"
#include "EmptyClusterAbstraction.h"
#include "EmptyCluster.h"
#include "EmptyClusterFactory.h"
#include "FlexibleAStar.h"
#include "HPAStar2.h"
#include "IncidentEdgesExpansionPolicy.h"
#include "clusterAbstraction.h"
#include "MacroNodeFactory.h"
#include "ManhattanHeuristic.h"
#include "OctileHeuristic.h"
#include "PerimeterSearch.h"
#include "PerimeterSearchFactory.h"
#include "RRExpansionPolicy.h"
#include "ScenarioManager.h"
#include "searchUnit.h"
#include "statCollection.h"
#include <cstdlib>
#include <sstream>
#include "Timer.h"

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 3;
ScenarioManager scenariomgr;
Experiment* nextExperiment;
int expnum=0;
bool runAStar=false;
bool scenario=false;
bool verbose = false;
bool allowDiagonals = true;
bool reducePerimeter = false;
bool bfReduction = false;
char* algName;

/**
 * This function is called each time a unitSimulation is deallocated to
 * allow any necessary stat processing beforehand
 */

void processStats(statCollection *stat)
{
	if(stat->getNumStats() == 0)
		return;
	
	processStats(stat, algName);
	stat->clearAllStats();
}


void processStats(statCollection* stat, const char* unitname)
{
	//stat->printStatsTable();
	std::ostringstream ss;
	ss << "results";
	//ss << "_csize"<<CLUSTERSIZE;
	
	statValue val;
	
	int ne, nt, ng, absne, absnt, abspm, insne, insnt, inspm;
	double st, absst, insst, pathdist;
	int expId = expnum;
	if(strcmp(unitname, "HPAStar2") == 0 && !getDisableGUI())
		expId--;

	ss << "_"<<unitname;
	//std::cout << "exp: "<<expId<<" ";
	//std::cout << unitname;
	FILE *f = fopen(ss.str().c_str(), "a+");

	ne = nt = ng = absne = absnt = abspm = insne = insnt = inspm = 0;
	st = absst = insst = pathdist = 0;
	bool exists;

	fprintf(f, "%i,\t", expId);
	fprintf(f, "%s,\t", unitname);

	exists = stat->lookupStat("nodesExpanded", unitname, val);
	assert(exists);
	ne = val.lval;
	fprintf(f, "%i,\t", ne);

	
	exists = stat->lookupStat("nodesTouched", unitname, val);
	assert(exists);
	nt = val.lval;
	fprintf(f, "%i,\t", nt);

	exists = stat->lookupStat("nodesGenerated", unitname, val);
	assert(exists);
	ng = val.lval;	
	fprintf(f, "%i,\t", ng);

	exists = stat->lookupStat("searchTime", unitname, val);
	assert(exists);
	st = val.fval;
	fprintf(f, "%.8f,\t", st);
	
	if(strcmp(unitname, "HPAStar2") == 0)
	{
		exists = stat->lookupStat("insNodesExpanded", unitname, val);
		assert(exists);
		insne = val.lval;
		fprintf(f, "%i,\t", insne);

		exists = stat->lookupStat("insNodesTouched", unitname, val);
		assert(exists);
		insnt = val.lval;
		fprintf(f, "%i,\t", insnt);
		
		exists = stat->lookupStat("insNodesGenerated", unitname, val);
		assert(exists);
		inspm = val.lval;
		fprintf(f, "%i,\t", inspm);
				
		exists = stat->lookupStat("insSearchTime", unitname, val);
		assert(exists);
		insst = val.fval;
		fprintf(f, "%.8f,\t", insst);

	}
	
	if(getDisableGUI())
	{
		exists = stat->lookupStat("distanceMoved", unitname, val);
		assert(exists);
		pathdist = val.fval;
	}
	fprintf(f, "%.3f,\t", pathdist);	
	fprintf(f, "%s\n", gDefaultMap);	

	fflush(f);
	fclose(f);
}

/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void createSimulation(unitSimulation * &unitSim)
{
	std::cout << "createSimulation.";
	std::cout << " nogui="<<(bfReduction?"true":"false");
	std::cout << " cardinal="<<(!allowDiagonals?"true":"false");
	std::cout << " pr="<<(reducePerimeter?"true":"false");
	std::cout << " bfr="<<(bfReduction?"true":"false");
	std::cout << std::endl;

	algName = (char*)"";
	Map* map = new Map(gDefaultMap);
	if(strcmp(gDefaultMap, map->getMapName()) != 0)
	{
		delete map;
		exit(1);
	}

	//map->scale(100, 100);
	int scalex = map->getMapWidth();
	int scaley = map->getMapHeight();
	if(scenariomgr.getNumExperiments() > 0)
	{
		scalex = scenariomgr.getNthExperiment(0)->getXScale();
		scaley = scenariomgr.getNthExperiment(0)->getYScale();
		if(scalex > 1 && scaley > 1) // stupid v3 scenario files 
			map->scale(scalex, scaley);
	}
	std::cout << " width: "<<map->getMapWidth()<<" height: ";
	std::cout <<map->getMapHeight()<<std::endl;

	FILE* fTimes = fopen("preproc_rsr", "a");

	Timer t;
	t.startTimer();

	EmptyClusterAbstraction* ecmap = new EmptyClusterAbstraction(
			map, new EmptyClusterFactory(), 
			new MacroNodeFactory(), new EdgeFactory(), allowDiagonals, reducePerimeter, bfReduction);

	

	ecmap->setVerbose(verbose);
	ecmap->buildClusters2();
	ecmap->buildEntrances();
	//ecmap->setDrawClusters(true);
	graph* absg = ecmap->getAbstractGraph(1);
	graph* g = ecmap->getAbstractGraph(0);

	fprintf(fTimes, "%s\t%d\t%d\t%f\n", gDefaultMap, g->getNumNodes(), g->getNumEdges(), t.endTimer());
	fflush(fTimes);
	fclose(fTimes);
	//delete ecmap;
	//exit(0);
	
	std::ostringstream ss;
	ss << "results_graphsize";
	//ss << "_csize"<<CLUSTERSIZE;
	
	FILE *f = fopen(ss.str().c_str(), "a+");

	fprintf(f, "%i,\t%i,\t", g->getNumNodes(), g->getNumEdges());
	fprintf(f, "%i,\t%i,\t", absg->getNumNodes(), ecmap->getNumAbsEdges()); 
	fprintf(f, "%f,\t%f,\t", ecmap->getAverageClusterSize(), ecmap->getAverageNodesPruned());
	fprintf(f, "%s\n", map->getMapName());
	fflush(f);
	fclose(f);
	
	std::cout << "map: "<<gDefaultMap;
	std::cout << "\noriginal map: nodes: "<<g->getNumNodes()<<" edges: "<<g->getNumEdges();
	std::cout << " absnodes: "<<absg->getNumNodes()<<" absedges: "<<ecmap->getNumAbsEdges();
	std::cout << "\navg_room_size: "<<ecmap->getAverageClusterSize();
	std::cout <<" avg_nodes_pruned: "<<ecmap->getAverageNodesPruned()<<std::endl;
	 
	// debugging
/*	edge_iterator ei = absg->getEdgeIter();
	edge* e = absg->edgeIterNext(ei);
	while(e)
	{
		node* f = absg->getNode(e->getFrom());
		node* t = absg->getNode(e->getTo());
		std::cout << "\n edge connects "<<f->getLabelL(kFirstData)<<","<<f->getLabelL(kFirstData+1)<< " and "<<t->getLabelL(kFirstData)<<","<<t->getLabelL(kFirstData+1);
		std::cout <<"(weight: "<<e->getWeight()<<" caps: "<<e->getCapability() << " clearance: "<<e->getClearance(e->getCapability())<<")";
		e = absg->edgeIterNext(ei);
	}
*/
	
	if(!getDisableGUI())
	{
		unitSim = new unitSimulation(ecmap);	
		unitSim->setCanCrossDiagonally(true);
		if(scenario)
		{
			unitSim->setNextExperimentPtr(&runNextExperiment);
			runNextExperiment(unitSim);
		}
	}
	else
	{
		gogoGadgetNOGUIScenario(ecmap);
	}
}

void gogoGadgetNOGUIScenario(HPAClusterAbstraction* ecmap)
{
//	std::cout << "\n diagonals? "<<ecmap->getAllowDiagonals()<<std::endl;
	FlexibleAStar* astar;
	astar = new FlexibleAStar(newExpansionPolicy(ecmap), newHeuristic());
	astar->verbose = verbose;

	HPAStar2* hpastar;
	hpastar = new HPAStar2(newExpansionPolicy(ecmap), newHeuristic(), false, false);
	hpastar->verbose = verbose;

//	IClusterAStarFactory* caf;
//	if(bfReduction)
//		caf = new PerimeterSearchFactory();
//	else
//	 	caf = new ClusterAStarFactory();

	statCollection stats;
	double optlen=0;
	double pslen=0;
	
	for(int i=0; i< scenariomgr.getNumExperiments(); i++)
	{
		expnum = i;
		nextExperiment = (Experiment*)scenariomgr.getNthExperiment(i);
		node* from = ecmap->getNodeFromMap(nextExperiment->getStartX(), nextExperiment->getStartY());
		node* to = ecmap->getNodeFromMap(nextExperiment->getGoalX(), nextExperiment->getGoalY());
		
		//std::cout << "ASTAR!!"<<std::endl;
		path* p = 0;
		double distanceTravelled = 0;
		if(verbose)
		{
			algName = (char*)astar->getName();
			astar->verbose = verbose;
			p = astar->getPath(ecmap, from, to);
			distanceTravelled = ecmap->distance(p);
			optlen = distanceTravelled;

			stats.addStat("distanceMoved", algName, distanceTravelled);
			astar->logFinalStats(&stats);
			processStats(&stats, astar->getName());
			stats.clearAllStats();
			delete p;
			//std::cout << "FINASTAR!!"<<std::endl;
		}
		
		//std::cout << "HPA*"<<std::endl;
		algName = (char*)hpastar->getName();
		hpastar->verbose = verbose;
		//Timer t;
		//t.startTimer();
		p = hpastar->getPath(ecmap, from, to);
		//hpastar->searchTime = t.endTimer();
		distanceTravelled = ecmap->distance(p);
		pslen = distanceTravelled;
		stats.addStat("distanceMoved", algName, distanceTravelled);
		hpastar->logFinalStats(&stats);
		processStats(&stats);
		stats.clearAllStats();
		delete p;
		//std::cout << "Fin HPA*"<<std::endl;

		if(optlen != pslen && verbose)
		{
			astar->verbose = true;
			hpastar->verbose = true;
			path* p = astar->getPath(ecmap, from, to);
			double tmp = ecmap->distance(p);
			delete p;
			p = 0;
			p = hpastar->getPath(ecmap, from, to);
			double tmp2 = ecmap->distance(p);
			delete p;

			std::cout << "\n opt: "<<tmp;
			std::cout << " pslen: "<<tmp2<<std::endl;
			std::cout << " previously, opt: "<<optlen<<" pslen: "<<pslen<<std::endl;
			break;
		}
	}
	
	delete hpastar;
	delete astar;
	delete ecmap;
//	sleep(20);
	exit(0);
}


/**
 * This function is called once after each [time-step and frame draw]
 * You can do any high level processing, drawing, etc in this function
 */
void frameCallback(unitSimulation *us)
{
	char tempStr[255];
	sprintf(tempStr, "Simulation time elapsed: %1.2f, Display Time: %1.2f",
					us->getSimulationTime(), us->getDisplayTime());
	submitTextToBuffer(tempStr);
	
	if ((mouseTracking) && (px1 != -1) && (px2 != -1) && (py1 != -1) && (py2 != -1))
	{
		glColor3f(1.0, 1.0, 1.0);
		GLdouble x1, y1, z1, rad;
		glBegin(GL_LINES);
		us->getMap()->getOpenGLCoord(px1, py1, x1, y1, z1, rad);
		glVertex3f(x1, y1, z1-rad);
		us->getMap()->getOpenGLCoord(px2, py2, x1, y1, z1, rad);
		glVertex3f(x1, y1, z1-rad);
		glEnd();
	}
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void initializeHandlers()
{
	installKeyboardHandler(myDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	installKeyboardHandler(myDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kNoModifier, '\t');
	installKeyboardHandler(myDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	installKeyboardHandler(myDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	installKeyboardHandler(myDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	installKeyboardHandler(myDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	installKeyboardHandler(myDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	installKeyboardHandler(myDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	installKeyboardHandler(myNewUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	installKeyboardHandler(myNewUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	installKeyboardHandler(myNewUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

	installCommandLineHandler(myCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	installCommandLineHandler(myScenarioGeneratorCLHandler, "-genscenarios", 
			"-genscenarios [.map filename] [number of scenarios]", 
			"Generates a scenario; a set of path problems on a given map");
	installCommandLineHandler(myExecuteScenarioCLHandler, "-scenario", "-scenario filename", 
			"Execute all experiments in a given .scenario file");

	installCommandLineHandler(myAllPurposeCLHandler, "-nogui", "-nogui", 
			"Run the app without a pretty interface (default = false). ");
	installCommandLineHandler(myAllPurposeCLHandler, "-v", "-v", 
			"Turn on verbose (debugging) mode (default = off)");
	installCommandLineHandler(myAllPurposeCLHandler, "-cardinal", "-cardinal", 
			"Disallow diagonal moves during search (default = false)");
	installCommandLineHandler(myAllPurposeCLHandler, "-pr", "-pr", 
			"Enable perimeter minimisation (default = false)");
	installCommandLineHandler(myAllPurposeCLHandler, "-bfr", "-bfr", 
			"Enable branching factor reduction using secondary edges (default = false)");

	installMouseClickHandler(myClickHandler);
}

int myAllPurposeCLHandler(char* argument[], int maxNumArgs)
{
	if(strcmp(argument[0], "-cardinal") == 0)
		allowDiagonals = false;
	else if(strcmp(argument[0], "-nogui") == 0)
		setDisableGUI(true);
	else if(strcmp(argument[0], "-v") == 0)
		verbose = true;
	else if(strcmp(argument[0], "-pr") == 0)
		reducePerimeter = true;
	else if(strcmp(argument[0], "-bfr") == 0)
		bfReduction = true;
	else
	{
		std::cout << "program invoked with incorrect parameters"<<std::endl;
		printCommandLineArguments();
		exit(-1);
	}

	return 1;
}

int myCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

int myScenarioGeneratorCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs < 2)
	{
		std::cout << "-genscenarios invoked with insufficient parameters"<<std::endl;
		printCommandLineArguments();
		exit(-1);
	}

	std::string map(argument[1]);
	std::string genscen(argument[0]);
	std::cout << "call: "<<genscen<<" "<<map<<" "<<argument[2]<<std::endl;
		
	ScenarioManager scenariomgr;
	int numScenarios = atoi(argument[2]);

	EmptyClusterAbstraction ecmap(new Map(map.c_str()), new EmptyClusterFactory(),
			new MacroNodeFactory(), new EdgeFactory(), allowDiagonals, reducePerimeter, bfReduction);
	
	scenariomgr.generateExperiments(&ecmap, numScenarios);
	std::cout << "generated: "<<scenariomgr.getNumExperiments()<< " experiments"<<std::endl;

	string outfile = map + ".scenario"; 
	scenariomgr.writeScenarioFile(outfile.c_str());
	std::cout << "writing scenario file: "<<outfile<<std::endl;
	exit(-1);
}

int myExecuteScenarioCLHandler(char *argument[], int maxNumArgs)
{	
	if(maxNumArgs < 1)
		return 0;
	
	//std::cout << "\n -scenario call: "<<argument[1] << std::endl;
	std::string infile(argument[1]);
	scenariomgr.loadScenarioFile(infile.c_str());	
	strncpy(gDefaultMap, scenariomgr.getNthExperiment(0)->getMapName(), 1024);
	
	scenario=true;
	return 2;
}

void myDisplayHandler(unitSimulation *unitSim, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t': unitSim->cyclemapAbstractionDisplay(); break;
		case 'p': unitSim->setSimulationPaused(!unitSim->getSimulationPaused()); break;
		case 'o':
			if (unitSim->getSimulationPaused())
			{
				unitSim->setSimulationPaused(false);
				unitSim->advanceTime(.1);
				unitSim->setSimulationPaused(true);
			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
		case '{': unitSim->setSimulationPaused(true); unitSim->offsetDisplayTime(-0.5); break;
		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			if (unitSim->getMapAbstractionDisplay())
				unitSim->getMapAbstractionDisplay()->toggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void myNewUnitKeyHandler(unitSimulation *unitSim, tKeyboardModifier mod, char)
{
	HPAClusterAbstraction* aMap = dynamic_cast<HPAClusterAbstraction*>(
			unitSim->getMapAbstraction());
	aMap->clearColours();
	for(int i=0; i<unitSim->getNumUnits(); i++)
	{
		unit* lastunit = dynamic_cast<searchUnit*>(unitSim->getUnit(0));
		if(lastunit)
		{
			lastunit->logFinalStats(unitSim->getStats());
			processStats(unitSim->getStats());
		}
	}
	unitSim->clearAllUnits();

	int x1, y1, x2, y2;
	unit *u, *targ;
	searchAlgorithm* astar;
	
	//std::cout << "\n absnodes: "<<unitSim->getMapAbstraction()->getAbstractGraph(1)->getNumNodes()<< " edges: "<<unitSim->getMapAbstraction()->getAbstractGraph(1)->getNumEdges();
	//std::cout << " cachesize: "<<((EmptyClusterAbstraction*)unitSim->getMapAbstraction())->getPathCacheSize();

	unitSim->getRandomLocation(x1, y1);
	unitSim->getRandomLocation(x2, y2);
	
//	x2 = 1; y2 = 5;
//	x1=16; y1=8;
	std::cout << "\n deploying unit to "<<x2<<","<<y2<<" with target at "<<x1<<","<<y1;
	
	unitSim->addUnit(targ = new unit(x1, y1));

	//IClusterAStarFactory* caf;
	//if(bfReduction)
	//	caf = new PerimeterSearchFactory();
	//else
	// 	caf = new ClusterAStarFactory();

	switch (mod)
	{
		case kShiftDown: 
		{
			astar = new HPAStar2(newExpansionPolicy(aMap), newHeuristic());
			astar->verbose = verbose;
			unitSim->addUnit(u=new searchUnit(x2, y2, targ, astar)); 
			u->setColor(0.3,0.7,0.3);
			targ->setColor(0.3,0.7,0.3);
			break;
		}
		default:
		{
			astar = new FlexibleAStar(newExpansionPolicy(aMap), newHeuristic());	
			astar->verbose = verbose;
			unitSim->addUnit(u=new searchUnit(x2, y2, targ, astar)); 
			u->setColor(1,1,0);
			targ->setColor(1,1,0);
			break;
		}
	}
	algName = (char*)u->getName();
	u->setSpeed(0.12);
//	u->setSpeed(0.000001);
	//unitSim->setmapAbstractionDisplay(1);
}

bool myClickHandler(unitSimulation *unitSim, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				std::cout << "kMouseDown!\n";
				break;
			case kMouseDrag:
				std::cout << "kMouseDrag!\n";
				break;
			case kMouseUp:
			{
				std::cout << "kMouseUp!\n";
			}
			break;
		}
		return true;
	}
	return false;
}

void runNextExperiment(unitSimulation *unitSim)
{	
	if(expnum == scenariomgr.getNumExperiments()) 
	{
		processStats(unitSim->getStats());
		assert(graph_object::gobjCount == 0);
		exit(0);
	}

	HPAClusterAbstraction* aMap = dynamic_cast<HPAClusterAbstraction*>(
			unitSim->getMapAbstraction());
	aMap->clearColours();
	

	Experiment* nextExperiment = dynamic_cast<Experiment*>(scenariomgr.getNthExperiment(expnum));
	
	searchUnit* nextUnit;
	unit* nextTarget = new unit(nextExperiment->getGoalX(), nextExperiment->getGoalY());

	if(runAStar)
	{
		HPAStar2* hpastar = new HPAStar2(newExpansionPolicy(aMap), newHeuristic());
		hpastar->verbose = verbose;
		algName = (char*)hpastar->getName();
		nextUnit = new searchUnit(nextExperiment->getStartX(), nextExperiment->getStartY(), nextTarget, hpastar); 
		nextUnit->setColor(0.1,0.1,0.5);
		nextTarget->setColor(0.1,0.1,0.5);
		expnum++;
		runAStar=false;
	}
	else
	{
		FlexibleAStar* astar = new FlexibleAStar(newExpansionPolicy(aMap), newHeuristic());	
		astar->verbose = verbose;
		algName = (char*)astar->getName();
		nextUnit = new searchUnit(nextExperiment->getStartX(), nextExperiment->getStartY(), nextTarget, astar); 
		nextUnit->setColor(0.5,0.1,0.1);
		nextTarget->setColor(0.5,0.1,0.1);
		runAStar=true;
	}
	std::cout << "running "<<algName<<" experiment"<<std::endl;
	nextUnit->setSpeed(0.05);

	unitSim->clearAllUnits();
	unitSim->addUnit(nextTarget);
	unitSim->addUnit(nextUnit);
}

void runSimulationNoGUI()
{
	std::cout << "\nok, no gui";
}

ExpansionPolicy* newExpansionPolicy(HPAClusterAbstraction* map)
{
	ExpansionPolicy* policy;
	if(bfReduction)
		policy = new RRExpansionPolicy(map);
	else
		policy = new IncidentEdgesExpansionPolicy(map);
	return policy;
}

Heuristic* newHeuristic()
{
	Heuristic* h;
//	if(allowDiagonals)
		h = new OctileHeuristic();
//	else
//		h = new ManhattanHeuristic();

	return h;
}

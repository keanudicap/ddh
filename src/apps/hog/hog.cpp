/*
 * @author: nathanst 
 * @author: dharabor
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

#include "aStar3.h"
#include "ClusterAStar.h"
#include "ClusterAStar.h"
#include "ClusterAStarFactory.h"
#include "ClusterNodeFactory.h"
#include "common.h"
#include "hog.h"
#include "DebugUtility.h"
#include "DefaultInsertionPolicy.h"
#include "DefaultRefinementPolicy.h"
#include "EdgeFactory.h"
#include "EmptyCluster.h"
#include "EmptyClusterInsertionPolicy.h"
#include "EmptyClusterAbstraction.h"
#include "FlexibleAStar.h"
#include "fpUtil.h"
#include "HierarchicalSearch.h"
#include "HierarchicalSearchRSR.h"
#include "HPAClusterAbstraction.h"
#include "HPAClusterFactory.h"
#include "IncidentEdgesExpansionPolicy.h"
#include "JPAExpansionPolicy.h"
#include "JumpPointAbstraction.h"
#include "JumpPointsExpansionPolicy.h"
#include "mapFlatAbstraction.h"
#include "MacroNodeFactory.h"
#include "ManhattanHeuristic.h"
#include "NodeFactory.h"
#include "NoInsertionPolicy.h"
#include "OctileDistanceRefinementPolicy.h"
#include "OctileHeuristic.h"
#include "EmptyClusterFactory.h"
#include "RRExpansionPolicy.h"
#include "ScenarioManager.h"
#include "searchUnit.h"
#include "statCollection.h"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

bool mouseTracking;
int px1, py1, px2, py2;
int absDispType = 3;
ScenarioManager scenariomgr;
Experiment* nextExperiment;
int expnum=0;
bool scenario=false;
bool verbose = false;
bool allowDiagonals = true;
bool reducePerimeter = false;
bool bfReduction = false;
bool checkOptimality = false;
char* algName;
HOG::AbstractionType absType = HOG::FLAT;

/**
 * This function is called each time a unitSimulation is deallocated to
 * allow any necessary stat processing beforehand
 */

void 
processStats(statCollection *stat)
{
	if(stat->getNumStats() == 0)
		return;
	
	processStats(stat, algName);
	stat->clearAllStats();
}


void 
processStats(statCollection* stat, const char* unitname)
{
	//stat->printStatsTable();
	std::ostringstream ss;
	ss << "results";
	//ss << "_csize"<<CLUSTERSIZE;
	
	statValue val;
	
	int ne, nt, ng, absne, absnt, abspm, insne, insnt, inspm;
	double st, absst, insst, pathdist;

	ss << "_"<<unitname;
	//std::cout << "exp: "<<expId<<" ";
	//std::cout << unitname;
	FILE *f = fopen(ss.str().c_str(), "a+");

	ne = nt = ng = absne = absnt = abspm = insne = insnt = inspm = 0;
	st = absst = insst = pathdist = 0;
	bool exists;

	fprintf(f, "%i,\t", expnum);
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
void 
createSimulation(unitSimulation * &unitSim)
{
	std::cout << "createSimulation.";
	std::cout << " nogui="<<(getDisableGUI()?"true":"false");
	std::cout << " cardinal="<<(!allowDiagonals?"true":"false");
	std::cout << " pr="<<(reducePerimeter?"true":"false");
	std::cout << " bfr="<<(bfReduction?"true":"false");
	std::cout << " abs=";
	switch(absType)
	{
		case HOG::HPA:
			std::cout << "HPA";
			break;
		case HOG::ERR:
			std::cout << "ERR";
			break;
		case HOG::FLAT:
			std::cout << "FLAT";
			break;
		case HOG::FLATJUMP:
			std::cout << "FLATJUMP";
			break;
		case HOG::JPA:
			std::cout << "JPA";
			break;
		default:
			std::cout << "Unknown?? Fix me!!";
			break;
	}
	std::cout << std::endl;

	algName = (char*)"";
	Map* map = new Map(gDefaultMap);
	std::cout << "map: "<<gDefaultMap;

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

	mapAbstraction* aMap = 0;
	switch(absType)
	{
		case HOG::HPA:
		{
			aMap = new HPAClusterAbstraction(map, new HPAClusterFactory(), 
					new ClusterNodeFactory(), new EdgeFactory());
			dynamic_cast<HPAClusterAbstraction*>(aMap)->buildClusters();
			dynamic_cast<HPAClusterAbstraction*>(aMap)->buildEntrances();
			dynamic_cast<HPAClusterAbstraction*>(aMap)->clearColours();
			break;
		}
		case HOG::ERR:
		{
			aMap = new EmptyClusterAbstraction(map, 
					new EmptyClusterFactory(), new MacroNodeFactory(),
				   	new EdgeFactory(), allowDiagonals, reducePerimeter, 
					bfReduction);

			dynamic_cast<EmptyClusterAbstraction*>(aMap)->setVerbose(verbose);
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->buildClusters();
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->buildEntrances();
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->clearColours();

			break;
		}
		case HOG::JPA:
		{
			aMap = new JumpPointAbstraction(map, new NodeFactory(), 
					new EdgeFactory(), verbose);
			break;
		}
		default:
			aMap = new mapFlatAbstraction(map);
			break;
	}


	if(verbose)
	{
		Heuristic* h = newHeuristic();
		DebugUtility debug(aMap, h);
		debug.printGraph(aMap->getAbstractGraph(1));
		delete h;
	}

	graph* g = aMap->getAbstractGraph(0);
	if(aMap->getNumAbstractGraphs() > 1)
	{
		graph* absg = aMap->getAbstractGraph(1);
		
		std::ostringstream ss;
		ss << "results_graphsize";

		int numAbsEdges = absg->getNumEdges();
		double avgNodesPruned = 0;
		double avgClusterSize = 0;
		if(dynamic_cast<EmptyClusterAbstraction*>(aMap))
		{
			EmptyClusterAbstraction* ecmap = 
				dynamic_cast<EmptyClusterAbstraction*>(aMap);
			numAbsEdges = ecmap->getNumAbsEdges();
			avgNodesPruned = ecmap->getAverageNodesPruned();
			avgClusterSize = ecmap->getAverageClusterSize();
		}

		FILE *f = fopen(ss.str().c_str(), "a+");
		fprintf(f, "%i,\t%i,\t", g->getNumNodes(), g->getNumEdges());
		fprintf(f, "%i,\t%i,\t", absg->getNumNodes(), numAbsEdges); 
		fprintf(f, "%f,\t%f,\t", avgClusterSize, avgNodesPruned); 
		fprintf(f, "%s\n", map->getMapName());
		fflush(f);
		fclose(f);
		
		std::cout << "map: "<<gDefaultMap << std::endl;
		std::cout << "original map: nodes: "<<g->getNumNodes()<<
			" edges: "<<g->getNumEdges();
		std::cout << " absnodes: "<<absg->getNumNodes()<<" absedges: "
			<<numAbsEdges;
		std::cout << "\navg_room_size: "<<avgClusterSize;
		std::cout <<" avg_nodes_pruned: "<<avgNodesPruned;
	}
	std::cout << std::endl;
	 
	if(!getDisableGUI())
	{
//		for(int i = 0; i < scenariomgr.getNumExperiments(); i++)
//		{
//			Experiment* nextExperiment = dynamic_cast<Experiment*>(
//					scenariomgr.getNthExperiment(i));
//			nextExperiment->print(std::cout);
//			std::cout << std::endl;
//		}

		unitSim = new unitSimulation(aMap);	
		unitSim->setCanCrossDiagonally(true);
		if(scenario)
			unitSim->setNextExperimentPtr(&runNextExperiment);
	}
	else
	{
		gogoGadgetNOGUIScenario(aMap);
	}
}

void 
gogoGadgetNOGUIScenario(mapAbstraction* aMap)
{
	int exitVal = 0;

	aStarOld* astar = 0;
	mapFlatAbstraction* gridmap = 0;
	if(checkOptimality)
	{
		// reference map and search alg for checking optimality
		astar = new aStarOld();
		gridmap = new mapFlatAbstraction(
				new Map(gDefaultMap));
	}

	searchAlgorithm* alg = newSearchAlgorithm(aMap, false);
	statCollection stats;
	double optlen=0;
	
	for(int i=0; i< scenariomgr.getNumExperiments(); i++)
	{
		expnum = i;
		nextExperiment = (Experiment*)scenariomgr.getNthExperiment(i);
		nextExperiment->print(std::cout);
		std::cout << std::endl;

		node* from = aMap->getNodeFromMap(nextExperiment->getStartX(), 
				nextExperiment->getStartY());
		node* to = aMap->getNodeFromMap(nextExperiment->getGoalX(), 
				nextExperiment->getGoalY());
		

		//std::cout << "HPA*"<<std::endl;
		algName = (char*)alg->getName();
		alg->verbose = verbose;
		path* p = alg->getPath(aMap, from, to);
		double distanceTravelled = aMap->distance(p);
		stats.addStat("distanceMoved", algName, distanceTravelled);
		alg->logFinalStats(&stats);
		processStats(&stats);
		stats.clearAllStats();
		delete p;
		//std::cout << "Fin HPA*"<<std::endl;

		if(checkOptimality)
		{
			// run A* to check the optimal path length
			node* s = gridmap->getNodeFromMap(nextExperiment->getStartX(),
					nextExperiment->getStartY());
			node* g = gridmap->getNodeFromMap(nextExperiment->getGoalX(),
					nextExperiment->getGoalY());

			p = astar->getPath(gridmap, s, g);
			optlen = aMap->distance(p);
			delete p;

			if(!fequal(optlen, distanceTravelled))
			{
				std::cout << "optimality check failed!";
				std::cout << "\noptimal path length: "<<optlen<<" computed length: ";
				std::cout << distanceTravelled<<std::endl;
				if(verbose)
				{
					std::cout << "Running A*: \n";
					astar->verbose = true;
					alg->verbose = true;
					path* p = astar->getPath(aMap, from, to);
					double tmp = aMap->distance(p);
					delete p;
					p = 0;
					std::cout << "\nRunning "<<alg->getName()<<": \n";
					p = alg->getPath(aMap, from, to);
					double tmp2 = aMap->distance(p);
					delete p;

					std::cout << "\n optimal: "<<tmp;
					std::cout << " computed: "<<tmp2<<std::endl;
					exitVal = 1;
				}
				break;
			}
		}
	}
	
	delete alg;
	delete aMap;

	if(checkOptimality)
	{
		delete astar;
		delete gridmap;
	}

	exit(exitVal);
}


/**
 * This function is called once after each [time-step and frame draw]
 * You can do any high level processing, drawing, etc in this function
 */
void 
frameCallback(unitSimulation *us)
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
void 
initializeHandlers()
{
	installKeyboardHandler(myDisplayHandler, "Toggle Abstraction", 
			"Toggle display of the ith level of the abstraction", 
			kAnyModifier, '0', '9');

	installKeyboardHandler(myDisplayHandler, "Cycle Abs. Display", 
			"Cycle which group abstraction is drawn", kNoModifier, '\t');

	installKeyboardHandler(myDisplayHandler, "Pause Simulation", 
			"Pause simulation execution.", kNoModifier, 'p');

	installKeyboardHandler(myDisplayHandler, "Step Simulation", 
			"If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');

	installKeyboardHandler(myDisplayHandler, "Step History", 
			"If the simulation is paused, step forward .1 sec in history",
			kAnyModifier, '}');

	installKeyboardHandler(myDisplayHandler, "Step History", 
			"If the simulation is paused, step back .1 sec in history", 
			kAnyModifier, '{');

	installKeyboardHandler(myDisplayHandler, "Step Abs Type", 
			"Increase abstraction type", kAnyModifier, ']');

	installKeyboardHandler(myDisplayHandler, "Step Abs Type", 
			"Decrease abstraction type", kAnyModifier, '[');

	installKeyboardHandler(myNewUnitKeyHandler, "Add A* Unit", 
			"Deploys a simple A* unit (non-abstract search)", kNoModifier, 'a');

	installKeyboardHandler(myNewUnitKeyHandler, "Add fast A* Unit", 
			"Deploys an A* unit performing abstract search", kShiftDown, 'a');

	installCommandLineHandler(myCLHandler, "-map", "-map filename", 
			"Selects the default map to be loaded.");

	installCommandLineHandler(myScenarioGeneratorCLHandler, "-genscenarios", 
			"-genscenarios [.map filename] [number of scenarios]", 
			"Generates a scenario; a set of path problems on a given map");

	installCommandLineHandler(myExecuteScenarioCLHandler, "-scenario", 
			"-scenario filename", 
			"Execute all experiments in a given .scenario file");

	installCommandLineHandler(myAllPurposeCLHandler, "-nogui", "-nogui", 
			"Run the app without a pretty interface "
			"(default = false). ");

	installCommandLineHandler(myAllPurposeCLHandler, "-v", "-v", 
			"Turn on verbose (debugging) mode "
			"(default = off)");

	installCommandLineHandler(myAllPurposeCLHandler, "-cardinal", "-cardinal", 
			"Disallow diagonal moves during search "
			"(default = false)");

	installCommandLineHandler(myAllPurposeCLHandler, "-checkopt", "-checkopt", 
			"Verify each experiment ran is solved optimally."
			"(default = false)");

	installCommandLineHandler(myAllPurposeCLHandler, "-abs", 
			"-abs [flat | flatjump | hpa | err | err_pr | err_bfr | err_pr_bfr]", 
			"Abstraction Type:\n"
			"\tflat = no abstraction (default)\n"
			"\tflatjump = like flat but use jump points to speed search\n"
			"\thpa = hpa cluster abstraction (cluster size = 10x10)\n"
			"\terr = empty rectangular rooms abstraction\n"
			"\terr_pr = err with perimeter reduction\n"
			"\terr_bfr = err with branching factor optimisations\n"
			"\terr_pr_bfr = err with both perimeter reduction and branching "
			"factor optimisations\n");

	installMouseClickHandler(myClickHandler);
}

int 
myAllPurposeCLHandler(char* argument[], int maxNumArgs)
{
	int argsParsed = 0;
	if(strcmp(argument[0], "-cardinal") == 0)
	{
		allowDiagonals = false;
		argsParsed++;
	}
	else if(strcmp(argument[0], "-nogui") == 0)
	{
		setDisableGUI(true);
		argsParsed++;
	}
	else if(strcmp(argument[0], "-v") == 0)
	{
		verbose = true;
		argsParsed++;
	}
	else if(strcmp(argument[0], "-checkopt") == 0)
	{
		checkOptimality = true;
		argsParsed++;
	}
	else if(strcmp(argument[0], "-abs") == 0)
	{
		argsParsed++;
		if(strcmp(argument[1], "hpa") == 0)
		{
			absType = HOG::HPA; 
			argsParsed++;
		}
		else if(strcmp(argument[1], "err") == 0)
		{
			argsParsed++;
			absType = HOG::ERR; 
		}
		else if(strcmp(argument[1], "err_pr") == 0)
		{
			argsParsed++;
			absType = HOG::ERR; 
			reducePerimeter = true;
		}
		else if(strcmp(argument[1], "err_bfr") == 0)
		{
			argsParsed++;
			absType = HOG::ERR; 
			bfReduction = true;
		}
		else if(strcmp(argument[1], "err_pr_bfr") == 0)
		{
			argsParsed++;
			absType = HOG::ERR; 
			bfReduction = true;
			reducePerimeter = true;
		}
		else if(strcmp(argument[1], "flat") == 0)
		{
			argsParsed++;
			absType = HOG::FLAT;
		}
		else if(strcmp(argument[1], "flatjump") == 0)
		{
			argsParsed++;
			absType = HOG::FLATJUMP;
		}
		else if(strcmp(argument[1], "jpa") == 0)
		{
			argsParsed++;
			absType = HOG::JPA;
		}
		else
		{
			std::cout << argument[1] << ": invalid abstraction type.\n";
			printCommandLineArguments();
			exit(1);
		}
	}
	else
	{
		std::cout << argument[0] << ": invalid parameter. \n";
		printCommandLineArguments();
		exit(1);
	}

	return argsParsed;
}

int 
myCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	FILE* fMap = fopen(argument[1], "r");
	if(!fMap)
	{
		strncpy(gDefaultMap, getHome(), 1024);
		strcat(gDefaultMap, argument[1]);
		fMap = fopen(gDefaultMap, "r");
		if(!fMap)
		{
			std::cerr << "Err: could not read map file "<<gDefaultMap<<std::endl;
			exit(1);
		}
		else
			fclose(fMap);
	}
	else
	{
		fclose(fMap);
		strncpy(gDefaultMap, argument[1], 1024);
	}

	return 2;
}

int 
myScenarioGeneratorCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs < 2)
	{
		std::cout << "-genscenarios invoked with insufficient parameters\n";
		printCommandLineArguments();
		exit(1);
	}

	std::string map(argument[1]);
	std::string genscen(argument[0]);
	std::cout << "call: "<<genscen<<" "<<map<<" "<<argument[2]<<std::endl;
		
	ScenarioManager scenariomgr;
	int numScenarios = atoi(argument[2]);

	EmptyClusterAbstraction aMap(new Map(map.c_str()), 
			new EmptyClusterFactory(), new MacroNodeFactory(), 
			new EdgeFactory(), allowDiagonals, reducePerimeter, bfReduction);
	
	scenariomgr.generateExperiments(&aMap, numScenarios);
	std::cout << "generated: "<<scenariomgr.getNumExperiments()<< 
		" experiments"<<std::endl;

	string outfile = map + ".scenario"; 
	scenariomgr.writeScenarioFile(outfile.c_str());
	std::cout << "writing scenario file: "<<outfile<<std::endl;
	exit(1);
}

int 
myExecuteScenarioCLHandler(char *argument[], int maxNumArgs)
{	
	if(maxNumArgs < 1)
		return 0;
	
	//std::cout << "\n -scenario call: "<<argument[1] << std::endl;
	std::string infile(argument[1]);
	try
	{
		scenariomgr.loadScenarioFile(infile.c_str());	
	}
	catch(std::invalid_argument& e)
	{
		std::cerr << e.what() <<std::endl;
		exit(1);
	}

	FILE* fMap = fopen(scenariomgr.getNthExperiment(0)->getMapName(), "r");
	if(!fMap)
	{
		strncpy(gDefaultMap, getHome(), 1024);
		char* hoghome = getHome();
		for(int i=0; hoghome[i] != '\0'; i++)
		{
			if(hoghome[i] != '/' && 
					hoghome[i+1] == '\0')
			{
				strcat(gDefaultMap, "/");
				break;
			}
		}
		strcat(gDefaultMap, scenariomgr.getNthExperiment(0)->getMapName());
		fMap = fopen(gDefaultMap, "r");
		if(!fMap)
		{
			std::cerr << "Err: could not read map file "<<gDefaultMap<<std::endl;
			exit(1);
		}
	}
	else
	{
		fclose(fMap);
		strncpy(gDefaultMap, scenariomgr.getNthExperiment(0)->getMapName(), 1024);
	}
	
	scenario=true;
	return 2;
}

void 
myDisplayHandler(unitSimulation *unitSim, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t': unitSim->cyclemapAbstractionDisplay(); break;

		case 'p': 
		   unitSim->setSimulationPaused(!unitSim->getSimulationPaused()); 
		   break;

		case 'o':
			if (unitSim->getSimulationPaused())
			{
				unitSim->setSimulationPaused(false);
				unitSim->advanceTime(.1);
				unitSim->setSimulationPaused(true);
			}
			break;

		case ']': absDispType = (absDispType+1)%3; break;

		case '[': absDispType = (absDispType+4)%3; break;

		case '{': 
			unitSim->setSimulationPaused(true); 
			unitSim->offsetDisplayTime(-0.5); 
			break;

		case '}': 
			unitSim->offsetDisplayTime(0.5); 
			break;

		default:
			if (unitSim->getMapAbstractionDisplay())
				unitSim->getMapAbstractionDisplay()->toggleDrawAbstraction(
						((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void 
myNewUnitKeyHandler(unitSimulation *unitSim, tKeyboardModifier mod, char)
{
	mapAbstraction* aMap = unitSim->getMapAbstraction();
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
	
	unitSim->getRandomLocation(x1, y1);
	unitSim->getRandomLocation(x2, y2);
	
	std::cout << "\n deploying unit to "<<x2<<","<<y2<<
		" with target at "<<x1<<","<<y1;
	
	unitSim->addUnit(targ = new unit(x1, y1));

	switch (mod)
	{
		case kShiftDown: 
		{
			astar = newSearchAlgorithm(aMap); 
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
}

bool 
myClickHandler(unitSimulation *unitSim, int, int, point3d loc, 
		tButtonType button, tMouseEventType mType)
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

void 
runNextExperiment(unitSimulation *unitSim)
{	
	if(expnum == scenariomgr.getNumExperiments()) 
	{
		processStats(unitSim->getStats());
		// TODO: fix this assert
		//assert(graph_object::gobjCount == 0);
		exit(0);
	}

	mapAbstraction* aMap = unitSim->getMapAbstraction();
	aMap->clearColours();

	Experiment* nextExperiment = dynamic_cast<Experiment*>(
			scenariomgr.getNthExperiment(expnum));
	
	searchUnit* nextUnit;
	unit* nextTarget = new unit(nextExperiment->getGoalX(), 
			nextExperiment->getGoalY());

	searchAlgorithm* alg = newSearchAlgorithm(aMap, true); 
	alg->verbose = verbose;
	algName = (char*)alg->getName();
	nextUnit = new searchUnit(nextExperiment->getStartX(), 
			nextExperiment->getStartY(), nextTarget, alg); 
	nextUnit->setColor(0.1,0.1,0.5);
	nextTarget->setColor(0.1,0.1,0.5);

	nextUnit->setSpeed(0.01);
	unitSim->clearAllUnits();
	unitSim->addUnit(nextTarget);
	unitSim->addUnit(nextUnit);
	std::cout << alg->getName() << ": ";
	std::cout << "exp "<<expnum<<" ";
	nextExperiment->print(std::cout);
	std::cout << std::endl;
	expnum++;
}

ExpansionPolicy* 
newExpansionPolicy(mapAbstraction* map)
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
	if(allowDiagonals)
		h = new OctileHeuristic();
	else
		h = new ManhattanHeuristic();

	return h;
}

searchAlgorithm*
newSearchAlgorithm(mapAbstraction* aMap, bool refineAbsPath)
{
	searchAlgorithm* alg = 0;
	switch(absType)
	{
		case HOG::HPA:
		{
			GenericClusterAbstraction* map = 
				dynamic_cast<GenericClusterAbstraction*>(aMap);
			alg = new HierarchicalSearch(new DefaultInsertionPolicy(map),
					new FlexibleAStar(newExpansionPolicy(map), 
						newHeuristic()),
					new DefaultRefinementPolicy(map));
			((HierarchicalSearch*)alg)->setName("HPA");
			alg->verbose = verbose;
			break;
		}
		case HOG::ERR:
		{
			EmptyClusterAbstraction* map = 
				dynamic_cast<EmptyClusterAbstraction*>(aMap);
			alg = new HierarchicalSearchRSR(
					map, new FlexibleAStar(
						newExpansionPolicy(map),
						newHeuristic()));
			((HierarchicalSearch*)alg)->setName("RSR");
			alg->verbose = verbose;
			break;
		}

		case HOG::FLATJUMP:
		{
			alg = new HierarchicalSearch(new NoInsertionPolicy(),
						new FlexibleAStar(
							new JumpPointsExpansionPolicy(), 
								newHeuristic()),
						new OctileDistanceRefinementPolicy(aMap));
			((HierarchicalSearch*)alg)->setName("JPS");
			alg->verbose = verbose;
			break;
		}
		case HOG::JPA:
		{
			alg = new HierarchicalSearch(new NoInsertionPolicy(),
						new FlexibleAStar(
							new JPAExpansionPolicy(), 
								newHeuristic()),
						new OctileDistanceRefinementPolicy(aMap));
			((HierarchicalSearch*)alg)->setName("JPAS");
			alg->verbose = verbose;
			break;
		}

		default:
		{
			alg = new FlexibleAStar(newExpansionPolicy(aMap), newHeuristic());
			alg->verbose = verbose;
			break;
		}
	}
	return alg;
}

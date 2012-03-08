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
#include "DebugUtility.h"
#include "DefaultInsertionPolicy.h"
#include "DefaultRefinementPolicy.h"
#include "EdgeFactory.h"
#include "EmptyCluster.h"
#include "EmptyClusterAbstraction.h"
#include "EmptyClusterFactory.h"
#include "EmptyClusterInsertionPolicy.h"
#include "FlexibleAStar.h"
#include "fpUtil.h"
#include "HierarchicalSearch.h"
#include "RSRSearch.h"
#include "HPAClusterAbstraction.h"
#include "HPAClusterFactory.h"
#include "hog.h"
#include "IncidentEdgesExpansionPolicy.h"
#include "JumpPointAbstraction.h"
#include "JumpPointExpansionPolicy.h"
#include "JumpPointRefinementPolicy.h"
#include "JumpPointSearch.h"
#include "mapFlatAbstraction.h"
#include "MacroNodeFactory.h"
#include "ManhattanHeuristic.h"
#include "NodeFactory.h"
#include "NoInsertionPolicy.h"
#include "NoRefinementPolicy.h"
#include "OctileDistanceRefinementPolicy.h"
#include "OctileHeuristic.h"
#include "OfflineJumpPointLocator.h"
#include "OnlineJumpPointLocator.h"
#include "RecursiveJumpPointExpansionPolicy.h"
#include "RRExpansionPolicy.h"
#include "ScenarioManager.h"
#include "searchUnit.h"
#include "statCollection.h"

#include <cstdlib>
#include <climits>
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
bool checkOptimality = false;
bool runNext = false;

std::string scenarioFile;

// search algorithm parameter
HOG::SearchMethod searchType = HOG::ASTAR;

// JPS default parameters
int jps_recursion_depth = 0;
bool jps_online = true;

// RSR default parameters
bool reducePerimeter = true;
bool bfReduction = true;

// number of times to run experiments in current scenario
// only used when running with -nogui
int repeat=1;

// when true, the input graph is exported to file.
bool export_graph=false;
std::string export_graph_filename("");
unsigned int export_graph_level = 0;

// when true, load an input graph from file instead of
// generating it from the map description
bool import_graph=false;
std::string import_graph_filename("");

/**
 * This function is called each time a unitSimulation is deallocated to
 * allow any necessary stat processing beforehand
 */
void 
processStats(statCollection *stat)
{
        if(stat->getNumStats() == 0)
                return;

		for(int i=0; i < stat->getNumOwners(); i++)
		{
			processStats(stat, stat->lookupOwnerID(i));
		}
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

	stat->clearAllStats();
}

/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void 
createSimulation(unitSimulation * &unitSim)
{
	Map* map = new Map(gDefaultMap);
	int scalex = map->getMapWidth();
	int scaley = map->getMapHeight();
	if(scenariomgr.getNumExperiments() > 0)
	{
		scalex = scenariomgr.getNthExperiment(0)->getXScale();
		scaley = scenariomgr.getNthExperiment(0)->getYScale();
		if(scalex > 1 && scaley > 1) // stupid v3 scenario files 
			map->scale(scalex, scaley);
	}

	// print the list of parameters in use
	bool asserts_enabled = false;
	assert((asserts_enabled = true));
	std::cout << "Configuration Parameters: "<<std::endl;
	std::cout << " build: ";
	if(asserts_enabled)
	{ 
		if(Jump::hog_asserts_enabled())
			std::cout << "debug"<<std::endl;
		else
			std::cout << "hybrid" << std::endl;
	}
	else
	{
		if(Jump::hog_asserts_enabled())
			std::cout << "hybrid"<<std::endl;
		else
			std::cout << "fast" << std::endl;
	}
	std::cout << " gui: "<<(getDisableGUI()?"false":"true") << std::endl;
	std::cout << " map file: "<<gDefaultMap << 
		" ("<<map->getMapWidth()<<"x"<<map->getMapHeight() << ")"<<std::endl;
	std::cout << " scenario file: "<<scenarioFile<<std::endl;
	std::cout << " import file: "<<import_graph_filename<<std::endl;
	std::cout << " export file: "<<export_graph_filename<<std::endl;
	std::cout << "Experiment Parameters:";
	std::cout << " repeat="<<repeat << std::endl;
	std::cout << " cardinal="<<(!allowDiagonals?"true":"false") << std::endl;
	std::cout << " search=";
	switch(searchType)
	{
		case HOG::HPA:
			std::cout << "HPA";
			break;
		case HOG::RSR:
			std::cout << "RSR";
			if(reducePerimeter)
				std::cout << "+pr";
			if(bfReduction)
				std::cout << "+bfr";
			break;
		case HOG::JPS:
			std::cout << "JPS";
			if(!jps_online)
				std::cout << "+preprocessing";
			std::cout << "(recursion_depth="<<jps_recursion_depth
				<< ")";
			break;
		case HOG::ASTAR:
			std::cout << "A*";
			break;
		default:
			std::cout << "Unknown?? Fix me!!";
			exit(1);
	}
	std::cout << std::endl;


	mapAbstraction* aMap = 0;
	switch(searchType)
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
		case HOG::RSR:
		{
			aMap = new EmptyClusterAbstraction(map, 
					new EmptyClusterFactory(), new MacroNodeFactory(),
				   	new EdgeFactory(), allowDiagonals, reducePerimeter, 
					bfReduction);

			//dynamic_cast<EmptyClusterAbstraction*>(aMap)->setVerbose(verbose);
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->buildClusters();
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->buildEntrances();
			dynamic_cast<EmptyClusterAbstraction*>(aMap)->clearColours();
			break;
		}
		case HOG::JPS:
		{
			if(!jps_online)
			{
				if(import_graph)
				{
					aMap = new JumpPointAbstraction(map, new NodeFactory(), 
							new EdgeFactory(), import_graph_filename,verbose);
				}
				else
				{
					aMap = new JumpPointAbstraction(map, new NodeFactory(), 
							new EdgeFactory(), verbose);
				}
			}
			else
			{
				aMap = new mapFlatAbstraction(map);
			}
			break;
		}
		default:
			aMap = new mapFlatAbstraction(map);
			break;
	}

	// export the level 1 abstract graph
	if(export_graph && export_graph_level < aMap->getNumAbstractGraphs())
	{
		export_search_graph(aMap->getAbstractGraph(export_graph_level));
	}

//	if(verbose && aMap->getNumAbstractGraphs() > 1)
//	{
//		Heuristic* h = newHeuristic();
//		DebugUtility debug(aMap, h);
//		debug.printGraph(aMap->getAbstractGraph(1));
//		delete h;
//	}

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
		int exitVal = 0; // nonzero indicates errors
		for(int i=0; i < repeat; i++)
		{
			exitVal = gogoGadgetNOGUIScenario(aMap);
			if(exitVal)
			{
				std::cout << "early termination\n";
				break;
			}
		}
		delete aMap;
		exit(exitVal);
	}
}

int
gogoGadgetNOGUIScenario(mapAbstraction* aMap)
{
	int exitVal = 0;

	FlexibleAStar* astar = 0;
	//aStarOld* astar = 0;
	mapFlatAbstraction* gridmap = 0;
	if(checkOptimality)
	{
		// scale the grid map if necessary
		Map* map = new Map(gDefaultMap);
		int scalex = map->getMapWidth();
		int scaley = map->getMapHeight();
		if(scenariomgr.getNumExperiments() > 0)
		{
			scalex = scenariomgr.getNthExperiment(0)->getXScale();
			scaley = scenariomgr.getNthExperiment(0)->getYScale();
			if(scalex > 1 && scaley > 1) // stupid v3 scenario files 
				map->scale(scalex, scaley);
		}

		// reference map and search alg for checking optimality
		gridmap = new mapFlatAbstraction(map);
		astar = new FlexibleAStar(new IncidentEdgesExpansionPolicy(gridmap), new OctileHeuristic());
		//astar = new aStarOld();
	}

	searchAlgorithm* alg = newSearchAlgorithm(aMap, false);
	statCollection stats;
	double optlen=0;
	
	for(int i=0; i< scenariomgr.getNumExperiments(); i++)
	{
		expnum = i;
		nextExperiment = (Experiment*)scenariomgr.getNthExperiment(i);

		node* from = aMap->getNodeFromMap(nextExperiment->getStartX(), 
				nextExperiment->getStartY());
		node* to = aMap->getNodeFromMap(nextExperiment->getGoalX(), 
				nextExperiment->getGoalY());

		alg->verbose = verbose;
		path* p = alg->getPath(aMap, from, to);
		double distanceTravelled = 0;

		if(p)
		{
			distanceTravelled = aMap->distance(p);	
		}

		stats.addStat("distanceMoved", alg->getName(), distanceTravelled);
		alg->logFinalStats(&stats);
		processStats(&stats, alg->getName());
		stats.clearAllStats();
		delete p;

		std::cout << alg->getName() << ": ";
		std::cout << "exp "<<expnum<<" ";
		nextExperiment->print(std::cout);
		std::cout << " " << distanceTravelled;
		std::cout << std::endl;

		if(checkOptimality)
		{
			// run A* to check the optimal path length
			node* s = gridmap->getNodeFromMap(nextExperiment->getStartX(),
					nextExperiment->getStartY());
			assert(s);
			node* g = gridmap->getNodeFromMap(nextExperiment->getGoalX(),
					nextExperiment->getGoalY());
			assert(g);
			optlen = 0;

			p = astar->getPath(gridmap, s, g);
			if(p)
			{
				optlen = gridmap->distance(p);
			}
			delete p;

			if(!fequal(optlen, distanceTravelled))
			{
				std::cout << "optimality check failed!" << std::endl;
				std::cout << "exp "<<expnum<<" ";
				nextExperiment->print(std::cout);
				std::cout << " " << distanceTravelled;
				std::cout << std::endl;
				std::cout << "optimal path length: "<<optlen<<" computed length: ";
				std::cout << distanceTravelled<<std::endl;
				verbose = true;
				if(verbose)
				{
					std::cout << "Running A*: \n";
					astar->verbose = true;
					alg->verbose = true;
					path* p = astar->getPath(gridmap, s, g);
					double tmp = gridmap->distance(p);
					delete p;
					p = 0;

					std::cout << "\nRunning "<<alg->getName()<<": \n";
					p = alg->getPath(aMap, from, to);
					double tmp2 = aMap->distance(p);
					delete p;

					std::cout << "\n optimal: "<<tmp;
					std::cout << " computed: "<<tmp2<<std::endl;
				}
				exitVal = 1;
				break;
			}
		}
	}
	
	delete alg;

	if(checkOptimality)
	{
		delete astar;
		delete gridmap;
	}

	return exitVal;
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

	installKeyboardHandler(myNewUnitKeyHandler, "Add fast A* Unit", 
			"Run Next Experiment", kNoModifier, 'n');

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

	installCommandLineHandler(myAllPurposeCLHandler, "-repeat", "-repeat [times]", 
			"Number of times to repeat the current scenario (default = 1)");

	installCommandLineHandler(myAllPurposeCLHandler, "-export", "-export [filename]", 
			"export the search graph to the specified file.");

	installCommandLineHandler(myAllPurposeCLHandler, "-import", "-import [filename]", 
			"import the search graph from a specified file (instead of generating it "
			"from a map file).");

	installCommandLineHandler(myAllPurposeCLHandler, "-cardinal", "-cardinal", 
			"Disallow diagonal moves during search "
			"(default = false)");

	installCommandLineHandler(myAllPurposeCLHandler, "-checkopt", "-checkopt", 
			"Verify each experiment ran is solved optimally."
			"(default = false)");

	installCommandLineHandler(myAllPurposeCLHandler, "-search", 
			"-search [algorithm]", 
			"Available Search Algorithms:\n"
			"\tastar = standard a* search\n"
			"\trsr = rectangular symmetry reduction\n" 
			"\tjps = online jump point search\n"
			);

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
	else if(strcmp(argument[0], "-export") == 0)
	{
		argsParsed++;
		if(argsParsed < maxNumArgs && *argument[1] != '-')
		{
			argsParsed++;
			export_graph=true;
			export_graph_filename.append(argument[1]);
		}
	}
	else if(strcmp(argument[0], "-import") == 0)
	{
		argsParsed++;
		if(argsParsed < maxNumArgs && *argument[1] != '-')
		{
			argsParsed++;
			import_graph=true;
			import_graph_filename.append(argument[1]);
		}
	}
	else if(strcmp(argument[0], "-repeat") == 0)
	{
		argsParsed++;
		repeat = atoi(argument[1]);
		if(repeat == 0 || repeat == INT_MAX || repeat == INT_MIN)
		{
			repeat = 1;
		}
		else
		{
			argsParsed++;
		}
	}
	else if(strcmp(argument[0], "-search") == 0)
	{
		argsParsed++;
		if(strcmp(argument[1], "astar") == 0)
		{
			searchType = HOG::ASTAR; 
			argsParsed++;
		}
		else if(strcmp(argument[1], "hpa") == 0)
		{
			searchType = HOG::HPA; 
			argsParsed++;
		}
		else if(strcmp(argument[1], "rsr") == 0)
		{
			argsParsed++;
			searchType = HOG::RSR;
			int rsr_params = 0;
			for(int i=2; i < maxNumArgs; i++)
			{
				if(*argument[i] == '-')
					break;
				rsr_params++;
			}
			if(parse_jps_args(argument+2, rsr_params))
			{
				argsParsed += rsr_params;
			}
			else
			{
				printCommandLineArguments();
				exit(1);
			}
		}
		else if(strcmp(argument[1], "jps") == 0)
		{
			argsParsed++;
			searchType = HOG::JPS;
			int jps_params = 0;
			for(int i=2; i < maxNumArgs; i++)
			{
				if(*argument[i] == '-')
					break;
				jps_params++;
			}
			if(parse_jps_args(argument+2, jps_params))
			{
				argsParsed += jps_params;
			}
			else
			{
				printCommandLineArguments();
				exit(1);
			}
		}
		else
		{
			std::cout << argument[1] << ": invalid search type.\n";
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
		scenarioFile = infile;
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
myNewUnitKeyHandler(unitSimulation *unitSim, tKeyboardModifier mod, char c)
{

	if(c == 'n' && mod == kNoModifier)
	{
		runNext = true;
		runNextExperiment(unitSim);
		runNext = false;
		return;
	}

	mapAbstraction* aMap = unitSim->getMapAbstraction();
	aMap->clearColours();

	for(int i=0; i<unitSim->getNumUnits(); i++)
	{
		unit* lastunit = dynamic_cast<searchUnit*>(unitSim->getUnit(0));
		if(lastunit)
		{
			lastunit->logFinalStats(unitSim->getStats());
			statCollection* allStats = unitSim->getStats();
			for(int i=0; i < allStats->getNumOwners(); i++)
			{
				processStats(allStats, allStats->lookupOwnerID(i));
			}
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

	//switch (mod)
	//{
	//	case kShiftDown: 
	//	{
	//		astar = newSearchAlgorithm(aMap); 
	//		astar->verbose = verbose;
	//		unitSim->addUnit(u=new searchUnit(x2, y2, targ, astar)); 
	//		u->setColor(0.3,0.7,0.3);
	//		targ->setColor(0.3,0.7,0.3);
	//		break;
	//	}
	//	default:
	//	{
	//		astar = new FlexibleAStar(newExpansionPolicy(aMap), newHeuristic());	
	//		astar->verbose = verbose;
	//		unitSim->addUnit(u=new searchUnit(x2, y2, targ, astar)); 
	//		u->setColor(1,1,0);
	//		targ->setColor(1,1,0);
	//		break;
	//	}
	//}
	astar = newSearchAlgorithm(aMap); 
	if(searchType == HOG::JPS || searchType == HOG::RSR)
	{
		// wrap JPS and RSR in order to extract a refined path that can be 
		// immediately executed (only need this when running the gui)
		std::string algname(astar->getName());
		astar = new HierarchicalSearch(
				new NoInsertionPolicy(),
				astar, 
				new OctileDistanceRefinementPolicy(aMap));
		((HierarchicalSearch*)astar)->setName(algname.c_str());
	}	
	astar->verbose = verbose;
	unitSim->addUnit(u=new searchUnit(x2, y2, targ, astar)); 
	u->setColor(1,1,0);
	targ->setColor(1,1,0);
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
	if(!runNext)
		return;

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
	if(searchType == HOG::JPS || searchType == HOG::RSR)
	{
		// wrap JPS and RSR in order to extract a refined path that can be 
		// immediately executed (only need this when running the gui)
		std::string algname(alg->getName());
		alg = new HierarchicalSearch(
				new NoInsertionPolicy(),
				alg, 
				new OctileDistanceRefinementPolicy(aMap));
		((HierarchicalSearch*)alg)->setName(algname.c_str());
	}	

	alg->verbose = verbose;
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
	switch(searchType)
	{
		case HOG::HPA:
		{
			GenericClusterAbstraction* map = 
				dynamic_cast<GenericClusterAbstraction*>(aMap);
			alg = new HierarchicalSearch(
					new DefaultInsertionPolicy(map),
					new FlexibleAStar(
						new IncidentEdgesExpansionPolicy(map),
						newHeuristic()),
					newRefinementPolicy(0, map, true));
			((HierarchicalSearch*)alg)->setName("HPA");
			break;
		}
		case HOG::RSR:
		{
			alg = new RSRSearch(bfReduction, newHeuristic());
			break;
		}

		case HOG::JPS:
		{
			alg = new JumpPointSearch(jps_online, jps_recursion_depth, 
					newHeuristic(), aMap);
			break;
		}

		default:
		{
			alg = new FlexibleAStar(
					new IncidentEdgesExpansionPolicy(aMap), newHeuristic());
			break;
		}
	}
	alg->verbose = verbose;
	return alg;
}

RefinementPolicy*
newRefinementPolicy(ExpansionPolicy* expander, mapAbstraction* map, bool refine)
{
	if(!refine)
		return new NoRefinementPolicy();

	RefinementPolicy* refPol = 0;
	switch(searchType)
	{
		case HOG::HPA:
		{
			GenericClusterAbstraction* gcaMap = 
				dynamic_cast<GenericClusterAbstraction*>(map);

			refPol = new DefaultRefinementPolicy(gcaMap);
			break;
		}

		case HOG::JPS:
		{
			refPol = new OctileDistanceRefinementPolicy(map);
			break;
		}

		default:
		{
			refPol = new NoRefinementPolicy();
			break;
		}
	}

	return refPol;
}

bool
parse_jps_args(char** argument, int maxArgs)
{
	for(int i=0; i < maxArgs; i++)
	{
		if(strncmp(argument[i], "depth=", 6) == 0)
		{
			int tmp = atoi(argument[i]+6);
			if(tmp != INT_MAX || tmp != INT_MIN)
			{
				jps_recursion_depth = tmp;
			}
		}
		else if(strcmp(argument[i], "online") == 0)
		{
			jps_online = true;
		}

		else if(strcmp(argument[i], "offline") == 0)
		{
			jps_online = false;
		}
		else
		{
			std::cerr << "unrecognised search parameter: " << argument[i]
				<< std::endl;
			return false;
		}
	}
	return true;
}

bool
parse_rsr_args(char** argument, int maxArgs)
{
	reducePerimeter = true;
	bfReduction = true;
	for(int i=0; i < maxArgs; i++)
	{
		if(strcmp(argument[i], "simple") == 0)
		{
			bfReduction = false;
			reducePerimeter = false;
		}
		else if(strcmp(argument[i], "bfr_only") == 0)
		{
			bfReduction = true;
			reducePerimeter = false;
		}

		else if(strcmp(argument[i], "pr_only") == 0)
		{
			bfReduction = false;
			reducePerimeter = true;
		}
		else
		{
			std::cerr << "unrecognised search parameter: " << argument[i]
				<< std::endl;
			return false;
		}
	}
	return true;
}

void
export_search_graph(graph* g)
{
	assert(export_graph && export_graph_filename.size() > 0);
	std::ofstream fout(export_graph_filename.c_str());
	if(fout.fail())
	{
		std::cout << "Export failed. Cannot open target file: "
			<< export_graph_filename << std::endl;
		return;
	}
	g->print(fout);
	fout.close();
}

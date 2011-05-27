/*
 *  HPAStar.cpp
 *  hog
 *
 *  Created by dharabor on 17/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "HPAStar2.h"

#include "ExpansionPolicy.h"
#include "DebugUtility.h"
#include "FlexibleAStar.h"
#include "Heuristic.h"
#include "HPACluster.h"
#include "HPAClusterAbstraction.h"
#include "ISearchAlgorithmFactory.h"
#include "timer.h"
#include "searchAlgorithm.h"
#include <stdexcept>

HPAStar2::HPAStar2(ExpansionPolicy* policy, Heuristic* heuristic,
		bool _refine, bool _fastRefinement) : searchAlgorithm()
{ 
	refineAbstractPath = _refine; fastRefinement = _fastRefinement; 
	astar = new FlexibleAStar(policy, heuristic);
}

HPAStar2::~HPAStar2() 
{
	delete astar;
}

/* 
Find an abstract path and refine it using the path cache

 NB: sometimes we may require a cached path which was obtained by planning in the reverse direction to current requirements
 ie. we store the path from n1 -> n2, but we may require the path from n2 -> n1. 
 In such cases, we specify the id of the node that should be at the head of the path. if the cached path doesn't meet those
 requirements, we reverse it. 
*/
path* HPAStar2::getPath(graphAbstraction* aMap, node* from, node* to, reservationProvider *rp)
{
	resetMetrics();
	astar->verbose = verbose;
	if(!checkParameters(from, to))
		return NULL;

	HPAClusterAbstraction* hpamap = dynamic_cast<HPAClusterAbstraction*>(aMap);
	assert(hpamap != 0); 		

	Timer t;
	path* thepath=0;
	if(verbose)
	{
		std::cout << "trying to find abstract path; inserting s+g"<<std::endl;
		hpamap->setVerbose(true);
	}
	t.startTimer();

	// NB: original HPA* implementation could be sped up here by running  a low
	// level search if both S+G are from the same cluster.
	hpamap->insertStartAndGoalNodesIntoAbstractGraph(from, to);		

	insertSearchTime = t.endTimer();
	nodesExpanded = insertNodesExpanded = hpamap->getNodesExpanded();
	nodesTouched = insertNodesTouched = hpamap->getNodesTouched();
	nodesGenerated = insertNodesGenerated = hpamap->getNodesGenerated();

	// find a path in the abstract graph
	graph *absg = hpamap->getAbstractGraph(1); 
	node* absstart = absg->getNode(from->getLabelL(kParent));
	node* absgoal = absg->getNode(to->getLabelL(kParent));

	if(verbose)
		std::cout << "searching for path in abstract graph"<<std::endl;

	path* abspath = astar->getPath(hpamap, absstart, absgoal);
	updateMetrics();
	
	// refine the path
	if(abspath) 
	{
		if(verbose)
		{
			DebugUtility debug(aMap, astar->getHeuristic());
			std::cout << "abstract path: ";
			debug.printPath(abspath);
			std::cout << std::endl;
		}

		if(refineAbstractPath)
		{
			if(verbose)
				std::cout << "refining abstract path..."<<std::endl;

			thepath = refinePath(abspath, hpamap);
			delete abspath;
		}
		else
		{
			if(verbose)
				std::cout << "no refinement; returning a non-refined path consisting only of non-abstract nodes"<<std::endl;

			/* return a non-refined path consisting only of non-abstract nodes */
			thepath = abspath; 
			while(abspath)
			{
				abspath->n = hpamap->getNodeFromMap(abspath->n->getLabelL(kFirstData), abspath->n->getLabelL(kFirstData+1));
				abspath = abspath->next;
			}
		}
		if(verbose) {
			std::cout << "solution: \n"; 
			DebugUtility debug(aMap, astar->getHeuristic());
			debug.printPath(thepath);
		}
	}
	else
		if(verbose) std::cout << "no solution found!" <<std::endl;


	t.startTimer();
	hpamap->removeStartAndGoalNodesFromAbstractGraph();
	insertSearchTime += t.endTimer();
	searchTime += insertSearchTime;

	//std::cout << "\n thepath distance: "<<aMap->distance(thepath);
	//std::cout << "the path: "<<std::endl;
	//printPath(thepath);
	return thepath;
}

// TODO: add support for refining paths across multliple levels of hierarchy.
// This probably means storing a downward pointer with each abstract node.
path* HPAStar2::refinePath(path* abspath, HPAClusterAbstraction* hpamap) 
{
	astar->verbose = false; // always off
	graph *absg = hpamap->getAbstractGraph(1); 
	path* thepath = 0;
	while(abspath->next)
	{
		path* segment;
		if(fastRefinement)
		{
			edge* e = absg->findEdge(abspath->n->getNum(), abspath->next->n->getNum());
			if(e == NULL)
				throw std::runtime_error("HPAStar2::getPath -- something went horribly wrong; couldn't find cached path. ");

			segment = hpamap->getPathFromCache(e)->clone();
			if(e->getFrom() != abspath->n->getNum()) // fix segments if necessary
				segment = segment->reverse();				
		}
		else
		{
			node* llstart = hpamap->getNodeFromMap(abspath->n->getLabelL(kFirstData), 
					abspath->n->getLabelL(kFirstData+1));
			node* llgoal =  hpamap->getNodeFromMap(abspath->next->n->getLabelL(kFirstData), 
					abspath->next->n->getLabelL(kFirstData+1));

			segment = astar->getPath(hpamap, llstart, llgoal); 
			assert(segment != 0);
			updateMetrics();

			if(verbose) 
			{
			   	std::cout << "refined segment: "<<std::endl; 
				DebugUtility debug(hpamap, astar->getHeuristic());
				debug.printPath(segment); 
				std::cout << " distance: "<<hpamap->distance(segment)<<std::endl; 
			}
		}
		
		// append segment to refined path
		if(thepath == 0)
			thepath = segment;										
		path* tail = thepath->tail();	

		// avoid overlap between successive segments (i.e one segment ends with the same node as the next begins)
		if(tail->n->getNum() == segment->n->getNum()) 
		{
			tail->next = segment->next;
			segment->next = 0;
			delete segment;
		}
		
		abspath = abspath->next;
	}

	astar->verbose = verbose;
	return thepath;
}


void HPAStar2::updateMetrics()
{	
	this->nodesExpanded += this->astar->getNodesExpanded();
	this->nodesTouched += this->astar->getNodesTouched();
	this->nodesGenerated += this->astar->getNodesGenerated();
	this->searchTime += this->astar->getSearchTime();
}

void HPAStar2::logFinalStats(statCollection* stats)
{
	searchAlgorithm::logFinalStats(stats);

	stats->addStat("insNodesExpanded",getName(),getInsertNodesExpanded());
	stats->addStat("insNodesTouched",getName(),getInsertNodesTouched());
	stats->addStat("insNodesGenerated",getName(),getInsertNodesGenerated());
	stats->addStat("insSearchTime",getName(),getInsertSearchTime());
}

void HPAStar2::resetMetrics()
{
	insertNodesExpanded = insertNodesTouched = insertNodesGenerated = 0;
	nodesExpanded = nodesTouched = nodesGenerated = 0;
	insertSearchTime = searchTime = 0;
}

bool HPAStar2::checkParameters(node* from, node* to)
{
	if(!from || !to)
		return false;

	if(from->getUniqueID() == to->getUniqueID())
		return false;
		
	if(from->getLabelL(kFirstData) == to->getLabelL(kFirstData) 
			&& from->getLabelL(kFirstData+1) == to->getLabelL(kFirstData+1))
		return false;

	if(from->getLabelL(kAbstractionLevel) != to->getLabelL(kAbstractionLevel))
		return false;

	return true;
	
}

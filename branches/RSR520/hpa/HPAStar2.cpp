/*
 *  HPAStar.cpp
 *  hog
 *
 *  Created by dharabor on 17/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "HPAStar2.h"
#include "HPAClusterAbstraction.h"
#include "IClusterAStarFactory.h"
#include "ClusterNode.h"
#include "HPACluster.h"
#include <stdexcept>

HPAStar2::HPAStar2(bool _refine, bool _fastRefinement, IClusterAStarFactory* _caf) 
{ 
	init(_refine, _fastRefinement, _caf); 
}

HPAStar2::HPAStar2(IClusterAStarFactory* caf) 
{ init(true, false, caf); 
}
		
void HPAStar2::init(bool _refine, bool _fastRefinement, IClusterAStarFactory* _caf) 
{ 
	refineAbstractPath = _refine; fastRefinement = _fastRefinement; caf = _caf;
}

HPAStar2::~HPAStar2() 
{
	delete caf; 
}

/* 
Find an abstract path and refine it using the path cache

 NB: sometimes we may require a cached path which was obtained by planning in the reverse direction to current requirements
 ie. we store the path from n1 -> n2, but we may require the path from n2 -> n1. 
 In such cases, we specify the id of the node that should be at the head of the path. if the cached path doesn't meet those
 requirements, we reverse it. 
*/
path* HPAStar2::getPath(graphAbstraction* aMap, node* _from, node* _to, reservationProvider *rp)
{
	resetMetrics();
	if(!checkParameters(aMap, _from, _to))
		return NULL;
	if(_from->getLabelL(kAbstractionLevel) > 0 || _to->getLabelL(kAbstractionLevel) > 0)
	{
		throw std::invalid_argument("HPAStar2::getPath failed. from/to nodes must be non-abstract nodes");
	}
		
	HPAClusterAbstraction* hpamap = dynamic_cast<HPAClusterAbstraction*>(aMap);
	assert(hpamap != 0); 		
	this->setGraphAbstraction(hpamap);
	ClusterNode* from = dynamic_cast<ClusterNode*>(_from);
	ClusterNode* to = dynamic_cast<ClusterNode*>(_to);
					
	AbstractClusterAStar* castar = caf->newClusterAStar();
	castar->verbose = verbose;
	path* thepath=0;

	// if from/goal are in the same cluster, try to find a path directly
	if(from->getParentClusterId() == to->getParentClusterId())
	{
		if(verbose)
			std::cout << "start + goal in the same cluster; trying to find a non-abstract path"<<std::endl;

		HPACluster *cluster = hpamap->getCluster(from->getParentClusterId());
		castar->setCorridorNodes(cluster->getNodes());
		thepath = castar->getPath(aMap, from, to, rp);
		updateMetrics(*castar);
	}
	
	// no direct path or from/goal not in the same cluster
	if(thepath==0)
	{
		if(verbose)
		{
			std::cout << "trying to find abstract path; inserting s+g"<<std::endl;
			hpamap->setVerbose(true);
		}

		hpamap->insertStartAndGoalNodesIntoAbstractGraph(from, to);		
		this->nodesExpanded += hpamap->getNodesExpanded();
		this->nodesTouched += hpamap->getNodesTouched();
		this->searchTime += hpamap->getSearchTime();
		if(this->peakmemory < hpamap->getPeakMemory())
			this->peakmemory = hpamap->getPeakMemory();
				
		// find a path in the abstract graph
		graph *absg = hpamap->getAbstractGraph(1); 
		node* absstart = absg->getNode(from->getLabelL(kParent));
		node* absgoal = absg->getNode(to->getLabelL(kParent));
		castar->setCorridorNodes(this->corridorNodes);

		if(verbose)
			std::cout << "searching for path in abstract graph"<<std::endl;

		path* abspath = castar->getPath(aMap, absstart, absgoal);
		updateMetrics(*castar);
		castar->markForVis = false;
		
		// refine the path
		if(abspath) 
		{
			if(verbose)
			{
				std::cout << "abstract path: ";
				printPath(abspath);
				std::cout << std::endl;
			}

			if(refineAbstractPath)
			{
				if(verbose)
					std::cout << "refining abstract path..."<<std::endl;

				thepath = refinePath(abspath, hpamap, *castar);
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
			if(verbose) {std::cout << "solution: \n"; printPath(thepath);}
		}
		else
			if(verbose) std::cout << "no solution found!" <<std::endl;


		hpamap->removeStartAndGoalNodesFromAbstractGraph();
		insertNodesExpanded = hpamap->getNodesExpanded(); // record insertion separately also.
		insertNodesTouched = hpamap->getNodesTouched();
		insertPeakMemory = hpamap->getPeakMemory();
		insertSearchTime = hpamap->getSearchTime();
	}
		
	//std::cout << "\n thepath distance: "<<aMap->distance(thepath);
	//std::cout << "the path: "<<std::endl;
	//printPath(thepath);
	delete castar;
	return thepath;
}

path* HPAStar2::refinePath(path* abspath, HPAClusterAbstraction* hpamap, AbstractClusterAStar& castar) 
{
	graph *absg = hpamap->getAbstractGraph(1); 
	path* thepath = 0;
	path* tail;
	ClusterNode *debug1, *debug2;
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
			ClusterNode* llstart = dynamic_cast<ClusterNode*>(
					hpamap->getNodeFromMap(abspath->n->getLabelL(kFirstData), abspath->n->getLabelL(kFirstData+1)));
			ClusterNode* llgoal = dynamic_cast<ClusterNode*>(
					hpamap->getNodeFromMap(abspath->next->n->getLabelL(kFirstData), abspath->next->n->getLabelL(kFirstData+1)));
			if(llstart->getParentClusterId() == llgoal->getParentClusterId()) // intra-edge refinement limited to a single cluster.
			{ 
				HPACluster* cluster = hpamap->getCluster(llstart->getParentClusterId());
				castar.setCorridorNodes(cluster->getNodes());
			}
			else
				 castar.setCorridorNodes(NULL); 
				
			segment = castar.getPath(hpamap,llstart, llgoal); 
			if(segment == 0)
			{
				std::cout << "null segment??"<<std::endl;
				segment = castar.getPath(hpamap,llstart, llgoal); 
			}

			updateMetrics(castar);
			if(verbose) 
			{
			   	std::cout << "refined segment: "<<std::endl; 
				printPath(segment); 
				std::cout << " distance: "<<hpamap->distance(segment)<<std::endl; 
			}

			debug1 = llstart;
			debug2 = llgoal;
		}
		
		// append segment to refined path
		if(thepath == 0)
			thepath = segment;										
		tail = thepath->tail();	

		// avoid overlap between successive segments (i.e one segment ends with the same node as the next begins)
		if(tail->n->getNum() == segment->n->getNum()) 
		{
			tail->next = segment->next;
			segment->next = 0;
			delete segment;
		}
		
		abspath = abspath->next;
	}
	return thepath;
}


void HPAStar2::updateMetrics(AbstractClusterAStar& castar)
{	
	this->nodesExpanded += castar.getNodesExpanded();
	this->nodesTouched += castar.getNodesTouched();
	if(this->peakmemory < castar.getPeakMemory())
		this->peakmemory = castar.getPeakMemory();
	this->searchTime += castar.getSearchTime();
}

void HPAStar2::logFinalStats(statCollection* stats)
{
	ClusterAStar::logFinalStats(stats);

	stats->addStat("insNodesExpanded",getName(),getInsertNodesExpanded());
	stats->addStat("insNodesTouched",getName(),getInsertNodesTouched());
	stats->addStat("insPeakMemory",getName(),getInsertPeakMemory());
	stats->addStat("insSearchTime",getName(),getInsertSearchTime());
}

void HPAStar2::resetMetrics()
{
	insertNodesExpanded = insertNodesTouched = insertPeakMemory = 0;
	insertSearchTime = 0;
	this->nodesExpanded = 0;
	this->nodesTouched = 0;
	this->peakmemory = 0;
	this->searchTime = 0;
}

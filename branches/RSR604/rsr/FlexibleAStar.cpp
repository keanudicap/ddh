#include "FlexibleAStar.h"

#include "DebugUtility.h"
#include "ExpansionPolicy.h"
#include "graph.h"
#include "heap.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "path.h"
#include "reservationProvider.h"
#include "timer.h"
#include "unitSimulation.h"

FlexibleAStar::FlexibleAStar(ExpansionPolicy* policy, Heuristic* heuristic)
	: searchAlgorithm()
{
	this->policy = policy;
	this->heuristic = heuristic;
	this->markForVis = true;
}

FlexibleAStar::~FlexibleAStar()
{
	delete policy;
	delete heuristic;
}

const char* FlexibleAStar::getName()
{
	return "FlexibleAStar";
}

path* FlexibleAStar::getPath(graphAbstraction *aMap, node *start, node *goal,
		reservationProvider *rp)
{
	debug = new DebugUtility(aMap, heuristic);
	path* p = search(start, goal);
	delete debug;
	return p;
}

path* FlexibleAStar::search(node* start, node* goal)
{
	nodesExpanded=0;
	nodesTouched=0;
	searchTime =0;
	nodesGenerated = 0;

	if(verbose) 
	{
		std::cout << "getPath() mapLevel: ";
		std::cout <<start->getLabelL(kAbstractionLevel)<<std::endl;
	}

	if(!checkParameters(start, goal))
		return NULL;

	start->setLabelF(kTemporaryLabel, heuristic->h(start, goal));
	start->backpointer = 0;
	
	heap openList(30);
	std::map<int, node*> closedList;
	
	openList.add(start);
	path *p = NULL;
	
	Timer t;
	t.startTimer();
	while(1) 
	{
		node* current = ((node*)openList.remove()); 

		// check if the current node is the goal (early termination)
		if(current == goal)
		{
			p = extractBestPath(current);
			if(verbose)
				debug->printNode(std::string("goal found! "), current);
			break;
		}
		
		// expand current node
		expand(current, goal, &openList, &closedList);
		closeNode(current, &closedList);
				
		// terminate when the open list is empty
		if(openList.empty())
		{
			if(verbose) std::cout << "search failed. ";
			break;
		}
	}
	searchTime = t.endTimer();
	closedList.clear();

	if(verbose)
	{
		std::cout << "\n";
		debug->printPath(p); 
	}

	return p;	
}

void FlexibleAStar::closeNode(node* current, std::map<int, node*>* closedList)
{
	if(markForVis)
		current->drawColor = 2; // visualise expanded

	if(verbose)
	{	
		debug->printNode(std::string("closing... "), current);
		std::cout << " f: "<<current->getLabelF(kTemporaryLabel) <<std::endl;
	}
	closedList->insert(std::pair<int, node*>(current->getUniqueID(), current));


}

void FlexibleAStar::expand(node* current, node* goal, heap* openList,
		std::map<int, node*>* closedList)
{
	// expand the current node
	if(verbose) 
	{
		debug->printNode(std::string("expanding... "), current);
		double g = current->getLabelF(kTemporaryLabel) 
			- heuristic->h(current, goal);	
		std::cout << " fNew: "<<current->getLabelF(kTemporaryLabel);
		std::cout << " g: "<<g;
		std::cout << std::endl;
	}

	nodesExpanded++;

	policy->expand(current);
	for(node* neighbour = policy->first(); neighbour != 0; 
			neighbour = policy->next())
	{
		nodesTouched++;			
		if(closedList->find(neighbour->getUniqueID()) == closedList->end()) 
		{
			if(openList->isIn(neighbour)) 
			{	
				if(verbose) 
				{
					debug->printNode("\t\trelaxing...", neighbour);
					std::cout << " fOld: "<<neighbour->getLabelF(kTemporaryLabel);
				}

				relaxNode(current, neighbour, goal, policy->cost_to_n(), openList); 

				if(verbose)
				{
					double g = neighbour->getLabelF(kTemporaryLabel) 
						- heuristic->h(neighbour, goal);	
					std::cout << " fNew: "<<neighbour->getLabelF(kTemporaryLabel);
					std::cout << " g: "<<g;
				}
			}
			else
			{
				if(verbose) 
					debug->printNode("\t\tgenerating...", neighbour);

				neighbour->setLabelF(kTemporaryLabel, MAXINT); // initial fCost 
				neighbour->setKeyLabel(kTemporaryLabel); // store priority here 
				neighbour->backpointer = 0;  // reset any marked edges 
				openList->add(neighbour);
				relaxNode(current, neighbour, goal, policy->cost_to_n(), openList); 
				nodesGenerated++;

				if(verbose)
				{
					double g = neighbour->getLabelF(kTemporaryLabel) 
						- heuristic->h(neighbour, goal);	
					std::cout << " fNew: "<<neighbour->getLabelF(kTemporaryLabel);
					std::cout << " g: "<<g << " cost_to_n: "<<policy->cost_to_n();
					std::cout << " h: "<<heuristic->h(neighbour, goal);
				}
			}
			if(markForVis)
				neighbour->drawColor = 1; // visualise touched
		}
		else
			debug->debugClosedNode(current, neighbour, policy->cost_to_n(), goal);

		if(verbose)
			std::cout << std::endl;
	}
}

bool FlexibleAStar::checkParameters(node* from, node* to)
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

void FlexibleAStar::relaxNode(node* from, node* to, node* goal, double cost, heap* openList)
{
	double g_from = from->getLabelF(kTemporaryLabel) - heuristic->h(from, goal);
	double f_to = g_from + cost + heuristic->h(to, goal);
	
	if(f_to < to->getLabelF(kTemporaryLabel))
	{
		to->setLabelF(kTemporaryLabel, f_to);
		to->backpointer = from;
		openList->decreaseKey(to);
	}
}

path* FlexibleAStar::extractBestPath(node* goal)
{
	path* p = 0;
	for(node* n = goal; n != 0; n = n->backpointer)
		p = new path(n, p);

	return p;
}


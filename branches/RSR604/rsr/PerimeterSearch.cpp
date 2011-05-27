#include "PerimeterSearch.h"

#include "EmptyCluster.h"
#include "EmptyClusterAbstraction.h"
#include "MacroNode.h"
#include "MacroEdge.h"
#include <string>

PerimeterSearch::PerimeterSearch()
{
}

PerimeterSearch::~PerimeterSearch()
{

}

path* PerimeterSearch::getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp)
{
	visitedClusters.clear();
	return ClusterAStar::getPath(aMap, from, to, rp);
}

void PerimeterSearch::expand(node* current_, node* goal, edge_iterator begin, 
		unsigned int card, heap* openList, std::map<int, node*>& closedList, graph* g)
{
	if(verbose)
		std::cout << "PerimeterSearch expand..."<<std::endl;

	MacroNode* current = static_cast<MacroNode*>(
			current_);
	assert(current);

	AbstractClusterAStar::expand(current, goal, current->getEdgeIter(), 
			current->getNumEdges(), openList, closedList, g);

	
	// process any neighbours connected via secondary edges
	if(verbose)
		std::cout << "any secondary edges? "<<current->numSecondaryEdges()<<std::endl;
	if(current->numSecondaryEdges() > 0 && expandSecondary(current, goal))
	{
/*		edge* markedEdge = current->getMarkedEdge();
		if(markedEdge) // only start node has no marked edge (i.e. parent)
		{
				int parentId = markedEdge->getFrom()==current->getNum()?
						markedEdge->getTo():markedEdge->getFrom();
				MacroNode* parent = static_cast<MacroNode*>(
								g->getNode(parentId));
				assert(parent);

				if(parent->getParentClusterId() != current->getParentClusterId() && 
						expandSecondary(current))
				{
*/
					if(verbose)
						std::cout << "processing secondary edges; ";
					AbstractClusterAStar::expand(current, goal, current->secondaryEdgeIter(), 
							current->numSecondaryEdges(), openList, closedList, g);
					nodesExpanded--; // no double counting
/*				}
		}
		else if(nodesExpanded == 1) // process secondary edges associated with start node 
		{
				AbstractClusterAStar::expand(current, goal, current->secondaryEdgeIter(), 
								current->numSecondaryEdges(), openList, closedList, g);
				nodesExpanded--; // no double counting
		}
		else
		{
			if(verbose)
				std::cout << "skipping secondary edges"<<std::endl;
		}
		*/
	}
	else
	{
		if(verbose)
			std::cout << "no secondary edges; moving on"<<std::endl;
	}
}

bool PerimeterSearch::expandSecondary(MacroNode* current, node* goal)
{
	bool retVal = true;
	EmptyClusterAbstraction* aMap = dynamic_cast<EmptyClusterAbstraction*>(this->getGraphAbstraction());
	EmptyCluster* parentCluster = aMap->getCluster(current->getParentClusterId());

	if(visitedClusters.find(parentCluster->getId()) != visitedClusters.end())
	{
		EmptyCluster::RoomSide side = parentCluster->whichSide(current);
		node* best = parentCluster->getBestExpandedNode(side);

		if(verbose)
			std::cout << "best node along side "<<side<<" of cluster "<<parentCluster->getId()<<"... ";

		if(best)
		{
			if(verbose)
				std::cout << "exists! dominated? ";
			double g_cur = current->getLabelF(kTemporaryLabel) - h(current, goal);
			double g_best = best->getLabelF(kTemporaryLabel) - h(best, goal);

			double g_delta = g_best - g_cur;
			if(g_delta < 0)
				g_delta*=-1;

			double dist = h(current, best)*(ROOT_TWO-1);
			int mds = parentCluster->getHeight()<parentCluster->getWidth()?
				parentCluster->getHeight():parentCluster->getWidth();
			mds-=2; // max # of diagonal steps required to reach one side of the perimeter from another

			if(h(current, best) <= mds && g_delta > dist)
			{
				if(verbose)
				{
					printNode("dominated by", best, goal);
					std::cout <<"g_best: "<<g_best<<" g_cur "<<g_cur<<" g_delta: "<<g_delta<<" dist(cur, best) "<<dist;
					std::cout <<std::endl;
				}
				retVal = false;
			}
			else
			{
				if(g_cur < g_best)
				{
					parentCluster->setBestExpandedNode(current);
				}
			}
			if(verbose)
				std::cout << (g_delta > dist) <<std::endl;
		}
		else
		{
			if(verbose)
				std::cout << "null! setting current."<<std::endl;
			parentCluster->setBestExpandedNode(current);
		}
	}
	else
	{
		if(verbose)
			std::cout << "cluster "<<parentCluster->getId()<<" not visited before. adding to list."<<std::endl;
		visitedClusters[parentCluster->getId()] = true;
		assert(visitedClusters.find(parentCluster->getId()) != visitedClusters.end());
		parentCluster->resetBest();
		parentCluster->setBestExpandedNode(current);
	}

	if(verbose)
		std::cout<<"expandSecondary result: "<<retVal<<std::endl;
	return retVal;
}


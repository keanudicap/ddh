#include "PerimeterSearch.h"
#include "MacroNode.h"
#include "MacroEdge.h"
#include <string>

PerimeterSearch::PerimeterSearch()
{
}

PerimeterSearch::~PerimeterSearch()
{

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
	if(current->numSecondaryEdges() > 0)
	{
		edge* markedEdge = current->getMarkedEdge();
		if(markedEdge) // only start node has no marked edge (i.e. parent)
		{
			int parentId = markedEdge->getFrom()==current->getNum()?
				markedEdge->getTo():markedEdge->getFrom();
			MacroNode* parent = static_cast<MacroNode*>(
					g->getNode(parentId));
			assert(parent);

			if(parent->getParentClusterId() != current->getParentClusterId())
			{
				if(verbose)
					std::cout << "processing secondary edges; \n";
				AbstractClusterAStar::expand(current, goal, current->secondaryEdgeIter(), 
						current->numSecondaryEdges(), openList, closedList, g);
				nodesExpanded--; // no double counting
			}
			else
			{
				if(verbose)
					std::cout << "skipping secondary edges (same cluster as parent)"<<std::endl;
			}
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
				std::cout << "node has secondary edges but no marked edge?!"<<std::endl;
		}
	}
	else
	{
		if(verbose)
			std::cout << "no secondary edges; moving on"<<std::endl;
	}
}

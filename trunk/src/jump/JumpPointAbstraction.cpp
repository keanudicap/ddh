#include "JumpPointAbstraction.h"

#include "IEdgeFactory.h"
#include "INodeFactory.h"
#include "OnlineJumpPointLocator.h"
#include "graph.h"
#include "map.h"
#include "OctileHeuristic.h"
#include <limits.h>

JumpPointAbstraction::JumpPointAbstraction(Map* _m, INodeFactory* _nf, 
		IEdgeFactory* _ef, bool _verbose) : mapAbstraction(_m), 
	verbose(_verbose)
{
	nf = _nf;
	ef = _ef;

	makeJumpPointGraph();
	verifyHierarchy();
}

JumpPointAbstraction::~JumpPointAbstraction()
{
	delete nf;
	delete ef;
}

mapAbstraction*
JumpPointAbstraction::clone(Map* _m)
{
	return new JumpPointAbstraction(_m, nf->clone(), ef->clone());
}

bool 
JumpPointAbstraction::pathable(node* n, node* m)
{
	return true;
}

void
JumpPointAbstraction::verifyHierarchy()
{
	graph* g = getAbstractGraph(0); 

	for(int i=0; i<g->getNumNodes(); i++)
	{
		node* p = g->getNode(i);

		if(p->getNumOutgoingEdges() != 0)
		{
			repairAbstraction();			
			break;
		}

		if(p->getNumIncomingEdges() != 0)
		{
			repairAbstraction();
			break;
		}
	}
}

void
JumpPointAbstraction::removeNode(node* n)
{
	graph* g = getAbstractGraph(0); 

	edge_iterator it = n->getEdgeIter();
	for(edge* e = n->edgeIterNext(it); e != 0; e = n->edgeIterNext(it))
	{
		g->removeEdge(e);
		delete e;
	}
	g->removeNode(n);
	delete n;
}

void 
JumpPointAbstraction::removeEdge(edge *e, unsigned int absLevel)
{
	if(absLevel != 0)
		return;

	graph* g = getAbstractGraph(0);
	g->removeEdge(e);
	delete e;
}

void 
JumpPointAbstraction::addNode(node *n)
{
	graph* g = getAbstractGraph(0);
	g->addNode(n);
}

void 
JumpPointAbstraction::addEdge(edge *e, unsigned int absLevel)
{
	if(absLevel != 0)
		return;

	graph* g = getAbstractGraph(1); 
	g->addDirectedEdge(e);
}

void 
JumpPointAbstraction::repairAbstraction()
{
	if(verbose)
		std::cout << "repairAbstraction"<<std::endl;

	if(getNumAbstractGraphs() > 1)
		return;

	graph* g = getAbstractGraph(0);
	abstractions.pop_back();
	delete g;

	makeJumpPointGraph();
}

// Finds jump point successors for each neighbour in the grid.
// Searches in the direction of each traversable neighbour.
// If there is no jump point successor in a particular direction,
// the array of neighbours is padded with an edge to the current node itself.
void
JumpPointAbstraction::makeJumpPointGraph()
{
	// compute the set of nodes in the graph
	graph* g = makeMapNodes(this->getMap(), nf);
	abstractions.push_back(g);
	OnlineJumpPointLocator jpl(this); 

	// compute jump point neighbours of each node 
	OctileHeuristic heuristic;
	for(int i=0; i < g->getNumNodes(); i++)
	{
		node* n = g->getNode(i);
		for(int j = 1; j<9; j++)
		{
			// connect n to a jump point in every direction
			int nx = n->getLabelL(kFirstData);
			int ny = n->getLabelL(kFirstData+1);

			node* neighbour = 0;
			switch(j)
			{
				case 1:
					neighbour = jpl.findJumpNode(Jump::N, nx, ny, -1, -1);
					break;
				case 2:
					neighbour = jpl.findJumpNode(Jump::NE, nx, ny, -1, -1);
					break;
				case 3:
					neighbour = jpl.findJumpNode(Jump::E, nx, ny, -1, -1);
					break;
				case 4:
					neighbour = jpl.findJumpNode(Jump::SE, nx, ny, -1, -1);
					break;
				case 5:
					neighbour = jpl.findJumpNode(Jump::S, nx, ny, -1, -1);
					break;
				case 6:
					neighbour = jpl.findJumpNode(Jump::SW, nx, ny, -1, -1);
					break;
				case 7:
					neighbour = jpl.findJumpNode(Jump::W, nx, ny, -1, -1);
					break;
				case 8:
					neighbour = jpl.findJumpNode(Jump::NW, nx, ny, -1, -1);
					break;
			}

			edge* e = 0;
			if(neighbour) 
			{
				e = new edge(n->getNum(), neighbour->getNum(),
						heuristic.h(n, neighbour));
			}
			else
			{
				// pad out the array when a neighbour doesn't exist
				// (so we can achieve constant-time lookup of any neighbour) 
				e = new edge(n->getNum(), n->getNum(), 0); 
			}

			g->addDirectedEdge(e);
			if(verbose)
			{
				node* from = g->getNode(e->getFrom());
				node* to = g->getNode(e->getTo());
				std::cout << "jpa edge: ("<< from->getLabelL(kFirstData) << ", "
					<< from->getLabelL(kFirstData+1) << ") -> ("<<
					to->getLabelL(kFirstData)<<","<<
					to->getLabelL(kFirstData+1)<<")"<<
					" cost: "<< e->getWeight() <<std::endl;
			}
		}
	}
}


#include "JumpPointAbstraction.h"

#include "IEdgeFactory.h"
#include "INodeFactory.h"
#include "OnlineJumpPointLocator.h"
#include "graph.h"
#include "map.h"
#include "OctileHeuristic.h"

#include <cstdlib>
#include <climits>
#include <iostream>
#include <fstream>

JumpPointAbstraction::JumpPointAbstraction(Map* _m, INodeFactory* _nf, 
		IEdgeFactory* _ef, bool _verbose) : mapAbstraction(_m), 
	verbose(_verbose)
{
	nf = _nf;
	ef = _ef;

	makeJumpPointGraph();
	verifyHierarchy();
}

JumpPointAbstraction::JumpPointAbstraction(Map* _m, INodeFactory* _nf, 
		IEdgeFactory* _ef, std::string filename, bool _verbose) 
	: mapAbstraction(_m), verbose(_verbose)
{
	nf = _nf;
	ef = _ef;

	importGraph(filename);
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

void
JumpPointAbstraction::importGraph(std::string filename)
{
	// compute the set of nodes in the graph
	graph* g = makeMapNodes(this->getMap(), nf);
	abstractions.push_back(g);

	std::ifstream fin(filename.c_str());
	std::string nextline;
	int line_num=1;

	getline(fin, nextline);
	int begin = nextline.find(std::string("nodes="));
	int end = nextline.find(std::string(" "), begin);
	int intVal = atoi(nextline.substr(begin+6, end).c_str());
	if(intVal==0 || intVal == INT_MAX || intVal == INT_MIN)
	{
		std::cout << "Error importing graph: broken header on line."
			<< line_num << std::endl;
		exit(1);
	}

	line_num++;
	nextline.clear();
	getline(fin, nextline);
	while(fin.good())
	{
		begin = 0;
		end = nextline.find("[")-1;

		if(end <= begin)
		{
			// line doesn't begin with a node identifier
			std::cout << "Error importing graph: cannot find nodeid "
				"on line " << line_num << std::endl;
			exit(1);
		}

		intVal = atoi(nextline.substr(begin, end-begin).c_str());
		if((intVal == 0 && nextline.at(begin) != '0') || intVal == INT_MAX || intVal == INT_MIN)
		{
			// record doesn't have an integral node identifier 
			std::cout << "Error importing graph: invalid nodeid "
				"on line " << line_num << std::endl;
			exit(1);
		}
		int nodeId = intVal;

		// read edge records
		while(1)
		{
			// identify the range of the substring containing the edge record
			end = nextline.find("]", begin);
			begin = nextline.find("[", begin);

			if(begin ==-1 && end == -1)
			{
				// node has no (more) outgoing edges
				break;
			}

			if(begin==-1 || end == -1)
			{
				// invalid edge record
				std::cout << "Error importing graph: invalid edge record on"
					" line " << line_num << std::endl;
				begin = nextline.find(" ", begin+1);
				exit(1);
			}

			// read neighbour id
			int commapos = nextline.find(",", begin);
			intVal = atoi(nextline.substr(begin+1, commapos-(begin+1)).c_str());
			if((intVal==0 && nextline.at(begin+1) != '0')  // a bit buggy
					|| intVal == INT_MAX || intVal == INT_MIN)
			{
				std::cout << "Error importing graph: cannot read neighbourId on "
					"line " << line_num << std::endl;
				begin = nextline.find(" ", begin+1);
				exit(1);
			}
			int neighbourId = intVal;
			assert(begin+1 < end);

			// read edge cost
			begin = commapos;
			double doubleVal = atof(nextline.substr(begin+1, (end-1)-(begin+1)).c_str());
			if((doubleVal == 0 && nextline.at(begin+1) != '0') || doubleVal == HUGE_VAL)
			{
				std::cout << "Error importing graph: cannot read edge cost on "
					"	line " << line_num << std::endl;
				exit(1);
			}

			edge* e = new edge(nodeId, neighbourId, doubleVal);
			int numEdges = g->getNumEdges();
			g->addEdge(e);
			assert(g->getNumEdges() > numEdges);

			begin = end+1;
			end++;
		}
		line_num++;
		nextline.clear();
		getline(fin, nextline);
	}
}


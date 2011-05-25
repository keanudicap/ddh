#include "JumpPointAbstraction.h"

#include "IEdgeFactory.h"
#include "INodeFactory.h"
#include "graph.h"
#include "map.h"
#include "OctileHeuristic.h"

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

void
JumpPointAbstraction::makeJumpPointGraph()
{
	// compute the set of nodes in the graph
	graph* g = makeMapNodes(this->getMap(), nf);
	abstractions.push_back(g);

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
					neighbour = findJumpNode(Jump::N, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::N, nx, ny);
					break;
				case 2:
					neighbour = findJumpNode(Jump::NE, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::NE, nx, ny);
					break;
				case 3:
					neighbour = findJumpNode(Jump::E, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::E, nx, ny);
					break;
				case 4:
					neighbour = findJumpNode(Jump::SE, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::SE, nx, ny);
					break;
				case 5:
					neighbour = findJumpNode(Jump::S, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::S, nx, ny);
					break;
				case 6:
					neighbour = findJumpNode(Jump::SW, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::SW, nx, ny);
					break;
				case 7:
					neighbour = findJumpNode(Jump::W, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::W, nx, ny);
					break;
				case 8:
					neighbour = findJumpNode(Jump::NW, nx, ny);
					if(!neighbour)
						neighbour = findObstacleJumpNode(Jump::NW, nx, ny);
					break;
			}

			edge* e = 0;
			if(neighbour && neighbour->getNum() != n->getNum())
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
				std::cout << "jpa edge: ("<< n->getLabelL(kFirstData) << ", "
					<<n->getLabelL(kFirstData+1) << ") -> ("<<
					neighbour->getLabelL(kFirstData)<<","<<
					neighbour->getLabelL(kFirstData+1)<<")"<<
					" cost: "<<heuristic.h(n, neighbour)<<std::endl;
			}
		}
	}
}

node* 
JumpPointAbstraction::findJumpNode(Jump::Direction d, int x, int y)
{
	const int jumplimit = INT_MAX;

	node* n = this->getNodeFromMap(x, y);

	switch(d)
	{
		case Jump::N:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int ny = y-steps;
				n = this->getNodeFromMap(x, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				// n is a jump node if we cannot prove a shorter path to 
				// a diagonal neighbour exists
				if(!this->getNodeFromMap(x-1, ny) && 
						this->getNodeFromMap(x-1, ny-1))
				{
					break;
				}

				if(!this->getNodeFromMap(x+1, ny) &&
						this->getNodeFromMap(x+1, ny-1))
				{
					break;
				}
				
			}
			break;
		}

		case Jump::S:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int ny = y+steps;
				n = this->getNodeFromMap(x, ny);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				if(!this->getNodeFromMap(x-1, ny) && 
						this->getNodeFromMap(x-1, ny+1))
				{
					break;
				}

				if(!this->getNodeFromMap(x+1, ny) &&
						this->getNodeFromMap(x+1, ny+1))
				{
					break;
				}
			}
			break;
		}

		case Jump::E:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				n = this->getNodeFromMap(nx, y);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				if(!this->getNodeFromMap(nx, y-1) && 
						this->getNodeFromMap(nx+1, y-1))
				{
					break;
				}

				if(!this->getNodeFromMap(nx, y+1) &&
						this->getNodeFromMap(nx+1, y+1))
				{
					break;
				}
			}
			break;
		}

		case Jump::W:
		{

			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps;
				n = this->getNodeFromMap(nx, y);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
					break;
				
				if(!this->getNodeFromMap(nx, y-1) && 
						this->getNodeFromMap(nx-1, y-1))
				{
					break;
				}

				if(!this->getNodeFromMap(nx, y+1) &&
						this->getNodeFromMap(nx-1, y+1))
				{
					break;
				}
			}
			break;
		}

		case Jump::NE:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				int ny = y-steps;
				n = this->getNodeFromMap(nx, ny);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				// n is a jump node if a SE neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!this->getNodeFromMap(nx, ny+1) && 
						this->getNodeFromMap(nx+1, ny+1))
				{
					break;
				}

				// n is a jump node if a NW neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!this->getNodeFromMap(nx-1, ny) && 
						this->getNodeFromMap(nx-1, ny-1))
				{
					break;
				}
			
				// n is a jump node if we can reach other jump nodes by
				// travelling vertically or horizontally 
				if(findJumpNode(Jump::N, nx, ny) || 
						findJumpNode(Jump::E, nx, ny))
					break;
			}
			break;
		}

		case Jump::SE:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				int ny = y+steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				if(!this->getNodeFromMap(nx, ny-1) && 
						this->getNodeFromMap(nx+1, ny-1))
				{
					break;
				}

				if(!this->getNodeFromMap(nx-1, ny) && 
						this->getNodeFromMap(nx-1, ny+1))
				{
					break;
				}

				if(findJumpNode(Jump::S, nx, ny) || 
						findJumpNode(Jump::E, nx, ny))
					break;
			}
			break;
		}

		case Jump::NW:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps; 
				int ny = y-steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				if(!this->getNodeFromMap(nx, ny+1) && 
						this->getNodeFromMap(nx-1, ny+1))
				{
					break;
				}

				if(!this->getNodeFromMap(nx+1, ny) && 
						this->getNodeFromMap(nx+1, ny-1))
				{
					break;
				}

				if(findJumpNode(Jump::N, nx, ny) || 
						findJumpNode(Jump::W, nx, ny))
					break;
			}
			break;
		}

		case Jump::SW:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps;
				int ny = y+steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
					break;

				if(!this->getNodeFromMap(nx, ny-1) && 
						this->getNodeFromMap(nx-1, ny-1))
				{
					break;
				}

				if(!this->getNodeFromMap(nx+1, ny) && 
						this->getNodeFromMap(nx+1, ny+1))
				{
					break;
				}

				if(findJumpNode(Jump::S, nx, ny) || 
						findJumpNode(Jump::W, nx, ny))
					break;
			}
			break;
		}

		default:
			break;
	}

	return n;
}


// Sometimes ::findJumpNode fails to return a jump node because it reaches
// an obstacle. This method returns the node immediately prior to the
// obstacle and designates it a jump point.
node*
JumpPointAbstraction::findObstacleJumpNode(Jump::Direction d, int x, int y)
{
	const int jumplimit = INT_MAX;
	node* n = 0; 

	switch(d)
	{
		case Jump::N:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int ny = y-steps;
				n = this->getNodeFromMap(x, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x, y-(steps-1));
					break;
					
				}
			}
			break;
		}

		case Jump::S:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int ny = y+steps;
				n = this->getNodeFromMap(x, ny);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x, y+(steps-1));
					break;
					
				}
			}
			break;
		}

		case Jump::E:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				n = this->getNodeFromMap(nx, y);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x+(steps-1), y);
					break;
				}
			}
			break;
		}

		case Jump::W:
		{

			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps;
				n = this->getNodeFromMap(nx, y);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x-(steps-1), y);
					break;
				}
			}
			break;
		}

		case Jump::NE:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				int ny = y-steps;
				n = this->getNodeFromMap(nx, ny);
				
				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x+(steps-1), y-(steps-1));
					break;
					
				}
			}
			break;
		}

		case Jump::SE:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x+steps;
				int ny = y+steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x+(steps-1), y+(steps-1));
					break;
					
				}
			}
			break;
		}

		case Jump::NW:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps; 
				int ny = y-steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x-(steps-1), y-(steps-1));
					break;
				}
			}
			break;
		}

		case Jump::SW:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int nx = x-steps;
				int ny = y+steps;
				n = this->getNodeFromMap(nx, ny);

				// the node just before an obstacle is a jump point
				if(n == 0)
				{
					n = this->getNodeFromMap(x-(steps-1), y+(steps-1));
					break;
				}
			}
			break;
		}

		default:
			break;
	}

	return n;
}


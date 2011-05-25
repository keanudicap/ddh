#include "JumpPointsExpansionPolicy.h"

#include "fpUtil.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "ProblemInstance.h"

JumpPointsExpansionPolicy::JumpPointsExpansionPolicy()
	: ExpansionPolicy()
{
	jumplimit = INT_MAX;
	neighbourIndex = 0;
}

JumpPointsExpansionPolicy::~JumpPointsExpansionPolicy()
{
	neighbours.clear();
}


void 
JumpPointsExpansionPolicy::expand(node* t) throw(std::logic_error)
{
	ExpansionPolicy::expand(t);


	neighbours.clear();
	computeNeighbourSet();

	neighbourIndex = 0;
}

void 
JumpPointsExpansionPolicy::computeNeighbourSet()
{
	mapAbstraction* map = problem->getMap();
	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);

	Jump::Direction which = directionToParent();
	switch(which)
	{
		case Jump::N:
		{
			node* n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			// add SE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::SE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::NE:
		{
			node* n = findJumpNode(Jump::SW, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			// add NW neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NW, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add SE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::E:
		{
			node* n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

		 	// add NW neighbour only if N neighbour is null	
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::SE:
		{
			node* n = findJumpNode(Jump::NW, x, y); 
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SW, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add NE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::S:
		{
			node* n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			// add NE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}
			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::SW:
		{
			node* n = findJumpNode(Jump::NE, x, y); 
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}

			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::NW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::W:
		{
			node* n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

		 	// add NE neighbour only if N neighbour is null 
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				n = findJumpNode(Jump::SE, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}

		case Jump::NW:
		{
			node* n = findJumpNode(Jump::SE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			// add NE neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				n = findJumpNode(Jump::NE, x, y); 
				if(n)
					neighbours.push_back(n);
			}

			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				n = findJumpNode(Jump::SW, x, y);
				if(n)
					neighbours.push_back(n);
			}
			break;
		}
		case Jump::NONE:
		{
			graph* g = map->getAbstractGraph(0);
			neighbor_iterator iter = target->getNeighborIter();
			for(int nodeId = target->nodeNeighborNext(iter); 
							nodeId != -1 ;
							nodeId = target->nodeNeighborNext(iter))
			{
					node* n = g->getNode(nodeId);
					//assert(n);
					neighbours.push_back(n);
			}
			
/*			node* n = findJumpNode(Jump::N, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::S, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::E, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::W, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::NE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::NW, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::SE, x, y);
			if(n)
				neighbours.push_back(n);

			n = findJumpNode(Jump::SW, x, y);
			if(n)
				neighbours.push_back(n);
*/
		}
	}
}

node* 
JumpPointsExpansionPolicy::first()
{
	if(neighbours.size() > 0)
		return neighbours.at(0);
	return 0;
}

node* 
JumpPointsExpansionPolicy::next()
{
	node* nextnode = 0;

	if(hasNext())
	{
		neighbourIndex++;
		nextnode = n();
	}

	return nextnode;
}

node* 
JumpPointsExpansionPolicy::n()
{
	node* retVal = 0;
	unsigned int numNeighbours = neighbours.size();
	if(numNeighbours > 0 && neighbourIndex < numNeighbours)
	{
		retVal = neighbours.at(neighbourIndex);
	}
	return retVal;
}

double 
JumpPointsExpansionPolicy::cost_to_n()
{
	node* current = n();
	return problem->getHeuristic()->h(target, current);
}

bool 
JumpPointsExpansionPolicy::hasNext()
{
	if(neighbourIndex+1 < neighbours.size())
		return true;
	return false;
}


// direction from the target (=node being expanded) to its parent
Jump::Direction 
JumpPointsExpansionPolicy::directionToParent()
{
	node* parent = target->backpointer;	
	if(!parent)
		return Jump::NONE;

	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);
	int px = parent->getLabelL(kFirstData);
	int py = parent->getLabelL(kFirstData+1);
	
	if(py == y)
	{
		if(px > x)
			return Jump::E;
		else
			return Jump::W;
	}

	if(py < y)
	{
		if(px < x)
			return Jump::NW;
		else if(px > x)
			return Jump::NE;
		else
			return Jump::N;

	}

	if(py > y)
	{
		if(px < x)
			return Jump::SW;
		else if(px > x)
			return Jump::SE;
		else
			return Jump::S;
	}

	throw std::logic_error("JumpPointsExpansionPolicy::directionToParent"
			" failed to determine direction to parent!");
}

// Finds the nearest jump node neighbour (if any) for the target node
// in a given direction.
//
// @return: a jump node (null  if none is found)
node*
JumpPointsExpansionPolicy::findJumpNode(Jump::Direction d, int x, int y)
{
	mapAbstraction* map = problem->getMap();

//	node* n = 0; // jump node in Direction d
	node* n = map->getNodeFromMap(x, y);

	int goalx = problem->getGoalNode()->getLabelL(kFirstData);
	int goaly = problem->getGoalNode()->getLabelL(kFirstData+1);

	switch(d)
	{
		case Jump::N:
		{
			for(int steps=1; steps <= jumplimit ; steps++)
			{
				int ny = y-steps;
				n = map->getNodeFromMap(x, ny);
				if(n == 0)
					break;

				// (ny == goaly) implies n is a jump node 
//				if(ny == goaly)
				if(x == goalx && ny == goaly)
					break;

				// n is a jump node if we cannot prove a shorter path to 
				// a diagonal neighbour exists
				if(!map->getNodeFromMap(x-1, ny) && 
						map->getNodeFromMap(x-1, ny-1))
				{
					break;
				}

				if(!map->getNodeFromMap(x+1, ny) &&
						map->getNodeFromMap(x+1, ny-1))
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
				n = map->getNodeFromMap(x, ny);
				if(n == 0)
					break;

				//if(ny == goaly)
				if(x == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(x-1, ny) && 
						map->getNodeFromMap(x-1, ny+1))
				{
					break;
				}

				if(!map->getNodeFromMap(x+1, ny) &&
						map->getNodeFromMap(x+1, ny+1))
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
				n = map->getNodeFromMap(nx, y);
				if(n == 0)
					break;

//				if(nx == goalx)
				if(nx == goalx && y == goaly)
					break;

				if(!map->getNodeFromMap(nx, y-1) && 
						map->getNodeFromMap(nx+1, y-1))
				{
					break;
				}

				if(!map->getNodeFromMap(nx, y+1) &&
						map->getNodeFromMap(nx+1, y+1))
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
				n = map->getNodeFromMap(nx, y);
				if(n == 0)
					break;

//				if(nx == goalx)
				if(nx == goalx && y == goaly)
					break;

				if(!map->getNodeFromMap(nx, y-1) && 
						map->getNodeFromMap(nx-1, y-1))
				{
					break;
				}

				if(!map->getNodeFromMap(nx, y+1) &&
						map->getNodeFromMap(nx-1, y+1))
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
				n = map->getNodeFromMap(nx, ny);

				// stop if we hit an obstacle (no jump node exists)
				if(n == 0)
					break;

				// n is jump node if it share a row or column with the goal 
				//if(nx == goalx || ny == goaly)
				if(nx == goalx && ny == goaly)
					break;

				// n is a jump node if a SE neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!map->getNodeFromMap(nx, ny+1) && 
						map->getNodeFromMap(nx+1, ny+1))
				{
					break;
				}

				// n is a jump node if a NW neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!map->getNodeFromMap(nx-1, ny) && 
						map->getNodeFromMap(nx-1, ny-1))
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				//if(nx == goalx || ny == goaly)
				if(nx == goalx && ny == goaly)
					break;
				
				if(!map->getNodeFromMap(nx, ny-1) && 
						map->getNodeFromMap(nx+1, ny-1))
				{
					break;
				}

				if(!map->getNodeFromMap(nx-1, ny) && 
						map->getNodeFromMap(nx-1, ny+1))
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				//if(nx == goalx || ny == goaly)
				if(nx == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(nx, ny+1) && 
						map->getNodeFromMap(nx-1, ny+1))
				{
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny-1))
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				//if(nx == goalx || ny == goaly)
				if(nx == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(nx, ny-1) && 
						map->getNodeFromMap(nx-1, ny-1))
				{
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny+1))
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


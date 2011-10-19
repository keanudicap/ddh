#include "OnlineJumpPointLocator.h"
#include "graph.h"
#include "mapAbstraction.h"


OnlineJumpPointLocator::OnlineJumpPointLocator(mapAbstraction* _map) 
	: JumpPointLocator(_map)
{
	jumplimit = INT_MAX;
}

OnlineJumpPointLocator::~OnlineJumpPointLocator()
{
}


// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a 
// jump point successor.
//
// @return: a jump point successor or 0 if no such jump point exists.
node*
OnlineJumpPointLocator::findJumpNode(Jump::Direction d, int x, int y, 
		int goalx, int goaly)
{
	fringe.clear();

	node* n = 0;
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

				if(x == goalx && ny == goaly)
					break;

				// n is a jump node if we cannot prove a shorter path to 
				// a diagonal neighbour exists
				if(!map->getNodeFromMap(x-1, ny) && 
						map->getNodeFromMap(x-1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(x-1,ny-1));
					break;
				}

				if(!map->getNodeFromMap(x+1, ny) &&
						map->getNodeFromMap(x+1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(x+1,ny-1));
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

				if(x == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(x-1, ny) && 
						map->getNodeFromMap(x-1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(x-1, ny+1));
					break;
				}

				if(!map->getNodeFromMap(x+1, ny) &&
						map->getNodeFromMap(x+1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(x+1, ny+1));
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

				if(nx == goalx && y == goaly)
					break;

				if(!map->getNodeFromMap(nx, y-1) && 
						map->getNodeFromMap(nx+1, y-1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, y-1));
					break;
				}

				if(!map->getNodeFromMap(nx, y+1) &&
						map->getNodeFromMap(nx+1, y+1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, y+1));
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

				if(nx == goalx && y == goaly)
					break;

				if(!map->getNodeFromMap(nx, y-1) && 
						map->getNodeFromMap(nx-1, y-1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, y-1));
					break;
				}

				if(!map->getNodeFromMap(nx, y+1) &&
						map->getNodeFromMap(nx-1, y+1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, y+1));
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
				if(nx == goalx && ny == goaly)
					break;

				// n is a jump node if a SE neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!map->getNodeFromMap(nx, ny+1) && 
						map->getNodeFromMap(nx+1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, ny+1));
					break;
				}

				// n is a jump node if a NW neighbour exists which cannot be
				// reached by a shorter path than one involving n.
				if(!map->getNodeFromMap(nx-1, ny) && 
						map->getNodeFromMap(nx-1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, ny-1));
					break;
				}
			
				// n is a jump node if we can reach other jump nodes by
				// travelling vertically or horizontally 
				node* tmp = findJumpNode(Jump::N, nx, ny, goalx, goaly);
				node* tmp2 = findJumpNode(Jump::E, nx, ny, goalx, goaly);
				if(tmp || tmp2)
				{
					if(tmp) fringe.push_back(tmp);
					if(tmp2) fringe.push_back(tmp2);
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				if(nx == goalx && ny == goaly)
					break;
				
				if(!map->getNodeFromMap(nx, ny-1) && 
						map->getNodeFromMap(nx+1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, ny-1));
					break;
				}

				if(!map->getNodeFromMap(nx-1, ny) && 
						map->getNodeFromMap(nx-1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, ny+1));
					break;
				}

				node* tmp = findJumpNode(Jump::S, nx, ny, goalx, goaly);
				node* tmp2 = findJumpNode(Jump::E, nx, ny, goalx, goaly);
				if(tmp || tmp2)
				{
					if(tmp) fringe.push_back(tmp);
					if(tmp2) fringe.push_back(tmp2);
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				if(nx == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(nx, ny+1) && 
						map->getNodeFromMap(nx-1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, ny+1));
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, ny-1));
					break;
				}

				node* tmp = findJumpNode(Jump::N, nx, ny, goalx, goaly);
				node* tmp2 = findJumpNode(Jump::W, nx, ny, goalx, goaly);
				if(tmp || tmp2)
				{
					if(tmp) fringe.push_back(tmp);
					if(tmp2) fringe.push_back(tmp2);
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
				n = map->getNodeFromMap(nx, ny);
				if(n == 0)
					break;

				if(nx == goalx && ny == goaly)
					break;

				if(!map->getNodeFromMap(nx, ny-1) && 
						map->getNodeFromMap(nx-1, ny-1))
				{
					fringe.push_back(map->getNodeFromMap(nx-1, ny-1));
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny+1))
				{
					fringe.push_back(map->getNodeFromMap(nx+1, ny+1));
					break;
				}

				node* tmp = findJumpNode(Jump::S, nx, ny, goalx, goaly);
				node* tmp2 = findJumpNode(Jump::W, nx, ny, goalx, goaly);
				if(tmp || tmp2)
				{
					if(tmp) fringe.push_back(tmp);
					if(tmp2) fringe.push_back(tmp2);
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


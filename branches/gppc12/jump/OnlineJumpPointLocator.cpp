#include "OnlineJumpPointLocator.h"
#include "graph.h"
#include "mapAbstraction.h"

#include <climits>

OnlineJumpPointLocator::OnlineJumpPointLocator(mapAbstraction* _map) 
	: JumpPointLocator(_map)
{
	jumplimit = INT_MAX;
	cutCorners = false;
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

				// n is a jump node if we cannot prove a symmetric path to 
				// either of two diagonal neighbours exists
				if(!canStep(x, y-(steps-1), Jump::NW) && 
						canStep(x, ny, Jump::NW))
				{
					break;
				}

				if(!canStep(x, y-(steps-1), Jump::NE) && 
						canStep(x, ny, Jump::NE))
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

				if(x == goalx && ny == goaly)
					break;
				
				// n is a jump node if we cannot prove a symmetric path to 
				// either of two diagonal neighbours exists
				if(!canStep(x, y+(steps-1), Jump::SW) && 
						canStep(x, ny, Jump::SW))
				{
					break;
				}

				if(!canStep(x, y+(steps-1), Jump::SE) && 
						canStep(x, ny, Jump::SE))
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

				if(nx == goalx && y == goaly)
					break;
				
				// n is a jump node if we cannot prove a symmetric path to 
				// either of two diagonal neighbours exists
				if(!canStep(x+(steps-1), y, Jump::NE) && 
						canStep(x, ny, Jump::SW))
				{
					break;
				}

				if(!canStep(x+(steps-1), y, Jump::SE) && 
						canStep(x, ny, Jump::SE))
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

				if(nx == goalx && y == goaly)
					break;

				
				// n is a jump node if we cannot prove a symmetric path to 
				// either of two diagonal neighbours exists
				if(!canStep(x-(steps-1), y, Jump::NW) && 
						canStep(x, ny, Jump::SW))
				{
					break;
				}

				if(!canStep(x-(steps-1), y, Jump::SW) && 
						canStep(x, ny, Jump::SE))
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
				if(canStep(x+(steps-1), y-(steps-1), Jump::NE))
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
				if(findJumpNode(Jump::N, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::E, nx, ny, goalx, goaly))
				{
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
					break;
				}

				if(!map->getNodeFromMap(nx-1, ny) && 
						map->getNodeFromMap(nx-1, ny+1))
				{
					break;
				}

				if(findJumpNode(Jump::S, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::E, nx, ny, goalx, goaly))
				{
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
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny-1))
				{
					break;
				}

				if(findJumpNode(Jump::N, nx, ny, goalx, goaly) || 
					findJumpNode(Jump::W, nx, ny, goalx, goaly))
				{
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
					break;
				}

				if(!map->getNodeFromMap(nx+1, ny) && 
						map->getNodeFromMap(nx+1, ny+1))
				{
					break;
				}

				if(findJumpNode(Jump::S, nx, ny, goalx, goaly) ||
					findJumpNode(Jump::W, nx, ny, goalx, goaly))
				{
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

bool
OnlineJumpPointLocator::canStep(int x, int y, Jump::Direction checkdir)
{
	assert(checkdir != Jump::NONE);
	if(!map->getNodeFromMap(x, y))
		return false;

	bool retVal = true;
	switch(checkdir)
	{
		case Jump::N:
			if(!map->getNodeFromMap(x, y-1))
				retVal =  false;
			retVal =  true;
			break;

		case Jump::S:
			if(!map->getNodeFromMap(x, y+1))
				retVal =  false;
			retVal =  true;
			break;

		case Jump::E:
			if(!map->getNodeFromMap(x+1, y))
				retVal =  false;
			retVal =  true;
			break;

		case Jump::W:
			if(!map->getNodeFromMap(x-1, y))
				retVal =  false;
			retVal =  true;
			break;

		case Jump::S:
			if(!map->getNodeFromMap(x+1, y-1))
				retVal =  false;
			retVal =  true;
			break;

		case Jump::NE:
			if(!map->getNodeFromMap(x+1, y-1))
				retVal =  false;
			if( cutCorners )
			{
				if( !map->getNodeFromMap(x+1, y) && 
					!map->getNodeFromMap(x, y-1) )
				{
					retVal =  false;
				}
			}
			else
			{
				if( !map->getNodeFromMap(x+1, y) ||
					!map->getNodeFromMap(x, y-1) )
				{
					retVal =  false;
				}
			}

			retVal =  true;
			break;

		case Jump::SE:
			if(!map->getNodeFromMap(x+1, y+1))
				retVal =  false;
			if( cutCorners )
			{
				if( !map->getNodeFromMap(x+1, y) && 
					!map->getNodeFromMap(x, y+1) )
				{
					retVal =  false;
				}
			}
			else
			{
				if( !map->getNodeFromMap(x+1, y) ||
					!map->getNodeFromMap(x, y+1) )
				{
					retVal =  false;
				}
			}

			retVal =  true;
			break;

		case Jump::NW:
			if(!map->getNodeFromMap(x-1, y-1))
				retVal =  false;
			if( cutCorners )
			{
				if( !map->getNodeFromMap(x-1, y) && 
					!map->getNodeFromMap(x, y-1) )
				{
					retVal =  false;
				}
			}
			else
			{
				if( !map->getNodeFromMap(x-1, y) ||
					!map->getNodeFromMap(x, y-1) )
				{
					retVal =  false;
				}
			}

			retVal =  true;
			break;

		case Jump::SW:
			if(!map->getNodeFromMap(x-1, y+1))
				retVal =  false;
			if( cutCorners )
			{
				if( !map->getNodeFromMap(x-1, y) && 
					!map->getNodeFromMap(x, y+1) )
				{
					retVal =  false;
				}
			}
			else
			{
				if( !map->getNodeFromMap(x-1, y) ||
					!map->getNodeFromMap(x, y+1) )
				{
					retVal =  false;
				}
			}

			retVal =  true;
			break;


		case Jump::NONE:
			retVal = false;
	}

	return retVal;
}

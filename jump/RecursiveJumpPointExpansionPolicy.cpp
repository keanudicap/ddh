#include "RecursiveJumpPointExpansionPolicy.h"

#include "JumpPointLocator.h"
#include "fpUtil.h"
#include "Heuristic.h"
#include "mapAbstraction.h"
#include "ProblemInstance.h"

#include <limits.h>

RecursiveJumpPointExpansionPolicy::JumpParams::JumpParams() : 
	origin(0), first_dir(Jump::NONE), last_dir(Jump::NONE), succ(0),
	jumpcost(0), depth(0)
{ 
}

RecursiveJumpPointExpansionPolicy::JumpParams::~JumpParams()  
{ 
}

RecursiveJumpPointExpansionPolicy::JumpParams::JumpParams(
		JumpParams& other)
{
	this->origin = other.origin;
	this->first_dir = other.first_dir;
	this->last_dir = other.last_dir;
	this->succ = other.succ;
	this->jumpcost = other.jumpcost;
	this->depth = other.depth;
}

RecursiveJumpPointExpansionPolicy::RecursiveJumpPointExpansionPolicy(
		JumpPointLocator* _jpl) : ExpansionPolicy()
{
	neighbourIndex = 0;
	jpl = _jpl; 

	// some values to limit recursion
	MAX_NEIS = 8;
	MAX_DEPTH = 3;
	MAX_NODES = pow(MAX_NEIS, MAX_DEPTH);
	ONE_OVER_MAX_NEIS = 1/((double)MAX_NEIS);
	
	// all_dirs is an array of bitfields packed as follows:
	// 0-7: the set of directions in which to search for jump point successors.
	// 8-15: the set of directions in which a jump point successor was found.
	// 16-23: travel direction (immediately preceeding jump point to here).
	all_dirs = new int[MAX_NODES]; 
	all_nodes = new node*[MAX_NODES]; // all nodes up to depth [MAX_DEPTH]
	all_costs = new double[MAX_NODES]; // gcosts to all nodes
}

RecursiveJumpPointExpansionPolicy::~RecursiveJumpPointExpansionPolicy()
{
	for(int i=0; i < neighbours.size(); i++)
	{
		delete neighbours.at(i);
	}
	neighbours.clear();
	delete all_dirs;
	delete all_nodes;
	delete all_costs;
	delete jpl;
}


void 
RecursiveJumpPointExpansionPolicy::expand(node* t) throw(std::logic_error)
{
	// clear any data from previous expansion operations
	for(int i=0; i < neighbours.size(); i++)
	{
		delete neighbours.at(i);
	}
	neighbours.clear();
	neighbourIndex = 0;

	// assign a new current node and identify its neighbours
	ExpansionPolicy::expand(t);
	computeNeighbourSet();

}

// Computes the set of jump point neighbours for the target node which is being
// expanded. If the target node has only a single successor, we do not generate
// it. Instead, we immediately jump again in a recursive fashion, each time
// stepping in the direction of the single successor until we identify a jump
// point with a branching factor >= 2. When we find such a jump point, we
// generate it as a successor of the target node.
// 
// NB: it is possible that after some number of recursive steps we eventually
// reach a node that has zero jump point successors. In this case we conclude
// that the target node is sterile -- i.e. it cannot appear on the optimal path
// to the goal.
//
void 
RecursiveJumpPointExpansionPolicy::computeNeighbourSet()
{
	mapAbstraction* map = problem->getMap();
	int x = target->getLabelL(kFirstData);
	int y = target->getLabelL(kFirstData+1);

	// Retrieve the direction of travel used to reach the current node.
	Jump::Direction which = getDirection(this->target); 

	// Look for jump point successors in the same direction
	switch(which)
	{
		case Jump::S:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add SE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				which = Jump::SE;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}
			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				which = Jump::SW;	
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::SW:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::S;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::W;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add NW neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				which = Jump::NW;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}

			// add SE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				which = Jump::SE;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::W:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

		 	// add NW neighbour only if N neighbour is null	
			if(!map->getNodeFromMap(x, y-1))
			{
				which = Jump::NW;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				which = Jump::SW;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::NW:
		{
			JumpParams params;
			findJumpNode(which, params); 
			if(params.succ)
				addNeighbour(params);

			which = Jump::N;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::W;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add SW neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				which = Jump::SW;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}

			// add NE neighbour only if E neighbour is null
			if(!map->getNodeFromMap(x+1, y))
			{
				which = Jump::NE;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::N:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add NE neighbour only if E neighbour is null 
			if(!map->getNodeFromMap(x+1, y))
			{
				which = Jump::NE;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}
			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				which = Jump::NW;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::NE:
		{
			JumpParams params;
			findJumpNode(which, params); 
			if(params.succ)
				addNeighbour(params);

			which = Jump::N;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::E;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				which = Jump::SE;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}

			// add NW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				which = Jump::NW;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::E:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

		 	// add NE neighbour only if N neighbour is null 
			if(!map->getNodeFromMap(x, y-1))
			{
				which = Jump::NE;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			// add SE neighbour only if S neighbour is null
			if(!map->getNodeFromMap(x, y+1))
			{
				which = Jump::SE;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}

		case Jump::SE:
		{
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::S;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::E;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			// add NE neighbour only if N neighbour is null
			if(!map->getNodeFromMap(x, y-1))
			{
				which = Jump::NE;
				findJumpNode(which, params); 
				if(params.succ)
					addNeighbour(params);
			}

			// add SW neighbour only if W neighbour is null
			if(!map->getNodeFromMap(x-1, y))
			{
				which = Jump::SW;
				findJumpNode(which, params);
				if(params.succ)
					addNeighbour(params);
			}
			break;
		}
		case Jump::NONE:
		{
			// when a node has no parent (usually only the start node)
			// we generate jump point successors in every traversable direction
			which = Jump::N;
			JumpParams params;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::S;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::E;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::W;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::NE;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::NW;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::SE;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);

			which = Jump::SW;
			findJumpNode(which, params);
			if(params.succ)
				addNeighbour(params);
		}
	}
}

node* 
RecursiveJumpPointExpansionPolicy::first()
{
	neighbourIndex = 0;
	return n();
}

node* 
RecursiveJumpPointExpansionPolicy::next()
{
	if(neighbourIndex < neighbours.size())
	{
		neighbourIndex++;
	}
	return n();
}

node* 
RecursiveJumpPointExpansionPolicy::n()
{
	unsigned int numNeighbours = neighbours.size();
	if(neighbourIndex < numNeighbours)
	{
		n_ = neighbours.at(neighbourIndex)->succ;
		assert(n_);
	}
	else
	{
		n_ = 0;
	}
	return n_;
}

double 
RecursiveJumpPointExpansionPolicy::cost_to_n()
{
	double retVal = DBL_MAX;
	if(neighbourIndex < neighbours.size())
	{
		retVal = neighbours.at(neighbourIndex)->jumpcost;
	}
	return retVal;
}

bool 
RecursiveJumpPointExpansionPolicy::hasNext()
{
	if(neighbourIndex+1 < neighbours.size())
		return true;
	return false;
}

// Find the next jump point reachable from (x, y) in direction d. This 
// operation involves running a depth-limited search with ::target as the root.
// At each step, we identify child nodes by searching for jump points in the 
// direction of each forced and natural neighbour.
// The search follows each branch down to some maximum depth, trying to
// determine if the subtree leads to a dead-end or to a branching node which
// has > 1 jump point successors.
// If we reach a dead-end, the branch is marked sterile as it leads nowhere.
// If we reach a branching node, or hit the depth limit, the branch is kept.
// The chosen successor of (x, y) is the first branching node encountered when
// walking the generated search tree.
//
// TODO: 
// 1. Generating a search tree here is less efficient than searching in
// a graph. For example: a node may appear multiple times with different
// costs. Also, we rely on MAX_DEPTH to break cycles contained in the map.
// 2. Could identify the jump point successor during search instead of walking
// the tree at the end as is currently done.
//
// @param dir: the direction to jump in, relative to ::target
// @param out: output parameter containing a description of the branching jump 
// node successor of ::target (if any such node was found).
//
void
RecursiveJumpPointExpansionPolicy::findJumpNode(Jump::Direction dir, JumpParams& out)
{
	// initialise the return value
	out.origin = this->target;
	out.first_dir = dir;
	out.last_dir = dir;
	out.jumpcost = 0;
	out.succ = 0;
	out.depth = 0;

	// can't jump past the goal node
	if(this->target->getUniqueID() == 
			problem->getGoalNode()->getUniqueID())
		return;

	// clear all junk from last time
	for(int i=0; i < MAX_NODES; i++)
	{
		all_nodes[i] = 0;
		all_dirs[i] = 0;
		all_costs[i] = DBL_MAX;
	}

	// generate the root node
	all_nodes[0] = this->target;
	if(MAX_DEPTH > 0)
	{
		all_dirs[0] = jpl->computeSuccessors(dir, 
				this->target->getLabelL(kFirstData), 
				this->target->getLabelL(kFirstData+1));
	}
	all_dirs[0] |= (dir << 16);

	// perform a depth-limited search to identify the first branching jump point
	// after ::target in travel direction 'dir'
	node* goal = problem->getGoalNode();
	int goalx = goal->getLabelL(kFirstData);
	int goaly = goal->getLabelL(kFirstData+1);
	int goalid = goal->getUniqueID();
	int cindex = 0;
	int depth = 0; 
	while(all_dirs[0] & 255) // while root has unexplored subtrees
	{
		assert(depth >= 0);
		assert(current);
		node* current = all_nodes[cindex];

		// backtrack; maximum depth reached or goal node reached
		if( depth == this->MAX_DEPTH || 
			current->getUniqueID() == goalid)
		{
			// add this node to the list of found jump points for the parent.
			int pindex = ((cindex-1)*ONE_OVER_MAX_NEIS); 
			int travel_dir = all_dirs[cindex] >> 16;
			all_dirs[pindex] |= (travel_dir << 8);
			assert( !(travel_dir & (travel_dir-1)) );  // single direction only

			cindex = pindex;
			depth--;
			continue;
		}

		// count number of jump point successors reached from the current node
		// (pointless exploring subtree further if branching factor > 1)
		int num_found = 0;
		const int found_dirs = (all_dirs[cindex] >>8) & 255;
		if(found_dirs)
		{
			for(int i=1; i < 256; i = i << 1)
			{
				if(i & found_dirs)
					num_found++;
			}
		}

		// backtrack; explored the entire subtree or reached a branching node 
		if(!(all_dirs[cindex] & 255) || num_found > 1)
		{
			// if we found a jump point in the subtree, add this node as
			// a jump point of the parent
			int pindex = ((cindex-1)*ONE_OVER_MAX_NEIS); 
			int travel_dir = all_dirs[cindex] >> 16;
			assert( !(travel_dir & (travel_dir-1)) );  // single direction only
			if(found_dirs)
			{
				all_dirs[pindex] |= (travel_dir << 8);
			}

			cindex = pindex;
			depth--;
			continue;
		}

		// branch; search for jump points reachable from the current node
		int search_dirs = all_dirs[cindex] & 255;
		int nextdir = 1;
		int next_index = 0;
		node* next = 0;
		while(search_dirs && !next)
		{
			// get one of the remaining search directions
			if(nextdir & search_dirs)
			{
				int offset = -1;
				if(nextdir == Jump::N)
					offset = 0;
				else if(nextdir == Jump::S)
					offset = 1;
				else if(nextdir == Jump::E)
					offset = 2;
				else if(nextdir == Jump::W)
					offset = 3;
				else if(nextdir == Jump::NE)
					offset = 4;
				else if(nextdir == Jump::SE)
					offset = 5;
				else if(nextdir == Jump::SW)
					offset = 6;
				else if(nextdir == Jump::NW)
					offset = 7;
				assert(offset != -1);

				// try to find a jump point
				int cx = current->getLabelL(kFirstData);
				int cy = current->getLabelL(kFirstData+1);	
				next = jpl->findJumpNode((Jump::Direction)nextdir, cx, cy, goalx, goaly);
				next_index = cindex*MAX_NEIS + offset + 1;
				assert(!all_nodes[next_index]);
				all_nodes[next_index] = next;

				// remove nextdir from remaining search directions
				all_dirs[cindex] &= !nextdir; 
			}
			nextdir  = nextdir << 1;
		}

		// branch; found a jump point successor
		if(next)
		{
			// record travel direction to new successor
			all_dirs[next_index] = nextdir << 16;

			// record directions to successor's forced and natural neighbours
			all_dirs[next_index] |= jpl->computeSuccessors((Jump::Direction)nextdir,
					next->getLabelL(kFirstData),
					next->getLabelL(kFirstData+1));

			cindex = next_index;
			depth++;
		}
	}

	// extract a path from the root to the first node with > 1 found_dirs
	cindex  = 0;
	node* succ = 0;
	double cost = 0;
	int found_dirs = (all_dirs[cindex] >> 8) & 255;
	while(found_dirs)
	{
		// found_dirs represents only a single successor; jump further
		if(!(found_dirs & (found_dirs-1))) 
		{
			int offset = -1;
			if(found_dirs == Jump::N)
				offset = 0;
			else if(found_dirs == Jump::S)
				offset = 1;
			else if(found_dirs == Jump::E)
				offset = 2;
			else if(found_dirs == Jump::W)
				offset = 3;
			else if(found_dirs == Jump::NE)
				offset = 4;
			else if(found_dirs == Jump::SE)
				offset = 5;
			else if(found_dirs == Jump::SW)
				offset = 6;
			else if(found_dirs == Jump::NW)
				offset = 7;
			assert(offset != -1);
			int next_index = cindex*MAX_NEIS + offset + 1;
			cost += problem->getHeuristic()->h(
					all_nodes[cindex], all_nodes[next_index]);
			cindex = next_index;
			found_dirs = (all_dirs[cindex] >> 8) & 255;
		}
		// found_dirs represents multiple successors; stop here
		else
		{
			succ = all_nodes[cindex];
			break;
		}
	}

	if(succ)
	{
		out.origin = this->target;
		out.first_dir = dir;
		out.last_dir = (Jump::Direction)(all_dirs[cindex] >> 16);
		out.jumpcost = cost;
		out.depth = depth;
		assert(!(out.last_dir & (out.last_dir-1)) && out.last_dir < 256);
	}
	
	return;
} 

// Updates the direction and parent labels the current neighbour 
// (as returned by ::n()). This operation is called by an external search 
// algorithm when a new best path to node n is found with parent(n) = ::target.
// NB: Direction tracking is necessary to support recursive jumping.
void
RecursiveJumpPointExpansionPolicy::label_n()
{
	assert(neighbourIndex < neighbours.size());
	JumpParams* nparams = neighbours.at(neighbourIndex);
	nparams->succ = this->target; // ::n() == nparams->succ
	directions[nparams->succ->getUniqueID()] = nparams->last_dir;
	nparams->succ->backpointer = this->target;
}

// Returns the travel direction used to reach a given node n. If
// the node has not previously been encountered (e.g. the start node)
// Jump::NONE is is returned.
//
// NB: Unlike non-recursive jumping, there is no guarantee that p(n) can
// be reached by travelling in the opposite direction to the return value.
Jump::Direction 
RecursiveJumpPointExpansionPolicy::getDirection(node* n)
{
	DirectionList::iterator it = directions.find(n->getUniqueID());
	if(it == directions.end())
		return Jump::NONE;
	return (*it).second;
}

void
RecursiveJumpPointExpansionPolicy::addNeighbour(JumpParams& out)
{
	// NB: no need to track best parent; let the search do that and change
	// direction labels only on ::label_n()
	assert(out.succ);
	assert(out.jumpcost > 0);
	JumpParams* myparams = new JumpParams(out);
	neighbours.push_back(myparams);
}

// Returns all intermediate jump nodes between ::target and ::n(). 
// @param out: node* array where pointers to each intermediate node are stored
// @param size: the number of intermediate nodes that should be stored.
// 				set this to ::MAX_DEPTH to guarantee all nodes are written.
//
// 	NB: Neither ::target nor ::n() are written to @param out.
void
RecursiveJumpPointExpansionPolicy::getIntermediateNodes(node** out, int size)
{
	int cindex  = 0;  
	for(int i=0; i < size && i < MAX_DEPTH; i++)
	{
		int found_dirs = (all_dirs[cindex] >> 8) & 255;
		if(!(found_dirs & (found_dirs-1))) 
		{ 
			int offset = -1;
			if(found_dirs == Jump::N)
				offset = 0;
			else if(found_dirs == Jump::S)
				offset = 1;
			else if(found_dirs == Jump::E)
				offset = 2;
			else if(found_dirs == Jump::W)
				offset = 3;
			else if(found_dirs == Jump::NE)
				offset = 4;
			else if(found_dirs == Jump::SE)
				offset = 5;
			else if(found_dirs == Jump::SW)
				offset = 6;
			else if(found_dirs == Jump::NW)
				offset = 7;
			assert(offset != -1);

			int next_index = cindex*MAX_NEIS + offset + 1;
			out[i] = all_nodes[next_index];
			cindex = next_index;
		}
		else // reached ::n() at some level before MAX_DEPTH; stop looping.
		{
			assert(all_nodes[cindex]->getUniqueID() == n()->getUniqueID());
			break;
		}
	}
}

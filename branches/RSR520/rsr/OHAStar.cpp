#include "OHAStar.h"

#include "graph.h"
#include "MacroNode.h"
#include "mapAbstraction.h"
#include "EmptyClusterAbstraction.h"
#include "map.h"
#include "MacroEdge.h"
#include "MacroNode.h"
#include "timer.h"

#include <typeinfo>
#include <cfloat>

OHAStar::OHAStar()
{
	cardinal = false;
}

OHAStar::~OHAStar()
{
}

// TODO: if s+g are low level, get their abstract parents and search on those
// fail otherwise?.
path* OHAStar::getPath(graphAbstraction *_aMap, node *from, node *to, reservationProvider *rp)
{
	visitedClusters.clear();
	assert(visitedClusters.size() == 0);
	Timer t;
	t.startTimer();

	EmptyClusterAbstraction *aMap = dynamic_cast<EmptyClusterAbstraction*>(_aMap);
	assert(aMap);
	MacroNode* mfrom = dynamic_cast<MacroNode*>(from);
	MacroNode* mto = dynamic_cast<MacroNode*>(to);

	if(!mfrom || !mto)
	{
		std::cout << "OHA* failed; start or goal not of type MacroNode";
		std::cout << "start type: "<<typeid(from).name()<<" goal type: ";
		std::cout << typeid(to).name() <<std::endl;
		return 0;
	}

	aMap->verifyClusters();

	// abstract or low-level search? this decision affects the behaviour of ::relaxEdge
	if(mfrom->getLabelL(kAbstractionLevel) == 0 || 
			mto->getLabelL(kAbstractionLevel) == 0)
	{
		if(verbose)
			std::cout << "inserting s+g into abstract graph"<<std::endl;
		aMap->insertStartAndGoalNodesIntoAbstractGraph(mfrom, mto);
		//int mfp = mfrom->getLabelL(kParent);
		//int mtp = mto->getLabelL(kParent);
		//std::cout << "parent ids: "<<mfp<<" "<<mtp<<std::endl;

		graph* g = aMap->getAbstractGraph(1);
		mfrom = dynamic_cast<MacroNode*>(g->getNode(mfrom->getLabelL(kParent)));
		mto = dynamic_cast<MacroNode*>(g->getNode(mto->getLabelL(kParent)));
		assert(mfrom && mto);
	}

	mfrom->setMacroParent(mfrom);
	double insertTime = t.endTimer();

	if(verbose)
	{
		std::cout << "new problem;";
		std::cout << "start: ";
		mfrom->Print(std::cout);	
		std::cout << "goal: ";
		mto->Print(std::cout);
		std::cout << std::endl;
	}

	path* p = ClusterAStar::getPath(aMap, mfrom, mto, rp);	

	t.startTimer();
	p = refinePath(p);
	aMap->removeStartAndGoalNodesFromAbstractGraph();
	aMap->verifyClusters();

	double refineTime = t.endTimer();
	searchTime+=insertTime + refineTime;

	return p;
}

// if ::cardinal == true, edges with non-integer costs will not be evaluated during search.
// i.e. we pretend the graph has no diagonal edges (characteristic of a 4-connected graph).
/*bool OHAStar::evaluate(node* _current, node* _target, edge* e)
{
	MacroNode* current = dynamic_cast<MacroNode*>(_current);;
	MacroNode* target = dynamic_cast<MacroNode*>(_target);;

	bool retVal = ClusterAStar::evaluate(current, target, e);

	if(cardinal)
	{
		if(e->getWeight() != (int)e->getWeight())
			return false;
		if(e->getWeight() > 2 && !retVal)
			std::cerr << "found a macro edge with non-integer costs! wtf?";
	}

	if(e->getWeight() > 1.5)
	{
		if(verbose)
		{
	//		std::cout << "e is macro...";

		}
	}

	if(dynamic_cast<MacroEdge*>(e))
	{
	//	if(verbose)
	//		std::cout << " dynamic cast successful... ";

		if(&*current->getMacroParent() == &*current )
		{
			retVal = true;
	//		if(verbose)
	//			std::cout << " node is mp. return true"<<std::endl;
		}
		else
		{
			retVal = false;		
	//		if(verbose)
	//			std::cout<<" node is not mp. return false"<<std::endl;
		}
	}
	//else
	//	if(e->getWeight() > 1.5 && verbose)
	//		std::cout << " dynamic cast failed. "<<std::endl;

	return retVal;
}
*/

// relaxEdge keeps track of the shortest path to each node.
// the method is called every time a node is generated.
// See Intro to Algorithms (Cormen et al) for the canonical version.
//
// This version is specifically tailored for pathfinding within empty rooms.
// We make sure each room traversal is optimal by keeping track of the distance
// to a 'macro parent' -- which is the ancestor node that was first expanded in
// the current cluster. See [Harabor & Botea, 2010] for details
//
// @param: openList - a binary heap priority queue
// @param: g - abstract graph
// @param: e - the edge being considered
// @param: fromId - the Id of the node being expanded
// @param: toId - the Id of the node being generated
/*void OHAStar::relaxEdge(heap *openList, graph *g, edge *e, int fromId, 
		int toId, node *goal)
{
	MacroNode* from = dynamic_cast<MacroNode*>(g->getNode(fromId));
	MacroNode* to = dynamic_cast<MacroNode*>(g->getNode(toId));
	assert(from && to);

	double f_to = DBL_MAX; 
	MacroNode* mp = from->getMacroParent(); // macro parent 

	// if 'from' has a macro parent, we relax the edge with respect to its macro parent
	// otherwise, standard A* relaxation is used
	if(mp == 0) 
	{
		double g_from = from->getLabelF(kTemporaryLabel) - h(from, goal);
		f_to = g_from + e->getWeight() + h(to, goal);
		//double g_from = from->getLabelF(kTemporaryLabel) ;
		//f_to = g_from + e->getWeight() ;
	}
	else
	{
		if(from->getParentClusterId() == to->getParentClusterId())
		{
			double g_mp = mp->getLabelF(kTemporaryLabel) - h(mp, goal); 
			f_to = g_mp + h(mp, to)  + h(to, goal); // NB: h(mp, to) is exact
			//double g_mp = mp->getLabelF(kTemporaryLabel) ; 
			//f_to = g_mp + h(mp, to)  ; // NB: h(mp, to) is exact
		}
		else
		{
			mp = to;
			double g_from = from->getLabelF(kTemporaryLabel) - h(from, goal);
			f_to = g_from + e->getWeight() + h(to, goal);
			//double g_from = from->getLabelF(kTemporaryLabel) ;
			//f_to = g_from + e->getWeight() ;
		}
	}


	// minimise number of macro parents
	if(f_to == to->getLabelF(kTemporaryLabel) && &*to->getMacroParent() == &*to)
	{
		//if(verbose)
		//{
		//	std::cout << "revoking macro parent status of "<<to->getName();
		//	std::cout << "new parent: "<<from->getName();
		//}
		if(mp->getParentClusterId() == to->getParentClusterId())
			to->setMacroParent(mp);
	}

	// update priority and macro parent if necessary 
	if(f_to < to->getLabelF(kTemporaryLabel))
	{
		//if(verbose)
		//{
		//	std::cout << "\t\trelaxing "<<to->getName()<<" old priority: ";
		//	std::cout << to->getLabelF(kTemporaryLabel)<<" new priority: "<<f_to;
		//	std::cout << " g: "<<f_to-h(to, goal)<<" h: "<<h(to, goal)<<std::endl;
		//}
		//if(verbose)
		//{
		//	std::cout << " relaxing "<<to->getName()<<" from: "<<from->getName()<< " f(to): "<<f_to;
		//	if(!mp)
		//		std::cout << " no mp!"<<std::endl;
		//	else
		//	{
		//		double g_mp = mp->getLabelF(kTemporaryLabel) - h(mp, goal); 
		//		std::cout << " fmp: "<<mp->getLabelF(kTemporaryLabel)<<" hmp: "<<h(mp, goal);
		//		std::cout << " gmp: "<<g_mp<< " h(to, goal): "<<h(to, goal);
		//		std::cout << " mp(to) = "<<mp->getName()<<std::endl;
		//	}
		//}

		to->setLabelF(kTemporaryLabel, f_to);
		openList->decreaseKey(to);
		to->markEdge(e);
		to->setMacroParent(mp);
	}
//	else
//		if(verbose)
//			std::cout << " not relaxing!"<<std::endl;
}
*/

// Extracts the optimal path once the goal is found.
// Unlike the canonical implementation, which simply follows marked edges
// from the goal to the start node, this version alternates between following
// marked edges and macro parents.
// Each macro parent/sibling pair are the endpoints of two nodes on the
// optimal path that enter and exit an empty room.
// Meanwhile, each marked edge is an transition from one empty room to another 
// along the optimal path
//
// @param: g - the search graph
// @param: current - the id of the goal node
path* OHAStar::extractBestPath(graph *g, unsigned int current)
{
	path* p = 0;
	edge* e = 0;

	while(1)
	{
		MacroNode* cn = dynamic_cast<MacroNode*>(g->getNode(current));
		assert(cn);
		p = new path(cn, p);
		if(verbose)
			std::cout << current <<"cn: "<<cn->getName();

		MacroNode* mp = cn->getMacroParent();
		if(mp)
		{
			if(cn->getNum() != mp->getNum())
			{
				current = mp->getNum();
				if(verbose)
					std::cout << " following macro parent "<<mp->getName()<<std::endl;
			}
			else
			{
				e = cn->getMarkedEdge();
				if(e != 0)
				{
					current = e->getFrom() == current?e->getTo():e->getFrom();
					if(verbose)
						std::cout << " following marked edge"<<std::endl;
				}
				else
				{
					// cn must be the start node (cannot follow macro parent or marked edge)
					if(verbose)
						std::cout << " is start! finished!"<<std::endl; 
					break;
				}
			}
		}
		else
		{
				// special case for searching on the non-abstracted graph
				// where none of the nodes have macro parents
				e = cn->getMarkedEdge();
				if(e == 0)
					break;
				current = e->getFrom() == current?e->getTo():e->getFrom();

				if(verbose)
					std::cout << current <<" <- ";
		}
	}

	return p;
}

// refines an abstract path; each abstract path is made up of a sequence of connected
// fragments where each fragment has a start and goal
path* OHAStar::refinePath(path* _abspath)
{
	mapAbstraction* aMap = dynamic_cast<mapAbstraction*>(getGraphAbstraction());
	path* thepath = 0;
	path* tail = 0;
	path* abspath = _abspath;

	while(abspath->next)
	{
		if(verbose)
		{
			std::cout << "refining segment: "<<abspath->n->getName()<<", ";
			std::cout <<abspath->next->n->getName()<<std::endl;
		}

		node *fs = aMap->getNodeFromMap(abspath->n->getLabelL(kFirstData), 
				abspath->n->getLabelL(kFirstData+1));
		assert(fs);
		node* fg = aMap->getNodeFromMap(abspath->next->n->getLabelL(kFirstData), 
				abspath->next->n->getLabelL(kFirstData+1));
		assert(fg);


		path* segment = new path(fs, 0);
		path* segtail = segment;
		while(segtail->n->getNum() != fg->getNum())
		{
			node* n = closestNeighbour(segtail->n, fg);
			segtail->next = new path(n, 0);
			segtail = segtail->next;
		}
		if(verbose)
			std::cout << std::endl;
			
		// append segment to refined path
		if(thepath == 0)
		{
			thepath = segment;										
		}
		else
		{
			// successive segments overlap (one finishes where other begins); fix this 
			assert(tail->n->getNum() == segment->n->getNum());
			tail->next = segment->next;
			segment->next = 0;
			delete segment;
		}
		tail = segtail;
		abspath = abspath->next;			
	}

	delete _abspath;
	return thepath;
}

// use ::h to evaluate all neighbours of 'from' with respect to their distance to 'to'
// return the neighbour with the lowest value (i.e. the one closest to 'to')
node* OHAStar::closestNeighbour(node* from, node* to)
{	
	graph* g = getGraphAbstraction()->getAbstractGraph(0);
	double mindist = DBL_MAX;
	node* closest = 0;

	neighbor_iterator niter = from->getNeighborIter();
	int nid = from->nodeNeighborNext(niter);
	while(nid != -1)
	{
		node* n = g->getNode(nid);
		double hdist = h(from, n) + h(n, to);
		if(hdist < mindist)
		{
			closest = n;
			mindist = hdist;
		}
		nid = from->nodeNeighborNext(niter);
	}

	return closest;
}

// expand all nodes along the perimeter of the current rectangle
void 
OHAStar::expand(node* current_, node* goal_, edge_iterator iter, unsigned int card, 
		heap* openList, std::map<int, node*>& closedList, graph* g)
{
	AbstractClusterAStar::expand(current_, goal_, iter, card, openList, closedList, g);
	expandMacro(current_, goal_, openList, closedList, g);
}

void 
OHAStar::expandMacro(node* current_, node* goal_, heap* openList, 
		std::map<int, node*>& closedList, graph* g)
{

	MacroNode* current = dynamic_cast<MacroNode*>(current_);
	assert(current);
	MacroNode* goal = dynamic_cast<MacroNode*>(goal_);
	assert(goal);

	if(verbose)
	{
		printNode(std::string("expandMacro..."),current_);
		std::cout << " in cluster "<<current->getParentClusterId() << std::endl;
	}

	EmptyClusterAbstraction* ecmap = dynamic_cast<EmptyClusterAbstraction*>(getGraphAbstraction());
	assert(ecmap);

	EmptyCluster* parentCluster  = ecmap->getCluster(current->getParentClusterId());
	nodeTable* perimeter = parentCluster->getParents();


//	std::cout << "perimeter size: "<<perimeter->size()<<std::endl;
	unsigned int count=0;
	for(nodeTable::iterator it = perimeter->begin(); it != perimeter->end(); it++)
	{
		assert(count < perimeter->size());
		node* nb = it->second;
		MacroNode* neighbour = dynamic_cast<MacroNode*>(nb);
		assert(neighbour);
		//if(neighbour->getNum() == current->getNum())
		//	continue;
		
		if(verbose) 
		{
			printNode(std::string("\tneighbour... "), neighbour);
		}
		if(closedList.find(neighbour->getUniqueID()) == closedList.end()) // ignore nodes on the closed list
		{
			// if a node on the openlist is reachable via this new edge, 
			// relax the edge (see cormen et al)
//			if(visitedClusters.find(parentCluster->getId()) != visitedClusters.end())
			if(openList->isIn(neighbour))
			{	
				if(verbose) std::cout << "\t\trelaxing"<<std::endl;
				relaxMacro(openList, current, neighbour, goal); 
				nodesTouched++;
			}
			else
			{
				if(verbose) std::cout << "\t\tadding to open list"<<std::endl;
				neighbour->setLabelF(kTemporaryLabel, MAXINT); // initial fCost = inifinity
				neighbour->setKeyLabel(kTemporaryLabel); // an initial key value for prioritisation in the openlist
				neighbour->setMacroParent(current);
				neighbour->reset();
				openList->add(neighbour);
				relaxMacro(openList, current, neighbour, goal); 
				nodesTouched++;
			}
			if(markForVis)
				neighbour->drawColor = 1; // visualise touched
		}
		else
		{
			if(verbose) 
			{
				std::cout << "\t\tclosed!"<<std::endl;
			}
			double fclosed = neighbour->getLabelF(kTemporaryLabel);
			double gclosed =  fclosed - h(neighbour, goal);

			// alternate fcost
			double altg = current->getLabelF(kTemporaryLabel) - h(current, goal) + h(current, neighbour);
			double altf = altg + h(neighbour, goal);

			if(altf < fclosed)
			{
				std::cout << "node "<<neighbour->getName()<<" expanded out of order! ";
				std::cout << " fClosed = "<<fclosed << " fAlt: "<<altf;
				std::cout << " gClosed = "<<gclosed<< "; gAlt: "<<altg;
				printNode("\nfaulty node: ", neighbour, goal); 
				std::cout << std::endl;
				printNode(" alt parent: ", current, goal);
				std::cout << std::endl;
			}

		}
		count++;
	}
		
	if(markForVis)
		current->drawColor = 2; // visualise expanded

	//if(verbose) printNode(string("closing... "), current);
	closedList[current->getUniqueID()] = current;	

	if(visitedClusters.find(parentCluster->getId()) == visitedClusters.end())
			visitedClusters[parentCluster->getId()] = true;
}

void 
OHAStar::relaxMacro(heap *openList, MacroNode* from, MacroNode* to, node* goal)
{
	assert(from->getParentClusterId() == to->getParentClusterId());
	double g_from = from->getLabelF(kTemporaryLabel) - h(from, goal); 
	double f_to = g_from + h(from, to) + h(to, goal); 

	// update priority and macro parent 
	if(f_to < to->getLabelF(kTemporaryLabel))
	{
		to->setLabelF(kTemporaryLabel, f_to);
		to->setMacroParent(from);
		to->markEdge(0);
		openList->decreaseKey(to);
	}
}

void 
OHAStar::relaxEdge(heap* openList, graph* g, edge* e, int fromId,
		int toId, node* goal)
{
	MacroNode* from = dynamic_cast<MacroNode*>(g->getNode(fromId));
	MacroNode* to = dynamic_cast<MacroNode*>(g->getNode(toId));
	assert(from && to);

	double g_from = from->getLabelF(kTemporaryLabel) - h(from, goal);
	double f_to = g_from + e->getWeight() + h(to, goal);
	
	if(f_to < to->getLabelF(kTemporaryLabel))
	{
		to->setLabelF(kTemporaryLabel, f_to);
		to->setMacroParent(0);
		to->markEdge(e);
		openList->decreaseKey(to);
	}
}

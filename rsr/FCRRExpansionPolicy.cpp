#include "FCRRExpansionPolicy.h"
#include "EmptyClusterAbstraction.h"
#include "ProblemInstance.h"
#include "TileExpansionPolicy.h"

FCRRExpansionPolicy::FCRRExpansionPolicy() :
	ExpansionPolicy()
{
	policy = new TileExpansionPolicy();
}

FCRRExpansionPolicy::~FCRRExpansionPolicy()
{
}

void
FCRRExpansionPolicy::expand(node* n) throw(std::logic_error)
{
	ExpansionPolicy::expand(n);

	policy->setProblemInstance(
			new ProblemInstance(problem->getStartNode(), 
				problem->getGoalNode(),
				problem->getMap(),
				problem->getHeuristic()));
}

node* FCRRExpansionPolicy::first()
{
	/*
	which = 0;
	if(edgesPolicy == 0)
	{
		edgesPolicy = new IncidentEdgesExpansionPolicy(
			map->getAbstractGraph(target->getLabelL(kAbstractionLevel)));	
	}
	
	edgesPolicy->expand(target);
	return edgesPolicy->first();
	*/
	return 0;
}

node* FCRRExpansionPolicy::n()
{
	/*
	if(which >= max)
		return 0;

	node* retVal = edgesPolicy->n();
	if(retVal == 0)
	{
		MacroNode* t = static_cast<MacroNode*>(target);

		EmptyCluster* room = map->getCluster(t->getParentClusterId());
		EmptyCluster::RoomSide side = room->whichSide(t);
		
		int tx = t->getLabelL(kFirstData);
		int ty = t->getLabelL(kFirstData+1);

		int nx, ny;
		switch(side)
		{
			case EmptyCluster::TOP:
				nx = tx;
				ny = room->getVOrigin()+room->getHeight()-1;
				break;
			case EmptyCluster::BOTTOM:
				nx = tx;
				ny = room->getVOrigin();
				break;
			case EmptyCluster::RIGHT:
				nx = room->getHOrigin();
				ny = ty;
				break;
			case EmptyCluster::LEFT:
				nx = room->getHOrigin()+room->getWidth()-1;
				ny = ty;
				break;
			default:
				nx = ny = -1;
				break;
		}

		node* neighbour = map->getNodeFromMap(nx, ny);

		if(neighbour->getLabelL(kAbstractionLevel) != 
				t->getLabelL(kAbstractionLevel))
		{
			graph* g = map->getAbstractGraph(t->getLabelL(kAbstractionLevel));
			neighbour = g->getNode(neighbour->getLabelL(kParent));
			assert(neighbour);
		}
	}
	return retVal;
	*/
	return 0;
}

node* FCRRExpansionPolicy::next()
{
	/*
	node* retVal = edgesPolicy->next();
	if(!retVal)
	{
		which++;
		retVal = n();
	}
	return retVal;
}

bool FCRRExpansionPolicy::hasNext()
{
	if(which < max)
		return true;
	return false;
}

double FCRRExpansionPolicy::cost_to_n() const
{
	if(edgesPolicy->n())
		return edgesPolicy->cost_to_n();
	else
		return map->h(target, n());
		*/
	return 0;
}

bool 
FCRRExpansionPolicy::hasNext()
{
	return false;
}

void FCRRExpansionPolicy::label_n()
{

	node* tmp = this->n();
	if(tmp)
		tmp->backpointer = this->target;
}


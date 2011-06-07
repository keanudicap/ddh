#include "JumpPointLocator.h"

#include "graph.h"
#include "mapAbstraction.h"

JumpPointLocator::JumpPointLocator(mapAbstraction* _map)
{
	this->map = _map;
}

JumpPointLocator::~JumpPointLocator()
{
}


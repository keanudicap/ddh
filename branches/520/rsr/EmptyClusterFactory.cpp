#include "EmptyClusterFactory.h"

EmptyClusterFactory::EmptyClusterFactory()
{
}

EmptyClusterFactory::~EmptyClusterFactory()
{
}

EmptyCluster* EmptyClusterFactory::createCluster(int x, int y)
{
	return new EmptyCluster(x, y);
}

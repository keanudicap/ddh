#ifndef EMPTYCLUSTERFACTORY_H
#define EMPTYCLUSTERFACTORY_H

#include "IHPAClusterFactory.h"
#include "EmptyCluster.h"

class EmptyClusterFactory : public IHPAClusterFactory 
{
	public:
		EmptyClusterFactory();
		virtual ~EmptyClusterFactory();
		virtual EmptyCluster* createCluster(int x, int y);
};

#endif


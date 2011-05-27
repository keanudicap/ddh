/*
 *  IHPAClusterFactory.h
 *  hog
 *
 *  Created by dharabor on 11/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef IHPACLUSTERFACTORY_H
#define IHPACLUSTERFACTORY_H

#include "HPACluster.h"

class AbstractClusterAStar;
class IHPAClusterFactory
{
	public:
		virtual ~IHPAClusterFactory() {}
		virtual HPACluster* createCluster(int xpos, int ypos) = 0;
};

#endif

/*
 *  ClusterFactory.h
 *  hog
 *
 *  Created by dharabor on 11/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HPACLUSTERFACTORY_H
#define HPACLUSTERFACTORY_H

#include "IHPAClusterFactory.h"

class AbstractClusterAStar;
class HPAClusterFactory : public IHPAClusterFactory
{
	public:
		HPAClusterFactory();
		virtual HPACluster* createCluster(int xpos, int ypos);
		virtual HPACluster* createCluster(int xpos, int ypos, int width, int height, AbstractClusterAStar* castar);	
};

#endif


/*
 *  PerimeterSearchFactory.h
 *  hog
 *
 *	@author: dharabor
 *	@created: 16/07/2010
 */

#ifndef PERIMETERSEARCHFACTORY_H
#define PERIMETERSEARCHFACTORY_H

#include "ClusterAStarFactory.h"
#include "PerimeterSearch.h"

class PerimeterSearchFactory : public ClusterAStarFactory 
{
	public:
		virtual ~PerimeterSearchFactory() { }
		virtual AbstractClusterAStar* newClusterAStar();
};

#endif

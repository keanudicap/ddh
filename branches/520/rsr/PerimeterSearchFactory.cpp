/*
 *  PerimeterSearchFactory.cpp
 *  hog
 *
 *  Created by dharabor on 14/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "PerimeterSearchFactory.h"
#include "PerimeterSearch.h"

AbstractClusterAStar* PerimeterSearchFactory::newClusterAStar()
{
	PerimeterSearch* ps = new PerimeterSearch();
	return ps;
}

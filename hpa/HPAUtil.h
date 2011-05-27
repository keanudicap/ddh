/*
 *  HPAUtil.h
 *  hog
 *
 *  Created by dharabor on 11/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HPAUTIL_H
#define HPAUTIL_H

#include <map>
#include <stdexcept>

class node;
class path;
class HPACluster;

namespace HPAUtil
{
	typedef std::map<int, path*> pathTable;
	typedef std::map<int, node*> nodeTable;
	typedef std::map<int, HPACluster*> clusterTable;

	double h(node* from, node* to) throw(std::invalid_argument);
}

#endif

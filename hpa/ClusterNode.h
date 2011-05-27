/*
 *  ClusterNode.h
 *  hog
 *
 *  Created by dharabor on 13/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef CLUSTERNODE_H
#define CLUSTERNODE_H

#include "graph.h"
#include <ostream>

class ClusterNode : public node
{
	public:
		ClusterNode(const char* name, int _parentClusterId);
		ClusterNode(const char* name);
		ClusterNode(const ClusterNode* n);
		~ClusterNode(); 
		virtual graph_object* clone() const { return new ClusterNode(this); }
		virtual void reset() { this->markEdge(0); }
		int getParentClusterId() { return parentClusterId; }
		void setParentClusterId(int _id) { parentClusterId = _id; }

		void print(std::ostream& out);

	private:
		void init();
		int parentClusterId;
	
};

#endif

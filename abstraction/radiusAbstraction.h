/*
 * $Id: radiusAbstraction.h,v 1.6 2006/10/18 23:53:25 nathanst Exp $
 *
 *  radiusAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "mapAbstraction.h"

#ifndef RADIUSABSTRACTION_H
#define RADIUSABSTRACTION_H

class radiusAbstraction : public mapAbstraction {
public:
	radiusAbstraction(Map *, int);
	~radiusAbstraction();
	mapAbstraction *clone(Map *_m) { return new radiusAbstraction(_m, radius); }
	
	virtual bool pathable(node *from, node *to);
	
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void verifyHierarchy();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void removeNode(node *n);
	/** remove edge from abstraction */
  virtual void removeEdge(edge *e, unsigned int absLevel);
	/** add node to abstraction */
	virtual void addNode(node *n);
	/** add edge to abstraction */
	virtual void addEdge(edge *e, unsigned int absLevel);
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single repairAbstraction call. */
  virtual void repairAbstraction();	
private:
	void buildAbstraction();
	void buildNodeIntoParent(node *n, node *parent);
	void abstractionBFS(node *which, node *parent, int depth);

	void addEdges(graph *g);
	void addNodes(graph *g);
	
	int radius;
};

#endif

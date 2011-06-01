/*
 * $Id: loadedCliqueAbstraction.cpp,v 1.9 2006/10/24 18:14:52 nathanst Exp $
 *
 *  loadedCliqueAbstraction.cpp
 *  hog
 *
 * Created by Nathan Sturtevant on 3/10/06.
 * Copyright 2006 Nathan Sturtevant, University of Alberta.
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

#include "loadedCliqueAbstraction.h"
#include <cstdlib>
#include <cstring>

using namespace std;

#include "heap.h"
#include "fpUtil.h"
#include <cmath>
#include <memory>
#include <vector>
#include <assert.h>
#include <stdio.h>

enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const int verbose = kMiscMessages;//kMiscMessages;//kRepairGraph;

/**
* Construct a new graph hierarchy.
 *
 * Constructions a new graph abstraction hierarchy from the graph using the
 * designated abstraction method.
 */
loadedCliqueAbstraction::loadedCliqueAbstraction(char *fname)
:graphAbstraction()
{
	levelDraw = 1;
	buildAbstractions(loadGraph(fname));
}

loadedCliqueAbstraction::~loadedCliqueAbstraction()
{
	cleanMemory();
}

double loadedCliqueAbstraction::h(node *a, node *b)
{
	//	const double root2m1 = sqrt(2.0)-1;
	//  double answer;// = sqrt((rv1.x-rv2.x)*(rv1.x-rv2.x)+(rv1.y-rv2.y)*(rv1.y-rv2.y)+(rv1.z-rv2.z)*(rv1.z-rv2.z));
	//	if (fabs(rv1.x-rv2.x) < fabs(rv1.y-rv2.y)) {
	//		answer = root2m1*fabs(rv1.x-rv2.x)+fabs(rv1.y-rv2.y);
	//	} else {
	//		answer = root2m1*fabs(rv1.y-rv2.y)+fabs(rv1.x-rv2.x);
	//	}
	//	return answer;
	if (a->getLabelF(kXCoordinate) == kUnknownPosition)
		getNodeLoc(a);
	if (b->getLabelF(kXCoordinate) == kUnknownPosition)
		getNodeLoc(b);
	double d1 = a->getLabelF(kXCoordinate)-b->getLabelF(kXCoordinate);
	double d2 = a->getLabelF(kYCoordinate)-b->getLabelF(kYCoordinate);
	double d3 = a->getLabelF(kZCoordinate)-b->getLabelF(kZCoordinate);
	return sqrt(d1*d1+d2*d2+d3*d3);
}

void loadedCliqueAbstraction::toggleDrawAbstraction(int which)
{
	bool drawThis = ((levelDraw>>which)&0x1);
	if (!drawThis)
		levelDraw |= (1<<which);
	else
		levelDraw = levelDraw&(~(1<<which));
}

void loadedCliqueAbstraction::openGLDraw()
{
	glDisable(GL_LIGHTING);
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		if ((levelDraw >> x) & 1) drawGraph(abstractions[x]);
		//glCallList(displayLists[x]);
	}
	glEnable(GL_LIGHTING);
}

void loadedCliqueAbstraction::drawGraph(graph *g)
{
	if ((g == 0) || (g->getNumNodes() == 0)) return;
	
	int abLevel = g->getNode(0)->getLabelL(kAbstractionLevel);	
	
	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	node_iterator ni;
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		node *n;
		n = g->getNode(e->getFrom());
		
		if (e->getLabelL(kEdgeCapacity) == 0)      glColor4f(.5, .5, .5, 1);
		else if (e->getLabelL(kEdgeCapacity) <= 0) glColor4f(.2, .2, .2, 1);
		else if (e->getMarked())                  glColor4f(1, 1, 1, 1);
		else if (abLevel%2)
			glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
		else glColor4f(((GLfloat)(abLevel%15)/15.0), 1-((GLfloat)(abLevel%15)/15.0), 0, 1);
		recVec rv = getNodeLoc(n);
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->getNode(e->getTo());
		rv = getNodeLoc(n);
		
		glVertex3f(rv.x, rv.y, rv.z);
	}
	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
		drawLevelConnections(n);
	glEnd();
	//  if (verbose&kBuildGraph) printf("Done\n");
}

void loadedCliqueAbstraction::drawLevelConnections(node *n)
{
	//	int x, y;
	//	double offsetx, offsety;
	//	recVec ans;
	//if (n->getNumOutgoingEdges()+n->getNumIncomingEdges() == 0) return;
	
	if (n->getLabelL(kAbstractionLevel) == 0) return;
	else {
		glColor4f(.6, .6, .6, .6);
		recVec v = getNodeLoc(n);
		for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++)
		{
			recVec v1 = getNodeLoc(abstractions[n->getLabelL(kAbstractionLevel)-1]->getNode(n->getLabelL(kFirstData+cnt)));
			glVertex3f(v.x, v.y, v.z);
			glVertex3f(v1.x, v1.y, v1.z);
		}
	}
	//return ans;
}

recVec loadedCliqueAbstraction::getNodeLoc(node *n)
{
	//  double offsetx, offsety;
	recVec ans;
	
	if (n->getLabelF(kXCoordinate) != kUnknownPosition)
	{
		ans.x = n->getLabelF(kXCoordinate);
		ans.y = n->getLabelF(kYCoordinate);
		ans.z = n->getLabelF(kZCoordinate);
		return ans;
	}
	
	int totNodes = 0;
	ans.x = ans.y = ans.z = 0;
	for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++)
	{
		int absLevel = n->getLabelL(kAbstractionLevel)-1;
		node *nextChild = abstractions[absLevel]->getNode(n->getLabelL(kFirstData+cnt));
		recVec tmp = getNodeLoc(nextChild);
		int weight = nextChild->getLabelL(kNumAbstractedNodes);
		totNodes += weight;
		ans.x += weight*tmp.x;
		ans.y += weight*tmp.y;
		ans.z += weight*tmp.z;
	}
	ans.x /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
		ans.y /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
			ans.z /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
				
				n->setLabelF(kXCoordinate, ans.x);
				n->setLabelF(kYCoordinate, ans.y);
				n->setLabelF(kZCoordinate, ans.z);
				return ans;
}


graph *loadedCliqueAbstraction::loadGraph(char *fname)
{
	FILE *f = fopen(fname, "r");
	if (!f)
	{
		printf("Error opening %s for loading\n", fname);
		exit(1);
	}
	char nextLine[255];
	std::vector<unsigned int> IDS;
	bool nodes = true;
	graph *g = new graph();
	fgets(nextLine, 255, f);
	const double VERSION = 1.0;
	double version;
	sscanf(nextLine, "VERSION %lf", &version);
	if (version != VERSION)
	{
		printf("Got %f; code can only handle version 1.0\n", version);
		exit(1);
	}
	
	while ((!feof(f)) && fgets(nextLine, 255, f))
	{
		if (nextLine[0] == '#')
			continue;
		if (strncmp("edges", nextLine, 5) == 0)
		{
			nodes = false;
			continue;
		}
		if (nodes)
		{
			// ID x y z
			int ID;
			double x, y, z;
			sscanf(nextLine, "%d %lf %lf %lf", &ID, &x, &y, &z);
			//printf("Loaded node %d\n", ID);
			if (IDS.size() <= (unsigned int)ID)
				IDS.resize(ID+1);
			node *n;
			IDS[ID] = g->addNode(n = new node("l1"));
			n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
			n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
			n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			n->setLabelF(kXCoordinate, x);
			n->setLabelF(kYCoordinate, y);
			n->setLabelF(kZCoordinate, z);
			n->setLabelL(kNodeBlocked, 0);
		}
		else {
			int ID1, ID2;
			sscanf(nextLine, "%d %d", &ID1, &ID2);
			g->addEdge(new edge(IDS[ID1], IDS[ID2], h(g->getNode(ID1), g->getNode(ID2))));
			//printf("Loaded edge between %d and %d\n", ID1, ID2);
		}
	}
	return g;
}

void loadedCliqueAbstraction::verifyHierarchy()
{
	cout << "VERIFY START" << endl;
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		// first make sure graph is ok
		abstractions[x]->verifyGraph();
		// then make sure abstraction is ok
		node_iterator ni = abstractions[x]->getNodeIter();
		for (node *n = abstractions[x]->nodeIterNext(ni); n; n = abstractions[x]->nodeIterNext(ni))
		{
			// verify graph, because there may be issues...
			if (n->getLabelL(kParent) != -1)
			{
				graph *g = abstractions[x+1];
				node *parent = g->getNode(n->getLabelL(kParent));
				bool found = false;
				for (int y = 0; y < parent->getLabelL(kNumAbstractedNodes); y++)
				{
					if (parent->getLabelL(kFirstData+y) == (long)n->getNum())
					{ found = true; break; }
				}
				if (!found)
				{
					cout << "VERIFY: Graph doesn't verify; child:" << endl << *n << endl;
					cout << "VERIFY: Graph doesn't verify; parent:" << endl << *parent << endl;
				}
			}
			if (x > 0)
			{
				graph *g = abstractions[x-1];
				for (int y = 0; y < n->getLabelL(kNumAbstractedNodes); y++)
				{
					node *child = g->getNode(n->getLabelL(kFirstData+y));
					if (!child)
					{
						cout << "VERIFY: Graph doesn't verify; CHILD is null, parent:" << endl << *n << endl;
					}
					else if (child->getLabelL(kParent) != (long)n->getNum())
					{
						cout << "VERIFY: Graph doesn't verify; parent:" << endl << *n << endl;
						cout << "VERIFY: Graph doesn't verify; child:" << endl << *child << endl;
					}
				}
			}
			else {
				//				int x1, y1;
				//				getTileFromNode(n, x1, y1);
				//				if (n != getNodeFromMap(x1, y1))
				//					cout << "VERIFY: node doesn't correspond to underlying map" << endl << *n << endl;
			}
		}
		// verify edges
		edge_iterator ei = abstractions[x]->getEdgeIter();
		for (edge *e = abstractions[x]->edgeIterNext(ei); e; e = abstractions[x]->edgeIterNext(ei))
		{
			node *p1, *p2;
			p1 = findNodeParent(abstractions[x]->getNode(e->getFrom()));
			p2 = findNodeParent(abstractions[x]->getNode(e->getTo()));
			if (p1 == p2)
				continue;
			if ((p1 == 0) || (p2 == 0))
			{
				cout << "VERIFY: One edge parent is null, and the other isn't " << *e << endl << *p1 << endl << *p2 << endl;
				continue;
			}
			if (!abstractions[x+1]->findEdge(p1->getNum(), p2->getNum()))
			{
				cout << "Didn't find parent edge of " << *e << " at abslevel " << x << endl;
				cout << *p1 << endl << *p2 << endl;
			}
			p1 = abstractions[x]->getNode(e->getFrom());
			p2 = abstractions[x]->getNode(e->getTo());
			// VERIFY edge weights
			if (!fequal(e->getWeight(), h(p1, p2)))
			{
				cout << "VERIFY: Edge weight doesn't match heuristic cost. Level: " << x << endl;
				cout << *p1 << endl << *p2 << endl << *e << endl;
				cout << "P1: (" << p1->getLabelF(kXCoordinate) << ", " << p1->getLabelF(kYCoordinate)
					<< ", " << p1->getLabelF(kZCoordinate) << ")" << endl;
				if (p1->getLabelL(kAbstractionLevel) == 0)
					cout << "P1: (" << p1->getLabelL(kFirstData) << ", " << p1->getLabelL(kFirstData+1) << ")" << endl;
				cout << "P2: (" << p2->getLabelF(kXCoordinate) << ", " << p2->getLabelF(kYCoordinate)
					<< ", " << p2->getLabelF(kZCoordinate) << ")" << endl;
				if (p2->getLabelL(kAbstractionLevel) == 0)
					cout << "P2: (" << p2->getLabelL(kFirstData) << ", " << p2->getLabelL(kFirstData+1) << ")" << endl;
				cout << "weight: " << e->getWeight() << " heuristic " << h(p1, p2) << endl;
			}
			// VERIFY edge weights
			if (e->getLabelL(kEdgeCapacity) == 0)
				cout << "VERIFY: Edge capacity is 0?!? " << e << endl;
			if (x > 0) // we can verify the capacity
			{
				int count = 0;
				// we should find kEdgeCapacity edges between the children of p1 and p2
				p1 = abstractions[x]->getNode(e->getFrom());
				p2 = abstractions[x]->getNode(e->getTo());
				for (int c1 = 0; c1 < p1->getLabelL(kNumAbstractedNodes); c1++)
				{
					for (int c2 = 0; c2 < p2->getLabelL(kNumAbstractedNodes); c2++)
					{
						if (abstractions[x-1]->findEdge(p1->getLabelL(kFirstData+c1),
																						p2->getLabelL(kFirstData+c2)))
						{
							count++;
						}
					}
				}
				if (count != e->getLabelL(kEdgeCapacity))
				{
					cout << "VERIFY: Edge capactiy of " << *e << " is "
					<< e->getLabelL(kEdgeCapacity) << " but we only found " << count
					<< " edges below that abstract into it." << endl;
				}
			}
		}
	}
	cout << "VERIFY END" << endl;
}

void loadedCliqueAbstraction::cleanMemory()
{
	while (abstractions.size() > 0)
	{
		delete abstractions.back();
		abstractions.pop_back();
	}
	clearDisplayLists();
	while (displayLists.size() > 0)
		displayLists.pop_back();
}

void loadedCliqueAbstraction::clearDisplayLists()
{
	for (unsigned int x = 0; x < displayLists.size(); x++)
	{
		if (displayLists[x] != 0) glDeleteLists(displayLists[x], 1);
		displayLists[x] = 0;
	}
}

void loadedCliqueAbstraction::buildAbstractions(graph *_g)
{
	int totalNodes = 0;
	cleanMemory();
	abstractions.push_back(_g);
	graph *g = abstractions[0];
	if (displayLists.size() != 1)
		displayLists.push_back(0);
	//if (verbose)
	printf("Base graph (0) has %d nodes\n", g->getNumNodes());
	g->printStats();
	
	for (int x = 1; ; x++)
	{
		if (g->getNumEdges() == 0) break;
		if (verbose&kMiscMessages) printf("Building abstraction #%2d\n", x);
		abstractions.push_back(abstractGraph(g));
		g = abstractions.back();
		if (verbose&kMiscMessages)
		{
			printf("Abstract graph #%2d has %d nodes\n", x, g->getNumNodes());
			g->printStats();
		}
		totalNodes += g->getNumNodes();
		displayLists.push_back(0);
	}
	// printf("%d nodes, excluding bottom level", totalNodes);
}

graph *loadedCliqueAbstraction::abstractGraph(graph *g)
{
	return cliqueAbstractGraph(g);
}

graph *loadedCliqueAbstraction::neighborAbstractGraph(graph *g, int width)
{
	std::vector<node *> remainingNodes;
	graph *aGraph = new graph();
	node_iterator ni = g->getNodeIter();
	node *newNode;
	
	// do we want to abstract big nodes first?
	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		if (n->getLabelL(kParent) != -1) continue;
		newNode = new node("");
		aGraph->addNode(newNode);
		
		newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
		newNode->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
		newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
		newNode->setLabelL(kNodeBlocked, 0);
		newNode->setLabelF(kXCoordinate, kUnknownPosition);
		
		addNodesToParent(g, n, newNode, width);
	}
	
	// now add all the edges
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->getNode(e->getFrom())->getLabelL(kParent);
		int to = g->getNode(e->getTo())->getLabelL(kParent);
		edge *f=0;//, *g=0;
			if ((from != to) && (!(f = aGraph->findEdge(to, from))))
			{
				double weight = h(aGraph->getNode(from), aGraph->getNode(to));
				f = new edge(from, to, weight);
				f->setLabelL(kEdgeCapacity, 1);
				aGraph->addEdge(f);
			}
			else if (f) f->setLabelL(kEdgeCapacity, f->getLabelL(kEdgeCapacity)+1);
			//		else if (g)
			//			g->setLabel(kEdgeCapacity, g->getLabelL(kEdgeCapacity)+1);
	}
	
	return aGraph;
}

void loadedCliqueAbstraction::addNodesToParent(graph *g, node *n, node *parent, int width)
{
	if (n->getLabelL(kParent) != -1)
		return;
	
	// add this node; add all neighbors
	int oldChildren = parent->getLabelL(kNumAbstractedNodes);
	parent->setLabelL(kFirstData+oldChildren, n->getNum());
	parent->setLabelL(kNumAbstractedNodes, oldChildren+1);
	n->setLabelL(kParent, parent->getNum());
	
	if (width <= 0)
		return;
	edge_iterator ei = n->getEdgeIter();
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		if (e->getFrom() == n->getNum())
			addNodesToParent(g, g->getNode(e->getTo()), parent, width-1);
		else
			addNodesToParent(g, g->getNode(e->getFrom()), parent, width-1);
	}
}


graph *loadedCliqueAbstraction::cliqueAbstractGraph(graph *g)
{
	//	int abLevel = g->getNode(0)->getLabelL(kAbstractionLevel);
	//	if (g != abstractions[0])
	//		return graphAbstraction::abstractGraph(g);
	
	//  printf("Doing special map abstraction at level %d\n", (int)g->getNode(0)->getLabelL(kAbstractionLevel));
	
	// 1) join singly linked paths into single nodes
	// 2) join 4-cliques
	// 3) join 3-cliques
	// 4) join 2-cliques
	std::vector<node *> remainingNodes;
	graph *aGraph = new graph();
	node_iterator ni = g->getNodeIter();
	node *newNode;
	
	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		if (n->getLabelL(kParent) != -1) continue;
		int numEdges = n->getNumEdges();//getNumOutgoingEdges() + n->getNumIncomingEdges();
			
			if (numEdges == 1)
			{
				// add to stack; we will merge these after we've finished everything else
				remainingNodes.push_back(n);
				continue;
			}
			
			std::vector<int> neighbor(numEdges);
			edge *e;
			edge_iterator ei = n->getEdgeIter();
			for (int x = 0; x < numEdges; x++)
			{
				e = n->edgeIterNext(ei);
				if (e == 0)
				{
					cout << "That's impossible; we were told we had "
					<< numEdges << ":(" << n->getNumOutgoingEdges()
					<< "+" << n->getNumIncomingEdges()
					<< ") edges, we're on #" << x << " and we got nil!"
					<< endl;
					numEdges = x;
					continue;
				}
				if (e->getFrom() == n->getNum()) neighbor[x] = e->getTo();
				else                             neighbor[x] = e->getFrom();
			}
			
			// check for all cliques involving 4 nodes
			for (int x = 0; x < numEdges-2; x++)
			{
				if (g->getNode(neighbor[x])->getLabelL(kParent) != -1) continue;
				for (int y = x+1; y < numEdges-1; y++)
				{
					if (g->getNode(neighbor[y])->getLabelL(kParent) != -1) continue;
					for (int z = y+1; z < numEdges; z++)
					{
						if (g->getNode(neighbor[z])->getLabelL(kParent) != -1) continue;
						// each node has edge to us; now we also must have
						// x<->y, y<->z, x<->z
						//						if ((g->findEdge(neighbor[x], neighbor[y]) || g->findEdge(neighbor[y], neighbor[x])) &&
						//								(g->findEdge(neighbor[y], neighbor[z]) || g->findEdge(neighbor[z], neighbor[y])) &&
						//								(g->findEdge(neighbor[z], neighbor[x]) || g->findEdge(neighbor[x], neighbor[z])))
						if (g->findEdge(neighbor[x], neighbor[y]) &&
								g->findEdge(neighbor[y], neighbor[z]) &&
								g->findEdge(neighbor[z], neighbor[x]))
						{
							// we have a 4-clique!
							int nnum = aGraph->addNode(newNode = new node("4c"));
							n->setLabelL(kParent, nnum);
							g->getNode(neighbor[x])->setLabelL(kParent, nnum);
							g->getNode(neighbor[y])->setLabelL(kParent, nnum);
							g->getNode(neighbor[z])->setLabelL(kParent, nnum);
							
							newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
							newNode->setLabelL(kNumAbstractedNodes, 4); // number of abstracted nodes
							newNode->setLabelL(kNodeBlocked, 0);
							newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
							newNode->setLabelF(kXCoordinate, kUnknownPosition);
							newNode->setLabelL(kFirstData, n->getNum()); // nodes stored here
							newNode->setLabelL(kFirstData+1, neighbor[x]); // nodes stored here
							newNode->setLabelL(kFirstData+2, neighbor[y]); // nodes stored here
							newNode->setLabelL(kFirstData+3, neighbor[z]); // nodes stored here
							
							x = y = numEdges;
							break;
						}
					}
				}
			}
			
			// check for all cliques involving 3 nodes
			if (n->getLabelL(kParent) == -1)
			{
				for (int x = 0; x < numEdges-1; x++)
				{
					if (g->getNode(neighbor[x])->getLabelL(kParent) != -1) continue;
					for (int y = x+1; y < numEdges; y++)
					{
						if (g->getNode(neighbor[y])->getLabelL(kParent) != -1) continue;
						// each node has edge to us; now we also must have
						// x<->y
						//						if (g->findEdge(neighbor[x], neighbor[y]) || g->findEdge(neighbor[y], neighbor[x]))
						if (g->findEdge(neighbor[x], neighbor[y]))
						{
							// we have a 3-clique!
							int nnum = aGraph->addNode(newNode = new node("3c"));
							n->setLabelL(kParent, nnum);
							g->getNode(neighbor[x])->setLabelL(kParent, nnum);
							g->getNode(neighbor[y])->setLabelL(kParent, nnum);
							
							newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
							newNode->setLabelL(kNumAbstractedNodes, 3); // number of abstracted nodes
							newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
							newNode->setLabelL(kNodeBlocked, 0);
							newNode->setLabelF(kXCoordinate, kUnknownPosition);
							newNode->setLabelL(kFirstData, n->getNum()); // nodes stored here
							newNode->setLabelL(kFirstData+1, neighbor[x]); // nodes stored here
							newNode->setLabelL(kFirstData+2, neighbor[y]); // nodes stored here
							
							x = numEdges;
							break;
						}
					}
				}
			}
			
			//		if (n->getLabelL(kParent) == -1)
			//		{
			//			// check for all cliques involving 2 nodes
			//			for (int x = 0; x < numEdges; x++)
			//			{
			//				if (g->getNode(neighbor[x])->getLabelL(kParent) != -1)
			//					continue;
			//				// we have a 2-clique!
			//				int nnum = aGraph->addNode(newNode = new node("2c"));
			//				n->setLabel(kParent, nnum);
			//				g->getNode(neighbor[x])->setLabel(kParent, nnum);
			//
			//				newNode->setLabel(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
			//				newNode->setLabel(kNumAbstractedNodes, 2); // number of abstracted nodes
			//				newNode->setLabel(kNodeBlocked, 0);
			//				newNode->setLabel(kXCoordinate, -1);
			//				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
			//				newNode->setLabel(kFirstData, n->getNum()); // nodes stored here
			//				newNode->setLabel(kFirstData+1, neighbor[x]); // nodes stored here
			//				break;
			//			}
			//		}
			
			// we didn't find a 3 or 4 clique
			if (n->getLabelL(kParent) == -1) remainingNodes.push_back(n);
	}
	
	while (remainingNodes.size() > 0)
	{
		node *orphan = (node*)remainingNodes.back();
		if (!orphan) break;
		remainingNodes.pop_back();
		if (orphan->getLabelL(kParent) != -1) continue;
		int numEdges = orphan->getNumOutgoingEdges() + orphan->getNumIncomingEdges();
		if (numEdges == 0) continue;
		
		edge_iterator ei = orphan->getEdgeIter();
		for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei))
		{
			int neighbor = (e->getFrom() == orphan->getNum())?e->getTo():e->getFrom();
			if (g->getNode(neighbor)->getLabelL(kParent) == -1)
			{
				unsigned int pNum;
				pNum = aGraph->addNode(newNode = new node("2c"));
				orphan->setLabelL(kParent, pNum);
				g->getNode(neighbor)->setLabelL(kParent, pNum);
				
				newNode->setLabelL(kAbstractionLevel, orphan->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
				newNode->setLabelL(kNumAbstractedNodes, 2); // number of abstracted nodes
				newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
				newNode->setLabelL(kNodeBlocked, 0);
				newNode->setLabelF(kXCoordinate, kUnknownPosition);
				newNode->setLabelL(kFirstData, orphan->getNum()); // nodes stored here
				newNode->setLabelL(kFirstData+1, neighbor); // nodes stored here
				break;
			}
		}
		if ((orphan->getLabelL(kParent) == -1) && (orphan->getNumEdges() == 1))
		{
			ei = orphan->getEdgeIter();
			for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei))
			{
				int neighbor = (e->getFrom() == orphan->getNum())?e->getTo():e->getFrom();
				//printf("merging %d into %d (%d)\n", orphan->getNum(), neighbor, g->getNode(neighbor)->getLabelL(kParent));
				node *adoptee = g->getNode(neighbor);
				orphan->setLabelL(kParent, adoptee->getLabelL(kParent));
				
				node *adopteeParent = aGraph->getNode(adoptee->getLabelL(kParent));
				adopteeParent->setLabelL(kFirstData+adopteeParent->getLabelL(kNumAbstractedNodes), orphan->getNum());
				adopteeParent->setLabelL(kNumAbstractedNodes, adopteeParent->getLabelL(kNumAbstractedNodes)+1);
				break;
			}
		}
		if (orphan->getLabelL(kParent) == -1)
		{
			orphan->setLabelL(kParent, aGraph->addNode(newNode = new node("stranded*")));
			newNode->setLabelL(kAbstractionLevel, orphan->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
			newNode->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
			newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			newNode->setLabelL(kNodeBlocked, 0);
			newNode->setLabelF(kXCoordinate, kUnknownPosition);
			newNode->setLabelL(kFirstData, orphan->getNum()); // nodes stored here
		}
	}
	
	// now add all the edges
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->getNode(e->getFrom())->getLabelL(kParent);
		int to = g->getNode(e->getTo())->getLabelL(kParent);
		edge *f=0;//, *g=0;
							//if ((from != to) && (!(f = aGraph->findEdge(to, from))) && (!(g = aGraph->findEdge(from, to))))
			if ((from != to) && (!(f = aGraph->findEdge(to, from))))
			{
				//			double weight = (aGraph->getNode(from)->getLabelL(kNumAbstractedNodes))+
				//															 (aGraph->getNode(to)->getLabelL(kNumAbstractedNodes));
				//			weight /= 2;
				//			weight += e->getWeight();
				double weight = h(aGraph->getNode(from), aGraph->getNode(to));
				f = new edge(from, to, weight);
				f->setLabelL(kEdgeCapacity, 1);
				aGraph->addEdge(f);
				if (verbose&kBuildGraph)
					printf("Adding edge from %d (%d) to %d (%d) weight %1.2f\n", from, e->getFrom(), to, e->getTo(), weight);
			}
			else if (f) f->setLabelL(kEdgeCapacity, f->getLabelL(kEdgeCapacity)+1);
			//		else if (g)
			//			g->setLabel(kEdgeCapacity, g->getLabelL(kEdgeCapacity)+1);
	}
	
	return aGraph;
}


//graph *loadedCliqueAbstraction::cliqueAbstractGraph(graph *g)
//{
//  //printf("getting abstract graph of level %d\n", g->getNode(0)->getLabelL(kAbstractionLevel));
//	
//  // 1) join singly linked paths into single nodes
//  // 2) join 4-cliques
//  // 3) join 3-cliques
//  // 4) join 2-cliques
//  std::vector<node *> remainingNodes;
//  graph *aGraph = new graph();
//  node_iterator ni = g->getNodeIter();
//  node *newNode;
//	
//  // no tunnels for now -- they aren't cliques
//  //	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
//  //	{
//  //		if (n->getLabelL(kParent) != -1)
//  //			continue;
//  //		if (n->getNumEdges() == 2)
//  //		{
//  //			neighbor_iterator nbi = n->getNeighborIter();
//  //			int n1 = n->nodeNeighborNext(nbi);
//  //			int n2 = n->nodeNeighborNext(nbi);
//  //			if ((g->getNode(n1)->getNumEdges() == 2) || (g->getNode(n2)->getNumEdges() == 2))
//  //			{
//  //				int nnum = aGraph->addNode(newNode = new node("tunnel"));
//  //				n->setLabel(kParent, nnum);
//  //				
//  //				newNode->setLabel(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//  //				newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//  //				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//  //				newNode->setLabel(kNodeBlocked, 0);
//  //				newNode->setLabel(kXCoordinate, -1);
//  //				newNode->setLabel(kFirstData, n->getNum()); // nodes stored here
//  //				
//  //				addTunnel(n, g, newNode);
//  //			}
//  //		}
//  //	}
//  ni = g->getNodeIter();
//  for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni)) {
//    if (n->getLabelL(kParent) != -1) continue;
//    int numEdges = n->getNumEdges();//getNumOutgoingEdges() + n->getNumIncomingEdges();
//			
//			// why abstract solo nodes; they just get confusing later?
//			//		if (numEdges == 0)
//			//		{
//			//			n->setLabel(kParent, aGraph->addNode(newNode = new node("1c")));
//			//			newNode->setLabel(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//			//			newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//			//			newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//			//			newNode->setLabel(kNodeBlocked, 0);
//			//			newNode->setLabel(kXCoordinate, -1);
//			//			newNode->setLabel(kFirstData, n->getNum()); // nodes stored here
//			//			continue;
//			//		}
//			if (numEdges == 1) {
//				// add to stack; we will merge these after we've finished everything else
//				remainingNodes.push_back(n);
//				continue;
//			}
//			
//			int neighbor[numEdges];
//			edge *e;
//			edge_iterator ei = n->getEdgeIter();
//			for (int x = 0; x < numEdges; x++) {
//				e = n->edgeIterNext(ei);
//				if (e == 0) {
//					cout << "That's impossible; we were told we had "
//					<< numEdges << ":(" << n->getNumOutgoingEdges()
//					<< "+" << n->getNumIncomingEdges()
//					<< ") edges, we're on #" << x << " and we got nil!"
//					<< endl;
//					numEdges = x;
//					continue;
//				}
//				if (e->getFrom() == n->getNum()) neighbor[x] = e->getTo();
//				else                             neighbor[x] = e->getFrom();
//			}
//			
//			// check for all cliques involving 4 nodes
//			for (int x = 0; x < numEdges-2; x++) {
//				if (g->getNode(neighbor[x])->getLabelL(kParent) != -1) continue;
//				for (int y = x+1; y < numEdges-1; y++) {
//					if (g->getNode(neighbor[y])->getLabelL(kParent) != -1) continue;
//					for (int z = y+1; z < numEdges; z++) {
//						if (g->getNode(neighbor[z])->getLabelL(kParent) != -1) continue;
//						// each node has edge to us; now we also must have
//						// x<->y, y<->z, x<->z
//						//						if ((g->findEdge(neighbor[x], neighbor[y]) || g->findEdge(neighbor[y], neighbor[x])) &&
//						//								(g->findEdge(neighbor[y], neighbor[z]) || g->findEdge(neighbor[z], neighbor[y])) &&
//						//								(g->findEdge(neighbor[z], neighbor[x]) || g->findEdge(neighbor[x], neighbor[z])))
//						if (g->findEdge(neighbor[x], neighbor[y]) &&
//								g->findEdge(neighbor[y], neighbor[z]) &&
//								g->findEdge(neighbor[z], neighbor[x])) {
//							// we have a 4-clique!
//							int nnum = aGraph->addNode(newNode = new node("4c"));
//							n->setLabelL(kParent, nnum);
//							g->getNode(neighbor[x])->setLabelL(kParent, nnum);
//							g->getNode(neighbor[y])->setLabelL(kParent, nnum);
//							g->getNode(neighbor[z])->setLabelL(kParent, nnum);
//							
//							newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//							newNode->setLabelL(kNumAbstractedNodes, 4); // number of abstracted nodes
//							newNode->setLabelL(kNodeBlocked, 0);
//							newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//							newNode->setLabelF(kXCoordinate, -1);
//							newNode->setLabelL(kFirstData, n->getNum()); // nodes stored here
//							newNode->setLabelL(kFirstData+1, neighbor[x]); // nodes stored here
//							newNode->setLabelL(kFirstData+2, neighbor[y]); // nodes stored here
//							newNode->setLabelL(kFirstData+3, neighbor[z]); // nodes stored here
//							
//							x = y = numEdges;
//							break;
//						}
//					}
//				}
//			}
//			
//			// check for all cliques involving 3 nodes
//			if (n->getLabelL(kParent) == -1) {
//				for (int x = 0; x < numEdges-1; x++) {
//					if (g->getNode(neighbor[x])->getLabelL(kParent) != -1) continue;
//					for (int y = x+1; y < numEdges; y++) {
//						if (g->getNode(neighbor[y])->getLabelL(kParent) != -1) continue;
//						// each node has edge to us; now we also must have
//						// x<->y
//						//						if (g->findEdge(neighbor[x], neighbor[y]) || g->findEdge(neighbor[y], neighbor[x]))
//						if (g->findEdge(neighbor[x], neighbor[y])) {
//							// we have a 3-clique!
//							int nnum = aGraph->addNode(newNode = new node("3c"));
//							n->setLabelL(kParent, nnum);
//							g->getNode(neighbor[x])->setLabelL(kParent, nnum);
//							g->getNode(neighbor[y])->setLabelL(kParent, nnum);
//							
//							newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//							newNode->setLabelL(kNumAbstractedNodes, 3); // number of abstracted nodes
//							newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//							newNode->setLabelL(kNodeBlocked, 0);
//							newNode->setLabelF(kXCoordinate, -1);
//							newNode->setLabelL(kFirstData, n->getNum()); // nodes stored here
//							newNode->setLabelL(kFirstData+1, neighbor[x]); // nodes stored here
//							newNode->setLabelL(kFirstData+2, neighbor[y]); // nodes stored here
//							
//							x = numEdges;
//							break;
//						}
//					}
//				}
//			}
//			
//			//		if (n->getLabelL(kParent) == -1)
//			//		{
//			//			// check for all cliques involving 2 nodes
//			//			for (int x = 0; x < numEdges; x++)
//			//			{
//			//				if (g->getNode(neighbor[x])->getLabelL(kParent) != -1)
//			//					continue;
//			//				// we have a 2-clique!
//			//				int nnum = aGraph->addNode(newNode = new node("2c"));
//			//				n->setLabel(kParent, nnum);
//			//				g->getNode(neighbor[x])->setLabel(kParent, nnum);
//			//
//			//				newNode->setLabel(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//			//				newNode->setLabel(kNumAbstractedNodes, 2); // number of abstracted nodes
//			//				newNode->setLabel(kNodeBlocked, 0);
//			//				newNode->setLabel(kXCoordinate, -1);
//			//				newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//			//				newNode->setLabel(kFirstData, n->getNum()); // nodes stored here
//			//				newNode->setLabel(kFirstData+1, neighbor[x]); // nodes stored here
//			//				break;
//			//			}
//			//		}
//			
//			// we didn't find a 3 or 4 clique
//			if (n->getLabelL(kParent) == -1) remainingNodes.push_back(n);
//  }
//	
//  while (remainingNodes.size() > 0) {
//    node *orphan = (node*)remainingNodes.back();
//    if (!orphan) break;
//    remainingNodes.pop_back();
//    if (orphan->getLabelL(kParent) != -1) continue;
//    int numEdges = orphan->getNumOutgoingEdges() + orphan->getNumIncomingEdges();
//    if (numEdges == 0) continue;
//		
//    edge_iterator ei = orphan->getEdgeIter();
//    for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei)) {
//      int neighbor = (e->getFrom() == orphan->getNum())?e->getTo():e->getFrom();
//      if (g->getNode(neighbor)->getLabelL(kParent) == -1) {
//				unsigned int pNum;
//				pNum = aGraph->addNode(newNode = new node("2c"));
//				orphan->setLabelL(kParent, pNum);
//				g->getNode(neighbor)->setLabelL(kParent, pNum);
//				
//				newNode->setLabelL(kAbstractionLevel, orphan->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//				newNode->setLabelL(kNumAbstractedNodes, 2); // number of abstracted nodes
//				newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//				newNode->setLabelL(kNodeBlocked, 0);
//				newNode->setLabelF(kXCoordinate, -1);
//				newNode->setLabelL(kFirstData, orphan->getNum()); // nodes stored here
//				newNode->setLabelL(kFirstData+1, neighbor); // nodes stored here
//				break;
//      }
//    }
//    if ((orphan->getLabelL(kParent) == -1) && (orphan->getNumEdges() == 1)) {
//      ei = orphan->getEdgeIter();
//      for (edge *e = orphan->edgeIterNext(ei); e; e = orphan->edgeIterNext(ei)) {
//				int neighbor = (e->getFrom() == orphan->getNum())?e->getTo():e->getFrom();
//				//printf("merging %d into %d (%d)\n", orphan->getNum(), neighbor, g->getNode(neighbor)->getLabelL(kParent));
//				node *adoptee = g->getNode(neighbor);
//				orphan->setLabelL(kParent, adoptee->getLabelL(kParent));
//				
//				node *adopteeParent = aGraph->getNode(adoptee->getLabelL(kParent));
//				adopteeParent->setLabelL(kFirstData+adopteeParent->getLabelL(kNumAbstractedNodes), orphan->getNum());
//				adopteeParent->setLabelL(kNumAbstractedNodes, adopteeParent->getLabelL(kNumAbstractedNodes)+1);
//				break;
//      }
//    }
//    if (orphan->getLabelL(kParent) == -1) {
//      orphan->setLabelL(kParent, aGraph->addNode(newNode = new node("stranded*")));
//      newNode->setLabelL(kAbstractionLevel, orphan->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//      newNode->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
//      newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
//      newNode->setLabelL(kNodeBlocked, 0);
//      newNode->setLabelF(kXCoordinate, -1);
//      newNode->setLabelL(kFirstData, orphan->getNum()); // nodes stored here
//    }
//		
//		
//    //				// we aren't going to push nodes into their neighbors for the moment, because it ruins the
//    //				// clique property of nodes.
//    ////      else {
//    ////				//printf("merging %d into %d (%d)\n", orphan->getNum(), neighbor, g->getNode(neighbor)->getLabelL(kParent));
//    ////				node *adoptee = g->getNode(neighbor);
//    ////				orphan->setLabel(kParent, adoptee->getLabelL(kParent));
//    ////				
//    ////				node *adopteeParent = aGraph->getNode((int)adoptee->getLabelL(kParent));
//    ////				adopteeParent->setLabel(kFirstData+(int)adopteeParent->getLabelL(kNumAbstractedNodes), orphan->getNum());
//    ////				adopteeParent->setLabel(kNumAbstractedNodes, adopteeParent->getLabelL(kNumAbstractedNodes)+1);
//    //// 			}
//    //		}
//    // else
//    //		if (orphan->getLabelL(kParent) == -1)
//    //		{
//    //		orphan->setLabel(kParent, aGraph->addNode(newNode = new node("stranded*")));
//    //		newNode->setLabel(kAbstractionLevel, orphan->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
//    //		newNode->setLabel(kNumAbstractedNodes, 1); // number of abstracted nodes
//    //		newNode->setLabel(kParent, -1); // parent of this node in abstraction hierarchy
//    //		newNode->setLabel(kNodeBlocked, 0);
//    //		newNode->setLabel(kXCoordinate, -1);
//    //		newNode->setLabel(kFirstData, orphan->getNum()); // nodes stored here
//    //		}
//  }
//	
//  // now add all the edges
//  edge_iterator ei = g->getEdgeIter();
//  for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei)) {
//    int from = g->getNode(e->getFrom())->getLabelL(kParent);
//    int to = g->getNode(e->getTo())->getLabelL(kParent);
//    edge *f=0;//, *g=0;
//							//if ((from != to) && (!(f = aGraph->findEdge(to, from))) && (!(g = aGraph->findEdge(from, to))))
//			if ((from != to) && (!(f = aGraph->findEdge(to, from)))) {
//				//			double weight = (aGraph->getNode(from)->getLabelL(kNumAbstractedNodes))+
//				//															 (aGraph->getNode(to)->getLabelL(kNumAbstractedNodes));
//				//			weight /= 2;
//				//			weight += e->getWeight();
//				double weight = h(aGraph->getNode(from), aGraph->getNode(to));
//				f = new edge(from, to, weight);
//				f->setLabelL(kEdgeCapacity, 1);
//				aGraph->addEdge(f);
//				if (verbose&kBuildGraph)
//					printf("Adding edge from %d (%d) to %d (%d) weight %1.2f\n", from, e->getFrom(), to, e->getTo(), weight);
//			}
//			else if (f) f->setLabelL(kEdgeCapacity, f->getLabelL(kEdgeCapacity)+1);
//			//		else if (g)
//			//			g->setLabel(kEdgeCapacity, g->getLabelL(kEdgeCapacity)+1);
//  }
//	
//  return aGraph;
//}
	

void loadedCliqueAbstraction::addTunnel(node *n, graph *g, node *newNode)
{
	if (verbose&kBuildGraph) printf("Adding node %d to tunnel\n", n->getNum());
	// check to see if we have neighbors with bf 2 which we can merge with
	neighbor_iterator nbi = n->getNeighborIter();
	int n1 = n->nodeNeighborNext(nbi);
	int n2 = n->nodeNeighborNext(nbi);
	if ((g->getNode(n1)->getLabelL(kParent) == -1) && (g->getNode(n1)->getNumEdges() == 2))
	{
		newNode->setLabelL(kFirstData+newNode->getLabelL(kNumAbstractedNodes), n1); // nodes stored here
		newNode->setLabelL(kNumAbstractedNodes, newNode->getLabelL(kNumAbstractedNodes)+1);
		g->getNode(n1)->setLabelL(kParent, newNode->getNum());
		addTunnel(g->getNode(n1), g, newNode);
	}
	if ((g->getNode(n2)->getLabelL(kParent) == -1) && (g->getNode(n2)->getNumEdges() == 2))
	{
		newNode->setLabelL(kFirstData+newNode->getLabelL(kNumAbstractedNodes), n2); // nodes stored here
		newNode->setLabelL(kNumAbstractedNodes, newNode->getLabelL(kNumAbstractedNodes)+1);
		g->getNode(n2)->setLabelL(kParent, newNode->getNum());
		addTunnel(g->getNode(n2), g, newNode);
	}
}

bool loadedCliqueAbstraction::pathable(unsigned int from, unsigned int to)
{
	return pathable(abstractions[0]->getNode(from), abstractions[0]->getNode(to));
}

bool loadedCliqueAbstraction::pathable(node *from, node *to)
{
	//printf("At nodes #%d and %d\n", from->getNum(), to->getNum());
	while (from != to)
	{
		if ((!from) || (!to) ||
				(abstractions[from->getLabelL(kAbstractionLevel)]->getNumEdges() == 0))
			return false;
		
		from = abstractions[from->getLabelL(kAbstractionLevel)+1]->
			getNode(from->getLabelL(kParent));
		to = abstractions[to->getLabelL(kAbstractionLevel)+1]->
			getNode(to->getLabelL(kParent));
	}
	if ((from == 0) || (to == 0))
		return false;
	return true;
}

//path *loadedCliqueAbstraction::getQuickPath(node *from, node *to)
//{
//	if (absMethod != kloadedCliqueAbstraction)
//		return 0;
//	std::vector<node *> fromChain, toChain;
//	getParentHierarchy(from, to, fromChain, toChain);
//
//	if (!getAbstractGraph(fromChain.back()->getLabelL(kAbstractionLevel))->
//			findEdge(fromChain.back()->getNum(), toChain.back()->getNum()))
//		return 0;
//	
//	path *p = new path(fromChain.back(), new path(toChain.back()));
//	fromChain.pop_back(); toChain.pop_back();
//	while (fromChain.size() > 0)
//	{
//		// 1 if we have an edge to the next node in path, add it
//		// 2 if we have bf 1, add neighbor and repeat
//		// 2 otherwise check each of our neighbors, and add the one that
//		//   has the path to the neighbor as well as that edge
//		return 0;
//	}
//	return p;
//}
	
void loadedCliqueAbstraction::addNode(node *)
{
	assert(false);
}

void loadedCliqueAbstraction::addEdge(edge *, unsigned int)
{
	assert(false);
}

// for now we'll immediately handle splits, but in the future we should queue up splits
// and process them in batch form(?)
void loadedCliqueAbstraction::removeEdge(edge *e, unsigned int absLevel)
{
	if (e == 0)
		return;
	edge *ep = findEdgeParent(e, absLevel);
	if (ep)
	{
		ep->setLabelL(kEdgeCapacity, ep->getLabelL(kEdgeCapacity)-1);
		if (ep->getLabelL(kEdgeCapacity) == 0) removeEdge(ep, absLevel+1);
		//printf("Removing edge %p in abstract graph %d\n", e, absLevel);
		abstractions[absLevel]->removeEdge(e);
		delete e;
		return;
	}
	
	// it's an internal edge, so it might break a link in a parent abstract node
	if (absLevel+1 < abstractions.size())
	{
		node *pNode = abstractions[absLevel+1]->getNode(abstractions[absLevel]->getNode(e->getFrom())->getLabelL(kParent));
		addNodeToRepairQ(pNode);
	}
	
	//printf("Removing edge %p in abstract graph %d\n", e, absLevel);
	abstractions[absLevel]->removeEdge(e);
	delete e;
	return;
}

void loadedCliqueAbstraction::addNodeToRepairQ(node *n)
{
	// key is unsigned, so it has to be >= 0
	if (n)
	{
		if ((n->key >= modifiedNodeQ.size()) ||
				((n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] != n)))
			//		if ((n->key < 0) || (n->key >= modifiedNodeQ.size()) ||
			//				((n->key >= 0) && (n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] != n)))
		{
			n->key = modifiedNodeQ.size();
			if (verbose&kRepairGraph)
				cout << "REM: Adding " << *n << " to modified queue" << endl;
			modifiedNodeQ.push_back(n);
		}
	}
}

void loadedCliqueAbstraction::removeNodeFromRepairQ(node *n)
{
	// key is unsigned, so it has to be >= 0
	//	if ((n->key >= 0) && (n->key < modifiedNodeQ.size()) &&
	//			(modifiedNodeQ[n->key] == n))
	if ((n->key < modifiedNodeQ.size()) && (modifiedNodeQ[n->key] == n))
	{
		modifiedNodeQ[n->key] = modifiedNodeQ.back();
		modifiedNodeQ[n->key]->key = n->key;
		modifiedNodeQ.pop_back();
	}
}

void loadedCliqueAbstraction::removeNode(node *n)
{
	if (n == 0) return;
	if (verbose&kRepairGraph)
		cout << "REM: Removing " << *n << endl;
	removeNodeFromRepairQ(n);
	edge_iterator ei;
	ei = n->getEdgeIter();
	unsigned int absLevel = n->getLabelL(kAbstractionLevel);
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		edge *ep = findEdgeParent(e, absLevel);
		if (!ep)// edge is internal to a node
		{
			node *pNode = abstractions[absLevel+1]->getNode(n->getLabelL(kParent));
			if (!pNode) continue;
			addNodeToRepairQ(pNode);
			continue;
		}
		ep->setLabelL(kEdgeCapacity, ep->getLabelL(kEdgeCapacity)-1);
		if (ep->getLabelL(kEdgeCapacity) == 0)
		{
			removeEdge(ep, n->getLabelL(kAbstractionLevel)+1);
		}
	}
	
	node *np = findNodeParent(n);
	if (np)
	{
		resetLocationCache(np);
		//np->setLabel(kXCoordinate, -1);
		if (np->getLabelL(kNumAbstractedNodes) == 1)
		{
			// our parent might be in the modified node Q. If it is,
			// we can just take it out
			removeNodeFromRepairQ(np);
			
			if (verbose&kRepairGraph)
				printf("Removing parent!\n");
			removeNode(np);
			n->setLabelL(kParent, -1);
		} else {
			// find this node (n) and removed it from the list
			for (int x = 0; x < np->getLabelL(kNumAbstractedNodes); x++)
			{
				if (np->getLabelL(kFirstData+x) == (long)n->getNum())
				{
					np->setLabelL(kFirstData+x,
												np->getLabelL((kFirstData+np->getLabelL(kNumAbstractedNodes)-1)));
					break;
				}
			}
			np->setLabelL(kNumAbstractedNodes, np->getLabelL(kNumAbstractedNodes)-1);
			resetLocationCache(np);
		}
	}
	unsigned int oldID;
	// now we can safely remove this node from the graph
	node *changed = abstractions[absLevel]->removeNode(n, oldID);
	// have to handle changing node here...rename it in the parent & children if necessary
	if (changed)
	{
		renameNodeInAbstraction(changed, oldID);
	}
}

void loadedCliqueAbstraction::repairAbstraction()
{
	// actually want to sort items...based on abstraction level, doing
	// lowest abstraction level first
	while (modifiedNodeQ.size() > 0)
	{
		node *temp, *changed = modifiedNodeQ.back();
		modifiedNodeQ.pop_back();
		// selection sort...assuming that modifiedNodeQ is small for now
		for (unsigned int x = 0; x < modifiedNodeQ.size(); x++)
		{
			if (modifiedNodeQ[x]->getLabelL(kAbstractionLevel) <
					changed->getLabelL(kAbstractionLevel))
			{
				temp = modifiedNodeQ[x];
				modifiedNodeQ[x] = changed;
				changed->key = x;
				changed = temp;
			}
		}
		if (verbose&kRepairGraph)
			cout << "REM: Choosing to repair: " << *changed << endl;
		int count = getChildGroups(changed);
		if (count != 1)
		{
			if (verbose&kRepairGraph)
				cout << "REM: We got " << count << " groups out of " << *changed << endl;
			splitNode(changed, count);
		}
	}
}

/*
 * Takes a node and does a simple bfs on its children to find connected
 * components. Marks the group of each child with kTemporary label &
 * returns the number of groups found.
 */
int loadedCliqueAbstraction::getChildGroups(node *which)
{
	std::vector<node *> seenStack;
	seenStack.reserve(which->getLabelL(kNumAbstractedNodes));
	unsigned int absLevel = which->getLabelL(kAbstractionLevel);
	if (absLevel == 0)
		return 0;
	graph *g = abstractions[absLevel-1];
	for (int x = 0; x < which->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(which->getLabelL(kFirstData+x));
		nextChild->setLabelL(kTemporaryLabel, -1);
	}
	int currGroup = -1;
	for (int x = 0; x < which->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(which->getLabelL(kFirstData+x));
		
		if (nextChild->getLabelL(kTemporaryLabel) != -1)
			continue;
		
		currGroup++;
		nextChild->setLabelL(kTemporaryLabel, currGroup);
		if (verbose&kRepairGraph)
			cout << *nextChild << " now assigned to group " << currGroup << endl;
		do {
			if (seenStack.size() > 0)
			{
				nextChild = seenStack.back();
				seenStack.pop_back();
			}
			edge_iterator ei = nextChild->getEdgeIter();
			for (edge *e = nextChild->edgeIterNext(ei); e; e = nextChild->edgeIterNext(ei))
			{
				unsigned int neighbor = (e->getFrom() == nextChild->getNum()) ?
				(e->getTo()):(e->getFrom());
				if ((g->getNode(neighbor)->getLabelL(kParent) == (long)which->getNum()) &&
						(g->getNode(neighbor)->getLabelL(kTemporaryLabel) == -1))
				{
					g->getNode(neighbor)->setLabelL(kTemporaryLabel, currGroup);
					if (verbose&kRepairGraph)
						cout << *g->getNode(neighbor) << " now added to group " << currGroup << endl;
					seenStack.push_back(g->getNode(neighbor));
				}
			}
		} while (seenStack.size() > 0);
	}
	return currGroup+1;
}


/*
 * Takes a node & splits it so that it is connected.
 */
void loadedCliqueAbstraction::splitNode(node *parent, int numGroups)
{
	// get all children nodes that we are re-assigning
	std::vector<node *> children;
	children.reserve(parent->getLabelL(kNumAbstractedNodes));
	for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
	{
		children[x] = abstractions[parent->getLabelL(kAbstractionLevel)-1]->
		getNode(parent->getLabelL(kFirstData+x));
	}
	
	// 1. if parent has no neighbors, we can just split the parent into separate pieces
	//    since it has already been split. Then we're done.
	if (parent->getNumEdges() == 0)
	{
		if (verbose&kRepairGraph)
			cout << "REM: parent has no neighbors, just splitting and ending" << endl;
		for (int x = 1; x < numGroups; x++)
			extractGroupIntoNewNode(parent, x);
		return;
	}
	
	std::vector<int> groupSize(numGroups);
	for (int x = 0; x < numGroups; x++) groupSize[x] = getGroupSize(parent, x);
	// now, progress through the following steps until we are done
	// 2. all single nodes with bf 1 should merge into neighbors
	// 3. other single nodes should try to join nodes if they can form a clique
	// 4. remaining single nodes abstract by themselves
	int reassigned = 0;
	for (int x = 0; x < numGroups; x++)
	{
		if (reassigned == numGroups-1) // done when we assign all but 1
			break;
		if (groupSize[x] == 1)
		{
			node *n = getNodeInGroup(parent, x);
			if (n->getNumEdges() == 0)
			{
				if (verbose&kRepairGraph)
					cout << "REM: Reassigning group " << reassigned << " by splitting off group " << x << endl;
				extractGroupIntoNewNode(parent, x);
				reassigned++;
			}
			else if (n->getNumEdges() == 1)
			{
				if (verbose&kRepairGraph)
					cout << "REM: Reassigning group " << reassigned << " by (neighbor) merging group " << x << endl;
				mergeGroupIntoNeighbor(parent, x);
				reassigned++;
			}
			else { // (n->getNumEdges() > 1) 	   
				node *neighbor;
				if ((neighbor = findNeighborCliques(parent, x)))
				{
					if (verbose&kRepairGraph)
						cout << "REM: Reassigning group " << reassigned << " by (neighbor) merging " << x << endl;
					mergeGroupIntoNeighbor(parent, x, neighbor);
				}
				else {
					if (verbose&kRepairGraph)
						cout << "REM: Reassigning group " << reassigned << " by extraction " << x << endl;
					extractGroupIntoNewNode(parent, x);
				}
				reassigned++;
			}
		}
	}
	// 5. all groups >= 2 nodes should break off by themselves
	// we do this last so that we make sure to abstract off any single nodes properly
	// no use leaving them sitting here alone if they could have gone off with another node,
	// and we sent a 2+ group off instead
	for (int x = 0; x < numGroups; x++)
	{
		if (reassigned == numGroups-1) // done when we assign all but 1
			break;
		if (getGroupSize(parent, x) > 0)
		{
			if (verbose&kRepairGraph)
				cout << "REM: Reassigning (big) group " << reassigned << " by extracting " << x << endl;
			extractGroupIntoNewNode(parent, x);
			reassigned++;
		}
	}
}

/*
 * Find any node in parent that is a member of "group"
 */
node *loadedCliqueAbstraction::getNodeInGroup(node *parent, int group)
{
	graph *g = abstractions[parent->getLabelL(kAbstractionLevel)-1];
	for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(parent->getLabelL(kFirstData+x));
		
		if (nextChild->getLabelL(kTemporaryLabel) == group)
			return nextChild;
	}
	return 0;
}

/*
 * Count how many nodes are a member of "group"
 */
int loadedCliqueAbstraction::getGroupSize(node *parent, int group)
{
	graph *g = abstractions[parent->getLabelL(kAbstractionLevel)-1];
	int answer = 0;
	for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(parent->getLabelL(kFirstData+x));
		if (nextChild->getLabelL(kTemporaryLabel) == group)
			answer++;
	}
	return answer;
}

/*
 * check to see if the node in the group can be merged into another
 * node as part of a clique. Returns the neighbor node at the child level
 */
// PRECOND: group only has 1 node
node *loadedCliqueAbstraction::findNeighborCliques(node *parent, int group)
{
	node *child = 0;
	graph *g = abstractions[parent->getLabelL(kAbstractionLevel)-1];
	for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(parent->getLabelL(kFirstData+x));
		if (nextChild->getLabelL(kTemporaryLabel) == group)
		{
			child = nextChild;
			break;
		}
	}
	if (child)
		return findNeighborCliques(child);
	return 0;
}

/*
 * check to see if the node can be merged into another
 * node as part of a clique. Returns the neighbor node.
 */
node *loadedCliqueAbstraction::findNeighborCliques(node *child)
{
	graph *g = abstractions[child->getLabelL(kAbstractionLevel)];
	edge_iterator ei = child->getEdgeIter();
	for (edge *e = child->edgeIterNext(ei); e; e = child->edgeIterNext(ei))
	{
		node *nextNode = g->getNode((e->getFrom() == child->getNum())?(e->getTo()):(e->getFrom()));
		if (checkNeighborClique(child, nextNode))
			return nextNode;
	}
	return 0;
}

/*
 * check to see if the node is connected to every other node in the group of
 * the neighbor. If a node has only 1 neighbor it doesn't have to be connected to
 * it, but it must be connected to at least 1 node and every node of degree 2 or greater
 */
bool loadedCliqueAbstraction::checkNeighborClique(node *child, node *neighbor)
{
	node *neighborParent = abstractions[neighbor->getLabelL(kAbstractionLevel)+1]->getNode(neighbor->getLabelL(kParent));
	if (neighborParent == 0)
		return false;
	graph *g = abstractions[child->getLabelL(kAbstractionLevel)];
	int matches = 0;
	for (int x = 0; x < neighborParent->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextChild = g->getNode(neighborParent->getLabelL(kFirstData+x));
		// we only require that we connect to every node in the abstraction
		// that has more than 1 neighbor
		if (g->findEdge(nextChild->getNum(), child->getNum()))
			matches++;
		else if (nextChild->getNumEdges() > 1)
			return false;
	}
	
	if (matches) return true;
	
	return false;
}

/*
 * Take all nodes in group of parent and move them into neighbor's group
 * neighbor is one level lower than the parent
 */
void loadedCliqueAbstraction::mergeGroupIntoNeighbor(node *parent, int group, node *neighbor)
{
	graph *g;
	if (neighbor == 0)
	{ // only 1 possible neighbor, so find it
		g = abstractions[parent->getLabelL(kAbstractionLevel)-1];
		for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
		{
			node *nextChild = g->getNode(parent->getLabelL(kFirstData+x));
			if (nextChild->getLabelL(kTemporaryLabel) == group)
			{
				edge_iterator ei = nextChild->getEdgeIter();
				edge *e = nextChild->edgeIterNext(ei);
				if (e->getFrom() == nextChild->getNum())
					neighbor = g->getNode(e->getTo());
				else
					neighbor = g->getNode(e->getFrom());
				break;
			}
		}
	}
	assert(neighbor->getLabelL(kAbstractionLevel)+1 == parent->getLabelL(kAbstractionLevel));
	if (verbose&kRepairGraph)
		cout << "Merging group " << group << " into " << *neighbor << endl;
	g = abstractions[parent->getLabelL(kAbstractionLevel)];
	node *newParent = g->getNode(neighbor->getLabelL(kParent));
	if (verbose&kRepairGraph)
		cout << "(Child of " << *newParent << ")" << endl;
	// what happens if that node has no parent???
	if (newParent == 0)
	{
		printf("GOTTA HANDLE THE NO PARENT CASE IN mergeGroupIntoNeighbor --- but logic says we shouldn't hit this unless we add edges\n");
		exit(0);
	}
	if (newParent == parent)
	{
		printf("HOW DID THAT HAPPEN? oldParent = newParent\n");
		exit(0);
	}
	else {
		assert(parent->getLabelL(kAbstractionLevel) == newParent->getLabelL(kAbstractionLevel));
		transferGroup(group, parent, newParent);
	}
}

/*
 * take all member of group in parent and make them their own node
 */
void loadedCliqueAbstraction::extractGroupIntoNewNode(node *parent, int group)
{
	// make new node...
	node *newNode;
	graph *g = abstractions[parent->getLabelL(kAbstractionLevel)];
	
	/*unsigned int gpNum = */g->addNode(newNode = new node("split node"));
	newNode->setLabelL(kAbstractionLevel, parent->getLabelL(kAbstractionLevel));
	newNode->setLabelL(kNumAbstractedNodes, 0);
	newNode->setLabelL(kParent, -1);
	newNode->setLabelF(kXCoordinate, kUnknownPosition);
	newNode->setLabelL(kNodeBlocked, 0);
	
	transferGroup(group, parent, newNode);
	
	// now, what do I do with newNode? It needs to see if it can be combine with it's neighbors, etc...
	insertNodeIntoHierarchy(newNode);
}

/*
 * Given a new node that is connected, but doesn't have a parent or have its edges
 * abstracted, and insert it into graph.
 */
void loadedCliqueAbstraction::insertNodeIntoHierarchy(node *newNode)
{
	node *neighbor;
	if (newNode->getNumEdges() == 0)
		return;
	if (newNode->getNumEdges() == 1)
	{
		graph *g = abstractions[newNode->getLabelL(kAbstractionLevel)];
		edge *e = newNode->getEdge(0);
		node *newParent = g->getNode((e->getFrom() == newNode->getNum())?(e->getTo()):(e->getFrom()));
		checkAndCreateParent(newParent);
		g = abstractions[newNode->getLabelL(kAbstractionLevel)+1];
		newParent = g->getNode(newParent->getLabelL(kParent));
		
		newParent->setLabelL(kFirstData+newParent->getLabelL(kNumAbstractedNodes), newNode->getNum());
		newParent->setLabelL(kNumAbstractedNodes, newParent->getLabelL(kNumAbstractedNodes)+1);
		//newParent->setLabel(kXCoordinate, -1);
		resetLocationCache(newParent);
		if (verbose&kRepairGraph)
		{
			printf("Collapsing node into neighbor: ");
			printf("New parent (%d) now has %ld abstracted nodes\n", newParent->getNum(),
						 newParent->getLabelL(kNumAbstractedNodes));
		}
		newNode->setLabelL(kParent, newParent->getNum());		
	}
	else if ((neighbor = findNeighborCliques(newNode)))
	{ // add newnode to neighbor's parent
		checkAndCreateParent(neighbor);
		node *parent = abstractions[neighbor->getLabelL(kAbstractionLevel)+1]->getNode(neighbor->getLabelL(kParent));
		newNode->setLabelL(kParent, parent->getNum());
		parent->setLabelL(kFirstData+parent->getLabelL(kNumAbstractedNodes), newNode->getNum());
		parent->setLabelL(kNumAbstractedNodes, parent->getLabelL(kNumAbstractedNodes)+1);
		//parent->setLabel(kXCoordinate, -1);
		resetLocationCache(parent);
		if (verbose&kRepairGraph)
		{
			printf("Cliquing node into neighbor: ");
			printf("New parent (%d) now has %ld abstracted nodes\n", parent->getNum(),
						 parent->getLabelL(kNumAbstractedNodes));
		}
		
		unsigned int absLevel = newNode->getLabelL(kAbstractionLevel);
		edge_iterator ei = newNode->getEdgeIter();
		for (edge *e = newNode->edgeIterNext(ei); e; e = newNode->edgeIterNext(ei))
		{
			abstractUpEdge(absLevel, e);
		}
	}
	else {
		if (verbose&kRepairGraph)
			cout << "We stay lonely: " << *newNode << endl;
		// add direct parent
		checkAndCreateParent(newNode);
		// add edges...
		unsigned int absLevel = newNode->getLabelL(kAbstractionLevel);
		edge_iterator ei = newNode->getEdgeIter();
		for (edge *e = newNode->edgeIterNext(ei); e; e = newNode->edgeIterNext(ei))
		{
			abstractUpEdge(absLevel, e);
		}
		graph *g = abstractions[absLevel+1];
		insertNodeIntoHierarchy(g->getNode(newNode->getLabelL(kParent)));
	}
}

/*
 * Check to see if a node has a parent. If it doesn't, create one, and optionally
 * extend the abstraction level of the graph as well.
 */
void loadedCliqueAbstraction::checkAndCreateParent(node *which)
{
	if (which->getLabelL(kParent) != -1)
		return;
	unsigned int absLevel = which->getLabelL(kAbstractionLevel);
	if (absLevel+1 == abstractions.size())
	{
		abstractions.push_back(new graph());
	}
	node *parent;
	unsigned int id = abstractions[absLevel+1]->addNode(parent = new node("created"));
	which->setLabelL(kParent, id);
	parent->setLabelL(kAbstractionLevel, absLevel+1);
	parent->setLabelL(kNumAbstractedNodes, 1);
	parent->setLabelL(kParent, -1);
	parent->setLabelF(kXCoordinate, kUnknownPosition);
	parent->setLabelL(kNodeBlocked, 0);
	parent->setLabelL(kFirstData, which->getNum());
}

/*
 * take all nodes in a group of old parent and move them into newParent
 */
void loadedCliqueAbstraction::transferGroup(int group, node *oldParent, node *newParent)
{
	std::vector<node *> groupies;
	
	assert(oldParent->getLabelL(kAbstractionLevel) == newParent->getLabelL(kAbstractionLevel));
	
	groupies.reserve(oldParent->getLabelL(kNumAbstractedNodes));
	graph *g = abstractions[oldParent->getLabelL(kAbstractionLevel)-1];
	for (int x = 0; x < oldParent->getLabelL(kNumAbstractedNodes); x++)
	{
		node *nextNode = g->getNode(oldParent->getLabelL(kFirstData+x));
		if (nextNode->getLabelL(kTemporaryLabel) == group)
		{
			groupies.push_back(nextNode);
			
			// before I move this node over, I have to change its old edges...FIRST
			edge_iterator ei = nextNode->getEdgeIter();
			for (edge *e = nextNode->edgeIterNext(ei); e; e = nextNode->edgeIterNext(ei))
			{
				edge *ep = findEdgeParent(e, nextNode->getLabelL(kAbstractionLevel));
				if (ep)
				{
					ep->setLabelL(kEdgeCapacity, ep->getLabelL(kEdgeCapacity)-1);
					if (ep->getLabelL(kEdgeCapacity) == 0)
					{
						removeEdge(ep, nextNode->getLabelL(kAbstractionLevel)+1);
					}
				}
			}				
			
			oldParent->setLabelL(kFirstData+x, oldParent->getLabelL(kFirstData+oldParent->getLabelL(kNumAbstractedNodes)-1));
			oldParent->setLabelL(kNumAbstractedNodes, oldParent->getLabelL(kNumAbstractedNodes)-1);
			if (verbose&kRepairGraph)
				printf("Old parent (%d) now has %ld abstracted nodes\n", oldParent->getNum(), oldParent->getLabelL(kNumAbstractedNodes));
			newParent->setLabelL(kFirstData+newParent->getLabelL(kNumAbstractedNodes), nextNode->getNum());
			newParent->setLabelL(kNumAbstractedNodes, newParent->getLabelL(kNumAbstractedNodes)+1);
			if (verbose&kRepairGraph)
				printf("New parent (%d) now has %ld abstracted nodes\n", newParent->getNum(), newParent->getLabelL(kNumAbstractedNodes));
			nextNode->setLabelL(kParent, newParent->getNum());
			resetLocationCache(oldParent);
			resetLocationCache(newParent);
			//oldParent->setLabel(kXCoordinate, -1);
			//newParent->setLabel(kXCoordinate, -1);
			x--;
		}
	}
	
	//	g = abstractions[oldParent->getLabelL(kAbstractionLevel)-1];
	for (unsigned int x = 0; x < groupies.size(); x++)
	{
		node *which = groupies[x];
		// now, re-add parent edges
		edge_iterator ei = which->getEdgeIter();
		for (edge *e = which->edgeIterNext(ei); e; e = which->edgeIterNext(ei))
		{
			if (verbose&kRepairGraph)
				cout << "Group member: " << *which << " has edge " << *e << " which we want to abstract up" << endl;
			abstractUpEdge(which->getLabelL(kAbstractionLevel), e);
		}
	}
	
}

/*
 * given an edge e at a particular abstraction level, keep abstracting the edge
 * until it disappears...
 *
 * note that if we connect an orphan node (ie one with degree 1) we might
 * break the assumptions of the abstraction & then want to break a node out
 *
 * same thing with connecting a node with degree 0 to a node with degree 1
 */
void loadedCliqueAbstraction::abstractUpEdge(unsigned int absLevel, edge *e)
{
	// 1 find edge in parent
	graph *g = abstractions[absLevel];
	graph *pg = abstractions[absLevel+1];
	int par1, par2;
	par1 = g->getNode(e->getFrom())->getLabelL(kParent);
	par2 = g->getNode(e->getTo())->getLabelL(kParent);
	if (par1 == par2)
		return;
	if (par1 == -1)
	{ //addNodeToRepairQ(g->getNode(e->getFrom()));
		if (verbose&kRepairGraph)
			cout << "This node has " << g->getNode(e->getFrom())->getNumEdges() <<
				" edge(s), but no parent: " << endl << *g->getNode(e->getFrom()) << endl;
		return;
	}
	if (par2 == -1)
	{ //addNodeToRepairQ(g->getNode(e->getFrom()));
		if (verbose&kRepairGraph)
			cout << "This node has " << g->getNode(e->getTo())->getNumEdges() <<
				" edge(s), but no parent: " << endl << *g->getNode(e->getTo()) << endl;
		return;
	}
	edge *pe = pg->findEdge(par1, par2);
	
	// 2 if it exists, just add to the count
	if (pe)
		pe->setLabelL(kEdgeCapacity, pe->getLabelL(kEdgeCapacity)+1);
	// 3 if it doesn't exist, add it, and recurse
	else {
		pg->addEdge(pe = new edge(par1, par2, h(pg->getNode(par1), pg->getNode(par2))));
		pe->setLabelL(kEdgeCapacity, 1);
		if (verbose&kRepairGraph)
			cout << "*** Abstracting " << *e << " with edge " << *pe << endl;
		abstractUpEdge(absLevel+1, pe);
	}
}

void loadedCliqueAbstraction::resetLocationCache(node *n)
{
	unsigned int absLevel = n->getLabelL(kAbstractionLevel);
	while (true)
	{
		n->setLabelF(kXCoordinate, kUnknownPosition);
		int parent = n->getLabelL(kParent);
		if (parent == -1)
			break;
		absLevel++;
		n = abstractions[absLevel]->getNode(parent);
	}
}

node *loadedCliqueAbstraction::findNodeParent(node *n)
{
	unsigned int absLevel = n->getLabelL(kAbstractionLevel);
	if (absLevel < abstractions.size()-1)
		return abstractions[absLevel+1]->getNode(n->getLabelL(kParent));
	return 0;
}

/*
 * Given an edge at a level of abstraction, find the edge that
 * this edge abstracts into.
 */
edge *loadedCliqueAbstraction::findEdgeParent(edge *e, unsigned int absLevel)
{
	if (absLevel >= abstractions.size()-1) return 0;
	
	graph *g = abstractions[absLevel];
	
	node *from = g->getNode(e->getFrom());
	node *to = g->getNode(e->getTo());
	
	g = abstractions[absLevel+1];
	from = g->getNode(from->getLabelL(kParent));
	to = g->getNode(to->getLabelL(kParent));
	
	if (from == to) return 0;
	return g->findEdge(from->getNum(), to->getNum());
	//	edge *ee = g->findEdge(from->getNum(), to->getNum());
	//	if (ee)
	//		return ee;
	//	return g->findEdge(to->getNum(), from->getNum());
}

void loadedCliqueAbstraction::renameNodeInAbstraction(node *which, unsigned int oldID)
{
	unsigned int absLevel = which->getLabelL(kAbstractionLevel);
	
	// adjust children
	if (absLevel > 0)
	{
		for (int x = 0; x < which->getLabelL(kNumAbstractedNodes); x++)
		{
			abstractions[absLevel-1]->getNode(which->getLabelL(kFirstData+x))->setLabelL(kParent, which->getNum());
		}
	}
	
	// adjust parent
	if (absLevel < abstractions.size()-1)
	{
		node *parent = abstractions[absLevel+1]->getNode(which->getLabelL(kParent));
		if (parent)
		{
			for (int x = 0; x < parent->getLabelL(kNumAbstractedNodes); x++)
			{
				if (parent->getLabelL(kFirstData+x) == (long)oldID)
				{
					parent->setLabelL(kFirstData+x, which->getNum());
					break;
				}
			}
		}
	}
}

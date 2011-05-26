/*
 *  HPAClusterAbstraction.cpp
 *  hog
 *
 *  Created by dharabor on 10/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "HPAClusterAbstraction.h"
#include "HPACluster.h"
#include "HPAClusterFactory.h"
#include "ClusterNode.h"
#include "ClusterNodeFactory.h"
#include "ClusterAStar.h"

#include "IClusterAStarFactory.h"
#include "map.h"
#include "NodeFactory.h"
#include "EdgeFactory.h"
#include "path.h"
#include <stdexcept>
#include <sstream>

const unsigned int DEFAULTCLUSTERSIZE = 10;

// TODO: throw an exception if anything of the parameter pointers are NULL.
HPAClusterAbstraction::HPAClusterAbstraction(Map* m, IHPAClusterFactory* _cf, 
	INodeFactory* _nf, IEdgeFactory* _ef, bool allowDiagonals_) 
	throw(std::invalid_argument)
	: mapAbstraction(m), cf(_cf), nf(_nf), ef(_ef), 
	clustersize(DEFAULTCLUSTERSIZE), allowDiagonals(allowDiagonals_)
{	
	
	node* n = nf->newNode("test");
	if(!dynamic_cast<ClusterNode*>(n))
	{
		delete n;
		throw std::invalid_argument("HPAClusterAbstraction requires a node factory"
				"that produces ClusterNode objects");
	}
	delete n;
		
	abstractions.push_back(getMapGraph(this->getMap(), nf, ef, allowDiagonals)); 
	abstractions.push_back(new graph());	
	startid = goalid = -1;
	drawClusters=true;
	
	nodesExpanded = nodesTouched = nodesGenerated = 0;
	searchTime = 0;
	verbose = false;
}

HPAClusterAbstraction::~HPAClusterAbstraction()
{
	delete ef;
	delete cf;
	delete nf;
	
	for(HPAUtil::pathTable::iterator it = pathCache.begin(); it != pathCache.end(); it++)
		delete (*it).second;
	pathCache.erase(pathCache.begin(), pathCache.end());

	for(HPAUtil::clusterTable::iterator it = clusters.begin(); it != clusters.end(); it++)
		delete (*it).second;
	clusters.erase(clusters.begin(), clusters.end());
}

HPACluster* HPAClusterAbstraction::getCluster(int cid)
{		
	HPAUtil::clusterTable::iterator it = clusters.find(cid);
	if(it != clusters.end())
		return (*it).second;
	
	return 0;
}

void HPAClusterAbstraction::addCluster(HPACluster* cluster) 
{ 
	clusters[cluster->getClusterId()] = cluster;
} 

void HPAClusterAbstraction::buildClusters()
{
	int mapwidth = this->getMap()->getMapWidth();
	int mapheight= this->getMap()->getMapHeight();

	/* need to split the map into fixed-size cluster areas that will form the basis of our abstract graph building later */
	int csize = getClusterSize();
	for(int x=0; x<mapwidth; x+=csize)
		for(int y=0; y<mapheight; y+= csize)
		{	
			int cwidth=csize;
			if(x+cwidth > mapwidth)
				cwidth = mapwidth - x;
			int cheight=csize;
			if(y+cheight > mapheight)
				cheight = mapheight - y;
				
			HPACluster *cluster = cf->createCluster(x,y);
			cluster->setWidth(cwidth);
			cluster->setHeight(cheight);
			//cluster->setVerbose(getVerbose());
			addCluster( cluster ); // nb: also assigns a new id to cluster
			cluster->addNodesToCluster(this);
		}
}

void HPAClusterAbstraction::buildEntrances()
{
	cluster_iterator it = this->getClusterIter();
	HPACluster* cluster = this->clusterIterNext(it);
	while(cluster)
	{
		cluster->setVerbose(getVerbose());
		cluster->setAllowDiagonals(allowDiagonals);
		cluster->buildEntrances(this);
		cluster = this->clusterIterNext(it);
	}
}

/* paths are cached in the direction of the edge (from, to) */
void HPAClusterAbstraction::addPathToCache(edge* e, path* p)
{
	if(e == NULL || p == NULL)
		return;
	
	pathCache[e->getUniqueID()] = p;
}

/* Cache for known paths. Stores one path for each abstract edge. */
path* HPAClusterAbstraction::getPathFromCache(edge* e)
{
	if(e == NULL)
		return 0;

	HPAUtil::pathTable::iterator it = pathCache.find(e->getUniqueID());
	if(it != pathCache.end())
		return (*it).second;
	
	return 0;
}

HPACluster* HPAClusterAbstraction::clusterIterNext(cluster_iterator &iter) const
{
	if (iter != clusters.end())
	{
		HPACluster* c = (*iter).second;
		iter++;
		return c;
	}
  return 0;
}

/* Remove any nodes we added into the abstract graph to facilitate some search 
 * query. 
 * NB: nodes removed in reverse order to creation (i.e. goal, then start)
 */
void HPAClusterAbstraction::removeStartAndGoalNodesFromAbstractGraph() throw(std::runtime_error)
{
	graph* g = abstractions[1];
	ClusterNode* start = dynamic_cast<ClusterNode*>(g->getNode(startid));
	ClusterNode* goal = dynamic_cast<ClusterNode*>(g->getNode(goalid));
	
	//	std::cout << " erasing goal...";
	if(goal)
	{		
		edge_iterator ei = goal->getEdgeIter();
		edge* e = goal->edgeIterNext(ei);
		while(e)
		{
			g->removeEdge(e);
			delete pathCache[e->getUniqueID()];
			pathCache.erase(e->getUniqueID());
			delete e;
			ei = goal->getEdgeIter();
			e = goal->edgeIterNext(ei);
		}
		
		HPACluster* goalCluster = clusters[goal->getParentClusterId()];
		goalCluster->removeParent(goal->getUniqueID());
		g->removeNode(goal->getNum()); 

		goalid = -1;
		node* originalGoal = getNodeFromMap(goal->getLabelL(kFirstData), 
				goal->getLabelL(kFirstData+1));
		originalGoal->setLabelL(kParent, goalid);
		delete goal;
		goal = 0;
	}
	
	//	std::cout << "\n erasing start..";
	if(start)
	{		
		edge_iterator ei = start->getEdgeIter();
		edge* e = start->edgeIterNext(ei);
		while(e)
		{
			delete pathCache[e->getUniqueID()];
			pathCache.erase(e->getUniqueID());
			g->removeEdge(e);
			delete e;
			ei = start->getEdgeIter();
			e = start->edgeIterNext(ei);
		}
		
		HPACluster* startCluster = clusters[start->getParentClusterId()];
		startCluster->removeParent(start->getUniqueID()); 
		g->removeNode(startid); 
		
		startid = -1;
		node* originalStart = getNodeFromMap(start->getLabelL(kFirstData), 
				start->getLabelL(kFirstData+1));
		originalStart->setLabelL(kParent, startid);
		delete start;
		start=0;
	}


}

void HPAClusterAbstraction::insertStartAndGoalNodesIntoAbstractGraph(node* _start, node* _goal) throw(std::invalid_argument)
{
	ClusterNode* start = static_cast<ClusterNode*>(_start);
	ClusterNode* goal = static_cast<ClusterNode*>(_goal);
	
	if(start == NULL || goal == NULL)
		throw std::invalid_argument("insertion error: null start or goal");
		
	if(start->getLabelL(kAbstractionLevel) != 0 || goal->getLabelL(kAbstractionLevel) != 0)
		throw std::invalid_argument("insertion error: start or goal is an abstract node (only non-abstract nodes can be inserted into the abstract graph)");

	nodesExpanded = nodesTouched = nodesGenerated = 0;
	searchTime = 0;

	if(start->getLabelL(kParent) == -1) // only add nodes that don't already exist in the abstract graph
	{	
		ClusterNode* absstart;
		HPACluster* startCluster = clusters[start->getParentClusterId()];
	
		absstart = static_cast<ClusterNode*>(start->clone());
		absstart->setLabelL(kAbstractionLevel, start->getLabelL(kAbstractionLevel)+1);
		abstractions[1]->addNode(absstart);
		startid = absstart->getNum();
		startCluster->addParent(absstart, this);
		start->setLabelL(kParent, startid);
		this->startid = startid;
		int numnodes = abstractions[1]->getNumNodes();
		assert(startid+1 == numnodes);

		nodesExpanded = startCluster->getSearchAlgorithm()->getNodesExpanded();
		nodesTouched = startCluster->getSearchAlgorithm()->getNodesTouched();
		nodesGenerated = startCluster->getSearchAlgorithm()->getNodesGenerated();
		searchTime = startCluster->getSearchAlgorithm()->getSearchTime();
	}
	if(goal->getLabelL(kParent) == -1)
	{
		ClusterNode* absgoal;
		HPACluster* goalCluster = clusters[goal->getParentClusterId()];

		absgoal = static_cast<ClusterNode*>(goal->clone());
		absgoal->setLabelL(kAbstractionLevel, goal->getLabelL(kAbstractionLevel)+1);
		abstractions[1]->addNode(absgoal);
		goalid = absgoal->getNum();
		goalCluster->addParent(absgoal, this);
		goal->setLabelL(kParent, goalid);
		this->goalid = goalid;
		
		nodesExpanded+= goalCluster->getSearchAlgorithm()->getNodesExpanded();
		nodesTouched+= goalCluster->getSearchAlgorithm()->getNodesTouched();
		nodesGenerated+= goalCluster->getSearchAlgorithm()->getNodesGenerated();
		searchTime+= goalCluster->getSearchAlgorithm()->getSearchTime();
	}
}


void HPAClusterAbstraction::openGLDraw()
{
	Map* map = this->getMap();
	map->setDrawLand(false);
	GLdouble xx, yy, zz, zzg, rr;
	
	double depthmod = 0.4;
	for(int level=0; level<2; level++)
	{
		graph* mygraph = getAbstractGraph(level);
		node_iterator ni = mygraph->getNodeIter();	
		node* cur = mygraph->nodeIterNext(ni);
		while(cur)
		{
			glBegin(GL_QUADS);
			int x = cur->getLabelL(kFirstData);
			int y = cur->getLabelL(kFirstData+1);
			map->getOpenGLCoord(x, y, xx, yy, zz, rr);
			zzg = zz-rr*0.5;
			zz = zz-rr*depthmod;

			if(cur->drawColor == 0)
			{
				glColor3f(0.9, 0.9, 0.9);
				glVertex3f(xx-rr, yy-rr, zz+0.01);
				glVertex3f(xx-rr, yy+rr, zz+0.01);
				glVertex3f(xx+rr, yy+rr, zz+0.01);
				glVertex3f(xx+rr, yy-rr, zz+0.01);
			}
			if(cur->drawColor == 1)
			{
				glColor3f(0.4, 0.4, 0.4);
				glVertex3f(xx-rr, yy-rr, zz);
				glVertex3f(xx-rr, yy+rr, zz);
				glVertex3f(xx+rr, yy+rr, zz);
				glVertex3f(xx+rr, yy-rr, zz);
			}
			if(cur->drawColor == 2)
			{
				glColor3f(0.6, 0.6, 0.6);
				glVertex3f(xx-rr, yy-rr, zz);
				glVertex3f(xx-rr, yy+rr, zz);
				glVertex3f(xx+rr, yy+rr, zz);
				glVertex3f(xx+rr, yy-rr, zz);
			}
			glEnd();

			glColor3f (0.5F, 0.5F, 0.5F);
			glLineWidth(0.3f);
			glBegin(GL_LINE_STRIP);
			glVertex3f(xx-rr, yy-rr, zzg);
			glVertex3f(xx+rr, yy-rr, zzg);
			glVertex3f(xx+rr, yy+rr, zzg);
			glVertex3f(xx-rr, yy+rr, zzg);
			glVertex3f(xx-rr, yy-rr, zzg);
			glEnd();

			cur = mygraph->nodeIterNext(ni);
		}
	}
	

	cluster_iterator it = getClusterIter();
	HPACluster *cluster = clusterIterNext(it);
	while(cluster)
	{
		cluster->openGLDraw();
		cluster = clusterIterNext(it);
	}
}

void HPAClusterAbstraction::clearColours()
{
	for(unsigned int i=0; i<getNumAbstractGraphs(); i++)
	{
		graph* g = getAbstractGraph(i);
		node_iterator ni = g->getNodeIter();	
		node* n = g->nodeIterNext(ni);
		while(n)
		{
			n->drawColor = 0;
			n = g->nodeIterNext(ni);
		}
	}
}



// debugging method
void HPAClusterAbstraction::printUniqueIdsOfAllNodesInGraph(graph *g)
{
	node_iterator it = g->getNodeIter();
	node* n = (*it);
	while(n)
	{
		std::cout << "addr: "<<&(*n) << "uid: "<< n->getUniqueID() <<" ("<< n->getLabelL(kFirstData)<<","<< n->getLabelL(kFirstData+1)<<")"<<std::endl;
		n = g->nodeIterNext(it);
	}
}

void HPAClusterAbstraction::print(std::ostream& out)
{
	out << "HPAClusterAbstraction. "<<getNumClusters()<<" clusters.";
	out << " csize: "<<getClusterSize()<<std::endl;
	cluster_iterator it = getClusterIter();
	HPACluster* cluster = clusterIterNext(it);
	while(cluster)
	{
		cluster->print(out);
		out<<std::endl;
		cluster = clusterIterNext(it);
	}

}

// returns heuristic lowerbound on the distance between two tiles.
// if diagonal moves are allowed this is the octile distance.
// if diagonal moves are not allowed, this is the manhattan distance.
double HPAClusterAbstraction::h(node* a, node* b)
{
	if(a == NULL || b == NULL) 
		throw std::invalid_argument("null node");
	
	double answer = 0.0;
	if(getAllowDiagonals())
	{
		int x1 = a->getLabelL(kFirstData);
		int x2 = b->getLabelL(kFirstData);
		int y1 = a->getLabelL(kFirstData+1);
		int y2 = b->getLabelL(kFirstData+1);
		
		const double root2m1 = ROOT_TWO-1;//sqrt(2.0)-1;
		if (fabs(x1-x2) < fabs(y1-y2))
			answer = root2m1*fabs(x1-x2)+fabs(y1-y2);
		else
			answer = root2m1*fabs(y1-y2)+fabs(x1-x2);
	}
	else
	{
		int ax = a->getLabelL(kFirstData);
		int ay = a->getLabelL(kFirstData+1);
		int bx = b->getLabelL(kFirstData);
		int by = b->getLabelL(kFirstData+1);
		//std::cout << "from: "<<ax<<","<<ay<<") to: ("<<bx<<","<<by<<") ";

		int deltax = ax - bx;
		if(deltax < 0) deltax *=-1;

		int deltay = ay - by;
		if(deltay < 0) deltay *=-1;

		//std::cout << "deltax: "<<deltax<<" deltay: "<<deltay<<std::endl;
		answer = deltax + deltay;
	}

	return answer;
}

void HPAClusterAbstraction::verifyClusters()
{
	cluster_iterator iter = getClusterIter();
	HPACluster* cluster = clusterIterNext(iter);
	while(cluster)
	{
		cluster->verifyCluster();
		cluster = clusterIterNext(iter);
	}
}



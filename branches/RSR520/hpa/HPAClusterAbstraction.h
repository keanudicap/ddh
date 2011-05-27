/*
 *  HPAClusterAbstraction.h
 *  hog
 
	Alternative implementation of HPA*.
	Based on code from HAA*.
 *
 *  Created by dharabor on 10/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef HPACLUSTERABSTRACTION_H
#define HPACLUSTERABSTRACTION_H

#include "mapAbstraction.h"
#include "HPAUtil.h"

#include <iostream>
#include <stdexcept>

class IHPAClusterFactory;
class INodeFactory;
class IEdgeFactory;
class HPACluster;
class Map;
class IClusterAStarFactory;
class path;

typedef HPAUtil::clusterTable::const_iterator cluster_iterator;
class HPAClusterAbstraction : public mapAbstraction
{
	#ifdef UNITTEST
		friend class HPAClusterAbstractionTest;
	#endif

	public:
		HPAClusterAbstraction(Map* m, IHPAClusterFactory* cf, INodeFactory* nf, 
				IEdgeFactory* ef, bool allowDiagonals=true) 
			throw(std::invalid_argument);
		virtual ~HPAClusterAbstraction();
		
		virtual void buildEntrances();
		virtual void buildClusters();
		
		/* insert and remove start/goal */
		virtual void insertStartAndGoalNodesIntoAbstractGraph(node*, node*) throw(std::invalid_argument);
		virtual void removeStartAndGoalNodesFromAbstractGraph() throw(std::runtime_error);
		int getStartId() { return startid; }
		int getGoalId() { return goalid; }
		
		/* cluster getters and iterator functions */
		cluster_iterator getClusterIter() const { return clusters.begin(); }
		virtual	HPACluster* clusterIterNext(cluster_iterator&) const;
		virtual HPACluster* getCluster(int cid);
		int getNumClusters() { return clusters.size(); } 

		int getClusterSize() { return clustersize; } 
		void setClusterSize(unsigned int csz) { clustersize = csz; }

		/* measure insertion effort */
		long getNodesExpanded() { return nodesExpanded; }
		void setNodesExpanded(int num) {  nodesExpanded = num; }
		long getNodesTouched() { return nodesTouched; }
		void setNodesTouched(int num) {  nodesTouched = num; }
		long getPeakMemory() { return peakMemory; }
		void setPeakMemory(int num) {  peakMemory = num; }
		double getSearchTime() { return searchTime; }
		void setSearchTime(double newtime) { searchTime = newtime; }

		/* path caching for quick refinement */
		void addPathToCache(edge* e, path* p);
		path* getPathFromCache(edge* e);
		int getPathCacheSize() { return pathCache.size(); }

		/* drawing and overlay methods */
		virtual void openGLDraw(); 
		void clearColours(); // stop visualising nodes expanded/touched

		void setDrawClusters(bool draw) { drawClusters = draw; }
		bool getDrawClusters() { return drawClusters; }
		
		/* all the pure-virtual stuff we need to implement for a concrete mapAbstraction */
		virtual bool pathable(node*, node*) { return false; }
		virtual void verifyHierarchy() {}
		virtual void removeNode(node*) {} 
		virtual void removeEdge(edge*, unsigned int) {}
		virtual void addNode(node*) {}
		virtual void addEdge(edge*, unsigned int) {}
		virtual void repairAbstraction() {}
		virtual mapAbstraction* clone(Map *) { return NULL; }
	
		virtual double h(node *a, node *b);
	
		IEdgeFactory* getEdgeFactory() { return ef; }
		INodeFactory* getNodeFactory() { return nf; }
		inline bool getVerbose() { return verbose; }
		inline void setVerbose(bool _v) { verbose = _v; }

//		inline void setAllowDiagonals(bool value) { allowDiagonals = value; }
		inline bool getAllowDiagonals() { return allowDiagonals; }

		void print(std::ostream& out);
		void verifyClusters();

		
	protected:
		void addCluster(HPACluster* cluster);
		int getNumberOfAbstractionLevels() { return abstractions.size(); }
		void printUniqueIdsOfAllNodesInGraph(graph *g);
	
	private:
		IHPAClusterFactory* cf;
		INodeFactory* nf;
		IEdgeFactory* ef;

		HPAUtil::clusterTable clusters;
		HPAUtil::pathTable pathCache;

		unsigned int clustersize;
		int startid, goalid; // for use during insertion
		bool drawClusters; 

		long nodesExpanded; 
		long nodesTouched; 
		double searchTime;
		long peakMemory;
		bool verbose;
		bool allowDiagonals;
};

#endif

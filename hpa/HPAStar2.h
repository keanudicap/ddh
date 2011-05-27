/*
 *  HPAStar.h
 *  hog
 *
 *  Created by dharabor on 17/11/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HPASTAR2_H
#define HPASTAR2_H

#include "searchAlgorithm.h"

class ExpansionPolicy;
class DebugUtility;
class FlexibleAStar;
class Heuristic;
class HPAClusterAbstraction;
class HPAStar2 : public searchAlgorithm 
{
	public:
		HPAStar2(ExpansionPolicy*, Heuristic*, bool _refine=true, bool _fastRefinement=false);
		virtual ~HPAStar2();
		virtual const char* getName() { return "HPAStar2"; }
		virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);	
		
		long getInsertNodesExpanded() { return insertNodesExpanded; }
		long getInsertNodesTouched() { return insertNodesTouched; }
		long getInsertNodesGenerated() { return insertNodesGenerated; }
		double getInsertSearchTime() { return insertSearchTime; }
		virtual void logFinalStats(statCollection* sc);
		
		void setRefineAbstractPathFlag(bool _refine) { refineAbstractPath = _refine; }
		bool getRefineAbstractPathFlag() { return refineAbstractPath; }
		void setFastRefinement(bool _fastRefinement) { refineAbstractPath = true;  fastRefinement = _fastRefinement; }
		bool getFastRefinement() { return fastRefinement; }

		
	protected:
		virtual path* refinePath(path* abspath, HPAClusterAbstraction* hpamap);
				
	private:		
		void updateMetrics();  
		bool checkParameters(node* from, node* to);
		void resetMetrics();

		// A* implementation being used
		FlexibleAStar* astar;

		// insertion metrics
		long insertNodesExpanded;
		long insertNodesTouched;
		long insertNodesGenerated;
		double insertSearchTime;

		// search options
		bool refineAbstractPath;
		bool fastRefinement; // should we use the path cache in HPAClusterAbstraction to speed up refinement?
		
};

#endif

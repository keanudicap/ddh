#include <deque>
#include <vector>
#include <algorithm>
#include <assert.h>
#include "Entry.h"

#include "constants.h"
#include "graph.h"
#include "map.h"
#include "mapFlatAbstraction.h"
#include "path.h"

#include "EdgeFactory.h"
#include "JumpPointAbstraction.h"
#include "JumpPointSearch.h"
#include "NodeFactory.h"
#include "OctileHeuristic.h"
#include "OctileDistanceRefinementPolicy.h"

#include <iomanip>
#include <iostream>
#include <fstream>

namespace JPS
{
	bool online_search = false;
}

const char *GetName()
{
	if(JPS::online_search)
	{
		return "JPS";
	}
	return "JPS+pre";
}
void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename)
{
	printf("Preprocessing map...");
	if(JPS::online_search)
	{
		std::cout << "not required\n";
	}
	else{
		Map* themap = new Map(width, height, bits);
		JumpPointAbstraction absMap(themap, new NodeFactory(), new EdgeFactory(), false);
		printf("done.\n");

		printf("Writing results to file '%s'...", filename);
		graph* g = absMap.getAbstractGraph(0);
		std::ofstream fout(filename);
		if(fout.fail())
		{
			printf("error. cannot open file; aborting\n");
			exit(1);
		}
		g->print(fout);
		fout.close();
		printf("done\n");
	}
}

void *PrepareForSearch(std::vector<bool> &bits, int w, int h, const char *filename)
{
	mapAbstraction* absMap = 0;
	Map* themap = new Map(w, h, bits);
	if(JPS::online_search)
	{
		printf("PrepareForSearch...");
		absMap = new mapFlatAbstraction(themap, true, false);
		printf("done\n");
	}
	else
	{
		printf("reading from file '%s'...", filename);
		absMap = new JumpPointAbstraction(themap,
				filename, new NodeFactory(), new EdgeFactory(), false);
		printf("done\n");
	}
	return absMap;
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &thepath)
{
	mapAbstraction* absMap = (mapAbstraction*)data;
	bool rdepth = 0;
	JumpPointSearch jps(new OctileHeuristic(), absMap, JPS::online_search, rdepth);
	//printf("running %s", jps.getName());


	node* start = absMap->getNodeFromMap(s.x, s.y);
	node* goal = absMap->getNodeFromMap(g.x, g.y);
	path* p = jps.getPath(absMap, start, goal);

	if(p)
	{
		// p mentions only turning points; add the intermediate nodes
		OctileDistanceRefinementPolicy refPol(absMap);
		path* tmp = refPol.refine(p);
		delete p;
		p = tmp;

		while(p)
		{
			xyLoc loc;
			loc.x = p->n->getLabelL(kFirstData);
			loc.y = p->n->getLabelL(kFirstData+1);
			thepath.push_back(loc);
			p = p->next;
		}

	//	printf("%.3f\n",goal->getLabelF(kTemporaryLabel));
		delete tmp;
	}
//	else
//	{
//		printf("0\n");
//	}

	return true;
}


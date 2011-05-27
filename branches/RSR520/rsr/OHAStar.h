// OHAStar.h
//
// A* variant that speeds up search by operating on an empty cluster
// abstraction.
//
// The main difference between this implementation and the canonical
// implementation is that for each expanded node OHA* must keep track of a 
// "macro parent" node. 
// A node is said to be a macro parent if its expansion involves crossing
// from another cluster to the node's parent cluster.
// The distance to each descendant node in the same cluster is calculated
// with respect to the macro parent.
//
// Also, when extracting the best path (i.e. after the goal is reached)
// we follow macro parents rather than marked edges (as is the case in the
// canonical implementation.
//
// @author: dharabor
// @created: 07/04/2010

#ifndef OHASTAR_H
#define OHASTAR_H

#include "ClusterAStar.h"
#include <ext/hash_map>

class MacroNode;
class OHAStar : public ClusterAStar
{

	public:

		#ifdef UNITTEST
			friend class OHAStarTest;
		#endif

		OHAStar();
		virtual ~OHAStar();
		virtual const char* getName() { return "OHAStar"; }

		inline void setCardinal(bool _cardinal) { cardinal = _cardinal; }
		inline bool getCardinal() { return cardinal; }

		virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);

	protected:
		virtual void expand(node* current, node* to, edge_iterator iter, unsigned int card, 
				heap* openList, std::map<int, node*>& closedList, graph* g);
		virtual void expandMacro(node* current, node* to, heap* openList, std::map<int, node*>& closedList, graph* g);
//		virtual bool evaluate(node* current, node* target, edge* e);
		virtual void relaxEdge(heap *nodeHeap, graph *g, edge *e, int source, 
				int nextNode, node *to);
		virtual void relaxMacro(heap *openList, MacroNode* from, MacroNode* to, node* goal);
		void relaxMacro(heap *openList, graph *g, int fromId, int toId, node* goal);
		virtual path *extractBestPath(graph *g, unsigned int current);
		virtual path* refinePath(path* p);

	private:
		bool cardinal; // if true, diagonal edges are ignored (pretend graph is 4-connected)
		node* closestNeighbour(node* from, node* to);
		__gnu_cxx::hash_map<int, bool> visitedClusters;

};

#endif


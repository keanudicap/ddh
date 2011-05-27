#ifndef PERIMETERSEARCH_H
#define PERIMETERSEARCH_H

#include "ClusterAStar.h"

class MacroNode;
class PerimeterSearch : public ClusterAStar
{
	public:
		PerimeterSearch(); 
		~PerimeterSearch(); 

		virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);


	protected:
		virtual void expand(node* current_, node* goal, edge_iterator begin, unsigned int card, 
				heap* openList, std::map<int, node*>& closedList, graph* g);

	private:
		bool expandSecondary(MacroNode* current, node* goal);
		__gnu_cxx::hash_map<int, bool> visitedClusters;
};

#endif

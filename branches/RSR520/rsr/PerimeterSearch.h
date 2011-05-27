#ifndef PERIMETERSEARCH_H
#define PERIMETERSEARCH_H

#include "ClusterAStar.h"

class PerimeterSearch : public ClusterAStar
{
	public:
		PerimeterSearch(); 
		~PerimeterSearch(); 

	protected:
		virtual void expand(node* current_, node* goal, edge_iterator begin, unsigned int card, 
				heap* openList, std::map<int, node*>& closedList, graph* g);
};

#endif

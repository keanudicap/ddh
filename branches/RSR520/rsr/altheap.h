#ifndef ALTHEAP_H
#define ALTHEAP_H

/* Identical to HOG's regular heap except ties are broken
 * in favour of the node with the smallest gcost.
 * Such nodes end up higher on the heap than other nodes 
 * with identical priority
 *
 * To facilitate the tie breaking altheap adds a ::h function
 * and features a goal node attribute which must be set when
 * the heap is created.
 *
 * @author: dharabor
 * @created: 03/05/2010
 */


#include "heap.h"

class graph_object;
class altheap : public heap
{

	public:	
		altheap(node* goal, int s = DEFAULT_SIZE);
		~altheap(); 

		inline void	setGoal(node* newgoal) { goal = newgoal; } 
		inline node* getGoal() { return goal; } 
		virtual bool rotate(graph_object* first, graph_object* second);

	private:
		double h(node* from, node* to);
		node* goal;
};

#endif


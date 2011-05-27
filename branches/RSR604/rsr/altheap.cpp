#include "altheap.h"
#include "constants.h"
#include "fpUtil.h"
#include "HPAUtil.h"

altheap::altheap(node* goal, int s) : heap(s)
{
	this->goal = goal;
}

altheap::~altheap()
{

}

// returns true when:
//  - priority of second < priority of first
//  - priorities are equal but gcost of second < gcost of first
// 
// otherwise, returns false
bool altheap::rotate(graph_object* first, graph_object* second)
{
  if (fless(second->getKey(), first->getKey()))
	  return true;

  if(fequal(first->getKey(), second->getKey()))
  {
	if(dynamic_cast<node*>(first) && dynamic_cast<node*>(second))
	{
		// break ties in favour of the node with smallest gcost
		double gcost_f = dynamic_cast<node*>(first)->getLabelF(kTemporaryLabel) - 
			HPAUtil::h(dynamic_cast<node*>(first), goal);	
		double gcost_s = dynamic_cast<node*>(second)->getLabelF(kTemporaryLabel) - 
			HPAUtil::h(dynamic_cast<node*>(second), goal);

		if(fless(gcost_s, gcost_f))
			return true;
	}
  }
  return false;
}

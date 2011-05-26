#ifndef DEBUGUTILITY_H
#define DEBUGUTILITY_H

#include <string>

class Heuristic;
class node;
class path;
class graphAbstraction;

class DebugUtility
{
	public:
		DebugUtility(graphAbstraction* map, Heuristic* h);
		virtual ~DebugUtility();
	
		void debugClosedNode(node* c, node* n, double c_to_n_cost, node* goal);
		void printNode(std::string msg, node* n, node* goal=0);
		void printPath(path* p);

	private:
		graphAbstraction* map;
		Heuristic* heuristic;
};

#endif

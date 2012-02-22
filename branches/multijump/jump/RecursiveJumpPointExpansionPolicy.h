#ifndef RECURSIVEJUMPPOINTEXPANSIONPOLICY_H
#define RECURSIVEJUMPPOINTEXPANSIONPOLICY_H

// RecursiveJumpPointExpansionPolicy.h
//
// This expansion policy expands on the ideas implemented in
// JumpPointExpansionPolicy. Specifically:
// Rather than generate the set of immediately adjacent jump points as successors
// we will instead try to jump further when we can prove that the branching
// factor of the currently expanding node is 1.
// To prove such a thing, we employ a recursive procedure that looks
// at the children of each successor node. If none can be found we say that the 
// successor is sterile and reduce the branching factor of the expanding node by 1.
//
// When we can reduce the branching factor of the expanding node to exactly 1, we
// jump past the remaining successor (without generating it) and on to a node that is
// further away -- which we generate instead as a successor.
//
// For theoretical details see: [Harabor & Grastien, 2012]
//
// nstead of generating only the set 
// of adjacent jump points as successors during node expansion, we 
// @author: dharabor
// @created: 06/01/2010

#include "ExpansionPolicy.h"
#include "Jump.h"

#include <ext/hash_map>
#include <vector>
#include <stdexcept>

class JumpPointLocator;
class RecursiveJumpPointExpansionPolicy : 
	public ExpansionPolicy
{
	// a container for storing last_dir values of generated nodes
	typedef __gnu_cxx::hash_map<int, Jump::Direction> DirectionList;

	// A structure for describing jump point search operations.
	class JumpParams
	{
		public:
			JumpParams();
			virtual ~JumpParams();
			JumpParams(JumpParams& other);

			node* origin; // the starting node from which to begin
			node* succ; // a jump point successor of ::origin
			Jump::Direction first_dir; // direction in which to search
			Jump::Direction last_dir; // the last direction of travel before ::succ
			double jumpcost; // cumulative cost of reaching ::succ
			unsigned int depth; // recursion depth at which ::succ was found.
	};


	public:
		RecursiveJumpPointExpansionPolicy(JumpPointLocator* jpl);
		virtual ~RecursiveJumpPointExpansionPolicy();

		void getIntermediateNodes(node** out, int size); 
		int getMaxDepth() { return MAX_DEPTH; } 

		// ExpansionPolicy stuff
		virtual void expand(node* t) throw(std::logic_error);
		virtual node* first();
		virtual node* next();
		virtual node* n();
		virtual double cost_to_n();
		virtual bool hasNext();
		int jumplimit; 

	private:
		void computeNeighbourSet();
		void findJumpNode(Jump::Direction dir, JumpParams& out);
		void addNeighbour(JumpParams& out);
		void label_n();
		Jump::Direction getDirection(node* n);

		// the set of reachable jump points from ::target (and their costs)
		std::vector<JumpParams*> neighbours; // jump point successors for ::target
		DirectionList directions; // last_dir labels for every generated node

		unsigned int neighbourIndex; // index of the neighbour returned by ::n()
		JumpPointLocator* jpl;

		int MAX_NEIS;
		int MAX_NODES;
		int MAX_DEPTH; 
		double ONE_OVER_MAX_NEIS;

		// bookkeeping structures used during expansion operations
		// NB: all_dirs is a bitfield packed as follows:
		// bits 0-7: directions of all forced and natural neighbours
		// bits 8-15: directions in which we've located a jump point successor
		// bits: 16-23: most recent travel direction to reach the current node
		int* all_dirs;
		node** all_nodes;
		double* all_costs; 
};

#endif


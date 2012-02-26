#ifndef JUMPINFO_H
#define JUMPINFO_H

// JumpInfo.h
//
// A structure for describing jump point search operations.
// A single jumping operation may involve several steps/jumps or just one.
//
// Each step is described in terms of three variables:
// 	* a jump node 
// 	* a jump cost (from the immediate predecessor to the current jump node)
// 	* a direction (of travel, taken from the immediate predecessor)
//
// @author: dharabor
// @created: 26/02/2012
//

#include "graph.h"
#include "Jump.h"

#include <vector>

class JumpInfo
{
	public:
		JumpInfo();
		virtual ~JumpInfo();
		JumpInfo(JumpInfo& other);

		inline void clear()
		{
			this->jumpnodes.clear();
			this->jumpdirs.clear();
			this->jumpcosts.clear();
		}

		inline void addJump(node* n, Jump::Direction lastdir, double cost)
		{
			if(!n || lastdir == Jump::NONE)
				return;

			jumpnodes.push_back(n);
			jumpdirs.push_back(lastdir);
			jumpcosts.push_back(cost);
		}

		inline node* getNode(int index) 
		{ 
			if(index < jumpnodes.size()) 
				return jumpnodes.at(index);
			return 0;
		}

		inline Jump::Direction getDirection(int index)
		{
			if(index < jumpdirs.size()) 
				return jumpdirs.at(index);
			return Jump::NONE;
		}

		inline double getCost(int index)
		{
			if(index < jumpcosts.size()) 
				return jumpcosts.at(index);
			return 0;
		}

		inline void setCost(int index, double cost)
		{
			if(index < jumpcosts.size()) 
				jumpcosts[index] = cost;
		}

		inline unsigned int nodecount() 
		{ 
			return jumpnodes.size(); 
		}

		inline unsigned int edgecount()
		{
			return nodecount() - 1;
		}

	private:
		std::vector<node*> jumpnodes;
		std::vector<double> jumpcosts; 
		std::vector<Jump::Direction> jumpdirs; 
};

#endif


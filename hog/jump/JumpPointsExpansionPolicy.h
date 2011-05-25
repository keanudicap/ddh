#ifndef JUMPPOINTSEXPANSIONPOLICY_H
#define JUMPPOINTSEXPANSIONPOLICY_H

// JumpPointsExpansionPolicy.h
//
// This expansion policy reduces the branching factor
// of a node n during search by ignoring any neighbours which
// could be reached by an equivalent (or shorter) path that visits
// the parent of n but not n itself.
//
// An extension of this idea is to generate jump nodes located in the
// same direction as the remaining neighbours. 
//
// TODO: Jump nodes need to be better explained. Perhaps a reference
// to a draft paper.
//
// Based on an idea suggsted by Alban Grastien and Adi Botea.
//
// @author: dharabor
// @created: 06/01/2010

#include "ExpansionPolicy.h"
#include "Jump.h"
#include <vector>
#include <stdexcept>

class JumpPointsExpansionPolicy : public ExpansionPolicy
{

	public:
		JumpPointsExpansionPolicy();
		virtual ~JumpPointsExpansionPolicy();

		virtual void expand(node* t) throw(std::logic_error);
		virtual node* first();
		virtual node* next();
		virtual node* n();
		virtual double cost_to_n();
		virtual bool hasNext();

		int jumplimit; 

	private:
		Jump::Direction directionToParent();
		void computeNeighbourSet();
		node* findJumpNode(Jump::Direction d, int x, int y);

		std::vector<node*> neighbours;
		unsigned int neighbourIndex; 
};

#endif


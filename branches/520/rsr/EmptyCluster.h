/*
 * EmptyCluster.h
 *
 *  Creates a maximal size cluster which is obstacle free. 
 *  Nodes and edges which appear along the perimeter of such clusters are
 *  added to the abstract graph. 
 *
 * 	NB: Clusters are extended in size using a floodfill approach.
 * 	NB2: Every possible transition point is created when an entrance
 * 	between two adjacent clusters is identified.
 *
 * @author: dharabor
 * @created: 10/02/2010
 */

#ifndef EMPTYCLUSTER_H
#define EMPTYCLUSTER_H

#ifdef OS_MAC
	#include "GLUT/glut.h"
	#include <OpenGL/gl.h>
#else
	#include <GL/glut.h>
	#include <GL/gl.h>
#endif

#include "HPACluster.h"
#include <vector>

class graph;
class HPAClusterAbstraction;
class Entrance;
class MacroNode;
class edge;

class EmptyCluster : public HPACluster
{
	#ifdef UNITTEST
		friend class EmptyClusterTest;
	#endif

	public:
		EmptyCluster(const int x, const int y, bool perimeterReduction=true, bool bfReduction = false)
			throw(std::invalid_argument);
		virtual ~EmptyCluster();

		virtual void buildEntrances(HPAClusterAbstraction* hpamap)
			throw(std::invalid_argument);

		virtual void addNodesToCluster(HPAClusterAbstraction*) 
			throw(std::invalid_argument);

		virtual void addNodesToCluster(HPAClusterAbstraction*, int** clearance) 
			throw(std::invalid_argument);

		virtual void addParent(node *, HPAClusterAbstraction*) 
			throw(std::invalid_argument);

		void extend(HPAClusterAbstraction*);
		void extend(HPAClusterAbstraction*, int** clearance);


		MacroNode* nextNodeInColumn(int x, int y, HPAClusterAbstraction* hpamap,
				bool leftToRight);
		MacroNode* nextNodeInRow(int x, int y, HPAClusterAbstraction* hpamap,
				bool topToBottom);

		unsigned int getNumSecondaryEdges() { return secondaryEdges.size(); }
		edge* findSecondaryEdge(unsigned int fromId, unsigned int toId);

		virtual void openGLDraw();

		int macro;

	protected:
		virtual void connectParent(node*, HPAClusterAbstraction*);
		virtual void buildHorizontalEntrances(HPAClusterAbstraction* hpamap);
		virtual void buildVerticalEntrances(HPAClusterAbstraction* hpamap);

		virtual void processVerticalEntrance(HPAClusterAbstraction* hpamap,
				int x, int y, int length);
		virtual void processHorizontalEntrance(HPAClusterAbstraction* hpamap,
				int x, int y, int length);

	private:
		void initOpenGLCoordinates(HPAClusterAbstraction*);
		void frameCluster(HPAClusterAbstraction*);

		void addSingleMacroEdge(node* from, node* to, double weight, 
				graph* absg, bool secondaryEdge = false);
		void addMacroEdges(HPAClusterAbstraction *aMap);
		void addCardinalMacroEdges(HPAClusterAbstraction *aMap);
		void addDiagonalMacroEdges(HPAClusterAbstraction* hpamap);
		void addDiagonalFanMacroEdges(HPAClusterAbstraction* hpamap);

		bool canExtendClearanceSquare(HPAClusterAbstraction* hpamap);
		bool canExtendHorizontally(HPAClusterAbstraction* hpamap); 
		bool canExtendVertically(HPAClusterAbstraction* hpamap);

		bool isIncidentWithInterEdge(node* n, HPAClusterAbstraction* hpamap);
		int findVerticalLength(int x, int y, HPAClusterAbstraction* hpamap);
		int findHorizontalLength(int x, int y, HPAClusterAbstraction* hpamap);

		GLdouble glx, gly, glz;  // OpenGL origin coordinates
		GLdouble glHeight, glWidth;

		std::vector<edge*> secondaryEdges;

		bool perimeterReduction;
		bool bfReduction;
};

#endif


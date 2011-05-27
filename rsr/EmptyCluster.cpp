#include "EmptyCluster.h"

#include "ClusterNode.h"
#include "Entrance.h"
#include "graph.h"
#include "HPAClusterAbstraction.h"
#include "MacroEdge.h"
#include "MacroNode.h"
#include "map.h"

const int DUMMYDIMENSION = 10; // pass this to keep HPACluster constructor happy

EmptyCluster::EmptyCluster(const int x, const int y, bool perimeterReduction_, bool bfReduction_) 
	throw(std::invalid_argument)
	: HPACluster(x, y, DUMMYDIMENSION, DUMMYDIMENSION),
	  perimeterReduction(perimeterReduction_), bfReduction(bfReduction_)
{
	if(x < 0 || y <0)
		throw std::invalid_argument("EmptyCluster: x,y coordinates must be >= 0");

	macro = 0;
}

EmptyCluster::~EmptyCluster()
{
	std::vector<edge*>::iterator iter = secondaryEdges.begin();
	while(iter != secondaryEdges.end())
	{
		edge *e = *iter;
		delete e;
		secondaryEdges.erase(iter);
	}
	//assert(secondaryEdges.size() == 0);
}

void EmptyCluster::addNodesToCluster(HPAClusterAbstraction* aMap, int** clearance)
	throw(std::invalid_argument)
{
	if(aMap == NULL)
		throw std::invalid_argument("EmptyCluster::addNodesToCluster cluster"
			   	"abstraction parameter is null");
	extend(aMap, clearance);

	for(int x=getHOrigin(); x<getHOrigin()+getWidth(); x++)
	{
		for(int y=getVOrigin(); y<getVOrigin()+getHeight(); y++)
		{
			ClusterNode* n = dynamic_cast<ClusterNode*>(aMap->getNodeFromMap(x, y));
			if(n)
			{
				addNode(n);
			}
			else
			{
				throw std::invalid_argument("EmptyCluster: tried to add to "
						"cluster a node of type other than ClusterNode");
			}
		}
	}

	frameCluster(aMap);
	//if(aMap->getAllowDiagonals())
	//	addMacroEdges(aMap);
//	else
//		addCardinalMacroEdges(aMap);
}

void 
EmptyCluster::buildEntrances(HPAClusterAbstraction* hpamap) 
	throw(std::invalid_argument)
{
	if(getVerbose())
	{
		std::cout << "buildEntrances; ";
		print(std::cout);
		std::cout << std::endl;
	}
	HPACluster::buildEntrances(hpamap);

	if(hpamap->getAllowDiagonals())
	{
		buildDiagonalEntrances(hpamap);
		addMacroEdges(hpamap);
	}
	else
		addCardinalMacroEdges(hpamap);
}

void EmptyCluster::addNodesToCluster(HPAClusterAbstraction* aMap)
	throw(std::invalid_argument)
{
	if(aMap == NULL)
		throw std::invalid_argument("EmptyCluster::addNodesToCluster cluster"
			   	"abstraction parameter is null");
	extend(aMap);

	if(getVerbose())
	{
		std::cout << "cheight: "<<getHeight()<<" cwidth: ";
		std::cout <<getWidth()<<std::endl;
	}

	for(int x=getHOrigin(); x<getHOrigin()+getWidth(); x++)
	{
		for(int y=getVOrigin(); y<getVOrigin()+getHeight(); y++)
		{
			ClusterNode* n = dynamic_cast<ClusterNode*>(aMap->getNodeFromMap(x, y));
			if(n)
			{
				addNode(n);
			}
			else
			{
				throw std::invalid_argument("EmptyCluster: tried to add to "
						"cluster a node of type other than ClusterNode");
			}
		}
	}

	frameCluster(aMap);
	//addMacroEdges(aMap);
}


// Every node framing the cluster is added to the abstract graph. Also, 
// every edge between two adjacent framed nodes will appear in the graph.
// NB: there is a special case where the cluster has size 1x1.
// In this case the single node will be added to the graph when 
// buildHorizontalEntrances and buildVerticalEntrances are called
// rather than by this method
void EmptyCluster::frameCluster(HPAClusterAbstraction* aMap)
{
        if(getVerbose())
                std::cout << "frameCluster "<<this->getId()<<std::endl;
		graph* absg = aMap->getAbstractGraph(1);

        // add nodes along left border
        MacroNode* first = 0;
        MacroNode* last = 0;
        int x = this->getHOrigin();
        int y = this->getVOrigin();
        for( ; y<this->getVOrigin()+this->getHeight(); y++)
        {
			MacroNode* n = dynamic_cast<MacroNode*>(aMap->getNodeFromMap(x,y));
			//assert(n);
			if(!perimeterReduction || (isIncidentWithInterEdge(n, aMap) && perimeterReduction))
			{
				addParent(n, aMap);
				if(last)
				{
//					if(n->getUniqueID() != last->getUniqueID() &&
//							this->isIncidentWithInterEdge(n, aMap))
					if(n->getUniqueID() != last->getUniqueID() &&
					     ((isIncidentWithInterEdge(last, aMap) && perimeterReduction) ||
						 !perimeterReduction))
					{
						addSingleMacroEdge(
								absg->getNode(n->getLabelL(kParent)), 
								absg->getNode(last->getLabelL(kParent)), 
								aMap->h(n, last), absg);
						last = n;
					}
				}
				else
				{
					first = n;
					last = n;
				}
			}
        }

        // add nodes along bottom border
        y = this->getVOrigin()+this->getHeight()-1;
        for(x=this->getHOrigin()+1; x<this->getHOrigin()+this->getWidth(); x++)
        {
			MacroNode* n = dynamic_cast<MacroNode*>(aMap->getNodeFromMap(x,y));
			//assert(n);
			if(!perimeterReduction || (isIncidentWithInterEdge(n, aMap) && perimeterReduction))
			{
				addParent(n, aMap);
				if(last)
				{
					if(n->getUniqueID() != last->getUniqueID() &&
					     ((isIncidentWithInterEdge(last, aMap) && perimeterReduction) ||
						 !perimeterReduction))
					{
						addSingleMacroEdge(
								absg->getNode(n->getLabelL(kParent)), 
								absg->getNode(last->getLabelL(kParent)), 
								aMap->h(n, last), absg);
						last = n;
					}
				}
				else
				{
					first = n;
					last = n;
				}
			}
        }

        // don't try to add edges twice if the cluster has dimension = 1
        if(getWidth() > 1 && getHeight() > 1)
        {
			// add nodes along right border
			x = this->getHOrigin()+this->getWidth()-1;
			for(y=this->getVOrigin()+this->getHeight()-2; y>=this->getVOrigin(); y--)
			{
				MacroNode* n = dynamic_cast<MacroNode*>(aMap->getNodeFromMap(x,y));
				//assert(n);
				if(!perimeterReduction || (isIncidentWithInterEdge(n, aMap) && perimeterReduction))
				{
					addParent(n, aMap);
					if(last)
					{
						//if(n->getUniqueID() != last->getUniqueID() &&
						//		this->isIncidentWithInterEdge(n, aMap))
						if(n->getUniqueID() != last->getUniqueID() &&
							 ((isIncidentWithInterEdge(last, aMap) && perimeterReduction) ||
							 !perimeterReduction))
						{
							addSingleMacroEdge(
								absg->getNode(n->getLabelL(kParent)), 
								absg->getNode(last->getLabelL(kParent)), 
								aMap->h(n, last), absg);
							last = n;
						}
					}
					else
					{
						first = n;
						last = n;
					}
				}
			}

			// add nodes along top border
			y = this->getVOrigin();
			for(x=this->getHOrigin()+this->getWidth()-2; x>=this->getHOrigin()+1; x--)
			{
				MacroNode* n = dynamic_cast<MacroNode*>(aMap->getNodeFromMap(x,y));
				//assert(n);
				if(!perimeterReduction || (isIncidentWithInterEdge(n, aMap) && perimeterReduction))
				{
					addParent(n, aMap);
					if(last)
					{
//						if(n->getUniqueID() != last->getUniqueID() &&
//								this->isIncidentWithInterEdge(n, aMap))
						if(n->getUniqueID() != last->getUniqueID() &&
							 ((isIncidentWithInterEdge(last, aMap) && perimeterReduction) ||
							 !perimeterReduction))
						{
							addSingleMacroEdge(
								absg->getNode(n->getLabelL(kParent)), 
								absg->getNode(last->getLabelL(kParent)), 
								aMap->h(n, last), absg);
							last = n;
						}
					}
					else
					{
						first = n;
						last = n;
					}
				}
			}

			// connect the first and last nodes to be considered
			if(first && last && first->getUniqueID() != last->getUniqueID())
			{
				addSingleMacroEdge(
						absg->getNode(first->getLabelL(kParent)), 
						absg->getNode(last->getLabelL(kParent)), 
						aMap->h(first, last), absg);
			}
        }
}

// use this when working with 4-connected grid maps
// (see [Harabor & Botea, 2010])
void EmptyCluster::addCardinalMacroEdges(HPAClusterAbstraction *aMap)
{
//	this->print(std::cout);
//	std::cout << std::endl;
	if(getVerbose())
	{
		std::cout << "adding cardinal macro edges for cluster "<<getId()<<" origin ";
		std::cout <<"("<< getHOrigin()<<", "<<getVOrigin()<<")";
		std::cout <<" diagonal edges allowed? "<<getAllowDiagonals()<< " "<<std::endl;
	}
	graph* absg = aMap->getAbstractGraph(1);
	macro = 0;

	// connect nodes on directly opposite sides of the perimeter
	if(this->getWidth() > 1)
	{
		int lx = this->getHOrigin();
		int rx = lx + this->getWidth()-1;
		for(int y=this->getVOrigin(); y<this->getVOrigin()+this->getHeight(); y++)
		{
			node *left = absg->getNode(
					aMap->getNodeFromMap(lx, y)->getLabelL(kParent));
			node *right = absg->getNode(
					aMap->getNodeFromMap(rx, y)->getLabelL(kParent));
			if(left && right)
				addSingleMacroEdge(left, right, aMap->h(left, right), absg, true);
			else if(left && !right)
			{	
				right = nextNodeInColumn(rx, y, aMap, false);
				if(right)
					addSingleMacroEdge(left, right, aMap->h(left, right), absg, true);
				right = nextNodeInColumn(rx, y, aMap, true);
				if(right)
					addSingleMacroEdge(left, right, aMap->h(left, right), absg, true);
			}
			else if(!left && right)
			{
				left = nextNodeInColumn(lx, y, aMap, false);
				if(left)
					addSingleMacroEdge(left, right, aMap->h(left, right), absg, true);
				left = nextNodeInColumn(lx, y, aMap, true);
				if(left)
					addSingleMacroEdge(left, right, aMap->h(left, right), absg, true);
			}
		}
	}

	if(this->getHeight() > 1)
	{
		int ty = this->getVOrigin();
		int by = this->getVOrigin()+this->getHeight()-1;
		for(int x=this->getHOrigin(); x<this->getHOrigin()+this->getWidth(); x++)
		{
			node *top = absg->getNode(
					aMap->getNodeFromMap(x, ty)->getLabelL(kParent));
			node *bottom = absg->getNode(
					aMap->getNodeFromMap(x, by)->getLabelL(kParent));
			if(top && bottom)
				addSingleMacroEdge(top, bottom, aMap->h(top, bottom), absg, true);
			else if(top && !bottom)
			{
				bottom = nextNodeInRow(x, by, aMap, false);
				if(bottom)
					addSingleMacroEdge(top, bottom, aMap->h(top, bottom), absg, true);
				bottom = nextNodeInRow(x, by, aMap, true);
				if(bottom)
					addSingleMacroEdge(top, bottom, aMap->h(top, bottom), absg, true);
			}
			else if(!top && bottom)
			{
				top = nextNodeInRow(x, ty, aMap, false);
				if(top)
					addSingleMacroEdge(top, bottom, aMap->h(top, bottom), absg, true);
				top = nextNodeInRow(x, ty, aMap, true);
				if(top)
					addSingleMacroEdge(top, bottom, aMap->h(top, bottom), absg, true);
			}
		}
	}

	if(getVerbose())
		std::cout << macro << " macro edges added for cluster "<<getId()<<std::endl;
}


void
EmptyCluster::addDiagonalMacroEdges(HPAClusterAbstraction* aMap)
{
	graph* absg = aMap->getAbstractGraph(1);
	macro = 0;

	// first, add diagonal edges between nodes on orthogonal sides of the 
	// cluster
	int max = this->getHeight() > this->getWidth()?
		this->getWidth():this->getHeight();
	for(int offset=1; offset<max; offset++)
	{
		// connect left and bottom sides of cluster
		int fx = this->getHOrigin()+offset;
		int fy = this->getVOrigin();
		int sx = this->getHOrigin();
		int sy = this->getVOrigin()+offset;
		node *first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));
		node *second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));
		if(first && second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		else if(!first && second)
		{
			first = nextNodeInRow(fx, fy, aMap, true);
			if(first)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}
		else if(!second && first)
		{
			second = nextNodeInColumn(sx, sy, aMap, true);
			if(second)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}

		// connect top and right sides of cluster
		fx = this->getHOrigin()+this->getWidth()-1-offset;
		sx = this->getHOrigin()+this->getWidth()-1;
		first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));
		second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));
		if(first && second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		else if(!first && second)
		{
			first = nextNodeInRow(fx, fy, aMap, false);
			if(first)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}
		else if(first && !second)
		{
			second = nextNodeInColumn(sx, sy, aMap, true);
			if(second)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}

		// connect left and bottom sides of cluster
		fx = this->getHOrigin()+offset;
		fy = this->getVOrigin()+getHeight()-1;
		sx = this->getHOrigin();
		sy = this->getVOrigin()+this->getHeight()-1-offset;
		first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));
		second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));
		if(first && second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		else if(!first && second)
		{
			first = nextNodeInRow(fx, fy, aMap, true);
			if(first)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}
		else if(first && !second)
		{
			second = nextNodeInColumn(sx, sy, aMap, false);
			if(second)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}

		// connect bottom and right sides sides of cluster
		fx = this->getHOrigin()+this->getWidth()-1;
		fy = this->getVOrigin()+getHeight()-1-offset;
		sx = this->getHOrigin()+this->getWidth()-1-offset;
		sy = this->getVOrigin()+this->getHeight()-1;
		first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));
		second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));
		if(first && second) 
			addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		else if(!first && second)
		{
			first = nextNodeInColumn(fx, fy, aMap, false);
			if(first)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}
		else if(first && !second)
		{
			second = nextNodeInRow(sx, sy, aMap, false);
			if(second)
				addSingleMacroEdge(first, second, aMap->h(first, second), absg);
		}
	}

}

void
EmptyCluster::addDiagonalFanMacroEdges(HPAClusterAbstraction* aMap)
{
	graph* absg = aMap->getAbstractGraph(1);
	macro = 0;


	// add edges connecting nodes on the top side to nodes on the bottom 
	// side of the cluster
	int max = this->getHeight() > this->getWidth()?
		this->getWidth():this->getHeight();
	max--;  // max # diagonal steps we can take in crossing this cluster
	if(getVerbose())
	{
		std::cout << "adding fan edges between top and bottom"<<std::endl;
		std::cout << "max diagonal steps: "<<max<<std::endl;
	}
	for(int fx=this->getHOrigin(); 
			fx<this->getHOrigin()+this->getWidth();
			fx++)
	{
		int fy = this->getVOrigin();
		node* first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));

		if(!first) continue;

		// nodes in the fan are in the range [minx, maxx] 
		int minx = (fx-max)<this->getHOrigin()?
			(this->getHOrigin()):(fx-max); 
		int maxx = (fx+max)>(this->getHOrigin()+this->getWidth()-1)?
			(this->getHOrigin()+this->getWidth()-1):(fx+max);

		int sx = minx+1;
		int sy = this->getVOrigin()+this->getHeight()-1;
		node* second = 0;
		for( ; sx <= maxx-1; sx++)
		{
			second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));

			if(!second) continue;
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
		}
		
		// check if nodes exist at fan edges; if not, extend the fan area until we hit something
		second = nextNodeInRow(minx, sy, aMap, false);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);

		second = nextNodeInRow(maxx, sy, aMap, true);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
	}

	// when using perimeter reduction we also need to connect bottom to top
	// in this case we only try to connect neighbours at the edge of the fan
	if(getVerbose())
	{
		std::cout << "adding fan edges between bottom and top"<<std::endl;
		std::cout << "max diagonal steps: "<<max<<std::endl;
	}
	for(int fx=this->getHOrigin();
			fx<this->getHOrigin()+this->getWidth();
			fx++)
	{
		int fy = this->getVOrigin()+this->getHeight()-1;
		node* first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));

		if(!first) continue;

		// nodes in the fan are in the range [minx, maxx] 
		int minx = (fx-max)<this->getHOrigin()?
			(this->getHOrigin()):(fx-max); 
		int maxx = (fx+max)>(this->getHOrigin()+this->getWidth()-1)?
			(this->getHOrigin()+this->getWidth()-1):(fx+max);

		int sy = this->getVOrigin();
		node* second = 0;
		
		// check if nodes exist at fan edges; if not, extend the fan area until we hit something
		second = nextNodeInRow(minx, sy, aMap, false);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);

		second = nextNodeInRow(maxx, sy, aMap, true);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
	}

	if(getVerbose())
	{
		std::cout << "adding fan edges between left and right"<<std::endl;
	}
	// add edges connecting nodes on the left side to nodes on the right 
	// side of the cluster
	for(int fy=this->getVOrigin();
			fy<this->getVOrigin()+this->getHeight();
			fy++)
	{
		int fx = this->getHOrigin();
		node* first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));

		if(!first) continue;

		int miny = (fy-max)<this->getVOrigin()?
			(this->getVOrigin()):(fy-max); 
		int maxy = (fy+max)>(this->getVOrigin()+this->getHeight()-1)?
			(this->getVOrigin()+this->getHeight()-1):(fy+max);

		int sx = this->getHOrigin()+this->getWidth()-1;
		int sy = miny;
		sy = miny+1;
		node* second = 0;
		for( ; sy <= maxy-1; sy++) 
		{
			second = absg->getNode(
				aMap->getNodeFromMap(sx, sy)->getLabelL(kParent));

			if(!second) continue;
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
		}
		
		// check if nodes exist at fan edges; if not, extend the fan area until we hit something
		second = nextNodeInColumn(sx, miny, aMap, false);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);

		second = nextNodeInColumn(sx, maxy, aMap, true);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
	}

	if(getVerbose())
	{
		std::cout << "adding fan edges between right and left"<<std::endl;
	}
	// add edges connecting nodes on the left side to nodes on the right 
	// side of the cluster
	for(int fy=this->getVOrigin();
			fy<this->getVOrigin()+this->getHeight();
			fy++)
	{
		int fx = this->getHOrigin()+this->getWidth()-1;
		node* first = absg->getNode(
				aMap->getNodeFromMap(fx, fy)->getLabelL(kParent));

		if(!first) continue;

		int miny = (fy-max)<this->getVOrigin()?
			(this->getVOrigin()):(fy-max); 
		int maxy = (fy+max)>(this->getVOrigin()+this->getHeight()-1)?
			(this->getVOrigin()+this->getHeight()-1):(fy+max);

		int sx = this->getHOrigin();
		node* second = 0;
		
		// check if nodes exist at fan edges; if not, extend the fan area until we hit something
		second = nextNodeInColumn(sx, miny, aMap, false);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);

		second = nextNodeInColumn(sx, maxy, aMap, true);
		if(second)
			addSingleMacroEdge(first, second, aMap->h(first, second), absg, true);
	}
}

// use this when working with 8-connected grid maps
void EmptyCluster::addMacroEdges(HPAClusterAbstraction *aMap)
{
	if(getVerbose())
	{
		std::cout << "adding macro edges for cluster "<<getId()<<" origin ";
		std::cout <<"("<< getHOrigin()<<", "<<getVOrigin()<<")";
		std::cout <<" diagonal edges allowed? "<<getAllowDiagonals()<< " "<<std::endl;
	}

	if(this->getHeight() > 1 && this->getWidth() > 1)
	{
		addDiagonalMacroEdges(aMap);
		addDiagonalFanMacroEdges(aMap);
	}

	if(getVerbose())
		std::cout << macro << " macro edges added for cluster "<<getId()<<std::endl;
}

// returns true if node n_ has a neighbour with a different parent cluster. 
bool EmptyCluster::isIncidentWithInterEdge(node* n_, HPAClusterAbstraction* hpamap)
{
	bool retVal = false;
	MacroNode* n = static_cast<MacroNode*>(n_);
	//assert(n);

	int nx = n->getLabelL(kFirstData);
	int ny = n->getLabelL(kFirstData+1);
	for(int nbx = nx-1; nbx < nx+2; nbx++)
		for(int nby = ny-1; nby < ny+2; nby++)
		{
			MacroNode* nb = static_cast<MacroNode*>(
					hpamap->getNodeFromMap(nbx, nby));
			if(!getAllowDiagonals() && nbx != nx && nby != ny)
				continue;
			else if(nb && 
					nb->getParentClusterId() != n->getParentClusterId())
			{
				retVal = true;
			}
		}
	
//	if(getVerbose())
//	{
//		std::cout << "checking incidence w/ inter edge for node: ";
//		n->print(std::cout);
//		std::cout << retVal <<std::endl;
//	}

	return retVal;
}

void EmptyCluster::addSingleMacroEdge(node* from_, node* to_, double weight, graph* absg, bool secondaryEdge)
{
	//assert(from_ && to_);
	//assert(from_->getUniqueID() != to_->getUniqueID());

	MacroNode* from = static_cast<MacroNode*>(from_);
	MacroNode* to = static_cast<MacroNode*>(to_);


	//assert(from && to);
	//assert(from->getParentClusterId() == to->getParentClusterId());

	MacroEdge* e = static_cast<MacroEdge*>(
			absg->findEdge(from->getNum(), to->getNum()));
	if(e == 0 && from->getParentClusterId() == to->getParentClusterId())
	{
		e = static_cast<MacroEdge*>(findSecondaryEdge(from->getNum(), to->getNum()));
		if(e == 0 && secondaryEdge && bfReduction)
		{
			e = new MacroEdge(from->getNum(), to->getNum(), weight);
			from->addSecondaryEdge(e);
			to->addSecondaryEdge(e);
			secondaryEdges.push_back(e);
		}
		else
		{
			e = new MacroEdge(from->getNum(), to->getNum(), weight);
			absg->addEdge(e);
		}

		macro++;
		if(getVerbose())
		{
			std::cout << "added";
			if(secondaryEdge)
				std::cout << " secondary ";
			std::cout << " macro edge: [("<<from->getLabelL(kFirstData)<<", ";
			std::cout <<from->getLabelL(kFirstData+1)<<") <-> (";
			std::cout << to->getLabelL(kFirstData) << ", ";
			std::cout << to->getLabelL(kFirstData+1);
			std::cout <<") wt: "<<weight<< " ] "<<std::endl;
		}
	}
	else
	{
		e->setSecondary(secondaryEdge);
	}
}

void EmptyCluster::extend(HPAClusterAbstraction* aMap)
{
	setWidth(1);
	setHeight(1);
	Map* themap = aMap->getMap();
	int hsize=0;
	int y = this->getVOrigin();
	for(int x=this->getHOrigin(); x<themap->getMapWidth(); x++)
	{
		ClusterNode *n = dynamic_cast<ClusterNode*>(aMap->getNodeFromMap(x,y));
		if(!n || n->getParentClusterId() != -1)
			break;
		hsize++;
	}
	
	this->setWidth(hsize);
	int vsize=0;
	for( ; y<themap->getMapHeight(); y++)
	{
		bool rowok=true;
		for(int x=this->getHOrigin(); x<this->getHOrigin()+this->getWidth(); x++)
		{
			ClusterNode *n = dynamic_cast<ClusterNode*>(aMap->getNodeFromMap(x,y));
			if(getVerbose())
			{
				if(n)
					std::cout << "("<<x<<", "<<y<<") is ok!"<<std::endl;
			}
			if(!n || n->getParentClusterId() != -1)
			{
				rowok=false;
			}
		}
		if(!rowok)
			break;
		vsize++;
	}
	this->setHeight(vsize);

	initOpenGLCoordinates(aMap);
}

void EmptyCluster::extend(HPAClusterAbstraction* aMap, int** clearance)
{
	ClusterNode* n = dynamic_cast<ClusterNode*>(aMap->getNodeFromMap(getHOrigin(), getVOrigin()));
	if(!n || n->getParentClusterId() != -1)
	{
		setHeight(0);
		setWidth(0);
		std::cout << "WARNING!! aborting cluster creation; origin is invalid or already assigned!!"<<std::endl;
		return;
	}

	setWidth(1);
	setHeight(1);
	//std::cout << "cluster: "<<getHOrigin()<<", "<<getVOrigin()<<" ht: "<<getHeight()<<" wt: "<<getWidth()<<std::endl;
	while(canExtendClearanceSquare(aMap))
	{
		setHeight(getHeight() + 1);
		setWidth(getWidth() + 1);
	}
	//std::cout << "csq done; height: "
	//	<<getHeight()<<" width: "<<getWidth()<<std::endl;


	while(canExtendHorizontally(aMap))
	{
		setWidth(getWidth() + 1);
	//	std::cout << " extending horizontally!"<<std::endl;
	}

	while(canExtendVertically(aMap))
	{
		setHeight(getHeight() + 1);
	//	std::cout << " extending vertically!"<<std::endl;
	}

	initOpenGLCoordinates(aMap);
}

void EmptyCluster::initOpenGLCoordinates(HPAClusterAbstraction* aMap)
{
	Map* map = aMap->getMap();
	GLdouble xx, yy, zz,rr;

	map->getOpenGLCoord(this->getHOrigin(), this->getVOrigin(), xx, yy, zz, rr);
	glx = xx;
	gly = yy;
	glz = zz-rr*0.5;

	map->getOpenGLCoord(this->getHOrigin()+this->getWidth()-1, 
			this->getVOrigin(), xx, yy, zz, rr);
	glWidth = xx - glx;

	map->getOpenGLCoord(this->getHOrigin(), this->getVOrigin()+this->getHeight()-1, 
			xx, yy, zz, rr);
	glHeight = yy - gly;

	glx-=rr;
	gly-=rr;
	glHeight+=2*rr;
	glWidth+=2*rr;
}

void EmptyCluster::openGLDraw()
{
//	glColor3f (0.6F, 0.9F, 0.4F);
	glColor3f (0.0F, 0.6F, 0.0F);
	//glColor3f(0.1, 0.1, 0.7);
	glLineWidth(3.0f);
	glBegin(GL_LINE_STRIP);
	glVertex3f(glx, gly, glz);
	glVertex3f(glx+glWidth, gly, glz);
	glVertex3f(glx+glWidth, gly+glHeight, glz);
	glVertex3f(glx, gly+glHeight, glz);
	glVertex3f(glx, gly, glz);
	glEnd();
	glLineWidth(1.0f);
}

void EmptyCluster::connectParent(node*, HPAClusterAbstraction*)
{
}

// identifies entrances between horizontally adjacent clusters.
// an abstract node is added for each node in the entrance area
// and an abstract edge is added for each (straight) edge in the entrance area 
// connecting the two clusters.
//
// if 'getAllowDiagonals()' is set we also add an abstract edge for each diagonal
// edge in the entrance area between the two clusters
void EmptyCluster::processHorizontalEntrance(
		HPAClusterAbstraction* hpamap, int x, int y, int length)
{
	if(getVerbose())
	{
		std::cout << "hEnt at "<<x<<", "<<y<<" of length: "<<length<<std::endl;
		std::cout << "straight transitions: "<<std::endl;
	}

	// process straight transitions
	for(int xprime=x; xprime <x+length; xprime++)
	{	
		node* endpoint1 = hpamap->getNodeFromMap(xprime, y); 
		node* endpoint2 = hpamap->getNodeFromMap(xprime, y-1);
		this->addTransitionPoint(endpoint1, endpoint2, hpamap);
	}

	if(getAllowDiagonals())
	{
		if(getVerbose())
		{
			std::cout << "first set of diagonals: "<<std::endl;
		}

		for(int xprime=x+1; xprime <x+length; xprime++)
		{	
			node* endpoint1 = hpamap->getNodeFromMap(xprime, y); 
			node* endpoint2 = hpamap->getNodeFromMap(xprime-1, y-1);
			//assert(endpoint1 && endpoint2);

			this->addTransitionPoint(endpoint1, endpoint2, hpamap);
		}

		if(getVerbose())
		{
			std::cout << "second set of diagonals: "<<std::endl;
		}

		for(int xprime=x+1; xprime <x+length; xprime++)
		{	
			node* endpoint1 = hpamap->getNodeFromMap(xprime, y-1); 
			node* endpoint2 = hpamap->getNodeFromMap(xprime-1, y);
			//assert(endpoint1 && endpoint2);

			this->addTransitionPoint(endpoint1, endpoint2, hpamap);
		}
	}

}

void EmptyCluster::buildHorizontalEntrances(HPAClusterAbstraction* hpamap)
{
	if(getVerbose())
		std::cout << "buildHorizontalEntrances"<<std::endl;
	int mapheight = hpamap->getMap()->getMapHeight();
	int y = this->getVOrigin()+this->getHeight();
	if(y == mapheight)
		return; 

	// scan for vertical entrances along the eastern border 
	int x = this->getHOrigin();
	while(x < this->getHOrigin()+this->getWidth())
	{
		int length = findHorizontalEntranceLength(x,y, hpamap);
	
		if(length == 0)
			x++;
		else
		{
			// add inter-edges to connect adjacent clusters 
			processHorizontalEntrance(hpamap, x, y, length);
			x += length;
		}
	}
}

// each node in the veritcal entrance area is represented by a node in 
// the abstract graph. 
// further, each (straight) edge in the entrance area, connecting the two
// adjacent clusters, is represented by a node in the abstract graph.
//
// if 'getAllowDiagonals()' is true, every diagonal edge connecting the two
// adjacent clusters is represented by an edge in the abstract graph.
void EmptyCluster::processVerticalEntrance(
		HPAClusterAbstraction* hpamap, int x, int y, int length)
{
	if(getVerbose())
	{
		std::cout << "vEnt at "<<x<<", "<<y<<" of length: "<<length<<std::endl;
		std::cout << "straight transitions:"<<std::endl;
	}
	
	// first add straight transitions
	for(int yprime=y; yprime <y+length; yprime++)
	{
		node* endpoint1 = hpamap->getNodeFromMap(x, yprime); 
		node* endpoint2 = hpamap->getNodeFromMap(x-1, yprime);
		this->addTransitionPoint(endpoint1, endpoint2, hpamap);
	}

	if(getAllowDiagonals())
	{
		if(getVerbose())
		{
			std::cout << "first set of diagonals: "<<std::endl;
		}

		for(int yprime=y+1; yprime <y+length; yprime++)
		{	
			node* endpoint1 = hpamap->getNodeFromMap(x, yprime); 
			node* endpoint2 = hpamap->getNodeFromMap(x-1, yprime-1);
			//assert(endpoint1 && endpoint2);
			this->addTransitionPoint(endpoint1, endpoint2, hpamap);
		}

		if(getVerbose())
		{
			std::cout << "second set of diagonals: "<<std::endl;
		}

		for(int yprime=y+1; yprime <y+length; yprime++)
		{	
			node* endpoint1 = hpamap->getNodeFromMap(x-1, yprime); 
			node* endpoint2 = hpamap->getNodeFromMap(x, yprime-1);
			//assert(endpoint1 && endpoint2);
				
			this->addTransitionPoint(endpoint1, endpoint2, hpamap);
		}
	}
	
}

void EmptyCluster::buildVerticalEntrances(HPAClusterAbstraction* hpamap)
{
	if(getVerbose())
		std::cout << "buildHorizontalEntrances"<<std::endl;
	int mapwidth = hpamap->getMap()->getMapWidth();
	int x = this->getHOrigin()+this->getWidth();
	if(x == mapwidth)
		return; 

	// scan for vertical entrances along the eastern border 
	int y = this->getVOrigin();
	while(y < this->getVOrigin()+this->getHeight())
	{
		int length = findVerticalEntranceLength(x,y, hpamap);
	
		if(length == 0)
			y++;
		else
		{
			processVerticalEntrance(hpamap, x, y, length);
			y += length;
		}
	}
}

bool EmptyCluster::canExtendClearanceSquare(HPAClusterAbstraction* hpamap)
{
	int x = getHOrigin() + getWidth();
	int y = getVOrigin();
	for( ; y <= getVOrigin() + getHeight(); y++)
	{
		ClusterNode* n = dynamic_cast<ClusterNode*>(hpamap->getNodeFromMap(x, y));
		if(!n || n->getParentClusterId() != -1)
			return false;
	}

	x = getHOrigin();
	y = getVOrigin() + getHeight();
	for( ; x <= getHOrigin() + getWidth(); x++)
	{
		ClusterNode* n = dynamic_cast<ClusterNode*>(hpamap->getNodeFromMap(x, y));
		if(!n || n->getParentClusterId() != -1)
			return false;
	}

	return true;
}

bool EmptyCluster::canExtendHorizontally(HPAClusterAbstraction* hpamap)
{
	int x = getHOrigin() + getWidth();
	for(int y = getVOrigin(); y<getVOrigin() + getHeight(); y++)
	{
		ClusterNode* n = dynamic_cast<ClusterNode*>(hpamap->getNodeFromMap(x, y));
		if(!n || n->getParentClusterId() != -1)
			return false;
	}
	return true;
}

bool EmptyCluster::canExtendVertically(HPAClusterAbstraction* hpamap)
{
	int y = getVOrigin() + getHeight();
	for(int x = getHOrigin(); x<getHOrigin() + getWidth(); x++)
	{
		ClusterNode* n = dynamic_cast<ClusterNode*>(hpamap->getNodeFromMap(x, y));
		if(!n || n->getParentClusterId() != -1)
			return false;
	}
	return true;

}

// returns the length of a contiguous horizontal entrance area starting at
// coordinate (x, y)
int EmptyCluster::findHorizontalLength(int x, int y, 
		HPAClusterAbstraction* hpamap)
{
	int length = 0;
	for(int xprime = x;
		   	xprime < (this->getHOrigin()+this->getWidth());
		   	xprime++)
	{
		node* n = hpamap->getNodeFromMap(xprime, y);
		if(isIncidentWithInterEdge(n, hpamap))
		{
			if(n->getLabelL(kParent) == -1)
				std::cout << " \nnode has no abs parent: "<<n->getLabelL(kFirstData)<<", "<<n->getLabelL(kFirstData+1)<<std::endl;
			//assert(n->getLabelL(kParent) != -1);
			length++;
		}
		else
			break;	
	}
	return length;
}

// returns the length of a contiguous vertical entrance area starting at
// coordinate (x, y)
int EmptyCluster::findVerticalLength(int x, int y, 
		HPAClusterAbstraction* hpamap)
{
	int length = 0;
	for(int yprime = y; 
			yprime < (this->getVOrigin()+this->getHeight());
		   	yprime++)
	{
		node* n = hpamap->getNodeFromMap(x, yprime);
		if(isIncidentWithInterEdge(n, hpamap))
		{
			//assert(n->getLabelL(kParent) != -1);
			length++;
		}
		else
			break;	
	}
	return length;
}

MacroNode*
EmptyCluster::nextNodeInRow(int x, int y, HPAClusterAbstraction* hpamap,
		bool leftToRight)
{
	MacroNode* retVal = 0;
	graph* absg = hpamap->getAbstractGraph(1);
	if(leftToRight)
	{
		for( ; x < (this->getHOrigin() + this->getWidth()) ; x++)
		{
			int nodeId = hpamap->getNodeFromMap(x, y)->getLabelL(kParent);
			if(nodeId != -1)
			{
				retVal = static_cast<MacroNode*>(absg->getNode(nodeId));
				break;
			}
		}
	}
	else
	{
		for( ; x >= this->getHOrigin(); x--)
		{
			int nodeId = hpamap->getNodeFromMap(x, y)->getLabelL(kParent);
			if(nodeId != -1)
			{
				retVal = static_cast<MacroNode*>(absg->getNode(nodeId));
				break;
			}
		}
	}

	return retVal;
}

MacroNode*
EmptyCluster::nextNodeInColumn(int x, int y, HPAClusterAbstraction* hpamap,
		bool topToBottom)
{
	MacroNode* retVal = 0;
	graph* absg = hpamap->getAbstractGraph(1);

	if(topToBottom)
	{
		for( ; y < (this->getVOrigin() + this->getHeight()); y++)
		{
			int nodeId = hpamap->getNodeFromMap(x, y)->getLabelL(kParent);
			if(nodeId != -1)
			{
				retVal = static_cast<MacroNode*>(absg->getNode(nodeId));
				break;
			}
		}
	}
	else
	{
		for( ; y >= this->getVOrigin(); y--)
		{
			int nodeId = hpamap->getNodeFromMap(x, y)->getLabelL(kParent);
			if(nodeId != -1)
			{
				retVal = static_cast<MacroNode*>(absg->getNode(nodeId));
				break;
			}
		}
	}

	return retVal;
}

void EmptyCluster::addParent(node *n, HPAClusterAbstraction* hpamap) 
	throw(std::invalid_argument)
{
	//assert(n->getLabelL(kParent) == -1);

	int absLevel =  n->getLabelL(kAbstractionLevel);
	if(absLevel > 0)
	{
		HPACluster::addParent(n, hpamap);
	}
	else
	{
		ClusterNode* absn = static_cast<ClusterNode*>(n->clone());
		absn->setLabelL(kAbstractionLevel, 1);
		graph* g = hpamap->getAbstractGraph(1);
		g->addNode(absn);
		addParent(absn, hpamap); // also creates intra-edges
		n->setLabelL(kParent, absn->getNum());
	}
}

edge* EmptyCluster::findSecondaryEdge(unsigned int fromId, unsigned int toId)
{
	edge* retVal = 0;
	for(unsigned int i=0; i<secondaryEdges.size(); i++)
	{
		edge* e = secondaryEdges.at(i);
		if((e->getFrom() == fromId && e->getTo() == toId) ||
			(e->getTo() == fromId && e->getFrom() == toId))
		{
			e = retVal;
			break;
		}
	}
	return retVal;
}

// calculate which side of the room's perimeter this node belongs to
EmptyCluster::RoomSide EmptyCluster::whichSide(node* n_)
{
	MacroNode* n = static_cast<MacroNode*>(n_);

	int nx = n->getLabelL(kFirstData);
	int ny = n->getLabelL(kFirstData+1);

	if(ny == this->getVOrigin() && nx >= this->getHOrigin())
		return TOP;
	if(ny == (this->getVOrigin()+this->getHeight()-1) && nx >= this->getHOrigin())
		return BOTTOM;
	if(nx == this->getHOrigin() && ny >= this->getVOrigin())
		return LEFT;
	if(nx == (this->getHOrigin()+this->getWidth()-1) && ny >= this->getVOrigin())
		return RIGHT;	
	
	return NONE;
}

// nb: nodes from the corners of each room belong to 2 sides simultaneously
void EmptyCluster::setBestExpandedNode(node *n)
{
	int nx = n->getLabelL(kFirstData);
	int ny = n->getLabelL(kFirstData+1);

	if(ny == this->getVOrigin() && nx >= this->getHOrigin())
	{
		bestTop = n;
	}
	if(ny == (this->getVOrigin()+this->getHeight()-1) && nx >= this->getHOrigin())
	{
		bestBottom = n;
	}
	if(nx == this->getHOrigin() && ny >= this->getVOrigin())
	{
		bestLeft = n;
	}
	if(nx == (this->getHOrigin()+this->getWidth()-1) && ny >= this->getVOrigin())
	{
		bestRight = n;
	}
}

node* EmptyCluster::getBestExpandedNode(RoomSide side)
{
	switch(side)
	{
		case TOP:
			return bestTop;
		case BOTTOM:
			return bestBottom;
		case LEFT:
			return bestLeft;
		case RIGHT: 
			return bestRight;
		default:
			return 0;
	}
}

#include "EmptyClusterAbstraction.h"

#include "ClusterNode.h"
#include "EmptyCluster.h"
#include "IEdgeFactory.h"
#include "IHPAClusterFactory.h"
#include "INodeFactory.h"
#include "graph.h"
#include "heap.h"
#include "map.h"
#include "MacroEdge.h"
#include "MacroNode.h"

EmptyClusterAbstraction::EmptyClusterAbstraction(Map* m, IHPAClusterFactory* cf, 
	INodeFactory* nf, IEdgeFactory* ef, bool allowDiagonals, bool perimeterReduction_,
	bool bfReduction_) 
	throw(std::invalid_argument)
	: HPAClusterAbstraction(m, cf, nf, ef, allowDiagonals),
	  perimeterReduction(perimeterReduction_), bfReduction(bfReduction_)
{
	sgEdge = 0;
}

EmptyClusterAbstraction::~EmptyClusterAbstraction()
{
	assert(sgEdge == 0);
}

int EmptyClusterAbstraction::getNumMacro()
{
	int macro = 0;
	cluster_iterator ci = getClusterIter();
	EmptyCluster* cluster = clusterIterNext(ci);
	while(cluster)
	{
		macro += cluster->macro;
		cluster = clusterIterNext(ci);
	}
	return macro;
}

// Decomposes the map into a set of empty (obstacle free) clusters.
// simple flood-fill based decomposition that extends a cluster until
// one of 3 terminating conditions is met:
//  	1. Clearance at some tile is greater than clearance at origin
//  	2. An obstacle is encountered
//  	3. The edge of the map is encountered.
//
//  Condition 1 is important to avoid extending clusters into areas that could be
//  assigned to larger clusters.
void EmptyClusterAbstraction::buildClusters()
{
	if(getVerbose())
		std::cout << "buildClusters...."<<std::endl;

	Map* m = this->getMap();
	int mapheight = m->getMapHeight();
	int mapwidth = m->getMapWidth();

	// calculate clearance values; store in a 2-D array
	int** clearance;
    clearance = new int*[mapwidth];
	for(int i = 0; i<mapwidth; i++)
		clearance[i] = new int[mapheight];
	for(int y=0; y<mapheight; y++)
		for(int x=0; x<mapwidth; x++)
			clearance[x][y] = 0;
	computeClearance(clearance);

	for(int y=0; y<mapheight; y++)
		for(int x=0; x<mapwidth; x++)
		{
			ClusterNode* cur = dynamic_cast<ClusterNode*>(this->getNodeFromMap(x, y));
			if(cur && cur->getParentClusterId() == -1)
			{
				EmptyCluster* cluster = new EmptyCluster(x, y, perimeterReduction, bfReduction);	
				cluster->setVerbose(getVerbose());
				cluster->setAllowDiagonals(getAllowDiagonals());
				addCluster(cluster);
				cluster->addNodesToCluster(this, clearance);
				if(this->getVerbose())
				{
					std::cout << "new cluster @ ("<<x<<","<<y<<") "
						" id: "<< cluster->getId()<< " height: "<<cluster->getHeight()<<" "
			   			"width: "<<cluster->getWidth()<<std::endl;
				}
			}	
		}

	if(getVerbose())
	{
		graph* g = this->getAbstractGraph(1);
		std::cout << "abstract graph; nodes: "<< g->getNumNodes()<<" edges: "<<g->getNumEdges()<<std::endl;
	}

	//std::cout << "clearance values for map: "<<m->getMapName()<<std::endl;
//	for(int y=0; y<mapheight; y++)
//	{
//		for(int x=0; x<mapwidth; x++)
//			std::cout << clearance[x][y] << " ";
//		std::cout << std::endl;
//	}

	for(int i=0; i<mapwidth; i++)
		delete clearance[i];
	delete[] clearance;
}

void EmptyClusterAbstraction::buildClusters2()
{
	if(getVerbose())
		std::cout << "buildClusters...."<<std::endl;

	Map* m = this->getMap();
	int mapheight = m->getMapHeight();
	int mapwidth = m->getMapWidth();

	// calculate clearance values; store in a 2-D array
	int** clearance;
    clearance = new int*[mapwidth];
	for(int i = 0; i<mapwidth; i++)
		clearance[i] = new int[mapheight];
	for(int y=0; y<mapheight; y++)
		for(int x=0; x<mapwidth; x++)
			clearance[x][y] = 0;
	computeClearance(clearance);

	// set initial priorities of all potential cluster origins
	// based on # of interior nodes in maximal clearance square of each tile
	heap clusterheap(30, false);
	EmptyCluster* cluster = new EmptyCluster(0, 0, perimeterReduction, bfReduction);
	for(int y=0; y<mapheight; y++)
		for(int x=0; x<mapwidth; x++)
		{
			node* n = this->getNodeFromMap(x, y);
			if(n)
			{
				cluster->setHOrigin(x);
				cluster->setVOrigin(y);
				cluster->extend(this, clearance);	

				double interiorNodes = 0;
				if(cluster->getWidth() > 2 && cluster->getHeight() > 2)
					interiorNodes = (cluster->getWidth()-2)*(cluster->getHeight()-2);

				//std::cout << "heap: ("<<x<<", "<<y<<") clearance: "<<clearance[x][y]<<" priority: "<<interiorNodes<<std::endl;
				n->setLabelF(kTemporaryLabel, interiorNodes);
				n->setKeyLabel(kTemporaryLabel);
				clusterheap.add(n);
			}
		}

	// start making clusters; prefer clusters with more interior nodes to others
	// with less
	while(!clusterheap.empty())
	{
		ClusterNode* cur = dynamic_cast<ClusterNode*>(clusterheap.peek());
		double minExpectedInteriorNodes = cur->getLabelF(kTemporaryLabel);
		int x = cur->getLabelL(kFirstData);
		int y = cur->getLabelL(kFirstData+1);

		if(cur->getParentClusterId() == -1)
		{
			if(!cluster)
				cluster = new EmptyCluster(x, y, perimeterReduction, bfReduction);
			else
			{
				cluster->setHOrigin(x);
				cluster->setVOrigin(y);
			}
			cluster->extend(this, clearance);	

			double interiorNodes = 0;
			if(cluster->getWidth() > 2 && cluster->getHeight() > 2)
				interiorNodes = (cluster->getWidth()-2)*(cluster->getHeight()-2);

			if(interiorNodes >= minExpectedInteriorNodes)
			{
				cluster->setVerbose(getVerbose());
				cluster->setAllowDiagonals(getAllowDiagonals());
				addCluster(cluster);
				cluster->addNodesToCluster(this, clearance);
				cluster->verifyCluster();
				if(this->getVerbose())
				{
					std::cout << "new cluster @ ("<<x<<","<<y<<") "
						" id: "<< cluster->getId()<< " height: "<<cluster->getHeight()<<" "
						"width: "<<cluster->getWidth()<<" priority: "<<cur->getLabelF(kTemporaryLabel)<<std::endl;
				}
				clusterheap.remove();
				cluster = 0;
			}
			else
			{
				cur->setLabelF(kTemporaryLabel, interiorNodes); 
				clusterheap.decreaseKey(cur);
			}
		}
		else
		{
			clusterheap.remove();
		}
		
	}

	for(int i=0; i<mapwidth; i++)
		delete clearance[i];
	delete[] clearance;
}

void EmptyClusterAbstraction::computeClearance(int** clearance)
{
	for(int x=getMap()->getMapWidth()-1;x>=0; x--)
	{
		for(int y=getMap()->getMapHeight()-1;y>=0; y--)
		{
			node* n = getNodeFromMap(x,y);
			if(n) 
			{	
				int x = n->getLabelL(kFirstData);
				int y = n->getLabelL(kFirstData+1);
				node *adj1, *adj2, *adj3; // adjacent neighbours
				adj1 = getNodeFromMap(x+1, y+1);
				adj2 = getNodeFromMap(x+1,y);
				adj3 = getNodeFromMap(x,y+1);
				
				if(adj1 && adj2 && adj3)
				{	
					int min = clearance[x+1][y+1]; 
					int tmp1 = clearance[x+1][y]; 
					int tmp2 = clearance[x][y+1]; 
					min = tmp1<min?tmp1:min;
					min = tmp2<min?tmp2:min;
					clearance[x][y] = min+1; 
				}
				else // tile is on a border or perimeter boundary. clearance = 1
				{
					clearance[x][y] = 1;
				}
			}
			else
			{
				clearance[x][y] = 0;
			}
		}
	}
}


// each start/goal node is connected to the nearest abstract node along each border
// of the cluster. i.e. it has 4 neighbours (one above, below, to the left and right).
void
EmptyClusterAbstraction::insertStartAndGoalNodesIntoAbstractGraph(node* s, node* g)
	throw(std::invalid_argument)
{
	HPAClusterAbstraction::insertStartAndGoalNodesIntoAbstractGraph(s, g);

	graph* absg = this->getAbstractGraph(1);
	MacroNode* absStart = static_cast<MacroNode*>(absg->getNode(s->getLabelL(kParent)));
	MacroNode* absGoal = static_cast<MacroNode*>(absg->getNode(g->getLabelL(kParent)));

	assert(absStart->getLabelL(kFirstData) == s->getLabelL(kFirstData));
	assert(absStart->getLabelL(kFirstData+1) == s->getLabelL(kFirstData+1));
	assert(absGoal->getLabelL(kFirstData) == g->getLabelL(kFirstData));
	assert(absGoal->getLabelL(kFirstData+1) == g->getLabelL(kFirstData+1));

	if(absStart == 0 || absGoal == 0)
	{
		throw std::invalid_argument("either start or goal node not inserted into abstract graph");
	}

	if(absStart->getParentClusterId() == absGoal->getParentClusterId())
	{
		if(!absg->findEdge(absStart->getNum(), absGoal->getNum()))
		{
			sgEdge = new MacroEdge(absStart->getNum(), absGoal->getNum(), this->h(absStart, absGoal));
			absg->addEdge(sgEdge);
		}	
	}
	else
	{
		// only connect nodes that do not already exist in the abstract graph 
		if(this->getStartId() != -1)
		{
			if(getAllowDiagonals())
				connectSG(absStart);
			else
				cardinalConnectSG(absStart);
		}
		if(this->getGoalId() != -1)
		{
			if(getAllowDiagonals())
				connectSG(absGoal);
			else
				cardinalConnectSG(absGoal);
		}
	}
	
}

void 
EmptyClusterAbstraction::removeStartAndGoalNodesFromAbstractGraph() 
	throw(std::runtime_error)
{
	graph* absg = this->getAbstractGraph(1);
	if(sgEdge)
	{
		absg->removeEdge(sgEdge);
		delete sgEdge;
		sgEdge = 0;
	}

	HPAClusterAbstraction::removeStartAndGoalNodesFromAbstractGraph();	
}

void EmptyClusterAbstraction::connectSG(MacroNode* absNode)
{
	EmptyCluster* nodeCluster = this->getCluster(absNode->getParentClusterId());
	graph* absg = this->getAbstractGraph(1);

	const int x = absNode->getLabelL(kFirstData);
	const int y = absNode->getLabelL(kFirstData+1);
	if(getVerbose())
		std::cout << "inserting node ("<<x<<", "<<y<<") into abstract graph"<<std::endl;

	if(getVerbose())
		std::cout << "connecting to nodes along the top of the cluster"<<std::endl;

	// connect to nodes along top perimeter of cluster
	int maxDiagonalSteps = y - nodeCluster->getVOrigin();
	int minx = (x-maxDiagonalSteps)<nodeCluster->getHOrigin()?nodeCluster->getHOrigin():(x-maxDiagonalSteps);
	int maxx = (x+maxDiagonalSteps)>(nodeCluster->getHOrigin()+nodeCluster->getWidth()-1)?
		(nodeCluster->getHOrigin()+nodeCluster->getWidth()-1):(x+maxDiagonalSteps);

	MacroNode* absNeighbour = 0;
	int ny = nodeCluster->getVOrigin();
	int nx = minx+1;
	for( ; nx <= maxx-1; nx++)
	{
		assert(this->getNodeFromMap(nx, ny));
		absNeighbour = static_cast<MacroNode*>(
				absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
		if(absNeighbour && &*absNeighbour != &*absNode)
			connectSGToNeighbour(absNode, absNeighbour);
	}

	// try to connect to nearest entrance along the top border not in the fan of absNode
	if(nodeCluster->getHeight() > 1 && nodeCluster->getWidth() > 1)
	{
		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInRow(minx-1, ny, this, false);
		else
			absNeighbour = nodeCluster->nextNodeInRow(minx, ny, this, false);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);

		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInRow(maxx+1, ny, this, true);
		else
			absNeighbour = nodeCluster->nextNodeInRow(maxx, ny, this, true);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);
	}

	if(getVerbose())
		std::cout << "connecting to nodes along the bottom of the cluster"<<std::endl;
	// connect to nodes along the bottom perimeter of cluster
	maxDiagonalSteps = (nodeCluster->getVOrigin()+nodeCluster->getHeight()-1) - y;
	minx = (x-maxDiagonalSteps)<nodeCluster->getHOrigin()?nodeCluster->getHOrigin():(x-maxDiagonalSteps);
	maxx = (x+maxDiagonalSteps)>(nodeCluster->getHOrigin()+nodeCluster->getWidth()-1)?
		(nodeCluster->getHOrigin()+nodeCluster->getWidth()-1):(x+maxDiagonalSteps);

	nx = minx+1;
	ny = nodeCluster->getVOrigin()+nodeCluster->getHeight()-1;
	for( ; nx <= maxx-1; nx++)
	{
		assert(this->getNodeFromMap(nx, ny));
		absNeighbour = static_cast<MacroNode*>(
				absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
		if(absNeighbour && &*absNeighbour != &*absNode)
			connectSGToNeighbour(absNode, absNeighbour);
	}

	// try to connect to nearest entrance along the bottom border not in the fan of absNode
	if(nodeCluster->getHeight() > 1 && nodeCluster->getWidth() > 1)
	{
		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInRow(minx-1, ny, this, false);
		else
			absNeighbour = nodeCluster->nextNodeInRow(minx, ny, this, false);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);

		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInRow(maxx+1, ny, this, true);
		else
			absNeighbour = nodeCluster->nextNodeInRow(maxx, ny, this, true);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);

	}
	
	if(getVerbose())
		std::cout << "connecting to nodes along the left of the cluster"<<std::endl;
	// connect to nodes along the left perimeter of cluster
	maxDiagonalSteps = x - nodeCluster->getHOrigin();	
	int miny = (y-maxDiagonalSteps)<nodeCluster->getVOrigin()?nodeCluster->getVOrigin():(y-maxDiagonalSteps);
	int maxy = (y+maxDiagonalSteps)>(nodeCluster->getVOrigin()+nodeCluster->getHeight()-1)?
		(nodeCluster->getVOrigin()+nodeCluster->getHeight()-1):(y+maxDiagonalSteps);

	nx = nodeCluster->getHOrigin();
	ny = miny+1;
	for( ; ny <= maxy-1; ny++)
	{
		assert(this->getNodeFromMap(nx, ny));
		absNeighbour = static_cast<MacroNode*>(
				absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
		if(absNeighbour && &*absNeighbour != &*absNode)
			connectSGToNeighbour(absNode, absNeighbour);
	}

	// connect to nearest nodes outside fan area (if necessary)
	if(nodeCluster->getHeight() > 1 && nodeCluster->getWidth() > 1)
	{
		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInColumn(nx, miny-1, this, false);
		else
			absNeighbour = nodeCluster->nextNodeInColumn(nx, miny, this, false);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);

		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInColumn(nx, maxy+1, this, true);
		else
			absNeighbour = nodeCluster->nextNodeInColumn(nx, maxy, this, true);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);
	}

	if(getVerbose())
		std::cout << "connecting to nodes along the right of the cluster"<<std::endl;
	// connect to nodes along the right perimeter of cluster
	maxDiagonalSteps = (nodeCluster->getHOrigin()+nodeCluster->getWidth()-1) - x;
	miny = (y-maxDiagonalSteps)<nodeCluster->getVOrigin()?nodeCluster->getVOrigin():(y-maxDiagonalSteps);
	maxy = (y+maxDiagonalSteps)>(nodeCluster->getVOrigin()+nodeCluster->getHeight()-1)?
		(nodeCluster->getVOrigin()+nodeCluster->getHeight()-1):(y+maxDiagonalSteps);

	nx = nodeCluster->getHOrigin()+nodeCluster->getWidth()-1;
	ny = miny+1;
	for( ; ny <= maxy-1; ny++)
	{
		assert(this->getNodeFromMap(nx, ny));
		absNeighbour = static_cast<MacroNode*>(
				absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
		if(absNeighbour && &*absNeighbour != &*absNode)
			connectSGToNeighbour(absNode, absNeighbour);
	}

	// connect to nearest nodes outside fan area (if necessary)
	if(nodeCluster->getHeight() > 1 && nodeCluster->getWidth() > 1)
	{
		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInColumn(nx, miny-1, this, false);
		else
			absNeighbour = nodeCluster->nextNodeInColumn(nx, miny, this, false);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);

		if(maxDiagonalSteps == 0)
			absNeighbour = nodeCluster->nextNodeInColumn(nx, maxy+1, this, true);
		else
			absNeighbour = nodeCluster->nextNodeInColumn(nx, maxy, this, true);

		if(absNeighbour)
		   	connectSGToNeighbour(absNode, absNeighbour);
	}
}

void EmptyClusterAbstraction::cardinalConnectSG(MacroNode* absNode)
{
	graph* absg = this->getAbstractGraph(1);
	EmptyCluster* nodeCluster = this->getCluster(absNode->getParentClusterId());

	int x = absNode->getLabelL(kFirstData);
	int y = absNode->getLabelL(kFirstData+1);
	if(getVerbose())
		std::cout << "inserting node ("<<x<<", "<<y<<") into abstract graph"<<std::endl;

	// connect to neighbour above
	int ny = nodeCluster->getVOrigin();
	int nx = x;
	MacroNode* absNeighbour = static_cast<MacroNode*>(
			absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
	if(absNeighbour == 0 || absNeighbour->getNum() == absNode->getNum())
	{
		absNeighbour = nodeCluster->nextNodeInRow(nx+1, ny, this, true);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);

		absNeighbour =  nodeCluster->nextNodeInRow(nx-1, ny, this, false);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);
	}
	else
		connectSGToNeighbour(absNode, absNeighbour);

	// connect to neighbour below
	ny = nodeCluster->getVOrigin()+nodeCluster->getHeight()-1;
	nx = x;
	absNeighbour = static_cast<MacroNode*>(
			absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
	if(absNeighbour == 0 || absNeighbour->getNum() == absNode->getNum())
	{
		absNeighbour = nodeCluster->nextNodeInRow(nx+1, ny, this, true);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);

		absNeighbour =  nodeCluster->nextNodeInRow(nx-1, ny, this, false);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);
	}
	else
		connectSGToNeighbour(absNode, absNeighbour);
		

	// connect to neighbour to the left
	ny = y; 
	nx = nodeCluster->getHOrigin();
	absNeighbour = static_cast<MacroNode*>(
			absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
	if(absNeighbour == 0 || absNeighbour->getNum() == absNode->getNum())
	{
		absNeighbour = nodeCluster->nextNodeInColumn(nx, ny+1, this, true);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);

		absNeighbour =  nodeCluster->nextNodeInColumn(nx, ny-1, this, false);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);
	}
	else
		connectSGToNeighbour(absNode, absNeighbour);


	// connect to neighbour to the right
	ny = y; 
	nx = nodeCluster->getHOrigin()+nodeCluster->getWidth()-1;
	absNeighbour = static_cast<MacroNode*>(
			absg->getNode(this->getNodeFromMap(nx, ny)->getLabelL(kParent)));
	if(absNeighbour == 0 || absNeighbour->getNum() == absNode->getNum())
	{
		absNeighbour = nodeCluster->nextNodeInColumn(nx, ny+1, this, true);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);

		absNeighbour =  nodeCluster->nextNodeInColumn(nx, ny-1, this, false);
		if(absNeighbour)
			connectSGToNeighbour(absNode, absNeighbour);
	}
	else
		connectSGToNeighbour(absNode, absNeighbour);
}

void EmptyClusterAbstraction::connectSGToNeighbour(MacroNode* absNode, MacroNode* absNeighbour)
{
	assert(this->getNodeFromMap(absNode->getLabelL(kFirstData),
					absNode->getLabelL(kFirstData+1))->getLabelL(kParent) != -1);
	assert(this->getNodeFromMap(absNeighbour->getLabelL(kFirstData), 
					absNeighbour->getLabelL(kFirstData+1))->getLabelL(kParent) != -1);

	if(absNode->getUniqueID() == absNeighbour->getUniqueID())
		return;

	graph* absg = this->getAbstractGraph(1);

	assert((int)absNeighbour->getNum() < absg->getNumNodes());
	assert((int)absNode->getNum() < absg->getNumNodes());

	MacroEdge* e = new MacroEdge(absNode->getNum(), absNeighbour->getNum(), h(absNode, absNeighbour));
	absg->addEdge(e);
	if(getVerbose())
	{
		std::cout << "absNeighbour ("<<absNeighbour->getLabelL(kFirstData)<<", "
			<<absNeighbour->getLabelL(kFirstData+1)<<") weight: "<<e->getWeight() <<std::endl;
	}
}

// h heuristic
//double EmptyClusterAbstraction::h(node* from, node* to)
//{
//	int fx = from->getLabelL(kFirstData);
//	int fy = from->getLabelL(kFirstData+1);
//	int tx = to->getLabelL(kFirstData);
//	int ty = to->getLabelL(kFirstData+1);
////	std::cout << "from: "<<fx<<","<<fy<<") to: ("<<tx<<","<<ty<<") ";
//
//	int deltax = fx - tx;
//	if(deltax < 0) deltax *=-1;
//
//	int deltay = fy - ty;
//	if(deltay < 0) deltay *=-1;
//
////	std::cout << "deltax: "<<deltax<<" deltay: "<<deltay<<std::endl;
//	return deltax + deltay;
//}

EmptyCluster* EmptyClusterAbstraction::clusterIterNext(cluster_iterator& it) const
{
       return static_cast<EmptyCluster*>(HPAClusterAbstraction::clusterIterNext(it));
}

EmptyCluster* EmptyClusterAbstraction::getCluster(int cid)
{
       return static_cast<EmptyCluster*>(HPAClusterAbstraction::getCluster(cid));
}

double EmptyClusterAbstraction::getAverageClusterSize()
{
	double total;
	cluster_iterator iter = this->getClusterIter();
	EmptyCluster* cluster = 0;
	while((cluster = this->clusterIterNext(iter)))
	{
		total += cluster->getHeight()*cluster->getWidth();
	}

	return total/this->getNumClusters();
}

double EmptyClusterAbstraction::getAverageNodesPruned()
{
	double total;
	cluster_iterator iter = this->getClusterIter();
	EmptyCluster* cluster = 0;
	while((cluster = this->clusterIterNext(iter)))
	{
		int all = cluster->getHeight()*cluster->getWidth();
		int nparents = cluster->getNumParents();
		total += all - nparents;
	}

	return total/this->getNumClusters();
	
}

int EmptyClusterAbstraction::getNumAbsEdges()
{
	graph* g = this->getAbstractGraph(1);
	int numEdges = g->getNumEdges();

	cluster_iterator iter = this->getClusterIter();
	EmptyCluster* cluster = 0;
	while((cluster = this->clusterIterNext(iter)))
	{
		numEdges += cluster->getNumSecondaryEdges();
	}
	
	return numEdges;
}

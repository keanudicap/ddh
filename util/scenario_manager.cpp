#include "scenario_manager.h"
#include "flexible_astar.h"

#include <cstdlib>
#include <string>

static int head_offset = 0;
static int tail_offset = 0;
static const int MAXTRIES=10000000;

warthog::scenario_manager::scenario_manager() : version_(1)
{
}

warthog::scenario_manager::~scenario_manager()
{
	for(unsigned int i=0; i < experiments_.size(); i++)
	{
		delete experiments_[i];
	}
	experiments_.clear();
}

void 	
warthog::scenario_manager::generate_experiments(
		warthog::gridmap* map, int num) 
{
	assert(map); // need a test here; throw exception if absMap is null
	
	head_offset = tail_offset = 0;
	int tries=0;
	int generated=0;
	while(generated < num)
	{	
		if(tries >= MAXTRIES)
		{
			std::cerr << "err; scenario_manager::generate_experiments"
				<< " tried "<<tries << " times but could not generate "
				"a valid experiment. giving up.\n";
			exit(1);
		}
		
		experiment* exp = generate_single_experiment(map); 
		if(exp != NULL) 
		{
			this->add_experiment(exp);
			generated++;
			if((generated % 10) == 0)
			{
				head_offset += 10;
				tail_offset += 20;
				std::cout << "\rgenerated: "<< generated << "/" << num;
				std::cout << std::flush;
			}
		}
		tries++;
		if((tries % 5) == 0)
		{
			head_offset += 5;
			tail_offset += 10;
		}
	}
	std::cout << " experiments_." << std::endl;
	sort();
}

warthog::experiment* 
warthog::scenario_manager::generate_single_experiment(warthog::gridmap* map)
{
	return 0;
	/*
	graph *g = absMap->getGraph(0);
	const char* _map = absMap->getMap()->getMapName();

	node *r1, *r2;
	experiment* newexp;

	r1 = r2 = 0;
	path *p=0;

	if(head_offset + 100 >= g->getNumNodes())
		head_offset = 0;
	if(tail_offset + 100 >= g->getNumNodes())
		tail_offset = 0;

	int id1 = (rand() % 100) + head_offset;
	int id2 = g->getNumNodes() - ((rand() % 100) + tail_offset);
	//std::cout << "id1: "<<id1 << " id2: "<< id2;

	r1 = g->getNode(id1);
	r2 = g->getNode(id2);

	aStarOld searchalg;
	p = searchalg.getPath(absMap, r1, r2);

	if(!p)
	{
	//	std::cout << " no path;" <<std::endl;
		return NULL;
	}
	//std::cout << " found path;" << std::endl;
		
	double dist = r2->getLabelF(kTemporaryLabel); // fValue
	int x1, x2, y1, y2;
	int mapwidth = absMap->getMap()->getMapWidth();
	int mapheight = absMap->getMap()->getMapHeight();
	
	x1 = r1->getLabelL(kFirstData); y1 = r1->getLabelL(kFirstData+1);
	x2 = r2->getLabelL(kFirstData); y2 = r2->getLabelL(kFirstData+1);
	newexp = new experiment(x1, y1, x2, y2, mapwidth, mapheight, 0, dist, _map);
	
	delete p;
	return newexp;
	*/
}

void 
warthog::scenario_manager::load_scenario(const char* filelocation)
{
	std::ifstream infile;
	infile.open(filelocation,std::ios::in);

	if(!infile.good())
	{
		std::cerr << "err; scenario_manager::load_scenario "
		<< "Invalid scenario file: "<<filelocation << std::endl;
		infile.close();
		exit(1);
	}

	sfile_ = filelocation;


	// Check if a version number is given
	float version=0;
	std::string first;
	infile >> first;
	if(first != "version")
	{
		version = 0.0;
		infile.seekg(0,std::ios::beg);
	}

	infile >> version;
	if(version == 1.0 || version == 0)
	{
		load_v1_scenario(infile);
	}

	else if(version == 3)
	{
		load_v3_scenario(infile);
	}
	else
	{
		std::cerr << "err; scenario_manager::load_scenario "
			<< " scenario file contains invalid version number. \n";
		infile.close();
		exit(1);
	}

	infile.close();
}

// V1.0 is the version officially supported by HOG
void 
warthog::scenario_manager::load_v1_scenario(std::ifstream& infile)
{
	int sizeX = 0, sizeY = 0; 
	int bucket;
	std::string map;  
	int xs, ys, xg, yg;
	std::string dist;

	while(infile>>bucket>>map>>sizeX>>sizeY>>xs>>ys>>xg>>yg>>dist)
	{
		double dbl_dist = strtod(dist.c_str(),0);
		experiments_.push_back(
				new experiment(xs,ys,xg,yg,sizeX,sizeY,dbl_dist,map));

		int precision = 0;
		if(dist.find(".") != std::string::npos)
		{
			precision = dist.size() - (dist.find(".")+1);
		}
		experiments_.back()->set_precision(precision);
	}
}

void 
warthog::scenario_manager::load_v3_scenario(std::ifstream& infile)
{
	int xs, ys, xg, yg;
	std::string dist;
	std::string mapfile;
	while(infile>>mapfile>>xs>>ys>>xg>>yg>>dist)
	{
		double dbl_dist = strtod(dist.c_str(),0);
		experiments_.push_back(
			new experiment(xs, ys, xg, yg, 1, 1, dbl_dist, mapfile));

		int precision = 0;
		if(dist.find(".") != std::string::npos)
		{
			precision = dist.size() - (dist.find(".")+1);
		}
		experiments_.back()->set_precision(precision);
	}
}

void 
warthog::scenario_manager::write_scenario(const char* filelocation)
{
	if(experiments_.size() == 0) // nothing to write
		return;

	std::ofstream scenariofile;
	scenariofile.precision(16);
	scenariofile.open(filelocation, std::ios::out);
	scenariofile << "version " << version_<<std::endl;

	for(unsigned int i=0; i<experiments_.size(); i++)
	{	
		experiment*	cur = experiments_.at(i);
		cur->print(scenariofile);
		scenariofile << std::endl;
	}
	scenariofile.close();		
}

void 
warthog::scenario_manager::sort()
{
	for(unsigned int i=0; i < experiments_.size(); i++)
	{
		for(unsigned int j = i; j < experiments_.size(); j++)
		{
			if(experiments_.at(j)->distance() <
				   	experiments_.at(i)->distance())
			{
				experiment* tmp = experiments_.at(i);
				experiments_.at(i) = experiments_.at(j);
				experiments_.at(j) = tmp;
			}
		}
	}
}

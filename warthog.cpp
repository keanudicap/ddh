#ifndef WARTHOG_H
#define WARTHOG_H

#include "blockmap.h"
#include "cuckoo_table.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "hash_table.h"
#include "heap.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"

#include "getopt.h"

#include <iomanip>
#include <sstream>
#include <tr1/unordered_map>
#include <memory>

void blockmap_access_test();
void gridmap_access_test();
void heap_insert_test();
void cuckoo_table_test();
void unordered_map_test();
void hash_table_test();
void gridmap_expansion_policy_test();
void flexible_astar_test();
int main(int argc, char** argv)
{
	flexible_astar_test();
}

void flexible_astar_test()
{
	bool check_opt = false;
	warthog::scenario_manager scenmgr;
	scenmgr.load_scenario("orz700d.map.scen");

	std::shared_ptr<warthog::gridmap>
	   	map(new warthog::gridmap(scenmgr.get_experiment(0)->map().c_str(), true));

	std::shared_ptr<warthog::gridmap_expansion_policy>
	   	expander(new warthog::gridmap_expansion_policy(map));

	std::shared_ptr<warthog::octile_heuristic>
		heuristic(new warthog::octile_heuristic(map->width(), map->height()));

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::gridmap_expansion_policy> astar(heuristic, expander);
	//astar.set_verbose(true);

	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(startid, goalid);
		if(len == DBL_MAX)
		{
			len = 0;
		}

		if(!check_opt)
			continue;

		std::cerr << "exp "<<i<<" ";
		exp->print(std::cerr);
		std::cerr << std::endl;

		std::stringstream stroptlen;
		stroptlen << std::fixed << std::setprecision(exp->precision());
		stroptlen << exp->distance();

		std::stringstream strpathlen;
		strpathlen << std::fixed << std::setprecision(exp->precision());
		strpathlen << len;

		if(stroptlen.str().compare(strpathlen.str()))
		{
			std::cerr << std::setprecision(6);
			std::cerr << "optimality check failed!" << std::endl;
			std::cerr << std::endl;
			std::cerr << "optimal path length: "<<stroptlen.str()
				<<" computed length: ";
			std::cerr << strpathlen.str()<<std::endl;
			std::cerr << "precision: " << exp->precision()<<std::endl;
			exit(1);
		}
	}


	

}

void gridmap_expansion_policy_test()
{
	std::shared_ptr<warthog::gridmap> map(
			new warthog::gridmap("CSC2F.map", true));

	warthog::gridmap_expansion_policy policy(map);
	unsigned int nodeid[2] = {89, 0};

	for(int i=0; i < 2; i++)
	{
		std::cout << "nid: "<<nodeid[i]<<std::endl;
		unsigned int mapwidth = map->width();
		policy.expand(nodeid[i], 0);
		for(unsigned int nid = policy.first();
				nid != warthog::UNDEF;
				nid = policy.next())
		{
			unsigned int x = UINT_MAX;
			unsigned int y = UINT_MAX;
			x = nid % mapwidth;
			y = nid / mapwidth;
			std::cout << "neighbour: " << nid << " (" << x << ", "<<y
				<< ") cost: " << policy.cost_to_n() << std::endl;
		}
	}
}

void hash_table_test()
{
	warthog::hash_table mytable;
	for(int i=0; i < 10000000; i++)
	{
		mytable.insert(i);
	}
	for(int i=0; i < 10000000; i++)
	{
		if(!mytable.contains(i))
		{
			std::cerr << "table does not contain "<<i<<std::endl;
			exit(1);
		}
	}
	//mytable.print();
}

void unordered_map_test()
{
	std::tr1::unordered_map<unsigned int, unsigned int> mymap;
	mymap.rehash(1024);
	for(int i=0; i < 10000000; i++)
	{
		mymap[i] = i;
	}

	for(int i=0; i < 10000000; i++)
	{
		mymap.find(i);
	}
}

void cuckoo_table_test()
{
	std::cout << "cuckoo_table_test\n";
	warthog::cuckoo_table table(1024);
	//table.set_verbose(true);
	int errors = 0;
	for(int i=0; i < 10000000; i++)
	{
		table.insert(i);
		if(!table.contains(i))
		{
			errors++;
		}
	}
	table.metrics(std::cout);
	std::cout << "errors: "<<errors<<std::endl;
	std::cout << "/cuckoo_table_test\n";
}

void blockmap_access_test()
{
	std::cout << "blockmap_access_test..."<<std::endl;
	const char* file = "orz700d.map";
	std::cout << "loading "<<file<<std::endl;
	warthog::blockmap mymap(file, false);

	for(int i=0; i < 1<<28; i++)
	{
		int x = (rand()/RAND_MAX)*mymap.width();
		int y = (rand()/RAND_MAX)*mymap.height();
		for(int nx = x-1; nx < x+2; nx++)
		{
			for(int ny = y-1; ny < y+2; ny++)
			{
				mymap.get_label(nx, ny);
			}
		}
	}
	std::cout << "/blockmap_access_test..."<<std::endl;
}

void heap_insert_test()
{
	std::cout << "heap_insert_test...\n";
	unsigned int heapnodes = 1000000;
	warthog::heap myheap(heapnodes, true);
	warthog::search_node** nodes = new warthog::search_node*[heapnodes];
	for(int i=heapnodes; i > 0 ; i--)
	{
		nodes[i] = new warthog::search_node(i);
		nodes[i]->update(0, i, 0);
		myheap.push(nodes[i]);
	}
	// test duplicate detection
	for(int i=heapnodes; i > 0 ; i--)
	{
		myheap.push(nodes[i]);
	}
	assert(myheap.size() == heapnodes);

	// test pop
	for(unsigned int i=0; i < heapnodes; i++)
	{
		assert(myheap.size() == heapnodes-i);
		delete myheap.pop();
	}
	delete [] nodes;
	assert(warthog::search_node::get_refcount() == 0);
	std::cout << "/heap_insert_test...\n";
}

void gridmap_access_test()
{
	std::cout << "gridmap_access_test..."<<std::endl;
	const char* file = "orz700d.map";
	std::cout << "loading map..."<<file<<std::endl;
	warthog::gridmap mymap(file, false);
//	std::cout << "map\n";
	//mymap.print(std::cout);
//	std::cout << "done."<<std::endl;

	for(int i=0; i < 1<<28; i++)
	{
		int x = (rand()/RAND_MAX)*mymap.width();
		int y = (rand()/RAND_MAX)*mymap.height();
		for(int nx = x-1; nx < x+2; nx++)
		{
			for(int ny = y-1; ny < y+2; ny++)
			{
				mymap.get_label(nx, ny);
			}
		}
		//std::cout << i << "\r" << std::flush;
	}
	std::cout << "gridmap_access_test..."<<std::endl;
}

#endif


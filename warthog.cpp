// warthog.cpp
//
// @author: dharabor
// @created: August 2012
//

#include "cfg.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps_expansion_policy.h"
#include "jps_expansion_policy2.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"

#include "getopt.h"

#include <iomanip>
#include <sstream>
#include <tr1/unordered_map>
#include <memory>

// check computed solutions are optimal
bool check_opt = false;
// print debugging info during search
bool verbose = false;

void
help()
{
	std::cerr << "Valid options:\n"
	<< "--alg [jps | astar]\n"
	<< "--scen [filename]\n"
	<< "--map [filename] (optional)\n"
	<< "--checkopt (optional)\n"
	<< "--verbose (optional)\n";
}

void
check_optimality(double len, warthog::experiment* exp)
{
	if(!check_opt)
	{
		return;
	}

	std::stringstream strpathlen;
	strpathlen << std::fixed << std::setprecision(exp->precision());
	strpathlen << len;

	std::stringstream stroptlen;
	stroptlen << std::fixed << std::setprecision(exp->precision());
	stroptlen << exp->distance();

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

void
run_jps_plus(warthog::scenario_manager& scenmgr, warthog::gridmap& map)
{
	warthog::jps_expansion_policy2 expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy2> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tlen\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps+" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps(warthog::scenario_manager& scenmgr, warthog::gridmap& map)
{
	warthog::jps_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tlen\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_astar(warthog::scenario_manager& scenmgr, warthog::gridmap& map)
{
	warthog::gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::gridmap_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);


	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tlen\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid), 
				map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "astar" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"scen",  required_argument, 0, 0},
		{"alg",  required_argument, 0, 1},
		{"map",  optional_argument, 0, 2},
		{"checkopt",  no_argument, 0, 3},
		{"verbose",  no_argument, 0, 4},
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, valid_args);

	// sanity checking 
	if(cfg.get_param_value("checkopt") != "")
	{
		check_opt = true;
	}

	if(cfg.get_param_value("verbose") != "")
	{
		verbose = true;
	}

	std::string sfile = cfg.get_param_value("scen");
	std::string mfile = cfg.get_param_value("map");
	std::string alg = cfg.get_param_value("alg");

	// load scenario file
	if(sfile == "")
	{
		help();
		exit(0);
	}
	warthog::scenario_manager scenmgr;
	scenmgr.load_scenario(sfile.c_str());
	
	// load the map
	if(mfile == "")
	{
		mfile  = scenmgr.get_experiment(0)->map();
	}
	warthog::gridmap map(mfile.c_str());

	// run the target algorithm
	if(alg == "")
	{
		help();
		exit(0);
	}

	if(alg == "jps")
	{
		run_jps(scenmgr, map);
	}

	if(alg == "jps+")
	{
		run_jps_plus(scenmgr, map);
	}

	if(alg == "astar")
	{
		run_astar(scenmgr, map);
	}
}


#include "cfg.h"

warthog::util::cfg::cfg()
{
}

warthog::util::cfg::~cfg()
{
}

void 
warthog::util::cfg::parse_args(int argc, char** argv, warthog::util::param params[])
{
	int current_opt;
	int c = getopt_long(argc, argv, "", params, &current_opt );
	while(c != -1)
	{
		if(params[current_opt].has_arg == 0)
		{
			params_[params[current_opt].name] = "1";
		}
		else
		{
			params_[params[current_opt].name] = optarg;
		}

		std::cerr << params[current_opt].name << " = " 
			<< params_[params[current_opt].name] << std::endl;
		c = getopt_long(argc, argv, "", params, &current_opt );	
	}

	for(int index = optind; index < argc; index++)
	{
		std::cerr << "Unrecognised parameter: " << argv[index] << std::endl;
	}
}

std::string
warthog::util::cfg::get_param_value(std::string param_name)
{
	std::string ret("");
	std::map<std::string, std::string>::iterator it = 
		params_.find(param_name);
	if(it != params_.end())
	{
		ret =  (*it).second;
	}
	return ret;
}

void
warthog::util::cfg::print(std::ostream& out)
{
	out << "cfg\n";
	for(std::map<std::string, std::string>::iterator it = params_.begin();
			it != params_.end(); it++)
	{
		out << (*it).first <<"="<<(*it).second<<std::endl;
	}
}

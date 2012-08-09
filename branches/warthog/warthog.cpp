#ifndef WARTHOG_H
#define WARTHOG_H

#include "gridmap.h"

#include "getopt.h"

int main(int argc, int** argv)
{
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
}

#endif


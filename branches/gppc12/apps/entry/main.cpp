#include <stdio.h>
#include <stdint.h>
#include "ScenarioManager.h"
#include "Timer.h"
#include "Entry.h"

void LoadMap(const char *fname, std::vector<bool> &map, int &h, int &w);

int main(int argc, char **argv)
{
	std::vector<xyLoc> thePath;

	std::vector<bool> mapData;
	int width, height;

	LoadMap(argv[1], mapData, height, width);

	PreprocessMap(mapData, width, height, "unnamed");
	void *reference = PrepareForSearch(mapData, width, height, "unnamed");

	ScenarioManager scen;
	scen.loadScenarioFile(argv[2]);

	double elapsedTime = 0;
	Timer t;
	for (int x = 0; x < scen.getNumExperiments(); x++)
    {
		bool done;
		do {
			xyLoc s, g;
			s.x = scen.getNthExperiment(x)->getStartX();
			s.y = scen.getNthExperiment(x)->getStartY();
			g.x = scen.getNthExperiment(x)->getGoalX();
			g.y = scen.getNthExperiment(x)->getGoalY();
			t.startTimer();
			done = GetPath(reference, s, g, thePath);
			elapsedTime += t.endTimer();
		} while (done == false);
    }
	printf("%1.5f seconds elapsed\n", elapsedTime);
}

void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f)
    {
		fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
		map.resize(height*width);
		for (int y = 0; y < height; y++)
		{
			char c;
			for (int x = 0; x < width; x++)
			{
				fscanf(f, "%c", &c);
			//	std::cout << c;
				map[y*width+x] = (c == '.' || c == 'G' || c == 'S');
			}
		    fscanf(f, "%c", &c);
			//std::cout <<"!"<< std::endl;
		}
		fclose(f);
    }
}

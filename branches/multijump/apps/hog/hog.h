/*
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

class Heuristic;
class ExpansionPolicy;
class mapAbstraction;
class RefinementPolicy;

namespace HOG
{
	typedef enum
	{ 
		HPA, ERR, FLAT, FLATJUMP, JPA, MULTIJUMP
	} 
	AbstractionType;
}

void myDisplayHandler(unitSimulation *, tKeyboardModifier, char key);
void myNewUnitKeyHandler(unitSimulation *, tKeyboardModifier, char key);
int myCLHandler(char *argument[], int maxNumArgs);
int myScenarioGeneratorCLHandler(char *argument[], int maxNumArgs);
int myAllPurposeCLHandler(char* argument[], int maxNumArgs);
int myExecuteScenarioCLHandler(char *argument[], int maxNumArgs);
bool myClickHandler(unitSimulation *, int x, int y, point3d loc, tButtonType, tMouseEventType);
void runNextExperiment(unitSimulation *unitSim);
void processStats(statCollection* stat, const char* unitname);
void gogoGadgetNOGUIScenario(mapAbstraction* ecmap);
ExpansionPolicy* newExpansionPolicy(mapAbstraction* map);
Heuristic* newHeuristic();
searchAlgorithm* newSearchAlgorithm(mapAbstraction* aMap, bool refine=true);
RefinementPolicy* newRefinementPolicy(ExpansionPolicy*, mapAbstraction*, bool);

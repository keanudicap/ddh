#include "ExpansionPolicy.h"
#include "mapAbstraction.h"
#include "graph.h"

ExpansionPolicy::ExpansionPolicy()
{
	this->target = 0;
}

ExpansionPolicy::~ExpansionPolicy() 
{
}

void ExpansionPolicy::expand(node* t)
{
	this->target = t;
}

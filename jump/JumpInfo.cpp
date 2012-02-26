#include "JumpInfo.h"


JumpInfo::JumpInfo()
{ 
}

JumpInfo::~JumpInfo()  
{ 
}

JumpInfo::JumpInfo(JumpInfo& other)
{
	for(unsigned int i=0; i < other.nodecount(); i++)
	{
		this->jumpdirs.push_back(other.jumpdirs.at(i));
		this->jumpnodes.push_back(other.jumpnodes.at(i));
		this->jumpcosts.push_back(other.jumpcosts.at(i));
	}
}


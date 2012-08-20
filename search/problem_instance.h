#ifndef WARTHOG_PROBLEM_INSTANCE_H
#define WARTHOG_PROBLEM_INSTANCE_H

namespace warthog
{

class problem_instance
{
	public:
		problem_instance() :  goal_id_(0), start_id_(0)  { }
		~problem_instance() { } 

		inline void
		set_goal(unsigned int goal_id)
		{
			goal_id_ = goal_id;
		}

		inline void
		set_start(unsigned int start_id)
		{
			start_id_ = start_id;
		}

		inline unsigned int 
		get_goal() { return goal_id_; }

		inline unsigned int
		get_start() { return start_id_; }

	private:
		unsigned int goal_id_;
		unsigned int start_id_;

		problem_instance(const warthog::problem_instance& other) { }

		warthog::problem_instance& 
		operator=(const warthog::problem_instance& other) { return *this; } 
};

}

#endif


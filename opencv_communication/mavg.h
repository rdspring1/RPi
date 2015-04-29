#ifndef MAVG_H
#define MAVG_H

#include <list>

class Mavg
{
	public:
		Mavg(unsigned size) : match_list(size, false) {}

		void update(bool value)
		{
			if(match_list.front())
			{
				--match_count;
			}	
			match_list.pop_front();

			if(value)
			{
				++match_count;
			}
			match_list.push_back(value);
		}

		unsigned count() const
		{
			return match_count;
		}

		double avg() const
		{
			return (double) match_count / (double) match_list.size();
		}

	private:
		std::list<bool> match_list;
		unsigned match_count = 0;
};
#endif /* MAVG_H */

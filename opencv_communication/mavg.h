#ifndef MAVG_H
#define MAVG_H

#include <list>

class Mavg
{
	public:
		Mavg(unsigned size = DEFAULT_THRESHOLD) : match_list(size, false) {}

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
			return ((double) match_count + 1.0) / ((double) match_list.size() + 2);
		}

	private:
		std::list<bool> match_list;
		unsigned match_count = 0;
		const static unsigned DEFAULT_THRESHOLD = 3;
};
#endif /* MAVG_H */

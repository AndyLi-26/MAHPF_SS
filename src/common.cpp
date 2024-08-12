#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path)
{
    os<<"[";
    for (const PathEntry& entry : path) {
        os<<entry.location<<", ";
    }
    os<<"]"<<endl;
	return os;
}


bool isSamePath(const Path& p1, const Path& p2)
{
	if (p1.size() != p2.size())
		return false;
	for (unsigned i = 0; i < p1.size(); i++)
	{
		if (p1[i].location != p2[i].location)
			return false;
	}
	return true;
}

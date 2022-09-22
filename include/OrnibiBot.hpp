#ifndef  ORNIBIBOT_HPP

#include <iostream>
#include <vector>
#include <ros/ros.h>


class OrnibiBot{
    private:
    
    public:
    OrnibiBot();
    ~OrnibiBot();
    std::vector<int> get_wing_position();

};

#endif
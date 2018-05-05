#include <map>
#include <iostream>
#include <string>



int main() {
        std::map<int,std::string> map_list;
        /*  map_list.insert(std::pair<int,std::string>(2,"kit2"));
           map_list.insert(std::pair<int,std::string>(3,"kit3"));
           map_list.insert(std::pair<int,std::string>(1,"kit1"));
           std::cout << "the size is " << map_list.size()<< std::endl;
           std::cout << "The first kit is " << map_list.rbegin()->second<< std::endl;
           std::cout << "The thired kit is " << map_list[3]<< std::endl;*/
        if(map_list.find(2) != map_list.end()) {
                std::cout << "Find part "<< std::endl;
        }
        else {
                std::cout << "No part "<< std::endl;
        }

        return 0;
}

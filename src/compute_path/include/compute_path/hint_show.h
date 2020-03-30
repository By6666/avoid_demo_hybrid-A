#ifndef HINT_SHOW_H
#define HINT_SHOW_H

#include <iostream>
#include <string>

inline void HintShow(const std::string& str) {
  std::cout << std::endl;
  std::cout << "------------ HINT ------------" << std::endl;
  std::cout << "|** " << str << " **|" << std::endl;
  std::cout << "------------------------------" << std::endl;
  std::cout << std::endl;
}

#endif
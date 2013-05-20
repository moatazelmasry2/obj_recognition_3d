/*
 * utils.h
 *
 *  Created on: 24.08.2010
 *      Author: ropra
 */

#ifndef UTILS_H_
#define UTILS_H_

#include  <iostream>
#include <sstream>
#include <vector>

namespace myanalyeserns {

	class Utils {
	public:

		static std::string myItoa(int i) {
		  std::stringstream out;
		  out << i;
		  return out.str();
		}

		static std::string myFtoa(float i) {
			/*char falpha[128];
			sprintf(falpha,"%0.2f",i);
			return falpha;*/
		    std::stringstream out;
		    out << i;
		    return out.str();
		}

		static std::vector<std::string> splitLine(const std::string &s, char delim, std::vector<std::string> &elems) {
		    std::stringstream ss(s);
		    std::string item;
		    while(std::getline(ss, item, delim)) {
		        elems.push_back(item);
		    }
		    return elems;
		}


		/**
		 * default method used for splitting
		 */
		static std::vector<std::string> split(const std::string &s, char delim) {
		    std::vector<std::string> elems;
		    return splitLine(s, delim, elems);
		}

	};
}
#endif /* UTILS_H_ */

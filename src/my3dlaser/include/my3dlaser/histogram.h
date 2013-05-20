/*
 * histogram.h
 *
 *  Created on: 19.08.2010
 *      Author: ropra
 */

#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include <iostream>
#include <fstream>

namespace myanalyeserns {


class Histogram {

	std::string filename;

	public :

		Histogram() {
			for(int i = 0; i < 11; i++) {
				f1[i] = f2[i] = f3[i] = 0;
			}
		}

		std::string id;
		std::string name;

		float f1[11];
		float f2[11];
		float f3[11];
	};
}
#endif /* HISTOGRAM_H_ */

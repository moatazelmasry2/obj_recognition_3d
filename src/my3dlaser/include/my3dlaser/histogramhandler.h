/*
 * histogramfilehandler.h
 *
 *  Created on: 19.08.2010
 *      Author: ropra
 */

#ifndef HISTOGRAMFILEHANDLER_H_
#define HISTOGRAMFILEHANDLER_H_

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>


#include <boost/algorithm/string.hpp>

#include "my3dlaser/utils.h"
#include "my3dlaser/histogram.h"


#define HISTOGRAMS_FILE_PATH "/home/ropra/newhistograms.txt"

namespace myanalyeserns {

	class HistogramHandler {

	public:
		typedef myanalyeserns::Histogram Histogram;
		typedef std::vector<Histogram> HistVector;

	protected:
		Histogram parseLine(std::string line = HISTOGRAMS_FILE_PATH);
		std::string histogramToString(float f1[11], float f2[11], float f3[11]);

	public :

		Histogram * histograms;
		std::string filename;
		HistVector hists;

		HistogramHandler(std::string filename=HISTOGRAMS_FILE_PATH);
		HistVector readFile();
		HistVector getHistograms(std::string name);
		bool saveHistogram(myanalyeserns::Histogram hist, bool reset, bool override);
		bool saveHistogramAll(std::vector<myanalyeserns::Histogram> histVector, bool reset);
	};
}

#endif /* HISTOGRAMFILEHANDLER_H_ */

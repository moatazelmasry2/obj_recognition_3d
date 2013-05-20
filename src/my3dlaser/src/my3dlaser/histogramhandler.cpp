/*
 * histogramfilehandler.cpp
 *
 *  Created on: 19.08.2010
 *      Author: ropra
 */

#include "my3dlaser/histogramhandler.h"

myanalyeserns::HistogramHandler::HistogramHandler(std::string filename) {
	this->filename = filename;
}

myanalyeserns::HistogramHandler::HistVector myanalyeserns::HistogramHandler::readFile() {

	HistVector hists;

	std::filebuf fb;

	fb.open (this->filename.c_str(),std::ios::in);
	std::istream is(&fb);

	char line[1000];
	is.getline(line, 1000);
	while (line != NULL ) {
		std::string s = line;
		if (s.size() == 0 ) {
			break;
		}
		hists.push_back(parseLine(s));
		is.getline(line, 1000);
	}
	fb.close();
	std::cout <<"done readfile" << std::endl;
	this->hists = hists;
	return hists;
}


myanalyeserns::HistogramHandler::Histogram myanalyeserns::HistogramHandler::parseLine(std::string line) {

	std::vector<std::string> tokens;
	boost::split(tokens, line, boost::is_any_of("#"));
	Histogram histogram;
	histogram.id = tokens[0];
	histogram.name = tokens[1];

	std::vector<std::string> nums;
	std::string s;

	boost::split(nums, tokens[2], boost::is_any_of(","));
	if (nums.size() !=  33 ) {
		std::cerr << "HistogramHandler.cpp/parseLine(): number of tokens is less than 33, its: " << nums.size() <<std::endl;
		myanalyeserns::HistogramHandler::Histogram h;
		return h;
	}



	for (int i=0;  i< 11; i++) {

		histogram.f1[i] = std::atof(nums[i].c_str());
		histogram.f2[i] = std::atof(nums[i + 11].c_str());
		histogram.f3[i] = std::atof(nums[i + 22].c_str());

	}

	return histogram;
}


/*myanalyeserns::HistogramHandler::HistVector myanalyeserns::HistogramHandler::getHistograms(std::string name) {

	myanalyeserns::HistogramHandler::HistVector vec;
	//HistVector::iterator = vec.iterator();
}*/

bool myanalyeserns::HistogramHandler::saveHistogram(myanalyeserns::Histogram hist, bool reset, bool override) {

	std::filebuf fb;
	if (override) {
		fb.open (this->filename.c_str(),std::ios::out);
	} else {
		fb.open (this->filename.c_str(),std::ios::out | std::ios::app);
	}
	std::ostream os(&fb);
	os << hist.id << "#" << hist.name << "#" << histogramToString(hist.f1, hist.f2, hist.f3) << std::endl;
	fb.close();
	return true;
}


bool myanalyeserns::HistogramHandler::saveHistogramAll(std::vector<myanalyeserns::Histogram> histVector, bool reset) {

	if (reset && histVector.size() > 0 ) {
		std::filebuf fb;
		fb.open (this->filename.c_str(),std::ios::out);
		fb.close();
	}

	std::filebuf fb;
	fb.open (this->filename.c_str(),std::ios::out | std::ios::app);
	std::ostream os(&fb);
	std::vector<myanalyeserns::Histogram>::iterator it;
	for (it = histVector.begin(); it < histVector.end(); it++ ) {
		myanalyeserns::Histogram hist = *it;
		os << hist.id << "#" << hist.name << "#" << histogramToString(hist.f1, hist.f2, hist.f3) << std::endl;
	}
	fb.close();
	return true;

}

std::string myanalyeserns::HistogramHandler::histogramToString(float f1[11], float f2[11], float f3[11]) {

	std::string output;
	for (int i=0;i<11; i++) {

		output.append(myanalyeserns::Utils::myFtoa(f1[i]) );
		output.append(",");
	}

	for (int i=0;i<11; i++) {
		output.append(myanalyeserns::Utils::myFtoa(f2[i]) );
		output.append(",");
	}

	for (int i=0;i<11; i++) {
		output.append(myanalyeserns::Utils::myFtoa(f3[i]) );
		if (i < 10 ) {
			output.append(",");
		}
	}

	return output;
}

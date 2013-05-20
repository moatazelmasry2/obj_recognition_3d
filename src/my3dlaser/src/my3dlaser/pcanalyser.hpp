/*
 * pcanalyser.hpp
 *
 *  Created on: 17.08.2010
 *      Author: ropra
 */


#include <fstream>


/*void myanalyeserns::PointCloudAnalyser::onInit ()
{
	// Call the super onInit ()
	  PCLNodelet<pcl::FPFHSignature33>::onInit ();

	  nh = getMTPrivateNodeHandle();
	  //ros::NodeHandle nh;

	  //pub_output_ = nh.advertise<sensor_msgs::PointCloud2>("output", 5);
	  pub_output_ = nh.advertise<sensor_msgs::PointCloud2>("/tilt_scan/analyseroutput", 100);

	  //subscriber=nh.subscribe("input", 10, &myanalyeserns::PointCloudAnalyser::listenerCallback, this);
	  subscriber=nh.subscribe("/tilt_scan/output_fpfh", 100, &myanalyeserns::PointCloudAnalyser::listenerCallback, this);

	  // If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
	  //sub_input_.subscribe (nh, "input", max_queue_size_,  bind (&PointCloudAnalyser<pcl::PointXYZ>::listenerCallback, this, _1));
	  //message_filters::Subscriber<sensor_msgs::PointCloud2> m_sub_input_filter(nh, "input", 1);
	  //m_sub_input_filter.registerCallback(bind (&PointCloudAnalyser::listenerCallback, this, _1));

	  //pclros_filter.subscribe (nh, "input", 1,  bind (&PointCloudAnalyser::listenerCallback, this, _1));
	  counter = 1;
	  //histVector = histHandler.readFile();
}*/

void myanalyeserns::PointCloudAnalyser::onInit ()
{
	// Call the super onInit ()
	  PCLNodelet<pcl::FPFHSignature33>::onInit ();

	  //nh = getMTPrivateNodeHandle();
	  ros::NodeHandle nh;

	  pub_output_ = nh.advertise<sensor_msgs::PointCloud2>("/tilt_scan/analyseroutput", 100);

	  //subscriber=nh.subscribe("/tilt_scan/output_fpfh", 100, &myanalyeserns::PointCloudAnalyser::listenerCallback, this);

	  publisherCounter = 0;

	  sync_cloud_histogram = boost::make_shared <message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2 > > (3);
	  sub_cloud_filter_.subscribe (nh, "/tilt_scan/voxel_grid", 100);
	  sub_histogram_filter_.subscribe(nh, "/tilt_scan/output_fpfh", 100);
	  sync_cloud_histogram->connectInput (sub_cloud_filter_, sub_histogram_filter_);
	  sync_cloud_histogram->registerCallback (bind (&myanalyeserns::PointCloudAnalyser::cloud_histogram_callback, this, _1, _2));


	for (int i = 0; i < MAX_SINGLE_PUBLISHERS; i++ ) {

	  std::string pubName = "singlePub";
	  std::string num = myanalyeserns::Utils::myItoa(i);
	  pubName.append(num);
	  publishersVec[i] =  nh.advertise<sensor_msgs::PointCloud2>(pubName, 1);
	}

	testCompareObjects();
}

void myanalyeserns::PointCloudAnalyser::listenerCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

	NODELET_INFO("MESSAGE RECEIVED: %i", publisherCounter++);

	pcl::PointCloud<pcl::FPFHSignature33> cloud;

	///Signatur wird eingelesen
	pcl::fromROSMsg(*msg, cloud);

	//NODELET_INFO("receiving object with header.seq=%i", cloud.header.seq);
	NODELET_INFO("receiving object with header.seq=%i", cloud.header.seq);
	sensor_msgs::PointCloud2 output_blob;


	pcl::toROSMsg(cloud, output_blob);
	pub_output_.publish (*msg);

	 if (pcVector.size() > 0 ) {
		 NODELET_INFO("calling compareClouds");
		compareClouds3Features(cloud);
		//compareClouds(cloud);
	 } else {
		 NODELET_INFO("adding first object");
	 }
	 pcVector.push_back(cloud);
	 NODELET_INFO("DONE PUSHING");

	 //pcl_visualization::PCLHistogramVisualizer vis;
	 //vis.addFeatureHistogram(cloud, cloud.width);
	 //vis.spin();
}

void myanalyeserns::PointCloudAnalyser::cloud_histogram_callback(const sensor_msgs::PointCloud2::ConstPtr &voxelMsg, const sensor_msgs::PointCloud2::ConstPtr &histMsg) {

	ROS_INFO("cloud_histogram_callback: publisherCounter=%i", publisherCounter);
	//weiterleiten von input cloud
	if (publisherCounter >= MAX_SINGLE_PUBLISHERS) {
		publisherCounter = 0;
	}
	publishersVec[publisherCounter].publish(*voxelMsg);

	//normalize histogram and save it in file
	MyPointCloud histCloud;
	pcl::PointCloud<pcl::FPFHSignature33> cc;
	pcl::fromROSMsg(*histMsg, histCloud);
	Histogram hist3;
	hist3.id = myanalyeserns::Utils::myItoa(publisherCounter);
	computeNormalizedHistogram(histCloud, hist3.f1,hist3.f2,hist3.f3);
	publisherCounter++;
	histVector.push_back(hist3);
	histHandler.saveHistogram(hist3, false, false);
}



float myanalyeserns::PointCloudAnalyser::compareClouds(MyPointCloud input) {

	std::vector<MyPointCloud>::iterator it;

	std::cout << std::endl;

	for (it = pcVector.begin(); it < pcVector.end(); it++) {
		MyPointCloud obj2 = *it;
		float obj1Normalised[(int)obj2.points.size()], obj2Normalised[(int)obj2.points.size()];

		for (int i = 0; i < (int)input.points.size(); i++) {
			for (int j = 0; j < 33; j++) {
				if (i == 0 ) {
					obj1Normalised[0] += input.points[0].histogram[j];
					obj2Normalised[0] += (float)obj2.points[0].histogram[j];
				} else {
					obj1Normalised[i] += (float)input.points[i].histogram[j];
					obj2Normalised[i] += (float)obj2.points[i].histogram[j];
				}
			}
		}

		float sumObj1 = 0, sumObj2 = 0;

		for (int i=0; i< 33; i++) {
			sumObj1 += obj1Normalised[i];
			sumObj2 += obj2Normalised[i];
		}

		std::cout << "sumobject1: " <<sumObj1 <<"  -- Sumobject2: " << sumObj2<< std::endl;
		float result = 0;

		std::cout << "{";
		for (int i=0; i< 33; i++) {
			obj1Normalised[i] /= sumObj1;
			obj2Normalised[i] /= sumObj2;
			std::cout << obj1Normalised[i] << " " << obj2Normalised[i];
			std::cout << ",";

			//resultArrayNormalised[i] = min(obj1Normalised[i], obj2Normalised[i] /= sumObj2);
			result += std::min(obj1Normalised[i], obj2Normalised[i]);
		}
		std::cout << "}" << std::endl;
		std::cout << "result is: " << result << std::endl;
	}

	return 0.0f;
}

float myanalyeserns::PointCloudAnalyser::compareClouds3Features(MyPointCloud input) {

		MyPointCloud obj1 = input;

		std::cout << std::endl;

		std::vector<MyPointCloud>::iterator it;
		//Iterating over all point cloud histograms in our vector
		for (it = pcVector.begin(); it < pcVector.end(); it++) {
			MyPointCloud obj2 = *it;

			//6 arays that represent 3 features from obj1 + 3 features from obj2
			float obj1_f1[11] = {0.0}, obj1_f2[11] = {0.0},obj1_f3[11] = {0.0},obj2_f1[11] = {0.0},obj2_f2[11] = {0.0},obj2_f3[11] = {0.0};

			//Ierating over all points in the histogram
			for (int i = 0; i < (int)obj1.points.size(); i++) {
				//Every array has length 11, so iterating over 11
				for (int j = 0; j < 11; j++) {
					obj1_f1[j] += (float)obj1.points[i].histogram[j];
					obj2_f1[j] += (float)obj2.points[i].histogram[j];

					obj1_f2[j] += (float)obj1.points[i].histogram[j + 11];
					obj2_f2[j] += (float)obj2.points[i].histogram[j + 11];

					obj1_f3[j] += (float)obj1.points[i].histogram[j + 22];
					obj2_f3[j] += (float)obj2.points[i].histogram[j + 22];
				}
			}

			float sumObj1_f1 = 0, sumObj1_f2 = 0,sumObj1_f3 = 0,sumObj2_f1 = 0,sumObj2_f2 = 0,sumObj2_f3 = 0;

			//calculating the sum of each array/feature
			for (int i=0; i< 11; i++) {

				sumObj1_f1 += obj1_f1[i];
				sumObj1_f2 += obj1_f2[i];
				sumObj1_f3 += obj1_f3[i];
				sumObj2_f1 += obj2_f1[i];
				sumObj2_f2 += obj2_f2[i];
				sumObj2_f3 += obj2_f3[i];
			}

			float x1 = 0, x2 = 0, x3 = 0;

			//Normalizing each feature array with the calculated sum
			// And also calculating the sum of the min of each two corresponding features (f1 from the two objects)

			for (int i=0; i< 11; i++) {

				obj1_f1[i] /= sumObj1_f1;
				obj1_f2[i] /= sumObj1_f2;
				obj1_f3[i] /= sumObj1_f3;
				obj2_f1[i] /= sumObj2_f1;
				obj2_f2[i] /= sumObj2_f2;
				obj2_f3[i] /= sumObj2_f3;

				x1 += std::min(obj1_f1[i], obj2_f1[i]);
				x2 += std::min(obj1_f2[i], obj2_f2[i]);
				x3 += std::min(obj1_f3[i], obj2_f3[i]);

			}

			float prod = x1 * x2 * x3;
			float tmpSum = x1 + x2 + x3;
			float sum = 0;
			sum = 1 / (x1/tmpSum + x2/tmpSum  + x3/tmpSum);
			std::cout << "x1: " << x1 << " x2: " << x2 << "  x3: " << x3 << std::endl;
			std::cout << "sum is: " << tmpSum <<std::endl;
			std::cout << "prod is: " << prod <<std::endl;
		}
		return 0.0f;
}


void myanalyeserns::PointCloudAnalyser::computeNormalizedHistogram(MyPointCloud cloud, float *f1, float *f2, float *f3) {

	//Ierating over all points in the histogram
	for (int i = 0; i < (int)cloud.points.size(); i++) {
		//Every array has length 11, so iterating over 11
		for (int j = 0; j < 11; j++) {
			f1[j] += (float)cloud.points[i].histogram[j];
			f2[j] += (float)cloud.points[i].histogram[j + 11];
			f3[j] += (float)cloud.points[i].histogram[j + 22];

			if( (float)cloud.points[i].histogram[j] < 0 || (float)cloud.points[i].histogram[j + 11] < 0 || (float)cloud.points[i].histogram[j + 22] < 0 )
				ROS_ERROR("fpfh hist value negative");
		}
	}

	float sum_f1 = 0, sum_f2 = 0,sum_f3 = 0;

	//calculating the sum of each array/feature
	for (int i=0; i< 11; i++) {

		sum_f1 += f1[i];
		sum_f2 += f2[i];
		sum_f3 += f3[i];
	}

	//Normalizing each feature array with the calculated sum
	// And also calculating the sum of the min of each two corresponding features (f1 from the two objects)

	for (int i=0; i< 11; i++) {

		f1[i] /= sum_f1;
		f2[i] /= sum_f2;
		f3[i] /= sum_f3;
	}
}

void myanalyeserns::PointCloudAnalyser::testCompareObjects() {


	myanalyeserns::HistogramHandler h("/home/ropra/objects_dont_delete.txt");

	std::vector<myanalyeserns::Histogram> vec = h.readFile();

	Histogram hist = vec[0];

//	float similarityMatrix[vec.size()][vec.size()];
	std::ofstream outfile("/home/ropra/similarity.mat");

	for (int i=0;i < (int)vec.size(); i++) {
		for (int j = 0; j < (int)vec.size(); j++ ) {

			float x1=0, x2= 0, x3 = 0;
			for (int k = 0;k < 11; k++ ) {
				x1 += std::min(vec[i].f1[k], vec[j].f1[k]);
				x2 += std::min(vec[i].f2[k], vec[j].f2[k]);
				x3 += std::min(vec[i].f3[k], vec[j].f3[k]);
			}
			float prod = x1 * x2 * x3;
			float tmpSum = x1 + x2 + x3;

			outfile << prod << " ";
//			similarityMatrix[i][j] = prod;

			std::cout<< "comparing between " << vec[i].id.c_str() << " and " << vec[j].id.c_str() << std::endl;
			std::cout << "x1: " << x1 << " x2: " << x2 << "  x3: " << x3 << std::endl;
			std::cout << "sum is: " << tmpSum <<std::endl;
			std::cout << "prod is: " << prod <<std::endl << std::endl;
		}
		outfile << "\n";
	}
	outfile.close();


}

void myanalyeserns::PointCloudAnalyser::smallTests() {

	/*std::string s= "x,y,z";
	std::vector<std::string> tokens = myanalyeserns::Utils.split(s, ",");*/
}

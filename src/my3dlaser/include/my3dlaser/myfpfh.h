/*
 * myfpfh.h
 *
 *  Created on: 17.08.2010
 *      Author: ropra
 */

#ifndef MYFPFH_H_
#define MYFPFH_H_

#include <pcl/features/fpfh.h>
#include "my3dlaser/myfeature.h"

namespace myanalyeserns {

	template <typename PointInT, typename PointNT, typename PointOutT>
	class MyFPFHEstimation : virtual public MyFeatureFromNormals<PointInT, PointNT, PointOutT>, virtual public pcl::FPFHEstimation<PointInT, PointNT, PointOutT> {
		using MyFeatureFromNormals<PointInT, PointNT, PointOutT>::getName;
	}
}

#endif /* MYFPFH_H_ */

//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: q3DMASC                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: Dimitri Lague / CNRS / UEB                  #
//#                                                                        #
//##########################################################################

#include "q3DMASCTools.h"

//system
#include <assert.h>

using namespace masc;

bool Tools::RandomSubset(ccPointCloud* cloud, float ratio, CCLib::ReferenceCloud* inRatioSubset, CCLib::ReferenceCloud* outRatioSubset)
{
	if (!cloud)
	{
		ccLog::Warning("Invalid input cloud");
		return false;
	}
	if (!inRatioSubset || !outRatioSubset)
	{
		ccLog::Warning("Invalid input refence clouds");
		return false;
	}
	if (inRatioSubset->getAssociatedCloud() != cloud || outRatioSubset->getAssociatedCloud() != cloud)
	{
		ccLog::Warning("Invalid input reference clouds (associated cloud is wrong)");
		return false;
	}
	if (ratio < 0.0f || ratio > 1.0f)
	{
		ccLog::Warning(QString("Invalid parameter (ratio: %1)").arg(ratio));
		return false;
	}

	unsigned inSampleCount = static_cast<unsigned>(floor(cloud->size() * ratio));
	assert(inSampleCount <= cloud->size());
	unsigned outSampleCount = cloud->size() - inSampleCount;

	//we draw the smallest population (faster)
	unsigned targetCount = inSampleCount;
	bool defaultState = true;
	if (outSampleCount < inSampleCount)
	{
		targetCount = outSampleCount;
		defaultState = false;
	}

	//reserve memory
	std::vector<bool> pointInsideRatio;
	try
	{
		pointInsideRatio.resize(cloud->size(), defaultState);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return false;
	}

	if (!inRatioSubset->reserve(inSampleCount) || !outRatioSubset->reserve(outSampleCount))
	{
		ccLog::Warning("Not enough memory");
		inRatioSubset->clear();
		outRatioSubset->clear();
		return false;
	}

	//randomly choose the 'in' or 'out' indexes
	int randIndex = 0;
	unsigned randomCount = 0;
	while (randomCount < targetCount)
	{
		randIndex = ((randIndex + std::rand()) % cloud->size());
		if (pointInsideRatio[randIndex] == defaultState)
		{
			pointInsideRatio[randIndex] = !defaultState;
			++randomCount;
		}
	}

	//now dispatch the points
	{
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			if (pointInsideRatio[i])
				inRatioSubset->addPointIndex(i);
			else
				outRatioSubset->addPointIndex(i);
		}
		assert(inRatioSubset->size() == inSampleCount);
		assert(outRatioSubset->size() == outSampleCount);
	}

	return true;
}

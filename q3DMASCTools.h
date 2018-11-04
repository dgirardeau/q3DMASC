#pragma once

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

//Local
#include "FeaturesInterface.h"

//CCLib
#include <GenericProgressCallback.h>

//qCC_db
class ccPointCloud;

class QWidget;

//! 3DMASC classifier
namespace masc
{
	class Tools
	{
	public:

		static bool LoadFile(QString filename, Feature::Set& rawFeatures, std::vector<ccPointCloud*>& loadedClouds, CorePoints& corePoints);

		static bool SaveFeatureDescriptors(QString filename, const Feature::Set& features);

		static bool PrepareFeatures(const CorePoints& corePoints, Feature::Set& features, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr);

		static bool RandomSubset(ccPointCloud* cloud, float ratio, CCLib::ReferenceCloud* inRatioSubset, CCLib::ReferenceCloud* outRatioSubset);

		static CCLib::ScalarField* RetrieveSF(const ccPointCloud* cloud, const QString& sfName, bool caseSensitive = true);
	};

}; //namespace masc

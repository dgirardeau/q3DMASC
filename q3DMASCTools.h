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
#include "Features.h"

//qCC_db
#include <ccPointCloud.h>

//CCLib
#include <ReferenceCloud.h>

class QWidget;

//! 3DMASC classifier
namespace masc
{
	class Tools
	{
	public:

		static bool LoadFile(QString filename, ccPointCloud* pc1, ccPointCloud* pc2, FeatureRule::Set& features);

		static bool PrepareFeatures(const FeatureRule::Set& rules, Feature::Set& features, QString& error);

		static bool RandomSubset(ccPointCloud* cloud, float ratio, CCLib::ReferenceCloud* inRatioSubset, CCLib::ReferenceCloud* outRatioSubset);
	};

}; //namespace masc

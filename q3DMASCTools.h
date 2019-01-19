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
#include "q3DMASCClassifier.h"

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

		static bool LoadTrainingFile(QString filename, Feature::Set& rawFeatures, std::vector<ccPointCloud*>& loadedClouds, CorePoints& corePoints, TrainParameters& parameters);

		static bool SaveClassifier(QString filename, const Feature::Set& features, const masc::Classifier& classifier, QWidget* parent = nullptr);

		static bool LoadClassifierCloudLabels(QString filename, QSet<QString>& labels);

		typedef QMap<QString, ccPointCloud* > NamedClouds;

		static bool LoadClassifier(QString filename, const NamedClouds& clouds, Feature::Set& rawFeatures, masc::Classifier& classifier, QWidget* parent = nullptr);

		static bool PrepareFeatures(const CorePoints& corePoints, Feature::Set& features, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr, SFCollector* generatedScalarFields = nullptr);

		static bool RandomSubset(ccPointCloud* cloud, float ratio, CCLib::ReferenceCloud* inRatioSubset, CCLib::ReferenceCloud* outRatioSubset);

		static CCLib::ScalarField* RetrieveSF(const ccPointCloud* cloud, const QString& sfName, bool caseSensitive = true);
	};

}; //namespace masc

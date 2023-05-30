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

		typedef QMap<QString, ccPointCloud* > NamedClouds;

		static bool LoadTrainingFile(QString filename, Feature::Set& rawFeatures, std::vector<double>& scales, NamedClouds& loadedClouds, TrainParameters& parameters, CorePoints* corePoints = nullptr, QWidget* parent = nullptr);

		static bool LoadClassifierCloudLabels(QString filename, QList<QString>& labels, QString& corePointsLabel, bool& filenamesSpecified);

		static bool LoadClassifier(QString filename, NamedClouds& clouds, Feature::Set& rawFeatures, masc::Classifier& classifier, QWidget* parent = nullptr);

		static bool LoadFile(	const QString& filename,
								Tools::NamedClouds* clouds,
								bool cloudsAreProvided,
								std::vector<Feature::Shared>* rawFeatures = nullptr, //requires 'clouds'
								std::vector<double>* rawScales = nullptr,
								masc::CorePoints* corePoints = nullptr, //requires 'clouds'
								masc::Classifier* classifier = nullptr,
								TrainParameters* parameters = nullptr,
								QWidget* parent = nullptr);

		static bool SaveClassifier(QString filename, const Feature::Set& features, const QString corePointsRole, const masc::Classifier& classifier, QWidget* parent = nullptr);

        static bool PrepareFeatures(const CorePoints& corePoints, Feature::Set& features, QString& error,
                                    CCCoreLib::GenericProgressCallback* progressCb = nullptr, SFCollector* generatedScalarFields = nullptr);

		static bool RandomSubset(ccPointCloud* cloud, float ratio, CCCoreLib::ReferenceCloud* inRatioSubset, CCCoreLib::ReferenceCloud* outRatioSubset);

		static CCCoreLib::ScalarField* RetrieveSF(const ccPointCloud* cloud, const QString& sfName, bool caseSensitive = true);

		//! Helper: returns the classification SF associated to a cloud (if any)
		static CCCoreLib::ScalarField* GetClassificationSF(const ccPointCloud* cloud);
	};

}; //namespace masc

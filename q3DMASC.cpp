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

#include "q3DMASC.h"

//local
#include "q3DMASCDisclaimerDialog.h"
#include "Features.h"

//qCC_db
#include <ccPointCloud.h>

//qCC_io
#include <LASFields.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QMessageBox>
#include <QStringList>

//OpenCV
#include <opencv2/ml.hpp>

q3DMASCPlugin::q3DMASCPlugin(QObject* parent/*=0*/)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/q3DMASCPlugin/info.json" )
	, m_classifyAction(0)
	, m_trainAction(0)
{
}

void q3DMASCPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_classifyAction)
	{
		//classification: only one point cloud
		m_classifyAction->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
	}

	if (m_trainAction)
	{
		m_trainAction->setEnabled(m_app && m_app->dbRootObject() && m_app->dbRootObject()->getChildrenNumber() != 0); //need some loaded entities to train the classifier!
	}

	m_selectedEntities = selectedEntities;
}

QList<QAction*> q3DMASCPlugin::getActions()
{
	QList<QAction*> group;

	if (!m_trainAction)
	{
		m_trainAction = new QAction("Train classifier", this);
		m_trainAction->setToolTip("Train classifier");
		m_trainAction->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/q3DMASCPlugin/iconCreate.png")));
		connect(m_trainAction, SIGNAL(triggered()), this, SLOT(doTrainAction()));
	}
	group.push_back(m_trainAction);

	if (!m_classifyAction)
	{
		m_classifyAction = new QAction("Classify", this);
		m_classifyAction->setToolTip("Classify cloud");
		m_classifyAction->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/q3DMASCPlugin/iconClassify.png")));
		connect(m_classifyAction, SIGNAL(triggered()), this, SLOT(doClassifyAction()));
	}
	group.push_back(m_classifyAction);

	return group;
}

void q3DMASCPlugin::doClassifyAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	//disclaimer accepted?
	if (!ShowClassifyDisclaimer(m_app))
	{
		return;
	}

	if (m_selectedEntities.empty() || !m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities.front());

	//look for the classification field
	int classifSFIdx = cloud->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_CLASSIFICATION]); //LAS_FIELD_NAMES[LAS_CLASSIFICATION] = "Classification"
	if (!classifSFIdx)
	{
		m_app->dispToConsole("Missing 'Classification' field", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	CCLib::ScalarField* classifSF = cloud->getScalarField(classifSFIdx);
	if (!classifSF || classifSF->size() < cloud->size())
	{
		assert(false);
		return;
	}

	RTParams params;
	if (params.testDataRatio < 0 || params.testDataRatio > 0.99f)
	{
		m_app->dispToConsole("Invalid test data ratio", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	std::vector<Feature> features;
	features.push_back(Feature(Feature::DimZ, "Z"));
	features.push_back(Feature(Feature::ScalarField, "Intensity"));
	features.push_back(Feature(Feature::ScalarField, "Intensity"));

	int totalSampleCount = static_cast<int>(cloud->size());
	int testSampleCount = static_cast<int>(floor(totalSampleCount * params.testDataRatio));
	int sampleCount = totalSampleCount - testSampleCount;
	int attributesPerSample = static_cast<int>(features.size());

	m_app->dispToConsole(QString("[3DMASC] Training data: %1 samples with %2 feature(s) / %3 test samples").arg(sampleCount).arg(attributesPerSample).arg(testSampleCount), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//choose the sample indexes
	std::vector<bool> isSample;
	{
		try
		{
			isSample.resize(totalSampleCount, true);
		}
		catch (const std::bad_alloc&)
		{
			m_app->dispToConsole("Not enough memory", ccMainAppInterface::STD_CONSOLE_MESSAGE);
			return;
		}

		int randomCount = 0;
		while (randomCount < testSampleCount)
		{
			int randIndex = (std::rand() % totalSampleCount);
			if (isSample[randIndex])
			{
				isSample[randIndex] = false;
				++randomCount;
			}
		}
	}

	//NUMBER_OF_TRAINING_SAMPLES = number of points
	//ATTRIBUTES_PER_SAMPLE = number of scalar fields
	cv::Mat training_data, train_labels;
	cv::Mat test_data, test_labels;
	try
	{
		training_data.create(sampleCount, attributesPerSample, CV_32FC1);
		train_labels.create(sampleCount, 1, CV_8U);

		test_data.create(testSampleCount, attributesPerSample, CV_32FC1);
		test_labels.create(testSampleCount, 1, CV_8U);
	}
	catch (const cv::Exception& cvex)
	{
		ccLog::Error(cvex.msg.c_str());
		return;
	}

	//fill the classification labels vector
	{
		unsigned sampleIndex = 0;
		unsigned testSampleIndex = 0;
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			ScalarType pointClass = classifSF->getValue(i);
			int iClass = static_cast<int>(pointClass);
			if (iClass < 0 || iClass > 255)
			{
				m_app->dispToConsole("Classification values out of range (0-255)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}

			if (isSample[i])
			{
				train_labels.at<unsigned char>(sampleIndex++) = static_cast<unsigned char>(iClass);
			}
			else
			{
				test_labels.at<unsigned char>(testSampleIndex++) = static_cast<unsigned char>(iClass);
			}
		}
		assert(testSampleIndex + testSampleIndex == totalSampleCount);
	}


	//fill the training data matrix
	for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
	{
		QScopedPointer<IScalarFieldWrapper> source(nullptr);

		const Feature& f = features[fIndex];
		switch (f.source)
		{
		case Feature::ScalarField:
		{
			int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(f.name));
			if (sfIdx >= 0)
			{
				source.reset(new ScalarFieldWrapper(cloud->getScalarField(sfIdx)));
			}
			else
			{
				ccLog::Error(QString("Internal error: unknwon scalar field '%1'").arg(f.name));
				return;
			}
		}
		break;

		case Feature::DimX:
			source.reset(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimX));
			break;
		case Feature::DimY:
			source.reset(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimY));
			break;
		case Feature::DimZ:
			source.reset(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimZ));
			break;

		case Feature::Red:
			source.reset(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Red));
			break;
		case Feature::Green:
			source.reset(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Green));
			break;
		case Feature::Blue:
			source.reset(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Blue));
			break;
		}

		if (!source || !source->isValid())
		{
			assert(false);
			ccLog::Error(QString("Internal error: invalid source '%1'").arg(f.name));
		}

		unsigned sampleIndex = 0;
		unsigned testSampleIndex = 0;
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			double value = source->pointValue(i);
			if (isSample[i])
			{
				training_data.at<float>(sampleIndex++, fIndex) = static_cast<float>(value);
			}
			else
			{
				test_data.at<float>(testSampleIndex++, fIndex) = static_cast<float>(value);
			}
		}
		assert(testSampleIndex + testSampleIndex == totalSampleCount);
	}

	cv::Ptr<cv::ml::RTrees> rtrees;
	rtrees = cv::ml::RTrees::create();
	rtrees->setMaxDepth(params.maxDepth);
	rtrees->setMinSampleCount(params.minSampleCount);
	rtrees->setCalculateVarImportance(params.calcVarImportance);
	rtrees->setActiveVarCount(params.activeVarCount);
	cv::TermCriteria terminationCriteria(cv::TermCriteria::MAX_ITER, params.maxTreeCount, std::numeric_limits<double>::epsilon());
	rtrees->setTermCriteria(terminationCriteria);
	
	//rtrees->setRegressionAccuracy(0);
	//rtrees->setUseSurrogates(false);
	//rtrees->setMaxCategories(params.maxCategories); //not important?
	//rtrees->setPriors(cv::Mat());

	rtrees->train(training_data, cv::ml::ROW_SAMPLE, train_labels);

	if (!rtrees->isTrained())
	{
		//an error occurred?
		return;
	}

	//estimate the efficiency of the classiier
	{
		int goodGuessCount = 0;
		for (int j = 0; j < testSampleCount; ++j)
		{
			if (rtrees->predict(test_data.row(j)) == test_labels.at<int>(j))
			{
				++goodGuessCount;
			}
		}

		float acc = static_cast<float>(goodGuessCount) / testSampleCount;

		m_app->dispToConsole(QString("Correct = %1 / %2 --> Accuracy = %3").arg(goodGuessCount).arg(testSampleCount).arg(acc), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

}

//OpenCV

void q3DMASCPlugin::doTrainAction()
{
	//disclaimer accepted?
	if (!ShowTrainDisclaimer(m_app))
		return;

	//if (m_selectedEntities.size() != 2
	//	|| !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
	//	|| !m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
	//{
	//	m_app->dispToConsole("Select two point clouds!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	//	return;
	//}
	//
	//ccPointCloud* cloud1 = static_cast<ccPointCloud*>(m_selectedEntities[0]);
	//ccPointCloud* cloud2 = static_cast<ccPointCloud*>(m_selectedEntities[1]);
}

void q3DMASCPlugin::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	//cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandCanupoClassif));
}

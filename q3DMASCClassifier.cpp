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

#include "q3DMASCClassifier.h"

//Local
#include "ScalarFieldWrappers.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccLog.h>

//qCC_io
#include <LASFields.h>

//Qt
#include <QCoreApplication>
#include <QProgressDialog>

using namespace masc;

Classifier::Classifier()
{
}

bool Classifier::isValid() const
{
	return (m_rtrees && m_rtrees->isTrained());
}


bool Classifier::train(const TrainParameters& params, const Feature::Set& features, QString& errorMessage, QWidget* parentWidget/*=nullptr*/)
{
	if (features.empty())
	{
		errorMessage = QObject::tr("Training method called without any feature?!");
		return false;
	}
	if (!features.front() || !features.front()->cloud)
	{
		errorMessage = QObject::tr("Invalid feature (no associated point cloud");
		return false;
	}
	ccPointCloud* cloud = features.front()->cloud;

	//look for the classification field
	int classifSFIdx = cloud->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_CLASSIFICATION]); //LAS_FIELD_NAMES[LAS_CLASSIFICATION] = "Classification"
	if (!classifSFIdx)
	{
		errorMessage = QObject::tr("Missing 'Classification' field on input cloud");
		return false;
	}
	CCLib::ScalarField* classifSF = cloud->getScalarField(classifSFIdx);
	if (!classifSF || classifSF->size() < cloud->size())
	{
		assert(false);
		errorMessage = QObject::tr("Invalid 'Classification' field on input cloud");
		return false;
	}

	if (params.testDataRatio < 0 || params.testDataRatio > 0.99f)
	{
		errorMessage = QObject::tr("Invalid parameter (test data ratio)");
		return false;
	}

	//std::vector<Feature::Shared> features;
	//features.push_back(Feature::Shared(new PointFeature(cloud, PointFeature::Z, Feature::DimZ, "Z")));
	//features.push_back(Feature::Shared(new PointFeature(cloud, PointFeature::Intensity, Feature::ScalarField, "Intensity")));

	int totalSampleCount = static_cast<int>(cloud->size());
	int testSampleCount = static_cast<int>(floor(totalSampleCount * params.testDataRatio));
	int sampleCount = totalSampleCount - testSampleCount;
	int attributesPerSample = static_cast<int>(features.size());

	ccLog::Print(QString("[3DMASC] Training data: %1 samples with %2 feature(s) / %3 test samples").arg(sampleCount).arg(attributesPerSample).arg(testSampleCount));

	//randomly choose the sample indexes
	std::vector<bool> isSample;
	if (testSampleCount > 0)
	{
		try
		{
			isSample.resize(totalSampleCount, true);
		}
		catch (const std::bad_alloc&)
		{
			errorMessage = QObject::tr("Not enough memory");
			return false;
		}

		int randomCount = 0;
		int randIndex = 0;
		while (randomCount < testSampleCount)
		{
			randIndex = ((randIndex + std::rand()) % totalSampleCount);
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
		train_labels.create(sampleCount, 1, CV_32FC1);

		test_data.create(testSampleCount, attributesPerSample, CV_32FC1);
		test_labels.create(testSampleCount, 1, CV_32FC1);
	}
	catch (const cv::Exception& cvex)
	{
		errorMessage = cvex.msg.c_str();
		return false;
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
				errorMessage = QObject::tr("Classification values out of range (0-255)");
				return false;
			}

			if (isSample[i])
			{
				train_labels.at<float>(sampleIndex++) = static_cast<unsigned char>(iClass);
			}
			else
			{
				test_labels.at<float>(testSampleIndex++) = static_cast<unsigned char>(iClass);
			}
		}
		assert(sampleIndex + testSampleIndex == totalSampleCount);
	}


	//fill the training data matrix
	for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
	{
		QScopedPointer<IScalarFieldWrapper> source(nullptr);

		const Feature::Shared &f = features[fIndex];
		switch (f->source)
		{
		case Feature::ScalarField:
		{
			int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(f->sourceName));
			if (sfIdx >= 0)
			{
				source.reset(new ScalarFieldWrapper(cloud->getScalarField(sfIdx)));
			}
			else
			{
				errorMessage = QObject::tr("Internal error: unknwon scalar field '%1'").arg(f->sourceName);
				return false;
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
			errorMessage = QObject::tr("Internal error: invalid source '%1'").arg(f->sourceName);
			return false;
		}

		unsigned sampleIndex = 0;
		unsigned testSampleIndex = 0;
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			double value = source->pointValue(i);
			if (isSample[i])
			{
				assert(sampleIndex < sampleCount);
				training_data.at<float>(sampleIndex++, fIndex) = static_cast<float>(value);
			}
			else
			{
				assert(testSampleIndex< testSampleCount);
				test_data.at<float>(testSampleIndex++, fIndex) = static_cast<float>(value);
			}
		}
		assert(sampleIndex + testSampleIndex == totalSampleCount);
	}

	QProgressDialog pDlg(parentWidget);
	pDlg.setRange(0, 0); //infinite loop
	pDlg.setLabelText("Training classifier");
	pDlg.show();
	QCoreApplication::processEvents();

	m_rtrees = cv::ml::RTrees::create();
	m_rtrees->setMaxDepth(params.rt.maxDepth);
	m_rtrees->setMinSampleCount(params.rt.minSampleCount);
	m_rtrees->setCalculateVarImportance(params.rt.calcVarImportance);
	m_rtrees->setActiveVarCount(params.rt.activeVarCount);
	cv::TermCriteria terminationCriteria(cv::TermCriteria::MAX_ITER, params.rt.maxTreeCount, std::numeric_limits<double>::epsilon());
	m_rtrees->setTermCriteria(terminationCriteria);
	
	//rtrees->setRegressionAccuracy(0);
	//rtrees->setUseSurrogates(false);
	//rtrees->setMaxCategories(params.maxCategories); //not important?
	//rtrees->setPriors(cv::Mat());
	try
	{
		m_rtrees->train(training_data, cv::ml::ROW_SAMPLE, train_labels);
	}
	catch (const cv::Exception& cvex)
	{
		m_rtrees.release();
		errorMessage = cvex.msg.c_str();
		return false;
	}
	catch (const std::exception& stdex)
	{
		errorMessage = stdex.what();
		return false;
	}
	catch (...)
	{
		errorMessage = QObject::tr("Unknown error");
		return false;
	}

	pDlg.hide();
	QCoreApplication::processEvents();

	if (!m_rtrees->isTrained())
	{
		errorMessage = QObject::tr("Training failed for an unknown reason...");
		m_rtrees.release();
		return false;
	}

	//estimate the efficiency of the classiier
	{
		int goodGuessCount = 0;
		for (int j = 0; j < testSampleCount; ++j)
		{
			if (m_rtrees->predict(test_data.row(j)) == test_labels.at<float>(j))
			{
				++goodGuessCount;
			}
		}

		float acc = static_cast<float>(goodGuessCount) / testSampleCount;
		ccLog::Print(QString("Correct = %1 / %2 --> Accuracy = %3").arg(goodGuessCount).arg(testSampleCount).arg(acc));
	}

	//QString outputFilename = QCoreApplication::applicationDirPath() + "/classifier.yaml";
	return true;
}

bool Classifier::toFile(QString filename, QWidget* parentWidget/*=nullptr*/) const
{
	if (!m_rtrees)
	{
		ccLog::Warning(QObject::tr("Classifier hasn't been trained, can't save it"));
		return false;
	}
	
	//save the classifier
	QProgressDialog pDlg(parentWidget);
	pDlg.setRange(0, 0); //infinite loop
	pDlg.setLabelText(QObject::tr("Saving classifier"));
	pDlg.show();
	QCoreApplication::processEvents();

	m_rtrees->save(filename.toStdString());
	
	pDlg.close();
	QCoreApplication::processEvents();

	ccLog::Print("Classifier file saved to: " + filename);
	return true;
}

bool Classifier::fromFile(QString filename, QWidget* parentWidget/*=nullptr*/)
{
	//load the classifier
	QProgressDialog pDlg(parentWidget);
	pDlg.setRange(0, 0); //infinite loop
	pDlg.setLabelText(QObject::tr("Saving classifier"));
	pDlg.show();
	QCoreApplication::processEvents();

	m_rtrees = cv::ml::RTrees::load(filename.toStdString());

	pDlg.close();
	QCoreApplication::processEvents();

	if (!m_rtrees->isTrained())
	{
		ccLog::Warning(QObject::tr("Loaded classifier doesn't seem to be trained"));
	}

	return true;
}

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
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ccLog.h>

//qCC_io
#include <LASFields.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QCoreApplication>
#include <QProgressDialog>
#include <QtConcurrent>

using namespace masc;

Classifier::Classifier()
{
}

bool Classifier::isValid() const
{
	return (m_rtrees && m_rtrees->isTrained());
}

static QSharedPointer<IScalarFieldWrapper> GetSource(const Feature::Shared& f, const ccPointCloud* cloud)
{
	QSharedPointer<IScalarFieldWrapper> source(nullptr);
	if (!f)
	{
		assert(false);
		ccLog::Warning(QObject::tr("Internal error: invalid feature (nullptr)"));
	}

	switch (f->source)
	{
	case Feature::ScalarField:
	{
		assert(!f->sourceName.isEmpty());
		int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(f->sourceName));
		if (sfIdx >= 0)
		{
			source.reset(new ScalarFieldWrapper(cloud->getScalarField(sfIdx)));
		}
		else
		{
			ccLog::Warning(QObject::tr("Internal error: unknwon scalar field '%1'").arg(f->sourceName));
			return QSharedPointer<IScalarFieldWrapper>(nullptr);
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

	return source;
}

bool Classifier::classify(const Feature::Set& features, ccPointCloud* cloud, QString& errorMessage, QWidget* parentWidget/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		errorMessage = QObject::tr("Invalid input");
		return false;
	}
	
	if (!m_rtrees || !m_rtrees->isTrained())
	{
		errorMessage = QObject::tr("Classifier hasn't been trained yet");
		return false;
	}

	if (features.empty())
	{
		errorMessage = QObject::tr("Training method called without any feature?!");
		return false;
	}

	//look for the classification field
	CCLib::ScalarField* classificationSF = nullptr;
	int classifSFIdx = cloud->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_CLASSIFICATION]); //LAS_FIELD_NAMES[LAS_CLASSIFICATION] = "Classification"
	if (classifSFIdx < 0)
	{
		//create it if necessary
		ccScalarField* _classificationSF = new ccScalarField(LAS_FIELD_NAMES[LAS_CLASSIFICATION]);
		if (!_classificationSF->resizeSafe(cloud->size()))
		{
			_classificationSF->release();
			errorMessage = QObject::tr("Not enough memory");
			return false;
		}
		classifSFIdx = cloud->addScalarField(_classificationSF);
		classificationSF = _classificationSF;
	}
	else
	{
		classificationSF = cloud->getScalarField(classifSFIdx);
	}
	assert(classificationSF);
	classificationSF->fill(0); //0 = no classification?

	int sampleCount = static_cast<int>(cloud->size());
	int attributesPerSample = static_cast<int>(features.size());

	ccLog::Print(QObject::tr("[3DMASC] Classifying %1 points with %2 feature(s)").arg(sampleCount).arg(attributesPerSample));

	//allocate the data matrix
	cv::Mat test_data;
	try
	{
		test_data.create(1, attributesPerSample, CV_32FC1);
	}
	catch (const cv::Exception& cvex)
	{
		errorMessage = cvex.msg.c_str();
		return false;
	}

	//create the field wrappers
	std::vector< QSharedPointer<IScalarFieldWrapper> > wrappers;
	{
		wrappers.reserve(attributesPerSample);
		for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
		{
			const Feature::Shared &f = features[fIndex];
			if (!f)
			{
				assert(false);
				return false;
			}

			QSharedPointer<IScalarFieldWrapper> source = GetSource(f, cloud);
			if (!source || !source->isValid())
			{
				assert(false);
				errorMessage = QObject::tr("Internal error: invalid source '%1'").arg(f->sourceName);
				return false;
			}

			wrappers.push_back(source);
		}
	}

	QScopedPointer<ccProgressDialog> pDlg;
	if (parentWidget)
	{
		pDlg.reset(new ccProgressDialog(parentWidget));
		pDlg->setLabelText(QString("Classify (%1 points)").arg(sampleCount));
		pDlg->show();
		QCoreApplication::processEvents();
	}
	CCLib::NormalizedProgress nProgress(pDlg.data(), cloud->size());

	bool success = true;
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
		{
			double value = wrappers[fIndex]->pointValue(i);
			test_data.at<float>(0, fIndex) = static_cast<float>(value);
		}
		
		float predictedClass = m_rtrees->predict(test_data.row(0));
		classificationSF->setValue(i, static_cast<int>(predictedClass));

		if (pDlg && !nProgress.oneStep())
		{
			//process cancelled by the user
			success = false;
			break;
		}
	}
	classificationSF->computeMinAndMax();

	cloud->setCurrentDisplayedScalarField(classifSFIdx);
	cloud->showSF(true);
	if (parentWidget && cloud->getDisplay())
	{
		cloud->getDisplay()->redraw();
		QCoreApplication::processEvents();
	}

	return success;
}

bool Classifier::evaluate(const Feature::Set& features, CCLib::ReferenceCloud* testSubset, AccuracyMetrics& metrics, QString& errorMessage, QWidget* parentWidget/*=nullptr*/)
{
	metrics.sampleCount = metrics.goodGuess = 0;
	metrics.ratio = 0.0f;

	if (!m_rtrees || !m_rtrees->isTrained())
	{
		errorMessage = QObject::tr("Classifier hasn't been trained yet");
		return false;
	}

	if (features.empty())
	{
		errorMessage = QObject::tr("Training method called without any feature?!");
		return false;
	}
	if (!testSubset)
	{
		errorMessage = QObject::tr("No test subset provided");
		return false;
	}
	ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(testSubset->getAssociatedCloud());
	if (!cloud)
	{
		errorMessage = QObject::tr("Invalid test subset (associated point cloud is not a ccPointCloud)");
		return false;
	}

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

	int testSampleCount = static_cast<int>(testSubset->size());
	int attributesPerSample = static_cast<int>(features.size());

	ccLog::Print(QObject::tr("[3DMASC] Testing data: %1 samples with %2 feature(s)").arg(testSampleCount).arg(attributesPerSample));

	//allocate the data matrix
	cv::Mat test_data;
	try
	{
		test_data.create(testSampleCount, attributesPerSample, CV_32FC1);
	}
	catch (const cv::Exception& cvex)
	{
		errorMessage = cvex.msg.c_str();
		return false;
	}

	QScopedPointer<ccProgressDialog> pDlg;
	if (parentWidget)
	{
		pDlg.reset(new ccProgressDialog(parentWidget));
		pDlg->setLabelText(QString("Evaluating the classifier on %1 points").arg(testSampleCount));
		pDlg->show();
		QCoreApplication::processEvents();
	}
	CCLib::NormalizedProgress nProgress(pDlg.data(), testSampleCount);

	//fill the data matrix
	for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
	{
		const Feature::Shared &f = features[fIndex];
		if (!f)
		{
			assert(false);
			return false;
		}

		QSharedPointer<IScalarFieldWrapper> source = GetSource(f, cloud);
		if (!source || !source->isValid())
		{
			assert(false);
			errorMessage = QObject::tr("Internal error: invalid source '%1'").arg(f->sourceName);
			return false;
		}

		for (unsigned i = 0; i < testSubset->size(); ++i)
		{
			unsigned pointIndex = testSubset->getPointGlobalIndex(i);
			double value = source->pointValue(pointIndex);
			test_data.at<float>(i, fIndex) = static_cast<float>(value);
		}
	}

	//estimate the efficiency of the classifier
	{
		metrics.sampleCount = testSubset->size();
		metrics.goodGuess = 0;

		for (unsigned i = 0; i < testSubset->size(); ++i)
		{
			unsigned pointIndex = testSubset->getPointGlobalIndex(i);
			ScalarType pointClass = classifSF->getValue(pointIndex);
			int iClass = static_cast<int>(pointClass);
			//if (iClass < 0 || iClass > 255)
			//{
			//	errorMessage = QObject::tr("Classification values out of range (0-255)");
			//	return false;
			//}

			float predictedClass = m_rtrees->predict(test_data.row(i));
			if (static_cast<int>(predictedClass) == iClass)
			{
				++metrics.goodGuess;
			}

			if (pDlg && !nProgress.oneStep())
			{
				//process cancelled by the user
				return false;
			}
		}

		metrics.ratio = static_cast<float>(metrics.goodGuess) / metrics.sampleCount;
	}

	return true;
}

bool Classifier::train(	const ccPointCloud* cloud,
						const RandomTreesParams& params,
						const Feature::Set& features,
						QString& errorMessage,
						CCLib::ReferenceCloud* trainSubset/*=nullptr*/,
						ccMainAppInterface* app/*=nullptr*/,
						QWidget* parentWidget/*=nullptr*/)
{
	if (features.empty())
	{
		errorMessage = QObject::tr("Training method called without any feature?!");
		return false;
	}
	if (!cloud)
	{
		errorMessage = QObject::tr("Invalid input cloud");
		return false;
	}

	if (trainSubset && trainSubset->getAssociatedCloud() != cloud)
	{
		errorMessage = QObject::tr("Invalid train subset (associated point cloud is different)");
		return false;
	}

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

	int sampleCount = static_cast<int>(trainSubset ? trainSubset->size() : cloud->size());
	int attributesPerSample = static_cast<int>(features.size());

	if (app)
	{
		app->dispToConsole(QString("[3DMASC] Training data: %1 samples with %2 feature(s)").arg(sampleCount).arg(attributesPerSample));
	}

	cv::Mat training_data, train_labels;
	try
	{
		training_data.create(sampleCount, attributesPerSample, CV_32FC1);
		train_labels.create(sampleCount, 1, CV_32FC1);
	}
	catch (const cv::Exception& cvex)
	{
		errorMessage = cvex.msg.c_str();
		return false;
	}

	//fill the classification labels vector
	{
		for (int i = 0; i < sampleCount; ++i)
		{
			int pointIndex = (trainSubset ? static_cast<int>(trainSubset->getPointGlobalIndex(i)) : i);
			ScalarType pointClass = classifSF->getValue(pointIndex);
			int iClass = static_cast<int>(pointClass);
			//if (iClass < 0 || iClass > 255)
			//{
			//	errorMessage = QObject::tr("Classification values out of range (0-255)");
			//	return false;
			//}

			train_labels.at<float>(i) = static_cast<unsigned char>(iClass);
		}
	}

	//fill the training data matrix
	for (int fIndex = 0; fIndex < attributesPerSample; ++fIndex)
	{
		const Feature::Shared &f = features[fIndex];

		QSharedPointer<IScalarFieldWrapper> source = GetSource(f, cloud);
		if (!source || !source->isValid())
		{
			assert(false);
			errorMessage = QObject::tr("Internal error: invalid source '%1'").arg(f->sourceName);
			return false;
		}

		for (int i = 0; i < sampleCount; ++i)
		{
			int pointIndex = (trainSubset ? static_cast<int>(trainSubset->getPointGlobalIndex(i)) : i);
			double value = source->pointValue(pointIndex);
			training_data.at<float>(i, fIndex) = static_cast<float>(value);
		}
	}

	QScopedPointer<QProgressDialog> pDlg;
	if (parentWidget)
	{
		pDlg.reset(new QProgressDialog(parentWidget));
		pDlg->setRange(0, 0); //infinite loop
		pDlg->setLabelText("Training classifier");
		pDlg->show();
		QCoreApplication::processEvents();
	}

	m_rtrees = cv::ml::RTrees::create();
	m_rtrees->setMaxDepth(params.maxDepth);
	m_rtrees->setMinSampleCount(params.minSampleCount);
	m_rtrees->setCalculateVarImportance(true);
	m_rtrees->setActiveVarCount(params.activeVarCount);
	cv::TermCriteria terminationCriteria(cv::TermCriteria::MAX_ITER, params.maxTreeCount, std::numeric_limits<double>::epsilon());
	m_rtrees->setTermCriteria(terminationCriteria);
	
	//rtrees->setRegressionAccuracy(0);
	//rtrees->setUseSurrogates(false);
	//rtrees->setMaxCategories(params.maxCategories); //not important?
	//rtrees->setPriors(cv::Mat());

	QFuture<bool> future = QtConcurrent::run([&]()
	{
		// Code in this block will run in another thread
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
		return true;
	});

	while (!future.isFinished())
	{
#if defined(CC_WINDOWS)
		::Sleep(500);
#else
		usleep(500 * 1000);
#endif
		if (pDlg)
		{
			if (pDlg->wasCanceled())
			{
				future.cancel();
				break;
			}
			pDlg->setValue(pDlg->value() + 1);
		}
		QCoreApplication::processEvents();
	}

	if (pDlg)
	{
		pDlg->close();
		QCoreApplication::processEvents();
	}

	if (future.isCanceled() || !future.result() || !m_rtrees->isTrained())
	{
		errorMessage = QObject::tr("Training failed for an unknown reason...");
		m_rtrees.release();
		return false;
	}

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

	cv::String cvFilename = filename.toStdString();
	m_rtrees->save(cvFilename);
	
	pDlg.close();
	QCoreApplication::processEvents();

	ccLog::Print("Classifier file saved to: " + QString::fromStdString(cvFilename));
	return true;
}

bool Classifier::fromFile(QString filename, QWidget* parentWidget/*=nullptr*/)
{
	//load the classifier
	QScopedPointer<QProgressDialog> pDlg;
	if (parentWidget)
	{
		pDlg.reset(new QProgressDialog(parentWidget));
		pDlg->setRange(0, 0); //infinite loop
		pDlg->setLabelText(QObject::tr("Loading classifier"));
		pDlg->show();
		QCoreApplication::processEvents();
	}
	
	try
	{
		m_rtrees = cv::ml::RTrees::load(filename.toStdString());
	}
	catch (const cv::Exception& cvex)
	{
		ccLog::Warning(cvex.msg.c_str());
		ccLog::Error("Failed to load file: " + filename);
		return false;
	}

	if (pDlg)
	{
		pDlg->close();
		QCoreApplication::processEvents();
	}

	if (!m_rtrees->isTrained())
	{
		ccLog::Warning(QObject::tr("Loaded classifier doesn't seem to be trained"));
	}

	return true;
}

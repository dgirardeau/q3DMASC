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
#include "q3DMASCClassifier.h"
#include "q3DMASCTools.h"
#include "qClassify3DMASCDialog.h"
#include "qTrain3DMASCDialog.h"
#include "q3DMASCCommands.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>

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
		//m_classifyAction->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
		m_classifyAction->setEnabled(m_app->dbRootObject()->getChildrenNumber() != 0);
	}

	if (m_trainAction)
	{
		//m_trainAction->setEnabled(m_app && m_app->dbRootObject() && m_app->dbRootObject()->getChildrenNumber() != 0); //need some loaded entities to train the classifier!
		m_trainAction->setEnabled(true);
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

	QString inputFilename;
	{
		QSettings settings;
		settings.beginGroup("3DMASC");
		QString inputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
		inputFilename = QFileDialog::getOpenFileName(m_app->getMainWindow(), "Load 3DMASC classifier file", inputPath, "*.txt");
		if (inputFilename.isNull())
		{
			//process cancelled by the user
			return;
		}
		settings.setValue("FilePath", QFileInfo(inputFilename).absolutePath());
		settings.endGroup();
	}

	QSet<QString> cloudLabels;
	QString corePointsLabel;
	bool filenamesSpecified = false;
	if (!masc::Tools::LoadClassifierCloudLabels(inputFilename, cloudLabels, corePointsLabel, filenamesSpecified))
	{
		m_app->dispToConsole("Failed to read classifier file (see Console)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	if (cloudLabels.empty())
	{
		m_app->dispToConsole("Invalid classifier file (no cloud label defined)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	else if (cloudLabels.size() > 3)
	{
		m_app->dispToConsole("This classifier uses more than 3 clouds (the GUI version cannot handle it)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//now show a dialog where the user will be able to set the cloud roles
	Classify3DMASCDialog classifDlg(m_app);
	classifDlg.setCloudRoles(cloudLabels, corePointsLabel);
	classifDlg.classifierFileLineEdit->setText(inputFilename);
	static bool s_keepAttributes = false;
	classifDlg.keepAttributesCheckBox->setChecked(s_keepAttributes);
	if (!classifDlg.exec())
	{
		//process cancelled by the user
		return;
	}

	s_keepAttributes = classifDlg.keepAttributesCheckBox->isChecked();

	masc::Tools::NamedClouds clouds;
	QString mainCloudLabel;
	classifDlg.getClouds(clouds, mainCloudLabel);

	masc::Feature::Set features;
	masc::Classifier classifier;
	if (!masc::Tools::LoadClassifier(inputFilename, clouds, features, classifier, m_app->getMainWindow()))
	{
		return;
	}

	if (clouds.contains("TEST"))
	{
		//remove the test cloud (if any)
		delete clouds["TEST"];
		clouds.remove("TEST");
	}

	//the 'main cloud' is the cloud that should be classified
	masc::CorePoints corePoints;
	corePoints.origin = corePoints.cloud = clouds[mainCloudLabel];
	corePoints.role = mainCloudLabel;

	//prepare the main cloud
	ccProgressDialog progressDlg(true, m_app->getMainWindow());
	progressDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
	QString error;
	SFCollector generatedScalarFields;
	if (!masc::Tools::PrepareFeatures(corePoints, features, error, &progressDlg, &generatedScalarFields))
	{
		m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		generatedScalarFields.releaseAllSFs();
		return;
	}
	progressDlg.close();
	QCoreApplication::processEvents();
	progressDlg.setAutoClose(true); //restore the default behavior of the progress dialog

	//apply classifier
	{
		QString errorMessage;
		if (!classifier.classify(features, corePoints.cloud, errorMessage, m_app->getMainWindow()))
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			generatedScalarFields.releaseAllSFs();
			return;
		}

		if (!s_keepAttributes)
		{
			generatedScalarFields.releaseAllSFs();
		}
	}
}

struct FeatureSelection
{
	FeatureSelection(masc::Feature::Shared f = masc::Feature::Shared(nullptr)) : feature(f) {}
	masc::Feature::Shared feature;
	bool selected = true;
	bool prepared = false;
	float importance = std::numeric_limits<float>::quiet_NaN();
};

void q3DMASCPlugin::doTrainAction()
{
	//disclaimer accepted?
	if (!ShowTrainDisclaimer(m_app))
		return;

	QString inputFilename;
	{
		QSettings settings;
		settings.beginGroup("3DMASC");
		QString inputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
		inputFilename = QFileDialog::getOpenFileName(m_app->getMainWindow(), "Load 3DMASC training file", inputPath, "*.txt");
		if (inputFilename.isNull())
		{
			//process cancelled by the user
			return;
		}
		settings.setValue("FilePath", QFileInfo(inputFilename).absolutePath());
		settings.endGroup();
	}

	//load the cloud labels (PC1, PC2, CTX, etc.)
	QSet<QString> cloudLabels;
	QString corePointsLabel;
	bool filenamesSpecified = false;
	if (!masc::Tools::LoadClassifierCloudLabels(inputFilename, cloudLabels, corePointsLabel, filenamesSpecified))
	{
		m_app->dispToConsole("Failed to read classifier file (see Console)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	if (cloudLabels.empty())
	{
		m_app->dispToConsole("Invalid classifier file (no cloud label defined)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	static bool s_keepAttributes = false;
	masc::Tools::NamedClouds loadedClouds;
	masc::CorePoints corePoints;

	//if no filename is specified in the training file, we are bound to ask the user to specify them
	bool useCloudsFromDB = (!filenamesSpecified || QMessageBox::question(m_app->getMainWindow(), "Use clouds in DB", "Use clouds in db (yes) or clouds specified in the file(no)?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
	QString mainCloudLabel;
	if (useCloudsFromDB)
	{
		if (cloudLabels.size() > 3)
		{
			m_app->dispToConsole("This classifier uses more than 3 different clouds (the GUI version cannot handle it)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			return;
		}

		//now show a dialog where the user will be able to set the cloud roles
		Classify3DMASCDialog classifDlg(m_app, true);
		classifDlg.setWindowTitle("3DMASC Train");
		classifDlg.setCloudRoles(cloudLabels, corePointsLabel);
		classifDlg.classifierFileLineEdit->setText(inputFilename);
		classifDlg.keepAttributesCheckBox->setChecked(s_keepAttributes);
		if (!classifDlg.exec())
		{
			//process cancelled by the user
			return;
		}
		s_keepAttributes = classifDlg.keepAttributesCheckBox->isChecked();

		classifDlg.getClouds(loadedClouds, mainCloudLabel);
		m_app->dispToConsole("Training cloud: " + mainCloudLabel, ccMainAppInterface::STD_CONSOLE_MESSAGE);
		corePoints.origin = loadedClouds[mainCloudLabel];
		corePoints.role = mainCloudLabel;
	}

	static masc::TrainParameters s_params;
	masc::Feature::Set features;
	if (!masc::Tools::LoadTrainingFile(inputFilename, features, loadedClouds, corePoints, s_params))
	{
		m_app->dispToConsole("Failed to load the training file", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!corePoints.origin)
	{
		m_app->dispToConsole("Core points not defined", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!masc::Classifier::GetClassificationSF(corePoints.origin))
	{
		m_app->dispToConsole("Missing 'Classification' field on core points cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* group = new ccHObject("3DMASC");
	if (!useCloudsFromDB)
	{
		//add the loaded clouds to the main DB (so that we don't need to handle them anymore)
		for (masc::Tools::NamedClouds::const_iterator it = loadedClouds.begin(); it != loadedClouds.end(); ++it)
		{
			group->addChild(it.value());
		}
	}

	for (masc::Tools::NamedClouds::const_iterator it = loadedClouds.begin(); it != loadedClouds.end(); ++it)
	{
		m_app->dispToConsole(it.key() + " = " + it.value()->getName(), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	//show the training dialog for the first time
	Train3DMASCDialog trainDlg(m_app->getMainWindow());
	trainDlg.maxDepthSpinBox->setValue(s_params.rt.maxDepth);
	trainDlg.maxTreeCountSpinBox->setValue(s_params.rt.maxTreeCount);
	trainDlg.activeVarCountSpinBox->setValue(s_params.rt.activeVarCount);
	trainDlg.minSampleCountSpinBox->setValue(s_params.rt.minSampleCount);
	trainDlg.testDataRatioSpinBox->setValue(static_cast<int>(s_params.testDataRatio * 100));

	//display the loaded features and let the user select the ones to use
	trainDlg.setResultText("Select features and press 'Run'");
	std::vector<FeatureSelection> originalFeatures;
	originalFeatures.reserve(features.size());
	for (const masc::Feature::Shared& f : features)
	{
		originalFeatures.push_back(FeatureSelection(f));
		trainDlg.addFeature(f->toString(), originalFeatures.back().importance, originalFeatures.back().selected);
	}
	if (!trainDlg.exec())
	{
		delete group;
		return;
	}
	assert(!trainDlg.shouldSaveClassifier()); //the save button should be disabled at this point

	//compute the core points (if necessary)
	ccProgressDialog progressDlg(true, m_app->getMainWindow());
	if (!corePoints.prepare(&progressDlg))
	{
		m_app->dispToConsole("Failed to compute/prepare the core points!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete group;
		return;
	}
	
	if (corePoints.cloud != corePoints.origin)
	{
		//auto-hide the other clouds
		for (ccPointCloud* pc : loadedClouds)
		{
			pc->setEnabled(false);
		}
		//set an explicit name for the core points
		QString corePointsName = corePoints.origin->getName();
		switch (corePoints.selectionMethod)
		{
		case masc::CorePoints::NONE:
			break;
		case masc::CorePoints::RANDOM:
			corePointsName += "_SS_Random@" + QString::number(corePoints.selectionParam);
			break;
		case masc::CorePoints::SPATIAL:
			corePointsName += "_SS_Spatial@" + QString::number(corePoints.selectionParam);
			break;
		default:
			assert(false);
		}
		corePoints.cloud->setName(QString("Core points (%1)").arg(corePointsName));
		group->addChild(corePoints.cloud);
	}
	
	if (group->getChildrenNumber() != 0)
	{
		m_app->addToDB(group);
		QCoreApplication::processEvents();
	}
	else
	{
		delete group;
		group = nullptr;
	}

	//test role
	ccPointCloud* testCloud = nullptr;
	if (loadedClouds.contains("TEST"))
	{
		testCloud = loadedClouds["TEST"];
		loadedClouds.remove("TEST");
	}


	//train / test subsets
	QScopedPointer<CCLib::ReferenceCloud> trainSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	QScopedPointer<CCLib::ReferenceCloud> testSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	float previousTestSubsetRatio = -1.0f;

	SFCollector generatedScalarFields;

	for (int iteration = 0; ; ++iteration)
	{
		//look for selected features
		features.clear();
		masc::Feature::Set toPrepare;
		for (size_t i = 0; i < originalFeatures.size(); ++i)
		{
			originalFeatures[i].selected = trainDlg.isFeatureSelected(i);

			//if the feature is selected
			if (originalFeatures[i].selected)
			{
				if (!originalFeatures[i].prepared)
				{
					//we should prepare it first!
					toPrepare.push_back(originalFeatures[i].feature);
				}
				features.push_back(originalFeatures[i].feature);
			}
		}

		if (features.empty())
		{
			m_app->dispToConsole("No feature selected!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			continue;
		}

		//prepare the features
		if (!toPrepare.empty())
		{
			progressDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
			QString error;
			if (!masc::Tools::PrepareFeatures(corePoints, toPrepare, error, &progressDlg, &generatedScalarFields))
			{
				m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				generatedScalarFields.releaseAllSFs();
				return;
			}
			progressDlg.setAutoClose(true); //restore the default behavior of the progress dialog
			progressDlg.close();
			QCoreApplication::processEvents();
			m_app->redrawAll();

			//flag the prepared features as 'prepared' ;)
			for (FeatureSelection& fs : originalFeatures)
			{
				if (fs.selected && !fs.prepared)
					fs.prepared = true;
			}
		}

		masc::Classifier classifier;

		//retrieve parameters
		s_params.rt.maxDepth = trainDlg.maxDepthSpinBox->value();
		s_params.rt.maxTreeCount = trainDlg.maxTreeCountSpinBox->value();
		s_params.rt.activeVarCount = trainDlg.activeVarCountSpinBox->value();
		s_params.rt.minSampleCount = trainDlg.minSampleCountSpinBox->value();
		float testDataRatio = s_params.testDataRatio = trainDlg.testDataRatioSpinBox->value() / 100.0f;
		QScopedPointer<CCLib::ReferenceCloud> testSubset2;
		if (testCloud)
		{
			m_app->dispToConsole("Test data cloud provided (ignoring test data ratio)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			testDataRatio = 0.0f;
			testSubset2.reset(new CCLib::ReferenceCloud(testCloud));
			if (!testSubset2->reserve(testCloud->size()))
			{
				m_app->dispToConsole("Not enough memory to evaluate the classifier", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				generatedScalarFields.releaseAllSFs();
				return;
			}
			testSubset2->addPointIndex(0, testCloud->size());
		}

		if (testDataRatio < 0.0f || testDataRatio > 0.99f)
		{
			assert(false);
			m_app->dispToConsole("Invalid test data ratio", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		else
		{
			if (previousTestSubsetRatio != testDataRatio)
			{
				//randomly select the training points
				testSubset->clear();
				trainSubset->clear();
				if (!masc::Tools::RandomSubset(corePoints.cloud, testDataRatio, testSubset.data(), trainSubset.data()))
				{
					m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					generatedScalarFields.releaseAllSFs();
					return;
				}
				previousTestSubsetRatio = testDataRatio;
			}

			//train the classifier
			{
				QString errorMessage;
				if (!classifier.train(corePoints.cloud, s_params.rt, features, errorMessage, trainSubset.data(), m_app, m_app->getMainWindow()))
				{
					m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					generatedScalarFields.releaseAllSFs();
					return;
				}
				trainDlg.setFirstRunDone();
				trainDlg.shouldSaveClassifier();
			}

			//test the trained classifier
			{
				masc::Classifier::AccuracyMetrics metrics;
				QString errorMessage;
				if (!classifier.evaluate(features, testSubset2 ? testSubset2.data() : testSubset.data(), metrics, errorMessage, m_app->getMainWindow()))
				{
					m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					generatedScalarFields.releaseAllSFs();
					return;
				}

				QString resultText = QString("Correct guess = %1 / %2 --> accuracy = %3").arg(metrics.goodGuess).arg(metrics.sampleCount).arg(metrics.ratio);
				m_app->dispToConsole(resultText, ccMainAppInterface::STD_CONSOLE_MESSAGE);
				trainDlg.setResultText(resultText);

				cv::Mat importanceMat = classifier.getVarImportance();
				//m_app->dispToConsole(QString("Var importance size = %1 x %2").arg(importanceMat.rows).arg(importanceMat.cols));
				assert(static_cast<int>(features.size()) == importanceMat.rows);
				int selectedFeatureIndex = 0;
				for (size_t i = 0; i < originalFeatures.size(); ++i)
				{
					if (originalFeatures[i].selected)
					{
						//m_app->dispToConsole(QString("Feature #%1 importance = %2").arg(i + 1).arg(importanceMat.at<float>(i, 0)));
						assert(selectedFeatureIndex < importanceMat.rows);
						originalFeatures[i].importance = importanceMat.at<float>(selectedFeatureIndex, 0);
						++selectedFeatureIndex;
					}
					else
					{
						originalFeatures[i].importance = std::numeric_limits<float>::quiet_NaN();
					}
					trainDlg.setFeatureImportance(i, originalFeatures[i].importance);
				}

				trainDlg.sortByFeatureImportance();
			}
		}

		while (true)
		{
			if (!trainDlg.exec())
			{
				//the dialog can be closed
				if (!s_keepAttributes)
				{
					generatedScalarFields.releaseAllSFs();
				}
				return;
			}

			//if the save button has been clicked
			if (trainDlg.shouldSaveClassifier())
			{
				//ask for the output filename
				QString outputFilename;
				{
					QSettings settings;
					settings.beginGroup("3DMASC");
					QString outputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
					outputFilename = QFileDialog::getSaveFileName(m_app->getMainWindow(), "Save 3DMASC classifier", outputPath, "*.txt");
					if (outputFilename.isNull())
					{
						//process cancelled by the user
						continue;
					}
					settings.setValue("FilePath", QFileInfo(outputFilename).absolutePath());
					settings.endGroup();
				}

				//save the classifier
				if (masc::Tools::SaveClassifier(outputFilename, features, classifier, m_app->getMainWindow()))
				{
					m_app->dispToConsole("Classifier succesfully saved to " + outputFilename, ccMainAppInterface::STD_CONSOLE_MESSAGE);
					trainDlg.setClassifierSaved();
				}
				else
				{
					m_app->dispToConsole("Failed to save classifier file");
				}
			}
			else //we will run the classifier another time
			{
				//stop the local loop
				break;
			}
		}

		//we are going to restart the classification process
	}
}

void q3DMASCPlugin::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new Command3DMASCClassif));
}

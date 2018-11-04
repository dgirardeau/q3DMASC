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

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QFileDialog>

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
	if (!masc::Tools::LoadClassifierCloudLabels(inputFilename, cloudLabels))
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
	classifDlg.setCloudRoles(cloudLabels);
	if (!classifDlg.exec())
	{
		//process cancelled by the user
		return;
	}

	masc::Tools::NamedClouds clouds;
	QString mainCloudLabel;
	classifDlg.getClouds(clouds, mainCloudLabel);

	masc::Feature::Set features;
	masc::Classifier classifier;
	if (!masc::Tools::LoadClassifier(inputFilename, clouds, features, classifier, m_app->getMainWindow()))
	{
		return;
	}

	//the 'main cloud' is the cloud that should be classified
	masc::CorePoints corePoints;
	corePoints.origin = corePoints.cloud = clouds[mainCloudLabel];

	//prepare the main cloud
	ccProgressDialog pDlg(true, m_app->getMainWindow());
	pDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
	QString error;
	if (!masc::Tools::PrepareFeatures(corePoints, features, error, &pDlg))
	{
		m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	pDlg.close();
	QCoreApplication::processEvents();
	pDlg.setAutoClose(true); //restore the default behavior of the progress dialog

	//apply classifier
	{
		QString errorMessage;
		if (!classifier.classify(features, corePoints.cloud, errorMessage, m_app->getMainWindow()))
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}
}

void q3DMASCPlugin::doTrainAction()
{
	//disclaimer accepted?
	if (!ShowTrainDisclaimer(m_app))
		return;

	masc::TrainParameters params;
	if (params.testDataRatio < 0 || params.testDataRatio > 0.99f)
	{
		m_app->dispToConsole("Invalid test data ratio", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

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

	std::vector<ccPointCloud*> loadedClouds;
	masc::CorePoints corePoints;
	masc::Feature::Set features;
	if (!masc::Tools::LoadTrainingFile(inputFilename, features, loadedClouds, corePoints))
	{
		while (!loadedClouds.empty())
		{
			delete loadedClouds.back();
			loadedClouds.pop_back();
		}
		return;
	}

	//add the loaded clouds to the main DB (so that we don't need to handle them anymore)
	ccHObject* group = new ccHObject("3DMASC");
	for (ccPointCloud* pc : loadedClouds)
	{
		group->addChild(pc);
	}

	ccProgressDialog pDlg(true, m_app->getMainWindow());
	if (!corePoints.prepare(&pDlg))
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
	m_app->addToDB(group);
	QCoreApplication::processEvents();

	pDlg.setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
	QString error;
	if (!masc::Tools::PrepareFeatures(corePoints, features, error, &pDlg))
	{
		m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete group;
		return;
	}
	pDlg.close();
	QCoreApplication::processEvents();
	pDlg.setAutoClose(true); //restore the default behavior of the progress dialog

	//randomly select the training points
	QScopedPointer<CCLib::ReferenceCloud> trainSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	QScopedPointer<CCLib::ReferenceCloud> testSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	if (!masc::Tools::RandomSubset(corePoints.cloud, params.testDataRatio, testSubset.data(), trainSubset.data()))
	{
		m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//train the classifier
	masc::Classifier classifier;
	{
		QString errorMessage;
		if (!classifier.train(corePoints.cloud, params.rt, features, errorMessage, trainSubset.data(), m_app->getMainWindow()))
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		QString outputFilename;
		{
			QSettings settings;
			settings.beginGroup("3DMASC");
			QString outputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
			outputFilename = QFileDialog::getSaveFileName(m_app->getMainWindow(), "Save 3DMASC classifier", outputPath, "*.txt");
			if (outputFilename.isNull())
			{
				//process cancelled by the user
				return;
			}
			settings.setValue("FilePath", QFileInfo(outputFilename).absolutePath());
			settings.endGroup();
		}

		//save the classifier
		if (masc::Tools::SaveClassifier(outputFilename, features, classifier, m_app->getMainWindow()))
		{
			m_app->dispToConsole("Classifier succesfully saved to " + outputFilename, ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
	}

	//test classifier
	{
		masc::Classifier::AccuracyMetrics metrics;
		QString errorMessage;
		if (!classifier.evaluate(features, testSubset.data(), metrics, errorMessage, m_app->getMainWindow()))
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		m_app->dispToConsole(QString("Correct = %1 / %2 --> accuracy = %3").arg(metrics.goodGuess).arg(metrics.sampleCount).arg(metrics.ratio), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
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

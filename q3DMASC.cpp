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
#include "Features.h"

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
		m_classifyAction->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
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

	//TODO
}

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

	masc::TrainParameters params;
	if (params.testDataRatio < 0 || params.testDataRatio > 0.99f)
	{
		m_app->dispToConsole("Invalid test data ratio", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	Feature::Set features;
#if 0
	if (m_selectedEntities.empty() || !m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities.front());

	//features
	{
		Feature::Shared featureZ(new PointFeature(PointFeature::Z, cloud));
		features.push_back(featureZ);
		
		Feature::Shared featureIntensity(new PointFeature(PointFeature::Intensity, cloud));
		features.push_back(featureIntensity);
	}
#else
	QString inputFilename;
	{
		QSettings settings;
		settings.beginGroup("3DMASC");
		QString inputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
		inputFilename = QFileDialog::getOpenFileName(m_app->getMainWindow(), "Load 3DMASC script file", inputPath, "*.txt");
		if (inputFilename.isNull())
		{
			//process cancelled by the user
			return;
		}
		settings.setValue("FilePath", QFileInfo(inputFilename).absolutePath());
		settings.endGroup();
	}

	FeatureRule::Set rules;
	std::vector<ccPointCloud*> loadedClouds;
	masc::CorePoints corePoints;
	if (!masc::Tools::LoadFile(inputFilename, rules, loadedClouds, corePoints))
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
	if (!masc::Tools::PrepareFeatures(rules, corePoints, features, error, &pDlg))
	{
		m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete group;
		return;
	}
	pDlg.close();
	QCoreApplication::processEvents();
	pDlg.setAutoClose(true); //restore the default behavior of the progress dialog

#endif

	//randomly select the training points
	QScopedPointer<CCLib::ReferenceCloud> trainSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	QScopedPointer<CCLib::ReferenceCloud> testSubset(new CCLib::ReferenceCloud(corePoints.cloud));
	if (!masc::Tools::RandomSubset(corePoints.cloud, params.testDataRatio, trainSubset.data(), testSubset.data()))
	{
		m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	masc::Classifier classifier;
	//QString outputFilename = QCoreApplication::applicationDirPath() + "/classifier.yaml";
	//if (QFile(outputFilename).exists())
	//{
	//	if (!classifier.fromFile(outputFilename, m_app->getMainWindow()))
	//	{
	//		m_app->dispToConsole("Failed to load previous classifier file", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	//		return;
	//	}
	//	m_app->dispToConsole("Previous classifier loaded", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	//}
	//else
	{
		QString errorMessage;
		if (!classifier.train(params.rt, features, errorMessage, trainSubset.data(), m_app->getMainWindow()))
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		QString outputFilename;
		{
			QSettings settings;
			settings.beginGroup("3DMASC");
			QString outputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
			outputFilename = QFileDialog::getSaveFileName(m_app->getMainWindow(), "Save 3DMASC classifier", outputPath, "*.yaml");
			if (outputFilename.isNull())
			{
				//process cancelled by the user
				return;
			}
			settings.setValue("FilePath", QFileInfo(outputFilename).absolutePath());
			settings.endGroup();
		}

		//save the classifier
		classifier.toFile(outputFilename, m_app->getMainWindow());
		m_app->dispToConsole("Classifier succesfully created", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
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

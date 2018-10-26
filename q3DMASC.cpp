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
#include "Features.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>

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

	if (m_selectedEntities.empty() || !m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities.front());

	masc::TrainParameters params;
	if (params.testDataRatio < 0 || params.testDataRatio > 0.99f)
	{
		m_app->dispToConsole("Invalid test data ratio", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	Feature::Set features;
	{
		features.push_back(Feature::Shared(new PointFeature(cloud, PointFeature::Z, Feature::DimZ, "Z")));
		features.push_back(Feature::Shared(new PointFeature(cloud, PointFeature::Intensity, Feature::ScalarField, "Intensity")));
	}

	masc::Classifier classifier;
	QString errorMessage;
	if (!classifier.train(params, features, errorMessage, m_app->getMainWindow()))
	{
		m_app->dispToConsole(errorMessage);
		return;
	}

	//save the classifier
	QString outputFilename = QCoreApplication::applicationDirPath() + "/classifier.yaml";
	classifier.toFile(outputFilename, m_app->getMainWindow());
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

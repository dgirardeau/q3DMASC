//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qClassify3DMASCDialog.h"

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
//#include <QApplication>

//system
#include <limits>

static ccPointCloud* GetCloudFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
	assert(comboBox && dbRoot);
	if (!comboBox || !dbRoot)
	{
		assert(false);
		return 0;
	}

	//return the cloud currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return 0;
	}
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return 0;
	}
	return static_cast<ccPointCloud*>(item);
}

Classify3DMASCDialog::Classify3DMASCDialog(ccMainAppInterface* app, bool trainMode/*=false*/)
	: QDialog(app ? app->getMainWindow() : 0)
	, Ui::Classify3DMASCDialog()
	, m_app(app)
{
	setupUi(this);

	if (m_app)
	{
		//add list of clouds to the combo-boxes
		ccHObject::Container clouds;
		if (m_app->dbRootObject())
		{
			m_app->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
		}

		unsigned cloudCount = 0;
		for (size_t i = 0; i < clouds.size(); ++i)
		{
			if (clouds[i]->isA(CC_TYPES::POINT_CLOUD)) //as filterChildren only test 'isKindOf'
			{
				QString name = clouds[i]->getName() + QString(" [%1]").arg(clouds[i]->getUniqueID());
				QVariant uniqueID(clouds[i]->getUniqueID());
				cloud1ComboBox->addItem(name, uniqueID);
				cloud2ComboBox->addItem(name, uniqueID);
				cloud3ComboBox->addItem(name, uniqueID);
				cloud4ComboBox->addItem(name, uniqueID);
				testCloudComboBox->addItem(name, uniqueID);
				++cloudCount;
			}
		}

		//if 3 clouds are loaded, then there's chances that the first one is the global  cloud!
		cloud1ComboBox->setCurrentIndex(/*cloudCount > 0 ? (cloudCount > 2 ? 1 : 0) : */-1);
		connect(cloud1ComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onCloudChanged(int)));
		cloud2ComboBox->setCurrentIndex(/*cloudCount > 1 ? (cloudCount > 2 ? 2 : 1) : */-1);
		connect(cloud2ComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onCloudChanged(int)));
		cloud3ComboBox->setCurrentIndex(/*cloudCount > 2 ? 0 : */-1);
		connect(cloud3ComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onCloudChanged(int)));
		cloud4ComboBox->setCurrentIndex(/*cloudCount > 3 ? 0 : */-1);
		connect(cloud4ComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onCloudChanged(int)));
		testCloudComboBox->setCurrentIndex(-1);

		if (cloudCount == 0 && app)
		{
			app->dispToConsole(QString("You need at least 1 loaded cloud to ") + (trainMode ? "train a classifier" : "classify it..."), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
	}

	if (trainMode)
	{
		label->setText(tr("Trainer file"));
		warningLabel->setVisible(false);
	}

	onCloudChanged(0);
}

void Classify3DMASCDialog::setCloudRoles(const QList<QString>& roles, QString corePointsLabel)
{
	int index = 0;
	for (const QString& role : roles)
	{
		switch (index)
		{
		case 0:
			cloud1RadioButton->setText(role);
			if (corePointsLabel.isEmpty() || corePointsLabel == role)
				cloud1RadioButton->setChecked(true);
			break;
		case 1:
			cloud2RadioButton->setText(role);
			if (corePointsLabel == role)
				cloud2RadioButton->setChecked(true);
			break;
		case 2:
			cloud3RadioButton->setText(role);
			if (corePointsLabel == role)
				cloud3RadioButton->setChecked(true);
			break;
		case 3:
			cloud4RadioButton->setText(role);
			if (corePointsLabel == role)
				cloud4RadioButton->setChecked(true);
			break;
		default:
			//this dialog can't handle more than 3 roles!
			break;
		}
		++index;
	}

	if (index < 1)
	{
		cloud1RadioButton->setEnabled(false);
		cloud1ComboBox->setEnabled(false);
	}
	if (index < 2)
	{
		cloud2RadioButton->setEnabled(false);
		cloud2RadioButton->setVisible(false);
		cloud2ComboBox->setVisible(false);
	}
	if (index < 3)
	{
		cloud3RadioButton->setEnabled(false);
		cloud3RadioButton->setVisible(false);
		cloud3ComboBox->setVisible(false);
	}
	if (index < 4)
	{
		cloud4RadioButton->setEnabled(false);
		cloud4RadioButton->setVisible(false);
		cloud4ComboBox->setVisible(false);
	}
}

void Classify3DMASCDialog::getClouds(QMap<QString, ccPointCloud*>& clouds, QString& mainCloud) const
{
	if (!m_app)
	{
		assert(false);
		return;
	}
	
	if (cloud1RadioButton->isEnabled())
	{
		clouds.insert(cloud1RadioButton->text(), GetCloudFromCombo(cloud1ComboBox, m_app->dbRootObject()));
		if (cloud1RadioButton->isChecked())
		{
			mainCloud = cloud1RadioButton->text();
		}
	}
	if (cloud2RadioButton->isEnabled())
	{
		clouds.insert(cloud2RadioButton->text(), GetCloudFromCombo(cloud2ComboBox, m_app->dbRootObject()));
		if (cloud2RadioButton->isChecked())
		{
			mainCloud = cloud2RadioButton->text();
		}
	}
	if (cloud3RadioButton->isEnabled())
	{
		clouds.insert(cloud3RadioButton->text(), GetCloudFromCombo(cloud3ComboBox, m_app->dbRootObject()));
		if (cloud3RadioButton->isChecked())
		{
			mainCloud = cloud3RadioButton->text();
		}
	}
	if (cloud4RadioButton->isEnabled())
	{
		clouds.insert(cloud4RadioButton->text(), GetCloudFromCombo(cloud4ComboBox, m_app->dbRootObject()));
		if (cloud4RadioButton->isChecked())
		{
			mainCloud = cloud4RadioButton->text();
		}
	}
	if (testCloudComboBox->currentIndex() >= 0)
	{
		clouds.insert("TEST", GetCloudFromCombo(testCloudComboBox, m_app->dbRootObject()));
	}
}

void Classify3DMASCDialog::onCloudChanged(int dummy)
{
	if (!cloud1RadioButton->isEnabled())
	{
		//this means that no role has been defined yet
		buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
		return;
	}

	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(cloud1ComboBox->currentIndex() >= 0);
}

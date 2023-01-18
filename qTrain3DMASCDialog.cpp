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

#include "qTrain3DMASCDialog.h"

//Qt
#include <QTableWidgetItem>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>
#include <QTextStream>

//System
#include <assert.h>

#include <iostream>

static const int FeatureImportanceColumn = 1;

Train3DMASCDialog::Train3DMASCDialog(QWidget* parent/*=nullptr*/)
	: QDialog(parent)
	, Ui::Train3DMASCDialog()
	, classifierSaved(false)
	, saveRequested(false)
{
	setupUi(this);

	connect(closePushButton, SIGNAL(clicked()), this, SLOT(onClose()));
	connect(savePushButton, SIGNAL(clicked()), this, SLOT(onSave()));
	connect(exportToolButton, SIGNAL(clicked()), this, SLOT(onExportResults()));
}

void Train3DMASCDialog::clearResults()
{
	resultLabel->clear();
	tableWidget->clear();
}

int Train3DMASCDialog::addFeature(QString name, float importance, bool isChecked/*=true*/)
{
	int index = tableWidget->rowCount();
	tableWidget->setRowCount(index + 1);

	QTableWidgetItem* nameItem = new QTableWidgetItem(name);
	nameItem->setCheckState(isChecked ? Qt::Checked : Qt::Unchecked);
	tableWidget->setItem(index, 0, nameItem);

	QTableWidgetItem* importanceItem = new QTableWidgetItem(isnan(importance) ? QString() : QString::number(importance));
	tableWidget->setItem(index, 1, importanceItem);

	return index;
}

int Train3DMASCDialog::addScale(double scale, bool isChecked/*=true*/)
{
	int index = tableWidgetScales->rowCount();
	tableWidgetScales->setRowCount(index + 1);

	QTableWidgetItem* nameItem = new QTableWidgetItem(QString::number(scale));
	nameItem->setCheckState(isChecked ? Qt::Checked : Qt::Unchecked);
	tableWidgetScales->setItem(index, 0, nameItem);

	return index;
}

void Train3DMASCDialog::scaleStateChanged(QTableWidgetItem* item)
{
	// if the scale is not checked, remove automatically features based on this scale
	QString scale = "_SC" + item->text() + "_";
	for (int row = 0; row < tableWidget->rowCount(); row++)
	{
		QTableWidgetItem* nameItem = tableWidget->item(row, 0);
		QString name = nameItem->text();
		if (name.contains(scale))
			nameItem->setCheckState(item->checkState());
	}
}

void Train3DMASCDialog::connectScaleSelectionToFeatureSelection()
{
	connect(tableWidgetScales, &QTableWidget::itemChanged, this, &Train3DMASCDialog::scaleStateChanged);
}

void Train3DMASCDialog::setResultText(QString text)
{
	resultLabel->setText(text);
}

void Train3DMASCDialog::setFirstRunDone()
{
	runPushButton->setText(tr("Retry"));
	savePushButton->setEnabled(true);
}

bool Train3DMASCDialog::isFeatureSelected(QString featureName) const
{
	for (int index = 0; index < tableWidget->rowCount(); ++index)
	{
		QTableWidgetItem* item = tableWidget->item(index, 0);
		if (item->text() == featureName)
		{
			return (item->checkState() == Qt::Checked);
		}
	}

	assert(false);
	return false;
}

void Train3DMASCDialog::sortByFeatureImportance()
{
	tableWidget->sortByColumn(FeatureImportanceColumn, Qt::DescendingOrder);
}

void Train3DMASCDialog::setFeatureImportance(QString featureName, float importance)
{
	for (int index = 0; index < tableWidget->rowCount(); ++index)
	{
		if (tableWidget->item(index, 0)->text() == featureName)
		{
			QTableWidgetItem* item = tableWidget->item(index, FeatureImportanceColumn);
			item->setText(isnan(importance) ? QString() : QString::number(importance, 'f', 6));
			return;
		}
	}

	assert(false);
}

void Train3DMASCDialog::onClose()
{
	if (!classifierSaved && QMessageBox::question(this, "Classifier not saved", "Classifier not saved. Do you confirm you want to close the tool?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
		return;

	reject();
}

void Train3DMASCDialog::onSave()
{
	saveRequested = true;
	accept();
}

void Train3DMASCDialog::onExportResults()
{
	QSettings settings;
	settings.beginGroup("3DMASC");
	QString outputPath = settings.value("FilePath", QCoreApplication::applicationDirPath()).toString();
	QString outputFilename = QFileDialog::getSaveFileName(this, "Export feature importance matrix", outputPath, "*.csv");
	if (outputFilename.isNull())
	{
		//process cancelled by the user
		return;
	}
	settings.setValue("FilePath", QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//save the file
	QFile file(outputFilename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		QMessageBox::critical(this, "Error", "Failed to open file for writing: " + outputFilename);
		return;
	}

	QTextStream stream(&file);
	stream << "Feature;Importance" << Qt::endl;
	for (int index = 0; index < tableWidget->rowCount(); ++index)
	{
		QString featureName = tableWidget->item(index, 0)->text();
		QString importance = tableWidget->item(index, FeatureImportanceColumn)->text();
		stream << featureName << ";" << importance << Qt::endl;
	}
}

void Train3DMASCDialog::deleteLaterConfusionMatrix(std::unique_ptr<ConfusionMatrix>& ptr)
{
	m_confusionMatrixToDeleteLater.push_back(std::move(ptr));
}

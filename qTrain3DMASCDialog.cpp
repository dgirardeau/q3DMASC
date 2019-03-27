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

//System
#include <assert.h>

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

	QTableWidgetItem* importanceItem = new QTableWidgetItem(std::isnan(importance) ? QString() : QString::number(importance));
	tableWidget->setItem(index, 1, importanceItem);

	return index;
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
	tableWidget->sortByColumn(FeatureImportanceColumn, Qt::AscendingOrder);
}

void Train3DMASCDialog::setFeatureImportance(QString featureName, float importance)
{
	for (int index = 0; index < tableWidget->rowCount(); ++index)
	{
		if (tableWidget->item(index, 0)->text() == featureName)
		{
			QTableWidgetItem* item = tableWidget->item(static_cast<int>(index), FeatureImportanceColumn);
			item->setText(std::isnan(importance) ? QString() : QString::number(importance, 'f', 6));
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

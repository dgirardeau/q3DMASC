#pragma once

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

//Qt
#include <QDialog>

#include <ui_Train3DMASCDialog.h>

//! 3DMASC plugin 'train' dialog
class Train3DMASCDialog : public QDialog, public Ui::Train3DMASCDialog
{
	Q_OBJECT

public:

	//! Default constructor
	Train3DMASCDialog(QWidget* parent = nullptr);

	void clearResults();

	//! Adds a feature (entry) to the results table
	/** \return the row index
	**/
	int addFeature(QString name, float importance, bool isChecked = true);
	int addScale(double scale, bool isChecked = true);
	void setResultText(QString text);
	void setFirstRunDone();
	inline void setClassifierSaved() { classifierSaved = true; saveRequested = false; }

	bool isFeatureSelected(QString featureName) const;
	void setFeatureImportance(QString featureName, float importance);
	void sortByFeatureImportance();
	
	inline bool shouldSaveClassifier() const { return saveRequested; }

protected slots:

	void onClose();
	void onSave();
	void onExportResults();

protected: //members

	bool classifierSaved;
	bool saveRequested;
};

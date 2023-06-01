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

#include <ui_Classify3DMASCDialog.h>

class ccMainAppInterface;
class ccPointCloud;

//! 3DMASC plugin 'classify' dialog
class Classify3DMASCDialog : public QDialog, public Ui::Classify3DMASCDialog
{
	Q_OBJECT

public:

	//! Default constructor
	Classify3DMASCDialog(ccMainAppInterface* app, bool trainMode = false);
	~Classify3DMASCDialog();

	//! read settings
	void readSettings();
	//! write settings
	void writeSettings();

	//! Sets the clouds roles
	void setCloudRoles(const QList<QString>& roles, QString corePointsLabel);

	//! Returns the selected point clouds
	void getClouds(QMap<QString, ccPointCloud*>& clouds) const;

protected slots:

	void onCloudChanged(int);

protected:

	//! Gives access to the application (data-base, UI, etc.)
	ccMainAppInterface* m_app;

};

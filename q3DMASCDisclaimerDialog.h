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

#ifndef Q3DMASC_DISCLAIMER_DIALOG_HEADER
#define Q3DMASC_DISCLAIMER_DIALOG_HEADER

#include <ui_trainDisclaimerDlg.h>
#include <ui_classifyDisclaimerDlg.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QMainwindow>

//! Dialog for displaying the 3DSMAC/UEB disclaimer
class TrainDisclaimerDialog : public QDialog, public Ui::TrainDisclaimerDialog
{
public:
	//! Default constructor
	TrainDisclaimerDialog(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::TrainDisclaimerDialog()
	{
		setupUi(this);
	}
};

//whether disclaimer has already been displayed (and accepted) or not
static bool s_trainDisclaimerAccepted = false;

static bool ShowTrainDisclaimer(ccMainAppInterface* app)
{
	if (!s_trainDisclaimerAccepted)
	{
		//if the user "cancels" it, then he refuses the diclaimer!
		s_trainDisclaimerAccepted = TrainDisclaimerDialog(app ? app->getMainWindow() : 0).exec();
	}
	
	return s_trainDisclaimerAccepted;
}

//! Dialog for displaying the M3C2/UEB disclaimer
class ClassifyDisclaimerDialog : public QDialog, public Ui::ClassifyDisclaimerDialog
{
public:
	//! Default constructor
	ClassifyDisclaimerDialog(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::ClassifyDisclaimerDialog()
	{
		setupUi(this);
	}
};

//whether disclaimer has already been displayed (and accepted) or not
static bool s_classifyDisclaimerAccepted = false;

static bool ShowClassifyDisclaimer(ccMainAppInterface* app)
{
	if (!s_classifyDisclaimerAccepted)
	{
		//if the user "cancels" it, then he refuses the diclaimer!
		s_classifyDisclaimerAccepted = ClassifyDisclaimerDialog(app ? app->getMainWindow() : 0).exec();
	}
	
	return s_classifyDisclaimerAccepted;
}

#endif //Q3DMASC_DISCLAIMER_DIALOG_HEADER

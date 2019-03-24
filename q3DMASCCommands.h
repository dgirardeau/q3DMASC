#pragma once

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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

//CloudCompare
#include "../../ccCommandLineInterface.h"

//Local
#include "q3DMASCTools.h"

//qCC_db
#include <ccProgressDialog.h>

//Qt
#include <QDialog>

static const char COMMAND_3DMASC_CLASSIFY[] = "3DMASC_CLASSIFY";
static const char COMMAND_3DMASC_KEEP_ATTRIBS[] = "KEEP_ATTRIBUTES";

struct Command3DMASCClassif : public ccCommandLineInterface::Command
{
	Command3DMASCClassif() : ccCommandLineInterface::Command("3DMASC Classify", COMMAND_3DMASC_CLASSIFY) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[3DMASC]");
		if (cmd.arguments().size() < 2)
		{
			return cmd.error(QString("Missing parameter(s): classifier filename (.txt) and cloud roles after \"-%1\"").arg(COMMAND_3DMASC_CLASSIFY));
		}

		QString argument = cmd.arguments().front();

		bool keepAttributes = false;
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_3DMASC_KEEP_ATTRIBS))
		{
			keepAttributes = true;

			//local option confirmed, we can move on
			cmd.arguments().pop_front();
		}

		if (cmd.arguments().size() < 2)
		{
			return cmd.error(QString("Missing parameter(s): classifier filename (.txt) and cloud roles after \"-%1\"").arg(COMMAND_3DMASC_CLASSIFY));
		}

		QString classifierFilename = cmd.arguments().front();
		cmd.print("Classifier filename: " + classifierFilename);
		QCoreApplication::processEvents();
		cmd.arguments().pop_front();

		QString cloudRolesStr = cmd.arguments().front();
		cmd.print("Cloud roles: " + cloudRolesStr);
		QCoreApplication::processEvents();
		cmd.arguments().pop_front();

		//process the cloud roles description
		QStringList tokens = cloudRolesStr.split(QRegExp("\\s+"), QString::SkipEmptyParts);

		masc::Tools::NamedClouds cloudPerRole;
		QString mainCloudRole;
		for (const QString& token : tokens)
		{
			QStringList subTokens = token.split("=");
			int subTokenCount = subTokens.size();
			if (subTokenCount != 2)
			{
				return cmd.error("Malformed cloud roles description (expecting: \"PC1=1 PC2=3 CTX=2\" for instance)");
			}
			QString role = subTokens[0].toUpper();
			bool ok = false;
			unsigned cloudIndex = subTokens[1].toUInt(&ok);
			if (!ok || cloudIndex == 0)
			{
				return cmd.error("Malformed cloud roles description (expecting the cloud index corresponding to each role - starting from 1)");
			}
			if (cloudIndex > cmd.clouds().size())
			{
				return cmd.error(QString("Cloud index %1 exceeds the number of loaded clouds (=%2)").arg(cloudIndex).arg(cmd.clouds().size()));
			}
			cloudPerRole.insert(role, cmd.clouds()[cloudIndex-1].pc);

			if (mainCloudRole.isEmpty())
			{
				mainCloudRole = role;
				cmd.print("The classified cloud role will be " + role);
			}
		}

		//try to load the clouds roles from the classifier file
		QSet<QString> cloudLabels;
		if (!masc::Tools::LoadClassifierCloudLabels(classifierFilename, cloudLabels))
		{
			return cmd.error("Failed to read classifier file");
		}

		for (QString label : cloudLabels)
		{
			if (!cloudPerRole.contains(label.toUpper()))
			{
				return cmd.error(QString("Role %1 has not been defined").arg(label));
			}
		}

		masc::Feature::Set features;
		masc::Classifier classifier;
		if (!masc::Tools::LoadClassifier(classifierFilename, cloudPerRole, features, classifier, cmd.widgetParent()))
		{
			return cmd.error("Failed to load the classifier");
		}

		//internal consistency check
		if (!cloudPerRole.contains(mainCloudRole))
		{
			return cmd.error("Classified cloud not loaded/defined?!");
		}

		//remove the test cloud (if any)
		if (cloudPerRole.contains("TEST"))
		{
			delete cloudPerRole["TEST"];
			cloudPerRole.remove("TEST");
		}

		//the 'main cloud' is the cloud that should be classified
		masc::CorePoints corePoints;
		corePoints.origin = corePoints.cloud = cloudPerRole[mainCloudRole];

		//prepare the main cloud
		QScopedPointer<ccProgressDialog> pDlg;
		if (!cmd.silentMode())
		{
			pDlg.reset(new ccProgressDialog(true, cmd.widgetParent()));
			pDlg->setAutoClose(false); //we don't want the progress dialog to 'pop' for each feature
		}
		
		QString errorMessage;
		SFCollector generatedScalarFields;
		if (!masc::Tools::PrepareFeatures(corePoints, features, errorMessage, pDlg.data(), &generatedScalarFields))
		{
			generatedScalarFields.releaseAllSFs();
			return cmd.error(errorMessage);
		}

		if (pDlg)
		{
			pDlg->setAutoClose(true); //restore the default behavior of the progress dialog
			pDlg->close();
			QCoreApplication::processEvents();
		}

		//apply classifier
		{
			if (!classifier.classify(features, corePoints.cloud, errorMessage, cmd.widgetParent()))
			{
				generatedScalarFields.releaseAllSFs();
				return cmd.error(errorMessage);
			}

			if (!keepAttributes)
			{
				generatedScalarFields.releaseAllSFs();
			}
		}

		if (cmd.autoSaveMode())
		{
			for (CLCloudDesc& desc : cmd.clouds())
			{
				if (desc.pc == corePoints.origin)
				{
					QString errorStr = cmd.exportEntity(desc, "CLASSIFIED");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
					break;
				}
			}
		}

		return true;
	}
};

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

#ifndef Q_3DMASC_PLUGIN_HEADER
#define Q_3DMASC_PLUGIN_HEADER

//qCC
#include <ccStdPluginInterface.h>

//qCC_db
#include <ccHObject.h>

//! 3DMASC plugin
/** 3D Multi-cloud, multi-Attribute, multi-Scale, multi-Class classification
**/
class q3DMASCPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.q3DMASC" FILE "info.json")

public:

	//! Default constructor
	q3DMASCPlugin(QObject* parent = nullptr);

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction*> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

protected slots:

	void doClassifyAction();
	void doTrainAction();

protected:

	//! Calssift action
	QAction* m_classifyAction;
	//! Train action
	QAction* m_trainAction;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //Q_3DMASC_PLUGIN_HEADER

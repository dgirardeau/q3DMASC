//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCanupo2                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                          COPYRIGHT: OSUR                               #
//#                                                                        #
//##########################################################################

#ifndef QCANUPO2_PLUGIN_HEADER
#define QCANUPO2_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"

//! New classification plugin (Dimitri Lague / OSUR)
class qCanupo2Plugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	//replace qDummy by the plugin name (IID should be unique - let's hope your plugin name is unique ;)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qCanupo2")

public:

	//! Default constructor
	explicit qCanupo2Plugin(QObject* parent = 0);

	//inherited from ccPluginInterface
	virtual QString getName() const override { return "CANUPO V2"; }
	virtual QString getDescription() const override { return "CANUPO V2 Classification (D. Lague, A. le Guennec)"; }
	virtual QIcon getIcon() const override;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual void getActions(QActionGroup& group) override;

protected slots:

	void doAction();

protected:

	//! Default action
	QAction* m_action;
};

#endif //QCANUPO2_PLUGIN_HEADER


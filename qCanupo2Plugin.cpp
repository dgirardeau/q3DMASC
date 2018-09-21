//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qDummy                      #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#include "qCanupo2Plugin.h"

//Qt
#include <QtGui>

qCanupo2Plugin::qCanupo2Plugin(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qCanupo2Plugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	//if (m_action)
	//	m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction*> qCanupo2Plugin::getActions()
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	return QList<QAction*>{m_action};
}

void qCanupo2Plugin::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	m_app->dispToConsole("[qCanupo2Plugin] Hello world!",ccMainAppInterface::STD_CONSOLE_MESSAGE); //a standard message is displayed in the console
	m_app->dispToConsole("[qCanupo2Plugin] Warning: dummy plugin shouldn't be used as is!",ccMainAppInterface::WRN_CONSOLE_MESSAGE); //a warning message is displayed in the console
	m_app->dispToConsole("Dummy plugin shouldn't be used as is!",ccMainAppInterface::ERR_CONSOLE_MESSAGE); //an error message is displayed in the console AND an error box will pop-up!
}

QIcon qCanupo2Plugin::getIcon() const
{
	return QIcon(":/CC/plugin/qCanupo2/icon.png");
}

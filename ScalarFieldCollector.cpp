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

#include "ScalarFieldCollector.h"

//qCC_db
#include <ccPointCloud.h>

//CCLib
#include <ScalarField.h>

//system
#include <assert.h>

void SFCollector::push(ccPointCloud* cloud, CCCoreLib::ScalarField* sf, Behavior behavior)
{
	assert(!scalarFields.contains(sf));
	SFDesc desc;
	desc.behavior = behavior;
	desc.cloud = cloud;
	scalarFields[sf] = desc;
}

void SFCollector::releaseSFs(bool keepByDefault)
{
	for (Map::iterator it = scalarFields.begin(); it != scalarFields.end(); ++it)
	{
		const SFDesc& desc = it.value();
		CCCoreLib::ScalarField* sf = it.key();

		if (desc.behavior == ALWAYS_KEEP || (keepByDefault && desc.behavior == CAN_REMOVE))
		{
			ccLog::Warning(QString("[SFCollector] Keep scalar field '%1'").arg(sf->getName()));
			//keep this SF
			continue;
		}
		
		int sfIdx = desc.cloud->getScalarFieldIndexByName(sf->getName());
		if (sfIdx >= 0)
		{
			ccLog::Warning(QString("[SFCollector] Remove scalar field '%1'").arg(sf->getName()));
			desc.cloud->deleteScalarField(sfIdx);
		}
		else
		{
			ccLog::Warning(QString("[SFCollector] Scalar field '%1' can't be found anymore, impossible to remove it").arg(sf->getName()));
		}
	}

	scalarFields.clear();
}

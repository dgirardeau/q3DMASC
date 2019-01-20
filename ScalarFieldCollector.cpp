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

void SFCollector::push(ccPointCloud* cloud, CCLib::ScalarField* sf)
{
	(*this)[cloud].insert(sf);
}

void SFCollector::releaseAllSFs()
{
	for (QMap< ccPointCloud*, std::set<CCLib::ScalarField*> >::iterator it = begin(); it != end(); ++it)
	{
		ccPointCloud* cloud = it.key();
		std::set<CCLib::ScalarField*>& sfs = it.value();

		for (CCLib::ScalarField* sf : sfs)
		{
			int sfIdx = cloud->getScalarFieldIndexByName(sf->getName());
			if (sfIdx >= 0)
			{
				cloud->deleteScalarField(sfIdx);
			}
			else
			{
				ccLog::Warning(QString("[SFCollector] Scalar field '%1' can't be found anymore").arg(sf->getName()));
			}
		}

		sfs.clear();
	}

	clear();
}

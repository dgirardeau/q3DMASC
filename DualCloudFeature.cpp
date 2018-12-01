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

#include "DualCloudFeature.h"

using namespace masc;

bool DualCloudFeature::prepare(	const CorePoints& corePoints,
								QString& error,
								CCLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
	//TODO
	return false;
}

bool DualCloudFeature::checkValidity(QString &error) const
{
	if (!Feature::checkValidity(error))
	{
		return false;
	}

	unsigned char cloudCount = (cloud1 ? (cloud2 ? 2 : 1) : 0);
	if (cloudCount < 2)
	{
		error = "at least two clouds are required to compute context-based features";
		return false;
	}

	if (op != NO_OPERATION)
	{
		error = "math operations can't be defined on dual-cloud features";
		return false;
	}

	return true;
}
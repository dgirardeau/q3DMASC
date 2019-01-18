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
#include <QMap>

//system
#include <set>

class ccPointCloud;

namespace CCLib
{
	class ScalarField;
};

//! SF collector
/** For tracking the creation and removing a set of scalar fields
**/
class SFCollector : QMap< ccPointCloud*, std::set<CCLib::ScalarField*> >
{
	public:

		void push(ccPointCloud* cloud, CCLib::ScalarField* sf);

		void clear();
};

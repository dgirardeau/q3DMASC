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
class SFCollector
{
	public:

		enum Behavior { ALWAYS_KEEP, CAN_REMOVE, ALWAYS_REMOVE };

		void push(ccPointCloud* cloud, CCLib::ScalarField* sf, Behavior behavior);

		void releaseSFs(bool keepByDefault);

		struct SFDesc
		{
			ccPointCloud* cloud = nullptr;
			CCLib::ScalarField* sf = nullptr;
			Behavior behavior = CAN_REMOVE;
		};

		using Map = QMap< CCLib::ScalarField*, SFDesc >;
		Map scalarFields;
};

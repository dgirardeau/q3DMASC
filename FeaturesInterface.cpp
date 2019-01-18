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

#include "FeaturesInterface.h"

//qCC_db
#include <ccScalarField.h>

//system
#include <assert.h>

using namespace masc;

CCLib::ScalarField* Feature::PrepareSF(ccPointCloud* cloud, const char* resultSFName, SFCollector* generatedScalarFields/*=nullptr*/)
{
	if (!cloud || !resultSFName)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	CCLib::ScalarField* resultSF = nullptr;
	int sfIdx = cloud->getScalarFieldIndexByName(resultSFName);
	if (sfIdx >= 0)
	{
		resultSF = cloud->getScalarField(sfIdx);
	}
	else
	{
		ccScalarField* newSF = new ccScalarField(resultSFName);
		if (!newSF->resizeSafe(cloud->size()))
		{
			ccLog::Warning("Not enough memory");
			newSF->release();
			return nullptr;
		}
		cloud->addScalarField(newSF);
		
		if (generatedScalarFields)
		{
			//track the generated scalar-field
			generatedScalarFields->push(cloud, newSF);
		}

		resultSF = newSF;

	}

	assert(resultSF);
	resultSF->fill(NAN_VALUE);

	return resultSF;
}

bool Feature::PerformMathOp(CCLib::ScalarField* sf1, const CCLib::ScalarField* sf2, Feature::Operation op)
{
	if (!sf1 || !sf2 || sf1->size() != sf2->size() || op == Feature::NO_OPERATION)
	{
		//invalid input parameters
		return false;
	}

	for (unsigned i = 0; i < sf1->size(); ++i)
	{
		ScalarType s1 = sf1->getValue(i);
		ScalarType s2 = sf2->getValue(i);
		ScalarType s = NAN_VALUE;
		switch (op)
		{
		case Feature::MINUS:
			s = s1 - s2;
			break;
		case Feature::PLUS:
			s = s1 + s2;
			break;
		case Feature::DIVIDE:
			if (std::abs(s2) > std::numeric_limits<ScalarType>::epsilon())
				s = s1 / s2;
			break;
		case Feature::MULTIPLY:
			s = s1 * s2;
			break;
		default:
			assert(false);
			break;
		}
		sf1->setValue(i, s);
	}
	sf1->computeMinAndMax();

	return true;
}

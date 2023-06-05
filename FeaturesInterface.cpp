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

bool Feature::CheckSFExistence(ccPointCloud* cloud, const char* resultSFName)
{
	if (!cloud || !resultSFName)
	{
		assert(false);
		return false;
	}

	int sfIdx = cloud->getScalarFieldIndexByName(resultSFName);
	return (sfIdx >= 0);
}

CCCoreLib::ScalarField* Feature::PrepareSF(ccPointCloud* cloud, const char* resultSFName, SFCollector* generatedScalarFields/*=nullptr*/, SFCollector::Behavior behavior/*=SFCollector::CAN_REMOVE*/)
{
	if (!cloud || !resultSFName)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	CCCoreLib::ScalarField* resultSF = nullptr;
	int sfIdx = cloud->getScalarFieldIndexByName(resultSFName);
	if (sfIdx >= 0)
	{
//		ccLog::Warning("Existing SF: " + QString(resultSFName) + ", do not store in generatedScalarFields");
		resultSF = cloud->getScalarField(sfIdx);
	}
	else
	{
//		ccLog::Warning("SF does not exist, create it: " + QString(resultSFName)  + ", SFCollector::Behavior " + QString::number(behavior));
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
			generatedScalarFields->push(cloud, newSF, behavior);
		}

		resultSF = newSF;
		resultSF->fill(CCCoreLib::NAN_VALUE);
	}

	assert(resultSF);

	return resultSF;
}

ScalarType Feature::PerformMathOp(double s1, double s2, Operation op)
{
	ScalarType s = CCCoreLib::NAN_VALUE;
	switch (op)
	{
	case Feature::MINUS:
		s = static_cast<ScalarType>(s1 - s2);
		break;
	case Feature::PLUS:
		s = static_cast<ScalarType>(s1 + s2);
		break;
	case Feature::DIVIDE:
		if (std::abs(s2) > std::numeric_limits<ScalarType>::epsilon())
			s = static_cast<ScalarType>(s1 / s2);
		break;
	case Feature::MULTIPLY:
		s = static_cast<ScalarType>(s1 * s2);
		break;
	default:
		assert(false);
		break;
	}
	return s;
}

bool Feature::PerformMathOp(CCCoreLib::ScalarField* sf1, const CCCoreLib::ScalarField* sf2, Feature::Operation op)
{
	if (!sf1 || !sf2 || sf1->size() != sf2->size() || op == Feature::NO_OPERATION)
	{
		//invalid input parameters
		assert(false);
		return false;
	}

	for (unsigned i = 0; i < sf1->size(); ++i)
	{
		ScalarType s1 = sf1->getValue(i);
		ScalarType s2 = sf2->getValue(i);
		ScalarType s = PerformMathOp(s1, s2, op);
		sf1->setValue(i, s);
	}
	sf1->computeMinAndMax();

	return true;
}

bool Feature::PerformMathOp(const IScalarFieldWrapper& sf1, const IScalarFieldWrapper& sf2, Operation op, CCCoreLib::ScalarField* outSF)
{
	if (!outSF || sf1.size() != sf2.size() || sf1.size() != outSF->size() || op == Feature::NO_OPERATION)
	{
		//invalid input parameters
		assert(false);
		return false;
	}

	for (unsigned i = 0; i < sf1.size(); ++i)
	{
		double s1 = sf1.pointValue(i);
		double s2 = sf2.pointValue(i);
		ScalarType s = PerformMathOp(s1, s2, op);
		outSF->setValue(i, s);
	}
	outSF->computeMinAndMax();

	return true;
}

bool Feature::SaveSources(const Source::Set& sources, QString filename)
{
	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		ccLog::Warning("Failed to open file for writing: " + filename);
		return false;
	}
	
	QTextStream stream(&file);
	stream << "#Features_SF" << endl;
	for (const Source& s : sources)
	{
		stream << s.type << ":" << s.name << endl;
	}

	return true;
}

bool Feature::LoadSources(Source::Set& sources, QString filename)
{
	QFile file(filename);
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		ccLog::Warning("Failed to open file for reading: " + filename);
		return false;
	}

	QTextStream stream(&file);
	QString header = stream.readLine();
	if (!header.startsWith("#Features_SF"))
	{
		ccLog::Warning("Unexpected header");
		return false;
	}

	while (true)
	{
		QString line = stream.readLine();
		if (line.isNull())
			break;
		if (line.isEmpty())
			continue; //unexpected but we can survive
		QStringList tokens = line.split(':');
		if (tokens.size() != 2)
		{
			ccLog::Warning("Malformed file");
			return false;
		}

		Source src;
		bool ok = false;
		int sourceType = tokens[0].toInt(&ok);
		if (!ok || sourceType < Feature::Source::ScalarField || sourceType > Feature::Source::Blue)
		{
			ccLog::Warning("Unhandled source type");
			return false;
		}
		src.type = static_cast<Feature::Source::Type>(sourceType);
		src.name = tokens[1];
		
		sources.push_back(src);
	}

	return true;
}

bool Feature::ExtractSources(const Set& features, Source::Set& sources)
{
	sources.clear();
	try
	{
		sources.reserve(features.size());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return false;
	}

	for (Feature::Shared f : features)
	{
		sources.push_back(f->source);
	}

	return true;
}

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

#include "q3DMASCTools.h"

//qCC_io
#include <FileIOFilter.h>

//Qt
#include <QTextStream>
#include <QFile>

//system
#include <assert.h>

using namespace masc;

bool Tools::LoadFile(QString filename, ccPointCloud* pc1, ccPointCloud* pc2, FeatureRule::Set& features)
{
	QFile file(filename);
	if (!file.exists())
	{
		ccLog::Warning(QString("Can't find file '%1'").arg(filename));
		return false;
	}
	if (!file.open(QFile::Text | QFile::ReadOnly))
	{
		ccLog::Warning(QString("Can't open file '%1'").arg(filename));
		return false;
	}

	Scales::Shared scales(new Scales);
	assert(features.empty());

	QMap<QString, QScopedPointer<ccPointCloud> > clouds;

	QTextStream stream(&file);
	while (true)
	{
		QString line = stream.readLine();
		if (line.isNull())
		{
			//eof
			break;
		}

		if (line.startsWith("#"))
		{
			//comment
			continue;
		}

		//strip out the potential comment at the end of the mine as well
		int commentIndex = line.indexOf('#');
		if (commentIndex >= 0)
			line = line.left(commentIndex);

		QString upperLine = line.toUpper();
		if (upperLine.startsWith("CLOUD:")) //clouds
		{
			QString command = line.mid(6);
			QStringList tokens = command.split(':');
			if (tokens.size() != 2)
			{
				ccLog::Warning("Malformed file: expecting 2 tokens after 'cloud:'");
				return false;
			}
			QString pcName = tokens[0];
			QString pcFilename = tokens[1];
			//try to open the cloud
			{
				FileIOFilter::LoadParameters parameters;
				parameters.alwaysDisplayLoadDialog = false;
				CC_FILE_ERROR error = CC_FERR_NO_ERROR;
				ccHObject* object = FileIOFilter::LoadFromFile(pcFilename, parameters, error);
				if (error != CC_FERR_NO_ERROR || !object)
				{
					ccLog::Warning("Failed to open the file (see console)");
					if (object)
						delete object;
					return false;
				}
				if (!object->isA(CC_TYPES::POINT_CLOUD))
				{
					ccLog::Warning("File doesn't contain a single cloud");
					delete object;
					return false;
				}
				clouds.insert(pcName, QScopedPointer<ccPointCloud>(static_cast<ccPointCloud*>(object)));
			}
		}
		else if (upperLine.startsWith("SCALES:")) //scales
		{
			QString command = line.mid(7);
			QStringList tokens = command.split(';');
			if (tokens.empty())
			{
				ccLog::Warning("Malformed file: expecting at least one token after 'scales:'");
				return false;
			}

			try
			{
				for (const QString& token : tokens)
				{
					if (token.contains(':'))
					{
						//it's probably a range
						QStringList subTokens = token.split(':');
						if (subTokens.size() != 3)
						{
							ccLog::Warning(QString("Malformed file: expecting 3 tokens for a range of scales (%1)").arg(token));
							return false;
						}
						bool ok[3] = { true, true, true };
						double start = subTokens[0].toDouble(ok);
						double step = subTokens[1].toDouble(ok + 1);
						double stop = subTokens[2].toDouble(ok + 2);
						if (!ok[0] || !ok[1] || !ok[2])
						{
							ccLog::Warning(QString("Malformed file: invalid values in scales range (%1)").arg(token));
							return false;
						}
						if (stop < start || step <= 1.0-6)
						{
							ccLog::Warning(QString("Malformed file: invalid range (%1)").arg(token));
							return false;
						}
						for (double v = start; v <= stop + 1.0 - 6; v += step)
						{
							scales->values.push_back(v);
						}
					}
					else
					{
						bool ok = true;
						double v = token.toDouble(&ok);
						if (!ok)
						{
							ccLog::Warning(QString("Malformed file: invalid scale value (%1)").arg(token));
							return false;
						}
						scales->values.push_back(v);
					}
				}
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("Not enough memory");
				return false;
			}
		}
		else if (upperLine.startsWith("FEATURE:")) //feature
		{
			QString command = line.mid(8);
			QStringList tokens = command.split('_');
			if (tokens.empty())
			{
				ccLog::Warning("Malformed file: expecting at least one token after 'feature:'");
				return false;
			}

			QString typeStr = tokens[0].toUpper();
			//TODO
			
		}
		else
		{
			ccLog::Warning("Unrecognized token/line: " + (line.length() < 10 ? line : line.left(10) + "..."));
			return false;
		}
	}

	return true;
}

bool Tools::RandomSubset(ccPointCloud* cloud, float ratio, CCLib::ReferenceCloud* inRatioSubset, CCLib::ReferenceCloud* outRatioSubset)
{
	if (!cloud)
	{
		ccLog::Warning("Invalid input cloud");
		return false;
	}
	if (!inRatioSubset || !outRatioSubset)
	{
		ccLog::Warning("Invalid input refence clouds");
		return false;
	}
	if (inRatioSubset->getAssociatedCloud() != cloud || outRatioSubset->getAssociatedCloud() != cloud)
	{
		ccLog::Warning("Invalid input reference clouds (associated cloud is wrong)");
		return false;
	}
	if (ratio < 0.0f || ratio > 1.0f)
	{
		ccLog::Warning(QString("Invalid parameter (ratio: %1)").arg(ratio));
		return false;
	}

	unsigned inSampleCount = static_cast<unsigned>(floor(cloud->size() * ratio));
	assert(inSampleCount <= cloud->size());
	unsigned outSampleCount = cloud->size() - inSampleCount;

	//we draw the smallest population (faster)
	unsigned targetCount = inSampleCount;
	bool defaultState = true;
	if (outSampleCount < inSampleCount)
	{
		targetCount = outSampleCount;
		defaultState = false;
	}

	//reserve memory
	std::vector<bool> pointInsideRatio;
	try
	{
		pointInsideRatio.resize(cloud->size(), defaultState);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return false;
	}

	if (!inRatioSubset->reserve(inSampleCount) || !outRatioSubset->reserve(outSampleCount))
	{
		ccLog::Warning("Not enough memory");
		inRatioSubset->clear();
		outRatioSubset->clear();
		return false;
	}

	//randomly choose the 'in' or 'out' indexes
	int randIndex = 0;
	unsigned randomCount = 0;
	while (randomCount < targetCount)
	{
		randIndex = ((randIndex + std::rand()) % cloud->size());
		if (pointInsideRatio[randIndex] == defaultState)
		{
			pointInsideRatio[randIndex] = !defaultState;
			++randomCount;
		}
	}

	//now dispatch the points
	{
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			if (pointInsideRatio[i])
				inRatioSubset->addPointIndex(i);
			else
				outRatioSubset->addPointIndex(i);
		}
		assert(inRatioSubset->size() == inSampleCount);
		assert(outRatioSubset->size() == outSampleCount);
	}

	return true;
}


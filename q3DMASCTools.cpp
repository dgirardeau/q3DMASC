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

//Local
#include "ScalarFieldWrappers.h"

//qCC_io
#include <FileIOFilter.h>
#include <LASFields.h>
//qCC_db
#include <ccScalarField.h>

//CCLib
#include <GenericProgressCallback.h>

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

	QMap<QString, QSharedPointer<ccPointCloud> > clouds;

	QTextStream stream(&file);
	
	for (int lineNumber = 0; ; ++lineNumber)
	{
		QString line = stream.readLine();
		if (line.isNull())
		{
			//eof
			break;
		}
		++lineNumber;

		if (line.startsWith("#"))
		{
			//comment
			continue;
		}

		//strip out the potential comment at the end of the line as well
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
				ccLog::Warning("Malformed file: expecting 2 tokens after 'cloud:' on line #" + QString::number(lineNumber));
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
				clouds.insert(pcName, QSharedPointer<ccPointCloud>(static_cast<ccPointCloud*>(object)));
			}
		}
		else if (upperLine.startsWith("SCALES:")) //scales
		{
			QString command = line.mid(7);
			QStringList tokens = command.split(';');
			if (tokens.empty())
			{
				ccLog::Warning("Malformed file: expecting at least one token after 'scales:' on line #" + QString::number(lineNumber));
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
							ccLog::Warning(QString("Malformed file: invalid values in scales range (%1) on line #%2").arg(token).arg(lineNumber));
							return false;
						}
						if (stop < start || step <= 1.0-6)
						{
							ccLog::Warning(QString("Malformed file: invalid range (%1) on line #%2").arg(token).arg(lineNumber));
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
							ccLog::Warning(QString("Malformed file: invalid scale value (%1) on line #%2").arg(token).arg(lineNumber));
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
				ccLog::Warning("Malformed file: expecting at least one token after 'feature:' on line #" + QString::number(lineNumber));
				return false;
			}

			FeatureRule::Shared rule(new FeatureRule);

			//read the type
			QString typeStr = tokens[0].toUpper();
			{
				for (int iteration = 0; iteration < 1; ++iteration) //fake loop for easy break
				{
					PointFeature::PointFeatureType pointFeatureType = PointFeature::FromUpperString(typeStr);
					if (pointFeatureType != PointFeature::Invalid)
					{
						//we have a point feature
						PointFeature::Shared pointFeature(new PointFeature(pointFeatureType));

						//specific case: 'SF#'
						if (pointFeatureType == PointFeature::SF)
						{
							QString sfIndexStr = typeStr.mid(2);
							bool ok = true;
							int sfIndex = sfIndexStr.toInt(&ok);
							if (!ok)
							{
								ccLog::Warning(QString("Malformed file: expecting a valid integer value after 'SF' on line #%1").arg(lineNumber));
								return false;
							}
							pointFeature->sourceSFIndex = sfIndex;
						}

						rule->feature = pointFeature;
						break;
					}
					NeighborhoodFeature::NeighborhoodFeatureType neighborhoodFeatureType = NeighborhoodFeature::FromUpperString(typeStr);
					if (neighborhoodFeatureType != NeighborhoodFeature::Invalid)
					{
						//we have a neighborhood feature
						rule->feature = NeighborhoodFeature::Shared(new NeighborhoodFeature(neighborhoodFeatureType));
						break;
					}
					ContextBasedFeature::ContextBasedFeatureType contextBasedFeatureType = ContextBasedFeature::FromUpperString(typeStr);
					if (contextBasedFeatureType != ContextBasedFeature::Invalid)
					{
						//we have a context-based feature
						rule->feature = ContextBasedFeature::Shared(new ContextBasedFeature(contextBasedFeatureType));
						break;
					}
					DualCloudFeature::DualCloudFeatureType dualCloudFeatureType = DualCloudFeature::FromUpperString(typeStr);
					if (dualCloudFeatureType != DualCloudFeature::Invalid)
					{
						//we have a dual cloud feature
						rule->feature = DualCloudFeature::Shared(new DualCloudFeature(dualCloudFeatureType));
						break;
					}

					ccLog::Warning(QString("Malformed file: unrecognized token '%1' after 'feature:' on line #%2").arg(typeStr).arg(lineNumber));
					return false;
				}
			}
			assert(rule->feature);

			//read the scales
			{
				QString scaleStr = tokens[1].toUpper();
				if (!scaleStr.startsWith("SC"))
				{
					ccLog::Warning(QString("Malformed file: unrecognized token '%1' (expecting the scale descriptor 'SC...' on line #%2").arg(typeStr).arg(lineNumber));
					return false;
				}

				if (scaleStr == "SC0")
				{
					//no scale
				}
				else if (scaleStr == "SCX")
				{
					//all scales
					rule->scales = scales;
				}
				else
				{
					//read the specific scale index
					QString scaleStr = scaleStr.mid(2);
					bool ok = true;
					double scale = scaleStr.toDouble(&ok);
					if (!ok)
					{
						ccLog::Warning(QString("Malformed file: expecting a valid number after 'SC:' on line #%1").arg(lineNumber));
						return false;
					}
					rule->scales = Scales::Shared(new Scales);
					rule->scales->values.resize(1);
					rule->scales->values.front() = scale;
				}
			}

			//process the next tokens (may not be ordered)
			int cloudCount = 0;
			bool statDefined = false;
			bool mathDefined = false;
			for (int i = 2; i < tokens.size(); ++i)
			{
				QString token = tokens[i].toUpper();

				//is the token a 'stat' one?
				if (!statDefined)
				{
					if (token == "MEAN")
					{
						rule->stat = FeatureRule::MEAN;
						statDefined = true;
					}
					else if (token == "MODE")
					{
						rule->stat = FeatureRule::MODE;
						statDefined = true;
					}
					else if (token == "STD")
					{
						rule->stat = FeatureRule::STD;
						statDefined = true;
					}
					else if (token == "RANGE")
					{
						rule->stat = FeatureRule::RANGE;
						statDefined = true;
					}
					else if (token == "SKEW")
					{
						rule->stat = FeatureRule::SKEW;
						statDefined = true;
					}

					if (statDefined)
					{
						continue;
					}
				}

				//is the token a cloud name?
				if (cloudCount < 2)
				{
					bool cloudNameMatches = false;
					for (QMap<QString, QSharedPointer<ccPointCloud> >::const_iterator it = clouds.begin(); it != clouds.end(); ++it)
					{
						if (it.key().toUpper() == token)
						{
							if (cloudCount == 0)
								rule->cloud1 = it.value().data();
							else
								rule->cloud2 = it.value().data();
							++cloudCount;
							cloudNameMatches = true;
							break;
						}
					}
					
					if (cloudNameMatches)
					{
						continue;
					}
				}

				//is the token a 'math' one?
				if (cloudCount == 2 && rule->feature->getType() != Feature::Type::DualCloudFeature && !mathDefined)
				{
					if (token == "MINUS")
					{
						rule->op = FeatureRule::MINUS;
						mathDefined = true;
					}
					else if (token == "PLUS")
					{
						rule->op = FeatureRule::PLUS;
						mathDefined = true;
					}
					else if (token == "DIVIDE")
					{
						rule->op = FeatureRule::DIVIDE;
						mathDefined = true;
					}
					else if (token == "MULTIPLY")
					{
						rule->op = FeatureRule::MULTIPLY;
						mathDefined = true;
					}

					if (mathDefined)
					{
						continue;
					}
				}

				//is the token a 'context' descriptor?
				if (rule->feature->getType() == Feature::Type::ContextBasedFeature && token.startsWith("CTX"))
				{
					//read the context label
					QString ctxLabelStr = token.mid(2);
					bool ok = true;
					int ctxLabel = ctxLabelStr.toInt(&ok);
					if (!ok)
					{
						ccLog::Warning(QString("Malformed file: expecting a valid integer value after 'CTX' on line #%1").arg(lineNumber));
						return false;
					}
					static_cast<ContextBasedFeature*>(rule->feature.data())->ctxClassLabel = ctxLabel;
					continue;
				}

				//if we are here, it means we couldn't find a correspondance for the current token
				ccLog::Warning(QString("Malformed file: unrecognized or unexpected token '%1' on line #%2").arg(token).arg(lineNumber));
				return false;
			}

			//now check the consistency of the rule
			assert(rule && rule->feature);

			QString errorMessage;
			bool ruleIsValid = rule->checkValidity(errorMessage);
			if (!ruleIsValid)
			{
				ccLog::Warning("Malformed feature: " + errorMessage + QString("(line %1)").arg(lineNumber));
				return false;
			}

			//otherwise save it
			features.push_back(rule);
		}
		else
		{
			ccLog::Warning(QString("Line #%1: unrecognized token/command: ").arg(lineNumber) + (line.length() < 10 ? line : line.left(10) + "..."));
			return false;
		}
	}

	return true;
}

static CCLib::ScalarField* RetrieveSF(const ccPointCloud* cloud, const QString& sfName, bool caseSensitive = true)
{
	if (!cloud)
	{
		assert(false);
		return nullptr;
	}
	int sfIdx = -1;
	if (caseSensitive)
	{
		sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
	}
	else
	{
		QString sfNameUpper = sfName.toUpper();
		for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
		{
			if (QString(cloud->getScalarField(i)->getName()).toUpper() == sfNameUpper)
			{
				sfIdx = static_cast<int>(i);
				break;
			}
		}
	}
	
	if (sfIdx >= 0)
	{
		return cloud->getScalarField(sfIdx);
	}
	else
	{
		return nullptr;
	}
}

static const char* s_echoRatioSFName = "EchoRat";
static const char* s_NIRSFName = "NIR";
static const char* s_M3C2SFName = "M3C2 distance";
static const char* s_PCVSFName = "Illuminance (PCV)";
static const char* s_normDipSFName = "Norm dip";
static const char* s_normDipDirSFName = "Norm dip dir.";

static CCLib::ScalarField* RetrieveOrComputeSF(PointFeature::PointFeatureType featureType, int sourceSFIndex, ccPointCloud* cloud, QString& error)
{
	QString sfName;
	switch (featureType)
	{
	case PointFeature::Intensity:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_INTENSITY], false);
		if (!sf)
		{
			error = "Cloud has no 'intensity' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::X:
	case PointFeature::Y:
	case PointFeature::Z:
		//not a ScalarField source
		error = "Internal error (source is not a scalar field)";
		return nullptr;
	case PointFeature::NbRet:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS], false);
		if (!sf)
		{
			error = "Cloud has no 'number of returns' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::RetNb:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_RETURN_NUMBER], false);
		if (!sf)
		{
			error = "Cloud has no 'return number' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::EchoRat:
	{
		CCLib::ScalarField* _echoRatioSF = RetrieveSF(cloud, s_echoRatioSFName, true);
		if (_echoRatioSF)
		{
			//SF was already computed?
			return _echoRatioSF;
		}
		//otherwise we need to compute it
		CCLib::ScalarField* numberOfRetSF = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS], false);
		if (!numberOfRetSF)
		{
			error = "Can't compute the 'echo ratio' field: no 'Number of Return' SF available";
			return nullptr;
		}
		CCLib::ScalarField* retNumberSF = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_RETURN_NUMBER], false);
		if (!retNumberSF)
		{
			error = "Can't compute the 'echo ratio' field: no 'Return number' SF available";
			return nullptr;
		}
		if (retNumberSF->size() != numberOfRetSF->size() || retNumberSF->size() != cloud->size())
		{
			error = "Internal error (inconsistent scalar fields)";
			return nullptr;
		}
		ccScalarField* echoRatioSF = new ccScalarField(s_echoRatioSFName);
		if (!echoRatioSF->reserveSafe(retNumberSF->size()))
		{
			error = "Not enough memory";
			echoRatioSF->release();
			return nullptr;
		}

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			ScalarType p = retNumberSF->getValue(i);
			ScalarType q = numberOfRetSF->getValue(i);
			ScalarType ratio = (std::abs(q) > std::numeric_limits<ScalarType>::epsilon() ? p / q : NAN_VALUE);
			echoRatioSF->addElement(ratio);
		}
		echoRatioSF->computeMinAndMax();
		cloud->addScalarField(echoRatioSF);
		return echoRatioSF;
	}
	case PointFeature::R:
	case PointFeature::G:
	case PointFeature::B:
		//not a ScalarField source
		error = "Internal error (source is not a scalar field)";
		return nullptr;
	case PointFeature::NIR:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_NIRSFName, false);
		if (!sf)
		{
			error = "Cloud has no 'NIR' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::DipAng:
	case PointFeature::DipDir:
	{
		CCLib::ScalarField* _dipSF = RetrieveSF(cloud, (featureType == PointFeature::DipAng ? s_normDipSFName : s_normDipDirSFName), true);
		if (_dipSF)
		{
			//SF was already computed?
			return _dipSF;
		}
		//otherwise we need to compute it

		static const char* s_normDipSFName = "Norm dip";
		static const char* s_normDipDirSFName = "Norm dip dir.";
		//we need normals to cumpute Dip and Dip Dir. angles!
		if (!cloud->hasNormals())
		{
			error = "Cloud has no normals: can't compute dip or dip dir. angles";
			return nullptr;
		}

		ccScalarField* dipSF = new ccScalarField(featureType == PointFeature::DipAng ? s_normDipSFName : s_normDipDirSFName);
		if (!dipSF->reserveSafe(cloud->size()))
		{
			error = "Not enough memory";
			dipSF->release();
			return nullptr;
		}

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			const CCVector3& N = cloud->getPointNormal(i);
			PointCoordinateType dip_deg, dipDir_deg;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip_deg, dipDir_deg);
			dipSF->addElement(static_cast<ScalarType>(featureType == PointFeature::DipAng ? dip_deg : dipDir_deg));
		}
		dipSF->computeMinAndMax();
		cloud->addScalarField(dipSF);
		return dipSF;
	}
	case PointFeature::M3C2:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_M3C2SFName, true);
		if (!sf)
		{
			error = "Cloud has no 'm3c2 distance' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::PCV:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_PCVSFName, true);
		if (!sf)
		{
			error = "Cloud has no 'PCV/Illuminance' scalar field";
			return nullptr;
		}
		return sf;
	}
	case PointFeature::SF:
		if (sourceSFIndex < 0 || sourceSFIndex >= static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			error = QString("Can't retrieve the specified SF: invalid index (%1)").arg(sourceSFIndex);
			return nullptr;
		}
		return cloud->getScalarField(sourceSFIndex);
	default:
		break;
	}

	error = "Unhandled feature type";
	return nullptr;
}

static bool ExtractStatFromSF(	const CCLib::DgmOctree::octreeCell& cell,
								void** additionalParameters,
								CCLib::NormalizedProgress* nProgress = nullptr)
{
	//additional parameters
	FeatureRule::Stat stat       = *reinterpret_cast<FeatureRule::Stat*>  (additionalParameters[0]);
	CCLib::ScalarField* inputSF  =  reinterpret_cast<CCLib::ScalarField*> (additionalParameters[1]);
	CCLib::ScalarField* resultSF =  reinterpret_cast<CCLib::ScalarField*> (additionalParameters[2]);
	PointCoordinateType radius   = *reinterpret_cast<PointCoordinateType*>(additionalParameters[3]);
	assert(inputSF && resultSF);

	//number of points inside the current cell
	unsigned n = cell.points->size();

	//spherical neighborhood extraction structure
	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius, cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know the points inside the current cell
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}
		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j = 0; j < n; ++j, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	for (unsigned i = 0; i < n; ++i)
	{
		//retrieve the points around the current cell point
		cell.points->getPoint(i, nNSS.queryPoint);

		//we extract the point's neighbors
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned kNN = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, true);
		if (kNN == 0)
		{
			assert(false);
			continue;
		}

		double sum = 0.0;
		double sum2 = 0.0;
		ScalarType minValue = 0;
		ScalarType maxValue = 0;
		bool withMode = (stat == FeatureRule::MODE || stat == FeatureRule::SKEW);
		QMap<ScalarType, unsigned> modeCounter;

		for (unsigned k = 0; k < kNN; ++k)
		{
			unsigned index = nNSS.pointsInNeighbourhood[k].pointIndex;
			ScalarType v = inputSF->getValue(index);

			//track min and max values
			if (k != 0)
			{
				if (v < minValue)
					minValue = v;
				else if (v > maxValue)
					maxValue = v;
			}
			else
			{
				minValue = maxValue = v;
			}

			//compute average and std. dev.
			sum += v;
			sum2 += static_cast<double>(v) * v;

			if (withMode)
			{
				if (modeCounter.contains(v))
				{
					++modeCounter[v];
				}
				else
				{
					modeCounter[v] = 1;
				}
			}
		}

		double mode = NAN_VALUE;
		if (withMode)
		{
			int maxCounter = 0;
			//look for the value with the highest frequency
			for (QMap<ScalarType, unsigned>::const_iterator it = modeCounter.begin(); it != modeCounter.end(); ++it)
			{
				if (it.value() > maxCounter)
				{
					maxCounter = it.value();
					mode = it.key();
				}
			}
		}

		ScalarType outValue = NAN_VALUE;
		switch (stat)
		{
			case FeatureRule::MEAN:
				outValue = static_cast<ScalarType>(sum / kNN);
				break;
			case FeatureRule::MODE:
				outValue = static_cast<ScalarType>(mode);
				break;
			case FeatureRule::STD:
				outValue = static_cast<ScalarType>(sqrt(std::abs(sum2 * kNN - sum * sum)) / kNN);
				break;
			case FeatureRule::RANGE:
				outValue = maxValue - minValue;
				break;
			case FeatureRule::SKEW:
			{
				double mean = sum / kNN;
				double std = sqrt(std::abs(sum2 / kNN - mean * mean));
				if (std > std::numeric_limits<float>::epsilon()) //arbitrary epsilon
				{
					outValue = static_cast<ScalarType>((mean - mode) / std);
				}
				break;
			}
			default:
				assert(false);
				break;
		}
		resultSF->setValue(cell.points->getPointGlobalIndex(i), outValue);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

static CCLib::ScalarField* ExtractStat(	ccPointCloud* cloud,
										CCLib::ScalarField* sf,
										double scale,
										FeatureRule::Stat stat,
										CCLib::GenericProgressCallback* progressCb = nullptr)
{
	if (!cloud || !sf || scale <= 0.0 || stat == FeatureRule::NO_STAT)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	ccOctree::Shared octree = cloud->getOctree();
	if (!octree)
	{
		octree = cloud->computeOctree(progressCb);
		if (!octree)
		{
			ccLog::Warning("Failed to compute octree");
			return nullptr;
		}
	}

	CCLib::ScalarField* resultSF = nullptr;
	QString resultSFName = sf->getName() + QString("_") + FeatureRule::StatToString(stat) + "_" + QString::number(scale);
	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(resultSFName));
	if (sfIdx >= 0)
	{
		resultSF = cloud->getScalarField(sfIdx);
	}
	else
	{
		resultSF = new ccScalarField(qPrintable(resultSFName));
		if (!resultSF->reserveSafe(cloud->size()))
		{
			ccLog::Warning("Not enough memory");
			resultSF->release();
			return nullptr;
		}
	}
	resultSF->fill(NAN_VALUE);

	PointCoordinateType radius = static_cast<PointCoordinateType>(scale / 2);
	unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius); //scale is the diameter!

	//additionnal parameters
	void* additionalParameters[] = {	static_cast<void*>(&stat),
										static_cast<void*>(&sf),
										static_cast<void*>(&resultSF),
										static_cast<void*>(&radius)
	};

	if (octree->executeFunctionForAllCellsAtLevel(	octreeLevel,
													ExtractStatFromSF,
													additionalParameters,
													true,
													progressCb,
													qPrintable(QString("Extract stat @ scale %1").arg(scale))) == 0)
	{
		//something went wrong
		ccLog::Warning("Process failed");
		resultSF->release();
		return nullptr;
	}

	resultSF->computeMinAndMax();
	cloud->addScalarField(static_cast<ccScalarField*>(resultSF));

	return resultSF;
}


static bool PreparePointBasedFeature(FeatureRule& rule, QString& error)
{
	assert(rule.feature && rule.feature->getType() == Feature::Type::PointFeature);

	PointFeature* feature = static_cast<PointFeature*>(rule.feature.data());

	std::vector<PointFeature::Shared> preparedFeatures;

	//look for the source field (and compute it if necessary)
	CCLib::ScalarField* sf1 = RetrieveOrComputeSF(feature->type, rule.sourceSFIndex, rule.cloud1, error);
	if (!sf1)
	{
		//error should be up to date
		return false;
	}

	CCLib::ScalarField* sf2 = nullptr;
	if (rule.cloud2 && rule.op != FeatureRule::NO_OPERATION)
	{
		sf2 = RetrieveOrComputeSF(feature->type, rule.sourceSFIndex, rule.cloud2, error);
		if (!sf2)
		{
			//error should be up to date
			return false;
		}
	}

	//shall we extract a statistical measure?
	if (rule.scales && rule.stat != FeatureRule::NO_STAT)
	{
		//duplicate the feature for each scale
		for (double s : rule.scales->values)
		{
			CCLib::ScalarField* statSF1 = ExtractStat(rule.cloud1, sf1, s, rule.stat);
			if (!statSF1)
			{
				ccLog::Warning(QString("Failed to extract stat. from sf '%1' @ scale %2").arg(sf1->getName()).arg(s));
				return false;
			}
			PointFeature::Shared f1(new PointFeature(*feature));
			f1->cloud = rule.cloud1;
			f1->sourceName = statSF1->getName();
			preparedFeatures.push_back(f1);

			if (rule.cloud2 && sf2)
			{
				assert(rule.op != FeatureRule::NO_OPERATION);
				CCLib::ScalarField* statSF2 = ExtractStat(rule.cloud2, sf2, s, rule.stat);
				if (!statSF2)
				{
					ccLog::Warning(QString("Failed to extract stat. from sf '%1' @ scale %2").arg(sf2->getName()).arg(s));
					return false;
				}
				PointFeature::Shared f2(new PointFeature(*feature));
				f2->cloud = rule.cloud2;
				f2->sourceName = statSF2->getName();
				preparedFeatures.push_back(f2);
			}
		}
	}
	else
	{
		//only one version of the main feature
		feature->cloud = rule.cloud1;
		feature->sourceName = sf1->getName();
		preparedFeatures.push_back(rule.feature);
	}

	switch (feature->type)
	{
	case PointFeature::Intensity:
	case PointFeature::X:
	case PointFeature::Y:
	case PointFeature::Z:
	case PointFeature::NbRet:
	case PointFeature::RetNb:
	case PointFeature::EchoRat:
	case PointFeature::R:
	case PointFeature::G:
	case PointFeature::B:
	case PointFeature::NIR:
	case PointFeature::DipAng:
	case PointFeature::DipDir:
	case PointFeature::M3C2:
	case PointFeature::PCV:
	case PointFeature::SF:
	}

}

bool Tools::PrepareFeatures(const FeatureRule::Set& rules, Feature::Set& features, QString& error)
{
	for (const FeatureRule::Shared& rule : rules)
	{
		QString errorMessage("invalid pointer");
		if (!rule || !rule->checkValidity(errorMessage))
		{
			error = "Invalid rule/feature: " + error;
			return false;
		}

		if ()
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

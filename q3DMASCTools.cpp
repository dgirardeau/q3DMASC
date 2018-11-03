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
#include <ccPointCloud.h>

//Qt
#include <QTextStream>
#include <QFile>
#include <QFileInfo>
#include <QDir>

//system
#include <assert.h>

using namespace masc;

bool Tools::LoadFile(	QString filename,
						FeatureRule::Set& features,
						std::vector<ccPointCloud*>& loadedClouds,
						CorePoints& corePoints)
{
	QFileInfo fi(filename);
	if (!fi.exists())
	{
		ccLog::Warning(QString("Can't find file '%1'").arg(filename));
		return false;
	}

	QFile file(filename);
	if (!file.open(QFile::Text | QFile::ReadOnly))
	{
		ccLog::Warning(QString("Can't open file '%1'").arg(filename));
		return false;
	}

	assert(features.empty());
	Scales::Shared scales(new Scales);
	QMap<QString, ccPointCloud* > clouds;

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
			QStringList tokens = command.split('=');
			if (tokens.size() != 2)
			{
				ccLog::Warning("Malformed file: expecting 2 tokens after 'cloud:' on line #" + QString::number(lineNumber));
				return false;
			}
			QString pcName = tokens[0].trimmed();
			QString pcFilename = fi.absoluteDir().absoluteFilePath(tokens[1].trimmed());
			//try to open the cloud
			{
				FileIOFilter::LoadParameters parameters;
				parameters.alwaysDisplayLoadDialog = false;
				CC_FILE_ERROR error = CC_FERR_NO_ERROR;
				ccHObject* object = FileIOFilter::LoadFromFile(pcFilename, parameters, error);
				if (error != CC_FERR_NO_ERROR || !object)
				{
					//error message already issued
					if (object)
						delete object;
					return false;
				}
				ccHObject::Container cloudsInFile;
				object->filterChildren(cloudsInFile, false, CC_TYPES::POINT_CLOUD, true);
				if (cloudsInFile.empty())
				{
					ccLog::Warning("File doesn't contain a single cloud");
					delete object;
					return false;
				}
				else if (cloudsInFile.size() > 1)
				{
					ccLog::Warning("File contains more than one cloud, only the first one will be kept");
				}
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloudsInFile.front());
				for (size_t i = 1; i < cloudsInFile.size(); ++i)
				{
					delete cloudsInFile[i];
				}
				if (pc->getParent())
					pc->getParent()->detachChild(pc);
				pc->setName(pcName);
				clouds.insert(pcName, pc);
				loadedClouds.push_back(pc);
			}
		}
		else if (upperLine.startsWith("CORE_POINTS:")) //core points
		{
			if (corePoints.origin)
			{
				ccLog::Warning("Malformed file: can't declare core points twice! (line #" + QString::number(lineNumber) + ")");
				return false;
			}
			QString command = line.mid(12);
			QStringList tokens = command.split('_');
			if (tokens.empty())
			{
				ccLog::Warning("Malformed file: expecting tokens after 'core_points:' on line #" + QString::number(lineNumber));
				return false;
			}
			QString pcName = tokens[0].trimmed();
			if (!clouds.contains(pcName))
			{
				ccLog::Warning(QString("Malformed file: unknown cloud '%1' on line #%2 (make sure it is declared before the core points)").arg(pcName).arg(lineNumber));
				return false;
			}
			corePoints.origin = clouds[pcName];

			//should we sub-sample the origin cloud?
			if (tokens.size() > 1)
			{
				if (tokens[1].toUpper() == "SS")
				{
					if (tokens.size() < 3)
					{
						ccLog::Warning("Malformed file: missing token after 'SS' on line #" + QString::number(lineNumber));
						return false;
					}
					QString options = tokens[2];
					if (options.startsWith('R'))
					{
						corePoints.selectionMethod = CorePoints::RANDOM;
					}
					else if (options.startsWith('S'))
					{
						corePoints.selectionMethod = CorePoints::SPATIAL;
					}
					else
					{
						ccLog::Warning("Malformed file: unknown option after 'SS' on line #" + QString::number(lineNumber));
						return false;
					}

					//read the subsampling parameter (ignore the first character)
					bool ok = false;
					corePoints.selectionParam = options.mid(1).toDouble(&ok);
					if (!ok)
					{
						ccLog::Warning("Malformed file: expecting a number after 'SS_X' on line #" + QString::number(lineNumber));
						return false;
					}
				
				} //end of subsampling options
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
						QStringList subTokens = token.trimmed().split(':');
						if (subTokens.size() != 3)
						{
							ccLog::Warning(QString("Malformed file: expecting 3 tokens for a range of scales (%1)").arg(token));
							return false;
						}
						bool ok[3] = { true, true, true };
						double start = subTokens[0].trimmed().toDouble(ok);
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
						for (double v = start; v <= stop + 1.0e-6; v += step)
						{
							scales->values.push_back(v);
						}
					}
					else
					{
						bool ok = true;
						double v = token.trimmed().toDouble(&ok);
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
			QString typeStr = tokens[0].trimmed().toUpper();
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
							rule->sourceSFIndex = sfIndex;
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
					scaleStr = scaleStr.mid(2);
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
				QString token = tokens[i].trimmed().toUpper();

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
					for (QMap<QString, ccPointCloud* >::const_iterator it = clouds.begin(); it != clouds.end(); ++it)
					{
						QString key = it.key().toUpper();
						if (key == token)
						{
							if (cloudCount == 0)
								rule->cloud1 = it.value();
							else
								rule->cloud2 = it.value();
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
			bool ruleIsValid = rule->checkValidity(/*corePoints, */errorMessage);
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

static QSharedPointer<IScalarFieldWrapper> RetrieveField(PointFeature::PointFeatureType featureType, int sourceSFIndex, ccPointCloud* cloud, QString& error)
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
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::X:
		return QSharedPointer<IScalarFieldWrapper>(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimX));
	case PointFeature::Y:
		return QSharedPointer<IScalarFieldWrapper>(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimY));
	case PointFeature::Z:
		return QSharedPointer<IScalarFieldWrapper>(new DimScalarFieldWrapper(cloud, DimScalarFieldWrapper::DimZ));
	case PointFeature::NbRet:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS], false);
		if (!sf)
		{
			error = "Cloud has no 'number of returns' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::RetNb:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_RETURN_NUMBER], false);
		if (!sf)
		{
			error = "Cloud has no 'return number' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::EchoRat:
	{
		//retrieve the two scalar fields 'p/q'
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
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldRatioWrapper(retNumberSF, numberOfRetSF, "EchoRat"));
	}
	case PointFeature::R:
		return QSharedPointer<IScalarFieldWrapper>(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Red));
	case PointFeature::G:
		return QSharedPointer<IScalarFieldWrapper>(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Green));
	case PointFeature::B:
		return QSharedPointer<IScalarFieldWrapper>(new ColorScalarFieldWrapper(cloud, ColorScalarFieldWrapper::Blue));
	case PointFeature::NIR:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_NIRSFName, false);
		if (!sf)
		{
			error = "Cloud has no 'NIR' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::DipAng:
	case PointFeature::DipDir:
	{
		//we need normals to compute the dip and dip direction!
		if (!cloud->hasNormals())
		{
			error = "Cloud has no normals: can't compute dip or dip dir. angles";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new NormDipAndDipDirFieldWrapper(cloud, featureType == PointFeature::DipAng ? NormDipAndDipDirFieldWrapper::Dip : NormDipAndDipDirFieldWrapper::DipDir));
	}
	case PointFeature::M3C2:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_M3C2SFName, true);
		if (!sf)
		{
			error = "Cloud has no 'm3c2 distance' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::PCV:
	{
		CCLib::ScalarField* sf = RetrieveSF(cloud, s_PCVSFName, true);
		if (!sf)
		{
			error = "Cloud has no 'PCV/Illuminance' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::SF:
		if (sourceSFIndex < 0 || sourceSFIndex >= static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			error = QString("Can't retrieve the specified SF: invalid index (%1)").arg(sourceSFIndex);
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(cloud->getScalarField(sourceSFIndex)));
	default:
		break;
	}

	error = "Unhandled feature type";
	return nullptr;
}

static bool ExtractStatFromSF(	const CCVector3& queryPoint,
								const CCLib::DgmOctree* octree,
								unsigned char octreeLevel,
								FeatureRule::Stat stat,
								const IScalarFieldWrapper& inputField,
								PointCoordinateType radius,
								double& outputValue)
{
	if (!octree)
	{
		assert(false);
		return false;
	}
	outputValue = std::numeric_limits<double>::quiet_NaN();

	//spherical neighborhood extraction structure
	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	{
		nNSS.level = octreeLevel;
		nNSS.queryPoint = queryPoint;
		nNSS.prepare(radius, octree->getCellSize(nNSS.level));
		octree->getTheCellPosWhichIncludesThePoint(&nNSS.queryPoint, nNSS.cellPos, nNSS.level);
		octree->computeCellCenter(nNSS.cellPos, nNSS.level, nNSS.cellCenter);
	}

	//we extract the point's neighbors
	unsigned kNN = octree->findNeighborsInASphereStartingFromCell(nNSS, radius, true);
	if (kNN == 0)
	{
		return true;
	}

	//specific case
	if (stat == FeatureRule::RANGE)
	{
		double minValue = 0;
		double maxValue = 0;

		for (unsigned k = 0; k < kNN; ++k)
		{
			unsigned index = nNSS.pointsInNeighbourhood[k].pointIndex;
			double v = inputField.pointValue(index);

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
		}

		outputValue = maxValue - minValue;
		return true;
	}

	bool withSums = (stat == FeatureRule::MEAN || stat == FeatureRule::STD || stat == FeatureRule::SKEW);
	bool withMode = (stat == FeatureRule::MODE || stat == FeatureRule::SKEW);
	double sum = 0.0;
	double sum2 = 0.0;
	QMap<float, unsigned> modeCounter;

	for (unsigned k = 0; k < kNN; ++k)
	{
		unsigned index = nNSS.pointsInNeighbourhood[k].pointIndex;
		double v = inputField.pointValue(index);

		if (withSums)
		{
			//compute average and std. dev.
			sum += v;
			sum2 += v * v;
		}

		if (withMode)
		{
			//store the number of occurences of each value
			//DGM TODO: it would be better with a custom 'resolution' if the field is not an integer one
			float vf = static_cast<float>(v);
			if (modeCounter.contains(vf))
			{
				++modeCounter[vf];
			}
			else
			{
				modeCounter[vf] = 1;
			}
		}
	}

	double mode = std::numeric_limits<double>::quiet_NaN();
	if (withMode)
	{
		//look for the value with the highest frequency
		unsigned maxCounter = 0;
		for (QMap<ScalarType, unsigned>::const_iterator it = modeCounter.begin(); it != modeCounter.end(); ++it)
		{
			if (it.value() > maxCounter)
			{
				maxCounter = it.value();
				mode = it.key();
			}
		}
	}

	switch (stat)
	{
	case FeatureRule::MEAN:
		outputValue = sum / kNN;
		break;
	case FeatureRule::MODE:
		outputValue = mode;
		break;
	case FeatureRule::STD:
		outputValue = sqrt(std::abs(sum2 * kNN - sum * sum)) / kNN;
		break;
	case FeatureRule::RANGE:
		//we can't be here
		assert(false);
		return false;
	case FeatureRule::SKEW:
	{
		double mean = sum / kNN;
		double std = sqrt(std::abs(sum2 / kNN - mean * mean));
		if (std > std::numeric_limits<float>::epsilon()) //arbitrary epsilon
		{
			outputValue = (mean - mode) / std;
		}
		break;
	}
	default:
		ccLog::Warning("Unhandled STAT measure");
		assert(false);
		return false;
	}

	return true;
}

static CCLib::ScalarField* ExtractStat(	const CorePoints& corePoints, 
										ccPointCloud* sourceCloud,
										const IScalarFieldWrapper* sourceField,
										double scale,
										FeatureRule::Stat stat,
										const char* resultSFName,
										CCLib::GenericProgressCallback* progressCb = nullptr)
{
	if (!corePoints.cloud || !sourceCloud || !sourceField || scale <= 0.0 || stat == FeatureRule::NO_STAT || !resultSFName)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	ccOctree::Shared octree = sourceCloud->getOctree();
	if (!octree)
	{
		octree = sourceCloud->computeOctree(progressCb);
		if (!octree)
		{
			ccLog::Warning("Failed to compute octree");
			return nullptr;
		}
	}

	CCLib::ScalarField* resultSF = nullptr;
	int sfIdx = corePoints.cloud->getScalarFieldIndexByName(resultSFName);
	if (sfIdx >= 0)
	{
		resultSF = corePoints.cloud->getScalarField(sfIdx);
	}
	else
	{
		resultSF = new ccScalarField(resultSFName);
		if (!resultSF->resizeSafe(corePoints.cloud->size()))
		{
			ccLog::Warning("Not enough memory");
			resultSF->release();
			return nullptr;
		}
	}
	resultSF->fill(NAN_VALUE);

	PointCoordinateType radius = static_cast<PointCoordinateType>(scale / 2);
	unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius); //scale is the diameter!

	unsigned pointCount = corePoints.size();
	progressCb->setInfo(qPrintable(QString("Computing field: %1\n(core points: %2)").arg(resultSFName).arg(pointCount)));
	CCLib::NormalizedProgress nProgress(progressCb, pointCount);

	for (unsigned i = 0; i < pointCount; ++i)
	{
		double outputValue = 0;
		if (!ExtractStatFromSF(	*corePoints.cloud->getPoint(i),
								octree.data(),
								octreeLevel,
								stat,
								*sourceField,
								radius,
								outputValue))
		{
			//unexpected error
			resultSF->release();
			return nullptr;
		}

		ScalarType v = static_cast<ScalarType>(outputValue);
		resultSF->setValue(i, v);

		if (progressCb && !nProgress.oneStep())
		{
			//process cancelled by the user
			ccLog::Warning("Process cancelled");
			resultSF->release();
			return nullptr;
		}
	}

	resultSF->computeMinAndMax();
	int newSFIdx = corePoints.cloud->addScalarField(static_cast<ccScalarField*>(resultSF));
	//update display
	if (corePoints.cloud->getDisplay())
	{
		corePoints.cloud->setCurrentDisplayedScalarField(newSFIdx);
		corePoints.cloud->getDisplay()->redraw();
	}

	return resultSF;
}

static bool PerformMathOp(CCLib::ScalarField* sf1, const CCLib::ScalarField* sf2, FeatureRule::Operation op)
{
	if (!sf1 || !sf2 || sf1->size() != sf2->size() || op == FeatureRule::NO_OPERATION)
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
		case FeatureRule::MINUS:
			s = s1 - s2;
			break;
		case FeatureRule::PLUS:
			s = s1 + s2;
			break;
		case FeatureRule::DIVIDE:
			if (std::abs(s2) > std::numeric_limits<ScalarType>::epsilon())
				s = s1 / s2;
			break;
		case FeatureRule::MULTIPLY:
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

static Feature::Shared PreparePointBasedFeature(const FeatureRule& rule,
												double scale,
												const CorePoints& corePoints,
												QString& error,
												CCLib::GenericProgressCallback* progressCb = nullptr)
{
	if (!rule.cloud1 || !rule.feature || rule.feature->getType() != Feature::Type::PointFeature || !corePoints.cloud)
	{
		//invalid input
		assert(false);
		return false;
	}
	PointFeature::PointFeatureType featureType = static_cast<PointFeature*>(rule.feature.data())->type;

	//look for the source field
	QSharedPointer<IScalarFieldWrapper> field1 = RetrieveField(featureType, rule.sourceSFIndex, rule.cloud1, error);
	if (!field1)
	{
		//error should be up to date
		return false;
	}

	//shall we extract a statistical measure? (= scaled feature)
	if (std::isfinite(scale))
	{
		if (rule.stat == FeatureRule::NO_STAT)
		{
			assert(false);
			ccLog::Warning("Scaled features (SCx) must have an associated STAT measure");
			return false;
		}

		QSharedPointer<IScalarFieldWrapper> field2;
		if (rule.cloud2)
		{
			//no need to compute the second scalar field if no MATH operation has to be performed?!
			if (rule.op != FeatureRule::NO_OPERATION)
			{
				field2 = RetrieveField(featureType, rule.sourceSFIndex, rule.cloud2, error);
				if (!field2)
				{
					//error should be up to date
					return false;
				}
			}
			else
			{
				assert(false);
				ccLog::Warning("Feature has a second cloud associated but no MATH operation is defined");
			}
		}

		//build the final SF name
		QString resultSFName = rule.cloud1->getName() + "." + field1->getName() + QString("_") + FeatureRule::StatToString(rule.stat);
		if (field2 && rule.op != FeatureRule::NO_OPERATION)
		{
			//include the math operation as well if necessary!
			resultSFName += "_" + FeatureRule::OpToString(rule.op) + "_" + rule.cloud2->getName() + "." + field2->getName() + QString("_") + FeatureRule::StatToString(rule.stat);
		}
		resultSFName += "@" + QString::number(scale);

		CCLib::ScalarField* statSF1 = ExtractStat(corePoints, rule.cloud1, field1.data(), scale, rule.stat, qPrintable(resultSFName), progressCb);
		if (!statSF1)
		{
			error = QString("Failed to extract stat. from field '%1' @ scale %2").arg(field1->getName()).arg(scale);
			return false;
		}

		PointFeature::Shared feature(new PointFeature(*static_cast<PointFeature*>(rule.feature.data())));
		feature->cloud = corePoints.cloud;
		feature->sourceName = statSF1->getName();
		feature->scale = scale;

		if (rule.cloud2 && field2 && rule.op != FeatureRule::NO_OPERATION)
		{
			QString resultSFName2 = rule.cloud2->getName() + "." + field2->getName() + QString("_") + FeatureRule::StatToString(rule.stat) + "@" + QString::number(scale);
			int sfIndex2 = corePoints.cloud->getScalarFieldIndexByName(qPrintable(resultSFName2));
			CCLib::ScalarField* statSF2 = ExtractStat(corePoints, rule.cloud2, field2.data(), scale, rule.stat, qPrintable(resultSFName2), progressCb);
			if (!statSF2)
			{
				error = QString("Failed to extract stat. from field '%1' @ scale %2").arg(field2->getName()).arg(scale);
				return false;
			}

			//now perform the math operation
			if (!PerformMathOp(statSF1, statSF2, rule.op))
			{
				error = "Failed to perform the MATH operation";
				return false;
			}

			if (sfIndex2 < 0)
			{
				//release some memory
				sfIndex2 = corePoints.cloud->getScalarFieldIndexByName(qPrintable(resultSFName2));
				corePoints.cloud->deleteScalarField(sfIndex2);
			}
		}

		return feature;
	}
	else //non scaled feature
	{
		if (rule.cloud1 != corePoints.cloud && rule.cloud1 != corePoints.origin)
		{
			assert(false);
			error = "Scale-less features (SC0) can only be defined on the core points (origin) cloud";
			return false;
		}

		if (rule.cloud2)
		{
			if (rule.op != FeatureRule::NO_OPERATION)
			{
				assert(false);
				ccLog::Warning("MATH operations cannot be performed on scale-less features (SC0)");
				return false;
			}
			else
			{
				assert(false);
				ccLog::Warning("Feature has a second cloud associated but no MATH operation is defined");
			}
		}

		//build the final SF name
		QString resultSFName = /*rule.cloud1->getName() + "." + */field1->getName();
		//if (rule.cloud2 && field2 && rule.op != FeatureRule::NO_OPERATION)
		//{
		//	resultSFName += QString("_") + FeatureRule::OpToString(rule.op) + "_" + field2->getName();
		//}

		//retrieve/create a SF to host the result
		CCLib::ScalarField* resultSF = nullptr;
		int sfIdx = corePoints.cloud->getScalarFieldIndexByName(qPrintable(resultSFName));
		if (sfIdx >= 0)
		{
			//reuse the existing field
			resultSF = corePoints.cloud->getScalarField(sfIdx);
		}
		else
		{
			//copy the SF1 field
			resultSF = new ccScalarField(qPrintable(resultSFName));
			if (!resultSF->resizeSafe(corePoints.cloud->size()))
			{
				error = "Not enough memory";
				resultSF->release();
				return false;
			}

			//copy the values
			for (unsigned i = 0; i < corePoints.size(); ++i)
			{
				resultSF->setValue(i, field1->pointValue(corePoints.originIndex(i)));
			}
			resultSF->computeMinAndMax();
			int newSFIdx = corePoints.cloud->addScalarField(static_cast<ccScalarField*>(resultSF));
			//update display
			if (corePoints.cloud->getDisplay())
			{
				corePoints.cloud->setCurrentDisplayedScalarField(newSFIdx);
				corePoints.cloud->getDisplay()->redraw();
			}
		}

		rule.feature->cloud = corePoints.cloud;
		rule.feature->sourceName = resultSF->getName();
		rule.feature->scale = scale;

		//if (rule.cloud2 && field2 && rule.op != FeatureRule::NO_OPERATION)
		//{
		//	//now perform the math operation
		//	if (!PerformMathOp(*field1, *field2, rule.op, resultSF))
		//	{
		//		error = "Failed to perform the MATH operation";
		//		return false;
		//	}

		//	//sf2 is held by the second cloud for now
		//	//sf2->release();
		//	//sf2 = nullptr;
		//}

		return rule.feature;
	}
}

bool Tools::PrepareFeatures(const FeatureRule::Set& rules, const CorePoints& corePoints, Feature::Set& features, QString& error, CCLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (rules.empty() || !corePoints.origin)
	{
		//invalid input parameters
		assert(false);
		return false;
	}
	
	for (const FeatureRule::Shared& rule : rules)
	{
		QString errorMessage("invalid pointer");
		if (!rule || !rule->checkValidity(/*corePoints, */errorMessage))
		{
			error = "Invalid rule/feature: " + error;
			return false;
		}

		size_t scaleCount = (rule->scales ? rule->scales->values.size(): 1);
		for (size_t i = 0; i < scaleCount; ++i)
		{
			//retrieve the right scale
			double scale = std::numeric_limits<double>::quiet_NaN();
			if (rule->scales)
			{
				scale = rule->scales->values[i];
			}

			Feature::Shared preparedFeature;

			//we will prepare the different versions of the feature (one per scale, etc.)
			//depending on the feature type
			switch (rule->feature->getType())
			{
			case Feature::Type::PointFeature:
			{
				//Point feature
				preparedFeature = PreparePointBasedFeature(*rule, scale, corePoints, error, progressCb);
				break;
			}
			default:
				assert(false);
				break;
			}

			if (!preparedFeature)
			{
				//something failed (error should be up to date)
				return false;
			}
			
			//otherwise add the new feature
			features.push_back(preparedFeature);
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

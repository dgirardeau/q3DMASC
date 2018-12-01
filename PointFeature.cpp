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

#include "PointFeature.h"

//Local
#include "q3DMASCTools.h"

//qCC_io
#include <LASFields.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//CCLib
#include <WeibullDistribution.h>

//system
#include <assert.h>

//Qt
#include <QCoreApplication>

static const char* s_echoRatioSFName = "EchoRat";
static const char* s_NIRSFName = "NIR";
static const char* s_M3C2SFName = "M3C2 distance";
static const char* s_PCVSFName = "Illuminance (PCV)";
static const char* s_normDipSFName = "Norm dip";
static const char* s_normDipDirSFName = "Norm dip dir.";

using namespace masc;

bool PointFeature::checkValidity(QString &error) const
{
	if (!Feature::checkValidity(error))
	{
		return false;
	}

	assert(cloud1);

	if (scaled() && stat == NO_STAT)
	{
		error = "scaled point features need a STAT measure to be defined";
		return false;
	}

	if (op != NO_OPERATION && !scaled())
	{
		error = "math operations can't be defined on scale-less point features (SC0)";
		return false;
	}
	
	switch (type)
	{
	case PointFeature::Intensity:
	{
		if (cloud1->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_INTENSITY]) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(LAS_FIELD_NAMES[LAS_INTENSITY]);
			return false;
		}
		return true;
	}
	case PointFeature::X:
	case PointFeature::Y:
	case PointFeature::Z:
		return true;
	case PointFeature::NbRet:
	{
		if (cloud1->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS]) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS]);
			return false;
		}
		return true;
	}
	case PointFeature::RetNb:
	{
		if (cloud1->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_RETURN_NUMBER]) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(LAS_FIELD_NAMES[LAS_RETURN_NUMBER]);
			return false;
		}
		return true;
	}
	case PointFeature::EchoRat:
	{
		if (cloud1->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS]) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS]);
			return false;
		}
		if (cloud1->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_RETURN_NUMBER]) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(LAS_FIELD_NAMES[LAS_RETURN_NUMBER]);
			return false;
		}
		return true;
	}
	case PointFeature::R:
	case PointFeature::G:
	case PointFeature::B:
		if (!cloud1->hasColors())
		{
			error = "Cloud has no RGB color";
			return false;
		}
		return true;
	case PointFeature::NIR:
	{
		if (cloud1->getScalarFieldIndexByName(s_NIRSFName) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(s_NIRSFName);
			return false;
		}
		return true;
	}
	case PointFeature::DipAng:
	case PointFeature::DipDir:
	{
		if (!cloud1->hasNormals())
		{
			error = "Cloud has no normals";
			return false;
		}
		return true;
	}
	case PointFeature::M3C2:
	{
		if (cloud1->getScalarFieldIndexByName(s_M3C2SFName) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(s_M3C2SFName);
			return false;
		}
		return true;
	}
	case PointFeature::PCV:
	{
		if (cloud1->getScalarFieldIndexByName(s_PCVSFName) < 0)
		{
			error = QString("Cloud has no '%1' scalar field").arg(s_PCVSFName);
			return false;
		}
		return true;
	}
	case PointFeature::SF:
		if (sourceSFIndex >= static_cast<int>(cloud1->getNumberOfScalarFields()))
		{
			error = QString("Cloud has no scalar field #%1").arg(sourceSFIndex);
			return false;
		}
		return true;
	default:
		break;
	}

	return true;
}

QSharedPointer<IScalarFieldWrapper> PointFeature::retrieveField(ccPointCloud* cloud, QString& error)
{
	if (!cloud)
	{
		assert(false);
		return QSharedPointer<IScalarFieldWrapper>(nullptr);
	}
	
	switch (type)
	{
	case PointFeature::Intensity:
	{
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_INTENSITY], false);
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
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS], false);
		if (!sf)
		{
			error = "Cloud has no 'number of returns' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::RetNb:
	{
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_RETURN_NUMBER], false);
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
		CCLib::ScalarField* numberOfRetSF = Tools::RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_NUMBER_OF_RETURNS], false);
		if (!numberOfRetSF)
		{
			error = "Can't compute the 'echo ratio' field: no 'Number of Return' SF available";
			return nullptr;
		}
		CCLib::ScalarField* retNumberSF = Tools::RetrieveSF(cloud, LAS_FIELD_NAMES[LAS_RETURN_NUMBER], false);
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
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, s_NIRSFName, false);
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
		return QSharedPointer<IScalarFieldWrapper>(new NormDipAndDipDirFieldWrapper(cloud, type == PointFeature::DipAng ? NormDipAndDipDirFieldWrapper::Dip : NormDipAndDipDirFieldWrapper::DipDir));
	}
	case PointFeature::M3C2:
	{
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, s_M3C2SFName, true);
		if (!sf)
		{
			error = "Cloud has no 'm3c2 distance' scalar field";
			return nullptr;
		}
		return QSharedPointer<IScalarFieldWrapper>(new ScalarFieldWrapper(sf));
	}
	case PointFeature::PCV:
	{
		CCLib::ScalarField* sf = Tools::RetrieveSF(cloud, s_PCVSFName, true);
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
								Feature::Stat stat,
								const IScalarFieldWrapper& inputField,
								PointCoordinateType radius,
								double& outputValue)
{
	if (!octree)
	{
		assert(false);
		return false;
	}

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
	if (stat == Feature::RANGE)
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
	else
	{
		bool withSums = (stat == Feature::MEAN || stat == Feature::STD);
		bool storeValues = (stat == Feature::MODE || stat == Feature::SKEW);
		double sum = 0.0;
		double sum2 = 0.0;

		CCLib::WeibullDistribution::ScalarContainer values;
		if (storeValues)
		{
			try
			{
				values.resize(kNN);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("Not enough memory");
				return false;
			}
		}

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

			if (storeValues)
			{
				values[k] = static_cast<ScalarType>(v);
			}
		}

		switch (stat)
		{
		case Feature::MEAN:
		{
			outputValue = sum / kNN;
		}
		break;

		case Feature::MODE:
		{
			CCLib::WeibullDistribution w;
			w.computeParameters(values);
			outputValue = w.computeMode();
		}
		break;

		case Feature::STD:
		{
			outputValue = sqrt(std::abs(sum2 * kNN - sum * sum)) / kNN;
		}
		break;

		case Feature::RANGE:
		{
			//we can't be here
			assert(false);
		}
		return false;

		case Feature::SKEW:
		{
			CCLib::WeibullDistribution w;
			w.computeParameters(values);
			outputValue = w.computeSkewness();
		}
		break;

		default:
		{
			ccLog::Warning("Unhandled STAT measure");
			assert(false);
		}
		return false;

		}
	}

	return true;
}

static CCLib::ScalarField* ExtractStat(	const CorePoints& corePoints, 
										ccPointCloud* sourceCloud,
										const IScalarFieldWrapper* sourceField,
										double scale,
										Feature::Stat stat,
										const char* resultSFName,
										CCLib::GenericProgressCallback* progressCb = nullptr)
{
	if (!corePoints.cloud || !sourceCloud || !sourceField || scale <= 0.0 || stat == Feature::NO_STAT || !resultSFName)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	ccOctree::Shared octree = sourceCloud->getOctree();
	if (!octree)
	{
		ccLog::Print(QString("Computing octree of cloud %1 (%2 points)").arg(sourceCloud->getName()).arg(sourceCloud->size()));
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
	if (progressCb)
	{
		progressCb->setInfo(qPrintable(QString("Computing field: %1\n(core points: %2)").arg(resultSFName).arg(pointCount)));
	}
	ccLog::Print(QString("Computing field: %1 (core points: %2)").arg(resultSFName).arg(pointCount));
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
	//if (corePoints.cloud->getDisplay())
	{
		corePoints.cloud->setCurrentDisplayedScalarField(newSFIdx);
		//corePoints.cloud->getDisplay()->redraw();
		//QCoreApplication::processEvents();
	}

	return resultSF;
}

bool PointFeature::prepare(	const CorePoints& corePoints,
							QString& error,
							CCLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!cloud1 || !corePoints.cloud)
	{
		//invalid input
		assert(false);
		error = "internal error (no input core points)";
		return false;
	}

	//look for the source field
	assert(!field1);
	field1 = retrieveField(cloud1, error);
	if (!field1)
	{
		//error should be up to date
		return false;
	}

	//shall we extract a statistical measure? (= scaled feature)
	if (scaled())
	{
		if (stat == Feature::NO_STAT)
		{
			assert(false);
			ccLog::Warning("Scaled features (SCx) must have an associated STAT measure");
			return false;
		}

		if (cloud2)
		{
			//no need to compute the second scalar field if no MATH operation has to be performed?!
			if (op != Feature::NO_OPERATION)
			{
				assert(!field2);
				field2 = retrieveField(cloud2, error);
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
				return false;
			}
		}

		//build the final SF name
		QString resultSFName = cloud1Label + "." + field1->getName() + QString("_") + Feature::StatToString(stat);
		if (field2 && op != Feature::NO_OPERATION)
		{
			//include the math operation as well if necessary!
			resultSFName += "_" + Feature::OpToString(op) + "_" + cloud2Label + "." + field2->getName() + QString("_") + Feature::StatToString(stat);
		}
		resultSFName += "@" + QString::number(scale);

		//and the scalar field
		assert(!statSF1);
		statSF1 = PrepareSF(corePoints.cloud, qPrintable(resultSFName));
		if (!statSF1)
		{
			error = QString("Failed to prepare scalar field for field '%1' @ scale %2").arg(field1->getName()).arg(scale);
			return false;
		}
		sourceName = statSF1->getName();

		if (cloud2 && field2 && op != Feature::NO_OPERATION)
		{
			QString resultSFName2 = cloud2Label + "." + field2->getName() + QString("_") + Feature::StatToString(stat) + "@" + QString::number(scale);
			keepStatSF2 = (corePoints.cloud->getScalarFieldIndexByName(qPrintable(resultSFName2)) >= 0); //we remember that the scalar field was already existing!
			
			assert(!statSF2);
			statSF2 = PrepareSF(corePoints.cloud, qPrintable(resultSFName2));
			if (!statSF2)
			{
				error = QString("Failed to prepare scalar field for field '%1' @ scale %2").arg(field2->getName()).arg(scale);
				return false;
			}
		}

		return true;
	}
	else //non scaled feature
	{
		if (cloud1 != corePoints.cloud && cloud1 != corePoints.origin)
		{
			assert(false);
			error = "Scale-less features (SC0) can only be defined on the core points (origin) cloud";
			return false;
		}

		if (cloud2)
		{
			if (op != Feature::NO_OPERATION)
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
		QString resultSFName = /*cloud1Label + "." + */field1->getName();

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
			//if (corePoints.cloud->getDisplay())
			{
				corePoints.cloud->setCurrentDisplayedScalarField(newSFIdx);
				//corePoints.cloud->getDisplay()->redraw();
				//QCoreApplication::processEvents();
			}
		}

		sourceName = resultSF->getName();

		return true;
	}
}

bool PointFeature::computeStat(const CCLib::DgmOctree::NeighboursSet& pointsInNeighbourhood, const QSharedPointer<IScalarFieldWrapper>& sourceField, double& outputValue) const
{
	outputValue = std::numeric_limits<double>::quiet_NaN();

	if (!sourceField || stat == Feature::NO_STAT)
	{
		//invalid input parameters
		assert(false);
		return false;
	}

	size_t kNN = pointsInNeighbourhood.size();
	if (kNN == 0)
	{
		assert(false);
		return false;
	}

	//specific case
	if (stat == Feature::RANGE)
	{
		double minValue = 0;
		double maxValue = 0;

		for (size_t k = 0; k < kNN; ++k)
		{
			unsigned index = pointsInNeighbourhood[k].pointIndex;
			double v = sourceField->pointValue(index);

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
	else
	{
		bool withSums = (stat == Feature::MEAN || stat == Feature::STD);
		bool storeValues = (stat == Feature::MODE || stat == Feature::SKEW);
		double sum = 0.0;
		double sum2 = 0.0;

		CCLib::WeibullDistribution::ScalarContainer values;
		if (storeValues)
		{
			try
			{
				values.resize(kNN);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("Not enough memory");
				return false;
			}
		}

		for (unsigned k = 0; k < kNN; ++k)
		{
			unsigned index = pointsInNeighbourhood[k].pointIndex;
			double v = sourceField->pointValue(index);

			if (withSums)
			{
				//compute average and std. dev.
				sum += v;
				sum2 += v * v;
			}

			if (storeValues)
			{
				values[k] = static_cast<ScalarType>(v);
			}
		}

		switch (stat)
		{
		case Feature::MEAN:
		{
			outputValue = sum / kNN;
		}
		break;

		case Feature::MODE:
		{
			CCLib::WeibullDistribution w;
			w.computeParameters(values);
			outputValue = w.computeMode();
		}
		break;

		case Feature::STD:
		{
			outputValue = sqrt(std::abs(sum2 * kNN - sum * sum)) / kNN;
		}
		break;

		case Feature::RANGE:
		{
			//we can't be here
			assert(false);
		}
		return false;

		case Feature::SKEW:
		{
			CCLib::WeibullDistribution w;
			w.computeParameters(values);
			outputValue = w.computeSkewness();
		}
		break;

		default:
		{
			ccLog::Warning("Unhandled STAT measure");
			assert(false);
		}
		return false;

		}
	}

	return true;
}

bool PointFeature::finish(const CorePoints& corePoints, QString& error)
{
	if (!scaled())
	{
		//nothing to do
		return true;
	}

	if (!corePoints.cloud)
	{
		//invalid input
		assert(false);
		error = "internal error (no input core points)";
		return false;
	}

	bool success = true;

	if (statSF1)
	{
		statSF1->computeMinAndMax();

		//update display
		//if (corePoints.cloud->getDisplay())
		{
			int sfIndex1 = corePoints.cloud->getScalarFieldIndexByName(statSF1->getName());
			corePoints.cloud->setCurrentDisplayedScalarField(sfIndex1);
			//corePoints.cloud->getDisplay()->redraw();
			//QCoreApplication::processEvents();
		}
	}

	if (statSF2)
	{
		//now perform the math operation
		if (op != Feature::NO_OPERATION)
		{
			if (!PerformMathOp(statSF1, statSF2, op))
			{
				error = "Failed to perform the MATH operation";
				success = false;
			}
		}

		if (keepStatSF2)
		{
			statSF2->computeMinAndMax();
		}
		else
		{
			int sfIndex2 = corePoints.cloud->getScalarFieldIndexByName(statSF2->getName());
			if (sfIndex2 >= 0)
			{
				corePoints.cloud->deleteScalarField(sfIndex2);
			}
			else
			{
				assert(false);
				statSF2->release();
			}
			statSF2 = nullptr;
		}
	}

	return success;
}

QString PointFeature::toString() const
{
	//default keyword otherwise
	QString description = ToString(type);

	//special case for the 'SF' type
	if (type == SF)
	{
		//'SF#' + sf index
		description += QString::number(sourceSFIndex);
	}

	if (scaled())
	{
		description += QString("_SC%1_%2").arg(scale).arg(StatToString(stat));
	}
	else
	{
		description += "_SC0";
	}

	description += "_" + cloud1Label;

	if (cloud2 && !cloud2Label.isEmpty())
	{
		description += "_" + cloud2Label;

		if (op != NO_OPERATION)
		{
			description += "_" + OpToString(op);
		}
	}

	//Point features always have a scale equal to 0 by definition
	return description;
}

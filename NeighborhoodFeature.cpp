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

#include "NeighborhoodFeature.h"

//CCLib
#include <DgmOctreeReferenceCloud.h>
#include <Jacobi.h>

using namespace masc;

bool NeighborhoodFeature::checkValidity(QString &error) const
{
	if (!Feature::checkValidity(error))
	{
		return false;
	}

	if (stat != Feature::NO_STAT)
	{
		error = "Neighborhood features shouldn't be associated to a STAT measure";
		return false;
	}

	if (cloud2 && op == NO_OPERATION)
	{
		error = "Feature has a second cloud associated but no MATH operation is defined";
		return false;
	}

	return true;
}

bool NeighborhoodFeature::prepare(	const CorePoints& corePoints,
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

	if (!checkValidity(error))
	{
		assert(false);
		return false;
	}

	//build the final SF name
	QString resultSFName = ToString(type) + "_" + cloud1Label;
	if (cloud2)
	{
		//include the math operation as well if necessary!
		resultSFName += "_" + Feature::OpToString(op) + "_" + cloud2Label;
	}
	resultSFName += "@" + QString::number(scale);

	//and the scalar field
	assert(!sf1);
	sf1 = PrepareSF(corePoints.cloud, qPrintable(resultSFName));
	if (!sf1)
	{
		error = QString("Failed to prepare scalar %1 @ scale %2").arg(cloud1Label).arg(scale);
		return false;
	}
	sourceName = sf1->getName();

	if (cloud2 && op != Feature::NO_OPERATION)
	{
		QString resultSFName2 = ToString(type) + "_" + cloud2Label + "@" + QString::number(scale);
		keepSF2 = (corePoints.cloud->getScalarFieldIndexByName(qPrintable(resultSFName2)) >= 0); //we remember that the scalar field was already existing!

		assert(!sf2);
		sf2 = PrepareSF(corePoints.cloud, qPrintable(resultSFName2));
		if (!sf2)
		{
			error = QString("Failed to prepare scalar field for %1 @ scale %2").arg(cloud2Label).arg(scale);
			return false;
		}
	}

	return true;
}

bool NeighborhoodFeature::finish(const CorePoints& corePoints, QString& error)
{
	if (!corePoints.cloud)
	{
		//invalid input
		assert(false);
		error = "internal error (no input core points)";
		return false;
	}

	bool success = true;

	if (sf1)
	{
		sf1->computeMinAndMax();

		//update display
		//if (corePoints.cloud->getDisplay())
		{
			int sfIndex1 = corePoints.cloud->getScalarFieldIndexByName(sf1->getName());
			corePoints.cloud->setCurrentDisplayedScalarField(sfIndex1);
			//corePoints.cloud->getDisplay()->redraw();
			//QCoreApplication::processEvents();
		}
	}

	if (sf2)
	{
		//now perform the math operation
		if (op != Feature::NO_OPERATION)
		{
			if (!PerformMathOp(sf1, sf2, op))
			{
				error = "Failed to perform the MATH operation";
				success = false;
			}
		}

		if (keepSF2)
		{
			sf2->computeMinAndMax();
		}
		else
		{
			int sfIndex2 = corePoints.cloud->getScalarFieldIndexByName(sf2->getName());
			if (sfIndex2 >= 0)
			{
				corePoints.cloud->deleteScalarField(sfIndex2);
			}
			else
			{
				assert(false);
				sf2->release();
			}
			sf2 = nullptr;
		}
	}

	return success;
}

QString NeighborhoodFeature::toString() const
{
	//use the default keyword + the scale
	QString description = ToString(type) + "_SC" + QString::number(scale);

	description += "_" + cloud1Label;

	if (cloud2 && !cloud2Label.isEmpty())
	{
		description += "_" + cloud2Label;

		if (op != NO_OPERATION)
		{
			description += "_" + OpToString(op);
		}
	}

	return description;
}

bool NeighborhoodFeature::computeValue(CCLib::DgmOctree::NeighboursSet& pointsInNeighbourhood, const CCVector3& queryPoint, double& outputValue) const
{
	outputValue = std::numeric_limits<double>::quiet_NaN();

	size_t kNN = pointsInNeighbourhood.size();
	if (kNN == 0)
	{
		assert(false);
		return false;
	}

	switch (type)
	{
	//features relying on the PCA
	case PCA1:
	case PCA2:
	case SPHER:
	case LINEA:
	case PLANA:
	case FOM:
	case LINEF:
	case ORIENF:
	if (kNN >= 6)
	{
		CCLib::DgmOctreeReferenceCloud neighboursCloud(&pointsInNeighbourhood, static_cast<unsigned>(kNN));
		CCLib::Neighbourhood Z(&neighboursCloud);
		CCLib::SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (Jacobi<double>::ComputeEigenValuesAndVectors(Z.computeCovarianceMatrix(), eigVectors, eigValues, true))
		{
			Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //decreasing order of their associated eigenvalues
			switch (type)
			{
			case PCA1:
				outputValue = eigValues[0] / (eigValues[0] + eigValues[1] + eigValues[2]);
				break;
			case PCA2:
				outputValue = eigValues[1] / (eigValues[0] + eigValues[1] + eigValues[2]);
				break;
			case SPHER:
				if (std::abs(eigValues[0]) > std::numeric_limits<double>::epsilon())
					outputValue = eigValues[2] / eigValues[0];
				break;
			case LINEA:
				if (std::abs(eigValues[0]) > std::numeric_limits<double>::epsilon())
					outputValue = (eigValues[0] - eigValues[1]) / eigValues[0];
				break;
			case PLANA:
				if (std::abs(eigValues[0]) > std::numeric_limits<double>::epsilon())
					outputValue = (eigValues[1] - eigValues[2]) / eigValues[0];
				break;
			case FOM:
			{
				double m1 = 0.0, m2 = 0.0;
				CCVector3 e2(eigVectors.m_values[0][1], eigVectors.m_values[1][1], eigVectors.m_values[2][1]);
				for (size_t i = 0; i < kNN; ++i)
				{
					double dotProd = (*pointsInNeighbourhood[i].point - queryPoint).dot(e2);
					m1 += dotProd;
					m2 += dotProd * dotProd;
				}
				outputValue = (m1 * m1) / m2;
			}
			case LINEF:
			case ORIENF:
				//can't compute these values yet!
				break;
			default:
				//impossible
				assert(false);
				break;
			}
		}
		else
		{
			return false;
		}
	}
	break;

	case DipAng:
	case DipDir:
	if (kNN >= 3)
	{
		CCLib::DgmOctreeReferenceCloud neighboursCloud(&pointsInNeighbourhood, static_cast<unsigned>(kNN));
		CCLib::Neighbourhood Z(&neighboursCloud);
		const CCVector3* N = Z.getLSPlaneNormal();
		if (N)
		{
			//force +Z
			CCVector3 Np = (N->z < 0 ? -PC_ONE * *N : *N);
			PointCoordinateType dip_deg, dipDir_deg;
			ccNormalVectors::ConvertNormalToDipAndDipDir(Np, dip_deg, dipDir_deg);
			outputValue = (type == DipAng ? dip_deg : dipDir_deg);
		}
		else
		{
			return false;
		}
	}
	break;

	case ROUGH:

	case NBPTS:
		outputValue = static_cast<double>(kNN);
		break;

	case CURV:

	case ZRANGE:
	case Zmax:
	case Zmin:
	if (kNN >= 2)
	{
		PointCoordinateType minZ, maxZ;
		minZ = maxZ = pointsInNeighbourhood[0].point->z;
		for (size_t i = 1; i < kNN; ++i)
		{
			if (minZ < pointsInNeighbourhood[i].point->z)
				minZ = pointsInNeighbourhood[i].point->z;
			else if (maxZ > pointsInNeighbourhood[i].point->z)
				maxZ = pointsInNeighbourhood[i].point->z;
		}

		if (type == ZRANGE)
		{
			outputValue = maxZ - minZ;
		}
		else if (type == Zmax)
		{
			outputValue = maxZ - queryPoint.z;
		}
		else if (type == Zmax)
		{
			outputValue = queryPoint.z - minZ;
		}
		else
		{
			//impossible
			assert(false);
		}
	}

	case ANISO:
	if (kNN >= 3)
	{
		CCLib::DgmOctreeReferenceCloud neighboursCloud(&pointsInNeighbourhood, static_cast<unsigned>(kNN));
		CCLib::Neighbourhood Z(&neighboursCloud);
		const CCVector3* G = Z.getGravityCenter();
		if (G)
		{
			double r = sqrt(pointsInNeighbourhood.back().squareDistd);
			if (r > std::numeric_limits<double>::epsilon())
			{
				double d = (queryPoint - *G).normd();
				//Ratio of distance to center of mass and radius of sphere
				outputValue = d / r;
			}
		}
		else
		{
			return false;
		}
	}
	break;

	default:
	{
		ccLog::Warning("Unhandled STAT measure");
		assert(false);
		return false;
	}

	}

	return true;
}

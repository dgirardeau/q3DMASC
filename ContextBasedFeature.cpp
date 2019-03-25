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

#include "ContextBasedFeature.h"

//Qt
#include <QMutex>

using namespace masc;

bool ContextBasedFeature::prepare(	const CorePoints& corePoints,
									QString& error,
									CCLib::GenericProgressCallback* progressCb/*=nullptr*/,
									SFCollector* generatedScalarFields/*=nullptr*/)
{
	if (!cloud1 || !corePoints.cloud)
	{
		//invalid input
		assert(false);
		error = "internal error (no input core points)";
		return false;
	}

	if (!cloud2)
	{
		//invalid input
		assert(false);
		error = "internal error (no contextual cloud)";
		return false;
	}

	if (!checkValidity(corePoints.role, error))
	{
		assert(false);
		return false;
	}

	//build the final SF name
	QString typeStr = ToString(type);
	QString resultSFName = typeStr + "_" + cloud1Label + "_" + cloud2Label + "_" + QString::number(ctxClassLabel);
	if (scaled())
	{
		resultSFName += "@" + QString::number(scale);
	}
	else
	{
		resultSFName += "@kNN=" + QString::number(kNN);
	}

	//and the scalar field
	assert(!sf);
	sf = PrepareSF(corePoints.cloud, qPrintable(resultSFName), generatedScalarFields);
	if (!sf)
	{
		error = QString("Failed to prepare scalar %1 @ scale %2").arg(resultSFName).arg(scale);
		return false;
	}
	sourceName = sf->getName();

	if (!scaled()) //with 'kNN' neighbors, we can compute the values right away
	{
		//get the octree
		ccOctree::Shared octree = cloud2->getOctree();
		if (!octree)
		{
			ccLog::Print(QString("Computing octree of cloud %1 (%2 points)").arg(cloud2->getName()).arg(cloud2->size()));
			octree = cloud2->computeOctree(progressCb);
			if (!octree)
			{
				error = "Failed to compute octree (not enough memory?)";
				return false;
			}
		}

		//now extract the neighborhoods
		unsigned char octreeLevel = octree->findBestLevelForAGivenPopulationPerCell(static_cast<unsigned>(std::max(3, kNN)));
		ccLog::Print(QString("[Initial octree level] level = %1").arg(octreeLevel));

		unsigned pointCount = corePoints.size();
		QString logMessage = QString("Computing %1 on cloud %2 with context cloud %3\n(core points: %4)").arg(typeStr).arg(corePoints.cloud->getName()).arg(cloud2Label).arg(pointCount);
		if (progressCb)
		{
			progressCb->setMethodTitle(qPrintable("Compute " + typeStr));
			progressCb->setInfo(qPrintable(logMessage));
		}
		ccLog::Print(logMessage);
		CCLib::NormalizedProgress nProgress(progressCb, pointCount);

		QMutex mutex;
		bool error = false;
		double meanNeighborhoodSize = 0;
		int tenth = pointCount / 10;
#ifndef _DEBUG
#if defined(_OPENMP)
#pragma omp parallel for
#endif
#endif
		for (int i = 0; i < static_cast<int>(pointCount); ++i)
		{
			const CCVector3* P = corePoints.cloud->getPoint(i);
			CCLib::ReferenceCloud Yk(cloud2);
			double maxSquareDist = 0;

			ScalarType s = NAN_VALUE;
			
			int neighborhoodSize = 0;
			if (octree->findPointNeighbourhood(P, &Yk, static_cast<unsigned>(kNN), octreeLevel, maxSquareDist, 0, &neighborhoodSize) >= static_cast<unsigned>(kNN))
			{
				CCVector3d sumQ(0, 0, 0);
				for (int k = 0; k < kNN; ++k)
				{
					sumQ += CCVector3d::fromArray(Yk.getPoint(k)->u);
				}

				switch (type)
				{
				case DZ:
					s = static_cast<ScalarType>(P->z - sumQ.z / kNN);
					break;
				case DH:
					s = static_cast<ScalarType>(sqrt(pow(P->x - sumQ.x / kNN, 2.0) + pow(P->y - sumQ.y / kNN, 2.0)));
					break;
				}

				if (i && (i % tenth) == 0)
				{
					double density = meanNeighborhoodSize / tenth;
					if (density < 1.1)
					{
						if (octreeLevel + 1 < CCLib::DgmOctree::MAX_OCTREE_LEVEL)
							++octreeLevel;
					}
					else while (density > 2.9)
					{
						if (octreeLevel <= 5)
							break;
						--octreeLevel;
						density /= 2.0;
					}
					ccLog::Print(QString("[Adaptative octree level] Mean neighborhood size: %1 --> new level = %2").arg(meanNeighborhoodSize / tenth).arg(octreeLevel));
					meanNeighborhoodSize = 0;
				}
				else
				{
					meanNeighborhoodSize += neighborhoodSize;
				}
			}

			sf->setValue(i, s);

			if (progressCb)
			{
				mutex.lock();
				bool cancelled = !nProgress.oneStep();
				mutex.unlock();
				if (cancelled)
				{
					//process cancelled by the user
					ccLog::Warning("Process cancelled");
					error = true;
					break;
				}
			}
		}

		sf->computeMinAndMax();

		if (progressCb)
		{
			progressCb->stop();
		}

		if (error)
		{
			return false;
		}
	}

	return true;
}

bool ContextBasedFeature::computeValue(CCLib::DgmOctree::NeighboursSet& pointsInNeighbourhood, const CCVector3& queryPoint, ScalarType& outputValue) const
{
	CCVector3d sumQ(0, 0, 0);
	for (CCLib::DgmOctree::PointDescriptor& Pd : pointsInNeighbourhood)
	{
		sumQ += CCVector3d::fromArray(Pd.point->u);
	}

	switch (type)
	{
	case DZ:
		outputValue = static_cast<ScalarType>(queryPoint.z - sumQ.z / kNN);
		break;
	case DH:
		outputValue = static_cast<ScalarType>(sqrt(pow(queryPoint.x - sumQ.x / kNN, 2.0) + pow(queryPoint.y - sumQ.y / kNN, 2.0)));
		break;
	default:
		assert(false);
		outputValue = NAN_VALUE;
		return false;
	}

	return true;
}


bool ContextBasedFeature::finish(const CorePoints& corePoints, QString& error)
{
	if (!corePoints.cloud)
	{
		//invalid input
		assert(false);
		error = "internal error (no input core points)";
		return false;
	}

	bool success = true;

	if (sf)
	{
		sf->computeMinAndMax();

		//update display
		//if (corePoints.cloud->getDisplay())
		{
			int sfIndex = corePoints.cloud->getScalarFieldIndexByName(sf->getName());
			corePoints.cloud->setCurrentDisplayedScalarField(sfIndex);
			//corePoints.cloud->getDisplay()->redraw();
			//QCoreApplication::processEvents();
		}
	}

	return success;
}

bool ContextBasedFeature::checkValidity(QString corePointRole, QString &error) const
{
	if (!Feature::checkValidity(corePointRole, error))
	{
		return false;
	}

	if (type == Invalid)
	{
		assert(false);
		error = "invalid feature type";
		return false;
	}

	unsigned char cloudCount = (cloud1 ? (cloud2 ? 2 : 1) : 0);
	if (cloudCount < 2)
	{
		error = "at least two clouds are required to compute context-based features";
		return false;
	}

	if (!scaled() && kNN < 1)
	{
		error = QString("invalid kNN value for a scale-less context-based feature (%1)").arg(kNN);
		return false;
	}

	return true;
}

QString ContextBasedFeature::toString() const
{
	//use the default keyword + number of neighbors + the scale + the context class
	QString str = ToString(type);
	if (!scaled())
	{
		if (kNN != 1)
			str += QString::number(kNN);
		str += "_SC0";
	}
	else
	{
		str += "_SC" + QString::number(scale);
	}

	str += "_" + cloud1Label + "_" + cloud2Label + "_" + QString::number(ctxClassLabel);

	return str;
}

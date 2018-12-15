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

//Local
#include "FeaturesInterface.h"

namespace masc
{
	//! Neighborhood-based feature
	struct NeighborhoodFeature : public Feature
	{
	public: //NeighborhoodFeatureType

		typedef QSharedPointer<NeighborhoodFeature> Shared;

		enum NeighborhoodFeatureType
		{
			Invalid = 0
			, PCA1
			, PCA2
			, SPHER
			, LINEA
			, PLANA
			, DipAng
			, DipDir
			, ROUGH
			, NBPTS
			, CURV
			, ZRANGE
			, Zmax
			, Zmin
			, ANISO
			, FOM
			//, LINEF
			//, ORIENF
		};

		static QString ToString(NeighborhoodFeatureType type)
		{
			switch (type)
			{
			case Invalid:
				return "Invalid";
			case PCA1:
				return "PCA1";
			case PCA2:
				return "PCA2";
			case SPHER:
				return "SPHER";
			case LINEA:
				return "LINEA";
			case PLANA:
				return "PLANA";
			case DipAng:
				return "DipAng";
			case DipDir:
				return "DipDir";
			case ROUGH:
				return "ROUGH";
			case NBPTS:
				return "NBPTS";
			case CURV:
				return "CURV";
			case ZRANGE:
				return "ZRANGE";
			case Zmax:
				return "Zmax";
			case Zmin:
				return "Zmin";
			case ANISO:
				return "ANISO";
			case FOM:
				return "FOM";
			//case LINEF:
			//	return "LINEF";
			//case ORIENF:
			//	return "ORIENF";
			default:
				assert(false);
				break;
			}
			return "Invalid";
		}

		static inline NeighborhoodFeatureType FromString(const QString& token) { return FromUpperString(token.toUpper()); }
		static NeighborhoodFeatureType FromUpperString(const QString& token)
		{
			if (token == "PCA1")
				return PCA1;
			else if (token == "PCA2")
				return PCA2;
			else if (token == "SPHER")
				return SPHER;
			else if (token == "LINEA")
				return LINEA;
			else if (token == "PLANA")
				return PLANA;
			else if (token == "DipAng")
				return DipAng;
			else if (token == "DipDir")
				return DipDir;
			else if (token == "ROUGH")
				return ROUGH;
			else if (token == "NBPTS")
				return NBPTS;
			else if (token == "CURV")
				return CURV;
			else if (token == "ZRANGE")
				return ZRANGE;
			else if (token == "Zmax")
				return Zmax;
			else if (token == "Zmin")
				return Zmin;
			else if (token == "ANISO")
				return ANISO;
			else if (token == "FOM")
				return FOM;
			//else if (token == "LINEF")
			//	return LINEF;
			//else if (token == "ORIENF")
			//	return ORIENF;

			return Invalid;
		}

	public: //methods

		//! Default constructor
		NeighborhoodFeature(NeighborhoodFeatureType p_type)
			: type(p_type)
			, sf1(nullptr)
			, sf2(nullptr)
			, keepSF2(false)
		{
		}

		//inherited from Feature
		virtual Type getType() const override { return Type::NeighborhoodFeature; }
		virtual Feature::Shared clone() const override { return Feature::Shared(new NeighborhoodFeature(*this)); }
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr) override;
		virtual bool finish(const CorePoints& corePoints, QString& error) override;
		virtual bool checkValidity(QString &error) const override;
		virtual QString toString() const override;

		//! Compute the feature value on a set of points
		bool computeValue(CCLib::DgmOctree::NeighboursSet& pointsInNeighbourhood, const CCVector3& queryPoint, double& outputValue) const;

	public: //members

		//! Neighborhood feature type
		/** \warning different from the feature type
		**/
		NeighborhoodFeatureType type;

		//! Feature values
		CCLib::ScalarField *sf1, *sf2;
		bool keepSF2;
	};
}
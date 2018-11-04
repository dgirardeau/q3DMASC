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
#include "CorePoints.h"
#include "ScalarFieldWrappers.h"

//Qt
#include <QString>
#include <QSharedPointer>

class ccPointCloud;

namespace masc
{
	//! Point feature
	struct PointFeature : public Feature
	{
	public:	//PointFeatureType

		enum PointFeatureType
		{
			Invalid = 0
			, Intensity
			, X
			, Y
			, Z
			, NbRet
			, RetNb
			, EchoRat
			, R
			, G
			, B
			, NIR
			, DipAng
			, DipDir
			, M3C2
			, PCV
			, SF
		};

		static QString ToString(PointFeatureType type)
		{
			switch (type)
			{
			case Invalid:
				return "Invalid";
			case Intensity:
				return "INT";
			case X:
				return "X";
			case Y:
				return "Y";
			case Z:
				return "Z";
			case NbRet:
				return "NbRet";
			case RetNb:
				return "RetNb";
			case EchoRat:
				return "EchoRat";
			case R:
				return "R";
			case G:
				return "G";
			case B:
				return "B";
			case NIR:
				return "NIR";
			case DipAng:
				return "DipAng";
			case DipDir:
				return "DipDir";
			case M3C2:
				return "M3C2";
			case PCV:
				return "PCV";
			case SF:
				return "SF#";
			default:
				assert(false);
				break;
			}
			return "Invalid";
		}

		static inline PointFeatureType FromString(const QString& token) { return FromUpperString(token.toUpper()); }
		static PointFeatureType FromUpperString(const QString& token)
		{
			if (token == "INT")
				return Intensity;
			else if (token == "X")
				return X;
			else if (token == "Y")
				return Y;
			else if (token == "Z")
				return Z;
			else if (token == "NBRET")
				return NbRet;
			else if (token == "RETNB")
				return RetNb;
			else if (token == "ECHORAT")
				return EchoRat;
			else if (token == "R")
				return R;
			else if (token == "G")
				return G;
			else if (token == "B")
				return B;
			else if (token == "NIR")
				return NIR;
			else if (token == "NORMDIPANG")
				return DipAng;
			else if (token == "NORMDIPDIR")
				return DipDir;
			else if (token == "M3C2")
				return M3C2;
			else if (token == "PCV")
				return PCV;
			else if (token.startsWith("SF"))
				return SF;

			return Invalid;
		}

	public:	//methods

		//! Default constructor
		PointFeature(PointFeatureType p_type)
			: type(p_type)
			, sourceSFIndex(-1)
		{
			//auomatically set the right source for specific features
			switch (type)
			{
			case X:
				source = DimX;
				sourceName = "X";
				break;
			case Y:
				source = DimY;
				sourceName = "Y";
				break;
			case Z:
				source = DimZ;
				sourceName = "Z";
				break;
			case R:
				source = Red;
				sourceName = "Red";
				break;
			case G:
				source = Green;
				sourceName = "Green";
				break;
			case B:
				source = Blue;
				sourceName = "Blue";
				break;
			default:
				source = ScalarField;
				//sourceName --> TBD later
				break;
			}
		}

		//! Returns the feature type
		virtual Type getType() const override { return Type::PointFeature; }
		//! Clones this feature
		virtual Feature::Shared clone() const { return Feature::Shared(new PointFeature(*this)); }
		//! Prepares the feature (compute the scalar field, etc.)
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr);
		//! Returns the descriptor for this particular feature
		virtual QString toString() const override
		{
			//default keyword otherwise
			QString prefix = ToString(type);

			//special case for the 'SF' type
			if (type == SF)
			{
				//'SF#' + sf index
				prefix += QString::number(sourceSFIndex);
			}

			//Point features always have a scale equal to 0 by definition
			return prefix + "_SC0";
		}

	protected: //methods

		//! Returns the 'source' field from a given cloud
		QSharedPointer<IScalarFieldWrapper> retrieveField(ccPointCloud* cloud, QString& error);

	public:	//members

		//! Point feature type
		/** \warning different from the feature type
		**/
		PointFeatureType type;

		//! Source scalar field index (if the feature source is 'ScalarField')
		int sourceSFIndex;
	};

	//! Neighborhood-based feature
	struct NeighborhoodFeature : public Feature
	{
	public: //NeighborhoodFeatureType

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
			, LINEF
			, ORIENF
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
			case LINEF:
				return "LINEF";
			case ORIENF:
				return "ORIENF";
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
			else if (token == "LINEF")
				return LINEF;
			else if (token == "ORIENF")
				return ORIENF;

			assert(false);
			return Invalid;
		}

	public: //methods

		//! Default constructor
		NeighborhoodFeature(NeighborhoodFeatureType p_type)
			: type(p_type)
		{
		}

		//! Returns the feature type
		virtual Type getType() const override { return Type::NeighborhoodFeature; }
		//! Clones this feature
		virtual Feature::Shared clone() const { return Feature::Shared(new NeighborhoodFeature(*this)); }
		//! Prepares the feature (compute the scalar field, etc.)
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr);
		//! Returns the descriptor for this particular feature
		virtual QString toString() const override
		{
			//use the default keyword + the scale
			return ToString(type) + "_SC" + QString::number(scale);
		}

	public: //members

		//! Neighborhood feature type
		/** \warning different from the feature type
		**/
		NeighborhoodFeatureType type;
	};

	//! Context-based feature
	struct ContextBasedFeature : public Feature
	{
	public: //ContextBasedFeatureType

		enum ContextBasedFeatureType
		{
			Invalid = 0
			, DZ
			, DH
		};

		static QString ToString(ContextBasedFeatureType type)
		{
			switch (type)
			{
			case Invalid:
				return "Invalid";
			case DZ:
				return "DZ";
			case DH:
				return "DH";
			default:
				assert(false);
				break;
			}
			return "Invalid";
		}

		static inline ContextBasedFeatureType FromString(const QString& token) { return FromUpperString(token.toUpper()); }
		static ContextBasedFeatureType FromUpperString(const QString& token)
		{
			if (token.startsWith("DZ"))
				return DZ;
			else if (token.startsWith("DH"))
				return DH;

			assert(false);
			return Invalid;
		}

	public: //methods

		//! Default constructor
		ContextBasedFeature(ContextBasedFeatureType p_type,
							int p_kNN = 0,
							double p_scale = std::numeric_limits<double>::quiet_NaN(),
							int p_ctxClassLabel = 0)
			: type(p_type)
			, kNN(p_kNN)
			, ctxClassLabel(p_ctxClassLabel)
		{
			scale = p_scale;
		}

		//! Returns the feature type
		virtual Type getType() const override { return Type::ContextBasedFeature; }
		//! Clones this feature
		virtual Feature::Shared clone() const { return Feature::Shared(new ContextBasedFeature(*this)); }
		//! Prepares the feature (compute the scalar field, etc.)
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr);
		//! Returns the descriptor for this particular feature
		virtual QString toString() const override
		{
			//use the default keyword + number of neighbors + the scale + the context class
			return ToString(type) + QString::number(kNN) + "_SC" + QString::number(scale) + "CTX" + QString::number(ctxClassLabel);
		}

		//! Neighborhood feature type
		/** \warning different from the feature type
		**/
		ContextBasedFeatureType type;

		//Number of neighbors
		int kNN;
		//! Context class (label)
		int ctxClassLabel;
	};

	//! Dual-cloud feature
	struct DualCloudFeature : public Feature
	{
	public: //DualCloudFeatureType

		enum DualCloudFeatureType
		{
			Invalid = 0
			, IDIFF
		};

		static QString ToString(DualCloudFeatureType type)
		{
			switch (type)
			{
			case Invalid:
				return "Invalid";
			case IDIFF:
				return "IDIFF";
			default:
				assert(false);
				break;
			}
			return "Invalid";
		}

		static inline DualCloudFeatureType FromString(const QString& token) { return FromUpperString(token.toUpper()); }
		static DualCloudFeatureType FromUpperString(const QString& token)
		{
			if (token == "IDIFF")
				return IDIFF;

			assert(false);
			return Invalid;
		}

	public: //methods

		//! Default constructor
		DualCloudFeature(DualCloudFeatureType p_type)
			: type(p_type)
		{}

		//! Returns the feature type
		virtual Type getType() const override { return Type::DualCloudFeature; }
		//! Clones this feature
		virtual Feature::Shared clone() const { return Feature::Shared(new DualCloudFeature(*this)); }
		//! Prepares the feature (compute the scalar field, etc.)
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr);
		//! Returns the descriptor for this particular feature
		virtual QString toString() const override
		{
			//use the default keyword + "_SC" + the scale
			return ToString(type) + "_SC" + QString::number(scale);
		}

		//! Dual-cloud feature type
		/** \warning different from the feature type
		**/
		DualCloudFeatureType type;
	};
}
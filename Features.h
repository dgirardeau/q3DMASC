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

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QString>
#include <QSharedPointer>

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

		assert(false);
		return Invalid;
	}

public:	//methods

	//! Returns the feature type
	virtual Type getType() override { return Type::PointFeature; }

	//! Default constructor
	PointFeature(PointFeatureType p_type, ccPointCloud* p_cloud = nullptr)
		: Feature(p_cloud)
		, type(p_type)
	{
		//auomatically set the right source for specific features
		switch (type)
		{
		case Z:
			source = DimZ;
			sourceName = "X";
			break;
		case Z:
			source = DimZ;
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

	//! Returns the descriptor for this particular feature
	virtual QString toString() const override
	{
		//default keyword otherwise
		QString prefix = ToString(type);

		//special case for the 'SF' type
		if (type == SF)
		{
			if (!cloud)
			{
				//invalid cloud
				assert(false);
				return ToString(Invalid);
			}

			int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sourceName));
			if (sfIdx < 0)
			{
				//scalar field is not present?!
				assert(false);
				return ToString(Invalid);
			}

			//'SF#' + sf index
			prefix += QString::number(sfIdx);
		}

		//Point features always have a scale equal to 0 by definition
		return prefix + "_SC0";
	}

public:	//members

	//! Point feature type
	/** \warning different from the feature type
	**/
	PointFeatureType type;
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

	//! Returns the feature type
	virtual Type getType() override { return Type::NeighborhoodFeature; }

	//! Default constructor
	NeighborhoodFeature(NeighborhoodFeatureType p_type, ccPointCloud* p_cloud = nullptr)
		: Feature(p_cloud)
		, type(p_type)
	{
	}

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

//! Feature based on two clouds (e.g. for ContextBasedFeature and DualCloudFeature)
struct TwoCloudsFeature : public Feature
{
	//! Default constructor
	TwoCloudsFeature(ccPointCloud* p_cloud = nullptr, ccPointCloud* p_otherCloud = nullptr, double p_scale = std::numeric_limits<double>::quiet_NaN())
		: Feature(p_cloud, p_scale)
		, otherCloud(p_otherCloud)
	{
		assert(otherCloud);
	}
	
	//! Other cloud
	ccPointCloud* otherCloud;
};

//! Context-based feature
struct ContextBasedFeature : public TwoCloudsFeature
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

	//! Returns the feature type
	virtual Type getType() override { return Type::ContextBasedFeature; }

	//! Default constructor
	ContextBasedFeature(ContextBasedFeatureType p_type,
						ccPointCloud* p_cloud = nullptr,
						ccPointCloud* p_otherCloud = nullptr,
						int p_kNN = 0,
						double p_scale = std::numeric_limits<double>::quiet_NaN(),
						int p_ctxClassLabel = 0)
		: TwoCloudsFeature(p_cloud, p_otherCloud, p_scale)
		, type(p_type)
		, kNN(p_kNN)
		, ctxClassLabel(p_ctxClassLabel)
	{}

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
struct DualCloudFeature : public TwoCloudsFeature
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

	//! Returns the feature type
	virtual Type getType() override { return Type::DualCloudFeature; }

	//! Default constructor
	DualCloudFeature(	DualCloudFeatureType p_type,
						ccPointCloud* p_cloud = nullptr,
						ccPointCloud* p_otherCloud = nullptr,
						double p_scale = std::numeric_limits<double>::quiet_NaN()
					)
		: TwoCloudsFeature(p_cloud, p_otherCloud, p_scale)
		, type(p_type)
	{}

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

struct Scales
{
	typedef QSharedPointer<Scales> Shared;
	std::vector<double> values;
}; 

struct FeatureRule
{
	typedef QSharedPointer<FeatureRule> Shared;
	typedef std::vector<Shared> Set;

	enum Stat
	{
		NO_STAT,
		MEAN,
		MODE, //number with the highest frequency
		STD,
		RANGE,
		SKEW //(SKEW = (MEAN - MODE)/STD)
	};
	Stat stat = NO_STAT; //only considered if a scale is defined

	static QString StatToString(Stat stat)
	{
		switch (stat)
		{
		case MEAN:
			return "MEAN";
		case MODE:
			return "MODE";
		case STD:
			return "STD";
		case RANGE:
			return "RANGE";
		case SKEW:
			return "SKEW";
		default:
			break;
		};
		return QString();
	}

	enum Operation
	{
		NO_OPERATION, MINUS, PLUS, DIVIDE, MULTIPLY
	};
	Operation op = NO_OPERATION; //only considered if 2 clouds are defined

	static QString OpToString(Operation op)
	{
		switch (op)
		{
		case MINUS:
			return "MINUS";
		case PLUS:
			return "PLUS";
		case DIVIDE:
			return "DIVIDE";
		case MULTIPLY:
			return "MULTIPLY";
		default:
			break;
		};
		return QString();
	}

	Scales::Shared scales;
	Feature::Shared feature;
	ccPointCloud* cloud1 = nullptr;
	ccPointCloud* cloud2 = nullptr;

	//! Source scalar field index (if the feature source is 'ScalarField')
	int sourceSFIndex = -1;

	//! Checks the rule validity
	bool checkValidity(QString &error) const
	{
		int cloudCount = (cloud1 ? (cloud2 ? 2 : 1) : 0);

		if (feature == nullptr) //feature object should have already been instantiated
		{
			error = "feature rule has no associated feature";
			return false;
		}
		if (scales != nullptr && scales->values.empty())
		{
			error = "invalid scales definition";
			return false;
		}
		if (stat != FeatureRule::NO_STAT)
		{
			if (feature->getType() != Feature::Type::PointFeature)
			{
				error = "stat. measures can only be defined on Point features";
				return false;
			}
			if (!scales)
			{
				error = "stat. measures need at least one scale to be defined";
				return false;
			}
		}
		if (stat != FeatureRule::NO_OPERATION)
		{
			if (feature->getType() == Feature::Type::DualCloudFeature)
			{
				error = "math operation can't be defined on dual-cloud features";
				return false;
			}
			if (cloudCount < 2)
			{
				error = "at least two clouds are required to apply math operations";
				return false;
			}
		}
		if (feature->getType() == Feature::Type::DualCloudFeature || feature->getType() == Feature::Type::ContextBasedFeature)
		{
			if (cloudCount < 2)
			{
				error = "at least two clouds are required to compute dual-cloud or context-based features";
				return false;
			}
		}
		
		return true;
	}
};

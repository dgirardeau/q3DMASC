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

//qCC_db
#include <ccPointCloud.h>
//CCLib
#include <ScalarField.h>

//Qt
#include <QString>
#include <QSharedPointer>

class IScalarFieldWrapper
{
public:
	virtual double pointValue(unsigned index) const = 0;
	virtual bool isValid() const = 0;
};

class ScalarFieldWrapper : public IScalarFieldWrapper
{
public:
	ScalarFieldWrapper(CCLib::ScalarField* sf)
		: m_sf(sf)
	{}

	virtual inline double pointValue(unsigned index) const override { return m_sf->at(index); }
	virtual inline bool isValid() const { return m_sf != nullptr; }

protected:
	CCLib::ScalarField* m_sf;
};

class DimScalarFieldWrapper : public IScalarFieldWrapper
{
public:
	enum Dim { DimX = 0, DimY = 1, DimZ = 2 };
	
	DimScalarFieldWrapper(ccPointCloud* cloud, Dim dim)
		: m_cloud(cloud)
		, m_dim(dim)
	{}

	virtual inline double pointValue(unsigned index) const override { return m_cloud->getPoint(index)->u[m_dim]; }
	virtual inline bool isValid() const { return m_cloud != nullptr; }

protected:
	ccPointCloud* m_cloud;
	Dim m_dim;
};

class ColorScalarFieldWrapper : public IScalarFieldWrapper
{
public:
	enum Band { Red = 0, Green = 1, Blue = 2 };

	ColorScalarFieldWrapper(ccPointCloud* cloud, Band band)
		: m_cloud(cloud)
		, m_band(band)
	{}

	virtual inline double pointValue(unsigned index) const override { return m_cloud->getPointColor(index).rgb[m_band]; }
	virtual inline bool isValid() const { return m_cloud != nullptr && m_cloud->hasColors(); }

protected:
	ccPointCloud* m_cloud;
	Band m_band;
};

struct RTParams
{
	int maxDepth = 25; //To be left as a parameter of the training plugin (default 25)
	int minSampleCount = 1; //To be left as a parameter of the training plugin (default 1)
	int maxCategories = 0; //Normally not important as there’s no categorical variable
	const bool calcVarImportance = true; //Must be true
	int activeVarCount = 0; //USE 0 as the default parameter (works best)
	int maxTreeCount = 100; //Left as a parameter of the training plugin (default: 100)

	float testDataRatio = 0.2f; //percentage of test data
};

//! Generic feature descriptor
struct Feature
{
	//!Shared type
	typedef QSharedPointer<Feature> Shared;

	//! Feature type
	enum class Type
	{
		PointFeature,			/*!< Point features (scalar field, etc.) */
		NeighborhoodFeature,	/*!< Neighborhood based features for a given scale */
		ContextBasedFeature,	/*!< Contextual based features */
		DualCloudFeature		/*!< Dual Cloud features: requires 2 point clouds */
	};

	//! Returns the type (must be reimplemented by child struct)
	virtual Type getType() = 0;

	//! Sources of values for this feature
	enum Source
	{
		ScalarField, DimX, DimY, DimZ, Red, Green, Blue
	};

	//! Default constructor
	Feature(ccPointCloud* p_cloud, Source p_source, QString p_sourceName)
		: cloud(p_cloud)
		, source(p_source)
		, sourceName(p_sourceName)
	{
		assert(cloud);
	}

	//! Associated cloud
	ccPointCloud* cloud;
	//! Values source
	Source source;
	//! Feature source name (mandatory for scalar fields)
	QString sourceName;
};

//! Point feature
struct PointFeature : public Feature
{
public:	//PointFeatureType

	enum PointFeatureType
	{
		Invalid = 0
		, Intensity
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

	static PointFeatureType FromString(QString token)
	{
		token = token.toUpper();
		if (token == "INT")
			return Intensity;
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
		else if (token == "DIPANG")
			return DipAng;
		else if (token == "DIPDIR")
			return DipDir;
		else if (token == "M3C2")
			return M3C2;
		else if (token == "PCV")
			return PCV;
		else if (token == "SF#")
			return SF;

		assert(false);
		return Invalid;
	}

public:	//methods

	//! Returns the feature type
	virtual Type getType() override { return Type::PointFeature; }

	//! Default constructor
	PointFeature(ccPointCloud* p_cloud, PointFeatureType p_type, Source p_source, QString p_name)
		: Feature(p_cloud, p_source, p_name)
		, type(p_type)
	{}

	//! Returns the descriptor for this particular feature
	QString toString() const
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

	static NeighborhoodFeatureType FromString(QString token)
	{
		token = token.toUpper();

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
	NeighborhoodFeature(ccPointCloud* p_cloud, NeighborhoodFeatureType p_type, Source p_source, QString p_name, double p_scale = std::numeric_limits<double>::quiet_NaN())
		: Feature(p_cloud, p_source, p_name)
		, scale(p_scale)
		, type(p_type)
	{}

	//! Returns the descriptor for this particular feature
	QString toString() const
	{
		//use the default keyword + the scale
		return ToString(type) + "_SC" + QString::number(scale);
	}

public: //members

	//! Neighborhood scale (diameter)
	double scale;
	
	//! Neighborhood feature type
	/** \warning different from the feature type
	**/
	NeighborhoodFeatureType type;
};

//! Feature based on two clouds (e.g. for ContextBasedFeature and DualCloudFeature)
struct TwoCloudsFeature : public Feature
{
	//! Default constructor
	TwoCloudsFeature(ccPointCloud* p_cloud, ccPointCloud* p_otherCloud, Source p_source, QString p_name, double p_scale = std::numeric_limits<double>::quiet_NaN())
		: Feature(p_cloud, p_source, p_name)
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

	static ContextBasedFeatureType FromString(QString token)
	{
		token = token.toUpper();

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
	ContextBasedFeature(ccPointCloud* p_cloud,
						ccPointCloud* p_otherCloud,
						ContextBasedFeatureType p_type,
						int p_kNN,
						double p_scale,
						int p_ctxClassLabel,
						Source p_source,
						QString p_name)
		: TwoCloudsFeature(p_cloud, p_otherCloud, p_source, p_name)
		, type(p_type)
		, kNN(p_kNN)
		, scale(p_scale)
		, ctxClassLabel(p_ctxClassLabel)
	{}

	//! Returns the descriptor for this particular feature
	QString toString() const
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
	//! Scale (optional)
	double scale;
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

	static DualCloudFeatureType FromString(QString token)
	{
		token = token.toUpper();

		if (token == "IDIFF")
			return IDIFF;

		assert(false);
		return Invalid;
	}

public: //methods

	//! Returns the feature type
	virtual Type getType() override { return Type::DualCloudFeature; }

	//! Default constructor
	DualCloudFeature(	ccPointCloud* p_cloud,
						ccPointCloud* p_otherCloud,
						DualCloudFeatureType p_type,
						double p_scale,
						Source p_source,
						QString p_name
	)
		: TwoCloudsFeature(p_cloud, p_otherCloud, p_source, p_name)
		, type(p_type)
		, scale(p_scale)
	{}

	//! Returns the descriptor for this particular feature
	QString toString() const
	{
		//use the default keyword + "_SC" + the scale
		return ToString(type) + "_SC" + QString::number(scale);
	}

	//! Dual-cloud feature type
	/** \warning different from the feature type
	**/
	DualCloudFeatureType type;

	//! Scale (optional)
	double scale;
};

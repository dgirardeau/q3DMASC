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
#include "ScalarFieldWrappers.h"

//Qt
#include <QSharedPointer>

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
				return "NormDip";
			case DipDir:
				return "NormDipDir";
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
			else if (token == "NORMDIP")
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
		virtual Feature::Shared clone() const override { return Feature::Shared(new PointFeature(*this)); }
		//! Prepares the feature (compute the scalar field, etc.)
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr) override;
		//! Checks the feature definition validity
		virtual bool checkValidity(QString &error) const override;
		//! Returns the descriptor for this particular feature
		virtual QString toString() const override;

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
}

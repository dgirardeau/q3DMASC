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

		//inherited from Feature
		virtual Type getType() const override { return Type::ContextBasedFeature; }
		virtual Feature::Shared clone() const override { return Feature::Shared(new ContextBasedFeature(*this)); }
		virtual bool prepare(const CorePoints& corePoints, QString& error, CCLib::GenericProgressCallback* progressCb = nullptr) override;
		virtual bool checkValidity(QString &error) const override;
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
}
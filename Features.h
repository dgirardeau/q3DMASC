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
	int maxCategories = 0; //Normally not important as there�s no categorical variable
	const bool calcVarImportance = true; //Must be true
	int activeVarCount = 0; //USE 0 as the default parameter (works best)
	int maxTreeCount = 100; //Left as a parameter of the training plugin (default: 100)

	float testDataRatio = 0.2f; //percentage of test data
};

struct Feature
{
	enum Source
	{
		ScalarField, DimX, DimY, DimZ, Red, Green, Blue
	};

	Feature(Source p_source, QString p_name)
		: source(p_source)
		, name(p_name)
	{}

	Source source;
	QString name; //especially for scalar fields
};

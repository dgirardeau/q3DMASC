#pragma once

#include <QWidget>
#include <set>

#include "CCTypes.h"

#include <opencv2/core/mat.hpp>

namespace Ui {
class ConfusionMatrix;
}

class ConfusionMatrix : public QWidget
{
	Q_OBJECT

public:
	enum metrics
	{
		PRECISION = 0,
		RECALL = 1,
		F1_SCORE = 2
	};

	explicit ConfusionMatrix(const std::vector<ScalarType>& actual, const std::vector<ScalarType>& predicted, QWidget *parent = nullptr);
	~ConfusionMatrix() override;

	void computePrecisionRecallF1Score(cv::Mat& matrix, cv::Mat& precisionRecallF1Score, cv::Mat &vec_TP_FN);
	float computeOverallAccuracy(cv::Mat& matrix);
	void compute(const std::vector<ScalarType> &actual, const std::vector<ScalarType> &predicted);
	void setSessionRun(QString session, int run);
	float m_overallAccuracy;
	bool save(QString filePath);

private:
	std::set<ScalarType> classes;
	int nbClasses;
	Ui::ConfusionMatrix *ui;
	cv::Mat confusionMatrix;
	cv::Mat precisionRecallF1Score;
	std::vector<ScalarType> class_numbers;
};

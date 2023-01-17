#ifndef CONFUSIONMATRIX_H
#define CONFUSIONMATRIX_H

#include <QWidget>

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

	explicit ConfusionMatrix(std::vector<ScalarType>& actual, std::vector<ScalarType>& predicted, QWidget *parent = nullptr);
	~ConfusionMatrix();

	void computePrecisionRecallF1Score(cv::Mat& matrix, cv::Mat& precisionRecallF1Score);
	float computeOverallAccuracy(cv::Mat& matrix);
	void compute(std::vector<ScalarType>& actual, std::vector<ScalarType>& predicted);

private:
	Ui::ConfusionMatrix *ui;
};

#endif // CONFUSIONMATRIX_H

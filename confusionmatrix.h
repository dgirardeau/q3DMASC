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
	explicit ConfusionMatrix(QWidget *parent = nullptr);
	~ConfusionMatrix();

	void compute_precision_recall_f1_score(cv::Mat& confusion_matrix, cv::Mat &precision_recall_f1_score);
	void compute(std::vector<ScalarType>& reality, std::vector<ScalarType>& predicted);

private:
	Ui::ConfusionMatrix *ui;
};

#endif // CONFUSIONMATRIX_H

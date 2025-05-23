#pragma once

#include <QWidget>
#include <set>

#include <GenericDistribution.h>

#include <ccMainAppInterface.h>

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

	explicit ConfusionMatrix(	const CCCoreLib::GenericDistribution::ScalarContainer& actual,
								const CCCoreLib::GenericDistribution::ScalarContainer& predicted );
	~ConfusionMatrix() override;

	void computePrecisionRecallF1Score(cv::Mat& matrix, cv::Mat& precisionRecallF1Score, cv::Mat &vec_TP_FN);
	float computeOverallAccuracy(cv::Mat& matrix);
	void compute(	const CCCoreLib::GenericDistribution::ScalarContainer& actual,
					const CCCoreLib::GenericDistribution::ScalarContainer& predicted );
	void setSessionRun(QString session, int run);
	bool save(QString filePath);
	float getOverallAccuracy() const;

private:
	Ui::ConfusionMatrix* m_ui;
	cv::Mat m_confusionMatrix;
	cv::Mat m_precisionRecallF1Score;
	std::vector<ScalarType> m_classNumbers;
	float m_overallAccuracy;
};

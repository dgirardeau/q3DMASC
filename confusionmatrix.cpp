#include "confusionmatrix.h"
#include "ui_confusionmatrix.h"

#include <CCConst.h>

#include <iterator>
#include <set>
#include <algorithm>

#include <QBrush>

ConfusionMatrix::ConfusionMatrix(std::vector<ScalarType> &actual, std::vector<ScalarType> &predicted, QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ConfusionMatrix)
{
	ui->setupUi(this);
	this->setWindowFlag(Qt::WindowStaysOnTopHint);
	compute(actual, predicted);
	this->show();
	this->setMinimumSize(this->ui->tableWidget->sizeHint());
}

ConfusionMatrix::~ConfusionMatrix()
{
	delete ui;
}

void ConfusionMatrix::computePrecisionRecallF1Score(cv::Mat& matrix, cv::Mat& precisionRecallF1Score)
{
	int nbClasses = matrix.rows;

	// compute precision
	for (int predictedIdx = 0; predictedIdx < nbClasses; predictedIdx++)
	{
		float TP = 0;
		float FP = 0;
		for (int realIdx = 0; realIdx < nbClasses; realIdx++)
		{
			if (realIdx == predictedIdx)
				TP = matrix.at<int>(realIdx, realIdx);
			else
				FP += matrix.at<int>(realIdx, predictedIdx);
		}
		float TP_FP = TP + FP;
		if (TP_FP == 0)
			precisionRecallF1Score.at<float>(predictedIdx, PRECISION) = CCCoreLib::NAN_VALUE;
		else
			precisionRecallF1Score.at<float>(predictedIdx, PRECISION) = TP / TP_FP;
	}

	// compute recall
	for (int realIdx = 0; realIdx < nbClasses; realIdx++)
	{
		float TP = 0;
		float FN = 0;
		for (int predictedIdx = 0; predictedIdx< nbClasses; predictedIdx++)
		{
			if (realIdx == predictedIdx)
				TP = matrix.at<int>(realIdx, realIdx);
			else
				FN += matrix.at<int>(realIdx, predictedIdx);
		}
		float TP_FN = TP + FN;
		if (TP_FN == 0)
			precisionRecallF1Score.at<float>(realIdx, RECALL) = CCCoreLib::NAN_VALUE;
		else
			precisionRecallF1Score.at<float>(realIdx, RECALL) = TP / TP_FN;
	}

	// compute F1-score
	for (int realIdx = 0; realIdx < nbClasses; realIdx++)
	{
		float den = precisionRecallF1Score.at<float>(realIdx, PRECISION)
				+ precisionRecallF1Score.at<float>(realIdx, RECALL);
		if (den == 0)
			precisionRecallF1Score.at<float>(realIdx, F1_SCORE) = CCCoreLib::NAN_VALUE;
		else
			precisionRecallF1Score.at<float>(realIdx, F1_SCORE) =
					2
					* precisionRecallF1Score.at<float>(realIdx, PRECISION)
					* precisionRecallF1Score.at<float>(realIdx, RECALL)
					/ den;
	}

}

float ConfusionMatrix::computeOverallAccuracy(cv::Mat& matrix)
{
	int nbClasses = matrix.rows;
	float totalTrue = 0;
	float totalFalse = 0;
	float overallAccuracy = 0.;
	for (int realIdx = 0; realIdx < nbClasses; realIdx++)
	{
		for (int predictedIdx = 0; predictedIdx< nbClasses; predictedIdx++)
		{
			if (realIdx == predictedIdx)
				totalTrue += matrix.at<int>(realIdx, realIdx);
			else
				totalFalse += matrix.at<int>(realIdx, predictedIdx);
		}
	}
	if ((totalTrue + totalFalse) != 0)
		overallAccuracy = totalTrue / (totalTrue + totalFalse);
	else
		overallAccuracy = CCCoreLib::NAN_VALUE;

	return overallAccuracy;
}

void ConfusionMatrix::compute(std::vector<ScalarType>& actual, std::vector<ScalarType>& predicted)
{
	int idxActual;
	int idxPredicted;
	int actualClass;
	int predictedClass;

	// get the set of classes with the contents of the actual classes
	std::set<ScalarType> classes(actual.begin(), actual.end());
	int nbClasses = classes.size();
	cv::Mat confusionMatrix(nbClasses, nbClasses, CV_32S, cv::Scalar(0));
	cv::Mat precisionRecallF1Score(nbClasses, 3, CV_32F, cv::Scalar(0));

	// fill the confusion matrix
	for (int i = 0; i < actual.size(); i++)
	{
		actualClass = actual.at(i);
		idxActual = std::distance(classes.begin(), classes.find(actualClass));
		predictedClass = predicted.at(i);
		idxPredicted = std::distance(classes.begin(), classes.find(predictedClass));
		confusionMatrix.at<int>(idxActual, idxPredicted)++;
	}

	// compute precision recall F1-score
	computePrecisionRecallF1Score(confusionMatrix, precisionRecallF1Score);
	float overallAccuracy = computeOverallAccuracy(confusionMatrix);

	// display the overall accuracy
	this->ui->label_overallAccuracy->setText(QString::number(overallAccuracy, 'g', 2));

	std::set<ScalarType>::iterator itB = classes.begin();
	std::set<ScalarType>::iterator itE = classes.end();
	std::vector<ScalarType> vtr;
	vtr.assign(itB, itE);

	// BUILD THE QTABLEWIDGET

	this->ui->tableWidget->setColumnCount(2+ nbClasses + 3); // +2 for titles, +3 for precision / recall / F1-score
	this->ui->tableWidget->setRowCount(2 + nbClasses);
	// create a font for the table widgets
	QFont font;
	font.setBold(true);
	QTableWidgetItem *newItem = nullptr;
	// set the row and column names
	this->ui->tableWidget->setSpan(0, 0, 2, 2); // empty area
	this->ui->tableWidget->setSpan(0, 2, 1, nbClasses); // 'Predicted' header
	this->ui->tableWidget->setSpan(2, 0, nbClasses, 1); // 'Actual' header
	this->ui->tableWidget->setSpan(0, 2 + nbClasses, 1, 3); // empty area
	// Predicted
	newItem = new QTableWidgetItem("Predicted");
	newItem->setFont(font);
	newItem->setBackground(Qt::lightGray);
	newItem->setTextAlignment(Qt::AlignCenter);
	this->ui->tableWidget->setItem(0, 2, newItem);
	// Actual
	newItem = new QTableWidgetItem("Actual");
	newItem->setFont(font);
	newItem->setBackground(Qt::lightGray);
	newItem->setTextAlignment(Qt::AlignCenter);
	this->ui->tableWidget->setItem(2, 0, newItem);
	// add precision / recall / F1-score headers
	newItem = new QTableWidgetItem("Precision");
	newItem->setToolTip("TP / (TP + FP)");
	newItem->setFont(font);
	this->ui->tableWidget->setItem(1, 2 + nbClasses + PRECISION, newItem);
	newItem = new QTableWidgetItem("Recall");
	newItem->setToolTip("TP / (TP + FN)");
	newItem->setFont(font);
	this->ui->tableWidget->setItem(1, 2 + nbClasses + RECALL, newItem);
	newItem = new QTableWidgetItem("F1-score");
	newItem->setToolTip("Harmonic mean of precision and recall (the closer to 1 the better)\n2 x precision x recall / (precision + recall)");
	newItem->setFont(font);
	this->ui->tableWidget->setItem(1, 2 + nbClasses + F1_SCORE, newItem);
	// add column names and row names
	for (int idx = 0; idx < vtr.size(); idx++)
	{
		QString str = QString::number(vtr[idx]);
		newItem = new QTableWidgetItem(str);
		newItem->setFont(font);
		this->ui->tableWidget->setItem(1, 2 + idx, newItem);
		newItem = new QTableWidgetItem(str);
		newItem->setFont(font);
		this->ui->tableWidget->setItem(2 + idx, 1, newItem);
	}

	// FILL THE QTABLEWIDGET

	// add the confusion matrix values
	QBrush greenBrush(QColorConstants::Svg::palegreen);
	for (int row = 0; row < nbClasses; row++)
		for (int column = 0; column < nbClasses; column++)
		{
			QTableWidgetItem *newItem = new QTableWidgetItem(QString::number(confusionMatrix.at<int>(row, column)));
			if (row == column)
				newItem->setBackground(greenBrush); // green QColor(37, 190, 147, 1)
			else
				newItem->setBackground(QColorConstants::Svg::orange); // QColor(255, 129, 129, 1)
			this->ui->tableWidget->setItem(2 + row, + 2 + column, newItem);
		}

	// set precision / recall / F1-score values
	for (int realIdx=0; realIdx < nbClasses; realIdx++)
	{
		newItem = new QTableWidgetItem(QString::number(precisionRecallF1Score.at<float>(realIdx, PRECISION), 'g', 2));
		this->ui->tableWidget->setItem(2 + realIdx, 2 + nbClasses + PRECISION, newItem);
		newItem = new QTableWidgetItem(QString::number(precisionRecallF1Score.at<float>(realIdx, RECALL), 'g', 2));
		this->ui->tableWidget->setItem(2 + realIdx, 2 + nbClasses + RECALL, newItem);
		newItem = new QTableWidgetItem(QString::number(precisionRecallF1Score.at<float>(realIdx, F1_SCORE), 'g', 2));
		this->ui->tableWidget->setItem(2 + realIdx, 2 + nbClasses + F1_SCORE, newItem);
	}
}

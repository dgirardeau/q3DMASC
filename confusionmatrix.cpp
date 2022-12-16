#include "confusionmatrix.h"
#include "ui_confusionmatrix.h"

#include <CCConst.h>

#include <iterator>
#include <set>
#include <algorithm>

#define PRECISION 0
#define RECALL 1
#define F1_SCORE 2

ConfusionMatrix::ConfusionMatrix(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ConfusionMatrix)
{
	ui->setupUi(this);
}

ConfusionMatrix::~ConfusionMatrix()
{
	delete ui;
}

void ConfusionMatrix::compute_precision_recall_f1_score(cv::Mat& confusion_matrix, cv::Mat& precision_recall_f1_score)
{
	int nbClasses = confusion_matrix.rows;
	// compute precision
	for (int realIdx = 0; realIdx < nbClasses; realIdx++)
	{
		int TP = 0;
		int FP = 0;
		for (int predictedIdx = 0; predictedIdx< nbClasses; predictedIdx++)
		{
			if (realIdx == predictedIdx)
				TP = confusion_matrix.at<int>(realIdx, predictedIdx);
			else
				FP += confusion_matrix.at<int>(realIdx, predictedIdx);
		}
		int den = TP + FP;
		if (den == 0)
			precision_recall_f1_score.at<float>(PRECISION, realIdx) = CCCoreLib::NAN_VALUE;
		else
			precision_recall_f1_score.at<float>(PRECISION, realIdx) = TP / den;
	}
	// compute recall
	for (int predictedIdx = 0; predictedIdx < nbClasses; predictedIdx++)
	{
		int TP = 0;
		int FN = 0;
		for (int realIdx = 0; realIdx< nbClasses; realIdx++)
		{
			if (realIdx == predictedIdx)
				TP = confusion_matrix.at<int>(realIdx, predictedIdx);
			else
				FN += confusion_matrix.at<int>(realIdx, predictedIdx);
		}
		int den = TP + FN;
		if (den == 0)
			precision_recall_f1_score.at<float>(RECALL, realIdx) = CCCoreLib::NAN_VALUE;
		else
			precision_recall_f1_score.at<float>(RECALL, realIdx) = TP / den;
	}

	// compute F1-score

}

void ConfusionMatrix::compute(std::vector<ScalarType> &reality, std::vector<ScalarType> &predicted)
{
	std::set<ScalarType> classes(reality.begin(), reality.end());
	int idx_actual;
	int idx_predicted;
	int nbClasses = classes.size();
	int actual_class;
	int predicted_class;
	cv::Mat confusion_matrix(nbClasses, nbClasses, CV_32S, cv::Scalar(0));
	cv::Mat precision_recall_f1_score(nbClasses, 3, CV_32F, cv::Scalar(0));

	for (int i = 0; i < reality.size(); i++)
	{
		actual_class = reality.at(i);
		idx_actual = std::distance(classes.begin(), classes.find(actual_class));
		predicted_class = predicted.at(i);
		idx_predicted = std::distance(classes.begin(), classes.find(predicted_class));
		confusion_matrix.at<int>(idx_actual, idx_predicted)++;
	}
	// update the qTableWidget
	this->ui->tableWidget->setColumnCount(nbClasses + 2);
	this->ui->tableWidget->setRowCount(nbClasses + 2);
	for (uint row = 0; row < nbClasses; row++)
		for (uint column = 0; column < nbClasses; column++)
		{
			QTableWidgetItem *newItem = new QTableWidgetItem(QString::number(confusion_matrix.at<int>(row, column)));
			if (row == column)
				newItem->setBackground(QColor(37, 190, 147, 1)); // green
			else
				newItem->setBackground(QColor(255, 129, 129, 1));
			this->ui->tableWidget->setItem(row + 2, column + 2, newItem);
		}

	std::set<ScalarType>::iterator itB = classes.begin();
	std::set<ScalarType>::iterator itE = classes.end();
	std::vector<ScalarType> vtr;
	vtr.assign(itB, itE);
	QTableWidgetItem *newItem = nullptr;

	// set the row andd column names
	newItem = new QTableWidgetItem(QString::number(vtr[1]));
	this->ui->tableWidget->setItem(3, 1, newItem);
	this->ui->tableWidget->setSpan(0, 2, 1, 2);
	this->ui->tableWidget->setSpan(2, 0, 2, 1);
	newItem = new QTableWidgetItem("Predicted");
	QFont font(newItem->font());
	font.setBold(true);
	newItem->setFont(font);
	newItem->setBackground(Qt::lightGray);
	this->ui->tableWidget->setItem(0, 2, newItem);
	newItem = new QTableWidgetItem("True");
	newItem->setFont(font);
	newItem->setBackground(Qt::lightGray);
	this->ui->tableWidget->setItem(2, 0, newItem);

	// add data to the QTableWidget
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

	// compute precision recall F1-score

}

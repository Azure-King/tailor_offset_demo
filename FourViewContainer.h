#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QFrame>
#include <QLabel>
#include <QComboBox>
#include <QSlider>
#include <QHash>
#include <vector>
#include "BooleanOperations.h"

class Sketch2DView;

class FourViewContainer : public QWidget {
    Q_OBJECT

public:
    explicit FourViewContainer(QWidget* parent = nullptr);
    ~FourViewContainer();

    // Main view (top-left) - full functionality
    Sketch2DView* mainView() const { return m_mainView; }

    // Secondary views - view only (pan/zoom only)
    Sketch2DView* topRightView() const { return m_topRightView; }
    Sketch2DView* bottomLeftView() const { return m_bottomLeftView; }
    Sketch2DView* bottomRightView() const { return m_bottomRightView; }

    // Synchronize view state from main view to all views
    void synchronizeViews();

signals:
    // 预留信号：曲线偏置操作触发
    void offsetOperationTriggered(double distance);
    void pipelineStepChanged(int step);

private slots:
    void runFullPipeline();

private:
    void setupViews();
    void setupLayout();
    void processSelfIntersection();
    void processCurveOffset(double distance);
    void processDeselfIntersection();

    Sketch2DView* m_mainView = nullptr;
    Sketch2DView* m_topRightView = nullptr;
    Sketch2DView* m_bottomLeftView = nullptr;
    Sketch2DView* m_bottomRightView = nullptr;

    // 流水线中间数据：存储弧段结果以便在步骤间传递
    std::vector<std::vector<tailor_visualization::Arc>> m_mergedFillArcs;   // 自交处理+合并后的弧段
    std::vector<std::vector<tailor_visualization::Arc>> m_mergedOffsetArcs; // 偏置+合并后的弧段

    // 本地 segmentId → 原始输入边 ID 的映射，用于第一视图高亮溯源
    QHash<int, int> m_localToOriginalSegId;

    // 流水线控件
    QComboBox* m_fillTypeCombo = nullptr;
    QComboBox* m_connectTypeCombo = nullptr;
    QComboBox* m_joinConvexStyleCombo = nullptr;
    QSlider* m_offsetDistanceSlider = nullptr;
    QLabel* m_offsetValueLabel = nullptr;

    QGridLayout* m_layout = nullptr;
};

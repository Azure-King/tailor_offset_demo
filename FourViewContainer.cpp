#include "FourViewContainer.h"
#include "Sketch2DView.h"
#include "BooleanOperations.h"
#include "CurveOffset.h"

#include <QFrame>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QDebug>

// 辅助函数：将 Sketch2DView::Polygon 转换为 Arc 数组，并为每条边分配唯一 ID
static std::vector<tailor_visualization::Arc> polygonToArcs(
    const Sketch2DView::Polygon& poly, int& nextSegmentId) {
    std::vector<tailor_visualization::Arc> arcs;
    for (int i = 0; i < poly.vertices.size(); ++i) {
        const auto& v1 = poly.vertices[i];
        const auto& v2 = poly.vertices[(i + 1) % poly.vertices.size()];
        arcs.push_back(tailor_visualization::Arc(
            tailor_visualization::ArcPoint{ v1.point.x(), v1.point.y() },
            tailor_visualization::ArcPoint{ v2.point.x(), v2.point.y() },
            v1.bulge,
            tailor_visualization::ArcUserData(QRgba64(), nextSegmentId++)
        ));
    }
    return arcs;
}

// 辅助函数：将 Arc 数组转换为 Sketch2DView::OffsetResultPolygon（并携带 segmentId + sourceEdgeId + edgeTag + convexJoinVertex）
static Sketch2DView::OffsetResultPolygon arcsToPolygon(
    const std::vector<tailor_visualization::Arc>& arcs,
    const QColor& color = QColor(),
    bool isHole = false) {
    Sketch2DView::OffsetResultPolygon result;
    result.color = color;
    result.isHole = isHole;
    for (const auto& arc : arcs) {
        Sketch2DView::PolygonVertex vertex;
        vertex.point = QPointF(arc.Point0().x, arc.Point0().y);
        vertex.bulge = arc.Bulge();
        result.vertices.append(vertex);
        // 携带 segmentId（当前阶段合并标记）、sourceEdgeId（关系链根节点）和 edgeTag 用于溯源高亮
        result.edgeSegmentIds.append(arc.Data().segmentId);
        result.edgeSourceEdgeIds.append(arc.Data().sourceEdgeId);
        result.edgeTags.append(arc.Data().edgeTag);
        // 凸点连接弧对应的原始顶点坐标（仅在 edgeTag==1 时有效）
        result.edgeConvexJoinVertices.append(
            arc.Data().edgeTag == 1
                ? QPointF(arc.Data().convexJoinVertexX, arc.Data().convexJoinVertexY)
                : QPointF());
    }
    return result;
}

FourViewContainer::FourViewContainer(QWidget* parent)
    : QWidget(parent) {
    setupViews();
    setupLayout();
}

FourViewContainer::~FourViewContainer() = default;

void FourViewContainer::setupViews() {
    // Create all four views
    m_mainView = new Sketch2DView(this);
    m_topRightView = new Sketch2DView(this);
    m_bottomLeftView = new Sketch2DView(this);
    m_bottomRightView = new Sketch2DView(this);

    // Set secondary views to read-only mode
    m_topRightView->setReadOnly(true);
    m_bottomLeftView->setReadOnly(true);
    m_bottomRightView->setReadOnly(true);

    // Connect main view changes to auto-run the full pipeline
    connect(m_mainView, &Sketch2DView::polylineAdded, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polygonAdded, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polylineRemoved, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polygonRemoved, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polylineModified, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polygonModified, this, &FourViewContainer::runFullPipeline);
    connect(m_mainView, &Sketch2DView::polygonColorChanged, this, &FourViewContainer::runFullPipeline);

    // 第四视图偏置溯源交互：利用偏置器回调标记的 edgeTag 区分凸点弧/偏移弧/凹点弧
    // 通过关系链 sourceEdgeId 直接追溯到原始输入边，绕过两次布尔运算的 ID 变化
    connect(m_bottomRightView, &Sketch2DView::resultEdgeHovered, this, [this](int polygonIndex, int edgeIndex, int segmentId, int sourceEdgeId, qreal bulge) {
        Q_UNUSED(bulge);
        const auto& deselfRes = m_bottomRightView->deselfIntersectionResults();

        // 通过 edgeTags 判定边的类型（由偏置器回调在生成时标记）
        // 0=OffsetEdge（偏置边）, 1=JoinConvex（凸点连接弧）, 2=JoinConcave（凹点连接线）
        bool isConvexJoin = false;
        if (polygonIndex >= 0 && polygonIndex < deselfRes.size() &&
            edgeIndex >= 0 && edgeIndex < deselfRes[polygonIndex].edgeTags.size()) {
            isConvexJoin = (deselfRes[polygonIndex].edgeTags[edgeIndex] == 1);
        }

        if (isConvexJoin) {
            // 凸点连接弧 → 直接使用偏置器生成时存入的原始顶点坐标
            QVector<QPointF> sourceVertices;

            if (polygonIndex >= 0 && polygonIndex < deselfRes.size() &&
                edgeIndex >= 0 && edgeIndex < deselfRes[polygonIndex].edgeConvexJoinVertices.size()) {
                QPointF vertex = deselfRes[polygonIndex].edgeConvexJoinVertices[edgeIndex];
                if (!vertex.isNull()) {
                    sourceVertices.append(vertex);
                }
            }

            if (!sourceVertices.isEmpty()) {
                m_mainView->clearHighlightedSourceSegmentIds();
                m_topRightView->clearHighlightedSourceSegmentIds();
                m_bottomLeftView->clearHighlightedSourceSegmentIds();
                m_bottomRightView->clearHighlightedSourceSegmentIds();

                m_mainView->setHighlightedVertices(sourceVertices);
                m_topRightView->setHighlightedVertices(sourceVertices);
                m_bottomLeftView->setHighlightedVertices(sourceVertices);
                m_bottomRightView->setHighlightedVertices(sourceVertices);
                return;
            }
        }

        // 非凸点弧（偏置边/凹点连接线）→ 使用 segmentId 高亮完整边
        m_mainView->clearHighlightedVertices();
        m_topRightView->clearHighlightedVertices();
        m_bottomLeftView->clearHighlightedVertices();
        m_bottomRightView->clearHighlightedVertices();

        QSet<int> localSegIds;
        if (segmentId >= 0) {
            localSegIds.insert(segmentId);
        }
        m_topRightView->setHighlightedSourceSegmentIds(localSegIds);
        m_bottomLeftView->setHighlightedSourceSegmentIds(localSegIds);
        m_bottomRightView->setHighlightedSourceSegmentIds(localSegIds);

        // 第一视图：使用关系链 sourceEdgeId 直接映射回原始输入边 ID
        QSet<int> origSegIds;
        if (sourceEdgeId >= 0) {
            origSegIds.insert(sourceEdgeId);
        } else if (segmentId >= 0 && m_localToOriginalSegId.contains(segmentId)) {
            origSegIds.insert(m_localToOriginalSegId[segmentId]);
        }
        m_mainView->setHighlightedSourceSegmentIds(origSegIds);
    });
    connect(m_bottomRightView, &Sketch2DView::resultEdgeHoverEnded, this, [this]() {
        m_mainView->clearHighlightedSourceSegmentIds();
        m_mainView->clearHighlightedVertices();
        m_topRightView->clearHighlightedSourceSegmentIds();
        m_topRightView->clearHighlightedVertices();
        m_bottomLeftView->clearHighlightedSourceSegmentIds();
        m_bottomLeftView->clearHighlightedVertices();
        m_bottomRightView->clearHighlightedSourceSegmentIds();
        m_bottomRightView->clearHighlightedVertices();
    });
}

void FourViewContainer::setupLayout() {
    m_layout = new QGridLayout(this);
    m_layout->setContentsMargins(0, 0, 0, 0);
    m_layout->setSpacing(2);

    // 浮层控件样式
    const char* overlayStyle = R"(
        QWidget#overlayPanel {
            background: rgba(30, 30, 30, 160);
            border-radius: 6px;
            padding: 2px 6px;
        }
        QLabel { color: #ddd; font-size: 11px; }
        QComboBox { 
            background: #3a3a3a; color: #eee; border: 1px solid #555; 
            border-radius: 3px; padding: 1px 4px; font-size: 11px; min-width: 70px;
        }
        QComboBox::drop-down { border: none; }
        QComboBox QAbstractItemView { 
            background: #2a2a2a; color: #eee; selection-background-color: #5a5a5a;
        }
        QSlider::groove:horizontal { 
            background: #555; height: 4px; border-radius: 2px;
        }
        QSlider::handle:horizontal { 
            background: #888; width: 12px; margin: -4px 0; border-radius: 6px;
        }
    )";

    // --- 主视图 (左上) ---
    auto* frameMain = new QFrame(this);
    frameMain->setFrameShape(QFrame::StyledPanel);
    frameMain->setFrameShadow(QFrame::Sunken);
    auto* layoutMain = new QVBoxLayout(frameMain);
    layoutMain->setContentsMargins(0, 0, 0, 0);
    layoutMain->addWidget(m_mainView);

    // --- 第二视图 (右上)：视图填满，控件浮在上方 ---
    auto* frameTopRight = new QFrame(this);
    frameTopRight->setFrameShape(QFrame::StyledPanel);
    frameTopRight->setFrameShadow(QFrame::Sunken);
    auto* layoutTopRight = new QVBoxLayout(frameTopRight);
    layoutTopRight->setContentsMargins(0, 0, 0, 0);
    layoutTopRight->addWidget(m_topRightView);

    // 浮层面板：填充+连接方式 (父控件为 m_topRightView)
    auto* overlayTopRight = new QWidget(m_topRightView);
    overlayTopRight->setObjectName("overlayPanel");
    overlayTopRight->setStyleSheet(overlayStyle);
    auto* ol2 = new QHBoxLayout(overlayTopRight);
    ol2->setContentsMargins(6, 3, 6, 3);
    ol2->setSpacing(4);

    m_fillTypeCombo = new QComboBox(overlayTopRight);
    m_fillTypeCombo->addItem("NonZero", 0);
    m_fillTypeCombo->addItem("EvenOdd", 1);
    m_fillTypeCombo->addItem("Ignore", 2);
    m_fillTypeCombo->addItem("Positive", 3);
    m_fillTypeCombo->addItem("Winding=1", 4);
    m_fillTypeCombo->setCurrentIndex(1);
    ol2->addWidget(new QLabel("填充:", overlayTopRight));
    ol2->addWidget(m_fillTypeCombo);

    m_connectTypeCombo = new QComboBox(overlayTopRight);
    m_connectTypeCombo->addItem("外先", 0);
    m_connectTypeCombo->addItem("内先", 1);
    m_connectTypeCombo->setCurrentIndex(0);
    ol2->addWidget(new QLabel("连接:", overlayTopRight));
    ol2->addWidget(m_connectTypeCombo);

    // 下拉框变更时触发流水线
    connect(m_fillTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FourViewContainer::runFullPipeline);
    connect(m_connectTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FourViewContainer::runFullPipeline);

    overlayTopRight->adjustSize();
    overlayTopRight->move(8, 8);
    overlayTopRight->show();

    // --- 第三视图 (左下)：视图填满，偏置滑块浮在上方 ---
    auto* frameBottomLeft = new QFrame(this);
    frameBottomLeft->setFrameShape(QFrame::StyledPanel);
    frameBottomLeft->setFrameShadow(QFrame::Sunken);
    auto* layoutBottomLeft = new QVBoxLayout(frameBottomLeft);
    layoutBottomLeft->setContentsMargins(0, 0, 0, 0);
    layoutBottomLeft->addWidget(m_bottomLeftView);

    // 浮层面板：偏置滑块 (父控件为 m_bottomLeftView)
    auto* overlayBottomLeft = new QWidget(m_bottomLeftView);
    overlayBottomLeft->setObjectName("overlayPanel");
    overlayBottomLeft->setStyleSheet(overlayStyle);
    auto* ol3 = new QHBoxLayout(overlayBottomLeft);
    ol3->setContentsMargins(6, 3, 6, 3);
    ol3->setSpacing(4);

    m_offsetDistanceSlider = new QSlider(Qt::Horizontal, overlayBottomLeft);
    m_offsetDistanceSlider->setRange(-100, 100);
    m_offsetDistanceSlider->setValue(10);
    m_offsetDistanceSlider->setFixedWidth(160);
    ol3->addWidget(new QLabel("偏置:", overlayBottomLeft));
    ol3->addWidget(m_offsetDistanceSlider);

    m_offsetValueLabel = new QLabel("10", overlayBottomLeft);
    m_offsetValueLabel->setFixedWidth(32);
    m_offsetValueLabel->setAlignment(Qt::AlignCenter);
    m_offsetValueLabel->setStyleSheet("color: #fff; font-weight: bold;");
    ol3->addWidget(m_offsetValueLabel);

    connect(m_offsetDistanceSlider, &QSlider::valueChanged, this, [this](int val) {
        m_offsetValueLabel->setText(QString::number(val));
        runFullPipeline();
    });

    overlayBottomLeft->adjustSize();
    overlayBottomLeft->move(8, 8);
    overlayBottomLeft->show();

    // --- 第四视图 (右下) ---
    auto* frameBottomRight = new QFrame(this);
    frameBottomRight->setFrameShape(QFrame::StyledPanel);
    frameBottomRight->setFrameShadow(QFrame::Sunken);
    auto* layoutBottomRight = new QVBoxLayout(frameBottomRight);
    layoutBottomRight->setContentsMargins(0, 0, 0, 0);
    layoutBottomRight->addWidget(m_bottomRightView);

    // 2x2 grid
    m_layout->addWidget(frameMain, 0, 0);
    m_layout->addWidget(frameTopRight, 0, 1);
    m_layout->addWidget(frameBottomLeft, 1, 0);
    m_layout->addWidget(frameBottomRight, 1, 1);

    m_layout->setColumnStretch(0, 1);
    m_layout->setColumnStretch(1, 1);
    m_layout->setRowStretch(0, 1);
    m_layout->setRowStretch(1, 1);
}

void FourViewContainer::synchronizeViews() {
    // 清除所有视图的高亮状态
    m_mainView->clearHighlightedSourceSegmentIds();
    m_topRightView->clearHighlightedSourceSegmentIds();
    m_bottomLeftView->clearHighlightedSourceSegmentIds();
    m_bottomRightView->clearHighlightedSourceSegmentIds();

    // 清除第二视图的所有数据（不显示原始多边形）
    m_topRightView->clearSelection();
    m_topRightView->clearFillResults();
    m_topRightView->clearSelfIntersectionResults();
    m_topRightView->clearOffsetResults();
    m_topRightView->clearDeselfIntersectionResults();
    m_topRightView->update();

    // 清除第三视图的所有数据（不显示原始多边形）
    m_bottomLeftView->clearSelection();
    m_bottomLeftView->clearFillResults();
    m_bottomLeftView->clearSelfIntersectionResults();
    m_bottomLeftView->clearOffsetResults();
    m_bottomLeftView->clearDeselfIntersectionResults();
    m_bottomLeftView->update();

    // 清除第四视图的所有数据（不显示原始多边形）
    m_bottomRightView->clearSelection();
    m_bottomRightView->clearFillResults();
    m_bottomRightView->clearSelfIntersectionResults();
    m_bottomRightView->clearOffsetResults();
    m_bottomRightView->clearDeselfIntersectionResults();
    m_bottomRightView->update();
}

void FourViewContainer::processSelfIntersection() {
    const auto& polygons = m_mainView->polygons();
    qDebug() << "processSelfIntersection: polygons.size() =" << polygons.size();
    if (polygons.isEmpty()) {
        qDebug() << "无输入多边形";
        return;
    }

    // 预定义颜色调色板
    static std::vector<QColor> colorPalette = {
        QColor(255, 100, 100),   // 红色
        QColor(100, 200, 100),   // 绿色
        QColor(100, 100, 255),   // 蓝色
        QColor(255, 200, 100),   // 橙色
        QColor(200, 100, 255),   // 紫色
        QColor(100, 255, 200),  // 青色
        QColor(255, 150, 100),  // 橙红
        QColor(150, 100, 255),   // 紫蓝
    };

    // Step 1: 将所有曲线加入 tailor subject 集合，并为每条边分配唯一 ID
    tailor_visualization::BooleanOperations boolOp;
    int nextSegmentId = 0;
    for (const auto& poly : polygons) {
        auto arcs = polygonToArcs(poly, nextSegmentId);
        boolOp.AddSubjectPolygon(arcs);
    }

    // Step 2: 执行 OnlySubjectPattern 获取非自交曲线
    int fillTypeIndex = m_fillTypeCombo->currentData().toInt();
    const tailor_visualization::IFillType* fillType = nullptr;
    switch (fillTypeIndex) {
    case 0: fillType = std::addressof(*new tailor_visualization::NonZeroFillTypeWrapper()); break;
    case 1: fillType = std::addressof(*new tailor_visualization::EvenOddFillTypeWrapper()); break;
    case 2: fillType = std::addressof(*new tailor_visualization::IgnoreFillTypeWrapper()); break;
    case 3: fillType = std::addressof(*new tailor_visualization::PositiveWindFillTypeWrapper()); break;  // 环绕数>0
    case 4: fillType = std::addressof(*new tailor_visualization::SpecificWindingFillTypeWrapper(1)); break;
    default: fillType = std::addressof(*new tailor_visualization::EvenOddFillTypeWrapper()); break;
    }

    auto resultArcs = boolOp.ExecuteOnlySubjectPattern(fillType);
    qDebug() << "processSelfIntersection: resultArcs.size() =" << resultArcs.size();

    // Step 2.5: 合并具有相同 segmentId 的相邻弧段，消除单调分割产生的小线段
    if constexpr (tailor_visualization::ENABLE_CURVE_MERGE) {
        tailor_visualization::MergeAdjacentCurvesBatch(resultArcs);
    }

    // Step 2.6: 重新分配唯一的本地 segmentId，替换原始输入边 ID
    // 这样每条 fillResult 边都有唯一标识，高亮时能精确定位到打断后的具体边
    // 注意：sourceEdgeId 保持不变，作为关系链根节点用于全流水线溯源
    m_localToOriginalSegId.clear();
    int localSegId = 0;
    for (auto& polygonArcs : resultArcs) {
        for (auto& arc : polygonArcs) {
            int originalId = arc.Data().sourceEdgeId;
            m_localToOriginalSegId[localSegId] = originalId;
            arc.Data().segmentId = localSegId++;
        }
    }

    m_mergedFillArcs = resultArcs;  // 保存到中间数据供下个步骤使用
    qDebug() << "processSelfIntersection: after merge, resultArcs.size() =" << resultArcs.size();

    // 转换为填充结果（第二视图用，不同多边形不同颜色）
    QVector<Sketch2DView::OffsetResultPolygon> fillResults;
    for (size_t i = 0; i < resultArcs.size(); ++i) {
        QColor color = colorPalette[i % colorPalette.size()];
        fillResults.append(arcsToPolygon(resultArcs[i], color));
    }
    qDebug() << "processSelfIntersection: fillResults.size() =" << fillResults.size();

    // 转换为自交处理结果（橙色，用于参考）
    QVector<Sketch2DView::OffsetResultPolygon> selfIntersectionResults;
    for (const auto& arcs : resultArcs) {
        selfIntersectionResults.append(arcsToPolygon(arcs, QColor(255, 165, 0)));
    }

    // 设置第二视图：只显示填充结果
    m_topRightView->clearSelfIntersectionResults();
    m_topRightView->setFillResults(fillResults);
    m_topRightView->clearOffsetResults();
    m_topRightView->clearDeselfIntersectionResults();

    qDebug() << QString("自交处理完成: %1 个多边形").arg(fillResults.size());
    emit pipelineStepChanged(1);
}

void FourViewContainer::processCurveOffset(double distance) {
    qDebug() << "processCurveOffset: m_mergedFillArcs.size() =" << m_mergedFillArcs.size();
    if (m_mergedFillArcs.empty()) {
        qDebug() << "无合并后的弧段数据";
        return;
    }

    // 直接使用 Arc 类型（带 ArcUserData），偏置器模板化支持任意 UserData
    using ArcType = tailor_visualization::Arc;

    // 注册偏置器回调：标记每条输出边的类型到 ArcUserData.edgeTag
    tailor_offset::CurveOffseter<ArcType, double>::s_onOffsetEdge = [](int, ArcType& curve) {
        curve.Data().edgeTag = 0;  // OffsetEdge
    };
    tailor_offset::CurveOffseter<ArcType, double>::s_onJoinConvex = std::function<void(int, const tailor::Point<double>&, ArcType&)>(
        [](int /*sourceIndex*/, const tailor::Point<double>& joinVertex, ArcType& curve) {
            curve.Data().edgeTag = 1;  // JoinConvex
            curve.Data().convexJoinVertexX = joinVertex.x;
            curve.Data().convexJoinVertexY = joinVertex.y;
        });
    tailor_offset::CurveOffseter<ArcType, double>::s_onJoinConcave = [](int, int, ArcType& curve) {
        curve.Data().edgeTag = 2;  // JoinConcave
    };

    QVector<Sketch2DView::OffsetResultPolygon> offsetResults;
    m_mergedOffsetArcs.clear();

    for (size_t i = 0; i < m_mergedFillArcs.size(); ++i) {
        const auto& curves = m_mergedFillArcs[i];
        qDebug() << "  poly" << i << ": curves.size() =" << curves.size();
        // === DIAGNOSTIC: 打印偏置输入弧段的 sourceEdgeId ===
        for (size_t ci = 0; ci < curves.size(); ++ci) {
            qDebug() << "    input_curve[" << ci << "]: sourceEdgeId=" << curves[ci].Data().sourceEdgeId
                     << " segmentId=" << curves[ci].Data().segmentId;
        }
        if (curves.empty()) continue;

        // 执行偏置（直接使用合并后的弧段）
        bool ccw = true;  // 假设逆时针方向
        auto offsetResult = tailor_offset::CurveOffseter<ArcType, double>::OffsetClosed(curves, distance, ccw);

        // 将偏置结果收集为弧段数组
        if (offsetResult.Size() > 0) {
            std::vector<ArcType> offsetArcs;
            int oi = 0;
            for (const auto& arc : offsetResult) {
                offsetArcs.push_back(arc);
                // === DIAGNOSTIC: 打印偏置输出弧段的 sourceEdgeId ===
                qDebug() << "    offset_output[" << oi << "]: sourceEdgeId=" << arc.Data().sourceEdgeId
                         << " segmentId=" << arc.Data().segmentId
                         << " edgeTag=" << arc.Data().edgeTag
                         << " bulge=" << arc.Bulge();
                ++oi;
            }

            m_mergedOffsetArcs.push_back(offsetArcs);

            // 转换为显示数据（携带 segmentId、sourceEdgeId 和 edgeTag）
            Sketch2DView::OffsetResultPolygon resultPoly;
            resultPoly.color = QColor(100, 149, 237);  // 蓝色
            for (const auto& arc : offsetArcs) {
                resultPoly.vertices.append(Sketch2DView::PolygonVertex{
                    QPointF(arc.Point0().x, arc.Point0().y),
                    arc.Bulge()
                    });
                resultPoly.edgeSegmentIds.append(arc.Data().segmentId);
                resultPoly.edgeSourceEdgeIds.append(arc.Data().sourceEdgeId);
                resultPoly.edgeTags.append(arc.Data().edgeTag);
            }
            offsetResults.append(resultPoly);
        }
    }

    // 设置第三视图：显示偏置结果 + 填充结果
    const auto& fillResults = m_topRightView->fillResults();
    m_bottomLeftView->setFillResults(fillResults);  // 第二视图的内容
    m_bottomLeftView->setOffsetResults(offsetResults);  // 偏置结果
    m_bottomLeftView->clearSelfIntersectionResults();
    m_bottomLeftView->clearDeselfIntersectionResults();

    qDebug() << QString("偏置完成: %1 条曲线").arg(offsetResults.size());
    emit pipelineStepChanged(2);
}

void FourViewContainer::processDeselfIntersection() {
    const auto& fillResults = m_topRightView->fillResults();
    if (m_mergedOffsetArcs.empty()) {
        qDebug() << "无偏置弧段数据";
        return;
    }

    // 直接使用存储的偏置弧段数据（已合并过）
    tailor_visualization::BooleanOperations boolOp;
    for (const auto& arcs : m_mergedOffsetArcs) {
        boolOp.AddSubjectPolygon(arcs);
    }

    // 使用"正"填充（环绕数 > 0）
    tailor_visualization::PositiveWindFillTypeWrapper fillType;
    auto resultArcs = boolOp.ExecuteOnlySubjectPattern(&fillType);

    // 布尔运算后也做一次合并
    if constexpr (tailor_visualization::ENABLE_CURVE_MERGE) {
        tailor_visualization::MergeAdjacentCurvesBatch(resultArcs);
    }

    // 转换结果
    QVector<Sketch2DView::OffsetResultPolygon> results;
    qDebug() << "processDeselfIntersection: resultArcs.size() =" << resultArcs.size();
    for (size_t ri = 0; ri < resultArcs.size(); ++ri) {
        const auto& arcs = resultArcs[ri];
        qDebug() << "  deself poly" << ri << ": edges=" << arcs.size();
        for (size_t ai = 0; ai < arcs.size(); ++ai) {
            qDebug() << "    edge[" << ai << "]: sourceEdgeId=" << arcs[ai].Data().sourceEdgeId
                     << " segmentId=" << arcs[ai].Data().segmentId
                     << " edgeTag=" << arcs[ai].Data().edgeTag
                     << " bulge=" << arcs[ai].Bulge();
        }
        results.append(arcsToPolygon(arcs, QColor(138, 43, 226)));  // 紫色
    }

    // 设置第四视图：显示去自交结果 + 填充结果
    m_bottomRightView->setFillResults(fillResults);  // 第二视图的内容
    m_bottomRightView->setDeselfIntersectionResults(results);  // 去自交结果
    m_bottomRightView->clearOffsetResults();
    m_bottomRightView->clearSelfIntersectionResults();

    qDebug() << QString("去自交完成: %1 个多边形").arg(results.size());
    emit pipelineStepChanged(3);
}

void FourViewContainer::runFullPipeline() {
    qDebug() << "runFullPipeline called";
    // 先清除所有视图的原始数据
    synchronizeViews();
    // 执行完整流水线
    processSelfIntersection();
    qDebug() << "after processSelfIntersection, fillResults.size() =" << m_topRightView->fillResults().size();
    if (!m_topRightView->selfIntersectionResults().isEmpty() || !m_topRightView->fillResults().isEmpty()) {
        qDebug() << "calling processCurveOffset";
        processCurveOffset(m_offsetDistanceSlider->value());
        if (!m_bottomLeftView->offsetResults().isEmpty()) {
            processDeselfIntersection();
        }
    }
    qDebug() << "流水线执行完成";
}

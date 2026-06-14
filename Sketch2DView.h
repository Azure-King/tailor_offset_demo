#pragma once

#include <QWidget>
#include <QPointF>
#include <QVector>
#include <QMenu>
#include <QSet>
#include <QColor>

class Sketch2DView : public QWidget {
    Q_OBJECT

public:
    enum class Tool {
        Polyline,
        Polygon
    };

    // Public struct definitions
    struct PolygonVertex {
        QPointF point;
        qreal bulge = 0.0;
        QColor edgeColor = QColor(255, 255, 255);  // 边界颜色，默认白色
    };

    struct Polygon {
        QVector<PolygonVertex> vertices;
    };

    // Polyline is similar to Polygon but vertices are not connected (open shape)
    using Polyline = Polygon;

    explicit Sketch2DView(QWidget* parent = nullptr);

    void setTool(Tool tool);
    Tool tool() const { return m_tool; }

    void setReadOnly(bool readOnly);
    bool isReadOnly() const { return m_readOnly; }

    void clear();

    void setSelectedPolygon(int index, bool isPolygon);
    int selectedPolygonIndex() const { return m_selectedPolygonIndex; }

    void clearSelection();

    // 多选支持
    void addSelectedPolygon(int index);
    void removeSelectedPolygon(int index);
    void addSelectedPolyline(int index);
    void removeSelectedPolyline(int index);

    const QSet<int>& selectedPolygons() const { return m_selectedPolygons; }
    const QSet<int>& selectedPolylines() const { return m_selectedPolylines; }

    // Data access for view synchronization
    const QVector<Polyline>& polylines() const { return m_polylines; }
    const QVector<Polygon>& polygons() const { return m_polygons; }

    void setPolylines(const QVector<Polyline>& polylines);
    void setPolygons(const QVector<Polygon>& polygons);

    // 设置多边形边界颜色
    void setPolygonEdgeColor(int index, const QColor& color, bool isPolygon = true);
    void setPolygonEdgeColorBatch(const QSet<int>& indices, const QColor& color, bool isPolygon);

    // 删除指定的多段线
    void deletePolylines(const QSet<int>& indices);

    // 删除指定的多边形
    void deletePolygons(const QSet<int>& indices);

    // 偏置结果管理
    struct OffsetResultPolygon {
        QVector<PolygonVertex> vertices;
        QColor color;
        bool isHole = false;
        QVector<int> edgeSegmentIds; // 每条边的 segmentId，用于高亮溯源
    };
    void setOffsetResults(const QVector<OffsetResultPolygon>& results);
    const QVector<OffsetResultPolygon>& offsetResults() const { return m_offsetResults; }
    void clearOffsetResults() { m_offsetResults.clear(); update(); }

    // 自交处理结果管理
    void setSelfIntersectionResults(const QVector<OffsetResultPolygon>& results);
    const QVector<OffsetResultPolygon>& selfIntersectionResults() const { return m_selfIntersectionResults; }
    void clearSelfIntersectionResults() { m_selfIntersectionResults.clear(); update(); }

    // 去自交结果管理
    void setDeselfIntersectionResults(const QVector<OffsetResultPolygon>& results);
    const QVector<OffsetResultPolygon>& deselfIntersectionResults() const { return m_deselfIntersectionResults; }
    void clearDeselfIntersectionResults() { m_deselfIntersectionResults.clear(); update(); }

    // 高亮源边（用于第四视图的偏置溯源交互）
    void setHighlightedSourceSegmentIds(const QSet<int>& segmentIds);
    void clearHighlightedSourceSegmentIds();

    // 高亮源顶点（用于凸点连接弧溯源交互）
    void setHighlightedVertices(const QVector<QPointF>& vertices);
    void clearHighlightedVertices();

    // 填充结果管理（第二视图用，不同多边形不同颜色）
    void setFillResults(const QVector<OffsetResultPolygon>& results);
    const QVector<OffsetResultPolygon>& fillResults() const { return m_fillResults; }
    void clearFillResults() { m_fillResults.clear(); update(); }

    // Viewport state
    qreal scale() const { return m_scale; }
    QPointF offset() const { return m_offset; }
    void setScale(qreal scale);
    void setOffset(const QPointF& offset);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    struct Edge {
        int polygonIndex;
        int vertexIndex1;
        int vertexIndex2;
        bool isPolygon;
    };

    // Internal utility struct for arc computation
    struct ArcSegment {
        QPointF center;
        qreal radius = 0.0;
        qreal startAngleDeg = 0.0;
        qreal spanAngleDeg = 0.0;
    };

    static qreal degFromRad(qreal rad);
    static qreal radFromDeg(qreal deg);

    static qreal angleDegAt(const QPointF& center, const QPointF& p);

    // 创建多边形的 QPainterPath
    QPainterPath createPolygonPath(const Polygon& poly) const;
    QPainterPath createOffsetResultPath(const OffsetResultPolygon& poly) const;

    static qreal normalizedSpanDeg(qreal startDeg, qreal endDeg);

    // Bulge utility functions
    static qreal bulgeFromArc(const QPointF& p1, const QPointF& p2, const QPointF& throughPoint);
    static QPointF arcPointFromBulge(const QPointF& p1, const QPointF& p2, qreal bulge);
    static ArcSegment arcSegmentFromBulge(const QPointF& p1, const QPointF& p2, qreal bulge);
    static qreal splitArcBulge(qreal bulge);

    QPointF snapToPixelCenter(const QPointF& p) const;
    QPointF getEdgeMidpoint(const QPointF& p1, const QPointF& p2) const;
    bool isNearPoint(const QPointF& target, const QPointF& pos, qreal threshold = 8.0) const;
    bool isNearEdgeMidpoint(const QPointF& p1, const QPointF& p2, const QPointF& pos, qreal threshold = 8.0) const;
    Edge findNearbyEdgeMidpoint(const QPointF& pos, qreal threshold = 8.0) const;
    Edge findNearbyVertex(const QPointF& pos, qreal threshold = 8.0) const;
    // 查找附近的结果边（deselfIntersectionResults）
    struct ResultEdgeLoc {
        int polygonIndex = -1;  // 结果多边形索引
        int edgeIndex = -1;     // 边索引
        int segmentId = -1;     // 溯源 segmentId
        qreal bulge = 0.0;      // 该边的 bulge 值（用于区分凸点弧/弧偏置弧）
    };
    ResultEdgeLoc findNearbyResultEdge(const QPointF& screenPos, qreal threshold = 12.0) const;
    // 点到线/弧的最近距离（世界坐标）
    qreal pointToSegmentDistWorld(const QPointF& screenPos, const QPointF& a, const QPointF& b) const;
    qreal pointToArcDistWorld(const QPointF& screenPos, const QPointF& p1, const QPointF& p2, const ArcSegment& arc) const;

    QPointF worldToScreen(const QPointF& worldPos) const;
    QPointF screenToWorld(const QPointF& screenPos) const;

    QRectF getWorldBounds() const;

    Tool m_tool = Tool::Polyline;

    int m_polygonCounter = 0;
    int m_polylineCounter = 0;

    // Polyline and Polygon editing
    QVector<Polyline> m_polylines;
    QVector<Polygon> m_polygons;
    int m_selectedPolygonIndex = -1;
    QSet<int> m_selectedPolygons;   // 多边形的多选索引集合
    QSet<int> m_selectedPolylines;  // 多段线的多选索引集合
    int m_hoveredPolygonIndex = -1;  // 鼠标悬停的高亮多边形（只读模式下）

    // 结果边悬停状态
    ResultEdgeLoc m_hoveredResultEdge;            // 当前悬停的结果边
    QSet<int> m_highlightedSourceSegmentIds;       // 需要高亮的源边 segmentId 集合
    QVector<QPointF> m_highlightedVertices;        // 需要高亮的源顶点（凸点弧溯源）

    // 偏置结果
    QVector<OffsetResultPolygon> m_offsetResults;
    // 自交处理结果
    QVector<OffsetResultPolygon> m_selfIntersectionResults;
    // 去自交结果
    QVector<OffsetResultPolygon> m_deselfIntersectionResults;
    // 填充结果（第二视图用）
    QVector<OffsetResultPolygon> m_fillResults;

    // Dragging state
    enum class DragMode {
        None,
        Vertex,
        EdgeMidpoint
    };
    DragMode m_dragMode = DragMode::None;
    Edge m_draggedEdge;
    QPointF m_dragStartPos;
    QPointF m_originalPoint;
    int m_splitVertexIndex = -1;
    Qt::MouseButton m_dragButton = Qt::NoButton;

    // Edge to arc conversion
    Edge m_edgeToArc;
    QPointF m_arcThroughPoint;

    // Viewport
    qreal m_scale = 1.0;
    QPointF m_offset;

    bool m_isPanning = false;
    QPointF m_panStart;

    // Read-only mode
    bool m_readOnly = false;

    // Context menu state
    QPointF m_contextMenuPosition;
    bool m_hasDragged = false; // Track if dragging occurred before right click

    // Mouse position for HUD
    QPointF m_mousePos;

    bool computeArcThrough3Points(const QPointF& p1, const QPointF& p2, const QPointF& p3, ArcSegment& outArc) const;

signals:
    void polylineAdded(int index, const QString& name);
    void polygonAdded(int index, const QString& name);
    void polylineRemoved(int index);
    void polygonRemoved(int index);
    void polygonsDeleted(const QSet<int>& polylineIndices, const QSet<int>& polygonIndices);
    void selectionChanged(int polygonIndex);
    void polylineModified();
    void polygonModified();
    void polygonColorChanged(int polygonIndex, const QColor& color);
    // 结果边悬停信号：polygonIndex（结果数组中索引），edgeIndex（边索引），segmentId（溯源ID），bulge（弧凸度）
    void resultEdgeHovered(int polygonIndex, int edgeIndex, int segmentId, qreal bulge);
    void resultEdgeHoverEnded();
};

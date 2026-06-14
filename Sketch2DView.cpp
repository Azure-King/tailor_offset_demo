#include "Sketch2DView.h"

#include <QMouseEvent>
#include <QEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QContextMenuEvent>
#include <QMenu>
#include <QGuiApplication>
#include <QCoreApplication>
#include <QtMath>
#include <QPainterPath>
#include <random>
#include <cmath>
#include <QColorDialog>
#include <QDebug>

Sketch2DView::Sketch2DView(QWidget* parent)
    : QWidget(parent), m_scale(1.0), m_offset(0.0, 0.0) {
    setMouseTracking(true);
    setAutoFillBackground(true);

    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(18, 18, 18));
    setPalette(pal);

    setMinimumSize(400, 300);
}

void Sketch2DView::setTool(Tool tool) {
    if (m_readOnly) return;
    m_tool = tool;
    m_dragMode = DragMode::None;
    m_dragButton = Qt::NoButton;
    update();
}

void Sketch2DView::setScale(qreal scale) {
    m_scale = scale;
    update();
}

void Sketch2DView::setOffset(const QPointF& offset) {
    m_offset = offset;
    update();
}

void Sketch2DView::clear() {
    m_polylines.clear();
    m_polygons.clear();
    m_polygonCounter = 0;
    m_polylineCounter = 0;
    m_dragMode = DragMode::None;
    m_dragButton = Qt::NoButton;
    clearSelection();
    update();
}

qreal Sketch2DView::degFromRad(qreal rad) {
    return rad * 180.0 / M_PI;
}

qreal Sketch2DView::radFromDeg(qreal deg) {
    return deg * M_PI / 180.0;
}

qreal Sketch2DView::bulgeFromArc(const QPointF& p1, const QPointF& p2, const QPointF& throughPoint) {
    QLineF chord(p1, p2);
    qreal chordLength = chord.length();
    if (chordLength < 1e-6) return 0.0;

    QPointF midpoint = (p1 + p2) / 2.0;
    QPointF sagitta = throughPoint - midpoint;
    qreal sagittaLength = QLineF(QPointF(0, 0), sagitta).length();

    qreal cross = (p2.x() - p1.x()) * (throughPoint.y() - p1.y()) -
        (p2.y() - p1.y()) * (throughPoint.x() - p1.x());
    qreal sign = (cross < 0) ? 1.0 : -1.0;

    qreal bulge = sign * 2.0 * sagittaLength / chordLength;
    return bulge;
}

QPointF Sketch2DView::arcPointFromBulge(const QPointF& p1, const QPointF& p2, qreal bulge) {
    QLineF chord(p1, p2);
    qreal chordLength = chord.length();
    if (chordLength < 1e-6) return (p1 + p2) / 2.0;

    qreal sagitta = bulge * chordLength / 2.0;
    QPointF midpoint = (p1 + p2) / 2.0;
    QPointF perp((p2.y() - p1.y()), -(p2.x() - p1.x()));
    qreal perpLength = QLineF(QPointF(0, 0), perp).length();
    if (perpLength < 1e-6) return midpoint;
    perp = perp / perpLength;

    return midpoint + perp * sagitta;
}

Sketch2DView::ArcSegment Sketch2DView::arcSegmentFromBulge(const QPointF& p1, const QPointF& p2, qreal bulge) {
    ArcSegment arc;

    if (qAbs(bulge) < 1e-6) {
        arc.center = (p1 + p2) / 2.0;
        arc.radius = 0.0;
        arc.startAngleDeg = 0.0;
        arc.spanAngleDeg = 0.0;
        return arc;
    }

    qreal theta = 4.0 * qAtan(qAbs(bulge));
    qreal chordLength = QLineF(p1, p2).length();

    qreal sinHalfTheta = qSin(theta / 2.0);
    if (sinHalfTheta < 1e-6) {
        arc.center = (p1 + p2) / 2.0;
        arc.radius = 0.0;
        arc.startAngleDeg = 0.0;
        arc.spanAngleDeg = 0.0;
        return arc;
    }

    qreal radius = chordLength / (2.0 * sinHalfTheta);
    arc.radius = radius;

    QPointF midpoint = (p1 + p2) / 2.0;
    QPointF chordDir = (p2 - p1) / chordLength;
    QPointF perpDir(-chordDir.y(), chordDir.x());

    qreal distanceToCenter = radius * qCos(theta / 2.0);
    qreal sign = (bulge >= 0) ? 1.0 : -1.0;
    arc.center = midpoint + perpDir * (distanceToCenter * sign);

    arc.startAngleDeg = angleDegAt(arc.center, p1);
    qreal endAngleDeg = angleDegAt(arc.center, p2);

    qreal spanDeg = normalizedSpanDeg(arc.startAngleDeg, endAngleDeg);

    auto isAngleOnArc = [](qreal start, qreal span, qreal ang) {
        auto norm = [](qreal v) {
            while (v < 0) v += 360.0;
            while (v >= 360.0) v -= 360.0;
            return v;
        };
        start = norm(start);
        ang = norm(ang);

        if (qFuzzyIsNull(span)) return false;
        if (span > 0) {
            qreal end = norm(start + span);
            if (start <= end) return (ang >= start && ang <= end);
            return (ang >= start || ang <= end);
        } else {
            qreal end = norm(start + span);
            if (end <= start) return (ang <= start && ang >= end);
            return (ang <= start || ang >= end);
        }
    };

    QPointF testPoint = arcPointFromBulge(p1, p2, bulge);
    QPointF centerToTest = testPoint - arc.center;
    qreal testAngle = angleDegAt(arc.center, testPoint);

    if (!isAngleOnArc(arc.startAngleDeg, spanDeg, testAngle)) {
        spanDeg = (spanDeg > 0) ? (spanDeg - 360.0) : (spanDeg + 360.0);
    }

    arc.spanAngleDeg = spanDeg;
    return arc;
}

qreal Sketch2DView::splitArcBulge(qreal bulge) {
    if (qAbs(bulge) < 1e-6) {
        return 0.0;
    }

    qreal absBulge = qAbs(bulge);
    qreal sqrtTerm = qSqrt(1.0 + absBulge * absBulge);
    qreal newAbsBulge = absBulge / (sqrtTerm + 1.0);

    return (bulge >= 0) ? newAbsBulge : -newAbsBulge;
}

QPointF Sketch2DView::snapToPixelCenter(const QPointF& p) const {
    return QPointF(qRound(p.x()) + 0.5, qRound(p.y()) + 0.5);
}

QPointF Sketch2DView::worldToScreen(const QPointF& worldPos) const {
    const QPointF center(width() / 2.0, height() / 2.0);
    return center + QPointF((worldPos.x() - m_offset.x()) * m_scale, -(worldPos.y() - m_offset.y()) * m_scale);
}

QPointF Sketch2DView::screenToWorld(const QPointF& screenPos) const {
    const QPointF center(width() / 2.0, height() / 2.0);
    return m_offset + QPointF((screenPos.x() - center.x()) / m_scale, -(screenPos.y() - center.y()) / m_scale);
}

qreal Sketch2DView::angleDegAt(const QPointF& center, const QPointF& p) {
    QLineF l(center, p);
    return l.angle();
}

qreal Sketch2DView::normalizedSpanDeg(qreal startDeg, qreal endDeg) {
    qreal span = endDeg - startDeg;
    while (span <= -180.0) span += 360.0;
    while (span > 180.0) span -= 360.0;
    return span;
}

QPointF Sketch2DView::getEdgeMidpoint(const QPointF& p1, const QPointF& p2) const {
    return (p1 + p2) / 2.0;
}

bool Sketch2DView::isNearPoint(const QPointF& target, const QPointF& pos, qreal threshold) const {
    QPointF screenTarget = worldToScreen(target);
    QPointF delta = pos - screenTarget;
    return (delta.x() * delta.x() + delta.y() * delta.y()) < (threshold * threshold);
}

bool Sketch2DView::isNearEdgeMidpoint(const QPointF& p1, const QPointF& p2, const QPointF& pos, qreal threshold) const {
    QPointF midpoint = worldToScreen(getEdgeMidpoint(p1, p2));
    QPointF delta = pos - midpoint;
    return (delta.x() * delta.x() + delta.y() * delta.y()) < (threshold * threshold);
}

Sketch2DView::Edge Sketch2DView::findNearbyEdgeMidpoint(const QPointF& pos, qreal threshold) const {
    // Check polygons
    for (int i = 0; i < m_polygons.size(); ++i) {
        const auto& poly = m_polygons[i];
        for (int j = 0; j < poly.vertices.size(); ++j) {
            int next = (j + 1) % poly.vertices.size();
            const auto& v1 = poly.vertices[j];
            const auto& v2 = poly.vertices[next];

            QPointF checkPoint;
            if (qAbs(v1.bulge) < 1e-6) {
                checkPoint = getEdgeMidpoint(v1.point, v2.point);
            } else {
                checkPoint = arcPointFromBulge(v1.point, v2.point, v1.bulge);
            }

            if (isNearPoint(checkPoint, pos, threshold)) {
                return Edge{ i, j, next, true };
            }
        }
    }
    // Check polylines (don't wrap around)
    for (int i = 0; i < m_polylines.size(); ++i) {
        const auto& poly = m_polylines[i];
        for (int j = 0; j < poly.vertices.size() - 1; ++j) {
            int next = j + 1;
            const auto& v1 = poly.vertices[j];
            const auto& v2 = poly.vertices[next];

            QPointF checkPoint;
            if (qAbs(v1.bulge) < 1e-6) {
                checkPoint = getEdgeMidpoint(v1.point, v2.point);
            } else {
                checkPoint = arcPointFromBulge(v1.point, v2.point, v1.bulge);
            }

            if (isNearPoint(checkPoint, pos, threshold)) {
                return Edge{ i, j, next, false };
            }
        }
    }
    return Edge{ -1, -1, -1, false };
}

Sketch2DView::ResultEdgeLoc Sketch2DView::findNearbyResultEdge(const QPointF& screenPos, qreal threshold) const {
	ResultEdgeLoc result;

	// 只在去自交结果中查找
	for (int pi = 0; pi < m_deselfIntersectionResults.size(); ++pi) {
		const auto& poly = m_deselfIntersectionResults[pi];
		if (poly.vertices.isEmpty()) continue;

		for (int ei = 0; ei < poly.vertices.size(); ++ei) {
			int next = (ei + 1) % poly.vertices.size();
			const auto& v1 = poly.vertices[ei];
			const auto& v2 = poly.vertices[next];

			qreal distToEdge = -1.0;
			if (qAbs(v1.bulge) < 1e-6) {
				// 直线边：计算鼠标到线段的最近距离（世界坐标）
				distToEdge = pointToSegmentDistWorld(screenPos, v1.point, v2.point);
			} else {
				// 弧线边：计算鼠标到弧段的最近距离（世界坐标）
				ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
				distToEdge = pointToArcDistWorld(screenPos, v1.point, v2.point, arc);
			}

			// 转换为屏幕坐标的阈值判断
			qreal screenThreshold = threshold / m_scale;
			if (distToEdge >= 0 && distToEdge <= screenThreshold) {
				result.polygonIndex = pi;
				result.edgeIndex = ei;
				result.bulge = v1.bulge;
				// 获取溯源 segmentId 和 sourceEdgeId
				if (ei < poly.edgeSegmentIds.size()) {
					result.segmentId = poly.edgeSegmentIds[ei];
				}
				if (ei < poly.edgeSourceEdgeIds.size()) {
					result.sourceEdgeId = poly.edgeSourceEdgeIds[ei];
				}
				return result;
			}
		}
	}
	return result;
}

qreal Sketch2DView::pointToSegmentDistWorld(const QPointF& screenPos, const QPointF& a, const QPointF& b) const {
	QPointF p = screenToWorld(screenPos);

	QPointF ab = b - a;
	qreal abLenSq = ab.x() * ab.x() + ab.y() * ab.y();
	if (abLenSq < 1e-12) {
		QPointF delta = p - a;
		return std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
	}

	// 投影参数 t，钳制到 [0, 1]
	qreal t = QPointF::dotProduct(p - a, ab) / abLenSq;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;

	QPointF closest = a + t * ab;
	QPointF delta = p - closest;
	return std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
}

qreal Sketch2DView::pointToArcDistWorld(const QPointF& screenPos, const QPointF& p1, const QPointF& p2,
                                        const ArcSegment& arc) const {
	QPointF p = screenToWorld(screenPos);

	// 计算 p 相对于圆心的角度
	qreal pAngle = angleDegAt(arc.center, p);
	qreal distToCenter = QLineF(arc.center, p).length();

	// 将角度标准化到 [0, 360)
	auto normAngle = [](qreal a) {
		while (a < 0) a += 360.0;
		while (a >= 360.0) a -= 360.0;
		return a;
	};

	qreal start = normAngle(arc.startAngleDeg);
	qreal span = arc.spanAngleDeg;
	qreal end = normAngle(start + span);
	pAngle = normAngle(pAngle);

	// 判断 pAngle 是否在弧段的角度范围内
	bool inArcSpan;
	if (span >= 0) {
		inArcSpan = (start <= end) ? (pAngle >= start && pAngle <= end)
		                           : (pAngle >= start || pAngle <= end);
	} else {
		inArcSpan = (end <= start) ? (pAngle <= start && pAngle >= end)
		                           : (pAngle <= start || pAngle >= end);
	}

	if (inArcSpan) {
		// 在弧段内，距离 = |到圆心距离 - 半径|
		return std::abs(distToCenter - arc.radius);
	} else {
		// 在弧段外，取到两端点的最小距离
		qreal d1 = QLineF(p, p1).length();
		qreal d2 = QLineF(p, p2).length();
		return (d1 < d2) ? d1 : d2;
	}
}

Sketch2DView::Edge Sketch2DView::findNearbyVertex(const QPointF& pos, qreal threshold) const {
    // Check polygons first
    for (int i = 0; i < m_polygons.size(); ++i) {
        const auto& poly = m_polygons[i];
        for (int j = 0; j < poly.vertices.size(); ++j) {
            if (isNearPoint(poly.vertices[j].point, pos, threshold)) {
                return Edge{ i, j, -1, true };
            }
        }
    }
    // Check polylines
    for (int i = 0; i < m_polylines.size(); ++i) {
        const auto& poly = m_polylines[i];
        for (int j = 0; j < poly.vertices.size(); ++j) {
            if (isNearPoint(poly.vertices[j].point, pos, threshold)) {
                return Edge{ i, j, -1, false };
            }
        }
    }
    return Edge{ -1, -1, -1, false };
}

bool Sketch2DView::computeArcThrough3Points(const QPointF& p1, const QPointF& p2, const QPointF& p3, ArcSegment& outArc) const {
    const qreal x1 = p1.x(), y1 = p1.y();
    const qreal x2 = p2.x(), y2 = p2.y();
    const qreal x3 = p3.x(), y3 = p3.y();

    const qreal a = x1 - x2;
    const qreal b = y1 - y2;
    const qreal c = x1 - x3;
    const qreal d = y1 - y3;

    const qreal e = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0;
    const qreal f = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0;

    const qreal det = a * d - b * c;
    if (qAbs(det) < 1e-6) {
        return false;
    }

    const qreal cx = (d * e - b * f) / det;
    const qreal cy = (-c * e + a * f) / det;

    const QPointF center(cx, cy);
    const qreal r = QLineF(center, p1).length();
    if (r < 1e-6) return false;

    const qreal startDeg = angleDegAt(center, p1);
    const qreal midDeg = angleDegAt(center, p3);
    const qreal endDeg = angleDegAt(center, p2);

    qreal span1 = endDeg - startDeg;
    while (span1 <= -360.0) span1 += 360.0;
    while (span1 > 360.0) span1 -= 360.0;

    qreal span2 = (span1 > 0) ? (span1 - 360.0) : (span1 + 360.0);

    auto isAngleOnArc = [](qreal start, qreal span, qreal ang) {
        auto norm = [](qreal v) {
            while (v < 0) v += 360.0;
            while (v >= 360.0) v -= 360.0;
            return v;
        };
        start = norm(start);
        ang = norm(ang);

        if (qFuzzyIsNull(span)) return false;
        if (span > 0) {
            qreal end = norm(start + span);
            if (start <= end) return (ang >= start && ang <= end);
            return (ang >= start || ang <= end);
        } else {
            qreal end = norm(start + span);
            if (end <= start) return (ang <= start && ang >= end);
            return (ang <= start || ang >= end);
        }
    };

    const bool midOn1 = isAngleOnArc(startDeg, span1, midDeg);
    const qreal span = midOn1 ? span1 : span2;

    outArc.center = center;
    outArc.radius = r;
    outArc.startAngleDeg = startDeg;
    outArc.spanAngleDeg = span;
    return true;
}

void Sketch2DView::mousePressEvent(QMouseEvent* event) {
    const QPointF p = snapToPixelCenter(screenToWorld(event->position()));
    const QPointF screenPos = event->position();

    // Middle mouse button starts panning (always allowed, even in read-only mode)
    if (event->button() == Qt::MiddleButton) {
        m_isPanning = true;
        m_panStart = event->position();
        setCursor(Qt::ClosedHandCursor);
        QWidget::mousePressEvent(event);
        return;
    }

    // In read-only mode, only allow panning (middle mouse) and zooming (wheel)
    if (m_readOnly) {
        QWidget::mousePressEvent(event);
        return;
    }

    // Only handle left and right buttons for polygon interaction
    if (event->button() != Qt::LeftButton && event->button() != Qt::RightButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    // Reset drag flag when pressing right button
    if (event->button() == Qt::RightButton) {
        m_hasDragged = false;
    }

    // Polygon tool: check for vertex or edge midpoint interaction
    if (m_tool == Tool::Polyline || m_tool == Tool::Polygon) {
        // First check if we're clicking on a vertex (for dragging)
        Edge vertexEdge = findNearbyVertex(screenPos, 8.0);
        if (vertexEdge.polygonIndex >= 0) {
            m_dragMode = DragMode::Vertex;
            m_draggedEdge = vertexEdge;
            m_dragStartPos = screenPos;
            m_dragButton = event->button();
            if (vertexEdge.isPolygon) {
                m_originalPoint = m_polygons[vertexEdge.polygonIndex].vertices[vertexEdge.vertexIndex1].point;
            } else {
                m_originalPoint = m_polylines[vertexEdge.polygonIndex].vertices[vertexEdge.vertexIndex1].point;
            }
            setSelectedPolygon(vertexEdge.polygonIndex, vertexEdge.isPolygon);
            return;
        }

        // Check if we're clicking on an edge midpoint
        Edge edge = findNearbyEdgeMidpoint(screenPos, 8.0);
        if (edge.polygonIndex >= 0) {
            m_dragMode = DragMode::EdgeMidpoint;
            m_draggedEdge = edge;
            m_dragStartPos = screenPos;
            m_dragButton = event->button();
            m_edgeToArc = edge;
            setSelectedPolygon(edge.polygonIndex, edge.isPolygon);
            return;
        }

        // If not interacting with existing shape, do nothing
        // Creation is now done via context menu
        QWidget::mousePressEvent(event);
        return;
    }
}

void Sketch2DView::mouseMoveEvent(QMouseEvent* event) {
    const QPointF screenPos = event->position();
    m_mousePos = screenPos;

    // 总是触发更新以实时显示鼠标坐标
    update();

    // Handle panning
    if (m_isPanning) {
        QPointF delta = event->position() - m_panStart;
        m_offset -= QPointF(delta.x() / m_scale, -delta.y() / m_scale);
        m_panStart = event->position();
        update();
        QWidget::mouseMoveEvent(event);
        return;
    }

    // Handle polygon/polyline vertex dragging
    if (m_dragMode == DragMode::Vertex && event->buttons() & (m_dragButton == Qt::LeftButton ? Qt::LeftButton : Qt::RightButton)) {
        auto& poly = m_draggedEdge.isPolygon ? m_polygons[m_draggedEdge.polygonIndex] : m_polylines[m_draggedEdge.polygonIndex];
        QPointF deltaScreen = screenPos - m_dragStartPos;
        QPointF deltaWorld(deltaScreen.x() / m_scale, -deltaScreen.y() / m_scale);
        poly.vertices[m_draggedEdge.vertexIndex1].point = m_originalPoint + deltaWorld;

        // Mark as dragged if using right button
        if (m_dragButton == Qt::RightButton) {
            m_hasDragged = true;
        }

        if (m_draggedEdge.isPolygon) {
            emit polygonModified();
        } else {
            emit polylineModified();
        }
        update();
        QWidget::mouseMoveEvent(event);
        return;
    }

    // Handle edge midpoint dragging
    if (m_dragMode == DragMode::EdgeMidpoint && event->buttons() & (m_dragButton == Qt::LeftButton ? Qt::LeftButton : Qt::RightButton)) {
        auto& poly = m_draggedEdge.isPolygon ? m_polygons[m_draggedEdge.polygonIndex] : m_polylines[m_draggedEdge.polygonIndex];

        QPointF edgeStart = poly.vertices[m_draggedEdge.vertexIndex1].point;
        QPointF edgeEnd = poly.vertices[m_draggedEdge.vertexIndex2].point;
        const QPointF p = screenToWorld(screenPos);

        if (m_dragButton == Qt::RightButton) {
            // Right click: Directly modify bulge (arc conversion)
            m_arcThroughPoint = p;
            qreal newBulge = bulgeFromArc(edgeStart, edgeEnd, m_arcThroughPoint);
            poly.vertices[m_draggedEdge.vertexIndex1].bulge = newBulge;

            // Mark as dragged
            m_hasDragged = true;

            if (m_draggedEdge.isPolygon) {
                emit polygonModified();
            } else {
                emit polylineModified();
            }
        } else {
            // Left click: Always split the edge (no direction judgment)
            if (m_splitVertexIndex == -1) {
                // First movement: always split the edge
                m_splitVertexIndex = m_draggedEdge.vertexIndex2;

                QPointF midpoint;
                qreal halfBulge;

                const qreal originalBulge = poly.vertices[m_draggedEdge.vertexIndex1].bulge;
                const QColor originalEdgeColor = poly.vertices[m_draggedEdge.vertexIndex1].edgeColor;

                if (qAbs(originalBulge) < 1e-6) {
                    midpoint = getEdgeMidpoint(edgeStart, edgeEnd);
                    halfBulge = 0.0;
                    poly.vertices[m_draggedEdge.vertexIndex1].bulge = 0.0;
                } else {
                    midpoint = arcPointFromBulge(edgeStart, edgeEnd, originalBulge);
                    halfBulge = splitArcBulge(originalBulge);
                    poly.vertices[m_draggedEdge.vertexIndex1].bulge = halfBulge;
                }

                PolygonVertex newVertex;
                newVertex.point = midpoint;
                newVertex.bulge = halfBulge;
                newVertex.edgeColor = originalEdgeColor;
                poly.vertices.insert(m_splitVertexIndex, newVertex);

                // Set up to drag the newly inserted vertex
                m_originalPoint = midpoint;
                m_dragStartPos = screenPos;
                m_dragMode = DragMode::Vertex;
                m_draggedEdge.vertexIndex1 = m_splitVertexIndex;
                m_draggedEdge.vertexIndex2 = -1;

                if (m_draggedEdge.isPolygon) {
                    emit polygonModified();
                } else {
                    emit polylineModified();
                }
            } else {
                // Continue splitting/dragging
                poly.vertices[m_splitVertexIndex].point = p;
                if (m_draggedEdge.isPolygon) {
                    emit polygonModified();
                } else {
                    emit polylineModified();
                }
            }
        }
        update();
        QWidget::mouseMoveEvent(event);
        return;
    }

    // 只读模式下检测结果边悬停（用于第四视图偏置溯源交互）
    if (m_readOnly && !m_deselfIntersectionResults.isEmpty()) {
        ResultEdgeLoc hovered = findNearbyResultEdge(screenPos, 12.0);
        bool changed = (hovered.polygonIndex != m_hoveredResultEdge.polygonIndex ||
                        hovered.edgeIndex != m_hoveredResultEdge.edgeIndex);
        if (changed) {
            if (hovered.polygonIndex >= 0) {
                m_hoveredResultEdge = hovered;
                emit resultEdgeHovered(hovered.polygonIndex, hovered.edgeIndex,
                    hovered.segmentId, hovered.sourceEdgeId, hovered.bulge);
            } else {
                m_hoveredResultEdge = ResultEdgeLoc{};
                emit resultEdgeHoverEnded();
            }
        }
    }

    QWidget::mouseMoveEvent(event);
}

void Sketch2DView::leaveEvent(QEvent* event) {
    m_mousePos = QPointF();
    // 清除结果边悬停状态
    if (m_hoveredResultEdge.polygonIndex >= 0) {
        m_hoveredResultEdge = ResultEdgeLoc{};
        emit resultEdgeHoverEnded();
    }
    update();
    QWidget::leaveEvent(event);
}

void Sketch2DView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton && m_isPanning) {
        m_isPanning = false;
        setCursor(Qt::ArrowCursor);
    }

    // Reset drag state on left or right button release
    if (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) {
        m_dragMode = DragMode::None;
        m_splitVertexIndex = -1;
        m_dragButton = Qt::NoButton;
        m_edgeToArc = Edge{ -1, -1, -1 };
        m_arcThroughPoint = QPointF();
    }

    QWidget::mouseReleaseEvent(event);
}

void Sketch2DView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Apply transform
    painter.translate(width() / 2.0, height() / 2.0);
    painter.scale(m_scale, -m_scale); // Y轴向上
    painter.translate(-m_offset.x(), -m_offset.y());

    // Background grid
    const int grid = 25;
    painter.save();
    painter.setPen(QPen(QColor(40, 40, 40), 1));
    QRectF worldBounds = getWorldBounds();
    for (int x = qFloor(worldBounds.left() / grid) * grid; x <= worldBounds.right(); x += grid) {
        painter.drawLine(x, worldBounds.top(), x, worldBounds.bottom());
    }
    for (int y = qFloor(worldBounds.top() / grid) * grid; y <= worldBounds.bottom(); y += grid) {
        painter.drawLine(worldBounds.left(), y, worldBounds.right(), y);
    }
    painter.restore();

    // Axes
    painter.save();
    painter.setPen(QPen(QColor(80, 80, 80), 1));
    painter.drawLine(0, worldBounds.top(), 0, worldBounds.bottom());
    painter.drawLine(worldBounds.left(), 0, worldBounds.right(), 0);
    painter.restore();

    // Draw polylines
    painter.save();
    for (int i = 0; i < m_polylines.size(); ++i) {
        const auto& poly = m_polylines[i];

        QPen pen;
        QColor penColor;
        qreal penWidth = 2 / m_scale;

        if (m_selectedPolylines.contains(i)) {
            penColor = QColor(255, 215, 0); // 选中：金色
            penWidth = 3 / m_scale;
        } else {
            penColor = QColor(230, 230, 230); // 默认：灰色
        }

        pen = QPen(penColor, penWidth);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);

        if (poly.vertices.size() > 0) {
            QPointF startPoint = poly.vertices[0].point;
            QPainterPath path;
            path.moveTo(startPoint);

            for (int j = 0; j < poly.vertices.size() - 1; ++j) {
                int next = j + 1;
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];

                if (qAbs(v1.bulge) < 1e-6) {
                    path.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius,
                        arc.center.y() - arc.radius,
                        arc.radius * 2.0,
                        arc.radius * 2.0);
                    path.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }
            painter.drawPath(path);
        }
    }
    painter.restore();

    // 主视图：橙色轮廓（先渲染，被多边形填充/边线覆盖，仅露出外层轮廓）
    if (!m_highlightedSourceSegmentIds.isEmpty() && !m_polygons.isEmpty()) {
        painter.save();
        painter.setBrush(Qt::NoBrush);

        int cumulativeEdgeIdx = 0;
        for (int pi = 0; pi < m_polygons.size(); ++pi) {
            const auto& poly = m_polygons[pi];
            int n = poly.vertices.size();

            for (int ei = 0; ei < n; ++ei) {
                int globalIdx = cumulativeEdgeIdx + ei;
                if (!m_highlightedSourceSegmentIds.contains(globalIdx)) continue;

                int next = (ei + 1) % n;
                const auto& v1 = poly.vertices[ei];
                const auto& v2 = poly.vertices[next];

                QPen hlPen(QColor(255, 140, 0), 8 / m_scale);
                hlPen.setCapStyle(Qt::RoundCap);
                hlPen.setJoinStyle(Qt::RoundJoin);
                painter.setPen(hlPen);
                if (qAbs(v1.bulge) < 1e-6) {
                    painter.drawLine(v1.point, v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    QPainterPath edgePath;
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    edgePath.moveTo(v1.point);
                    edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                    painter.drawPath(edgePath);
                }
            }
            cumulativeEdgeIdx += n;
        }
        painter.restore();
    }

    // Draw polygons
    painter.save();
    for (int i = 0; i < m_polygons.size(); ++i) {
        const auto& poly = m_polygons[i];

        QPen pen;
        QColor penColor;
        QColor brushColor;
        qreal penWidth = 2 / m_scale;

        if (m_selectedPolygons.contains(i)) {
            penColor = QColor(255, 215, 0); // 选中：金色
            penWidth = 3 / m_scale;
            brushColor = QColor(255, 215, 0, 30);
        } else {
            penColor = QColor(144, 238, 144); // 默认：绿色
            brushColor = QColor(144, 238, 144, 50);
        }

        pen = QPen(penColor, penWidth);
        painter.setPen(pen);

        if (poly.vertices.size() > 0) {
            // 先绘制填充
            QPainterPath fillPath;
            QPointF startPoint = poly.vertices[0].point;
            fillPath.moveTo(startPoint);

            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];

                if (qAbs(v1.bulge) < 1e-6) {
                    fillPath.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius,
                        arc.center.y() - arc.radius,
                        arc.radius * 2.0,
                        arc.radius * 2.0);
                    fillPath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }

            fillPath.closeSubpath();
            painter.setPen(Qt::NoPen);
            if (brushColor.isValid() && brushColor.alpha() > 0) {
                painter.setBrush(brushColor);
            } else {
                painter.setBrush(Qt::NoBrush);
            }
            painter.drawPath(fillPath);

            // 然后每条边单独绘制边界
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];

                QColor edgePenColor = penColor;
                if (v1.edgeColor != QColor(255, 255, 255)) {
                    edgePenColor = v1.edgeColor;
                }
                QPen edgePen(edgePenColor, penWidth);
                painter.setPen(edgePen);
                painter.setBrush(Qt::NoBrush);

                if (qAbs(v1.bulge) < 1e-6) {
                    painter.drawLine(v1.point, v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    QPainterPath edgePath;
                    const QRectF rect(arc.center.x() - arc.radius,
                        arc.center.y() - arc.radius,
                        arc.radius * 2.0,
                        arc.radius * 2.0);
                    edgePath.moveTo(v1.point);
                    edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                    painter.drawPath(edgePath);
                }
            }
        }
    }
    painter.restore();

    // Draw polyline vertices and edge midpoints (for editing)
    if (!m_readOnly) {
        painter.save();
        for (int i = 0; i < m_polylines.size(); ++i) {
            const auto& poly = m_polylines[i];

            // Draw vertices
            painter.setPen(QPen(Qt::NoPen));
            for (const auto& v : poly.vertices) {
                painter.setBrush(QColor(230, 230, 230));
                painter.drawEllipse(v.point, 4 / m_scale, 4 / m_scale);
            }

            // Draw edge midpoints
            painter.setBrush(QColor(255, 255, 0));
            for (int j = 0; j < poly.vertices.size() - 1; ++j) {
                int next = j + 1;
                QPointF p1 = poly.vertices[j].point;
                QPointF p2 = poly.vertices[next].point;
                qreal bulge = poly.vertices[j].bulge;
                if (qAbs(bulge) < 1e-6) {
                    QPointF mid = getEdgeMidpoint(p1, p2);
                    painter.drawEllipse(mid, 3 / m_scale, 3 / m_scale);
                } else {
                    QPointF arcPoint = arcPointFromBulge(p1, p2, bulge);
                    painter.setBrush(QColor(255, 150, 0));
                    painter.drawEllipse(arcPoint, 3 / m_scale, 3 / m_scale);
                    painter.setBrush(QColor(255, 255, 0));
                }
            }
        }
        painter.restore();

        // Draw polygon vertices and edge midpoints (for editing)
        painter.save();
        for (int i = 0; i < m_polygons.size(); ++i) {
            const auto& poly = m_polygons[i];

            // Draw vertices
            painter.setPen(QPen(Qt::NoPen));
            for (const auto& v : poly.vertices) {
                painter.setBrush(QColor(144, 238, 144));
                painter.drawEllipse(v.point, 4 / m_scale, 4 / m_scale);
            }

            // Draw edge midpoints
            painter.setBrush(QColor(255, 255, 0));
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                QPointF p1 = poly.vertices[j].point;
                QPointF p2 = poly.vertices[next].point;
                qreal bulge = poly.vertices[j].bulge;
                if (qAbs(bulge) < 1e-6) {
                    QPointF mid = getEdgeMidpoint(p1, p2);
                    painter.drawEllipse(mid, 3 / m_scale, 3 / m_scale);
                } else {
                    QPointF arcPoint = arcPointFromBulge(p1, p2, bulge);
                    painter.setBrush(QColor(255, 150, 0));
                    painter.drawEllipse(arcPoint, 3 / m_scale, 3 / m_scale);
                    painter.setBrush(QColor(255, 255, 0));
                }
            }
        }
        painter.restore();
    }

    // Draw fill results (different colors for each polygon, for TopRightView)
    if (!m_fillResults.isEmpty()) {
        painter.save();
        for (const auto& poly : m_fillResults) {
            if (poly.vertices.isEmpty()) continue;

            // 使用多边形自己的颜色，如果没有则使用随机颜色
            QColor baseColor = poly.color;
            if (!baseColor.isValid()) {
                // 生成随机颜色
                static std::vector<QColor> palette = {
                    QColor(255, 100, 100),  // 红色
                    QColor(100, 200, 100),  // 绿色
                    QColor(100, 100, 255),  // 蓝色
                    QColor(255, 200, 100),  // 橙色
                    QColor(200, 100, 255),  // 紫色
                    QColor(100, 255, 200),  // 青色
                    QColor(255, 150, 150),  // 浅红
                    QColor(150, 255, 150),  // 浅绿
                };
                static int colorIndex = 0;
                baseColor = palette[colorIndex % palette.size()];
                colorIndex++;
            }

            QColor fillColor = poly.isHole ? QColor(baseColor.red(), baseColor.green(), baseColor.blue(), 40)
                                           : QColor(baseColor.red(), baseColor.green(), baseColor.blue(), 80);
            QColor penColor = poly.isHole ? QColor(baseColor.red(), baseColor.green(), baseColor.blue(), 200)
                                         : baseColor;

            QPainterPath fillPath;
            fillPath.moveTo(poly.vertices[0].point);
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];
                if (qAbs(v1.bulge) < 1e-6) {
                    fillPath.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    fillPath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }
            fillPath.closeSubpath();

            painter.setPen(Qt::NoPen);
            painter.setBrush(fillColor);
            painter.drawPath(fillPath);

            painter.setPen(QPen(penColor, 2 / m_scale));
            painter.setBrush(Qt::NoBrush);
            painter.drawPath(fillPath);
        }
        painter.restore();
    }

    // 绘制高亮的源边（segmentId 匹配的 fillResult 边，用于偏置溯源）
    // 第二/三/四视图：高亮 fillResults 中匹配的打断边
    if (!m_highlightedSourceSegmentIds.isEmpty() && !m_fillResults.isEmpty()) {
        painter.save();
        painter.setBrush(Qt::NoBrush);
        for (const auto& poly : m_fillResults) {
            if (poly.vertices.isEmpty()) continue;

            for (int j = 0; j < poly.vertices.size(); ++j) {
                int segId = (j < poly.edgeSegmentIds.size()) ? poly.edgeSegmentIds[j] : -1;
                if (segId < 0 || !m_highlightedSourceSegmentIds.contains(segId)) continue;

                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];

                // 亮金色外圈光晕
                QPen outerPen(QColor(255, 215, 0, 100), 8 / m_scale);
                outerPen.setCapStyle(Qt::RoundCap);
                outerPen.setJoinStyle(Qt::RoundJoin);
                painter.setPen(outerPen);

                if (qAbs(v1.bulge) < 1e-6) {
                    painter.drawLine(v1.point, v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    QPainterPath edgePath;
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    edgePath.moveTo(v1.point);
                    edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                    painter.drawPath(edgePath);
                }

                // 亮金色内线
                QPen innerPen(QColor(255, 215, 0), 4 / m_scale);
                innerPen.setCapStyle(Qt::RoundCap);
                innerPen.setJoinStyle(Qt::RoundJoin);
                painter.setPen(innerPen);

                if (qAbs(v1.bulge) < 1e-6) {
                    painter.drawLine(v1.point, v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    QPainterPath edgePath;
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    edgePath.moveTo(v1.point);
                    edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                    painter.drawPath(edgePath);
                }

                // 端点标记
                painter.setPen(Qt::NoPen);
                painter.setBrush(QColor(255, 215, 0));
                painter.drawEllipse(v1.point, 3 / m_scale, 3 / m_scale);
            }
        }
        painter.restore();
    }

    // Draw self-intersection processing results (orange, for reference)
    if (!m_selfIntersectionResults.isEmpty()) {
        painter.save();
        for (const auto& poly : m_selfIntersectionResults) {
            if (poly.vertices.isEmpty()) continue;

            QColor fillColor = poly.isHole ? QColor(255, 165, 0, 30) : QColor(255, 165, 0, 50);
            QColor penColor = poly.isHole ? QColor(255, 165, 0, 180) : poly.color.isValid() ? poly.color : QColor(255, 165, 0);

            QPainterPath fillPath;
            fillPath.moveTo(poly.vertices[0].point);
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];
                if (qAbs(v1.bulge) < 1e-6) {
                    fillPath.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    fillPath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }
            fillPath.closeSubpath();

            painter.setPen(Qt::NoPen);
            painter.setBrush(fillColor);
            painter.drawPath(fillPath);

            painter.setPen(QPen(penColor, 2 / m_scale));
            painter.setBrush(Qt::NoBrush);
            painter.drawPath(fillPath);
        }
        painter.restore();
    }

    // Draw offset results (blue, for BottomLeftView)
    if (!m_offsetResults.isEmpty()) {
        painter.save();
        for (const auto& poly : m_offsetResults) {
            if (poly.vertices.isEmpty()) continue;

            QColor fillColor = poly.isHole ? QColor(100, 149, 237, 30) : QColor(100, 149, 237, 50);
            QColor penColor = poly.isHole ? QColor(100, 149, 237, 180) : poly.color.isValid() ? poly.color : QColor(100, 149, 237);

            QPainterPath fillPath;
            fillPath.moveTo(poly.vertices[0].point);
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];
                if (qAbs(v1.bulge) < 1e-6) {
                    fillPath.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    fillPath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }
            fillPath.closeSubpath();

            painter.setPen(Qt::NoPen);
            painter.setBrush(fillColor);
            painter.drawPath(fillPath);

            painter.setPen(QPen(penColor, 2 / m_scale));
            painter.setBrush(Qt::NoBrush);
            painter.drawPath(fillPath);
        }
        painter.restore();
    }

    // Draw deself-intersection results (purple, for BottomRightView)
    if (!m_deselfIntersectionResults.isEmpty()) {
        painter.save();
        for (const auto& poly : m_deselfIntersectionResults) {
            if (poly.vertices.isEmpty()) continue;

            QColor fillColor = poly.isHole ? QColor(138, 43, 226, 30) : QColor(138, 43, 226, 50);
            QColor penColor = poly.isHole ? QColor(138, 43, 226, 180) : poly.color.isValid() ? poly.color : QColor(138, 43, 226);

            QPainterPath fillPath;
            fillPath.moveTo(poly.vertices[0].point);
            for (int j = 0; j < poly.vertices.size(); ++j) {
                int next = (j + 1) % poly.vertices.size();
                const auto& v1 = poly.vertices[j];
                const auto& v2 = poly.vertices[next];
                if (qAbs(v1.bulge) < 1e-6) {
                    fillPath.lineTo(v2.point);
                } else {
                    ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                    const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                        arc.radius * 2.0, arc.radius * 2.0);
                    fillPath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                }
            }
            fillPath.closeSubpath();

            painter.setPen(Qt::NoPen);
            painter.setBrush(fillColor);
            painter.drawPath(fillPath);

            painter.setPen(QPen(penColor, 2 / m_scale));
            painter.setBrush(Qt::NoBrush);
            painter.drawPath(fillPath);
        }
        painter.restore();
    }

    // 绘制悬停的高亮偏置边（用于第四视图溯源交互）
    if (m_hoveredResultEdge.polygonIndex >= 0 &&
        m_hoveredResultEdge.polygonIndex < m_deselfIntersectionResults.size()) {
        const auto& poly = m_deselfIntersectionResults[m_hoveredResultEdge.polygonIndex];
        int ei = m_hoveredResultEdge.edgeIndex;
        if (ei >= 0 && ei < poly.vertices.size()) {
            painter.save();
            painter.setBrush(Qt::NoBrush);
            int next = (ei + 1) % poly.vertices.size();
            const auto& v1 = poly.vertices[ei];
            const auto& v2 = poly.vertices[next];

            // 外发光效果：宽暗色笔
            QPen glowPen(QColor(0, 255, 255, 80), 8 / m_scale);
            glowPen.setCapStyle(Qt::RoundCap);
            glowPen.setJoinStyle(Qt::RoundJoin);
            painter.setPen(glowPen);

            if (qAbs(v1.bulge) < 1e-6) {
                painter.drawLine(v1.point, v2.point);
            } else {
                ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                QPainterPath edgePath;
                const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                    arc.radius * 2.0, arc.radius * 2.0);
                edgePath.moveTo(v1.point);
                edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                painter.drawPath(edgePath);
            }

            // 内层亮色笔
            QPen hlPen(QColor(0, 255, 255), 3 / m_scale);
            hlPen.setCapStyle(Qt::RoundCap);
            hlPen.setJoinStyle(Qt::RoundJoin);
            painter.setPen(hlPen);

            if (qAbs(v1.bulge) < 1e-6) {
                painter.drawLine(v1.point, v2.point);
            } else {
                ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
                QPainterPath edgePath;
                const QRectF rect(arc.center.x() - arc.radius, arc.center.y() - arc.radius,
                    arc.radius * 2.0, arc.radius * 2.0);
                edgePath.moveTo(v1.point);
                edgePath.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
                painter.drawPath(edgePath);
            }

            painter.restore();
        }
    }

    // 高亮源顶点（橙色圆点，凸点连接弧溯源，绘制在所有结果之上）
    if (!m_highlightedVertices.isEmpty()) {
        painter.save();
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 140, 0));
        for (const auto& pt : m_highlightedVertices) {
            painter.drawEllipse(pt, 6 / m_scale, 6 / m_scale);
        }
        painter.restore();
    }

    // Reset transform for HUD
    painter.resetTransform();

    // HUD - 左下角显示缩放等级，右下角显示鼠标坐标
    painter.save();
    painter.setPen(QPen(QColor(200, 200, 200), 1));
    painter.setBrush(Qt::NoBrush);
    painter.setFont(QFont("Segoe UI", 9));

    // 左下角：缩放等级
    painter.drawText(10, height() - 10, QString("Scale: %1x").arg(m_scale, 0, 'f', 2));

    // 右下角：鼠标坐标
    if (!m_mousePos.isNull()) {
        QPointF worldPos = screenToWorld(m_mousePos);
        QString coordText = QString("X: %1  Y: %2").arg(worldPos.x(), 0, 'f', 2).arg(worldPos.y(), 0, 'f', 2);
        QRectF textRect(0, height() - 20, width() - 10, 15);
        painter.drawText(textRect, Qt::AlignRight | Qt::AlignBottom, coordText);
    }
    painter.restore();
}

void Sketch2DView::wheelEvent(QWheelEvent* event) {
    const qreal delta = event->angleDelta().y();
    const qreal zoomFactor = qPow(1.1, delta / 120.0);

    const qreal oldScale = m_scale;
    m_scale *= zoomFactor;
    m_scale = qBound(0.1, m_scale, 10.0);

    const QPointF screenCenter(width() / 2.0, height() / 2.0);
    const QPointF mousePos = event->position();

    const QPointF oldWorldPos = m_offset + QPointF((mousePos.x() - screenCenter.x()) / oldScale, -(mousePos.y() - screenCenter.y()) / oldScale);
    const QPointF newWorldPos = m_offset + QPointF((mousePos.x() - screenCenter.x()) / m_scale, -(mousePos.y() - screenCenter.y()) / m_scale);

    m_offset += oldWorldPos - newWorldPos;

    update();
    event->accept();
}

QRectF Sketch2DView::getWorldBounds() const {
    const QPointF topLeft = screenToWorld(QPointF(0, 0));
    const QPointF bottomRight = screenToWorld(QPointF(width(), height()));
    return QRectF(qMin(topLeft.x(), bottomRight.x()),
                   qMin(topLeft.y(), bottomRight.y()),
                   qAbs(bottomRight.x() - topLeft.x()),
                   qAbs(bottomRight.y() - topLeft.y()));
}

void Sketch2DView::setSelectedPolygon(int index, bool isPolygon) {
    if (m_selectedPolygonIndex != index) {
        m_selectedPolygonIndex = index;
        m_selectedPolygons.clear();
        m_selectedPolylines.clear();
        if (index >= 0) {
            if (isPolygon) {
                m_selectedPolygons.insert(index);
            } else {
                m_selectedPolylines.insert(index);
            }
        }
        emit selectionChanged(m_selectedPolygonIndex);
        update();
    }
}

void Sketch2DView::addSelectedPolygon(int index) {
    if (!m_selectedPolygons.contains(index)) {
        m_selectedPolygons.insert(index);
        update();
    }
}

void Sketch2DView::removeSelectedPolygon(int index) {
    if (m_selectedPolygons.contains(index)) {
        m_selectedPolygons.remove(index);
        update();
    }
}

void Sketch2DView::addSelectedPolyline(int index) {
    if (!m_selectedPolylines.contains(index)) {
        m_selectedPolylines.insert(index);
        update();
    }
}

void Sketch2DView::removeSelectedPolyline(int index) {
    if (m_selectedPolylines.contains(index)) {
        m_selectedPolylines.remove(index);
        update();
    }
}

void Sketch2DView::clearSelection() {
    if (m_selectedPolygonIndex != -1 || !m_selectedPolygons.isEmpty() || !m_selectedPolylines.isEmpty()) {
        m_selectedPolygonIndex = -1;
        m_selectedPolygons.clear();
        m_selectedPolylines.clear();
        emit selectionChanged(m_selectedPolygonIndex);
        update();
    }
}

void Sketch2DView::deletePolylines(const QSet<int>& indices) {
    if (indices.isEmpty()) {
        return;
    }

    QList<int> sortedIndices = indices.values();
    std::sort(sortedIndices.begin(), sortedIndices.end(), std::greater<int>());

    for (int index : sortedIndices) {
        if (index >= 0 && index < m_polylines.size()) {
            m_polylines.removeAt(index);
            emit polylineRemoved(index);
        }
    }

    // 清除被删除对象的选择
    for (int index : indices) {
        m_selectedPolylines.remove(index);
        if (m_selectedPolygonIndex == index) {
            m_selectedPolygonIndex = -1;
        }
    }

    // 更新选择索引
    QSet<int> updatedSelectedPolylines;
    for (int index : m_selectedPolylines) {
        int newIndex = index - std::count_if(indices.begin(), indices.end(), [index](int deleted) { return deleted < index; });
        updatedSelectedPolylines.insert(newIndex);
    }
    m_selectedPolylines = updatedSelectedPolylines;

    update();
}

void Sketch2DView::deletePolygons(const QSet<int>& indices) {
    if (indices.isEmpty()) {
        return;
    }

    QList<int> sortedIndices = indices.values();
    std::sort(sortedIndices.begin(), sortedIndices.end(), std::greater<int>());

    for (int index : sortedIndices) {
        if (index >= 0 && index < m_polygons.size()) {
            m_polygons.removeAt(index);
            emit polygonRemoved(index);
        }
    }

    // 清除被删除对象的选择
    for (int index : indices) {
        m_selectedPolygons.remove(index);
        if (m_selectedPolygonIndex == index) {
            m_selectedPolygonIndex = -1;
        }
    }

    // 更新选择索引
    QSet<int> updatedSelectedPolygons;
    for (int index : m_selectedPolygons) {
        int newIndex = index - std::count_if(indices.begin(), indices.end(), [index](int deleted) { return deleted < index; });
        updatedSelectedPolygons.insert(newIndex);
    }
    m_selectedPolygons = updatedSelectedPolygons;

    update();
}

void Sketch2DView::setReadOnly(bool readOnly) {
    m_readOnly = readOnly;
    if (readOnly) {
        m_dragMode = DragMode::None;
        m_dragButton = Qt::NoButton;
        m_arcThroughPoint = QPointF();
        clearSelection();
    }
    update();
}

void Sketch2DView::setPolygons(const QVector<Polygon>& polygons) {
    m_polygons = polygons;
    update();
}

void Sketch2DView::setPolylines(const QVector<Polyline>& polylines) {
    m_polylines = polylines;
    update();
}

void Sketch2DView::setPolygonEdgeColor(int index, const QColor& color, bool isPolygon) {
    if (isPolygon) {
        if (index >= 0 && index < m_polygons.size()) {
            for (auto& v : m_polygons[index].vertices) {
                v.edgeColor = color;
            }
            emit polygonColorChanged(index, color);
            update();
        }
    } else {
        if (index >= 0 && index < m_polylines.size()) {
            for (auto& v : m_polylines[index].vertices) {
                v.edgeColor = color;
            }
            update();
        }
    }
}

void Sketch2DView::setPolygonEdgeColorBatch(const QSet<int>& indices, const QColor& color, bool isPolygon) {
    for (int index : indices) {
        if (isPolygon) {
            if (index >= 0 && index < m_polygons.size()) {
                for (auto& v : m_polygons[index].vertices) {
                    v.edgeColor = color;
                }
            }
        } else {
            if (index >= 0 && index < m_polylines.size()) {
                for (auto& v : m_polylines[index].vertices) {
                    v.edgeColor = color;
                }
            }
        }
    }
    update();
    emit polygonColorChanged(-1, color);
}

void Sketch2DView::contextMenuEvent(QContextMenuEvent* event) {
    if (m_readOnly) {
        QWidget::contextMenuEvent(event);
        return;
    }

    // Don't show context menu if dragging occurred
    if (m_hasDragged) {
        m_hasDragged = false;
        event->ignore();
        return;
    }

    // Store the position for potential creation
    m_contextMenuPosition = snapToPixelCenter(screenToWorld(event->pos()));

    QMenu menu(this);

    // Create polyline action
    if (m_tool == Tool::Polyline) {
        QAction* createAction = menu.addAction("创建多段线");
        connect(createAction, &QAction::triggered, this, [this]() {
            QVector<PolygonVertex> line;
            line.append(PolygonVertex{ m_contextMenuPosition, 0.0 });
            line.append(PolygonVertex{ m_contextMenuPosition + QPointF(100, 0), 0.0 });
            m_polylines.append(Polyline{ line });
            int index = m_polylines.size() - 1;
            m_polylineCounter++;
            emit polylineAdded(index, QString("多段线%1").arg(m_polylineCounter));
            setSelectedPolygon(index, false);
            update();
        });
    }

    // Create polygon action
    if (m_tool == Tool::Polygon) {
        QAction* createAction = menu.addAction("创建多边形");
        connect(createAction, &QAction::triggered, this, [this]() {
            QVector<PolygonVertex> triangle;
            triangle.append(PolygonVertex{ m_contextMenuPosition, 0.0 });
            triangle.append(PolygonVertex{ m_contextMenuPosition + QPointF(100, 0), 0.0 });
            triangle.append(PolygonVertex{ m_contextMenuPosition + QPointF(50, 86.6), 0.0 });
            m_polygons.append(Polygon{ triangle });
            int index = m_polygons.size() - 1;
            m_polygonCounter++;
            emit polygonAdded(index, QString("多边形%1").arg(m_polygonCounter));
            setSelectedPolygon(index, true);
            update();
        });
    }

    // 修改多边形边界颜色
    if (!m_selectedPolygons.isEmpty()) {
        menu.addSeparator();
        QAction* colorAction = menu.addAction("更改边界颜色...");
        connect(colorAction, &QAction::triggered, this, [this]() {
            if (!m_selectedPolygons.isEmpty()) {
                int firstIndex = *m_selectedPolygons.constBegin();
                if (firstIndex >= 0 && firstIndex < m_polygons.size()) {
                    const auto& vertices = m_polygons[firstIndex].vertices;
                    if (!vertices.isEmpty()) {
                        QColor currentColor = vertices[0].edgeColor;
                        QColor newColor = QColorDialog::getColor(currentColor, this, "选择边界颜色");
                        if (newColor.isValid()) {
                            setPolygonEdgeColorBatch(m_selectedPolygons, newColor, true);
                            update();
                        }
                    }
                }
            }
        });
    }

    if (menu.actions().isEmpty()) {
        QWidget::contextMenuEvent(event);
    } else {
        menu.exec(event->globalPos());
    }
}

QPainterPath Sketch2DView::createPolygonPath(const Polygon& poly) const {
    QPainterPath path;
    if (poly.vertices.isEmpty()) {
        return path;
    }

    QPointF startPoint = poly.vertices[0].point;
    path.moveTo(startPoint);

    for (int j = 0; j < poly.vertices.size(); ++j) {
        int next = (j + 1) % poly.vertices.size();
        const auto& v1 = poly.vertices[j];
        const auto& v2 = poly.vertices[next];

        if (qAbs(v1.bulge) < 1e-6) {
            path.lineTo(v2.point);
        } else {
            ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
            const QRectF rect(arc.center.x() - arc.radius,
                arc.center.y() - arc.radius,
                arc.radius * 2.0,
                arc.radius * 2.0);
            path.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
        }
    }

    path.closeSubpath();
    return path;
}

QPainterPath Sketch2DView::createOffsetResultPath(const OffsetResultPolygon& poly) const {
    QPainterPath path;
    if (poly.vertices.isEmpty()) {
        return path;
    }

    QPointF startPoint = poly.vertices[0].point;
    path.moveTo(startPoint);

    for (int j = 0; j < poly.vertices.size(); ++j) {
        int next = (j + 1) % poly.vertices.size();
        const auto& v1 = poly.vertices[j];
        const auto& v2 = poly.vertices[next];

        if (qAbs(v1.bulge) < 1e-6) {
            path.lineTo(v2.point);
        } else {
            ArcSegment arc = arcSegmentFromBulge(v1.point, v2.point, v1.bulge);
            const QRectF rect(arc.center.x() - arc.radius,
                arc.center.y() - arc.radius,
                arc.radius * 2.0,
                arc.radius * 2.0);
            path.arcTo(rect, arc.startAngleDeg, arc.spanAngleDeg);
        }
    }

    path.closeSubpath();
    return path;
}

void Sketch2DView::setOffsetResults(const QVector<OffsetResultPolygon>& results) {
    m_offsetResults = results;
    update();
}

void Sketch2DView::setSelfIntersectionResults(const QVector<OffsetResultPolygon>& results) {
    m_selfIntersectionResults = results;
    update();
}

void Sketch2DView::setDeselfIntersectionResults(const QVector<OffsetResultPolygon>& results) {
    m_deselfIntersectionResults = results;
    update();
}

void Sketch2DView::setFillResults(const QVector<OffsetResultPolygon>& results) {
    m_fillResults = results;
    update();
}

void Sketch2DView::setHighlightedSourceSegmentIds(const QSet<int>& segmentIds) {
    m_highlightedSourceSegmentIds = segmentIds;
    update();
}

void Sketch2DView::clearHighlightedSourceSegmentIds() {
    m_highlightedSourceSegmentIds.clear();
    update();
}

void Sketch2DView::setHighlightedVertices(const QVector<QPointF>& vertices) {
    m_highlightedVertices = vertices;
    update();
}

void Sketch2DView::clearHighlightedVertices() {
    m_highlightedVertices.clear();
    update();
}

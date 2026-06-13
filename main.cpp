#include <QApplication>
#include <QMainWindow>
#include <QSplitter>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <QAction>
#include <QToolBar>
#include <QMenu>
#include <algorithm>
#include <QFileDialog>
#include <QDebug>
#include <QKeySequence>
#include <QBrush>
#include <QColorDialog>
#include <QMessageBox>

#include "Sketch2DView.h"
#include "FourViewContainer.h"
#include "PolygonIO.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    QMainWindow window;
    window.setWindowTitle("TailorOffsetDemo");
    window.resize(1200, 700);

    // 创建主水平分割器
    auto* mainSplitter = new QSplitter(Qt::Horizontal, &window);

    // 创建左侧垂直分割器，用于分隔输入和偏置结果模型树
    auto* leftSplitter = new QSplitter(Qt::Vertical, mainSplitter);
    leftSplitter->setMinimumWidth(150);
    leftSplitter->setMaximumWidth(300);

    // ==================== 输入模型树（上面部分） ====================
    auto* inputModelTree = new QTreeWidget(leftSplitter);
    inputModelTree->setHeaderLabel("输入模型");
    inputModelTree->setSelectionMode(QAbstractItemView::ExtendedSelection);

    // 输入多段线节点
    auto* polylinesItem = new QTreeWidgetItem(inputModelTree);
    polylinesItem->setText(0, "多段线");
    polylinesItem->setExpanded(true);

    // 输入多边形节点
    auto* polygonsItem = new QTreeWidgetItem(inputModelTree);
    polygonsItem->setText(0, "多边形");
    polygonsItem->setExpanded(true);

    // ==================== 偏置结果模型树（下面部分） ====================
    auto* offsetResultsTree = new QTreeWidget(leftSplitter);
    offsetResultsTree->setHeaderLabel("偏置结果");
    offsetResultsTree->setSelectionMode(QAbstractItemView::ExtendedSelection);

    // 偏置结果节点
    auto* offsetResultsItem = new QTreeWidgetItem(offsetResultsTree);
    offsetResultsItem->setText(0, "曲线偏置结果");
    offsetResultsItem->setExpanded(true);

    // 设置左侧分割器的比例（输入在上，偏置结果在下）
    leftSplitter->setStretchFactor(0, 1);  // 输入模型树
    leftSplitter->setStretchFactor(1, 1);  // 偏置结果模型树
    leftSplitter->setSizes({ 200, 200 });

    // 使用 FourViewContainer 替代单个 Sketch2DView
    auto* viewContainer = new FourViewContainer(&window);
    viewContainer->setMinimumSize(800, 300);

    auto* view = viewContainer->mainView(); // 主视图用于工具选择

    mainSplitter->addWidget(leftSplitter);
    mainSplitter->addWidget(viewContainer);
    mainSplitter->setStretchFactor(0, 0);
    mainSplitter->setStretchFactor(1, 1);
    mainSplitter->setSizes({ 300, 1000 });

    window.setCentralWidget(mainSplitter);

    auto* toolbar = window.addToolBar("Tools");
    toolbar->setMovable(false);

    QAction* polygonAction = toolbar->addAction("Polygon");
    polygonAction->setCheckable(true);

    QAction* polylineAction = toolbar->addAction("Polyline");
    polylineAction->setCheckable(true);

    QAction* importAction = toolbar->addAction("Import");
    importAction->setShortcut(QKeySequence("Ctrl+I"));

    // 设置多边形为默认模式
    polygonAction->setChecked(true);
    view->setTool(Sketch2DView::Tool::Polygon);

    QObject::connect(polylineAction, &QAction::triggered, [&]() {
        polylineAction->setChecked(true);
        polygonAction->setChecked(false);
        view->setTool(Sketch2DView::Tool::Polyline);
        });

    QObject::connect(polygonAction, &QAction::triggered, [&]() {
        polygonAction->setChecked(true);
        polylineAction->setChecked(false);
        view->setTool(Sketch2DView::Tool::Polygon);
        });

    QObject::connect(importAction, &QAction::triggered, [&]() {
        QString filePath = QFileDialog::getOpenFileName(nullptr, "导入多边形", "", "Polygon Files (*.txt);;All Files (*)");
        if (filePath.isEmpty()) {
            return;
        }

        std::vector<tailor_visualization::Polygon> importedPolygons;
        bool success = tailor_visualization::PolygonIO::ImportFromFile(filePath, importedPolygons);

        if (!success || importedPolygons.empty()) {
            qDebug() << "导入失败或文件为空!";
            return;
        }

        // 获取当前的多边形，添加新的
        QVector<Sketch2DView::Polygon> currentPolygons = view->polygons();

        for (size_t i = 0; i < importedPolygons.size(); ++i) {
            const auto& polygon = importedPolygons[i];
            Sketch2DView::Polygon newPolygon;
            for (const auto& edge : polygon.edges) {
                Sketch2DView::PolygonVertex vertex;
                vertex.point = QPointF(edge.startPoint.x, edge.startPoint.y);
                vertex.bulge = edge.bulge;
                newPolygon.vertices.append(vertex);
            }

            // 添加到视图
            currentPolygons.append(newPolygon);
        }

        // 更新视图
        view->setPolygons(currentPolygons);

        // 同步更新其他三个视图
        viewContainer->synchronizeViews();

        // 更新模型树
        int startIndex = polygonsItem->childCount();
        for (size_t i = 0; i < importedPolygons.size(); ++i) {
            QString name = QString("多边形%1").arg(startIndex + (int)i + 1);
            auto* item = new QTreeWidgetItem(polygonsItem);
            item->setText(0, name);
            item->setData(0, Qt::UserRole, 1); // 1 = polygon
        }

        qDebug() << "成功导入" << importedPolygons.size() << "个多边形";
        });

    // 多段线添加信号
    QObject::connect(view, &Sketch2DView::polylineAdded, [polylinesItem](int index, const QString& name) {
        auto* item = new QTreeWidgetItem(polylinesItem);
        item->setText(0, name);
        item->setData(0, Qt::UserRole, 0); // 0 = polyline
        });

    // 多边形添加信号
    QObject::connect(view, &Sketch2DView::polygonAdded, [polygonsItem](int index, const QString& name) {
        auto* item = new QTreeWidgetItem(polygonsItem);
        item->setText(0, name);
        item->setData(0, Qt::UserRole, 1); // 1 = polygon
        });

    // ==================== 输入模型树选择变化 ====================
    QObject::connect(inputModelTree, &QTreeWidget::itemSelectionChanged, [inputModelTree, view]() {
        auto selectedItems = inputModelTree->selectedItems();

        // 清除主视图的选择
        view->clearSelection();

        if (selectedItems.isEmpty()) {
            return;
        }

        // 收集需要选中的类型和索引
        int selectedPolylineIndex = -1;
        int selectedPolygonIndex = -1;
        QSet<int> polylineIndices;
        QSet<int> polygonIndices;

        for (QTreeWidgetItem* item : selectedItems) {
            if (!item->parent()) {
                continue;
            }

            int type = item->data(0, Qt::UserRole).toInt();
            int index = item->parent()->indexOfChild(item);

            if (type == 0) { // polyline
                polylineIndices.insert(index);
                selectedPolylineIndex = index;
            } else if (type == 1) { // polygon
                polygonIndices.insert(index);
                selectedPolygonIndex = index;
            }
        }

        // 处理主视图中的多边形/多段线选择
        for (int index : polylineIndices) {
            view->addSelectedPolyline(index);
        }
        for (int index : polygonIndices) {
            view->addSelectedPolygon(index);
        }

        // 如果只有一个选中项，设置为主选择
        if (polylineIndices.size() + polygonIndices.size() == 1) {
            if (selectedPolylineIndex >= 0) {
                view->setSelectedPolygon(selectedPolylineIndex, false);
            } else if (selectedPolygonIndex >= 0) {
                view->setSelectedPolygon(selectedPolygonIndex, true);
            }
        }
        });

    // ==================== 输入模型树右键菜单 ====================
    inputModelTree->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(inputModelTree, &QTreeWidget::customContextMenuRequested, [inputModelTree, view, viewContainer, polylinesItem, polygonsItem](const QPoint& pos) {
        auto selectedItems = inputModelTree->selectedItems();
        if (selectedItems.isEmpty()) {
            return;
        }

        // 检查是否选中了有效的曲线对象（不是父节点）
        bool hasValidSelection = false;
        for (QTreeWidgetItem* item : selectedItems) {
            if (item->parent()) {
                hasValidSelection = true;
                break;
            }
        }

        if (!hasValidSelection) {
            return;
        }

        // 创建右键菜单
        QMenu menu;
        QAction* deleteAction = nullptr;
        QAction* exportAction = nullptr;
        QAction* changeEdgeColorAction = nullptr;

        // 检查是否有选中的多边形
        bool hasSelectedPolygons = false;
        for (QTreeWidgetItem* item : selectedItems) {
            if (item->parent()) {
                int type = item->data(0, Qt::UserRole).toInt();
                if (type == 1) { // polygon
                    hasSelectedPolygons = true;
                    break;
                }
            }
        }

        // 添加菜单项
        deleteAction = menu.addAction("删除");
        exportAction = menu.addAction("导出");
        if (hasSelectedPolygons) {
            changeEdgeColorAction = menu.addAction("更改边界颜色...");
        }

        QAction* result = menu.exec(inputModelTree->mapToGlobal(pos));

        // 处理更改边界颜色操作
        if (result && result == changeEdgeColorAction) {
            // 收集所有选中多边形的索引
            QSet<int> polygonIndices;
            QColor currentColor = QColor(255, 255, 255);

            for (QTreeWidgetItem* item : selectedItems) {
                if (!item->parent()) continue;
                int type = item->data(0, Qt::UserRole).toInt();
                if (type == 1) { // polygon
                    int index = item->parent()->indexOfChild(item);
                    if (index < 0) continue;

                    const auto& polygons = view->polygons();
                    if (index >= polygons.size()) continue;

                    if (!polygons[index].vertices.isEmpty()) {
                        currentColor = polygons[index].vertices[0].edgeColor;
                    }
                    polygonIndices.insert(index);
                }
            }

            if (polygonIndices.isEmpty()) return;

            // 打开颜色选择对话框
            QColor newColor = QColorDialog::getColor(currentColor, inputModelTree, "选择边界颜色");
            if (newColor.isValid()) {
                // 批量设置所有选中多边形的颜色
                view->setPolygonEdgeColorBatch(polygonIndices, newColor, true);
                // 立即同步其他视图
                QCoreApplication::processEvents();
                viewContainer->synchronizeViews();
            }
            return;
        }

        // 处理删除操作
        if (result == deleteAction) {
            // 收集需要删除的索引（使用 QSet）
            QSet<int> polylineIndicesToDelete;
            QSet<int> polygonIndicesToDelete;

            for (QTreeWidgetItem* item : selectedItems) {
                if (item->parent()) {
                    int type = item->data(0, Qt::UserRole).toInt();
                    int index = item->parent()->indexOfChild(item);

                    if (type == 0) { // polyline
                        polylineIndicesToDelete.insert(index);
                    } else if (type == 1) { // polygon
                        polygonIndicesToDelete.insert(index);
                    }
                }
            }

            // 删除视图中的多段线
            view->deletePolylines(polylineIndicesToDelete);

            // 删除视图中的多边形
            view->deletePolygons(polygonIndicesToDelete);

            // 删除模型树中的项（先收集项指针，避免迭代器失效）
            QList<QTreeWidgetItem*> itemsToDelete;
            for (QTreeWidgetItem* item : selectedItems) {
                if (item->parent()) {
                    itemsToDelete.append(item);
                }
            }

            // 删除模型树中的项
            for (QTreeWidgetItem* item : itemsToDelete) {
                delete item;
            }
            // 立即同步其他视图
            viewContainer->synchronizeViews();
        } else if (result == exportAction) {
            // 导出选中的多边形到文件
            QString filePath = QFileDialog::getSaveFileName(nullptr, "导出多边形", "", "Polygon Files (*.txt);;All Files (*)");
            if (filePath.isEmpty()) {
                return;
            }

            // 收集需要导出的多边形
            std::vector<tailor_visualization::Polygon> polygonsToExport;

            for (QTreeWidgetItem* item : selectedItems) {
                if (!item->parent()) {
                    continue;
                }

                int type = item->data(0, Qt::UserRole).toInt();
                int index = item->parent()->indexOfChild(item);

                tailor_visualization::Polygon polygon;

                if (type == 0) { // polyline - 导出为封闭形式
                    const auto& polylines = view->polylines();
                    if (index >= 0 && index < polylines.size()) {
                        const auto& polyline = polylines[index];
                        int n = polyline.vertices.size();
                        // 正向边
                        for (int j = 0; j < n - 1; ++j) {
                            const auto& vertex = polyline.vertices[j];
                            const auto& nextVertex = polyline.vertices[j + 1];
                            tailor_visualization::PolygonEdge edge;
                            edge.startPoint.x = vertex.point.x();
                            edge.startPoint.y = vertex.point.y();
                            edge.endPoint.x = nextVertex.point.x();
                            edge.endPoint.y = nextVertex.point.y();
                            edge.bulge = vertex.bulge;
                            polygon.edges.push_back(edge);
                        }
                        // 反向边
                        for (int j = n - 1; j > 0; --j) {
                            const auto& vertex = polyline.vertices[j];
                            const auto& prevVertex = polyline.vertices[j - 1];
                            tailor_visualization::PolygonEdge edge;
                            edge.startPoint.x = vertex.point.x();
                            edge.startPoint.y = vertex.point.y();
                            edge.endPoint.x = prevVertex.point.x();
                            edge.endPoint.y = prevVertex.point.y();
                            edge.bulge = -prevVertex.bulge;
                            polygon.edges.push_back(edge);
                        }
                    }
                } else if (type == 1) { // polygon
                    const auto& polygons = view->polygons();
                    if (index >= 0 && index < polygons.size()) {
                        const auto& poly = polygons[index];
                        for (int j = 0; j < poly.vertices.size(); ++j) {
                            const auto& vertex = poly.vertices[j];
                            const auto& nextVertex = poly.vertices[(j + 1) % poly.vertices.size()];

                            tailor_visualization::PolygonEdge edge;
                            edge.startPoint.x = vertex.point.x();
                            edge.startPoint.y = vertex.point.y();
                            edge.endPoint.x = nextVertex.point.x();
                            edge.endPoint.y = nextVertex.point.y();
                            edge.bulge = vertex.bulge;
                            polygon.edges.push_back(edge);
                        }
                    }
                }

                if (!polygon.edges.empty()) {
                    polygonsToExport.push_back(polygon);
                }
            }

            // 导出到文件
            if (!polygonsToExport.empty()) {
                bool success = tailor_visualization::PolygonIO::ExportToFile(filePath.toStdString(), polygonsToExport);
                if (!success) {
                    qDebug() << "导出失败!";
                } else {
                    qDebug() << "导出成功:" << filePath;
                }
            }
        }
        });

    // ==================== 偏置结果模型树右键菜单 ====================
    offsetResultsTree->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(offsetResultsTree, &QTreeWidget::customContextMenuRequested, [offsetResultsTree, viewContainer](const QPoint& pos) {
        auto selectedItems = offsetResultsTree->selectedItems();
        if (selectedItems.isEmpty()) {
            return;
        }

        // 创建右键菜单
        QMenu menu;
        QAction* exportAction = menu.addAction("导出");

        QAction* result = menu.exec(offsetResultsTree->mapToGlobal(pos));

        if (result == exportAction) {
            // 导出选中的偏置结果多边形到文件
            QString filePath = QFileDialog::getSaveFileName(nullptr, "导出偏置结果", "", "Polygon Files (*.txt);;All Files (*)");
            if (filePath.isEmpty()) {
                return;
            }

            // 收集需要导出的偏置结果多边形
            std::vector<tailor_visualization::Polygon> polygonsToExport;

            for (QTreeWidgetItem* item : selectedItems) {
                if (!item->parent()) {
                    continue;
                }

                // 获取偏置结果索引
                int type = item->data(0, Qt::UserRole).toInt();
                int index = item->parent()->indexOfChild(item);

                // 获取对应的偏置结果
                const auto& offsetResults = viewContainer->mainView()->offsetResults();
                if (index >= 0 && index < offsetResults.size()) {
                    const auto& resultPoly = offsetResults[index];
                    tailor_visualization::Polygon polygon;
                    for (int j = 0; j < resultPoly.vertices.size(); ++j) {
                        const auto& vertex = resultPoly.vertices[j];
                        const auto& nextVertex = resultPoly.vertices[(j + 1) % resultPoly.vertices.size()];

                        tailor_visualization::PolygonEdge edge;
                        edge.startPoint.x = vertex.point.x();
                        edge.startPoint.y = vertex.point.y();
                        edge.endPoint.x = nextVertex.point.x();
                        edge.endPoint.y = nextVertex.point.y();
                        edge.bulge = 0.0;
                        polygon.edges.push_back(edge);
                    }

                    if (!polygon.edges.empty()) {
                        polygonsToExport.push_back(polygon);
                    }
                }
            }

            // 导出到文件
            if (!polygonsToExport.empty()) {
                bool success = tailor_visualization::PolygonIO::ExportToFile(filePath.toStdString(), polygonsToExport);
                if (!success) {
                    qDebug() << "导出失败!";
                } else {
                    qDebug() << "导出成功:" << filePath;
                }
            }
        }
        });

    window.show();

    return app.exec();
}

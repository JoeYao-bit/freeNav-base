#include "view.h"
//#define QT_NO_OPENGL

#if defined(QT_PRINTSUPPORT_LIB)
#include <QtPrintSupport/qtprintsupportglobal.h>
#if QT_CONFIG(printdialog)
#include <QPrinter>
#include <QPrintDialog>
#endif
#endif
#ifndef QT_NO_OPENGL
#include <QtOpenGL>
#else
#include <QtWidgets>
#endif
#include <qmath.h>

#if QT_CONFIG(wheelevent)
void GraphicsView::wheelEvent(QWheelEvent *e)
{
    if (e->modifiers() & Qt::ControlModifier) {
        if (e->delta() > 0) {
            view->zoomIn(6);
        } else {
            view->zoomOut(6);
        }
        e->accept();
    } else {
        QGraphicsView::wheelEvent(e);
    }
}
#endif

View::View(const QString &name, QWidget *parent)
    : QFrame(parent)
{
    setFrameStyle(Sunken | StyledPanel);
    graphicsView = new GraphicsView(this);
    graphicsView->setRenderHint(QPainter::Antialiasing, false);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    graphicsView_second = new GraphicsView(this);
    graphicsView_second->setRenderHint(QPainter::Antialiasing, false);
    graphicsView_second->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView_second->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView_second->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView_second->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);


    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    QToolButton *zoomInIcon = new QToolButton;
    zoomInIcon->setAutoRepeat(true);
    zoomInIcon->setAutoRepeatInterval(33);
    zoomInIcon->setAutoRepeatDelay(0);
    zoomInIcon->setIcon(QPixmap(":/zoomin.png"));
    zoomInIcon->setIconSize(iconSize);

    QToolButton *zoomInIcon_second = new QToolButton;
    zoomInIcon_second->setAutoRepeat(true);
    zoomInIcon_second->setAutoRepeatInterval(33);
    zoomInIcon_second->setAutoRepeatDelay(0);
    zoomInIcon_second->setIcon(QPixmap(":/zoomin.png"));
    zoomInIcon_second->setIconSize(iconSize);

    QToolButton *zoomOutIcon = new QToolButton;
    zoomOutIcon->setAutoRepeat(true);
    zoomOutIcon->setAutoRepeatInterval(33);
    zoomOutIcon->setAutoRepeatDelay(0);
    zoomOutIcon->setIcon(QPixmap(":/zoomout.png"));
    zoomOutIcon->setIconSize(iconSize);

    QToolButton *zoomOutIcon_second = new QToolButton;
    zoomOutIcon_second->setAutoRepeat(true);
    zoomOutIcon_second->setAutoRepeatInterval(33);
    zoomOutIcon_second->setAutoRepeatDelay(0);
    zoomOutIcon_second->setIcon(QPixmap(":/zoomout.png"));
    zoomOutIcon_second->setIconSize(iconSize);

    zoomSlider = new QSlider;
    zoomSlider->setMinimum(0);
    zoomSlider->setMaximum(1000); // 最大放大倍数
    zoomSlider->setValue(300);
    zoomSlider->setTickPosition(QSlider::TicksRight);

    zoomSlider_second = new QSlider;
    zoomSlider_second->setMinimum(0);
    zoomSlider_second->setMaximum(1000); // 最大放大倍数
    zoomSlider_second->setValue(300);
    zoomSlider_second->setTickPosition(QSlider::TicksRight);

    // Zoom slider layout
    QVBoxLayout *zoomSliderLayout = new QVBoxLayout;
    zoomSliderLayout->addWidget(zoomInIcon);
    zoomSliderLayout->addWidget(zoomSlider);
    zoomSliderLayout->addWidget(zoomOutIcon);

    QVBoxLayout *zoomSliderLayout_second = new QVBoxLayout;
    zoomSliderLayout_second->addWidget(zoomInIcon_second);
    zoomSliderLayout_second->addWidget(zoomSlider_second);
    zoomSliderLayout_second->addWidget(zoomOutIcon_second);

    resetButton = new QToolButton;
    resetButton->setText(tr("reset"));
    resetButton->setEnabled(false);

    resetButton_second = new QToolButton;
    resetButton_second->setText(tr("reset"));
    resetButton_second->setEnabled(false);

    // Label layout
    QHBoxLayout *labelLayout = new QHBoxLayout;
    label = new QLabel(name);
    selectModeButton = new QToolButton;
    selectModeButton->setText(tr("Select"));
    selectModeButton->setCheckable(true);
    selectModeButton->setChecked(true);

    selectModeButton_second = new QToolButton;
    selectModeButton_second->setText(tr("Select"));
    selectModeButton_second->setCheckable(true);
    selectModeButton_second->setChecked(true);

    dragModeButton = new QToolButton;
    dragModeButton->setText(tr("Drag"));
    dragModeButton->setCheckable(true);
    dragModeButton->setChecked(false);

    dragModeButton_second = new QToolButton;
    dragModeButton_second->setText(tr("Drag"));
    dragModeButton_second->setCheckable(true);
    dragModeButton_second->setChecked(false);

    QMenuBar *bar = new QMenuBar(this);

    drawFinalPathSpecificButton = new QToolButton;
    drawFinalPathSpecificButton->setText(tr("SpecificFinalPath"));
    drawFinalPathSpecificButton->setCheckable(true);
    drawFinalPathSpecificButton->setChecked(true);

    current_agent_text = new QLabel(this);
    current_agent_text->setText(tr("Current Agent:"));

    drawCurrentInstance_second = new QToolButton;
    drawCurrentInstance_second->setText(QString("Whether draw current agent"));
    drawCurrentInstance_second->setCheckable(true);
    drawCurrentInstance_second->setChecked(true);

    colorValueMenu       = new QMenu(QStringLiteral("SetColorType: Default"), this);
    QAction *pOpenAction = new QAction(QStringLiteral("Green"), this);
    QAction *pSaveAction = new QAction(QStringLiteral("Red"), this);

    connect(pOpenAction, &QAction::triggered, this, &View::setColorToGreen);
    connect(pSaveAction, &QAction::triggered, this, &View::setColorToRed);

    QMenuBar *bar_grid_map = new QMenuBar(this);

    gridMapValueMenu              = new QMenu(QStringLiteral("SetGridMapContent: Empty"), this);
    QAction *drawHyperGraphAction = new QAction(QStringLiteral("HyperGraph"), this);
    QAction *drawInstance         = new QAction(QStringLiteral("Instances"), this);
    QAction *drawHeuristic        = new QAction(QStringLiteral("Heuristic"), this);
    QAction *drawCluster          = new QAction(QStringLiteral("AllClusters"), this);
    QAction *drawEmptyAction      = new QAction(QStringLiteral("Empty"), this);

    gridMapValueMenu->addAction(drawHyperGraphAction);
    gridMapValueMenu->addAction(drawInstance);
    gridMapValueMenu->addAction(drawHeuristic);
    gridMapValueMenu->addAction(drawCluster);
    gridMapValueMenu->addAction(drawEmptyAction);

    bar_grid_map->addMenu(gridMapValueMenu);

    bar->addMenu(colorValueMenu);

    QList<QAction*> fileAactions;
    fileAactions << pOpenAction << pSaveAction;
    colorValueMenu->addActions(fileAactions);

    antialiasButton = new QToolButton;
    antialiasButton->setText(tr("Antialiasing"));
    antialiasButton->setCheckable(true);
    antialiasButton->setChecked(false);

    openGlButton = new QToolButton;
    openGlButton->setText(tr("OpenGL"));
    openGlButton->setCheckable(true);

    antialiasButtonSecond = new QToolButton;
    antialiasButtonSecond->setText(tr("Antialiasing"));
    antialiasButtonSecond->setCheckable(true);
    antialiasButtonSecond->setChecked(true);
    graphicsView_second->setRenderHint(QPainter::Antialiasing, true);


    openGlButtonSecond = new QToolButton;
    openGlButtonSecond->setText(tr("OpenGL"));
    openGlButtonSecond->setCheckable(true);
    openGlButtonSecond->setChecked(true);
    graphicsView_second->setViewport( new QGLWidget(QGLFormat(QGL::SampleBuffers)));

#ifndef QT_NO_OPENGL
    openGlButton->setEnabled(QGLFormat::hasOpenGL());
#else
    openGlButton->setEnabled(false);
#endif
    printButton = new QToolButton;
    printButton->setIcon(QIcon(QPixmap(":/fileprint.png")));

    QButtonGroup *pointerModeGroup = new QButtonGroup(this);
    pointerModeGroup->setExclusive(true);
    pointerModeGroup->addButton(selectModeButton);
    pointerModeGroup->addButton(dragModeButton);

    labelLayout->addWidget(label);
    labelLayout->addStretch();
    labelLayout->addWidget(selectModeButton);
    labelLayout->addWidget(dragModeButton);

    labelLayout->addStretch();
    labelLayout->addWidget(bar);
    labelLayout->addWidget(drawFinalPathSpecificButton);

    labelLayout->addStretch();
    labelLayout->addWidget(antialiasButton);
    labelLayout->addWidget(openGlButton);
    labelLayout->addWidget(printButton);

//    labelLayout->addStretch();
//    labelLayout->addWidget(selectModeButton_second);
//    labelLayout->addWidget(dragModeButton_second);



    QGridLayout *topLayout = new QGridLayout;
    topLayout->addLayout(labelLayout, 0, 0);

    QButtonGroup *pointerModeGroup_second = new QButtonGroup(this);
    pointerModeGroup_second->setExclusive(true);
    pointerModeGroup_second->addButton(selectModeButton_second);
    pointerModeGroup_second->addButton(dragModeButton_second);

    agent_num_edit = new QLineEdit(this);
    agent_num_edit->setPlaceholderText(QString(to_string(current_shown_agent_).c_str()));
    agent_num_edit->setClearButtonEnabled(true);
    agent_num_edit->setValidator(new QIntValidator(agent_num_edit));
    agent_num_edit->setEchoMode(QLineEdit::Normal);

    //connect(agent_num_edit, SIGNAL(editingFinished()), this, SLOT(agentNumEditFinished()));

    // 只允许输入整型
    // ui->lineEdit->setValidator(new QIntValidator(ui->lineEdit));

    // 只允许输入数字
    // ui->lineEdit->setValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

    // 只能输入字母和数字
    // ui->lineEdit->setValidator(new QRegExpValidator(QRegExp("[a-zA-Z0-9]+$")));

    //agent_num_edit->setEchoMode();

    QHBoxLayout *labelLayout_second = new QHBoxLayout;
    labelLayout_second->addWidget(selectModeButton_second);
    labelLayout_second->addWidget(dragModeButton_second);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(bar_grid_map);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(current_agent_text);
    labelLayout_second->addWidget(agent_num_edit);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(drawCurrentInstance_second);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(antialiasButtonSecond);
    labelLayout_second->addWidget(openGlButtonSecond);

    topLayout->addLayout(labelLayout_second, 0, 1);

    QGridLayout *window_first = new QGridLayout;
    window_first->addWidget(graphicsView, 0, 0);
    window_first->addLayout(zoomSliderLayout, 0, 1);
    topLayout->addLayout(window_first, 1, 0);

    QGridLayout *window_second = new QGridLayout;
    window_second->addWidget(graphicsView_second, 0, 0);
    window_second->addLayout(zoomSliderLayout_second, 0, 1);
    topLayout->addLayout(window_second, 1, 1);


    topLayout->addWidget(resetButton, 2, 0);
    topLayout->addWidget(resetButton_second, 2, 1);

    topLayout->setColumnStretch(0, 2);
    topLayout->setColumnStretch(1, 3);

    QGridLayout *on_top_Layout = new QGridLayout;
    on_top_Layout->addLayout(topLayout, 0, 0);

    windowRatioSlider = new QSlider;
    windowRatioSlider->setMinimum(1);
    windowRatioSlider->setMaximum(1000); // 最大放大倍数
    windowRatioSlider->setValue(500);
    windowRatioSlider->setOrientation(Qt::Horizontal);

    on_top_Layout->addWidget(windowRatioSlider, 1, 0);

    setLayout(on_top_Layout);

    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetView()));
    connect(resetButton_second, SIGNAL(clicked()), this, SLOT(resetViewSecond()));

    connect(zoomSlider, SIGNAL(valueChanged(int)), this, SLOT(setupMatrix()));
    connect(zoomSlider_second, SIGNAL(valueChanged(int)), this, SLOT(setupMatrixSecond()));

    connect(windowRatioSlider, SIGNAL(valueChanged(int)), this, SLOT(updateWindowRatio()));

    connect(graphicsView->verticalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));

    connect(graphicsView->horizontalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));

    connect(selectModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    //connect(drawFinalPathSpecificButton, SIGNAL(toggled(bool)), this, SLOT(setDrawNodeInPathSpecific()));

    connect(dragModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    connect(dragModeButton_second, SIGNAL(toggled(bool)), this, SLOT(togglePointerModeSecond()));


    connect(antialiasButton, SIGNAL(toggled(bool)), this, SLOT(toggleAntialiasing()));
    connect(openGlButton, SIGNAL(toggled(bool)), this, SLOT(toggleOpenGL()));

    connect(antialiasButtonSecond, SIGNAL(toggled(bool)), this, SLOT(toggleAntialiasingSecond()));
    connect(openGlButtonSecond, SIGNAL(toggled(bool)), this, SLOT(toggleOpenGLSecond()));

    connect(zoomInIcon, SIGNAL(clicked()), this, SLOT(zoomIn()));
    connect(zoomOutIcon, SIGNAL(clicked()), this, SLOT(zoomOut()));

    connect(zoomInIcon_second, SIGNAL(clicked()), this, SLOT(zoomInSecond()));
    connect(zoomOutIcon_second, SIGNAL(clicked()), this, SLOT(zoomOutSecond()));

    connect(printButton, SIGNAL(clicked()), this, SLOT(print()));

    //connect(drawCurrentInstance_second, SIGNAL(toggled(bool)), this, SLOT(toggleDrawCurrentInstance()));

    setupMatrix();

}

QGraphicsView *View::view() const
{
    return static_cast<QGraphicsView *>(graphicsView);
}

void View::updateWindowRatio() {
    //topLayout->setSizeConstraint(QLayout::SetNoConstraint);
//    topLayout->setColumnStretch(0, windowRatioSlider->value());
//    topLayout->setColumnStretch(1, windowRatioSlider->maximum() - windowRatioSlider->value());
    //std::cout << " windowRatioSlider->value() " << windowRatioSlider->value() << " / windowRatioSlider->maximum() " << windowRatioSlider->maximum() << std::endl;
    //topLayout->update();
}

QGraphicsView *View::second_view() const {
    return static_cast<QGraphicsView *>(graphicsView_second);
}

void View::keyPressEvent(QKeyEvent *ev) {
    //std::cout << ev->key() << " is pressed " << std::endl;
}

void View::keyReleaseEvent(QKeyEvent *ev) {
    //std::cout << ev->key() << " is released " << std::endl;
}

void View::resetView()
{
    zoomSlider->setValue(250);
    setupMatrix();
    graphicsView->ensureVisible(QRectF(0, 0, 0, 0));
    resetButton->setEnabled(false);
}

void View::resetViewSecond()
{
    zoomSlider_second->setValue(250);
    setupMatrixSecond();
    graphicsView_second->ensureVisible(QRectF(0, 0, 0, 0));
    resetButton_second->setEnabled(false);
}

void View::setResetButtonEnabled()
{
    resetButton->setEnabled(true);
}

void View::setResetButtonEnabledSecond()
{
    resetButton_second->setEnabled(true);
}


void View::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 250) / qreal(50));

    QMatrix matrix;
    matrix.scale(scale, scale);

    graphicsView->setMatrix(matrix);
    setResetButtonEnabled();
}

void View::setupMatrixSecond()
{
    qreal scale = qPow(qreal(2), (zoomSlider_second->value() - 250) / qreal(50));
    QMatrix matrix;
    matrix.scale(scale, scale);
    graphicsView_second->setMatrix(matrix);
    setResetButtonEnabledSecond();
}

void View::togglePointerMode()
{
    graphicsView->setDragMode(selectModeButton->isChecked()
                              ? QGraphicsView::RubberBandDrag
                              : QGraphicsView::ScrollHandDrag);
    graphicsView->setInteractive(selectModeButton->isChecked());
}

void View::togglePointerModeSecond()
{
    graphicsView_second->setDragMode(selectModeButton_second->isChecked()
                              ? QGraphicsView::RubberBandDrag
                              : QGraphicsView::ScrollHandDrag);
    graphicsView_second->setInteractive(selectModeButton_second->isChecked());
}

void View::toggleOpenGL()
{
#ifndef QT_NO_OPENGL
    graphicsView->setViewport(openGlButton->isChecked() ? new QGLWidget(QGLFormat(QGL::SampleBuffers)) : new QWidget);
#endif
}


void View::toggleAntialiasing()
{
    graphicsView->setRenderHint(QPainter::Antialiasing, antialiasButton->isChecked());
}

void View::toggleOpenGLSecond()
{
#ifndef QT_NO_OPENGL
    graphicsView_second->setViewport(openGlButtonSecond->isChecked() ? new QGLWidget(QGLFormat(QGL::SampleBuffers)) : new QWidget);
#endif
}


void View::toggleAntialiasingSecond()
{
    graphicsView_second->setRenderHint(QPainter::Antialiasing, antialiasButtonSecond->isChecked());
}

// TODO: considering print to SVG picture in the future
void View::print()
{
#if QT_CONFIG(printdialog)
    QPrinter printer;
    QPrintDialog dialog(&printer, this);
    if (dialog.exec() == QDialog::Accepted) {
        // in the final result, at least one orientation reach boundary of scene
        QPainter painter(&printer);
        //graphicsView->render(&painter); // render just what in view and print it on file
        graphicsView->scene()->render(&painter); // render the whole scene and print it on file
    }
#endif
}

void View::zoomIn(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
}

void View::zoomOut(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
}

void View::zoomInSecond(int level)
{
    zoomSlider_second->setValue(zoomSlider_second->value() + level);
}

void View::zoomOutSecond(int level)
{
    zoomSlider_second->setValue(zoomSlider_second->value() - level);
}

void View::setAllNodesColor(int r, int g, int b) {
//    for(int level=0; level<all_level_nodes_.size(); level++) {
//        for(int i=0; i<all_level_nodes_[level].size(); i++) {
//            all_level_nodes_[level][i]->setColorRGB(r, g, b);
//        }
//    }
}

void View::setColorToGreen() {
    colorValueMenu->setTitle(tr("SetColorType: Green"));
    colorValueMenu->update();
    setAllNodesColor(0, 255, 0);
}

void View::setColorToRed() {
    colorValueMenu->setTitle(tr("SetColorType: Red"));
    colorValueMenu->update();
    setAllNodesColor(255, 0, 0);
}

void View::initGridMap() {
    // draw grid map
    int block_total_width = GRID_WIDTH + 2 * GRID_INTERVAL;
    all_grid_.clear();
    for(int j=0; j<dim_[1]; j++) {
        for(int i=0; i<dim_[0]; i++) {
            Grid *grid = new Grid(10);
            //std::cout << "i*block_total_width, j*block_total_width" << i*block_total_width << ", " << j*block_total_width << std::endl;
            grid->setPos(i*block_total_width, j*block_total_width);
            freeNav::Pointi<2> pt({i, j});
            if(is_occupied_(pt)) {
                grid->setColorRGB(150, 150,150);
            } else {
                grid->setColorRGB(255, 255, 255);
            }
            all_grid_.push_back(grid);
        }
    }
    if(instance_.size() > 0) {
        agent_start_ = new CircleOfGrid(15);
        agent_start_->setColorRGB(0, 255, 0);
        agent_start_->setPos(instance_[current_shown_agent_].first[0]*block_total_width,
                             instance_[current_shown_agent_].first[1]*block_total_width);
        agent_target_ = new CircleOfGrid(15);
        agent_target_->setColorRGB(255, 0,0);
        agent_target_->setPos(instance_[current_shown_agent_].second[0]*block_total_width,
                              instance_[current_shown_agent_].second[1]*block_total_width);
    }
}

void View::resetGridMapToEmpty() {
    for(auto& grid : all_grid_) {
        grid->strs_.clear();
    }
    for(int j=0; j<dim_[1]; j++) {
        for(int i=0; i<dim_[0]; i++) {
            freeNav::Pointi<2> pt({i, j});
            freeNav::Id grid_id = freeNav::PointiToId(pt, dim_);
            Grid *grid = all_grid_[grid_id];
            if(is_occupied_(pt)) {
                grid->setColorRGB(150, 150,150);
            } else {
                grid->setColorRGB(255, 255, 255);
            }
        }
    }
}

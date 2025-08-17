#ifndef WBMO_IMAGEVIEWERBASE_HPP
#define WBMO_IMAGEVIEWERBASE_HPP

#include "ui_image_viewer.h"
#include <QWidget>
#include <qpoint.h>

struct QObservation
{
    QPoint keypoint;
    QPoint projectedLandmark;
};

class ImageViewer : public QWidget
{
    Q_OBJECT
  public:
    explicit ImageViewer(QWidget* parent = nullptr);

    QImage grabImage() const;

  public slots:
    void scaleDrawnImage(int percent) { scaleDrawnImage(static_cast<float>(percent)); }
    void drawImage(const QImage& image);
    void drawImageWithObservations(const QImage& image, const QVector<QObservation>& observations);

  protected:
    void initialize();
    void scaleDrawnImage(float percent);
    void drawPointsOnImage(QPixmap& pixmap, const QVector<QPoint>& points);

    void drawPixmap(const QPixmap& pixmap);

    Ui::MainImageViewer ui;
    QImage image;
    bool drawKeypointsFlag = true;
};

#endif

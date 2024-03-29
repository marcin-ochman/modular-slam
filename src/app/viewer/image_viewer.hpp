#ifndef WBMO_IMAGEVIEWERBASE_HPP
#define WBMO_IMAGEVIEWERBASE_HPP

#include "ui_image_viewer.h"
#include <QWidget>

class ImageViewer : public QWidget
{
    Q_OBJECT
  public:
    ImageViewer(QWidget* parent = nullptr) : QWidget(parent) { initialize(); }

  public slots:
    void scaleDrawnImage(int percent) { scaleDrawnImage(static_cast<float>(percent)); }
    void drawImage(const QImage& image);

  protected:
    void scaleDrawnImage(float percent);
    void initialize();

    Ui::MainImageViewer ui;
    QImage image;
};

#endif

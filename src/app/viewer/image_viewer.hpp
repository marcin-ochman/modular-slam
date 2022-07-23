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
    void scaleDrawnImage(int percent);
    void drawImage(const QPixmap& pixmap);

  protected:
    void initialize();

    Ui::MainImageViewer ui;
    QPixmap pixmap;
};

#endif

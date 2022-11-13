#ifndef MAP_2D_WIDGET_HPP_
#define MAP_2D_WIDGET_HPP_

#include <QVector2D>
#include <QWidget>

class QScatterSeries;

class MapWidget : public QWidget
{
  public:
    MapWidget(QWidget* parent = nullptr);

  public slots:
    void addSensorPosition(int x, int y);

  private:
    QScatterSeries* m_series;
};

#endif // MAP_2D_WIDGET_HPP_

#ifndef MAP_2D_WIDGET_HPP_
#define MAP_2D_WIDGET_HPP_

#include <QVector2D>
#include <QWidget>

class QScatterSeries;

struct MapKeyframe
{
    int id;
    QVector2D position;
    float angle;
};

class MapWidget : public QWidget
{
  public:
    MapWidget(QWidget* parent = nullptr);

  public slots:
    void addKeyframe(int x, int y);

    // void addLandmark(int x, int y);
    // void updateLandmark();

  private:
    QScatterSeries* m_keyframeSeries;
};

#endif // MAP_2D_WIDGET_HPP_

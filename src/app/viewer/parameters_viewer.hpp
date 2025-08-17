#ifndef PARAMETERS_VIEWER_HPP_
#define PARAMETERS_VIEWER_HPP_

#include <QVariant>
#include <QWidget>

#include "modular_slam/parameters/parameters_handler.hpp"
#include "range.hpp"

class ParameterWidget : public QWidget
{
    Q_OBJECT

  public:
    ParameterWidget(const QString& name, float value, const Range& range, QWidget* parent = nullptr);
    ParameterWidget(const QString& name, const int, const QVector<int>& choices, QWidget* parent = nullptr);

    QString name() const;
    bool setValue(QVariant choice);
};

class QVBoxLayout;

class ParametersViewer : public QWidget
{
    Q_OBJECT

  public:
    ParametersViewer(QWidget* parent = nullptr);
    ~ParametersViewer();

  public slots:
    void addValueParameter(const QString& name, float value, const Range& range);
    void addChoiceParameter(const QString& name, const int, const QVector<int>& choices);
    void removeParameter(const QString& name);
    void setValue(const QString& name, QVariant value);

  signals:
    void valueChanged(const QString& name, QVariant value);

  private:
    std::shared_ptr<mslam::ParametersHandlerInterface> m_handler;

    QVBoxLayout* m_layout;
};
#endif // PARAMETERS_VIEWER_H_

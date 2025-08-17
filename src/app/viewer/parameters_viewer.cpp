#include "parameters_viewer.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>

ParameterWidget::ParameterWidget(const QString& parameterName, float value, const Range& range, QWidget* parent)
    : QWidget(parent)
{
    QHBoxLayout* layout = new QHBoxLayout(this);

    QLabel* label = new QLabel(parameterName, this);
    layout->addWidget(label);

    QSlider* control = new QSlider(Qt::Horizontal, this);
    layout->addWidget(control);

    control->setMinimum(range.min);
    control->setMaximum(range.max);
    control->setSingleStep(range.step);

    setLayout(layout);
}

ParameterWidget::ParameterWidget(const QString& parameterName, const int value, const QVector<int>& choices,
                                 QWidget* parent)
    : QWidget(parent)
{
    QHBoxLayout* layout = new QHBoxLayout(this);

    QLabel* label = new QLabel(parameterName, this);
    layout->addWidget(label);

    QComboBox* control = new QComboBox(this);
    layout->addWidget(control);

    for(auto& v : choices)
        control->addItem(QString::number(v), v);

    setLayout(layout);
}

QString ParameterWidget::name() const
{
    if(auto widget = qobject_cast<QLabel*>(layout()->itemAt(0)->widget()); widget)
        return widget->text();

    return QString();
}

bool ParameterWidget::setValue(QVariant value)
{
    if(value.typeId() == QMetaType::Int)
        return true;

    if(value.typeId() == QMetaType::Float)
        return true;

    return false;
}

ParametersViewer::ParametersViewer(QWidget* parent) : QWidget(parent)
{
    m_layout = new QVBoxLayout(this);

    setLayout(m_layout);
}

void ParametersViewer::addValueParameter(const QString& name, float value, const Range& range)
{
    ParameterWidget* widget = new ParameterWidget(name, value, range, this);

    m_layout->addWidget(widget);
}

void ParametersViewer::addChoiceParameter(const QString& name, const int value, const QVector<int>& choices)
{
    ParameterWidget* widget = new ParameterWidget(name, value, choices, this);

    m_layout->addWidget(widget);
}

void ParametersViewer::removeParameter(const QString& name)
{
    for(int i = 0; i < m_layout->count(); ++i)
    {
        ParameterWidget* widget = qobject_cast<ParameterWidget*>(m_layout->itemAt(i)->widget());
        if(widget && widget->name() == name)
        {
            m_layout->removeWidget(widget);
            widget->setParent(nullptr);
            delete widget;
        }
    }
}

void ParametersViewer::setValue(const QString& name, QVariant value)
{
    for(int i = 0; i < m_layout->count(); ++i)
    {
        ParameterWidget* widget = qobject_cast<ParameterWidget*>(m_layout->itemAt(i)->widget());

        if(widget)
            widget->setValue(value);
    }
}

ParametersViewer::~ParametersViewer() {}

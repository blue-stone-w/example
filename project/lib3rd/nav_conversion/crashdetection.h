#ifndef CRASHDETECTION_H
#define CRASHDETECTION_H

#include <QObject>
#include <QRect>
#include <QGeoCoordinate>
#include <QVariantList>

class CrashDetection : public QObject
{
    Q_OBJECT
public:
    explicit CrashDetection(QObject *parent = nullptr);

    //判断矩形和圆形是否相交
    Q_INVOKABLE bool rectCircleIntersect(QVariantList vertexs, QGeoCoordinate radiusCenter, double radius);

private:
    void WGS84toXY(const double Lat, const double Long, double &UTMNorthing, double &UTMEasting);
    double pointToLineDistance(QPointF &point, QPointF &point1, QPointF &point2);
};

#endif // CRASHDETECTION_H

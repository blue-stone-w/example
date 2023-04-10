#include "crashdetection.h"
#include "navsat_conversions.h"
#include <qmath.h>
#include <QDebug>

static constexpr double origin_longitude = 117.478660383;
static constexpr double origin_latitude = 34.398351105;

CrashDetection::CrashDetection(QObject *parent) : QObject(parent)
{

}

bool CrashDetection::rectCircleIntersect(QVariantList vertexs, QGeoCoordinate radiusCenter, double radius)
{
    if(vertexs.length() != 4) {
        return false;
    }

    QGeoCoordinate leftTopPoint = vertexs[0].value<QGeoCoordinate>();
    QGeoCoordinate rightTopPoint = vertexs[1].value<QGeoCoordinate>();
    QGeoCoordinate rightBottomPoint = vertexs[2].value<QGeoCoordinate>();
    QGeoCoordinate leftBottomPoint = vertexs[3].value<QGeoCoordinate>();

    if((radiusCenter.distanceTo(leftTopPoint) < radius) || (radiusCenter.distanceTo(rightTopPoint) < radius)
            || (radiusCenter.distanceTo(rightBottomPoint) < radius) || (radiusCenter.distanceTo(leftBottomPoint) < radius))
    {
        return true;
    }

    /*将顶点转成直角坐标*/
    QPointF vertexPoint1, vertexPoint2, vertexPoint3, vertexPoint4, centerPoint;

    WGS84toXY(leftTopPoint.latitude(), leftTopPoint.longitude(), vertexPoint1.ry(), vertexPoint1.rx());
    WGS84toXY(rightTopPoint.latitude(), rightTopPoint.longitude(), vertexPoint2.ry(), vertexPoint2.rx());
    WGS84toXY(rightBottomPoint.latitude(), rightBottomPoint.longitude(), vertexPoint3.ry(), vertexPoint3.rx());
    WGS84toXY(leftBottomPoint.latitude(), leftBottomPoint.longitude(), vertexPoint4.ry(), vertexPoint4.rx());
    WGS84toXY(radiusCenter.latitude(), radiusCenter.longitude(), centerPoint.ry(), centerPoint.rx());

    /*圆点到线段的距离*/
    if((pointToLineDistance(centerPoint, vertexPoint1, vertexPoint2) < radius)
            || (pointToLineDistance(centerPoint, vertexPoint2, vertexPoint3) < radius)
            || (pointToLineDistance(centerPoint, vertexPoint3, vertexPoint4) < radius)
            || (pointToLineDistance(centerPoint, vertexPoint4, vertexPoint1) < radius))
    {
        return true;
    }

    return false;
}


/*函数名： WGS84toXY
函数功能： 84坐标转成直角坐标,直角坐标系中x是代表east，y代表north
输入：
输出：NA*/
void CrashDetection::WGS84toXY(const double Lat, const double Long, double &UTMNorthing, double &UTMEasting)
{
    std::string UTMZone;
    RobotLocalization::NavsatConversions::LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, UTMZone);
    double originEasting = 0.0;
    double originNorthing = 0.0;
    RobotLocalization::NavsatConversions::LLtoUTM(origin_latitude, origin_longitude, originNorthing, originEasting, UTMZone);
    /*当前的直角坐标减去矿山原点的UTM*/
    UTMNorthing -= originNorthing;
    UTMEasting -= originEasting;

    return;
}


double CrashDetection::pointToLineDistance(QPointF &point, QPointF &point1, QPointF &point2)
{
    double px = point2.rx() - point1.rx();
    double py = point2.ry() - point1.ry();
    /*线段长度的平方*/
    double som = px * px + py * py;
    /*向量点积,*/
    double u =  ((point.rx() - point1.rx()) * px + (point.ry() - point1.ry()) * py) / som;

    /*当t（r）> 1时，最短距离即为point point2*/
    if (u > 1) {
        u = 1;
    }

    /*当t（r）< 0时，最短距离即为point point1*/
    if (u < 0) {
        u = 0;
    }

    //the closest point
    double x = point1.rx() + u * px;
    double y = point1.ry() + u * py;
    double dx = x - point.rx();
    double dy = y - point.ry();

    return qSqrt(dx*dx + dy*dy);
}

/*基于opencv实现的一些小功能*/

#ifndef OPENCV_U_H
# define OPENCV_U_H

#include <opencv2/opencv.hpp>
#include <vector>

// Mat转vector
template<typename Scalar>
vector<Scalar> convertMat2Vector(const Mat &mat)
{
  return (vector<Scalar>)(mat.reshape(1, 1)); //通道数不变，按行转为一行
}

// vector转Mat
template<typename Scalar>
cv::Mat convertVector2Mat(vector<Scalar> v, int channels, int rows)
{
  cv::Mat mat = cv::Mat(v); //将vector变成单列的mat
  cv::Mat dest = mat.reshape(channels, rows).clone(); / 必须clone()一份，否则返回出错
  return dest;
}

#endif

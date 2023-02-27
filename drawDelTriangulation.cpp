#include <opencv.hpp>
#include <vector>
  
using namespace std;

void drawDelTriangulation(const char* dataFile, int* vertexIdx, int triNum){
  vector<cv::Point> cvpt(3);
  for (size_t num = 0; num < triNum*3; num++){
    MUInt32 idx = vertexIdx[num++];
    cvpt[0] = cv::Point(warpedPoints[idx].x, warpedPoints[idx].y);
    idx = vertexIdx[num++];
    cvpt[1] = cv::Point(warpedPoints[idx].x, warpedPoints[idx].y);
    idx = vertexIdx[num];
    cvpt[2] = cv::Point(warpedPoints[idx].x, warpedPoints[idx].y);
    line(cvBgr, cvpt[0], cvpt[1], cv::Scalar(255, 255, 255, 0.0), 1, cv::LINE_AA, 0);
    line(cvBgr, cvpt[1], cvpt[2], cv::Scalar(255, 255, 255, 0.0), 1, cv::LINE_AA, 0);
    line(cvBgr, cvpt[2], cvpt[0], cv::Scalar(255, 255, 255, 0.0), 1, cv::LINE_AA, 0);
  }

  //MTChar dataFile[164];
  //auto timeStamp = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
  //long long time = timeStamp.count();
  //MSprintf(dataFile, _MMT("G:\\batch\\0x0400000000000640\\pkg\\output\\test_triangles_%d.png"), time);
  //const string path = dataFile;
  cv::imwrite(dataFile, cvBgr);
}

void drawPoint(MPOINT* origPoints, const char* path){
    cv::Mat cvimage = cv::Mat(bgHeight, bgWidth, CV_8UC3);
    MPOINT wpoint = origPoints[num];
    cv::Point cvp = cv::Point(wpoint.x, wpoint.y);
    cv::circle(cvimage, cvp, 1.5, cv::Scalar(0, 0, 255));
    cv::putText(cvimage, to_string(num-28), cvp, cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
    cv::imwrite(path, cvimage);
}

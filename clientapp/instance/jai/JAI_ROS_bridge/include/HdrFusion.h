#include <cv_bridge/cv_bridge.h>

#include <opencv2/photo.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>
class HdrFusion {
 public:
  HdrFusion();

  void fusion_hdr(std::vector<std::pair<int, uint8_t*>> images,
                  cv::Mat& dst_hdr, cv::Mat& dst_fusion, bool rgb);

 private:
  cv::Ptr<cv::AlignMTB> alignMTB;
  cv::Ptr<cv::MergeDebevec> merge_debevec;
  cv::Ptr<cv::MergeMertens> merge_mertens;
};
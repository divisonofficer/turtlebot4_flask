#pragma once
#include <wrapper.h>
class HdrFusion {
 public:
  HdrFusion();

  void fusion_hdr(std::vector<std::pair<int, cv::Mat>> images, cv::Mat& dst_hdr,
                  cv::Mat& dst_fusion, bool rgb);

 private:
  cv::Ptr<cv::MergeDebevec> merge_debevec;
  cv::Ptr<cv::MergeMertens> merge_mertens;
};

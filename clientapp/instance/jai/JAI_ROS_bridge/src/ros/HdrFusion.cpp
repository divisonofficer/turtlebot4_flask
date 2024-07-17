#include <HdrFusion.h>

void HdrFusion::fusion_hdr(std::vector<std::pair<int, uint8_t*>> images,
                           cv::Mat& dst_hdr, cv::Mat& dst_fusion, bool rgb) {
  std::vector<cv::Mat> image_list;
  std::vector<float> times;

  for (auto& [exposure, buffer] : images) {
    cv::Mat img = cv::Mat(1440, 1080, rgb ? CV_8UC3 : CV_8UC1, buffer);
    image_list.push_back(img);
    times.push_back(exposure / 1000000.0f);
  }

  alignMTB->process(image_list, image_list);
  cv::Mat hdr;
  merge_debevec->process(image_list, hdr, times);
  hdr.convertTo(dst_hdr, rgb ? CV_16UC3 : CV_16UC1);
  cv::Mat fusion;
  merge_mertens->process(image_list, fusion);
  fusion.convertTo(dst_fusion, rgb ? CV_8UC3 : CV_8UC1, 255);
}

HdrFusion::HdrFusion() {
  alignMTB = cv::createAlignMTB();
  merge_debevec = cv::createMergeDebevec();
  merge_mertens = cv::createMergeMertens();
}
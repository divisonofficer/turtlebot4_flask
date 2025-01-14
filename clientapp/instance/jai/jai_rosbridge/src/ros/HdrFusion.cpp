#include <HdrFusion.h>
#include <Logger.h>
void HdrFusion::fusion_hdr(std::vector<std::pair<int, cv::Mat>> images,
                           cv::Mat& dst_hdr, cv::Mat& dst_fusion, bool rgb) {
  // cv::Mat concatenated;
  // cv::vconcat(images.begin(), images.end(), concatenated);
  // concatenated.convertTo(dst_hdr, CV_8UC1);

  // std::vector<cv::Mat> image_list;
  // std::vector<float> times;
  // for (auto& [exposure, img] : images) {
  //   if (!rgb) {
  //     cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  //   } else {
  //     cv::cvtColor(img, img, cv::COLOR_BayerRG2BGR);
  //   }
  //   image_list.push_back(img);
  //   times.push_back(exposure / 1000000.0f);
  // }
  // cv::Mat hdr;
  // merge_debevec->process(image_list, hdr, times);
  // if (!rgb) {
  //   cv::cvtColor(hdr, hdr, cv::COLOR_BGR2GRAY);
  // }
  // hdr.convertTo(dst_hdr, rgb ? CV_16UC3 : CV_16UC1);
  // // cv::Mat fusion;
  // // merge_mertens->process(image_list, fusion);
  // // fusion.convertTo(dst_fusion, rgb ? CV_8UC3 : CV_8UC1, 255);
}

HdrFusion::HdrFusion() {
  merge_debevec = cv::createMergeDebevec();
  merge_mertens = cv::createMergeMertens();
}
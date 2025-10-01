import { makeAutoObservable } from "mobx";
import { lucidSocket } from "../connect/socket/subscribe";
import { httpPost } from "../connect/http/request";

// Brightness analysis interfaces
interface BrightnessInfo {
  exposure_us: number;
  exposure_index: number;
  mean: number;
  std: number;
  median: number;
  timestamp: number;
  valid: boolean;
  reason: string;
}

interface BrightnessAnalysis {
  exposure_us: number;
  exposure_index: number;
  camera_results: { [camera_name: string]: BrightnessInfo };
  overall_valid: boolean;
  timestamp: number;
}

interface HDRStepProgress {
  exposure_us: number;
  exposure_index: number;
  attempt: number;
  max_attempts: number;
  exposure_retry?: number;
  max_exposure_retries?: number;
  cameras_verified: number;
  exposure_valid: boolean;
  error?: string;
  critical_failure?: boolean;
  timestamp: number;
}

interface HDRFinalVerification {
  timestamp: number;
  camera_results: {
    [camera_name: string]: {
      camera_name: string;
      is_valid: boolean;
      is_monotonic: boolean;
      sufficient_increases: boolean;
      not_overexposed: boolean;
      brightness_values: number[];
      increase_ratios: number[];
      max_brightness: number;
      details: Array<{
        exposure_us: number;
        mean: number;
        std: number;
        frame_index: number;
      }>;
    };
  };
  overall_success: boolean;
}

class LucidStore {
  lucidStatusJson: string = "{}";
  lucidStatus: { [key: string]: any } = {};

  storageEnabled = false;

  // HDR Burst Status
  hdrBurstMode = false;
  hdrBurstInProgress = false;
  hdrBurstProgress: { [key: string]: any } = {};
  hdrBurstCurrentStep = "";
  hdrHighGainMode = false;
  hdrHighGainBurstCount = 0;

  // High-gain processing status
  hdrHighGainProcessing = false;
  hdrHighGainProcessingProgress: { [key: string]: any } = {};

  // Brightness Monitoring
  brightnessAnalysisList: BrightnessAnalysis[] = [];
  hdrStepProgressList: HDRStepProgress[] = [];
  hdrFinalVerification: HDRFinalVerification | null = null;

  constructor() {
    makeAutoObservable(this);

    lucidSocket.subscribe("status", (data) => {
      this.lucidStatusJson = JSON.stringify(data);
      this.lucidStatus = data;
      this.storageEnabled = data.storage_enabled;

      // Update HDR burst status
      this.hdrBurstMode = data.hdr_burst_mode || false;
      this.hdrBurstInProgress = data.hdr_burst_in_progress || false;
      this.hdrBurstProgress = data.hdr_burst_progress || {};
      this.hdrBurstCurrentStep = data.hdr_burst_current_step || "";
      this.hdrHighGainMode = data.hdr_high_gain_mode || false;
      this.hdrHighGainBurstCount = data.hdr_high_gain_burst_count || 0;

      // Update high-gain processing status
      this.hdrHighGainProcessing = data.hdr_high_gain_processing || false;
      this.hdrHighGainProcessingProgress =
        data.hdr_high_gain_processing_progress || {};
    });

    // Subscribe to brightness analysis events
    lucidSocket.subscribe("brightness_analysis", (data: BrightnessAnalysis) => {
      console.log("Brightness analysis received:", data);
      this.brightnessAnalysisList.push(data);
      // Keep only the last 20 analyses to prevent memory bloat
      if (this.brightnessAnalysisList.length > 20) {
        this.brightnessAnalysisList.shift();
      }
    });

    // Subscribe to HDR step progress events
    lucidSocket.subscribe("hdr_step_progress", (data: HDRStepProgress) => {
      console.log("HDR step progress received:", data);
      this.hdrStepProgressList.push(data);
      // Keep only the last 50 steps to prevent memory bloat
      if (this.hdrStepProgressList.length > 50) {
        this.hdrStepProgressList.shift();
      }
    });

    // Subscribe to HDR final verification events
    lucidSocket.subscribe(
      "hdr_final_verification",
      (data: HDRFinalVerification) => {
        console.log("HDR final verification received:", data);
        this.hdrFinalVerification = data;
      }
    );
  }

  fetchEnableStorage = () => {
    httpPost("/lucid/storage/enable").fetch();
  };

  fetchDisableStorage = () => {
    httpPost("/lucid/storage/disable").fetch();
  };

  fetchUpdateStatusAttributes = (update_attr: Object) => {
    httpPost("/lucid/status/update", update_attr).fetch();
  };

  // HDR Burst Methods
  fetchEnableHDRBurst = () => {
    httpPost("/lucid/hdr/enable").fetch();
  };

  fetchDisableHDRBurst = () => {
    httpPost("/lucid/hdr/disable").fetch();
  };

  fetchCaptureHDRBurst = () => {
    httpPost("/lucid/hdr/capture").fetch();
    // Clear previous monitoring data when starting new capture
    this.brightnessAnalysisList = [];
    this.hdrStepProgressList = [];
    this.hdrFinalVerification = null;
  };

  // HDR High-Gain Methods
  fetchEnableHDRHighGain = () => {
    httpPost("/lucid/hdr/high_gain/enable").fetch();
  };

  fetchDisableHDRHighGain = () => {
    httpPost("/lucid/hdr/high_gain/disable").fetch();
  };

  // Brightness monitoring helper methods
  getLatestBrightnessAnalysis = (): BrightnessAnalysis | null => {
    return this.brightnessAnalysisList.length > 0
      ? this.brightnessAnalysisList[this.brightnessAnalysisList.length - 1]
      : null;
  };

  getLatestStepProgress = (): HDRStepProgress | null => {
    return this.hdrStepProgressList.length > 0
      ? this.hdrStepProgressList[this.hdrStepProgressList.length - 1]
      : null;
  };

  clearBrightnessData = () => {
    this.brightnessAnalysisList = [];
    this.hdrStepProgressList = [];
    this.hdrFinalVerification = null;
  };
}

export const lucidStore = new LucidStore();

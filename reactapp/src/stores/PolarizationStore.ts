import { action, makeAutoObservable } from "mobx";
import {
  CaptureAppCapture,
  CaptureAppScene,
  CaptureAppSpace,
} from "../public/proto/capture";
import { httpGet } from "../connect/http/request";

export const PROPERTIES = [
  "s0_combined",
  "dop",
  "top",
  "cop",
  "aolp",
  "dolp",

  "docp",
  "s1_combined",
  "s2_combined",
  "s3_combined",
  "diffuse",
  "specular",
];

export const CHANNEL = ["r", "g", "b", "nir"];

class PolarizationStore {
  constructor() {
    makeAutoObservable(this);
  }

  mmLinearAngle = 0;
  mmQwpAngle = 0;
  capture?: CaptureAppCapture = undefined;
  space?: CaptureAppSpace = undefined;
  scene?: CaptureAppScene = undefined;
  viewFocusProperty?: string = undefined;
  randomTime = 0;
  colorDeepView?: {
    x: number;
    y: number;
    colorList: { [key: string]: number };
  } = undefined;

  @action
  fetchComputePolarization(
    props: {
      spaceId?: number;
      captureId?: number;
      sceneId?: number;
      angle?: number;
      qwpAngle?: number;
      doOnSuccess?: () => void;
    } = {}
  ) {
    console.log(props);
    const spaceId = props.spaceId || this.space?.spaceId;
    const captureId = props.captureId || this.capture?.captureId;
    const sceneId =
      props.sceneId !== undefined ? props.sceneId : this.scene?.sceneId;
    const linearAngle = props.angle || this.mmLinearAngle;
    const qwpPlateAngle = props.qwpAngle || this.mmQwpAngle;

    httpGet(`
      /polarization/linear/${spaceId}/${captureId}/${sceneId}/${linearAngle}/${qwpPlateAngle}
    `)
      .onSuccess((data) => {
        props.doOnSuccess && props.doOnSuccess();
        this.randomTime = new Date().getTime();
      })
      .fetch();
  }

  @action
  fetchColorDeepView(y: number, x: number) {
    httpGet(`/polarization/view/${this.viewFocusProperty}/${y}/${x}`)
      .onSuccess((data) => {
        this.colorDeepView = {
          x,
          y,
          colorList: data,
        };
      })
      .fetch();
  }
}

export const polarizationStore = new PolarizationStore();

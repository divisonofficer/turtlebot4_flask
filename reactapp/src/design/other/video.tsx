import { HTMLProps, useEffect, useRef, useState } from "react";
import ReactPlayer from "react-player";
export const VideoStream = (
  props: HTMLProps<HTMLImageElement> & { url: string }
) => {
  const boundary = "--frame--begin--point";
  const url = props.url;
  return <img {...props} src={url} />;
};

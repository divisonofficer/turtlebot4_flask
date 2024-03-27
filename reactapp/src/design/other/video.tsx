import { HTMLProps, useEffect, useRef, useState } from "react";
import ReactPlayer from "react-player";
import { httpDel } from "../../connect/http/request";

export const VideoStream = (
  props: HTMLProps<HTMLImageElement> & { url: string }
) => {
  const [url, setUrl] = useState<string | undefined>(undefined);

  useEffect(() => {
    setUrl(props.url + "/" + Date.now());
    return () => {
      console.log("Unmounting", props.url);
      url && httpDel(url).fetch();
    };
  }, []);

  return <img {...props} src={url} />;
};

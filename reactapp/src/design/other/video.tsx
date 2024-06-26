import { HTMLProps, useEffect, useRef, useState } from "react";
import ReactPlayer from "react-player";
import { httpDel } from "../../connect/http/request";
import { NotAllowedIcon } from "@chakra-ui/icons";
import { PlayCircle } from "@phosphor-icons/react";

export const VideoStream = (
  props: HTMLProps<HTMLImageElement> & { url: string; play?: boolean }
) => {
  const [url, setUrl] = useState<string | undefined>(undefined);
  const [isPlaying, setIsPlaying] = useState(props.play || false);

  const [isHover, setIsHover] = useState(false);
  useEffect(() => {
    return () => {
      console.log("Unmounting", props.url);
      url && httpDel(url).fetch();
    };
  }, []);

  useEffect(() => {
    if (!isPlaying) {
      setUrl(undefined);
    } else {
      setUrl(props.url + "/" + Date.now());
    }
  }, [isPlaying]);

  return props.play ? (
    <img
      {...props}
      style={{
        width: "100%",
        height: "auto",
        objectFit: "cover",
        flexGrow: 1,
      }}
      src={url}
      alt="video"
    />
  ) : (
    <div
      {...props}
      onMouseEnter={() => setIsHover(true)}
      onMouseLeave={() => setIsHover(false)}
      onClick={() => setIsPlaying(!isPlaying)}
      style={{
        ...props.style,
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        ...(isHover && {
          background: "rgba(0,0,0,0.5)",
          justifyContent: "center",
          alignItems: "center",
        }),
      }}
    >
      {isPlaying && (
        <img
          style={{
            width: "100%",
            height: "auto",
            objectFit: "cover",
            flexGrow: 1,
          }}
          src={url}
          alt="video"
        />
      )}
      {!isPlaying && (
        <div>
          <NotAllowedIcon color="white" />
        </div>
      )}
      {isHover && !isPlaying && (
        <div>
          {" "}
          <PlayCircle />
        </div>
      )}
    </div>
  );
};

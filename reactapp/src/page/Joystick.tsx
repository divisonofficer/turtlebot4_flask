import { ArrowRightIcon, ChevronRightIcon } from "@chakra-ui/icons";
import { Container, Flex, HStack, VStack } from "@chakra-ui/react";
import { useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";
import { H2, H4 } from "../design/text/textsystem";
import { VDivBlock, VDivMenu } from "../design/other/blocks";
import { VStackAdj } from "../design/other/flexs";
import { InputButton } from "../design/button/inputButtons";
import { Btn } from "../design/button/button";

const stickProfile = [
  {
    arrow: 225,
    d: 1,
    angle: 1,
  },
  {
    arrow: 270,
    d: 1,
    angle: 0,
  },
  {
    arrow: 315,
    d: 1,
    angle: -1,
  },
  {
    arrow: 180,
    d: 0,
    angle: 1,
  },
  {
    arrow: 270,
    d: 0,
    angle: 0,
  },
  {
    arrow: 0,
    d: 0,
    angle: -1,
  },

  {
    arrow: 135,
    d: -1,
    angle: 1,
  },
  {
    arrow: 90,
    d: -1,
    angle: 0,
  },
  {
    arrow: 45,
    d: -1,
    angle: -1,
  },
];

const JoysticPage = () => {
  const [intervalJob, setIntervalJob] = useState<NodeJS.Timeout>();

  const handleButtonDown = (index: number) => {
    console.log("down", index);
    const job = setInterval(() => {
      rosSocket.publish("/drive", {
        linear: {
          x: stickProfile[index].d,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: stickProfile[index].angle,
        },
      });
    }, 10);
    setIntervalJob(job);
  };

  const handleButtonUp = (index: number) => {
    console.log("up", index);
    if (intervalJob) {
      clearInterval(intervalJob);
      setIntervalJob(undefined);
    }
  };

  useEffect(() => {
    return () => {
      if (intervalJob) {
        clearInterval(intervalJob);
        setIntervalJob(undefined);
      }
    };
  }, [intervalJob]);

  const JoyStick = () => {
    return (
      <VStack>
        {[0, 1, 2].map((index) => (
          <HStack key={index}>
            {[0, 1, 2].map((hindex) => (
              <InputButton
                key={index * 3 + hindex}
                style={{
                  width: "5rem",
                  height: "5rem",
                }}
                onMouseDown={() => handleButtonDown(index * 3 + hindex)}
                onMouseUp={() => handleButtonUp(index * 3 + hindex)}
              >
                <ChevronRightIcon
                  style={{
                    transform: `rotate(${
                      stickProfile[index * 3 + hindex].arrow
                    }deg)`,
                    width: "2rem",
                    height: "2rem",
                  }}
                />
              </InputButton>
            ))}
          </HStack>
        ))}
      </VStack>
    );
  };

  const OakdPreview = () => {
    return (
      <VStack>
        <H4>Camera</H4>
        <img
          style={{
            width: " 20rem",
            height: "20rem",
          }}
          src="/ros/video/oakd_preview"
          alt=""
        />
      </VStack>
    );
  };

  const LidarPreview = () => {
    return (
      <VStack>
        <H4>LidarMap</H4>
        <img
          style={{
            width: " 20rem",
            height: "20rem",
          }}
          src="/ros/video/lidar"
          alt=""
        />
      </VStack>
    );
  };

  return (
    <VStack
      style={{
        width: "100%",
        height: "100%",
        alignItems: "flex-start",
      }}
    >
      <VDivMenu color="#A1E3CB">Joystick</VDivMenu>
      <VStackAdj
        style={{
          alignItems: "center",
          gap: "2rem",
          padding: "2rem",
        }}
      >
        <JoyStick />
        <OakdPreview />
        <LidarPreview />
      </VStackAdj>
    </VStack>
  );
};
export default JoysticPage;

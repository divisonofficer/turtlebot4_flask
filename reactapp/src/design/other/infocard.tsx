import { HStack, Icon, VStack } from "@chakra-ui/react";
import { Body1, H3 } from "../text/textsystem";
import { ElementType, ReactElement, ReactNode, useState } from "react";

const InfoCard = (props: {
  title: string;
  icon?: string | ElementType;
  value?: string | number;
  color?: string;
  progressBar?: ReactNode;
}) => {
  const Icon = props.icon;
  return (
    <VStack
      style={{
        width: "15rem",
        height: "8rem",
        padding: "1.5rem 1.2rem",
        background: props.color || "#E3F5FF",
        justifyContent: "space-between",
        borderRadius: "1rem",
      }}
    >
      <HStack
        style={{
          width: "100%",
          justifyContent: "space-between",
        }}
      >
        <Body1>{props.title}</Body1>
        {typeof props.icon === "string" && (
          <img
            src={props.icon}
            style={{
              width: "1rem",
              height: "1rem",
            }}
            alt={props.title}
          />
        )}

        {Icon && typeof Icon !== "string" && (
          <Icon
            style={{
              width: "1rem",
              height: "1rem",
            }}
          />
        )}
      </HStack>
      <HStack
        style={{
          width: "100%",
          justifyContent: "space-between",
        }}
      >
        <H3>{props.value}</H3>
        {props.progressBar && props.progressBar}
      </HStack>
    </VStack>
  );
};

const InfoCardBtn = (props: {
  title: string;
  color?: string;
  Icon: string | ElementType;
  onClick?: () => void;
  fontSize?: string;
}) => {
  const [hover, setHover] = useState(false);

  return (
    <VStack
      style={{
        width: "8rem",
        height: "8rem",
        borderRadius: "1rem",
        justifyContent: "center",
        background: (props.color || "#1C1C1C") + (hover ? "80" : "FF"),
        transition: "0.3s",
      }}
      onMouseEnter={() => setHover(true)}
      onMouseLeave={() => setHover(false)}
      onClick={() => props.onClick && props.onClick()}
    >
      {typeof props.Icon === "string" ? (
        <img
          src={props.Icon}
          style={{
            width: "2rem",
            height: "2rem",
          }}
          alt={props.title}
        />
      ) : (
        <props.Icon
          style={{
            width: "2rem",
            height: "2rem",
            // tint white
            filter: "invert(100%)",
          }}
        />
      )}
      <H3
        style={{
          color: "white",
          width: "100%",
          textAlign: "center",
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
        }}
        fontSize={props.fontSize || "1.6rem"}
      >
        {props.title}
      </H3>
    </VStack>
  );
};

export { InfoCard, InfoCardBtn };

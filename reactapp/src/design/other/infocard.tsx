import { HStack, Icon, VStack } from "@chakra-ui/react";
import { Body1, H3 } from "../text/textsystem";
import { ElementType, ReactElement, ReactNode } from "react";
import { JsxElement } from "typescript";

const InfoCard = (props: {
  title: string;
  icon?: string | ElementType;
  value?: string | number;
  color?: string;
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
      </HStack>
    </VStack>
  );
};

export { InfoCard };

import { Flex, Text } from "@chakra-ui/react";
import { isDisabled } from "@testing-library/user-event/dist/utils";
import { ReactNode, useState } from "react";
import { JsxElement } from "typescript";

const ButtonStyle = {};

const VarientBorderless = (isDisabled: boolean, isHovered: boolean) => {
  return {
    background: isHovered ? "#1C1C1C0D" : "transparent",
    color: isDisabled ? "##1C1C1C1A" : "#000000",
    transition: "background 0.3s ease",
  };
};

const VarientGray = (isDisabled: boolean, isHovered: boolean) => {
  return {
    background: isHovered ? "#1C1C1C33" : "#1C1C1C0D",
    color: isDisabled ? "##1C1C1C1A" : "#000000",
    transition: "background 0.3s ease",
  };
};

const VarientOutline = (isDisabled: boolean, isHovered: boolean) => {
  return {
    background: isHovered ? "#1C1C1C33" : "transparent",
    color: isDisabled ? "##1C1C1C1A" : "#000000",
    border: "2px solid #1C1C1C1A",
    transition: "background 0.3s ease",
  };
};

const VarientFilled = (isDisabled: boolean, isHovered: boolean) => {
  return {
    background: isHovered ? "#1C1C1C99" : isDisabled ? "#1C1C1C0D" : "#1C1C1C",
    color: isDisabled ? "##1C1C1C1A" : "#FFFFFF",
    transition: "background 0.3s ease",
  };
};

const SizeSm = {
  fontSize: "sm",
  padding: "0.4rem 1rem",
  borderRadius: "0.4rem",
};

const SizeMd = {
  fontSize: "md",
  padding: "0.7rem 2rem",
  borderRadius: "0.8rem",
};

const SizeLg = {
  fontSize: "lg",
  padding: "1rem 3rem",
  borderRadius: "1.2rem",
};

const Btn = (props: {
  icon?: ReactNode | string;
  children: string | ReactNode;
  onClick?: () => void;
  varient?: "borderless" | "gray" | "outline" | "filled";
  size?: "sm" | "md" | "lg";
}) => {
  const [isHovered, setIsHovered] = useState(false);
  const isDisabled = false;

  return (
    <Flex
      style={{
        ...ButtonStyle,
        ...VarientFilled(isDisabled, isHovered),
        ...SizeMd,
        ...(props.varient === "borderless" &&
          VarientBorderless(isDisabled, isHovered)),
        ...(props.varient === "gray" && VarientGray(isDisabled, isHovered)),
        ...(props.varient === "outline" &&
          VarientOutline(isDisabled, isHovered)),
        ...(props.varient === "filled" && VarientFilled(isDisabled, isHovered)),
        ...(props.size === "sm" && SizeSm),
        ...(props.size === "md" && SizeMd),
        ...(props.size === "lg" && SizeLg),
      }}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      onClick={props.onClick}
    >
      {props.icon &&
        (typeof props.icon === "string" ? (
          <img
            src={props.icon}
            style={{ width: "1rem", height: "1rem" }}
            alt=""
          />
        ) : (
          props.icon
        ))}
      {props.children &&
        (typeof props.children === "string" ? (
          <Text fontSize={props.size}>{props.children}</Text>
        ) : (
          props.children
        ))}
    </Flex>
  );
};

export { Btn };

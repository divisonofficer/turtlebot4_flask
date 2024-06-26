import { Flex, Text } from "@chakra-ui/react";
import { isDisabled } from "@testing-library/user-event/dist/utils";
import { ReactNode, useState } from "react";
import { JsxElement } from "typescript";

const ButtonStyle = {
  cursor: "pointer",
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
  children?: string | ReactNode;
  color?: string;
  onClick?: () => void;
  varient?: "borderless" | "gray" | "outline" | "filled";
  size?: "sm" | "md" | "lg";
}) => {
  const [isHovered, setIsHovered] = useState(false);
  const isDisabled = false;
  const varient = props.varient || "filled";

  const color = props.color || "#1c1c1c";

  const VarientBorderless = (isDisabled: boolean, isHovered: boolean) => {
    return {
      background: isHovered ? `${color}0D` : "transparent",
      color: isDisabled ? `${color}1A` : "#000000",
      transition: "background 0.3s ease",
    };
  };

  const VarientGray = (isDisabled: boolean, isHovered: boolean) => {
    return {
      background: isHovered ? `${color}33` : `${color}0D`,
      color: isDisabled ? `${color}1A` : "#000000",
      transition: "background 0.3s ease",
    };
  };

  const VarientOutline = (isDisabled: boolean, isHovered: boolean) => {
    return {
      background: isHovered ? `${color}33` : "transparent",
      color: isDisabled ? `${color}1A` : "#000000",
      border: `2px solid ${color}1A`,
      transition: "background 0.3s ease",
    };
  };

  const VarientFilled = (isDisabled: boolean, isHovered: boolean) => {
    const style = {
      background: isHovered
        ? `${color}99`
        : isDisabled
        ? `${color}0D`
        : `${color}FF`,
      color: isDisabled ? `${color}1A` : `#FFFFFF`,
      transition: "background 0.3s ease",
    };
    return style;
  };
  return (
    <Flex
      style={{
        ...ButtonStyle,
        ...VarientFilled(isDisabled, isHovered),
        ...SizeMd,
        ...(varient === "borderless" &&
          VarientBorderless(isDisabled, isHovered)),
        ...(varient === "gray" && VarientGray(isDisabled, isHovered)),
        ...(varient === "outline" && VarientOutline(isDisabled, isHovered)),
        ...(varient === "filled" && VarientFilled(isDisabled, isHovered)),
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

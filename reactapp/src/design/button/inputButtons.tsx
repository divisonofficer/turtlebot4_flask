import { Flex, FlexProps } from "@chakra-ui/react";
import { useState } from "react";

const InputButtonStyle = {
  borderRadius: "1.5rem",
  border: "2px solid #1C1C1C1A",
  padding: "1rem",
  alignItems: "center",
  justifyContent: "center",
  background: "#ffffff",
};

const InputButtonStyleHover = {
  background: "#DCDCDC",
};

const InputButton = (props: FlexProps) => {
  const [hover, setHover] = useState(false);

  return (
    <Flex
      {...props}
      style={{
        ...InputButtonStyle,
        ...(hover && InputButtonStyleHover),
      }}
      onMouseEnter={() => setHover(true)}
      onMouseLeave={() => setHover(false)}
    >
      {props.children}
    </Flex>
  );
};

export { InputButton };

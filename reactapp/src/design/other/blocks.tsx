import { VStack, useMediaQuery } from "@chakra-ui/react";
import { H3 } from "../text/textsystem";

const VDivBlock = (props: { color?: string }) => {
  return (
    <div
      style={{
        width: "100%",
        height: "8px",
        borderRadius: "4px",
        background: props.color || "#B1E3FF",
      }}
    ></div>
  );
};

const VDivMenu = (props: { color?: string; children: String }) => {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];
  return (
    <VStack
      style={{
        alignItems: isMobile ? "center" : "flex-start",
        width: "100%",
      }}
    >
      <H3
        style={{
          marginLeft: isMobile ? "0" : "1rem",
          marginTop: "1rem",
        }}
      >
        {props.children}
      </H3>
      <VDivBlock color={props.color} />
    </VStack>
  );
};
export { VDivBlock, VDivMenu };

import { Flex, FlexProps, VStack, useMediaQuery } from "@chakra-ui/react";
import { H1 } from "../text/textsystem";

const VStackAdj = (props: FlexProps) => {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];

  return <Flex direction={isMobile ? "column" : "row"} {...props} />;
};
export { VStackAdj };

export const PageRoot = (
  props: FlexProps & {
    title: string;
  }
) => {
  return (
    <Flex
      direction="column"
      style={{
        ...props.style,
        width: "100%",
        height: "100vh",
        alignItems: "flex-start",
        justifyContent: "flex-start",
      }}
    >
      <H1>{props.title}</H1>

      {props.children}
    </Flex>
  );
};
